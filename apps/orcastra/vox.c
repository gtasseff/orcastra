// vox.c — VOX sampler engine: 5 PSRAM slots, hold-to-record, turntable playback.
//
// RECORD: dry input blocks are appended straight into the selected slot's
// PSRAM region while that slot's coloured button (or its on-screen twin) is
// held; release commits the length. Recording forces silence out (no monitor path — the PDM mics
// would feed back through the speaker).
//
// PLAYBACK: a "record on a turntable" model. Touch X maps to rate in
// [-1, +1]: +1 plays forward at 1x, -1 plays the same audio backward at 1x,
// 0 holds the record still. The play head advances pos += rate per output
// sample with linear interpolation, and the rate itself is slewed (~20 ms
// time constant) so grabbing/dragging feels like pushing a platter rather
// than clicking between speeds. Press = needle drops where the spin will
// pull from: center/right presses start at the START playing forward, left
// presses start at the END playing in reverse. Release = needle lifts.
// While held the sample LOOPS like a locked groove — running past either
// end wraps around — with a ~2 ms edge fade at the seam (and at the needle
// drop) so every wrap is click-free.
#include "vox.h"
#include "pico/stdlib.h"       // __uninitialized_psram linker placement
#include "platform/psram.h"
#include "platform/diag.h"
#include <string.h>

#define VOX_SLOT_BYTES 0xF0000u                 // 960 KiB / slot (5 = 4.7 MB)
#define VOX_MAX (VOX_SLOT_BYTES / 2u)           // 491,520 samples ≈ 10.1 s
#define VOX_FS 48828u   // ACTUAL system rate (see FX_FS in fx.h)

// Linker-placed PSRAM storage: agentio's shadow framebuffer also lives in
// PSRAM via linker sections, so casting PSRAM_BASE directly would collide.
static int16_t __uninitialized_psram("vox_slots")
    s_slots[VOX_SLOTS * (VOX_SLOT_BYTES / 2u)];
static int16_t *s_base = 0;                     // slot 0 base (0 = no PSRAM)
// Cross-core: the UI core drives select/record/hold/press from touch
// handlers while the audio core runs vox_process() every block.
static volatile uint32_t s_len[VOX_SLOTS];
static volatile int s_sel = 0;
static volatile int s_rec = -1;                 // slot being recorded, or -1
static volatile uint32_t s_rec_n;

static volatile int s_down = 0;                 // finger on the pad
static volatile int s_hold = 0;                 // hold latch (hands-free loop)
static float s_pos = 0.0f;
static float s_rate = 0.0f, s_rate_tgt = 0.0f;

bool vox_init(void) {
#if defined(PICO_RUNTIME_SKIP_INIT_PSRAM) && PICO_RUNTIME_SKIP_INIT_PSRAM
    // SD-LAUNCHED BUILDS: psram_init() reports 0 here, and the sampler was
    // silently disabled because of it ("vox: PSRAM absent/too small (0 bytes)").
    //
    // It is not a probe. The BSP header is explicit: "Returns the PSRAM size in
    // bytes... it only reports what runtime_init already did." Launched apps set
    // PICO_RUNTIME_SKIP_INIT_PSRAM=1 precisely because the loader stub already
    // configured the window and re-probing it is forbidden by the launch
    // contract -- so runtime_init did nothing and there is nothing to report.
    //
    // PSRAM is nonetheless demonstrably present: s_slots below is placed there
    // by the linker (__uninitialized_psram), and on the PSRAM-resident target we
    // are executing our own instructions out of it. Use the board's
    // compile-time size, which is what runtime_init would have reported.
    size_t sz = PICO_PSRAM_SIZE_BYTES;
#else
    size_t sz = psram_init();
#endif
    if (sz < (size_t)VOX_SLOT_BYTES * VOX_SLOTS) {
        DIAG("vox: PSRAM absent/too small (%u bytes) - sampler disabled\n",
             (unsigned)sz);
        return false;
    }
    s_base = s_slots;
    memset(s_len, 0, sizeof s_len);
    DIAG("vox: %u slots x %u KiB in PSRAM (%u s each @ %u Hz)\n",
         VOX_SLOTS, VOX_SLOT_BYTES / 1024u, VOX_MAX / VOX_FS, VOX_FS);
    return true;
}

bool vox_ok(void) { return s_base != 0; }

void vox_select(int slot) {
    if (slot >= 0 && slot < VOX_SLOTS) { s_sel = slot; s_pos = 0.0f; }
}
int vox_selected(void) { return s_sel; }

unsigned vox_len_ds(int slot) {
    if (slot < 0 || slot >= VOX_SLOTS) return 0;
    return (unsigned)((uint64_t)s_len[slot] * 10u / VOX_FS);
}

const int16_t *vox_data(int slot, unsigned *len) {
    if (!s_base || slot < 0 || slot >= VOX_SLOTS) { *len = 0; return 0; }
    *len = (s_rec == slot) ? s_rec_n : s_len[slot];
    return s_base + (uint32_t)slot * VOX_MAX;
}

// ---- Seeding a slot from code (boot-time demo preload) --------------------
// Slots live in PSRAM, which is VOLATILE -- nothing recorded survives a power
// cycle. So a "factory" sample cannot be stored in a slot; it has to be
// regenerated at every boot. These two calls let main.c synthesise one.
// Deliberately NOT a general write API: call vox_seed_begin, fill up to `cap`
// samples, then vox_seed_commit with how many you wrote.
int16_t *vox_seed_begin(int slot, unsigned *cap) {
    if (!s_base || slot < 0 || slot >= VOX_SLOTS) { if (cap) *cap = 0; return 0; }
    if (cap) *cap = VOX_MAX;
    return s_base + (uint32_t)slot * VOX_MAX;
}

void vox_seed_commit(int slot, unsigned len) {
    if (!s_base || slot < 0 || slot >= VOX_SLOTS) return;
    if (len > VOX_MAX) len = VOX_MAX;
    s_len[slot] = len;
}

unsigned vox_capacity(void) { return VOX_MAX; }

void vox_hold_set(int on) {
    s_hold = on ? 1 : 0;
    if (s_hold && !s_down) {
        // Continue from where the needle sits; a bare HOLD with no prior
        // scratch spins up forward at 1x from the start.
        if (s_rate_tgt > -0.04f && s_rate_tgt < 0.04f) {
            s_rate_tgt = 1.0f;
            s_pos = 0.0f;
        }
    }
}
bool vox_held(void) { return s_hold != 0; }

void vox_rec_start(int slot) {
    if (!s_base || slot < 0 || slot >= VOX_SLOTS) return;
    s_rec = slot; s_rec_n = 0;
    s_sel = slot;
    s_down = 0; s_hold = 0;                      // recording silences playback
}

void vox_rec_stop(void) {
    if (s_rec < 0) return;
    s_len[s_rec] = s_rec_n;
    s_rec = -1;
    s_pos = 0.0f;
}

bool vox_recording(void) { return s_rec >= 0; }
unsigned vox_rec_ds(void) { return (unsigned)((uint64_t)s_rec_n * 10u / VOX_FS); }

void vox_play_press(float r) {
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    s_rate_tgt = r;
    s_rate = 0.0f;                              // spins up via the slew
    // Needle drops where the spin will pull from: reverse-side presses
    // start at the end of the sample, otherwise at the beginning.
    if (r < 0.0f && s_base && s_len[s_sel] > 2u)
        s_pos = (float)s_len[s_sel] - 1.0f;
    else
        s_pos = 0.0f;
    s_down = 1;
}
void vox_play_release(void) { s_down = 0; }

void vox_play_rate(float r) {
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    s_rate_tgt = r;
}

bool vox_playing(void) { return (s_down || s_hold) && s_base && s_len[s_sel] > 0; }

float vox_pos_frac(void) {
    if (!s_base || s_len[s_sel] == 0) return 0.0f;
    float f = s_pos / (float)s_len[s_sel];
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return f;
}

void vox_process(const int16_t *in, int16_t *out, int n) {
    // 1) Record: append input, silence out. (Runs before `out` is written,
    //    so in == out is safe.)
    if (s_rec >= 0 && s_base) {
        int16_t *dst = s_base + (uint32_t)s_rec * VOX_MAX;
        for (int i = 0; i < n && s_rec_n < VOX_MAX; i++)
            dst[s_rec_n++] = in[i];
        memset(out, 0, (size_t)n * sizeof out[0]);
        return;
    }
    // 2) Playback (or silence when idle/empty).
    if (!vox_playing()) {
        memset(out, 0, (size_t)n * sizeof out[0]);
        return;
    }
    const int16_t *buf = s_base + (uint32_t)s_sel * VOX_MAX;
    const float L = (float)s_len[s_sel] - 1.0f;     // wrap length (interp-safe)
    for (int i = 0; i < n; i++) {
        s_rate += 0.00098f * (s_rate_tgt - s_rate);     // platter inertia ~21ms
        s_pos += s_rate;
        // Locked groove: wrap around in both directions while held.
        if (s_pos >= L)       s_pos -= L;
        else if (s_pos < 0.0f) s_pos += L;
        int   di = (int)s_pos;
        float fr = s_pos - (float)di;
        float v = (1.0f - fr) * (float)buf[di] + fr * (float)buf[di + 1];
        // ~2 ms edge fade at both ends: declicks the loop seam and the
        // needle drop without audibly denting the sample.
        float e1 = s_pos * (1.0f / 96.0f);
        float e2 = (L - s_pos) * (1.0f / 96.0f);
        float g = e1 < e2 ? e1 : e2;
        if (g > 1.0f) g = 1.0f;
        out[i] = (int16_t)(v * g);
    }
}
