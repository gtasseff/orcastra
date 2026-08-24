// drums.c — synthesized drum kit + step sequencer. See drums.h.
//
// Voice recipes (all pure synthesis at FX_FS, classic analog drum-box
// character):
//   KICK   sine with exponential pitch sweep 160 -> 45 Hz, ~350 ms decay,
//          2 ms attack click.
//   SNARE  two damped tones (190/330 Hz) + high-passed noise, ~200 ms.
//   HAT-C  high-passed noise, ~55 ms. Closing chokes the open hat.
//   HAT-O  same voice, ~350 ms ring.
//   CLAP   three 10 ms noise bursts 12 ms apart into a band-passed tail.
// The sequencer advances on a sample counter (step = a 16th note =
// FX_FS * 15 / bpm samples), so timing is exact regardless of UI activity.
#include "drums.h"
#include "fx.h"                    // FX_FS
#include <string.h>
#include <math.h>

// UI-core-written state (audio core reads; single-word writes are atomic).
static volatile uint16_t d_pat[DRUM_BANKS][DRUM_VOICES]; // step bits per voice
static volatile int d_bank = 0;                // UI edit bank
static volatile int d_len  = 1;                // chain length (banks)
static volatile int d_playing = 0;
static volatile int d_bpm = 112;
static volatile int d_step = -1;               // playhead (audio core writes)
static volatile uint32_t d_hit = 0;            // manual trigger bitmask

// Audio-core sequencer state.
static int32_t d_count = 0;                    // samples until next step

// Voice states.
static float k_ph, k_f, k_env;                                 // kick
static float s_ph1, s_ph2, s_tenv, s_nenv, s_hp;               // snare
static float hc_env, ho_env, h_hp1, h_hp2;                     // hats
static float c_env, c_tail, c_bp1, c_bp2;                      // clap
static int   c_burst; static int32_t c_bt;
static uint32_t d_rng = 0xD5D5D5D5u;

static inline float dnoise(void) {             // white-ish, +/-1
    d_rng ^= d_rng << 13; d_rng ^= d_rng >> 17; d_rng ^= d_rng << 5;
    return (float)(int32_t)d_rng * (1.0f / 2147483648.0f);
}

void drums_toggle(int v, int s) {
    if (v < 0 || v >= DRUM_VOICES || s < 0 || s >= DRUM_STEPS) return;
    d_pat[d_bank][v] ^= (uint16_t)(1u << s);
}
int  drums_get(int v, int s) { return (d_pat[d_bank][v] >> s) & 1u; }
void drums_clear(void) {
    for (int v = 0; v < DRUM_VOICES; v++) d_pat[d_bank][v] = 0;
}
void drums_bank_set(int b) { if (b >= 0 && b < DRUM_BANKS) d_bank = b; }
int  drums_bank_get(void) { return d_bank; }
void drums_copy(int from, int to) {
    if (from < 0 || from >= DRUM_BANKS || to < 0 || to >= DRUM_BANKS) return;
    for (int v = 0; v < DRUM_VOICES; v++) d_pat[to][v] = d_pat[from][v];
}
void drums_len_set(int n) { if (n >= 1 && n <= DRUM_BANKS) d_len = n; }
int  drums_len_get(void) { return d_len; }
void drums_play(int on) {
    if (on && !d_playing) { d_step = -1; d_count = 0; }
    d_playing = on ? 1 : 0;
}
int  drums_playing(void) { return d_playing; }
void drums_bpm_set(int b) {
    if (b < 60) b = 60;
    if (b > 184) b = 184;
    d_bpm = b;
}
int  drums_bpm_get(void) { return d_bpm; }
int  drums_step_now(void) { return d_playing ? d_step : -1; }
void drums_trigger(int v) { if (v >= 0 && v < DRUM_VOICES) d_hit |= 1u << v; }

void drums_reset(void) {
    d_playing = 0; d_step = -1; d_count = 0; d_hit = 0;
    k_env = s_tenv = s_nenv = hc_env = ho_env = c_env = c_tail = 0.0f;
    k_ph = s_ph1 = s_ph2 = 0.0f; c_burst = 0;
}

int drums_active(void) {
    return d_playing || k_env > 0.001f || s_tenv > 0.001f || s_nenv > 0.001f
        || hc_env > 0.001f || ho_env > 0.001f || c_env > 0.001f
        || c_tail > 0.001f;
}

static void trig(int v) {
    switch (v) {
    case 0: k_env = 1.0f; k_f = 160.0f; k_ph = 0.0f; break;       // kick
    case 1: s_tenv = 1.0f; s_nenv = 1.0f; break;                  // snare
    case 2: hc_env = 1.0f; ho_env *= 0.15f; break;                // closed hat
    case 3: ho_env = 1.0f; break;                                 // open hat
    case 4: c_env = 1.0f; c_burst = 0; c_bt = 0; c_tail = 0.6f; break; // clap
    }
}

void drums_process(int16_t *inout, int n) {
    // Per-sample decay multipliers at FX_FS (tau: kick 120ms, snare tone
    // 60ms / noise 90ms, closed hat 20ms, open hat 120ms, clap tail 50ms).
    const float DK  = 0.99985f, DST = 0.99966f, DSN = 0.99977f;
    const float DHC = 0.999f,   DHO = 0.99983f, DCT = 0.99959f;
    for (int i = 0; i < n; i++) {
        // ---- sequencer ----
        if (d_playing) {
            if (--d_count <= 0) {
                d_count = (int32_t)((uint32_t)FX_FS * 15u / (uint32_t)d_bpm);
                int total = d_len * DRUM_STEPS;   // chain wraps at len banks
                d_step = (d_step + 1) % total;
                const volatile uint16_t *bank = d_pat[d_step >> 4];
                int col = d_step & (DRUM_STEPS - 1);
                for (int v = 0; v < DRUM_VOICES; v++)
                    if ((bank[v] >> col) & 1u) trig(v);
            }
        }
        if (d_hit) {                       // manual pads
            uint32_t h = d_hit; d_hit = 0;
            for (int v = 0; v < DRUM_VOICES; v++)
                if (h & (1u << v)) trig(v);
        }
        float mix = 0.0f;
        // ---- kick ----
        if (k_env > 0.001f) {
            // Sweep parks at 52 Hz (was 45): more of the ring sits where
            // small transducers still move air. Body stays a CLEAN sine
            // (the soft-clip experiment sounded fuzzy and thin — reverted
            // per bench 2026-07-29) plus a decaying octave harmonic at
            // ~104 Hz that carries the weight on bass-shy outputs.
            k_f += (52.0f - k_f) * 0.00015f;             // pitch swoop
            k_ph += k_f * (1.0f / (float)FX_FS);
            if (k_ph >= 1.0f) k_ph -= 1.0f;
            float body = sinf(6.2831853f * k_ph)
                       + 0.35f * k_env * sinf(2.0f * 6.2831853f * k_ph);
            float click = (k_env > 0.995f) ? 0.5f * dnoise() : 0.0f;
            mix += 1.00f * k_env * (body + click);   // 1.35 -> 1.15 -> 1.00
            k_env *= DK;
        }
        // ---- snare ----
        if (s_tenv > 0.001f || s_nenv > 0.001f) {
            s_ph1 += 190.0f * (1.0f / (float)FX_FS);
            s_ph2 += 330.0f * (1.0f / (float)FX_FS);
            if (s_ph1 >= 1.0f) s_ph1 -= 1.0f;
            if (s_ph2 >= 1.0f) s_ph2 -= 1.0f;
            float tone = 0.5f * sinf(6.2831853f * s_ph1)
                       + 0.35f * sinf(6.2831853f * s_ph2);
            float nz = dnoise();
            s_hp += 0.25f * (nz - s_hp);                 // ~2 kHz-ish HP
            float snap = nz - s_hp;
            // Mellow the snap: one-pole LP takes the sizzle off the top
            // (bench: "snare and clap SO HARSH" — twice), plus levels cut.
            // LEVELS RESTORED 2026-08-01, and the "harsh" verdicts are now
            // understood as a MONITORING artifact: both were given through the
            // 3.5 mm jack, whose coupling cap into a 32 ohm headphone load is a
            // HIGH-PASS (corner 8.8 kHz), so treble was massively
            // over-represented. Snare and clap really were harsh THERE. We cut
            // them, and on a flat amp path the kit then measured kick ~7.6 dB
            // over the hats and ~14 dB over the clap: "all I really hear is
            // overwhelming bass drum... we may have strangled the high hats and
            // the snare and clap". Never re-voice a mix on that jack.
            static float s_snlp;
            s_snlp += 0.45f * (snap - s_snlp);
            mix += 0.68f * s_tenv * tone + 0.62f * s_nenv * s_snlp * 1.6f;
            s_tenv *= DST; s_nenv *= DSN;
        }
        // ---- hats (shared metallic noise, two envelopes) ----
        if (hc_env > 0.001f || ho_env > 0.001f) {
            float nz = dnoise();
            h_hp1 += 0.55f * (nz - h_hp1);               // ~7 kHz HP x2
            float hp = nz - h_hp1;
            h_hp2 += 0.55f * (hp - h_hp2);
            float met = hp - h_hp2;
            mix += (1.70f * hc_env + 1.55f * ho_env) * met;
            hc_env *= DHC; ho_env *= DHO;
        }
        // ---- clap ----
        if (c_env > 0.001f || c_tail > 0.001f) {
            float nz = dnoise();
            // 1.4 kHz band-pass-ish colour
            c_bp1 += 0.17f * (nz - c_bp1);
            c_bp2 += 0.05f * (c_bp1 - c_bp2);
            float col = c_bp1 - c_bp2;
            float burst = 0.0f;
            if (c_burst < 3) {                           // 3 bursts, 12 ms apart
                int32_t T = (int32_t)(0.012f * (float)FX_FS);
                if (c_bt < (int32_t)(0.010f * (float)FX_FS))
                    burst = c_env;
                if (++c_bt >= T) { c_bt = 0; c_burst++; }
            } else {
                c_tail *= DCT;
            }
            mix += (1.9f * burst + 1.6f * (c_burst >= 3 ? c_tail : 0.0f)) * col;
            if (c_burst >= 3) c_env = 0.0f;
        }
        // ---- mix into the buffer ----
        float o = (float)inout[i] + 9500.0f * mix;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        inout[i] = (int16_t)o;
    }
}
