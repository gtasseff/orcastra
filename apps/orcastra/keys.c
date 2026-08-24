// keys.c — chord-pad voices. See keys.h.
//
// EPIANO: 2-op FM per voice (carrier + 4x modulator whose index decays
// fast) — the classic tine/Rhodes recipe: bell-ish strike melting into a
// warm sine body, ~3 s exponential ring.
// STRINGS: two PolyBLEP saws per voice detuned ±0.4% through a gentle
// one-pole LP, slow ~120 ms swell, long ring — ensemble pad.
// 4 voices = the widest chord (7ths). A new chord steals all voices.
// UI -> audio handshake mirrors the drums' d_hit pattern: the UI writes a
// pending-strike struct + flag; the audio core latches it between samples.
#include "keys.h"
#include "fx.h"                    // FX_FS
#include <math.h>
#include <string.h>

#define NV 4

// chord tables: semitone offsets from the root (-1 = unused voice)
static const int8_t CHORDS[KEYS_NCHORDS][NV] = {
    { 0, 4, 7, -1 },   // MAJ
    { 0, 3, 7, -1 },   // MIN
    { 0, 4, 7, 10 },   // DOM7
    { 0, 3, 7, 10 },   // MIN7
    { 0, 4, 7, 11 },   // MAJ7
    { 0, 5, 7, -1 },   // SUS4
    { 0, 2, 7, -1 },   // SUS2
    { 0, 3, 6, -1 },   // DIM
};

// UI-core-written strike request (audio core consumes).
static volatile float k_freq[NV];
static volatile int   k_nfreq = 0;
static volatile int   k_strike = 0;          // set by UI, cleared by audio
static volatile int   k_inst = KEYS_EPIANO;
static volatile int   k_sus = 1;             // sustain pedal (default down)
static volatile int   k_held = 0;            // finger still on the key
static volatile float k_vibd = 0.0f;         // finger-wiggle vibrato depth

// Audio-core voice state.
typedef struct {
    float f;                       // fundamental Hz
    float aenv;                    // amplitude envelope
    float atk;                     // attack ramp 0..1
    float cph, mph;                // FM carrier/modulator phases
    float ienv;                    // FM index envelope
    float s1, s2;                  // saw phases (strings)
} voice_t;
static voice_t vc[NV];
static float k_lp = 0.0f;          // strings shared LP state
static int   k_inst_live = KEYS_EPIANO;   // instrument latched at strike

void keys_instrument(int inst) {
    if (inst >= 0 && inst < KEYS_NINST) k_inst = inst;
}
int  keys_instrument_get(void) { return k_inst; }
void keys_sustain(int on) { k_sus = on ? 1 : 0; }
int  keys_sustain_get(void) { return k_sus; }
void keys_vibrato(float d) {
    if (d < 0.0f) d = 0.0f;
    if (d > 1.0f) d = 1.0f;
    k_vibd = d;
}

void keys_chord(int type, int root_semi) {
    if (type < 0 || type >= KEYS_NCHORDS) return;
    if (root_semi < 0 || root_semi > 12) return;
    // C3-based: chords sit in the piano's warm middle register.
    int n = 0;
    for (int v = 0; v < NV; v++) {
        if (CHORDS[type][v] < 0) break;
        float semi = (float)(root_semi + CHORDS[type][v]);
        k_freq[n++] = 130.813f * exp2f(semi * (1.0f / 12.0f));
    }
    k_nfreq = n;
    k_held = 1;
    k_strike = 1;                  // audio core latches on next block
}

void keys_release(void) { k_held = 0; }

void keys_reset(void) {
    k_strike = 0;
    for (int v = 0; v < NV; v++) vc[v].aenv = 0.0f;
    k_lp = 0.0f;
}

int keys_active(void) {
    if (k_strike) return 1;
    for (int v = 0; v < NV; v++)
        if (vc[v].aenv > 0.0015f) return 1;
    return 0;
}

// PolyBLEP residual (same shape the synth uses).
static inline float kblep(float t, float dt) {
    if (t < dt)        { float x = t / dt;          return x + x - x * x - 1.0f; }
    if (t > 1.0f - dt) { float x = (t - 1.0f) / dt; return x * x + x + x + 1.0f; }
    return 0.0f;
}

void keys_process(int16_t *inout, int n) {
    if (k_strike) {                // latch the pending chord
        k_strike = 0;
        k_inst_live = k_inst;
        int cnt = k_nfreq;
        for (int v = 0; v < NV; v++) {
            if (v < cnt) {
                vc[v].f = k_freq[v];
                vc[v].aenv = 1.0f;
                vc[v].atk = 0.0f;
                vc[v].ienv = 1.0f;
                vc[v].cph = vc[v].mph = 0.0f;
                vc[v].s1 = 0.0f; vc[v].s2 = 0.37f;   // decorrelate saws
            } else {
                vc[v].aenv = 0.0f;
            }
        }
    }
    // per-sample decay multipliers at FX_FS. Three envelope regimes:
    // HELD (finger down): piano-like slow sing (strings hold steady);
    // released w/ SUSTAIN: long ring; released w/o: damped but musical
    // (~0.75 s — the first cut's 0.2 s chop read as a bug at the bench).
    const float DEPI = 0.99988f;    // e-piano FM index (tine melts ~150 ms)
    int ep   = (k_inst_live == KEYS_EPIANO);
    int sus  = k_sus;
    int held = k_held;
    const float DEP = held ? 0.999996f
                    : sus  ? 0.999975f : 0.99982f;
    const float DST = held ? 1.0f
                    : sus  ? 0.999982f : 0.99985f;
    // Finger-wiggle vibrato: shared ~5.5 Hz pitch LFO, slewed depth so it
    // fades in/out musically instead of snapping.
    static float vib_ph = 0.0f, vib_d = 0.0f;
    for (int i = 0; i < n; i++) {
        vib_d += 0.00008f * (k_vibd - vib_d);
        vib_ph += 5.5f / (float)FX_FS;
        if (vib_ph >= 1.0f) vib_ph -= 1.0f;
        float vib = 1.0f + 0.008f * vib_d * sinf(6.2831853f * vib_ph);
        float mix = 0.0f;
        for (int v = 0; v < NV; v++) {
            if (vc[v].aenv <= 0.0015f) continue;
            float o;
            if (ep) {
                // 2-op FM: carrier f, modulator 4f, index 2.2 decaying fast
                vc[v].mph += vc[v].f * vib * (4.0f / (float)FX_FS);
                if (vc[v].mph >= 1.0f) vc[v].mph -= 1.0f;
                vc[v].cph += vc[v].f * vib * (1.0f / (float)FX_FS);
                if (vc[v].cph >= 1.0f) vc[v].cph -= 1.0f;
                float idx = 2.2f * vc[v].ienv + 0.12f;
                o = sinf(6.2831853f * vc[v].cph
                         + idx * sinf(6.2831853f * vc[v].mph));
                vc[v].ienv *= DEPI;
                // fast 2 ms attack, exponential ring
                vc[v].atk += 0.01f;
                if (vc[v].atk > 1.0f) vc[v].atk = 1.0f;
                vc[v].aenv *= DEP;
            } else {
                // strings: two detuned PolyBLEP saws
                float dt1 = vc[v].f * vib * 1.004f * (1.0f / (float)FX_FS);
                float dt2 = vc[v].f * vib * 0.996f * (1.0f / (float)FX_FS);
                vc[v].s1 += dt1; if (vc[v].s1 >= 1.0f) vc[v].s1 -= 1.0f;
                vc[v].s2 += dt2; if (vc[v].s2 >= 1.0f) vc[v].s2 -= 1.0f;
                o = (2.0f * vc[v].s1 - 1.0f - kblep(vc[v].s1, dt1))
                  + (2.0f * vc[v].s2 - 1.0f - kblep(vc[v].s2, dt2));
                o *= 0.45f;
                // slow ~120 ms swell, long ring
                vc[v].atk += 0.00017f;
                if (vc[v].atk > 1.0f) vc[v].atk = 1.0f;
                vc[v].aenv *= DST;
            }
            mix += o * vc[v].aenv * vc[v].atk;
        }
        if (!ep) {                 // strings: shared gentle LP (~2.4 kHz)
            k_lp += 0.27f * (mix - k_lp);
            mix = k_lp;
        }
        float s = (float)inout[i] + 7000.0f * mix;
        if (s >  32700.0f) s =  32700.0f;
        if (s < -32700.0f) s = -32700.0f;
        inout[i] = (int16_t)s;
    }
}
