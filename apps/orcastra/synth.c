// synth.c — touch instrument voice. See synth.h.
//
// Oscillator: phase accumulator with PolyBLEP edge smoothing for saw and
// square (naive edges alias badly at 48.8 kHz with bright waves; PolyBLEP
// is 2 adds + 2 mults only near the discontinuity). Sine is plain sinf on
// a per-sample phase (cheap enough at one voice).
// Filter: Chamberlin-style SVF lowpass, Q ~4 (juicy but stable), cutoff
// swept by the pad's Y axis over 150 Hz .. 7 kHz (log taper) — playing the
// pad feels like riding a wah over a buzzy string.
// Levels: osc at ~0.45 full scale into the SVF; resonance peaks are soft
// clipped; output clamped.
#include "synth.h"
#include "fx.h"            // FX_FS
#include <math.h>
#include <string.h>

// UI-core-written targets (audio core slews toward them).
static volatile float syn_f_tgt = 220.0f;    // oscillator Hz
static volatile float syn_c_tgt = 1200.0f;   // filter cutoff Hz
static volatile int   syn_gate_v = 0;
static volatile int   syn_wave_v = SYNTH_SAW;
static volatile float syn_shk = 0.0f;        // motion: shake energy 0..1
static volatile float syn_roll = 0.0f;       // motion: bank -1..1
static volatile float syn_tilt = 0.0f;       // motion: tip -1..1
static volatile float syn_dive = 0.0f;       // motion: twist rate -1..1
static volatile float syn_lvl_tgt = 1.0f;    // master level 0..1 (theremin)
static volatile float syn_gld = 0.002f;      // pitch slew/sample (see synth_glide)

// Audio-core state.
static float syn_ph = 0.0f;                  // osc phase 0..1
static float syn_f = 220.0f, syn_c = 1200.0f;
static float syn_env = 0.0f;                 // gate ramp 0..1
static float syn_lvl = 1.0f;                 // slewed master level
static float svf_lp = 0.0f, svf_bp = 0.0f;
static float syn_wob = 0.0f, syn_wob_t = 0.0f;  // rattle noise + its target
static float syn_trph = 0.0f;                // tremolo LFO phase
static float syn_bend = 0.0f;                // slewed dive bend (semitones)
static float syn_bar = 0.0f;                 // virtual bar position (integrated)
static uint32_t syn_wobn = 0;                // rattle re-roll countdown
static uint32_t syn_rng = 0x51E5EEDu;

void synth_motion(float shake, float roll, float tilt, float dive) {
    if (shake < 0.0f) shake = 0.0f;
    if (shake > 1.0f) shake = 1.0f;
    if (roll < -1.0f) roll = -1.0f;
    if (roll >  1.0f) roll =  1.0f;
    if (tilt < -1.0f) tilt = -1.0f;
    if (tilt >  1.0f) tilt =  1.0f;
    if (dive < -1.0f) dive = -1.0f;
    if (dive >  1.0f) dive =  1.0f;
    syn_shk = shake; syn_roll = roll; syn_tilt = tilt; syn_dive = dive;
}

void synth_gate(int on) { syn_gate_v = on ? 1 : 0; }

void synth_wave(int w) {
    if (w >= SYNTH_SAW && w <= SYNTH_SQUARE) syn_wave_v = w;
}
int synth_wave_get(void) { return syn_wave_v; }

void synth_level(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    syn_lvl_tgt = v;
}

void synth_freq(float hz) {
    if (hz < 55.0f)   hz = 55.0f;
    if (hz > 3520.0f) hz = 3520.0f;
    syn_f_tgt = hz;
}

void synth_glide(float k) {
    if (k < 0.0001f) k = 0.0001f;
    if (k > 0.01f)   k = 0.01f;
    syn_gld = k;
}

// X -> pitch: 2 octaves, 110..440 Hz (A2..A4), continuous (theremin glide).
// Y -> cutoff: 150 Hz .. 7 kHz log ("envelope intensity" feel: high = open
// and screaming, low = dark and hollow).
void synth_xy(float x, float y) {
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    if (y < 0.0f) y = 0.0f;
    if (y > 1.0f) y = 1.0f;
    // 2.5 octaves (110 Hz A2 .. 622 Hz D#5), widened from 2.0.
    //
    // WHY, and it is a reach argument rather than a musical one. The pad is
    // full width again (x 10..472) because it looks far better that way, but in
    // the production casing a thumb comfortably covers only about x 90..390.
    // Through touch_norm's 12 px end padding that is t = 0.155..0.840, i.e. 68%
    // of the sweep -- so at the old 2.0 octaves only 1.37 octaves were actually
    // playable without stretching for the bezel. At 2.5 the comfortable middle
    // carries 1.71 octaves, which is the "a bit more than a full octave from
    // left to right" the bench asked for, with the extra range spilling into
    // the corners you have to reach for rather than out of the useful span.
    //
    // Not widened further: resolution is the other side of this trade. 2.5
    // octaves over 438 usable px is ~14.6 px per semitone, still wide enough to
    // land a note deliberately; 4 octaves would put two semitones under one
    // fingertip and turn the pad into a glissando strip.
    syn_f_tgt = 110.0f * exp2f(2.5f * x);
    syn_c_tgt = 150.0f * exp2f(5.54f * y);       // *2^5.54 = ~46x -> ~7 kHz
}

int synth_active(void) { return syn_gate_v || syn_env > 0.002f; }

void synth_reset(void) {
    syn_ph = 0.0f; syn_env = 0.0f;
    syn_lvl = 1.0f; syn_lvl_tgt = 1.0f;
    syn_gld = 0.002f;
    svf_lp = svf_bp = 0.0f;
    syn_gate_v = 0;
    syn_wob = syn_wob_t = 0.0f; syn_wobn = 0;
    syn_trph = 0.0f; syn_bend = 0.0f; syn_bar = 0.0f;
    syn_shk = 0.0f; syn_roll = 0.0f; syn_tilt = 0.0f; syn_dive = 0.0f;
}

// PolyBLEP residual around a phase discontinuity at t=0/1.
static inline float pblep(float t, float dt) {
    if (t < dt)          { float x = t / dt;          return x + x - x * x - 1.0f; }
    if (t > 1.0f - dt)   { float x = (t - 1.0f) / dt; return x * x + x + x + 1.0f; }
    return 0.0f;
}

void synth_process(int16_t *out, int n) {
    int wave = syn_wave_v;
    for (int i = 0; i < n; i++) {
        // ~3 ms attack, ~30 ms release: click-free gate.
        syn_env += (syn_gate_v ? 0.007f : -0.0007f);
        if (syn_env > 1.0f) syn_env = 1.0f;
        if (syn_env < 0.0f) syn_env = 0.0f;
        // Pitch/cutoff glide (light slew: playable but not mushy).
        syn_f += syn_gld * (syn_f_tgt - syn_f);
        syn_c += 0.0015f * (syn_c_tgt - syn_c);
        syn_lvl += 0.002f * (syn_lvl_tgt - syn_lvl);   // ~10 ms level slew
        // Rattle: shake energy scatters pitch (re-roll a random target every
        // ~2 ms, slew toward it — nervous wobble, not white noise).
        if (syn_wobn-- == 0) {
            syn_wobn = 96;
            syn_rng ^= syn_rng << 13; syn_rng ^= syn_rng >> 17;
            syn_rng ^= syn_rng << 5;
            syn_wob_t = (float)(int32_t)syn_rng * (1.0f / 2147483648.0f);
        }
        syn_wob += 0.02f * (syn_wob_t - syn_wob);
        // Pitch dive: the snap RATE integrates into a virtual bar POSITION
        // (snap far edge down -> bar goes down and STAYS while displaced,
        // spring-returns over ~1.4 s). Range tuned against a reference
        // recording of real dive bombs (measured 30-39 semitone drops):
        // DOWN reaches -36 semitones (3 octaves); UP is bar-realistic +5.
        syn_bar += -8.0e-4f * syn_dive;               // integrate snap rate
        syn_bar *= 0.99995f;                          // spring return ~1.4 s
        if (syn_bar >  1.0f) syn_bar =  1.0f;
        if (syn_bar < -1.0f) syn_bar = -1.0f;
        float bt = syn_bar < 0.0f ? 36.0f * syn_bar : 5.0f * syn_bar;
        syn_bend += 0.002f * (bt - syn_bend);
        float f_eff = syn_f
                    * (1.0f + 0.045f * syn_shk * syn_wob)
                    * exp2f(syn_bend * (1.0f / 12.0f));
        float dt = f_eff * (1.0f / (float)FX_FS);
        syn_ph += dt;
        if (syn_ph >= 1.0f) syn_ph -= 1.0f;
        float osc;
        if (wave == SYNTH_SINE) {
            // A lone fundamental carries far less energy than saw/square
            // harmonics: boost so perceived loudness matches (+7 dB).
            osc = 2.2f * sinf(6.2831853f * syn_ph);
        } else if (wave == SYNTH_SQUARE) {
            osc = (syn_ph < 0.5f ? 1.0f : -1.0f)
                + pblep(syn_ph, dt)
                - pblep(syn_ph < 0.5f ? syn_ph + 0.5f : syn_ph - 0.5f, dt);
        } else {                                     // saw (default)
            osc = 2.0f * syn_ph - 1.0f - pblep(syn_ph, dt);
        }
        // SVF lowpass; damping k = 1/Q rides the tip-tilt: toward you
        // (tilt +1) -> k 0.10 (Q~10, screaming), away -> k 0.7 (dull thud).
        float k = 0.25f;
        if (syn_tilt > 0.0f) k -= 0.15f * syn_tilt;
        else                 k -= 0.45f * syn_tilt;
        float g = 6.2831853f * syn_c * (1.0f / (float)FX_FS);
        if (g > 1.2f) g = 1.2f;                      // stability guard
        svf_lp += g * svf_bp;
        float hp = osc - svf_lp - k * svf_bp;
        svf_bp += g * hp;
        float y = svf_lp;
        // Bank-tilt tremolo: rate 2..12 Hz and depth follow |roll|.
        float rollm = syn_roll < 0.0f ? -syn_roll : syn_roll;
        if (rollm > 0.05f) {
            syn_trph += (2.0f + 10.0f * rollm) * (1.0f / (float)FX_FS);
            if (syn_trph >= 1.0f) syn_trph -= 1.0f;
            float s = sinf(6.2831853f * syn_trph);
            y *= 1.0f - (0.75f * rollm) * (0.5f + 0.5f * s);
        }
        // Shake also trembles the amplitude a little (rattle body).
        y *= 1.0f - 0.20f * syn_shk * (syn_wob < 0 ? -syn_wob : syn_wob);
        // Soft clip the resonant peaks, then gate + level.
        if (y >  1.5f) y =  1.5f;
        if (y < -1.5f) y = -1.5f;
        y = y * (27.0f + y * y) / (27.0f + 9.0f * y * y);
        // 14000 clipped for real: 6170 pinned samples with 0.38 ms flat
        // tops in a 2 s take (bench 2026-08-03). 7000 = -6 dB, which puts
        // the synth page's peak near -6 dBFS with the pad held wide open.
        float o = y * syn_env * syn_lvl * 7000.0f;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}
