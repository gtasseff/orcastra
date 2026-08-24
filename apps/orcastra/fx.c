// fx.c — orcastra effect chain (mono at FX_FS = 48,828 Hz, block-based).
//
// PITCH: classic dual-tap delay-line shifter. A phase ramp sweeps a read tap
// through a WIN-sample window at (1 - rate) samples/sample; a second tap runs
// half a window behind. Each tap is faded out (triangular gains, gA+gB == 1)
// exactly when it wraps, so the splice is inaudible. rate = 2^octaves, so the
// sweep transposes without changing duration (a granular "rotating tape head").
// At rate == 1 the taps freeze into a constant small delay = bit-transparent.
//
// ECHO: single-tap feedback delay (300 ms). fx_amt scales wet mix (0..0.7)
// and feedback (0..0.6) together: far right on the pad ~= long trailing echo.
//
// Cost: ~25 flops/sample * 16 kHz = well under 1% of a 250 MHz M33F core.
#include "fx.h"
#include <math.h>
#include <string.h>

#define WIN     3072u                 // pitch crossfade window: ~63 ms
#define PBUF_N  4096u                 // pitch history (power of two)
#define PMASK   (PBUF_N - 1u)

// A pitch-shifter instance (each page gets its own, so states don't collide).
// `win` is the crossfade grain window in samples: big (1024 = 64 ms) sounds
// smooth/chorusy, small (384 = 24 ms) has the tight mechanical splice that
// hard-tune wants. Set per instance in fx_reset (memset clears it!).
typedef struct {
    int16_t  buf[PBUF_N];
    uint32_t w;                       // write index (free-running)
    float    ph;                      // tap phase in [0,1)
    float    win;                     // grain window, samples
} shifter_t;

static shifter_t sh_pad;              // FX pad page (variable rate)
static shifter_t sh_oct;              // octave page (fixed rate 2.0)

static uint32_t dw;                   // echo write index

static float cur_rate = 1.0f, tgt_rate = 1.0f;
static float cur_wet  = 0.0f, tgt_wet  = 0.0f;
// Octave page: one shifter, five voicings. `oct_mix` is the live crossfade
// (0 when off), `oct_wet` the MIX knob it fades TO, so toggling off and back on
// returns to the knob rather than a hardcoded 50%.
static float oct_mix  = 0.0f, oct_tgt  = 0.0f;
static float oct_wet  = 0.5f;         // MIX knob, 0..1 (50% default)
static float oct_rate = 2.0f, oct_rate_tgt = 2.0f;
static int   oct_on_v = 0;            // toggle state (MIX must know it)
static int   oct_mode_v = 0;          // 0 +12, 1 +24, 2 -12, 3 -24, 4 detune
static int   oct_det_pct = 50;        // detune slider: 50 = in tune
static void  oct_rate_recalc(void);   // defined with the octave page below

// Glitch state (see the algorithm comment above fx_glitch_process).
#define GBUF_N 65536u                       // 1.34 s rolling record @ 48.8 kHz
#define GMASK  (GBUF_N - 1u)
static int16_t  gbuf[GBUF_N];
static uint32_t ggw;                        // record head (free-running)
static int      g_on = 0;                   // armed?
static int      g_looping = 0;              // currently glitching?
static uint32_t g_wait;                     // samples until next trigger
static uint32_t g_start, g_len;             // frozen fragment [start, start+len)
static uint32_t g_t_left;                   // episode output samples remaining
static uint32_t g_seg_left;                 // samples until loop-length mutation
static float    g_pos;                      // read cursor in the fragment
static float    g_rate;                     // playback rate, constant per episode
static float    g_env;                      // dry<->wet crossfade (0=dry)

// Rearm gap range (samples) between bursts — set by fx_glitch_freq().
// Defaults to FREQ 6 (~0.45-3.8 s between takeovers).
static uint32_t g_gap_lo = FX_MS(455), g_gap_hi = FX_MS(3790);

static uint32_t g_rng = 0xC0FFEE21u;
static inline uint32_t rng(void) {          // xorshift32
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return g_rng;
}
static inline uint32_t rng_range(uint32_t lo, uint32_t hi) {
    return lo + rng() % (hi - lo + 1u);
}

// Slap delay state: 16384-sample line = up to 335 ms @ 48.8 kHz.
#define SBUF_N 16384u
#define SMASK  (SBUF_N - 1u)
static int16_t  sbuf[SBUF_N];
static uint32_t sw_;                        // slap write head
static float slap_mix = 0.0f, slap_tgt = 0.0f;      // echo level (0 / 0.65)
static float slap_d   = 5859.0f, slap_d_tgt = 5859.0f;  // tap in samples (120 ms)

// Metal (scooped high-gain) state.
static float met_mix = 0.0f, met_tgt = 0.0f;   // dry<->wet crossfade
static float met_drive = 30.0f;                // clipper input gain
static float met_hp_y = 0.0f, met_hp_x = 0.0f; // pre-clip one-pole HP @120 Hz
static float met_bq_z1 = 0.0f, met_bq_z2 = 0.0f; // mid-scoop biquad state
static float met_lp1 = 0.0f, met_lp2 = 0.0f;   // 2x one-pole LP (cab-ish)

// Overdrive (asymmetric-clip pedal style) state.
static float od_mix = 0.0f, od_tgt = 0.0f;     // dry<->wet crossfade
static float od_gain = 10.0f;                  // clipper input gain (drive)
static float od_hp_y = 0.0f, od_hp_x = 0.0f;   // input one-pole HP @~105 Hz
static float od_blp = 0.0f;                    // mid-hump knee LP @~720 Hz
static float od_dc_y = 0.0f, od_dc_x = 0.0f;   // post-clip DC blocker
static float od_lp1 = 0.0f;                    // tone one-pole LP @~3.3 kHz

#if FX_ENABLE_TREMOLO   /* disabled 2026-08-01, see fx.h */
// Tremolo (optical-style amplitude LFO) state.
static float trm_mix = 0.0f, trm_tgt = 0.0f;   // dry<->wet crossfade
static float trm_ph = 0.0f;                    // LFO phase 0..1
static float trm_inc = 5.0f / (float)FX_FS;    // LFO increment (rate 5 Hz)
static float trm_g = 1.0f;                     // current LFO gain

#endif

#if FX_ENABLE_PHASER   /* disabled 2026-08-01, see fx.h */
// Phaser (4-stage swept allpass) state.
static float ph_mix = 0.0f, ph_tgt = 0.0f;     // dry<->wet crossfade
static float ph_x1[4], ph_y1[4];               // allpass stage states
static float ph_wet = 0.0f;                    // last chain output (feedback)
static float ph_lfo = 0.0f;                    // LFO phase 0..1
static float ph_inc = 0.6f / (float)FX_FS;     // LFO increment (rate 0.6 Hz)
static float ph_a = -0.9f;                     // shared allpass coefficient

#endif

// Parametric EQ state (see the EQ section below for the algorithm).
#define EQ_BANDS 5
static float eq_stage[EQ_BANDS][5];            // staged b0,b1,b2,a1,a2 (UI)
static volatile uint32_t eq_gen = 0;           // bumped after staging
static float eq_c[EQ_BANDS][5];                // active coeffs (audio core)
static float eq_z[EQ_BANDS][2];                // TDF-II state
static float eq_mix = 0.0f, eq_tgt = 0.0f;

// Pitch-correction state.
static shifter_t sh_tune;                      // ROBOTIC engine (granular)
static float tun_mix = 0.0f, tun_tgt = 0.0f;   // dry<->wet crossfade
static int16_t tun_win[512];                   // sliding analysis window
static int   tun_root = 0;                     // key root (0 = C .. 11 = B)
static uint16_t tun_mask = 0x0AB5;             // scale mask (major default)
static int   tun_robot = 1;                    // 1 = robotic (instant retune)
static float tun_corr = 0.0f;                  // current correction (semis)
static float tun_corr_tgt = 0.0f;              // target correction
static int   tun_note = -1;                    // held target note (hysteresis)
static int   tun_uv = 0;                       // consecutive unvoiced blocks
static float tun_freq_dbg = 0.0f;              // last detected pitch (telemetry)
static float tun_m3[3];                        // last 3 raw pitches (midi)
static int   tun_m3n = 0;                      // how many are valid
static float tun_midi_s = 0.0f;                // smoothed measured pitch

// PSOLA synthesis state (see the pitch-correction section for the algorithm).
#define PSA_N 4096u
#define PSA_MASK (PSA_N - 1u)
#define PSA_GRAINS 4
typedef struct {
    uint32_t center;                           // pitch mark the grain is cut on
    float    ph, len, amp;                     // window phase/length/level
    int      on;
} grain_t;
static int16_t  psa_ring[PSA_N];               // input history for grains
static uint32_t psa_w;                         // ring write head
static float    psa_P = 384.0f;                // tracked pitch period (samples)
static float    psa_nse = 0.0f;                // samples to next synth epoch
static float    psa_ac = 384.0f;               // samples to next analysis epoch
static uint32_t psa_A[2];                      // last two pitch marks
static grain_t  psa_g[PSA_GRAINS];
static float    psa_tab[257];                  // Hann window table
static int      psa_tab_ok = 0;

// Paranoid (envelope-swept filter lead) state.
static float par_mix = 0.0f, par_tgt = 0.0f;   // dry<->wet crossfade
static float par_genv = 0.0f, par_gate = 0.0f; // gate follower + gate gain
static float par_fenv = 0.0f;                  // filter-sweep follower
static float par_hp_y = 0.0f, par_hp_x = 0.0f; // pre-drive one-pole HP @150 Hz
static float par_dc_y = 0.0f, par_dc_x = 0.0f; // post-clip DC blocker
static float par_bq_z1 = 0.0f, par_bq_z2 = 0.0f; // mid-contour biquad state
static float par_lp1 = 0.0f;                   // treble one-pole LP @4.2 kHz
static float par_ic1 = 0.0f, par_ic2 = 0.0f;   // TPT SVF integrator states
static float par_g = 0.049f;                   // SVF g = tan(pi*fc/fs), slewed
static float par_fmax = 2200.0f;               // sweep ceiling (RANGE knob)

// ---- Internal saturation counters -----------------------------------------
// "Zero clamped samples at the output" does NOT mean nothing saturated. Every
// stage clamps independently, and the ones with FEEDBACK can drive their own
// delay line onto its limit while the final mix still measures clean -- the
// output counter sits downstream and cannot see it. That is exactly what the
// delay did on 2026-07-30: at 0.85 feedback the line settled at 6.7x the input
// and saturated internally while the sweep reported "clean" at the output.
//
// These count the feedback lines only. Feed-forward stages clamp at their own
// output, which the pump's output counter already sees.
enum { FXC_DELAY = 0, FXC_VERB_COMB, FXC_VERB_AP, FXC_VERB_SHIM, FXC_FLANGE };
static volatile uint32_t fxc_mask;    // bit per stage that hit its limit
static volatile uint32_t fxc_count;   // total clamped samples

static inline float fxc_clamp(float v, float lim, unsigned bit) {
    if (v >  lim) { fxc_mask |= 1u << bit; fxc_count++; return  lim; }
    if (v < -lim) { fxc_mask |= 1u << bit; fxc_count++; return -lim; }
    return v;
}

// Snapshot and clear. Caller reports; see the `fxclip:` RTT line.
void fx_clip_report(uint32_t *mask, uint32_t *count) {
    if (mask)  *mask  = fxc_mask;
    if (count) *count = fxc_count;
    fxc_mask = 0; fxc_count = 0;
}

// Linear-interpolated read `delay` samples behind the write head.
static inline float sh_read(const shifter_t *s, float delay) {
    int   di = (int)delay;
    float fr = delay - (float)di;
    int16_t a = s->buf[(s->w - (uint32_t)di)      & PMASK];
    int16_t b = s->buf[(s->w - (uint32_t)di - 1u) & PMASK];
    return (1.0f - fr) * (float)a + fr * (float)b;
}

// Push one input sample, return the pitch-shifted sample at `rate`.
static inline float sh_tick(shifter_t *s, float rate, int16_t in) {
    s->w++;
    s->buf[s->w & PMASK] = in;
    s->ph += (1.0f - rate) / s->win;
    if (s->ph >= 1.0f) s->ph -= 1.0f;
    if (s->ph <  0.0f) s->ph += 1.0f;
    float pB = s->ph + 0.5f;
    if (pB >= 1.0f) pB -= 1.0f;
    float gA = 1.0f - fabsf(2.0f * s->ph - 1.0f);   // 0 at wrap, 1 mid
    return gA          * sh_read(s, 2.0f + s->ph * s->win)
         + (1.0f - gA) * sh_read(s, 2.0f + pB    * s->win);
}

// ---- Reverb tank storage (state here, DSP at the end of the file) ---------
// Comb lengths mutually prime at FX_FS so the echo trains never line up into
// a pitched ring. int16 lines: SRAM has ~20 KB free on this build and float
// lines would cost 21 KB. The tank runs near full scale (input is scaled by
// RV_IN to offset the combs' 1/(1-fb) DC gain), so 16 bits is not the
// resolution compromise it looks like.
// EIGHT combs, not four. Four produced only four discrete echo trains, which
// the ear resolves as separate repeats -- bench verdict 2026-07-30 was
// "metallic slap echo", and that is a DENSITY problem, not a decay-length one.
// Lengthening a sparse tank just makes the slaps last longer. All lengths are
// prime so no two trains ever line up into a resonant ring.
#define RV_NC 8
#define RV_NA 4
#define RV_TOTAL (1237u + 1319u + 1409u + 1499u + 1571u + 1657u + 1723u + \
                  1789u + 617u + 487u + 379u + 251u)
static const uint16_t RV_CLEN[RV_NC] = {
    1237u, 1319u, 1409u, 1499u, 1571u, 1657u, 1723u, 1789u
};
static const uint16_t RV_ALEN[RV_NA] = { 617u, 487u, 379u, 251u };
// Tank input scale, sized so a SUSTAINED full-scale input settles the comb
// states at RV_IN/(1-fb) ~= 0.56 rather than on their clamp, at the highest
// decay setting. The output normalisation is derived from fb rather than fixed,
// so raising DECAY does not also raise the wet level and make MIX meaningless
// (calibrated constant 1.86 puts wet within 0.3 dB of dry -- at a naive 1/RV_NC
// the wet path measured 5.9 dB BELOW dry and the tail vanished under the bench
// rig's noise floor).
// Tank input scale. NOW DERIVED FROM rv_fb rather than fixed (2026-08-01): set
// to 0.7*(1-fb) so the combs always settle at the same 0.7 utilisation whatever
// the DECAY setting. Two consequences: the tank can never saturate at long
// decays, AND rv_norm falls out as a CONSTANT (RV_CAL/(RV_NC*0.7)) instead of
// being recomputed, because the norm*rv_in product is what sets the wet level.
// Measured: tank peak actually DROPS from 0.11 to 0.06 as decay is extended.
static float      rv_in = 0.0224f;   // = 0.7*(1-0.968), the DECAY 9 default
#define RV_AP_G   0.5f     // allpass coefficient (classic Schroeder value)
#define RV_CAL    1.86f    // wet-vs-dry calibration (see fx_verb_decay)

// SHIMMER: an octave-up shifter INSIDE the reverb's feedback loop, so each
// trip round the loop climbs another octave -- the ascending, blooming
// "cathedral" character. Two things make it work rather than explode:
//   1. the shimmer feedback is scaled DOWN before it reaches the tank. Injecting
//      it un-attenuated multiplies loop gain by 1/rv_in (36x) -- simulation
//      showed exactly that runaway, output reaching 6.0 with the tail growing.
//   2. a one-pole lowpass after the shifter. An octave shifter pumps energy
//      upward every pass; without it the top end runs away on its own.
//   3. THE LOOP GAIN MUST STAY BELOW 1, and the arithmetic is exact:
//      the tank's gain from its input to its output goes as 1/(1-fb) while rv_in
//      goes as (1-fb), so those cancel and
//              shimmer loop gain = RV_CAL * rv_sh_g = 1.86 * rv_sh_g
//      independent of DECAY. Stability therefore needs rv_sh_g < 1/1.86 = 0.538.
//      The knob used to reach 0.60 (loop gain 1.116) -- ABOVE UNITY, so at
//      SHIMMER 10 the shimmer self-oscillated at every DECAY setting. It was
//      audible wherever the tank's own decay did not mask it, which is why it
//      chased the DECAY knob around as things changed: bench heard "a bell ring
//      that starts soft and whooshes up louder and then just suddenly cuts",
//      then "a swell at high shimmer and high decay... it doesn't sound correct
//      for a decay to end with a swell UP", then "a high frequency ring mod type
//      tone" -- that last one IS the self-oscillation, parked where the loop's
//      lowpass stops it climbing.
//      Two earlier claims in this comment were WRONG and are recorded so nobody
//      trusts them again: (a) "simulated stable to 0.6" came from a 6-second
//      window, far too short to see slow growth at a 7.5 s RT60; (b) an attempt
//      to fix it by giving the feedback its own FIXED input scale only MOVED the
//      instability from low decay to high decay, because that reintroduced the
//      1/(1-fb) term the proportional rv_in was cancelling.
//      Verified over a 16 s window at DECAY 10: gsh 0.60 flattens at -55 dB and
//      RISES to -52; 0.45 decays -30/-58/-70/-73 dB monotonically.
// Hence the step below is 0.045, capping rv_sh_g at 0.45 (loop gain 0.837).
#define RV_SH_WIN  1920.0f   // ~39 ms grain, the 30-50 ms the blueprint asks for
// One-pole LP inside the shimmer loop. LOWERED 0.45 -> 0.275 on 2026-08-01,
// i.e. the corner moves 4646 Hz -> 2499 Hz. Each trip round the loop shifts UP
// an octave, so a component climbs 1k -> 2k -> 4k -> 8k; at a 4.6 kHz corner the
// early passes were barely touched and energy PILED UP at the top. Bench verdict
// above SHIMMER 6: "a single-tone high frequency whine that just decayed with the
// rest of the sound... that frequency audibly stood out and also had a hissy aura
// around it" -- the standing tone is the pile-up, the hiss is the granular
// splice. A 2.5 kHz corner kills each pass harder so the ladder cannot climb.
// NOTE this only became audible once the bench moved to a clean full-bandwidth
// amp path; the old 3.5 mm monitoring was hiding it (same story as the flanger).
#define RV_SH_LP   0.275f

static int16_t  rv_buf[RV_TOTAL];
static uint16_t rv_coff[RV_NC], rv_aoff[RV_NA];
static uint16_t rv_cw[RV_NC], rv_aw[RV_NA];
static float    rv_lp[RV_NC];
static float    rv_mix = 0.0f, rv_tgt = 0.0f;   // slewed on/off
static float    rv_wet_set = 0.56f;             // MIX knob, default 7 -> 0.56
static float    rv_fb = 0.95f;     // DECAY knob; ~3.4 s RT60 here
static float    rv_norm = 0.332f;  // derived from rv_fb, see fx_verb_decay
static float    rv_damp = 0.15f;   // low = bright tail (bench: "brighten it up")
static float    rv_sh_g = 0.35f;   // SHIMMER amount, 0..0.6
static float    rv_sh_lp = 0.0f;   // loop lowpass state
static float    rv_sh_fb = 0.0f;   // feedback sample carried to the next tick
static shifter_t rv_sh;            // the octave-up shifter in the loop
static int      rv_ready = 0;

// ---- CHORUS WAS CUT 2026-08-01 --------------------------------------------
// Bench verdict, twice, on two different days: "horrible", then "chorus does
// sound bad... go ahead and get rid of it". Measured causes, kept here because
// they are the spec for any future attempt:
//   - depth was +/-2.5 ms on an 8 ms base = +/-31% delay modulation. Measured
//     +/-3.7 cents of wobble on a steady tone where a chorus wants +/-5..15,
//     and a proposed +/-1.0 ms brought it to +/-2.4.
//   - the 8 ms base put comb notches every 125 Hz starting at 62.5 Hz, right
//     through the guitar's fundamental range. That was the hollowness.
//   - there was NO bandlimiting anywhere, no tanh headroom, and no HF loss in
//     the feedback path, so it combed the full 24 kHz.
// If it comes back, the user supplied a multi-mode analog BBD chorus
// (bandlimit 8-12 kHz, tanh headroom, in-loop HF loss, CHORUS / DOUBLE TRACK /
// FILTER MATRIX modes, >=100 ms line) and one ruling: STAY MONO -- "we do not
// need to change architecture for one chorus effect". Do NOT lengthen the base
// delay to fix it; 5-15 ms is correct per the model. Fix depth and bandwidth.
// TREMOLO took its grid slot; its DSP was still intact and costs no buffer.
static float    dly_fb = 0.0f;   // DELAY feedback 0..0.85
static float    dly_lvl = 0.65f; // DELAY echo level 0..1

// ---- Bit crush: decimate + quantise, deliberately un-filtered -------------
// No anti-alias filter anywhere: the fold-back IS the effect. One knob drives
// both axes so a single slider goes from "slightly digital" to "broken toy".
static float    bc_mix, bc_tgt;
static float    bc_hold;         // sample-and-hold value
static int      bc_cnt;          // samples until the next sample is taken
static int      bc_step = 1;     // hold length, 1..24
static float    bc_q = 1.0f / 32768.0f;   // quantiser step, normalised

#if FX_ENABLE_RINGMOD   /* disabled 2026-08-01, see fx.h */
// ---- Ring mod: multiply by a tunable carrier ------------------------------
// Fully wet when engaged, like the classic box -- blending dry back in just
// sounds like a slightly odd tremolo. Quadrature oscillator, no sinf().
static float    rm_s, rm_c = 1.0f, rm_k;
static float    rm_wet_set = 0.7f;   // MIX knob: wet share, 0..1
static float    rm_mix, rm_tgt;

#endif

// ---- Flanger: short modulated delay WITH feedback -------------------------
// Modelled on a classic analog BBD flanger. REBUILT 2026-07-31: the verdict
// on v1 was "horribly robotic", and v1 earned it. It ran a bare
// `wr = x + 0.70*echo` -- full-bandwidth resonance recirculating with NO
// filtering anywhere in the loop -- and read the line with linear
// interpolation while sweeping 0.45..3.55 ms. Linear interpolation is a
// lowpass whose depth varies with the fractional part of the delay, so on a
// fast short sweep it both dulls the top end AND modulates that dulling. The
// ringing plus the wobbling dullness is what "robotic" was.
//
// What the model actually needs (all of it absent in v1):
//   1. 4-point Catmull-Rom interpolation, so short delays keep their chime.
//   2. An ASYMMETRIC triangle LFO -- 55% rise, 45% fall. The uneven sweep is
//      where the model's vocal quality comes from; a sine has no edge.
//   3. Feedback band-limited to the midrange: HP 150 Hz stops low-end boom,
//      LP 6 kHz stops the piercing top-end ring. THIS is the robotic fix.
//   4. A 6th-order Butterworth at 9 kHz on the WET ONLY, standing in for the
//      SAD1024 BBD's brutal roll-off.
// The delay window itself was never the problem: v1's 0.45..3.55 ms already
// sat inside the model's 0.3..4.0 ms.
#define FL_N     512u
#define FL_MASK  (FL_N - 1u)
#define FL_MIN    14.65f         // 0.3 ms at FX_FS
#define FL_MAX   195.31f         // 4.0 ms at FX_FS
#define FL_FMIN   24.41f         // 0.5 ms: FILTER MATRIX low end
#define FL_FMAX   97.66f         // 2.0 ms: FILTER MATRIX high end
#define FL_RISE    0.55f         // LFO duty: rise 55%, fall 45%
#define FL_FB_HP   0.980883f     // one-pole HP @ 150 Hz in the loop
#define FL_FB_LP   0.537949f     // one-pole LP @ 6 kHz in the loop
// The model's Y = 0.5*X + 0.5*Wet halves the DRY, which alone costs 6 dB --
// measured -4.0 dB vs dry on real playing. A pedal that drops 4-9 dB when you
// step on it reads as broken, and the vintage unit's famous drop is 1.5 dB, not
// nine. FL_MAKEUP restores the 50/50 BLEND to unity so FL_TRIM alone sets the
// drop; measured -1.6 dB vs dry, which is the authentic number.
// (For reference the OLD flanger BOOSTED +5.0 dB, part of why it overwhelmed.)
// DO NOT treat this as a flanger trim. fx_flange_process() runs on EVERY
// block whether the flanger is engaged or not (main.c audio chain), so this
// constant is really a GLOBAL output gain that happens to live in a flanger
// #define. Raising it to 1.80 on 2026-08-03 to chase the flanger's own 2.7 dB
// deficit lifted the entire instrument by 2.7 dB and pushed AUTO WAH, OD and
// SHRED to 0.0 dBFS -- three effects into clipping, for no flanger benefit at
// all. Back to 1.32; the flanger's deficit is fixed in its blend below, which
// is the only place that can move the wet path without moving everything.
#define FL_MAKEUP  1.32f
// FL_TRIM was 0.841395f, the vintage unit's authentic -1.5 dB drop. REMOVED
// 2026-08-01. Measured on real playing it put FLANGE at -1.86 dB and MATRIX at
// -3.09 dB below dry, and the bench felt it at once: "enabling the effect on its
// own weakens the overall signal... like a 30% volume drop". -3.09 dB IS 30% in
// amplitude, so that report was accurate to a hair. An effect that costs obvious
// level when engaged reads as weak -- the same complaint that got OD's level
// raised. With the trim gone: FLANGE -0.36 dB, MATRIX -1.59 dB at COLOR 6, +1.30
// at COLOR 10. MATRIX stays slightly down because a FROZEN comb's notches never
// move, so its losses never average out the way a sweep's do; judged acceptable
// rather than worth a mode-dependent makeup.
// FL_MAKEUP stays -- it corrects the structural 6 dB the 50/50 mix costs the
// dry, which is arithmetic, not an authenticity choice.
#define FL_TRIM    1.0f
#define FL_SHELF_A 0.012786f     // one-pole LP @ 100 Hz for the bass shelf
#define FL_SHELF_G 0.109f        // ...cutting below 100 Hz by ~1 dB
static int16_t  fl_buf[FL_N];
static uint32_t fl_w;
static float    fl_ph;                    // asymmetric-triangle LFO phase 0..1
static float    fl_k;                     // LFO phase step per sample
static float    fl_mix, fl_tgt;
static float    fl_fb = 0.595f;           // COLOR 0..0.85 (default knob 7)
static float    fl_fb_hp_y, fl_fb_hp_x;   // loop HP state
static float    fl_fb_lp;                 // loop LP state
static float    fl_shelf;                 // output bass-shelf state
static int      fl_matrix = 0;            // 1 = FILTER MATRIX (LFO frozen)
static float    fl_static = 48.0f;        // FILTER MATRIX delay TARGET, samples
// ...and the slewed value actually used. RATE is an integer 1..10 knob, so in
// MATRIX mode the static delay lands on ten discrete positions between 0.5 and
// 2.0 ms -- about 8 SAMPLES (0.167 ms) per step. Jumping the read pointer 8
// samples instantly is a discontinuity, i.e. a click at every step: bench
// verdict 2026-08-01 was "major artifacting when you hit each of the levels...
// totally digital, no smooth sweep". Every other parameter here is slewed; this
// one was not. ~25 ms time constant: smooth enough to kill the click, fast
// enough to still feel responsive under a finger. The residual is a ~11 cent
// momentary bend while it glides, which is musical rather than a click.
#define FL_STAT_SLEW 0.0008f
static float    fl_stat_cur = 48.0f;
// 6th-order Butterworth LP @ 9 kHz, fs = 48828: three cascaded RBJ sections.
// Q = 0.5176 / 0.7071 / 1.9319. Verified -0.01 dB at 6 kHz, -3.0 at 9 k,
// -20.8 at 12 k, -48.7 at 16 k.
static const float FL_BW[3][5] = {
    { 0.158883053f, 0.317766107f, 0.158883053f, -0.425567536f, 0.061099749f },
    { 0.181744469f, 0.363488937f, 0.181744469f, -0.486801733f, 0.213779607f },
    { 0.242074807f, 0.484149613f, 0.242074807f, -0.648396268f, 0.616695495f },
};
static float fl_bw_z[3][2];               // per-section biquad state

#if FX_ENABLE_VOWEL   /* disabled 2026-08-01, see fx.h */
// ---- Vowel filter state (DSP at the end of the file) ----------------------
// This replaced FREEZE's 8 KB grain buffer, and those 8 KB are what paid for the
// BBD chorus line. Note what is NOT here: a delay line. Two biquads, so the
// effect adds zero latency.
static float vw_mix, vw_tgt;
static float vw_pos_v = 2.0f;        // vowel path position, 0..VW_NV-1
static float vw_ph;                  // AUTO-mode LFO phase, 0..1
static float vw_k = 0.00002f;        // LFO step per sample (RATE)
static int   vw_manual;              // 1 = POSITION knob drives the vowel
static float vw_man_pos = 2.0f;      // MANUAL target ("ah")
// Two RBJ bandpass sections (constant peak gain), coefficients staged per block.
static float vw_b0[2], vw_b1[2], vw_b2[2], vw_a1[2], vw_a2[2];
static float vw_z1[2], vw_z2[2];

#endif

static void rv_init_offsets(void) {
    rv_sh.win = RV_SH_WIN;       // the shimmer shifter's grain window
    uint16_t o = 0;
    for (int c = 0; c < RV_NC; c++) { rv_coff[c] = o; o += RV_CLEN[c]; }
    for (int a = 0; a < RV_NA; a++) { rv_aoff[a] = o; o += RV_ALEN[a]; }
    rv_ready = 1;
}

void fx_reset(void) {
    memset(&sh_pad, 0, sizeof sh_pad);
    memset(&sh_oct, 0, sizeof sh_oct);
    sh_pad.win = (float)WIN;          // smooth 64 ms grains
    sh_oct.win = (float)WIN;
    memset(gbuf, 0, sizeof gbuf);
    memset(sbuf, 0, sizeof sbuf);
    sw_ = 0;
    ggw = 0; g_looping = 0; g_env = 0.0f;
    slap_mix = slap_tgt = 0.0f;
    met_mix = met_tgt = 0.0f;
    met_hp_y = met_hp_x = 0.0f;
    met_bq_z1 = met_bq_z2 = 0.0f;
    met_lp1 = met_lp2 = 0.0f;
    od_mix = od_tgt = 0.0f;
    od_hp_y = od_hp_x = 0.0f;
    od_blp = 0.0f;
    od_dc_y = od_dc_x = 0.0f;
    od_lp1 = 0.0f;
#if FX_ENABLE_TREMOLO
    trm_mix = trm_tgt = 0.0f;
    trm_ph = 0.0f; trm_g = 1.0f;
#endif
#if FX_ENABLE_PHASER
    ph_mix = ph_tgt = 0.0f;
    memset(ph_x1, 0, sizeof ph_x1);
    memset(ph_y1, 0, sizeof ph_y1);
    ph_wet = 0.0f; ph_lfo = 0.0f; ph_a = -0.9f;
#endif
    memset(&sh_tune, 0, sizeof sh_tune);
    sh_tune.win = 1172.0f;            // tight 24 ms grains: mechanical splice
    memset(eq_z, 0, sizeof eq_z);
    eq_mix = eq_tgt = 0.0f;
    memset(tun_win, 0, sizeof tun_win);
    memset(psa_ring, 0, sizeof psa_ring);
    memset(psa_g, 0, sizeof psa_g);
    psa_w = 0; psa_P = 384.0f; psa_nse = 0.0f;
    psa_ac = 384.0f; psa_A[0] = psa_A[1] = 0;
    if (!psa_tab_ok) {                // Hann table, built once
        for (int i = 0; i <= 256; i++)
            psa_tab[i] = 0.5f - 0.5f * cosf(6.2831853f * (float)i / 256.0f);
        psa_tab_ok = 1;
    }
    tun_mix = tun_tgt = 0.0f;
    tun_corr = tun_corr_tgt = 0.0f;
    tun_note = -1;
    tun_m3n = 0;
    par_mix = par_tgt = 0.0f;
    par_genv = par_gate = 0.0f;
    par_fenv = 0.0f;
    par_hp_y = par_hp_x = 0.0f;
    par_dc_y = par_dc_x = 0.0f;
    par_bq_z1 = par_bq_z2 = 0.0f;
    par_lp1 = 0.0f;
    par_ic1 = par_ic2 = 0.0f;
    par_g = 0.049f;
    cur_rate = tgt_rate = 1.0f;
    cur_wet  = tgt_wet  = 0.0f;
    oct_mix  = oct_tgt  = 0.0f;
    oct_on_v = 0;
    // Keep MODE/MIX/DETUNE (they are UI settings, not state) but land the rate
    // on target so re-entering the page does not glide up from the old pitch.
    oct_rate_recalc();
    oct_rate = oct_rate_tgt;
    // Reverb tank: clear the lines and the damping states, or a tail from the
    // previous page bleeds in when the chain is re-entered.
    memset(rv_buf, 0, sizeof rv_buf);
    for (int c = 0; c < RV_NC; c++) { rv_cw[c] = 0; rv_lp[c] = 0.0f; }
    for (int a = 0; a < RV_NA; a++) rv_aw[a] = 0;
    rv_mix = rv_tgt = 0.0f;
    memset(&rv_sh, 0, sizeof rv_sh);
    rv_sh.win = RV_SH_WIN;             // memset clears it, so set it after
    rv_sh_lp = rv_sh_fb = 0.0f;
    bc_mix = bc_tgt = 0.0f; bc_hold = 0.0f; bc_cnt = 0;
#if FX_ENABLE_RINGMOD
    rm_mix = rm_tgt = 0.0f; rm_s = 0.0f; rm_c = 1.0f;
#endif
    memset(fl_buf, 0, sizeof fl_buf);
    fl_w = 0; fl_ph = 0.0f; fl_mix = fl_tgt = 0.0f;
    fl_stat_cur = fl_static;      // land on target: entering the page must not
                                  // glide up from a stale position
    fl_fb_hp_y = fl_fb_hp_x = fl_fb_lp = 0.0f;
    fl_shelf = 0.0f;
    memset(fl_bw_z, 0, sizeof fl_bw_z);
#if FX_ENABLE_VOWEL
    vw_mix = vw_tgt = 0.0f;
    vw_ph = 0.0f; vw_pos_v = 2.0f;          // start on "ah"
    vw_z1[0] = vw_z1[1] = vw_z2[0] = vw_z2[1] = 0.0f;
#endif
}

// x = pitch, y = how much of the shifted voice to blend in. The y axis used to
// drive a 300 ms echo with its own 32 KB delay line; DELAY does that job
// properly now, and reclaiming those 32 KB is what pays for the reverb's
// 8-comb tank and the shimmer shifter. A wet/dry blend is the more useful
// second axis for a pitch pad anyway.
// pitch_oct is in OCTAVES, clamped to the pad's full-width range (+/-1.5). The
// clamp used to be +/-1.0, which silently swallowed the outer third of the axis
// once main.c widened the range to keep +/-1 OCT within thumb reach.
void fx_set_params(float pitch_oct, float fx_amt) {
    if (pitch_oct >  1.5f) pitch_oct =  1.5f;
    if (pitch_oct < -1.5f) pitch_oct = -1.5f;
    if (fx_amt < 0.0f) fx_amt = 0.0f;
    if (fx_amt > 1.0f) fx_amt = 1.0f;
    tgt_rate = exp2f(pitch_oct);
    tgt_wet  = fx_amt;
}

// Linear-interpolated read `delay` samples behind the pitch write head.
void fx_process(const int16_t *in, int16_t *out, int n) {
    // Per-block parameter slew (~4 blocks / 64 ms to settle): kills zipper
    // noise from discrete touch positions and gives click-free touch release.
    cur_rate += 0.25f * (tgt_rate - cur_rate);
    cur_wet  += 0.25f * (tgt_wet  - cur_wet);

    for (int i = 0; i < n; i++) {
        float shifted = sh_tick(&sh_pad, cur_rate, in[i]);
        float y = (1.0f - cur_wet) * (float)in[i] + cur_wet * shifted;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

// rate = 2^(semitones/12). DETUNE spans +/-1 semitone around unity, taken from
// the slider as (pct-50)/50 semitones -- so 50 lands on rate 1.0 exactly, where
// the shifter's taps freeze into a constant delay and the wet voice is a clean
// copy of the dry. That is the correct "in tune" behaviour and not a special
// case in the code.
static void oct_rate_recalc(void) {
    static const float SEMI[4] = { 12.0f, 24.0f, -12.0f, -24.0f };
    float semis = (oct_mode_v < 4) ? SEMI[oct_mode_v]
                                   : (float)(oct_det_pct - 50) / 50.0f;
    oct_rate_tgt = exp2f(semis / 12.0f);
}

void fx_octave_set(int on) {
    oct_on_v = on ? 1 : 0;
    oct_tgt  = oct_on_v ? oct_wet : 0.0f;
}

// MODE 0..4: +1 oct, +2 oct, -1 oct, -2 oct, detune.
void fx_octave_mode(int mode) {
    if (mode < 0) mode = 0;
    if (mode > 4) mode = 4;
    oct_mode_v = mode;
    oct_rate_recalc();
}

// MIX 0..100 % wet. Applies live whether the effect is on or off.
void fx_octave_mix(int pct) {
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    oct_wet = (float)pct * 0.01f;
    if (oct_on_v) oct_tgt = oct_wet;
}

// DETUNE 0..100 -> -100..+100 cents (50 = in tune). Only MODE 4 reads it.
void fx_octave_detune(int pct) {
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    oct_det_pct = pct;
    if (oct_mode_v == 4) oct_rate_recalc();
}

void fx_octave_process(const int16_t *in, int16_t *out, int n) {
    oct_mix  += 0.25f * (oct_tgt - oct_mix);        // click-free toggle
    // Slewing the RATE too means a mode change glides over ~64 ms instead of
    // stepping, and dragging the detune slider does not zipper.
    oct_rate += 0.25f * (oct_rate_tgt - oct_rate);
    // EQUAL-POWER crossfade, not linear. The shifted signal is decorrelated
    // from the dry, so the two sum incoherently: a linear 50/50 blend lands at
    // sqrt(0.5^2 + 0.5^2) = 0.707, i.e. 3 dB DOWN, which is why engaging
    // OCTAVE measurably dropped the output (bench 2026-08-03: -3.8 dB against
    // bypass, and "some effects still brought the volume down" from the family
    // demo). sqrt-law coefficients hold unity at every mix position, including
    // both extremes, instead of only being patched at 50%.
    float g_dry = sqrtf(1.0f - oct_mix), g_wet = sqrtf(oct_mix);
    for (int i = 0; i < n; i++) {
        float wet = sh_tick(&sh_oct, oct_rate, in[i]);
        float y = g_dry * (float)in[i] + g_wet * wet;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

// ---- Glitch: random sample-and-hold takeover looper -------------------------
//
// "Best of all ideas" hybrid (user-directed, 2026-07-17): deliberately
// wilder than the strict recreation. Grounded in what we learned —
//   - reference measurements: loops cluster 30-90 ms, episodes 0.3-2 s;
//   - patch physics: no pitch-shifting — micro-loops (< 30 ms) read as
//     synthetic notes at 1/period, and tape-style varispeed jumps/sags;
//   - a Max recreation re-rolls the loop size on a metro mid-burst;
//   - hard splice clicks read as a false pitch rise (user-reported), so
//     seams get a ~1 ms blend; the dry is fully muted while looping.
// A burst is a chain of short PATTERNS (0.1-0.5 s each). Every pattern
// rolls its own loop length, its own tape speed (1x favored, 0.5x..2x
// jumps, constant within the pattern so each stutter has a stable pitch),
// and its own source window — 50% the freshest audio, 50% anywhere in the
// last second of playing (the broken-CD laser skipping around the disc).
//
// A rolling recorder captures the dry signal continuously. A free-running
// random countdown decides when to glitch; recording stops while looping
// (the 100%-feedback-delay-that-stops-listening behavior), the dry is cut
// with a short crossfade, and the episode ends on a whole repeat before
// the dry returns and the countdown rearms.
static const float G_RATES[6] = { 0.5f, 0.75f, 1.0f, 1.25f, 1.5f, 2.0f };

// Start a new stutter pattern: length 25% micro drone (5-30 ms) / 55%
// stutter (30-90 ms, the measured cluster) / 20% chop (90-200 ms); speed
// 40% locked 1x, else uniform from G_RATES; window pinned at the freeze
// moment or thrown anywhere into history. The fragment tail gets a ~1 ms
// blend toward its head's original prelude so wraps don't click. (Blends
// write into gbuf history — repeated blending mushes ~1 ms spots,
// inaudible.)
static void g_new_seg(void) {
    uint32_t f = rng() % 100u;
    if (f < 25u)      g_len = rng_range(FX_MS(5),  FX_MS(30));
    else if (f < 80u) g_len = rng_range(FX_MS(30), FX_MS(90));
    else              g_len = rng_range(FX_MS(90), FX_MS(200));
    uint32_t rv = rng() % 100u;
    g_rate = (rv < 40u) ? 1.0f : G_RATES[rng() % 6u];
    if (rng() & 1u)
        g_start = ggw - g_len;                        // freshest audio
    else                                              // skip around the disc
        g_start = ggw - g_len - rng_range(0u, GBUF_N - g_len - 96u);
    if (g_len >= 192u) {
        for (uint32_t k = 0; k < 48u; k++) {
            float w = (float)(k + 1) * (1.0f / 49.0f);
            uint32_t ti = (g_start + g_len - 48u + k) & GMASK;
            float pre = (float)gbuf[(g_start - 48u + k) & GMASK];
            gbuf[ti] = (int16_t)((1.0f - w) * (float)gbuf[ti] + w * pre);
        }
    }
    g_pos = 0.0f;
}

static void g_trigger(void) {
    g_t_left   = rng_range(FX_MS(300), FX_MS(2000));  // episode: 0.3 .. 2 s
    g_seg_left = rng_range(FX_MS(100), FX_MS(500));   // pattern every 0.1-0.5 s
    g_new_seg();
    g_looping = 1;
}

void fx_glitch_set(int on) {
    g_on = on;
    if (!on) g_looping = 0;                 // env slews back to dry, no click
    g_wait = rng_range(g_gap_lo, g_gap_hi); // random gap to first/next burst
}

// FREQ 1..10: how often bursts take over. Scales the random rearm-gap range
// exponentially — 10 = 0.15..1.25 s (near-constant barrage), 6 = 0.45..3.8 s,
// 1 = ~1.8..15 s (rare surprises). Gaps stay random within the range.
void fx_glitch_freq(int f) {
    if (f < 1)  f = 1;
    if (f > 10) f = 10;
    float s = exp2f(0.4f * (float)(10 - f));
    g_gap_lo = (uint32_t)((float)FX_MS(150) * s);
    g_gap_hi = (uint32_t)((float)FX_MS(1250) * s);
}

int fx_glitch_active(void) { return g_looping; }

void fx_glitch_process(const int16_t *in, int16_t *out, int n) {
    for (int i = 0; i < n; i++) {
        float wet = 0.0f;
        if (!g_looping) {
            gbuf[++ggw & GMASK] = in[i];    // record only while not glitching
            if (g_on && ggw >= GBUF_N) {    // need a full second of history
                if (g_wait) g_wait--;
                else        g_trigger();
            }
        } else {
            // Replay of the frozen fragment at full volume — identical every
            // repeat, at the episode's fixed rate (1x, or an occasional
            // whole-episode 0.5x/2x varispeed). Seam is pre-blended.
            uint32_t di = (uint32_t)g_pos;
            float    fr = g_pos - (float)di;
            int16_t  a = gbuf[(g_start + di)      & GMASK];
            int16_t  b = gbuf[(g_start + di + 1u) & GMASK];
            wet = (1.0f - fr) * (float)a + fr * (float)b;
            g_pos += g_rate;
            if (g_t_left)  g_t_left--;
            if (g_seg_left) g_seg_left--;
            if (g_pos >= (float)g_len) {                         // repeat done
                g_pos -= (float)g_len;
                if (g_t_left == 0u) {                            // glitch over
                    g_looping = 0;
                    g_wait = rng_range(g_gap_lo, g_gap_hi);      // rearm gap
                } else if (g_seg_left == 0u) {                   // new pattern:
                    g_new_seg();                                 // length, speed
                    g_seg_left = rng_range(FX_MS(100), FX_MS(500)); // + window jump
                }
            }
        }
        // Dry <-> wet crossfade (~3 ms): dry is CUT while looping.
        float tgt = g_looping ? 1.0f : 0.0f;
        g_env += 0.0066f * (tgt - g_env);
        float y = (1.0f - g_env) * (float)in[i] + g_env * wet;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

// ---- Delay: one line, slapback through long repeats -----------------------
// Reuses the 16384-sample line the old single-repeat SLAP owned, so the
// maximum time is 335 ms at FX_FS. FEEDBACK at 0 reproduces SLAP exactly (one
// repeat); above 0 the tap is fed back into the line for a decaying train.
// The tap position still slews per-sample (~125 ms constant) so dragging TIME
// glides like a tape echo's varispeed head instead of stepping.
#define DLY_MIN_MS  40
#define DLY_MAX_MS  330

void fx_delay_set(int on) { slap_tgt = on ? 1.0f : 0.0f; }

void fx_delay_time_ms(int ms) {
    if (ms < DLY_MIN_MS) ms = DLY_MIN_MS;
    if (ms > DLY_MAX_MS) ms = DLY_MAX_MS;
    slap_d_tgt = (float)ms * ((float)FX_FS / 1000.0f);   // ms -> samples
}

// FEEDBACK 0..10 -> 0 .. 0.85. Capped below 1 so the line cannot run away;
// 0.85 already gives a long, clearly audible train.
void fx_delay_fb(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    dly_fb = 0.085f * (float)amount;
}

// MIX 0..10 -> echo level 0 .. 1.0 against a dry that always passes at full
// level, so this is an add, never a replacement.
void fx_delay_mix(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    dly_lvl = 0.1f * (float)amount;
}

void fx_delay_process(const int16_t *in, int16_t *out, int n) {
    slap_mix += 0.25f * (slap_tgt - slap_mix);   // click-free toggle
    for (int i = 0; i < n; i++) {
        slap_d += 0.000164f * (slap_d_tgt - slap_d);  // tape-style time glide
        // Read at the CURRENT write position before advancing, so the
        // feedback term has the echo it needs before the new sample lands.
        int   di = (int)slap_d;
        float fr = slap_d - (float)di;
        int16_t a = sbuf[(sw_ - (uint32_t)di)      & SMASK];
        int16_t b = sbuf[(sw_ - (uint32_t)di - 1u) & SMASK];
        float echo = (1.0f - fr) * (float)a + fr * (float)b;
        // Line input scaled by (1 - fb) so the train cannot build past the
        // source level. Without it the line settles at in/(1-fb) -- 6.7x the
        // input at fb 0.85 -- and SATURATES internally, which the output clamp
        // counter cannot see because the damage happens upstream of it. At
        // fb = 0 this is bit-identical to the old single-repeat slapback.
        float wr = (float)in[i] * (1.0f - dly_fb) + dly_fb * echo;
        wr = fxc_clamp(wr, 32700.0f, FXC_DELAY);
        sbuf[++sw_ & SMASK] = (int16_t)wr;
        // Constant-sum mix: dry + w*echo divided by (1 + w). Bit-identical to
        // a plain add at MIX 0, keeps the dry-to-echo RATIO untouched, and
        // bounds the total by the louder of the two instead of letting them
        // sum past full scale. Measured at maximum settings before this, a
        // sustained take reached -0.0 dBFS with nothing left to give.
        // Denominator is sqrt(1 + w*w), not (1 + w). The echo is a
        // time-shifted copy and therefore DECORRELATED from the dry, so the
        // pair sums in power, not amplitude: dividing by the amplitude sum
        // over-attenuates by up to 3 dB (bench 2026-08-03: engaging DELAY
        // measured -2.9 dB against bypass). The bound this was protecting
        // still holds -- sqrt(1+w*w) is the correct normaliser for
        // incoherent summation, so the total still cannot run away -- and the
        // output clamp below is unchanged.
        float w = slap_mix * dly_lvl;
        float y = ((float)in[i] + w * echo) / sqrtf(1.0f + w * w);
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

// ---- Metal: scooped-mids high-gain distortion (thrash-rhythm recipe) --------
// Chain (all normalized float):
//   1. one-pole HP @ ~120 Hz  — tightens lows BEFORE clipping (palm-mute chunk
//      stays defined instead of flubbing out; the classic trick of a
//      mid-focused overdrive pushing a scooped high-gain amp)
//   2. drive gain (5..60x) into a tanh-like soft clipper (Padé approx) + clamp
//   3. peaking EQ biquad: -12 dB at 650 Hz, Q 0.7 — THE mid scoop
//   4. two one-pole LPs @ ~4.2 kHz — 12 dB/oct fizz rolloff (4x12 cab-ish)
// RBJ biquad coefficients precomputed for fs=48828, f0=650, Q=0.7, -12 dB.
#define MET_B0  0.92033f
#define MET_B1 -1.78096f
#define MET_B2  0.86688f
#define MET_A1 -1.78096f
#define MET_A2  0.78720f
#define MET_HP_A  0.9847f    // one-pole HP @ ~120 Hz
#define MET_LP_A  0.4878f    // one-pole LP @ ~5.2 kHz (brighter = more bite)
#define MET_LEVEL 0.36f      // makeup/output level after clipping

void fx_metal_set(int on)      { met_tgt = on ? 1.0f : 0.0f; }
void fx_metal_drive(int amount) {
    if (amount < 5)  amount = 5;
    if (amount > 60) amount = 60;
    met_drive = (float)amount;
}

void fx_metal_process(const int16_t *in, int16_t *out, int n) {
    met_mix += 0.25f * (met_tgt - met_mix);      // click-free toggle
    for (int i = 0; i < n; i++) {
        float x = (float)in[i] * (1.0f / 32768.0f);
        // 1. tight pre-highpass
        met_hp_y = MET_HP_A * (met_hp_y + x - met_hp_x);
        met_hp_x = x;
        // 2. drive -> clipper. Crunch tuning: clamp at +/-1.8 (was 3.0) so the
        // wave slams the hard ceiling sooner - squarer edges, more odd
        // harmonics, less "smooth fuzz", more "crunch".
        float d = met_hp_y * met_drive;
        if (d >  1.8f) d =  1.8f;
        if (d < -1.8f) d = -1.8f;
        float c = d * (27.0f + d * d) / (27.0f + 9.0f * d * d);
        // 3. mid scoop (transposed direct form II)
        float y = MET_B0 * c + met_bq_z1;
        met_bq_z1 = MET_B1 * c - MET_A1 * y + met_bq_z2;
        met_bq_z2 = MET_B2 * c - MET_A2 * y;
        // 4. cab-ish rolloff
        met_lp1 += MET_LP_A * (y - met_lp1);
        met_lp2 += MET_LP_A * (met_lp1 - met_lp2);
        float wet = met_lp2 * MET_LEVEL * 32767.0f;
        float o = (1.0f - met_mix) * (float)in[i] + met_mix * wet;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}

// ---- Paranoid: envelope-swept resonant filter over a moderate crunch --------
// Models the reference lead's actual rig: a low-gain British crunch pedal
// (gain ~3/10, tight bass, deep mid contour, dark treble) feeding an
// envelope-controlled resonant lowpass (studio rack filter / envelope-filter
// pedal whose RANGE knob is ridden while playing). The sweep IS the sound:
// pick attacks throw the cutoff up toward the RANGE ceiling and it glides
// back down as the note decays — a vocal wah-like motion. Verified in
// simulation: attack opens to ~2.2 kHz, decays through ~1.2 kHz to ~670 Hz.
// A noise gate in front kills idle hiss (the earlier fuzz voicing amplified
// the ADC floor by ~40 dB; this chain runs far less gain AND gates).
// Chain: gate -> HP 150 Hz -> drive 12 -> soft clip -> DC block ->
//        mid contour (-8 dB @ 750 Hz) -> LP 4.2 kHz -> env-swept SVF LP
//        (Q=5, 250 Hz..RANGE ceiling) -> level.
#define PARA_GATE_TH 0.0015f  // gate threshold (~49 counts of int16). Was
                              // 0.004: bench guitar decay tails were still
                              // audible below that and got chopped mid-fade
                              // (2026-07-28). Idle ADC hiss sits well under
                              // 49 counts, so the gate still closes at rest.
#define PARA_HP_A    0.9809f  // pre-drive one-pole HP @ ~150 Hz (tight bass)
#define PARA_DRIVE   12.0f    // low-gain crunch (clipper input gain)
// RBJ peaking biquad, fs=48828, f0=750 Hz, Q=0.7, -8 dB (the mid contour).
#define PARA_B0  0.94080f
#define PARA_B1 -1.79489f
#define PARA_B2  0.86249f
#define PARA_A1 -1.79489f
#define PARA_A2  0.80329f
#define PARA_TRE_A   0.4175f  // one-pole LP @ ~4.2 kHz (dark treble)
#define PARA_SVF_K   0.2f     // SVF damping = 1/Q (Q = 5: juicy, no self-osc)
#define PARA_FMIN    250.0f   // sweep floor (Hz)
// Envelope -> sweep sensitivity. DELIBERATELY high enough to saturate, and
// that is not a bug -- it is where the quack comes from. s = fenv * SENS is
// clamped at 1.0, so every note slams the filter wide open on the attack and
// then closes as the envelope decays: the sweep lives in the DECAY, and pick
// force controls how LONG it stays open rather than how high it opens. Pinch
// harmonics work for exactly that reason (sharp spike, fast decay).
//
// Tried 3.5 on 2026-07-30 to make the peak proportional to pick force. It
// measured like better tracking and sounded worse -- "the difference before was
// better, it's too open by default now" -- so it went back to 12. A lesson
// about trusting a correlation over an ear.
#define PARA_SENS    12.0f
#define PARA_LEVEL   0.18f    // output level (~unity RMS with filter open)

void fx_para_set(int on) { par_tgt = on ? 1.0f : 0.0f; }

// RANGE 0..10 -> sweep ceiling 900..3200 Hz (log taper, like the pedal knob).
// The FLOOR is lifted from the original 600 Hz because below RANGE 6 the old
// ceiling never cleared 1.6 kHz and the bottom of the knob could only sound
// muddy. The TOP is back at the original 3200 Hz: a bench listen on
// 2026-07-30 tried 4500 and the verdict was "too open by default", with the
// sweet spots sitting at RANGE 7-8 (~2.2-2.5 kHz here). So: dark end usable,
// sweet spots where the ear put them.
void fx_para_range(int r) {
    if (r < 0)  r = 0;
    if (r > 10) r = 10;
    par_fmax = 900.0f * exp2f(0.183f * (float)r);
}

void fx_para_process(const int16_t *in, int16_t *out, int n) {
    par_mix += 0.25f * (par_tgt - par_mix);      // click-free toggle
    for (int i = 0; i < n; i++) {
        float x = (float)in[i] * (1.0f / 32768.0f);
        float ax = x < 0.0f ? -x : x;
        // 1. noise gate: fast-attack/slow-release follower drives a smooth
        //    gain ramp (~6 ms open, ~100 ms close — a closing gate should
        //    sound like the note fading, not a cut) — idle hiss goes to zero.
        par_genv += (ax > par_genv ? 0.0164f : 0.000164f) * (ax - par_genv);
        float gtgt = par_genv > PARA_GATE_TH ? 1.0f : 0.0f;
        par_gate += (gtgt > par_gate ? 0.0033f : 0.0002f) * (gtgt - par_gate);
        float xg = x * par_gate;
        // 2. sweep follower (keyed off the gated dry signal)
        float axg = ax * par_gate;
        par_fenv += (axg > par_fenv ? 0.0066f : 0.000131f) * (axg - par_fenv);
        // 3. crunch: tight HP -> low drive -> soft clip -> DC block
        par_hp_y = PARA_HP_A * (par_hp_y + xg - par_hp_x);
        par_hp_x = xg;
        float d = par_hp_y * PARA_DRIVE;
        if (d >  3.0f) d =  3.0f;
        if (d < -3.0f) d = -3.0f;
        float c = d * (27.0f + d * d) / (27.0f + 9.0f * d * d);
        float dc = c - par_dc_x + 0.9984f * par_dc_y;
        par_dc_x = c; par_dc_y = dc;
        // 4. mid contour (transposed direct form II) + dark treble
        float y = PARA_B0 * dc + par_bq_z1;
        par_bq_z1 = PARA_B1 * dc - PARA_A1 * y + par_bq_z2;
        par_bq_z2 = PARA_B2 * dc - PARA_A2 * y;
        par_lp1 += PARA_TRE_A * (y - par_lp1);
        // 5. envelope -> cutoff; tanf refreshed every 16 samples (1 ms) only
        if ((i & 15) == 0) {
            float s = par_fenv * PARA_SENS;
            if (s > 1.0f) s = 1.0f;
            float fc = PARA_FMIN + (par_fmax - PARA_FMIN) * powf(s, 0.7f);
            par_g = tanf(3.14159265f * fc * (1.0f / (float)FX_FS));
        }
        // 6. TPT state-variable lowpass (stable across the whole sweep)
        float a1 = 1.0f / (1.0f + par_g * (par_g + PARA_SVF_K));
        float a2 = par_g * a1;
        float a3 = par_g * a2;
        float v3 = par_lp1 - par_ic2;
        float v1 = a1 * par_ic1 + a2 * v3;
        float v2 = par_ic2 + a2 * par_ic1 + a3 * v3;
        par_ic1 = 2.0f * v1 - par_ic1;
        par_ic2 = 2.0f * v2 - par_ic2;
        float wet = v2 * PARA_LEVEL * 32767.0f;
        float o = (1.0f - par_mix) * (float)in[i] + par_mix * wet;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}

// ---- Overdrive: classic asymmetric-clip pedal ------------------------------
// The famous "super overdrive" recipe: a mid-hump gain shelf (lows below
// ~720 Hz get less gain into the clipper) and ASYMMETRIC clipping (one
// polarity clips earlier -> strong even harmonics at low drive, warming into
// odd-rich crunch as drive rises; verified numerically: H2 -22 dB at drive 2,
// H3 rising to -13 dB by drive 10). Dark-ish tone rolloff after the clip.
// Chain: HP 105 Hz -> mid-hump shelf (+1.5x highs) -> drive+bias -> clamp
//        -> Pade tanh -> DC block -> LP 3.3 kHz -> level.
#define OD_HP_A    0.9866f    // input one-pole HP @ ~105 Hz
#define OD_BLP_A   0.0885f    // mid-hump knee: one-pole LP @ ~720 Hz
#define OD_SHELF   1.5f       // highs get 2.5x the low-frequency gain
#define OD_BIAS    0.30f      // asymmetry -> even-harmonic warmth
#define OD_TONE_A  0.346f     // tone one-pole LP @ ~3.3 kHz
// Raised 0.11 -> 0.20 on 2026-07-31. Bench verdict was "warmer, just adding a
// little gain, not an overwhelming effect", and the measurement agreed: OD
// chords sat at -31.2 dBFS against SHRED's -23.3, so it was 7.9 dB quieter than
// its own neighbour on the grid. LEVEL 0.20 with the default DRIVE at 6 closes
// that gap exactly (+7.9 dB) and still peaks at only -13.1 dBFS.
// Chain simulation was validated against the real capture to 0.9 dB first --
// an earlier attempt at this judged a clipper by PEAK and got it backwards.
#define OD_LEVEL   0.20f

void fx_od_set(int on) { od_tgt = on ? 1.0f : 0.0f; }

// DRIVE knob 1..10 -> clipper input gain 2.5..25.
void fx_od_drive(int amount) {
    if (amount < 1)  amount = 1;
    if (amount > 10) amount = 10;
    od_gain = 2.5f * (float)amount;
}

void fx_od_process(const int16_t *in, int16_t *out, int n) {
    od_mix += 0.25f * (od_tgt - od_mix);         // click-free toggle
    for (int i = 0; i < n; i++) {
        float x = (float)in[i] * (1.0f / 32768.0f);
        od_hp_y = OD_HP_A * (od_hp_y + x - od_hp_x);
        od_hp_x = x;
        od_blp += OD_BLP_A * (od_hp_y - od_blp);
        float pre = od_hp_y + OD_SHELF * (od_hp_y - od_blp);
        float d = pre * od_gain + OD_BIAS;
        if (d >  3.0f) d =  3.0f;
        if (d < -3.0f) d = -3.0f;
        float c = d * (27.0f + d * d) / (27.0f + 9.0f * d * d);
        float dc = c - od_dc_x + 0.9984f * od_dc_y;
        od_dc_x = c; od_dc_y = dc;
        od_lp1 += OD_TONE_A * (dc - od_lp1);
        float wet = od_lp1 * OD_LEVEL * 32767.0f;
        float o = (1.0f - od_mix) * (float)in[i] + od_mix * wet;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}

#if FX_ENABLE_TREMOLO   /* disabled 2026-08-01, see fx.h */
// ---- Tremolo: smooth optical-style amplitude LFO ---------------------------
// out = in * (1 - depth*(0.5 + 0.5*sin)). Depth fixed at 0.7 (deep but never
// fully silent); the LFO gain is refreshed every 16 samples (1 ms) — far
// faster than any audible stepping at trem rates.
#define TRM_DEPTH 0.7f

void fx_trem_set(int on) { trm_tgt = on ? 1.0f : 0.0f; }

// RATE knob 1..10 = LFO speed 1..10 Hz.
void fx_trem_rate(int r) {
    if (r < 1)  r = 1;
    if (r > 10) r = 10;
    trm_inc = (float)r / (float)FX_FS;
}

void fx_trem_process(const int16_t *in, int16_t *out, int n) {
    trm_mix += 0.25f * (trm_tgt - trm_mix);      // click-free toggle
    for (int i = 0; i < n; i++) {
        if ((i & 15) == 0) {
            trm_ph += 16.0f * trm_inc;
            if (trm_ph >= 1.0f) trm_ph -= 1.0f;
            float s = sinf(6.2831853f * trm_ph);
            trm_g = 1.0f - TRM_DEPTH * (0.5f + 0.5f * s);
        }
        float wet = (float)in[i] * trm_g;
        float o = (1.0f - trm_mix) * (float)in[i] + trm_mix * wet;
        out[i] = (int16_t)o;                     // |o| <= |in|: no clamp needed
    }
}

#endif

#if FX_ENABLE_PHASER   /* disabled 2026-08-01, see fx.h */
// ---- Phaser: 4-stage swept allpass, 50/50 mix ------------------------------
// Four first-order allpass stages share one coefficient swept 250..1800 Hz
// (exponential triangle LFO, refreshed every 16 samples); light feedback
// (0.25) around the chain deepens the notches. Bench-simmed stable across
// the full sweep. Wet/dry 50/50 like the classic box (no depth knob).
#define PH_FB    0.25f
#define PH_FMIN  250.0f

void fx_phase_set(int on) { ph_tgt = on ? 1.0f : 0.0f; }

// RATE knob 1..10 -> LFO speed 0.1..3.25 Hz.
void fx_phase_rate(int r) {
    if (r < 1)  r = 1;
    if (r > 10) r = 10;
    ph_inc = (0.1f + 0.35f * (float)(r - 1)) / (float)FX_FS;
}

void fx_phase_process(const int16_t *in, int16_t *out, int n) {
    ph_mix += 0.25f * (ph_tgt - ph_mix);         // click-free toggle
    for (int i = 0; i < n; i++) {
        if ((i & 15) == 0) {
            ph_lfo += 16.0f * ph_inc;
            if (ph_lfo >= 1.0f) ph_lfo -= 1.0f;
            float tri = ph_lfo < 0.5f ? 2.0f * ph_lfo : 2.0f - 2.0f * ph_lfo;
            float f = PH_FMIN * exp2f(2.85f * tri);      // 250 -> ~1800 Hz
            float t = tanf(3.14159265f * f * (1.0f / (float)FX_FS));
            ph_a = (t - 1.0f) / (t + 1.0f);
        }
        float x = (float)in[i] * (1.0f / 32768.0f);
        float s = x + PH_FB * ph_wet;
        for (int k = 0; k < 4; k++) {            // y = a*x + x1 - a*y1
            float yk = ph_a * (s - ph_y1[k]) + ph_x1[k];
            ph_x1[k] = s;
            ph_y1[k] = yk;
            s = yk;
        }
        ph_wet = s;
        float wet = 0.5f * (x + s) * 32767.0f;
        float o = (1.0f - ph_mix) * (float)in[i] + ph_mix * wet;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}

#endif

// ---- Parametric EQ: 5 peaking bands (post-drive, pre-modulation) ------------
// Classic rack placement (user-specified): tone-shape the distorted signal
// before modulation/time effects. Each band is an RBJ peaking biquad
// (freq 40 Hz..12 kHz, gain +/-12 dB, Q 0.4..8). The UI core computes and
// STAGES coefficients (sinf/cosf off the audio core); the audio core copies
// the whole staged set at block start when the generation counter moves —
// a torn update costs at most one odd block, never instability-by-halves
// in steady state.
void fx_eq_set(int on) { eq_tgt = on ? 1.0f : 0.0f; }

// Compute + stage one band (call from the UI core). Flat gain short-circuits
// to identity so unused bands are bit-transparent.
void fx_eq_band(int i, float f0, float gain_db, float q) {
    if (i < 0 || i >= EQ_BANDS) return;
    if (f0 < 40.0f)     f0 = 40.0f;
    if (f0 > 12000.0f)  f0 = 12000.0f;
    if (gain_db >  12.0f) gain_db =  12.0f;
    if (gain_db < -12.0f) gain_db = -12.0f;
    if (q < 0.4f) q = 0.4f;
    if (q > 8.0f) q = 8.0f;
    if (gain_db > -0.05f && gain_db < 0.05f) {         // identity
        eq_stage[i][0] = 1.0f; eq_stage[i][1] = 0.0f; eq_stage[i][2] = 0.0f;
        eq_stage[i][3] = 0.0f; eq_stage[i][4] = 0.0f;
    } else {
        float A  = exp2f(gain_db * (1.0f / 12.041f));  // 10^(db/40)
        float w  = 6.2831853f * f0 / (float)FX_FS;
        float al = sinf(w) / (2.0f * q);
        float a0 = 1.0f + al / A;
        eq_stage[i][0] = (1.0f + al * A) / a0;
        eq_stage[i][1] = (-2.0f * cosf(w)) / a0;
        eq_stage[i][2] = (1.0f - al * A) / a0;
        eq_stage[i][3] = eq_stage[i][1];               // a1 == b1 for peaking
        eq_stage[i][4] = (1.0f - al / A) / a0;
    }
    eq_gen++;
}

void fx_eq_process(const int16_t *in, int16_t *out, int n) {
    static uint32_t seen = 0xFFFFFFFFu;
    if (eq_gen != seen) {
        seen = eq_gen;
        memcpy(eq_c, eq_stage, sizeof eq_c);
    }
    eq_mix += 0.25f * (eq_tgt - eq_mix);
    if (eq_tgt <= 0.0f && eq_mix < 0.001f) {
        if (out != in) memcpy(out, in, (size_t)n * sizeof out[0]);
        return;
    }
    for (int i = 0; i < n; i++) {
        float x = (float)in[i];
        float y = x;
        for (int b = 0; b < EQ_BANDS; b++) {           // TDF-II per band
            float v = eq_c[b][0] * y + eq_z[b][0];
            eq_z[b][0] = eq_c[b][1] * y - eq_c[b][3] * v + eq_z[b][1];
            eq_z[b][1] = eq_c[b][2] * y - eq_c[b][4] * v;
            y = v;
        }
        float o = (1.0f - eq_mix) * x + eq_mix * y;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}

// ---- Orcatune: real-time vocal pitch correction -----------------------------
// Per capture block: (1) detect the input's fundamental with normalized
// autocorrelation over lags 32..200 samples (80..500 Hz — vocal range;
// smallest peak >= 0.85 of the max avoids octave errors; parabolic refine
// gives cents accuracy — validated in simulation to <0.01 semitone),
// (2) snap to the nearest allowed note of the selected key/scale (with
// hysteresis so a boundary-hugging voice doesn't flap between notes),
// (3) slew a semitone correction toward the target — NATURAL glides
// (~120 ms, subtle repair) while ROBOTIC pins to the raw measurement (the
// hard-tune pitch-quantize effect), (4) resynthesize via TD-PSOLA.
//
// PSOLA (pitch-synchronous overlap-add): grains two pitch-periods long are
// cut from the input ring and replayed Hann-windowed at the TARGET period's
// spacing (Pout = P / rate). The waveform inside each grain is untouched —
// only the epoch spacing changes — so formants survive and the shift is far
// cleaner than resampling; the robotic character comes purely from the
// quantization staircase, which is the authentic hard-tune recipe. Grain
// level is scaled by Pout/P so the overlap sum stays at unity. Unvoiced
// input keeps the last period and relaxes the correction to unity, making
// the resynthesis near-transparent between notes.

// One output sample of PSOLA: push the input, track pitch-synchronous
// analysis epochs (pitch marks every P input samples — every grain is cut
// at the SAME waveform phase; without this the grains overlap-add back
// into the identity, sim-proven), maybe spawn a grain at the next synthesis
// epoch, and sum the active Hann-windowed grains.
static inline float psa_tick(int16_t in, float Pout) {
    psa_ring[++psa_w & PSA_MASK] = (int16_t)in;
    psa_ac -= 1.0f;
    if (psa_ac <= 0.0f) {                      // new pitch mark
        psa_A[1] = psa_A[0];
        psa_A[0] = psa_w;
        psa_ac += psa_P;
    }
    psa_nse -= 1.0f;
    if (psa_nse <= 0.0f) {
        for (int k = 0; k < PSA_GRAINS; k++) {
            if (psa_g[k].on) continue;
            float len = 2.0f * psa_P;
            if (len < 192.0f)  len = 192.0f;
            if (len > 2000.0f) len = 2000.0f;
            psa_g[k].len    = len;
            psa_g[k].ph     = 0.0f;
            psa_g[k].center = psa_A[1];        // >= 1 period old: the whole
            psa_g[k].amp    = Pout / psa_P;    //  window is already recorded
            psa_g[k].on     = 1;
            break;
        }
        psa_nse += (Pout > 48.0f) ? Pout : 48.0f;
    }
    float acc = 0.0f;
    for (int k = 0; k < PSA_GRAINS; k++) {
        grain_t *g = &psa_g[k];
        if (!g->on) continue;
        float w = psa_tab[(int)(g->ph * (256.0f / g->len))];
        uint32_t idx = g->center - (uint32_t)(g->len * 0.5f) + (uint32_t)g->ph;
        acc += w * g->amp * (float)psa_ring[idx & PSA_MASK];
        g->ph += 1.0f;
        if (g->ph >= g->len) g->on = 0;
    }
    return acc;
}
// Detection runs on a 3:1 DECIMATED stream (FX_FS/3 = 16,276 Hz) — nearly
// identical to the original bench-tuned 16 kHz rate, so the window and lag
// constants below are rate-invariant and detection CPU does not scale with
// the audio rate.
#define TUNE_DFS     ((float)FX_FS / 3.0f)
#define TUNE_LAG_MIN 32               // ~509 Hz
#define TUNE_LAG_MAX 232              // ~70 Hz (guitar low E + drop D)
#define TUNE_GATE_E  410000.0f        // energy gate: rms ~40 over 256 samps
#define TUNE_VOICED  0.50f            // min autocorr peak to count as pitched

// Allowed-note masks, one bit per pitch class relative to the key root.
// The sparse ones are the "abuse" settings: with only 2-3 legal notes,
// corrections reach multiple semitones and the classic yodel-snap appears.
static const uint16_t TUNE_MASKS[9] = {
    0x0AB5,                           // major  {0,2,4,5,7,9,11}
    0x05AD,                           // natural minor {0,2,3,5,7,8,10}
    0x0295,                           // major pentatonic {0,2,4,7,9}
    0x02B5,                           // penta + 4th {0,2,4,5,7,9} - fills the
                                      //   mid gap melodies pass through
    0x0A95,                           // penta + 7th {0,2,4,7,9,11} - fills
                                      //   the top gap below the octave
    0x0291,                           // major 6th chord {0,4,7,9}
    0x0091,                           // triad {0,4,7}
    0x0081,                           // root + fifth {0,7} - max drama
    0x0FFF,                           // chromatic
};

void fx_tune_set(int on) { tun_tgt = on ? 1.0f : 0.0f; }

void fx_tune_key(int root) {
    if (root >= 0 && root < 12) { tun_root = root; tun_note = -1; }
}

void fx_tune_scale(int s) {
    if (s >= 0 && s < 9) { tun_mask = TUNE_MASKS[s]; tun_note = -1; }
}

void fx_tune_mode(int robotic) { tun_robot = robotic ? 1 : 0; }

// Probe mode (tuner page): run pitch detection every block — readable via
// fx_tune_status — while the audio passes through untouched (unless the
// pitch corrector is also on, which keeps working normally).
static volatile int tun_probe = 0;   // UI core toggles, audio core reads
void fx_tune_probe(int on) { tun_probe = on ? 1 : 0; }

// Telemetry for RTT: last detected pitch (Hz x10, 0 = unvoiced) and the
// correction currently applied (cents).
void fx_tune_status(int *hz10, int *cents) {
    *hz10  = (int)(tun_freq_dbg * 10.0f);
    *cents = (int)(tun_corr * 100.0f);
}

static inline int tune_allowed(int note) {
    int pc = ((note % 12) - tun_root + 24) % 12;
    return (tun_mask >> pc) & 1;
}

void fx_tune_process(const int16_t *in, int16_t *out, int n) {
    if (tun_tgt <= 0.0f && tun_mix < 0.001f && !tun_probe) {
        if (out != in) memcpy(out, in, (size_t)n * sizeof out[0]);
        return;                                  // bypass: skip the analysis
    }
    // 3:1 decimation front-end (average of 3 = light anti-alias LPF), then
    // slide the 512-sample DECIMATED analysis window. ~85 new decimated
    // samples per 256-frame block; detection re-runs when 256 have
    // accumulated (~15.7 ms — the same cadence the detector was bench-tuned
    // at when the system ran 16 kHz).
    static int     dec_ph = 0, dec_new = 0;
    static int32_t dec_acc = 0;
    int16_t stage[192];
    int ns = 0;
    for (int i = 0; i < n; i++) {
        dec_acc += in[i];
        if (++dec_ph == 3) {
            stage[ns++] = (int16_t)(dec_acc / 3);
            dec_ph = 0; dec_acc = 0;
        }
    }
    memmove(tun_win, tun_win + ns, (size_t)(512 - ns) * sizeof(int16_t));
    memcpy(tun_win + 512 - ns, stage, (size_t)ns * sizeof(int16_t));
    dec_new += ns;
    int detect_now = dec_new >= 256;
    if (detect_now) dec_new = 0;

    // 1) Pitch detection (every ~15.7 ms).
    float freq = tun_freq_dbg;
    if (detect_now) {
        freq = 0.0f;
        const int16_t *a = tun_win + 256;
        float e1 = 0.0f;
        for (int i = 0; i < 256; i++) e1 += (float)a[i] * (float)a[i];
        if (e1 > TUNE_GATE_E) {
            static float cs[TUNE_LAG_MAX + 1];
            float cmax = 0.0f;
            for (int L = TUNE_LAG_MIN; L <= TUNE_LAG_MAX; L++) {
                const int16_t *b = a - L;
                float num = 0.0f, e2 = 0.0f;
                for (int i = 0; i < 256; i++) {
                    float fa = (float)a[i], fb = (float)b[i];
                    num += fa * fb; e2 += fb * fb;
                }
                float c = num / sqrtf(e1 * e2 + 1.0f);
                cs[L] = c;
                if (c > cmax) cmax = c;
            }
            if (cmax > TUNE_VOICED) {
                for (int L = TUNE_LAG_MIN + 1; L < TUNE_LAG_MAX; L++) {
                    if (cs[L] >= 0.85f * cmax && cs[L] >= cs[L - 1] &&
                        cs[L] >= cs[L + 1]) {
                        float den = cs[L - 1] - 2.0f * cs[L] + cs[L + 1];
                        float d = (den < -1e-9f || den > 1e-9f)
                                ? 0.5f * (cs[L - 1] - cs[L + 1]) / den : 0.0f;
                        freq = TUNE_DFS / ((float)L + d);
                        break;
                    }
                }
            }
        }
        tun_freq_dbg = freq;
    }
    if (tun_tgt <= 0.0f && tun_mix < 0.001f) {   // probe-only (tuner page):
        if (out != in) memcpy(out, in, (size_t)n * sizeof out[0]);
        return;                                  // detection done, audio dry
    }
    // 2) Snap to the nearest allowed note (with hysteresis) -> correction.
    // Runs at detection cadence so the retune feel is rate-invariant.
    // Mode split (bench telemetry 2026-07-17, two rounds):
    //   ROBOTIC pins against the RAW measurement (median-of-3 only, to kill
    //   single-frame octave blips): output = note - midi_now, so vibrato
    //   and tremolo are flattened OUT and detection grit becomes part of
    //   the hard-tune character. Correcting against a SMOOTHED measurement
    //   (first attempt) cancels only slow drift and lets the fast wobble
    //   through — the effect audibly vanishes.
    //   NATURAL corrects against a slewed measurement: only sustained
    //   off-pitch drift gets pulled in; vibrato survives. Subtle by design.
    static float tun_rate = 1.0f, tun_pout = 384.0f;
    if (detect_now) {
    if (freq > 0.0f) {
        float midi = 69.0f + 12.0f * log2f(freq * (1.0f / 440.0f));
        tun_m3[2] = tun_m3[1]; tun_m3[1] = tun_m3[0]; tun_m3[0] = midi;
        if (tun_m3n < 3) tun_m3n++;
        float med = midi;
        if (tun_m3n >= 3) {                    // median of the last 3
            float a3 = tun_m3[0], b3 = tun_m3[1], c3 = tun_m3[2];
            med = (a3 < b3) ? ((b3 < c3) ? b3 : (a3 < c3 ? c3 : a3))
                            : ((a3 < c3) ? a3 : (b3 < c3 ? c3 : b3));
        }
        if (tun_m3n == 1) tun_midi_s = med;    // first voiced block: jump
        else              tun_midi_s += 0.5f * (med - tun_midi_s);
        float m_used = tun_robot ? med : tun_midi_s;
        // ROBOTIC re-snaps to the nearest allowed note EVERY block (no
        // hysteresis): boundary notes chatter and flips whip — exaggeration
        // is the point. NATURAL holds the note until the voice clearly
        // commits elsewhere.
        if (tun_robot || tun_note < 0 || !tune_allowed(tun_note) ||
            m_used - (float)tun_note > 1.1f ||
            m_used - (float)tun_note < -1.1f) {
            int n0 = (int)floorf(m_used + 0.5f);
            float bd = 1e9f; int best = tun_note;
            for (int nn = n0 - 6; nn <= n0 + 6; nn++) {
                if (!tune_allowed(nn)) continue;
                float dd = m_used - (float)nn;
                if (dd < 0.0f) dd = -dd;
                if (dd < bd) { bd = dd; best = nn; }
            }
            tun_note = best;
        }
        float c = (float)tun_note - m_used;
        if (c >  4.0f) c =  4.0f;
        if (c < -4.0f) c = -4.0f;
        tun_corr_tgt = c;
        tun_uv = 0;
    } else if (++tun_uv >= 8) {
        tun_corr_tgt = 0.0f;                  // long unvoiced: relax to unity
        tun_m3n = 0;                          // restart pitch smoothing
    }                                         // brief gap: hold the correction
    // Track the pitch period for grain cutting (smoothed to avoid length
    // jumps; unvoiced blocks keep the last period).
    if (freq > 0.0f)
        psa_P += 0.5f * ((float)FX_FS / freq - psa_P);
    // 3) Retune speed: ROBOTIC snaps instantly, NATURAL glides (~120 ms).
    tun_corr += (tun_robot ? 1.0f : 0.12f) * (tun_corr_tgt - tun_corr);
    tun_rate = exp2f(tun_corr * (1.0f / 12.0f));
    tun_pout = psa_P / tun_rate;               // target epoch spacing
    }                                          // end detect_now
    float rate = tun_rate, pout = tun_pout;
    // 4) Synthesis + click-free dry/wet. Two engines, both kept warm so the
    //    MODE toggle is seamless:
    //      NATURAL -> TD-PSOLA: formant-true, clean steps (polished repair)
    //      ROBOTIC -> granular resampler: formants shift with pitch and the
    //                 24 ms splices snap audibly - the exaggerated hard-tune
    //                 character (user verdict: PSOLA alone is too natural)
    tun_mix += 0.25f * (tun_tgt - tun_mix);
    for (int i = 0; i < n; i++) {
        float wet_g = sh_tick(&sh_tune, rate, in[i]);
        float wet_p = psa_tick(in[i], pout);
        float wet = tun_robot ? wet_g : wet_p;
        float o = (1.0f - tun_mix) * (float)in[i] + tun_mix * wet;
        if (o >  32700.0f) o =  32700.0f;
        if (o < -32700.0f) o = -32700.0f;
        out[i] = (int16_t)o;
    }
}

// ---- Reverb: Schroeder-Moorer plate (4 damped combs -> 3 allpass) ---------
// Placed POST drums/keys in the pump, unlike every other effect here: reverb
// is the one effect a player expects to sit on the whole mix, and the kit and
// the keys are mixed in after the chain. See the call site in main.c.
//
// Comb lengths are mutually prime at FX_FS so their echo trains never line up
// into a pitched ring, and each comb's feedback runs through a one-pole
// lowpass, so successive reflections lose treble the way a real room does —
// without damping a Schroeder tank sounds like a metal bucket.
//
// int16 delay lines, not float: SRAM had ~20 KB free on this build (viz strips
// own most of the budget) and float lines would cost 21 KB. Resolution is not
// the tradeoff it looks like — the tank runs NEAR full scale because the input
// is scaled by RV_IN to compensate the combs' 1/(1-fb) DC gain, so the lines
// are well used rather than sitting 3 bits down.
void fx_verb_set(int on) { rv_tgt = on ? 1.0f : 0.0f; }

// DECAY 0..10 -> comb feedback 0.86..0.96, i.e. roughly 1.2 s to 4 s of tail.
// The output normalisation is recomputed here so a longer decay does not also
// get louder: without this, MIX would mean something different at every decay
// setting.
void fx_verb_decay(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    // 0.012 per step, not 0.010: at the old top (fb 0.96) RT60 was 4.8 s, heard
    // as ~2 s because the last 30 dB sits under the amp hiss. Bench asked for
    // longer twice. New range measured on the WET ONLY (measuring the mix is
    // wrong -- the dry transient dominates the peak and RT60 reads absurdly
    // short): DECAY 9 -> 6.2 s, DECAY 10 -> 7.5 s, and 10.3 s at max shimmer.
    // Zero tank saturation at every setting. Costs NO memory: the tank is
    // already allocated, this is one coefficient.
    rv_fb = 0.86f + 0.012f * (float)amount;
    rv_in = 0.7f * (1.0f - rv_fb);          // constant tank utilisation
    rv_norm = RV_CAL / ((float)RV_NC * 0.7f);
}

// SHIMMER 0..10 -> feedback gain 0..0.6 through the octave-up loop. Capped at
// 0.6 because simulation diverges by 0.8: the tail stops decaying and the
// output runs to several times full scale.
void fx_verb_shimmer(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    // 0.045, not 0.06: max 0.45 keeps the loop gain at 0.837, safely under the
    // 1/1.86 = 0.538 stability limit. At 0.06 the top of the knob was 1.116 and
    // self-oscillated. SHIMMER 10 is therefore about what SHIMMER 7.5 used to be,
    // but it now DECAYS instead of swelling.
    rv_sh_g = 0.045f * (float)amount;
}

// MIX 0..10 -> 0 .. 0.80 wet. Reverb is a blend, not a replacement, so even
// full travel leaves the dry signal audible.
void fx_verb_mix(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    rv_wet_set = 0.08f * (float)amount;
}

void fx_verb_process(const int16_t *in, int16_t *out, int n) {
    if (!rv_ready) rv_init_offsets();
    rv_mix += 0.25f * (rv_tgt - rv_mix);      // click-free toggle
    float wetg = rv_mix * rv_wet_set;
    for (int i = 0; i < n; i++) {
        float x = (float)in[i] * (1.0f / 32768.0f);
        // Dry and the shimmer feedback are summed BEFORE the input scaling.
        // Putting the feedback after it bypasses the attenuation and multiplies
        // the loop gain by 1/RV_IN -- simulation ran away exactly that way.
        // Dry and shimmer feedback share rv_in. This is the CORRECT structure:
        // the tank's gain goes as 1/(1-fb) and rv_in goes as (1-fb), so they
        // cancel and the shimmer loop gain is 1.86*rv_sh_g regardless of DECAY.
        float xin = (x + rv_sh_fb) * rv_in;
        float sum = 0.0f;
        for (int c = 0; c < RV_NC; c++) {
            uint16_t idx = rv_coff[c] + rv_cw[c];
            float y = (float)rv_buf[idx] * (1.0f / 32768.0f);
            sum += y;
            // one-pole lowpass inside the feedback path = room damping
            rv_lp[c] += (1.0f - rv_damp) * (y - rv_lp[c]);
            float w = xin + rv_fb * rv_lp[c];
            w = fxc_clamp(w, 0.999f, FXC_VERB_COMB);
            rv_buf[idx] = (int16_t)(w * 32767.0f);
            if (++rv_cw[c] >= RV_CLEN[c]) rv_cw[c] = 0;
        }
        sum *= rv_norm;
        for (int a = 0; a < RV_NA; a++) {
            uint16_t idx = rv_aoff[a] + rv_aw[a];
            float b = (float)rv_buf[idx] * (1.0f / 32768.0f);
            float o = -sum + b;
            float w = sum + RV_AP_G * b;
            w = fxc_clamp(w, 0.999f, FXC_VERB_AP);
            rv_buf[idx] = (int16_t)(w * 32767.0f);
            if (++rv_aw[a] >= RV_ALEN[a]) rv_aw[a] = 0;
            sum = o;
        }
        // Shimmer tap: the wet output goes back through an octave-up shifter
        // and a lowpass, and lands on the next sample's tank input. The
        // lowpass is not optional -- each pass adds an octave of energy and
        // the top end runs away without it.
        if (rv_sh_g > 0.0001f) {
            float sh = sh_tick(&rv_sh, 2.0f, (int16_t)(sum * 32767.0f))
                     * (1.0f / 32768.0f);
            rv_sh_lp += RV_SH_LP * (sh - rv_sh_lp);
            rv_sh_fb = rv_sh_g * rv_sh_lp;
            rv_sh_fb = fxc_clamp(rv_sh_fb, 0.999f, FXC_VERB_SHIM);
        } else {
            rv_sh_fb = 0.0f;
        }
        // ADDITIVE, not a crossfade. It used to be (1-wetg)*x + wetg*sum, which
        // at MIX 7 attenuated the DRY to 0.44 -- a 7.1 dB cut, measured -6.4 dB
        // overall on real playing. Reverb wet carries NO attack, so crossfading
        // strips the transient without replacing it: bench verdict was "I have no
        // attack at 7... it sounded like listening to someone playing distantly
        // down the hallway", and MIX 5 only partly helped because that is still
        // -4.3 dB. Keeping the dry at unity and ADDING wet measures +0.1 dB at
        // MIX 7 and +0.3 dB at MIX 10, with ZERO clipped samples -- diffuse
        // reverb peaks do not coincide with dry peaks, so the sum barely exceeds
        // the dry alone. Same lesson as the flanger trim: an effect that costs
        // obvious level when engaged reads as broken.
        float y = (x + wetg * sum) * 32767.0f;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

// ---- Bit crush ------------------------------------------------------------
void fx_crush_set(int on) { bc_tgt = on ? 1.0f : 0.0f; }

// AMOUNT 1..10 -> decimation 10..24x (4.9 kHz down to 2.0 kHz effective) and
// depth 12..6 bits, geometric in rate so each step is an equal musical
// interval of crush.
//
// RESCALED after a bench listen (2026-07-30). The first mapping spanned
// 1..24x / 16..5 bits, on my assumption that heavy decimation was damage to
// be rationed. Wrong instinct: with this effect the aggressive end IS the
// sound. The verdict was 10 unusable, 9 iffy, 8 the sweet spot (classic
// 8-bit chiptune character), 7 okay, and everything below that
// "meh, what's the point" -- so the whole useful range sat in three of ten
// positions and six were wasted being too polite. The old sweet spot (18x,
// 8 bits) now lands at 7, position 1 is a usable lo-fi telephone rather than
// nothing, and 10 is the ragged edge instead of a third of the travel.
static const unsigned char BC_STEP[10] = {
    10, 11, 12, 13, 15, 16, 18, 20, 22, 24
};
void fx_crush_amount(int amount) {
    if (amount < 1)  amount = 1;
    if (amount > 10) amount = 10;
    bc_step = BC_STEP[amount - 1];
    int bits = 12 - (amount - 1) * 6 / 9;         // 12..6 bits, 8 at amount 7
    bc_q = 2.0f / (float)(1 << (bits - 1));
}

void fx_crush_process(const int16_t *in, int16_t *out, int n) {
    bc_mix += 0.25f * (bc_tgt - bc_mix);
    for (int i = 0; i < n; i++) {
        float x = (float)in[i] * (1.0f / 32768.0f);
        if (--bc_cnt <= 0) {                      // sample and hold
            bc_cnt = bc_step;
            float q = x / bc_q;
            bc_hold = (q < 0.0f ? -floorf(-q + 0.5f) : floorf(q + 0.5f)) * bc_q;
            if (bc_hold >  0.999f) bc_hold =  0.999f;
            if (bc_hold < -0.999f) bc_hold = -0.999f;
        }
        float y = ((1.0f - bc_mix) * x + bc_mix * bc_hold) * 32767.0f;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

#if FX_ENABLE_RINGMOD   /* disabled 2026-08-01, see fx.h */
// ---- Ring mod -------------------------------------------------------------
void fx_ring_set(int on) { rm_tgt = on ? 1.0f : 0.0f; }

// FREQ 1..10 -> 1 Hz .. 1200 Hz, geometric. EXTENDED DOWN from 20 Hz: at a few
// hertz a ring modulator is simply a tremolo, so the bottom of the knob now
// covers the tremolo that was cut from the grid, and the top is still the
// metallic robot-voice register. 1, 2.2, 4.8, 10.6, 23, 51, 112, 247, 543,
// 1194 Hz -- so 1-3 is tremolo, 4-6 a rough throb, 7-10 ring mod proper.
void fx_ring_freq(int amount) {
    if (amount < 1)  amount = 1;
    if (amount > 10) amount = 10;
    float hz = powf(1200.0f, (float)(amount - 1) / 9.0f);
    rm_k = 6.2831853f * hz / (float)FX_FS;
}

// MIX 0..10 -> 0..1.0 wet. Fully wet is the classic pedal behaviour but it
// makes chords collapse: every note's sum-and-difference products pile up with
// no dry anchor holding the chord together (bench verdict 2026-07-30, "playing
// chords sounds like a disaster"). Blending dry back in keeps the core note
// intelligible under the dissonance, which is the standard way this is used.
void fx_ring_mix(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    rm_wet_set = 0.1f * (float)amount;
}

void fx_ring_process(const int16_t *in, int16_t *out, int n) {
    rm_mix += 0.25f * (rm_tgt - rm_mix);
    if (rm_k == 0.0f) fx_ring_freq(5);
    for (int i = 0; i < n; i++) {
        rm_s += rm_k * rm_c;
        rm_c -= rm_k * rm_s;
        float x = (float)in[i] * (1.0f / 32768.0f);
        float g = rm_mix * rm_wet_set;
        float y = ((1.0f - g) * x + g * (x * rm_s)) * 32767.0f;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
    float m2 = rm_s * rm_s + rm_c * rm_c;        // keep the carrier at unity
    float g = 1.5f - 0.5f * m2;
    rm_s *= g; rm_c *= g;
}

#endif

// ---- Flanger --------------------------------------------------------------
void fx_flange_set(int on) { fl_tgt = on ? 1.0f : 0.0f; }

// RATE 1..10 -> 0.05..10 Hz (geometric). In FILTER MATRIX mode this knob is
// re-purposed: the LFO is frozen and RATE positions the static comb instead.
void fx_flange_rate(int amount) {
    if (amount < 1)  amount = 1;
    if (amount > 10) amount = 10;
    float hz = 0.05f * powf(200.0f, (float)(amount - 1) / 9.0f);
    fl_k = hz / (float)FX_FS;                 // phase per sample, in cycles
    fl_static = FL_FMIN + (FL_FMAX - FL_FMIN) * (float)(amount - 1) / 9.0f;
}

// COLOR 0..10 -> feedback 0..0.85. Never 1.0: the original does not run away.
void fx_flange_color(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    fl_fb = 0.085f * (float)amount;
}

// MODE 0 = FLANGE (LFO sweeping), 1 = FILTER MATRIX (LFO frozen, static comb).
void fx_flange_mode(int matrix) { fl_matrix = matrix ? 1 : 0; }

// 4-point Catmull-Rom. Linear interpolation is a lowpass whose depth depends
// on the FRACTIONAL part of the delay, so on a fast short sweep it dulls the
// top end AND modulates that dulling -- half of what "robotic" was.
//
// NEVER convert the free-running write counter to float. `fl_w` counts samples
// forever and float32 holds integers exactly only to 2^24, which at FX_FS is
// 5.7 MINUTES of uptime. Past that (float)fl_w quantises: ULP reaches 1 sample
// by 5 min, 8 by 30 min, 16 by an hour. The fractional interpolation coefficient
// collapses to exactly 0.0 and the delay can only land on 16-sample steps, so
// the smooth sweep becomes a coarse staircase. Bench verdict when this bit, on a
// board that had been up for hours: "it sounds like trash... bitsmash +
// distortion". It also explains why the SAME build earned "sounds great" right
// after a flash and "a bit more robotic" an hour later -- the effect DEGRADED
// WITH UPTIME, which made every verdict irreproducible.
// Fix: integer ring arithmetic for the whole-sample part, float only for the
// fraction (which is small and exact). Measured read-position error is now 0.000
// samples at 1 min / 30 min / 1 h / 2 h, against up to 14.65 samples before.
// Every other delay reader in this file already did it this way (sh_read, the
// delay line); this was the lone exception. The deleted CHORUS shared the same
// bug, so its "sounds bad" verdict may have been this rather than its design.
static inline float fl_read(float delay) {
    uint32_t di = (uint32_t)delay;             // whole samples
    float    fr = delay - (float)di;           // fraction in [0,1), exact
    uint32_t i1 = (fl_w - di - 1u) & FL_MASK;  // == floor(fl_w - delay)
    float    t  = 1.0f - fr;                   // Catmull-Rom at t==1 returns y2
    float y0 = (float)fl_buf[(i1 - 1u) & FL_MASK];
    float y1 = (float)fl_buf[ i1 ];
    float y2 = (float)fl_buf[(i1 + 1u) & FL_MASK];
    float y3 = (float)fl_buf[(i1 + 2u) & FL_MASK];
    float a1 = 0.5f * (y2 - y0);
    float a2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    float a3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    return (((a3 * t + a2) * t + a1) * t + y1) * (1.0f / 32768.0f);
}

void fx_flange_process(const int16_t *in, int16_t *out, int n) {
    fl_mix += 0.25f * (fl_tgt - fl_mix);
    if (fl_k == 0.0f) fx_flange_rate(4);
    for (int i = 0; i < n; i++) {
        float d;
        if (fl_matrix) {
            // frozen comb, no LFO -- but glide to the target so stepping RATE
            // does not click (see FL_STAT_SLEW).
            fl_stat_cur += FL_STAT_SLEW * (fl_static - fl_stat_cur);
            d = fl_stat_cur;
        } else {
            // Asymmetric triangle: 55% of the cycle rising, 45% falling. The
            // uneven sweep is the vocal character; a sine has no edge to it.
            fl_ph += fl_k;
            if (fl_ph >= 1.0f) fl_ph -= 1.0f;
            float v = (fl_ph < FL_RISE)
                    ? (fl_ph * (1.0f / FL_RISE))
                    : (1.0f - (fl_ph - FL_RISE) * (1.0f / (1.0f - FL_RISE)));
            d = FL_MIN + (FL_MAX - FL_MIN) * v;
        }
        float echo = fl_read(d);
        float x = (float)in[i] * (1.0f / 32768.0f);
        // Feedback restricted to the midrange BEFORE it re-enters the line:
        // HP 150 Hz kills low-end boom, LP 6 kHz kills the piercing ring.
        // This is the fix for "robotic" -- v1 fed back full bandwidth forever.
        fl_fb_hp_y = FL_FB_HP * (fl_fb_hp_y + echo - fl_fb_hp_x);
        fl_fb_hp_x = echo;
        fl_fb_lp  += FL_FB_LP * (fl_fb_hp_y - fl_fb_lp);
        float wr = x + fl_fb * fl_fb_lp;
        wr = fxc_clamp(wr, 0.999f, FXC_FLANGE);
        fl_buf[fl_w & FL_MASK] = (int16_t)(wr * 32767.0f);
        fl_w++;
        // BBD bandlimit on the WET ONLY: 6th-order Butterworth at 9 kHz,
        // standing in for the SAD1024's roll-off.
        float wet = echo;
        for (int s = 0; s < 3; s++) {
            float w0 = wet - FL_BW[s][3] * fl_bw_z[s][0]
                           - FL_BW[s][4] * fl_bw_z[s][1];
            wet = FL_BW[s][0] * w0 + FL_BW[s][1] * fl_bw_z[s][0]
                                  + FL_BW[s][2] * fl_bw_z[s][1];
            fl_bw_z[s][1] = fl_bw_z[s][0];
            fl_bw_z[s][0] = w0;
        }
        // EQUAL-POWER blend. The comb-filtered wet is decorrelated from
        // the dry, so a linear 0.5/0.5 sums to sqrt(0.5^2+0.5^2) = 0.707 --
        // 3 dB down, which is the measured -2.7 dB when FLANGER is engaged
        // (bench 2026-08-03). Normalising by the power sum restores unity at
        // full mix and, critically, is a NO-OP at m = 0: the dry path through
        // this always-running stage is bit-identical to before.
        float m = fl_mix;
        float gd = 1.0f - 0.5f * m, gw = 0.5f * m;
        float y = (gd * x + gw * wet) / sqrtf(gd * gd + gw * gw);
        fl_shelf += FL_SHELF_A * (y - fl_shelf);
        y = (y - FL_SHELF_G * fl_shelf) * (FL_TRIM * FL_MAKEUP);
        y *= 32767.0f;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}

#if FX_ENABLE_VOWEL   /* disabled 2026-08-01, see fx.h */
// ---- Vowel filter ---------------------------------------------------------
// REPLACED FREEZE, 2026-07-31. Freeze's bench verdict was "terrible... I really
// don't like that effect as it currently stands", twice over. The measurements
// said it worked exactly as designed -- tonal drone, no seam buzz at the 11.9 Hz
// grain rate -- which is the point: the DESIGN was the problem. Looping 84 ms of
// a chord gives a static pad with no evolution, and a longer grain would have
// meant sharing GLITCH's 64 KB buffer, spending memory from an effect the bench
// loves on one it does not.
//
// This is two resonant formants tracking a path through five vowels, so the
// guitar audibly says "ee-eh-ah-oh-oo". Costs no delay line at all (freeing
// freeze's 8 KB for the chorus rebuild) and adds ZERO latency: biquads only, no
// buffer to read behind. Coefficients recompute once per block, not per sample.
//
// Formant pairs are the standard male-voice values. F2 sits ~7 dB under F1,
// which is what stops it sounding like two whistles instead of a voice.
#define VW_NV 5
static const float VW_F1[VW_NV] = { 270.0f, 530.0f, 730.0f, 570.0f, 300.0f };
static const float VW_F2[VW_NV] = { 2290.0f, 1840.0f, 1090.0f, 840.0f, 870.0f };
#define VW_Q1     9.0f       // formant bandwidths ~80 Hz -> Q about 9
#define VW_Q2    11.0f
#define VW_F2_G   0.45f      // F2 about -7 dB under F1
#define VW_WET    0.75f      // mostly wet: the filtering IS the effect
#define VW_MAKEUP 5.0f       // two narrow bandpasses throw most energy away

void fx_vowel_set(int on) { vw_tgt = on ? 1.0f : 0.0f; }

// RATE 1..10 -> 0.05..4 Hz sweep through the vowels (AUTO mode only).
void fx_vowel_rate(int amount) {
    if (amount < 1)  amount = 1;
    if (amount > 10) amount = 10;
    float hz = 0.05f * powf(80.0f, (float)(amount - 1) / 9.0f);
    vw_k = hz / (float)FX_FS;
}

// POSITION 0..10 -> a point on the ee-eh-ah-oh-oo path (MANUAL mode only).
void fx_vowel_pos(int amount) {
    if (amount < 0)  amount = 0;
    if (amount > 10) amount = 10;
    vw_man_pos = (float)amount * (float)(VW_NV - 1) / 10.0f;
}

// MODE 0 = AUTO (LFO sweeps the vowels), 1 = MANUAL (POSITION picks one).
void fx_vowel_mode(int manual) { vw_manual = manual ? 1 : 0; }

// RBJ bandpass, constant peak gain. Called twice per block, never per sample.
static void vw_stage(int s, float f0, float q) {
    float w = 6.2831853f * f0 / (float)FX_FS;
    float sn = sinf(w), cs = cosf(w);
    float al = sn / (2.0f * q);
    float a0 = 1.0f + al;
    vw_b0[s] =  al / a0;
    vw_b1[s] =  0.0f;
    vw_b2[s] = -al / a0;
    vw_a1[s] = -2.0f * cs / a0;
    vw_a2[s] = (1.0f - al) / a0;
}

void fx_vowel_process(const int16_t *in, int16_t *out, int n) {
    vw_mix += 0.25f * (vw_tgt - vw_mix);
    // Advance the vowel position once per block and restage coefficients. A
    // 5.24 ms granularity on a sub-4 Hz sweep is inaudible and keeps two sinf
    // calls per BLOCK instead of per sample.
    if (vw_manual) {
        vw_pos_v += 0.25f * (vw_man_pos - vw_pos_v);   // glide, no zipper
    } else {
        vw_ph += vw_k * (float)n;
        if (vw_ph >= 1.0f) vw_ph -= 1.0f;
        // Triangle across the vowel path so it runs ee->oo->ee, not a jump back.
        float t = vw_ph < 0.5f ? vw_ph * 2.0f : 2.0f - vw_ph * 2.0f;
        vw_pos_v = t * (float)(VW_NV - 1);
    }
    int   i0 = (int)vw_pos_v;
    if (i0 > VW_NV - 2) i0 = VW_NV - 2;
    float fr = vw_pos_v - (float)i0;
    float f1 = VW_F1[i0] + (VW_F1[i0 + 1] - VW_F1[i0]) * fr;
    float f2 = VW_F2[i0] + (VW_F2[i0 + 1] - VW_F2[i0]) * fr;
    vw_stage(0, f1, VW_Q1);
    vw_stage(1, f2, VW_Q2);
    for (int i = 0; i < n; i++) {
        float x = (float)in[i] * (1.0f / 32768.0f);
        float v = 0.0f;
        for (int s = 0; s < 2; s++) {
            float w0 = x - vw_a1[s] * vw_z1[s] - vw_a2[s] * vw_z2[s];
            float o  = vw_b0[s] * w0 + vw_b1[s] * vw_z1[s] + vw_b2[s] * vw_z2[s];
            vw_z2[s] = vw_z1[s];
            vw_z1[s] = w0;
            v += (s == 0) ? o : o * VW_F2_G;
        }
        v *= VW_MAKEUP;
        float wet = (1.0f - VW_WET) * x + VW_WET * v;
        float y = ((1.0f - vw_mix) * x + vw_mix * wet) * 32767.0f;
        if (y >  32700.0f) y =  32700.0f;
        if (y < -32700.0f) y = -32700.0f;
        out[i] = (int16_t)y;
    }
}
#endif

