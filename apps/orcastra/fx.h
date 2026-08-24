// fx.h — orcastra audio effects, mono at the system rate below.
#ifndef FX_H
#define FX_H
#include <stdint.h>

// ACTUAL system sample rate. audio_i2s_duplex derives MCLK from an INTEGER
// PWM divide of clk_sys: 250 MHz / (256 * 20) = 48,828.125 Hz for nominal
// 48 kHz (the same mechanism made "16 kHz" run at 16,009 Hz). Every
// time-based constant derives from this so pitch math stays exact — the
// tuner would read 29 cents sharp if we pretended 48000.
#define FX_FS 48828
// Milliseconds -> samples at the actual rate (compile-time constant).
#define FX_MS(x) ((uint32_t)((x) * ((float)FX_FS / 1000.0f) + 0.5f))

// ---- Disabled effects -----------------------------------------------------
// Trimmed 2026-08-01 to bring the grid from sixteen effects to twelve. A live
// demo showed sixteen is too much to get through: "There was no way I could
// show every effect on guitar and then switch to sampling mode..." Twelve is
// also exactly 4x3, so the grid has no hole and the tiles grow from 60 px to
// 82 px. The code is KEPT, not deleted, by explicit request -- flip a macro to
// 1 to bring one back.
//
// RE-ENABLING one is TWO steps. The macro alone is not enough:
//   1. set its macro to 1 here;
//   2. in main.c give it a grid cell -- add its index to MENU_GRID, bump
//      MENU_CELLS, and set MENU_ROWS so MENU_CELLS == MENU_COLS * MENU_ROWS.
// Mind the code budget. The shipping target (orcastra_psram) EXECUTES from
// PSRAM, so effect code costs PSRAM rather than SRAM and there is room --
// but the copy_to_ram targets still pay for it in SRAM, where .data ends a
// few hundred bytes below a 32 KB alignment boundary. Crossing that boundary
// moves .bss up 32,768 bytes and overflows RAM instantly, which presents as
// a link error naming a region overflow rather than anything about your
// effect. Check with tools/check_psram_xip.py, which reports SRAM headroom.
//
// Why each one went (bench verdicts, not guesses):
//   TREMOLO - never bad, just the least distinctive. It had already been cut
//             once on 2026-07-30 to free a slot for RINGMOD, briefly returned
//             on 2026-08-01 when CHORUS was cut, and went again in the trim.
//   PHASER  - sounds fine, but redundant with FLANGER, which after its
//             analog-BBD rebuild is the better of the two. Asked to
//             choose between them the bench chose FLANGER.
//   RINGMOD - the shakiest history: "playing chords sounds like a disaster,
//             and not a fun one". A MIX knob was added to keep chords alive,
//             but it still overlaps BITCRUSH's lo-fi territory and BITCRUSH is
//             the one with a found sweet spot (8).
//   VOWEL   - two resonant formants sweeping ee-eh-ah-oh-oo. Written to replace
//             FREEZE and then never heard on hardware, so it is UNPROVEN rather
//             than rejected. The most likely of the four to be worth reviving:
//             it costs no buffer, adds zero latency, and is the natural home
//             for tilt/accelerometer control.
#define FX_ENABLE_TREMOLO 0
#define FX_ENABLE_PHASER  0
#define FX_ENABLE_RINGMOD 0
#define FX_ENABLE_VOWEL   0

// Internal-saturation report, snapshot-and-clear. `mask` has a bit per feedback
// stage that hit its own limit (delay line, reverb combs, reverb allpasses, the
// shimmer loop, the flanger line); `count` is total clamped samples. These are
// invisible to an output-level clamp counter, which sits downstream of them.
void fx_clip_report(uint32_t *mask, uint32_t *count);

// Reset all effect state (clears delay lines; call when entering the FX page).
void fx_reset(void);

// Set parameter targets (slewed internally to avoid zipper noise/clicks):
//   pitch_oct: -1..+1 octaves (0 = no shift)
//   fx_amt:     0..1 echo amount (0 = dry; scales wet mix + feedback)
void fx_set_params(float pitch_oct, float fx_amt);

// Process n mono samples in -> out. Safe to call every capture block.
void fx_process(const int16_t *in, int16_t *out, int n);

// ---- Octave page: pitch-shifted voice blended under the dry ---------------
// Enable/disable the blend (mix is slewed; no clicks either way).
void fx_octave_set(int on);
// MODE 0..4: +1 octave (default), +2 octaves, -1 octave, -2 octaves, DETUNE.
// A mode change glides over ~64 ms rather than stepping.
void fx_octave_mode(int mode);
// MIX 0..100 % of the shifted voice. Dry is (100 - mix); 50 is the default and
// matches what this page did when it was octave-up-only.
void fx_octave_mix(int pct);
// DETUNE 0..100 -> -100..+100 cents, i.e. down a half-step to up a half-step,
// with 50 exactly in tune. Read only in MODE 4; at 50 the shifter's taps freeze
// and the wet voice is a clean copy of the dry.
void fx_octave_detune(int pct);
// Process n mono samples for the octave page (bit-transparent when off).
void fx_octave_process(const int16_t *in, int16_t *out, int n);

// ---- Glitch page: random sample-and-hold takeover looper ----
// Arm/disarm the glitch machine. Armed = dry passthrough until the random
// trigger fires, then the dry is cut and a 0.3-2 s burst of stutter
// PATTERNS plays at 100% level: every 0.1-0.5 s a new pattern rolls its
// own loop length (5-30 ms micro-drone / 30-90 ms stutter / 90-200 ms
// chop), its own tape speed (1x favored, 0.5x..2x jumps, stable within
// the pattern), and its own source window (freshest audio or anywhere in
// the last second — broken-CD skipping). Seams blended ~1 ms (no
// click-train); ends on a whole repeat.
void fx_glitch_set(int on);
// FREQ 1..10: burst frequency — exponential random-gap range from ~1.8-15 s
// (1, rare) through 0.45-3.8 s (6, default) to 0.15-1.25 s (10, barrage).
void fx_glitch_freq(int f);
// 1 while a fragment is looping (for UI/LED indication), else 0.
int  fx_glitch_active(void);
// Process n mono samples for the glitch page.
void fx_glitch_process(const int16_t *in, int16_t *out, int n);

// ---- Delay: slapback through long repeats, one 335 ms line ----------------
// FEEDBACK at 0 is exactly the old single-repeat slapback. Dry always passes
// at full level, so the echo is an add and never a replacement.
void fx_delay_set(int on);
void fx_delay_time_ms(int ms);        // 40..330 ms, glides like tape varispeed
void fx_delay_fb(int amount);         // 0..10 -> 0..0.85 feedback
void fx_delay_mix(int amount);        // 0..10 -> echo level 0..1.0
void fx_delay_process(const int16_t *in, int16_t *out, int n);

// ---- Metal page: scooped-mids high-gain distortion (thrash-style) ----
// Enable/disable (crossfade; click-free toggle).
void fx_metal_set(int on);
// Drive amount 5..60 (input gain into the clipper).
void fx_metal_drive(int amount);
// Process n mono samples for the metal page.
void fx_metal_process(const int16_t *in, int16_t *out, int n);

// ---- Overdrive page: classic asymmetric-clip pedal, mid-hump voicing ----
// Enable/disable (crossfade; click-free toggle).
void fx_od_set(int on);
// DRIVE 1..10 (clipper input gain 2.5..25; warm at low, crunchy at high).
void fx_od_drive(int amount);
// Process n mono samples for the overdrive page.
void fx_od_process(const int16_t *in, int16_t *out, int n);

#if FX_ENABLE_TREMOLO   /* disabled 2026-08-01, see fx.h */
// ---- Tremolo page: smooth amplitude LFO, fixed 70% depth ----
// Enable/disable (crossfade; click-free toggle).
void fx_trem_set(int on);
// RATE 1..10 = LFO speed in Hz.
void fx_trem_rate(int r);
// Process n mono samples for the tremolo page.
void fx_trem_process(const int16_t *in, int16_t *out, int n);

#endif

#if FX_ENABLE_PHASER   /* disabled 2026-08-01, see fx.h */
// ---- Phaser page: 4-stage swept allpass, 50/50 mix, light feedback ----
// Enable/disable (crossfade; click-free toggle).
void fx_phase_set(int on);
// RATE 1..10 -> sweep speed 0.1..3.25 Hz.
void fx_phase_rate(int r);
// Process n mono samples for the phaser page.
void fx_phase_process(const int16_t *in, int16_t *out, int n);

#endif

// ---- Parametric EQ: 5 RBJ peaking bands, post-drive pre-modulation ----
// Enable/disable (crossfaded). Band i: freq 40..12000 Hz, gain +/-12 dB,
// Q 0.4..8. fx_eq_band computes + stages coefficients from the UI core;
// the audio core adopts the staged set at the next block.
void fx_eq_set(int on);
void fx_eq_band(int i, float f0, float gain_db, float q);
void fx_eq_process(const int16_t *in, int16_t *out, int n);

// ---- Orcatune page: real-time vocal pitch correction ----
// Enable/disable (crossfade; click-free toggle). Detects the input's pitch
// (80-500 Hz), snaps it to the nearest allowed note of the key/scale, and
// corrects via varispeed shifting.
void fx_tune_set(int on);
// Key root 0..11 = C, C#, D, ... B.
void fx_tune_key(int root);
// Scale 0 = major, 1 = natural minor, 2 = major pentatonic, 3 = penta+4th,
// 4 = penta+7th, 5 = major 6th chord, 6 = triad, 7 = root+fifth (sparsest -
// biggest corrections), 8 = chromatic.
void fx_tune_scale(int s);
// 0 = NATURAL (gentle ~120 ms glide), 1 = ROBOTIC (instant hard-tune snap).
void fx_tune_mode(int robotic);
// Telemetry: last detected pitch (Hz x10, 0 = unvoiced) + applied correction
// (cents). For RTT diagnostics and the tuner page.
void fx_tune_status(int *hz10, int *cents);
// Probe mode (tuner page): run detection every block with the audio passed
// through dry (pitch correction keeps working if it is also enabled).
void fx_tune_probe(int on);
// Process n mono samples for the pitch-correction page.
void fx_tune_process(const int16_t *in, int16_t *out, int n);

// ---- Paranoid page: envelope-swept resonant filter over moderate crunch ----
// Enable/disable (crossfade; click-free toggle). Gated input: idle = silence.
void fx_para_set(int on);
// RANGE 0..10: sweep ceiling 600..3200 Hz (log taper) — ride it like a foot
// knob while playing; pick attacks open the filter, decays close it.
void fx_para_range(int r);
// Process n mono samples for the paranoid page.
void fx_para_process(const int16_t *in, int16_t *out, int n);

// ---- Reverb: Schroeder-Moorer hall, 8 combs + 4 allpasses, 1.2-7.5 s ------
// Bench note: a plucked note reads as ~2 s by ear through the 3.5 mm jack even
// at DECAY 9, because that path loses ~19 dB midband and buries the quiet end
// of the tail. The tail IS there — measure through an amp, not headphones.
// UNLIKE every other effect here, this one is called POST drums/keys in the
// pump, because those two are mixed in after the FX chain and reverb is the
// one effect a player expects across the whole mix.
void fx_verb_set(int on);
// MIX 0..10 -> 0..0.80 wet, ADDED on top of a full-level dry (not a crossfade;
// see fx.c -- crossfading cut the dry 7 dB and stole the pick attack).
void fx_verb_mix(int amount);
// DECAY 0..10 -> ~1.2 s to 7.5 s RT60 (extended 2026-08-01; the old 4 s top
// was heard as only ~2 s because the last 30 dB sits under the amp hiss).
// Costs no memory -- one coefficient. Output level is renormalised so a
// longer decay is not also louder.
void fx_verb_decay(int amount);
// SHIMMER 0..10 -> an octave-up shifter inside the feedback loop, so each
// pass climbs an octave. Capped at 0.6 gain: simulation diverges by 0.8.
void fx_verb_shimmer(int amount);
// Process n mono samples (bit-transparent when off).
void fx_verb_process(const int16_t *in, int16_t *out, int n);

// ---- CHORUS CUT 2026-08-01 (bench: "chorus does sound bad") ---------------
// TREMOLO reclaimed its grid slot (fx_trem_* below). The measured causes and the
// user's BBD chorus blueprint for any future attempt are recorded in fx.c where
// the storage used to be.

// ---- Bit crush: decimate + quantise, no anti-alias filter anywhere --------
// The fold-back is the effect. AMOUNT sweeps both axes from one slider.
void fx_crush_set(int on);
void fx_crush_amount(int amount);     // 1..10 -> 1..24x decimate, 16..5 bits
void fx_crush_process(const int16_t *in, int16_t *out, int n);

#if FX_ENABLE_RINGMOD   /* disabled 2026-08-01, see fx.h */
// ---- Ring mod: multiply by a carrier, fully wet when engaged --------------
void fx_ring_set(int on);
void fx_ring_freq(int amount);        // 1..10 -> 1..1200 Hz (1-3 = tremolo)
void fx_ring_mix(int amount);         // 0..10 -> 0..1.0 wet (dry anchors chords)
void fx_ring_process(const int16_t *in, int16_t *out, int n);

#endif

// ---- Flanger: classic analog BBD flanger model ----------------------------
// Rebuilt 2026-07-31 after a "horribly robotic" bench verdict. The delay window
// (0.3-4.0 ms) was never the problem; the missing pieces were feedback band-
// limiting (HP 150 Hz + LP 6 kHz IN the loop), 4-point Catmull-Rom
// interpolation, an asymmetric 55/45 triangle LFO, and a 6th-order Butterworth
// at 9 kHz on the wet only. Output carries the vintage -1.5 dB drop and a ~1 dB
// shelf below 100 Hz.
void fx_flange_set(int on);
// RATE 1..10 -> 0.05..10 Hz. In FILTER MATRIX mode it positions the static
// comb (0.5..2.0 ms) instead, since the LFO is frozen.
void fx_flange_rate(int amount);
// COLOR 0..10 -> feedback 0..0.85 (capped: the original rarely self-oscillates).
void fx_flange_color(int amount);
// MODE 0 = FLANGE, 1 = FILTER MATRIX (LFO frozen into a fixed comb filter).
void fx_flange_mode(int matrix);
void fx_flange_process(const int16_t *in, int16_t *out, int n);

#if FX_ENABLE_VOWEL   /* disabled 2026-08-01, see fx.h */
// ---- Vowel filter: two resonant formants, ee-eh-ah-oh-oo -----------------
// Replaced FREEZE 2026-07-31 (bench: "terrible", and the design was the reason,
// not the implementation). Two RBJ bandpasses track a path through five vowel
// formant pairs so the guitar talks. No delay line, so ZERO added latency, and
// it handed FREEZE's 8 KB grain buffer to the chorus.
void fx_vowel_set(int on);
void fx_vowel_rate(int amount);       // 1..10 -> 0.05..4 Hz sweep (AUTO only)
void fx_vowel_pos(int amount);        // 0..10 -> a point on the vowel path
void fx_vowel_mode(int manual);       // 0 = AUTO sweep, 1 = MANUAL position
void fx_vowel_process(const int16_t *in, int16_t *out, int n);

#endif

#endif
