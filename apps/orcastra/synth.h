// synth.h — touch-playable instrument voice (mono, FX_FS).
// One oscillator (PolyBLEP saw/square or sine) through a resonant SVF
// lowpass. Touch pad: X = pitch (2.5-octave continuous glide), Y = filter
// cutoff. Sound gates on touch with click-free attack/release ramps.
// The UI core writes targets (volatile); synth_process (audio core) slews
// and renders, REPLACING the live input so the whole FX chain applies.
#ifndef SYNTH_H
#define SYNTH_H
#include <stdint.h>

enum { SYNTH_SAW = 0, SYNTH_SINE = 1, SYNTH_SQUARE = 2 };

void synth_gate(int on);              // finger down / up
void synth_xy(float x, float y);      // pad position, 0..1 each
// Motion control (UI core, ~50 Hz from the IMU):
//   shake  0..1  = shake energy -> chaotic vibrato "rattle" (fades w/ env)
//   roll  -1..1  = bank left/right -> tremolo rate+depth
//   tilt  -1..1  = tip toward(+)/away(-) -> filter resonance up/down
//   dive   -1..1 = twist rate -> pitch bend +/-3 semitones
void synth_motion(float shake, float roll, float tilt, float dive);
void synth_wave(int w);               // SYNTH_SAW / SINE / SQUARE
int  synth_wave_get(void);
// Master level 0..1 (theremin volume axis). Slewed ~10 ms on the audio
// core; independent of the gate envelope. synth_reset() restores 1.0.
void synth_level(float v);
// Direct pitch in Hz (theremin: wider range than the pad's 2 octaves),
// clamped 55..3520. Leaves the filter cutoff wherever synth_xy put it.
void synth_freq(float hz);
// Pitch glide coefficient (per-sample slew toward the target). The pad
// default 0.002 (~10 ms) tracks a finger; the theremin sets ~0.0006 so
// coarse sensor updates melt into a portamento. synth_reset() restores
// the default.
void synth_glide(float k);
// True while sounding (gate on OR release tail still audible).
int  synth_active(void);
// Render n mono samples (audio core). Overwrites out.
void synth_process(int16_t *out, int n);
void synth_reset(void);

#endif // SYNTH_H
