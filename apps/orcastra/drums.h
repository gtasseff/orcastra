// drums.h — 16-step drum machine with synthesized voices (classic analog
// drum-box style: swept-sine kick, tone+noise snare, filtered-noise hats,
// triple-burst clap — no samples, ~zero memory).
// The UI core edits the pattern/tempo; the audio core sequences with
// sample accuracy inside drums_process(), which MIXES the kit into the
// passed buffer (post-FX-chain: drums stay clean while the live input is
// effected). Playback continues across pages — it's a backing beat.
#ifndef DRUMS_H
#define DRUMS_H
#include <stdint.h>

#define DRUM_VOICES 5           // KICK, SNARE, HAT-C, HAT-O, CLAP
#define DRUM_STEPS  16
#define DRUM_BANKS  4           // chainable 16-step banks (bars A..D)

// Grid edits target the UI-selected EDIT bank (drums_bank_set); playback
// chains banks 0..len-1 in order, one bar of 16th notes each.
void drums_toggle(int voice, int step);      // flip one grid cell (edit bank)
int  drums_get(int voice, int step);         // read one cell (edit bank)
void drums_clear(void);                      // clear the EDIT bank only
void drums_bank_set(int b);                  // select edit bank 0..3
int  drums_bank_get(void);
void drums_copy(int from, int to);           // duplicate one bank's pattern
void drums_len_set(int n);                   // chain length in banks, 1..4
int  drums_len_get(void);
void drums_play(int on);
int  drums_playing(void);
void drums_bpm_set(int bpm);                 // 60..184
int  drums_bpm_get(void);
int  drums_step_now(void);                   // GLOBAL playhead 0..len*16-1
                                             // (-1 idle); bank = step >> 4
void drums_trigger(int voice);               // manual hit (future: pads)
// Mix the kit into inout (audio core). Call every block while
// drums_playing() OR any voice is still ringing.
void drums_process(int16_t *inout, int n);
int  drums_active(void);                     // playing or tails ringing
void drums_reset(void);

#endif // DRUMS_H
