// keys.h — chord-pad instrument (KEYS page): latch a chord type, tap a
// piano key, a 4-voice polyphonic chord strikes and rings out. Two
// synthesized instruments (no samples): FM electric piano and a detuned-
// saw string ensemble. The UI core triggers; keys_process (audio core)
// renders and MIXES into the block post-FX-chain, like the drums — chords
// keep ringing across pages and can back a live guitar.
#ifndef KEYS_H
#define KEYS_H
#include <stdint.h>

enum { KEYS_MAJ = 0, KEYS_MIN, KEYS_DOM7, KEYS_MIN7,
       KEYS_MAJ7, KEYS_SUS4, KEYS_SUS2, KEYS_DIM, KEYS_NCHORDS };
enum { KEYS_EPIANO = 0, KEYS_STRINGS, KEYS_NINST };

void keys_chord(int type, int root_semi);   // strike chord (root 0..12 = C..C)
// Finger lifted: held notes enter their release (pedal setting decides
// ring vs damp). While held, notes sustain piano-like (very slow decay;
// strings hold steady).
void keys_release(void);
void keys_instrument(int inst);
int  keys_instrument_get(void);
// Sustain pedal: ON = notes ring out (~3-4 s), OFF = dampers down, quick
// fade. Toggling OFF also damps chords already ringing.
void keys_sustain(int on);
int  keys_sustain_get(void);
// Finger-wiggle vibrato (cello-style): depth 0..1 -> ~5.5 Hz pitch LFO.
void keys_vibrato(float depth);
int  keys_active(void);                     // voices still ringing
void keys_process(int16_t *inout, int n);   // audio core: mix into block
void keys_reset(void);

#endif // KEYS_H
