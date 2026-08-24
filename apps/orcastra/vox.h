// vox.h — VOX sampler: 5 PSRAM sample slots, hold-to-record, turntable playback.
// Slots map to the board's five coloured buttons (gray/yellow/green/blue/red),
// driven for real through the BSP's uartkbd coprocessor link — tap to select,
// hold to record. On-screen slot buttons remain as an equivalent touch path.
#ifndef VOX_H
#define VOX_H
#include <stdint.h>
#include <stdbool.h>

#define VOX_SLOTS 5

// Bring up PSRAM and lay out the slots. Returns false if PSRAM is absent
// (recording/playback then no-op and vox_ok() reports it).
bool vox_init(void);
bool vox_ok(void);

// Slot selection (tap a colored button).
void vox_select(int slot);
int  vox_selected(void);
// Recorded length of a slot in tenths of a second (0 = empty).
unsigned vox_len_ds(int slot);

// Hold-to-record: start overwrites the slot from its beginning; stop commits
// the recorded length. Recording silences playback.
void vox_rec_start(int slot);
void vox_rec_stop(void);
bool vox_recording(void);
unsigned vox_rec_ds(void);          // current recording length, tenths of s

// Turntable playback (touch pad): press drops the needle and starts playing
// at rate r — a press right of center starts at the BEGINNING playing
// forward, a press left of center starts at the END playing in reverse
// (slow near center, 1x at the far edge). While held the sample loops like
// a locked groove (wraps in both directions, seam declicked). Release lifts
// the needle; the rate is slewed inside (vinyl-style inertia).
void vox_play_press(float r);
void vox_play_release(void);
void vox_play_rate(float r);        // -1 (1x reverse) .. 0 (hold) .. +1 (1x fwd)
bool vox_playing(void);
float vox_pos_frac(void);           // playhead 0..1 of the selected slot

// Hold latch: when on, playback keeps looping at the last rate after the
// finger lifts — and off the sample page — so effects can be dialed in from
// the FX menu against the running loop. A bare HOLD (no prior scratch) loops
// forward at 1x from the start. Off stops playback.
void vox_hold_set(int on);
bool vox_held(void);

// Read access for the UI (waveform drawing): slot base + length in samples.
// While `slot` is recording, `len` is the live sample count so far (the UI
// may read behind the write head; the audio core only appends).
const int16_t *vox_data(int slot, unsigned *len);
unsigned vox_capacity(void);          // slot capacity in samples

// Seed a slot from code at boot. Slots are in PSRAM, which is VOLATILE, so a
// "factory" sample cannot be stored -- it must be regenerated every boot.
// Returns a writable base and the capacity in samples (0/NULL without PSRAM);
// fill it, then commit the number of samples written.
int16_t *vox_seed_begin(int slot, unsigned *cap);
void     vox_seed_commit(int slot, unsigned len);

// Per capture block: if recording, append `in`; produce playback (or silence)
// into `out`. In-place safe (in == out).
void vox_process(const int16_t *in, int16_t *out, int n);

#endif // VOX_H
