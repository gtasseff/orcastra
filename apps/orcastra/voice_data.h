// voice_data.h - embedded robotic singing voice (GENERATED companion).
//
// WHY EMBEDDED rather than synthesised on the device:
//   Porting the formant synthesiser to C would cost roughly 2-3 KB of code. That
//   was decisive when the app still linked COPY-TO-RAM, where code consumes
//   SRAM and only ~3 KB remained before a 32 KB alignment cliff. The shipping
//   target now EXECUTES FROM PSRAM, so that particular pressure is gone -- but
//   the decision stands on its own: an offline render is deterministic, costs no
//   audio-core cycles, and the .c file it produces is reviewable. Embedding it
//   costs a few hundred bytes of loader instead, and guarantees the board plays
//   back byte-for-byte what was approved at the bench.
//
// WHY __in_flash():
//   The SDK's copy-to-RAM linker script pulls anything in .rodata that is NOT
//   marked .flashdata into .data (RAM). A plain const array would try to land in
//   SRAM and fail the link. __in_flash("voice") puts it in .flashdata.voice,
//   which the script maps to flash; the loader reads it over XIP.
//
// Stored 16 kHz / 8-bit signed; the loader upsamples into the PSRAM slot.
#ifndef VOICE_DATA_H
#define VOICE_DATA_H
extern const signed char voice_data[];
extern const unsigned    voice_data_len;
extern const unsigned    voice_data_rate;
#endif
