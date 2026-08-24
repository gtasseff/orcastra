/* PSRAM *data* window for the SD-loadable app, moved above the 1 MB image
 * reservation in pico_flash_region.ld.
 *
 * The stock build places __uninitialized_psram data (the vox sample slots and
 * the agentio shadow framebuffer, 4.7 MB of it) at exactly 0x11000000 -- which
 * is where the loaded image itself now lives. Left alone, the image and the
 * audio buffers would overlap byte-for-byte. Starting the data window at
 * 0x11100000 keeps 1 MB for the image and 7 MB for data.
 */
PSRAM(rwx) : ORIGIN = 0x11100000, LENGTH = (7 * 1024 * 1024)
