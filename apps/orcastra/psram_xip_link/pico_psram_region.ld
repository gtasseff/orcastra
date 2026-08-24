/* PSRAM *data* window, moved above the 1 MB image reservation in
 * pico_flash_region.ld. Identical reasoning to ../psram_link's copy: the stock
 * build places __uninitialized_psram data (the vox sample slots, 4.7 MB) at
 * exactly 0x11000000, which is where the loaded image lives, so the image and
 * the audio buffers would overlap byte-for-byte. 1 MB for the image, 7 MB for
 * data.
 *
 * This matters MORE under the PSRAM-execution scheme than under copy_to_ram:
 * here the overlap would corrupt the code currently being fetched, not merely
 * a stale copy of it.
 */
PSRAM(rwx) : ORIGIN = 0x11100000, LENGTH = (7 * 1024 * 1024)
