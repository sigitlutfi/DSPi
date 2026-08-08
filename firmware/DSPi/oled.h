/*
 * oled.h — SSD1306 128x64 I²C OLED status screen with page slideshow.
 *
 * Drives an I²C master (controller) bus on i2c0 using GPIO 4 (SDA) and
 * GPIO 5 (SCL) at 400 kHz.  This is an independent second I²C bus: it
 * does not touch the I2C1 target control interface (defaults 18/19) nor
 * the S/PDIF RX input, whose default GPIO 5 this board does not use.
 *
 * CONTENT: pages built from existing firmware state
 * (audio_state.freq, master_volume_db, user_mute, active_input_source,
 * filter_recipes / channel names, and the live DSP effect configs):
 *   page 0..n-1  status + EQ bands 1-5 / 6-10
 *   page n       status + DSP effects (crossfeed / loudness / psybass /
 *                leveller) status
 * The EQ band pages come first (one per 5 bands) and the effects page is
 * always the last page.
 *
 * `oled_tick()` re-renders the selected page only when a line actually
 * changed (compared against the last frame).  In steady state it is a pure
 * no-op — the full-frame I²C write (~21 ms at 400 kHz) happens only on real
 * content changes, never periodically, so the status screen does not block
 * audio.  Slideshow auto-rotation is currently disabled: the display stays on
 * the page selected via oled_set_page() (default page 0) to avoid a periodic
 * full-frame flush clicking the audio stream.
 *
 * THREAD MODEL: all entry points are main-thread, called from main() /
 * the main loop alongside dac_hw_mute_tick().  All state read (audio_state,
 * filter_recipes, …) is written from the same core-0 main loop / pipeline,
 * so there is no cross-core race.
 *
 * See Documentation/Features/ for the planned OLED content specs.
 */

#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdbool.h>

#define OLED_I2C_INSTANCE   i2c0
#define OLED_SDA_PIN        4u
#define OLED_SCL_PIN        5u
#define OLED_I2C_BAUD       400000u
#define OLED_I2C_ADDR       0x3Cu
#define OLED_WIDTH          128u
#define OLED_HEIGHT         64u
#define OLED_PAGES          (OLED_HEIGHT / 8u)

/* Minimum spacing between full framebuffer flushes, in ms. */
#define OLED_REFRESH_MS     200u

/* How long each slideshow page stays on screen before auto-advancing, in ms. */
#define OLED_PAGE_DURATION_MS 3000u

/* One-time initialization: configures GPIO 4/5 for I2C0, sends the
 * SSD1306 init sequence and renders the initial status/EQ screen.
 * Idempotent.  Call from main() after core0_init(). */
void oled_init(void);

/* Main-loop tick.  Re-renders the selected page when content has changed
 * (throttled to at most one rebuild per OLED_REFRESH_MS).  Cheap no-op
 * otherwise.  Call once per main-loop iteration. */
void oled_tick(void);

/* Select which channel's EQ is shown on the EQ rows.  Clamped to
 * [0, NUM_CHANNELS).  Default 0 (first input channel). */
void oled_set_eq_channel(uint8_t ch);

/* Select which page is shown.  The page index wraps/clamps to the current
 * number of pages on the next rebuild.  With slideshow rotation disabled
 * this is the way to move between EQ / effects pages. */
void oled_set_page(uint8_t page);

/* Toggle the auto status-screen renderer.  Disabled automatically while
 * oled_text()/oled_clear() own the screen; re-enable with true. */
void oled_set_auto(bool on);

/* Clear the framebuffer and switch the display to custom (non-auto) mode.
 * Re-enable the status screen with oled_set_auto(true). */
void oled_clear(void);

/* Render an ASCII string at pixel column `col`, page row `row` (0..7) in
 * custom mode.  Marks the framebuffer dirty and disables the auto screen
 * until oled_set_auto(true). */
void oled_text(uint8_t col, uint8_t row, const char *s);

#endif /* OLED_H */
