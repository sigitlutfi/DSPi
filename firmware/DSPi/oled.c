/*
 * oled.c — SSD1306 128x64 I²C OLED status screen with page slideshow.
 *
 * Thin adaptation of the daschr/pico-ssd1306 driver (MIT, David Schramm)
 * to this firmware's main-loop conventions.  The library owns the driver,
 * the builtin 8x5 font, framing and addressing; this file supplies content.
 *
 * CONTENT — auto-rotating pages (each a full 8-row frame):
 *   page 0  status + EQ bands 1-5
 *   page 1  status + EQ bands 6-10
 *   (more pages can be added to the page table; see oled_pages[])
 *
 *   row 0  "DSPi <source>"          USB / SPDIF / I2S / ADAT
 *   row 1  "<rate>  <vol dB|MUTE>"  44.1k / 48.0k / 96.0k, master volume
 *   row 2  "EQ <channel> (N) p/q"   channel name + active band count + page
 *   row 3.. "<band> <type> <f> <g>"  up to 5 non-flat bands per page
 * All data comes from existing extern state (audio_state, active_input_source,
 * master_volume_db, user_mute, filter_recipes, get_default_channel_name) — the
 * DSP pipeline is not modified.  Everything is main-thread, so there is no
 * race with the audio path (which also runs on core 0).
 *
 * WRITE MODEL: oled_tick() advances the page index on a non-blocking timer
 * (absolute_time_t timeout — no sleep in the loop) and rebuilds the frame.
 * The frame is pushed to the display only when a line actually changed
 * (compared against the last rendered frame), so the ~21 ms full-frame I²C
 * write happens at most once per page rotation in steady state, plus when a
 * user event (source/rate/volume/EQ edit) changes a line.  The 200 ms throttle
 * additionally bounds how often the frame is even rebuilt.
 *
 * oled_text()/oled_clear() remain as generic framebuffer primitives.  Calling
 * them switches the display to custom mode; the auto slideshow resumes via
 * oled_set_auto(true).
 *
 * See Documentation/Features/ for the planned OLED content specs.
 */

#include <stdio.h>
#include <string.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "oled.h"
#include "ssd1306.h"

#include "config.h"
#include "audio_input.h"
#include "usb_audio.h"
#include "dsp_pipeline.h"

// Bands shown per EQ page (rows 3..7 = 5 rows).
#define OLED_PAGE_BANDS     5u

static ssd1306_t oled_disp;
static bool oled_auto = true;       // auto slideshow on
static char oled_frame[OLED_PAGES][OLED_WIDTH / 6 + 1];  // last rendered lines
static uint8_t oled_eq_channel = 0;
static uint8_t oled_page_idx = 0;   // current slideshow page
static absolute_time_t oled_next_page;
static absolute_time_t oled_next_flush;

static const char *oled_source_name(void) {
    switch (active_input_source) {
        case INPUT_SOURCE_USB:    return "USB";
        case INPUT_SOURCE_SPDIF:  return "SPDIF";
        case INPUT_SOURCE_I2S:    return "I2S";
        case INPUT_SOURCE_ADAT:   return "ADAT";
        default:                  return "SPDIF";
    }
}

static void oled_format_rate(uint32_t hz, char *buf, size_t n) {
    if (hz >= 100000u)      snprintf(buf, n, "%dk", (int)(hz / 1000u));
    else if (hz >= 10000u)  snprintf(buf, n, "%.1fk", hz / 1000.0f);
    else if (hz >= 1000u)   snprintf(buf, n, "%.1fk", hz / 1000.0f);
    else                    snprintf(buf, n, "%u", (unsigned)hz);
}

static const char *oled_filter_type_label(int t) {
    switch (t) {
        case FILTER_PEAKING:         return "PK";
        case FILTER_LOWSHELF:        return "LS";
        case FILTER_HIGHSHELF:       return "HS";
        case FILTER_LOWPASS:         return "LP";
        case FILTER_HIGHPASS:        return "HP";
        case FILTER_NOTCH:           return "NT";
        case FILTER_ALLPASS:         return "AP";
        case FILTER_ALLPASS1:        return "A1";
        case FILTER_LOWSHELF1:       return "L1";
        case FILTER_HIGHSHELF1:      return "H1";
        case FILTER_LINKWITZ_TRANSFORM: return "LT";
        case FILTER_LOWPASS1:        return "LP1";
        case FILTER_HIGHPASS1:       return "HP1";
        default:                     return "??";
    }
}

static void oled_format_freq(float f, char *buf, size_t n) {
    if (f >= 1000.0f) snprintf(buf, n, "%.1fk", f / 1000.0f);
    else              snprintf(buf, n, "%g", f);
}

// Count non-flat EQ bands on the selected channel.
static int oled_count_active_bands(void) {
    int n = 0;
    for (int b = 0; b < MAX_BANDS; b++) {
        if (filter_recipes[oled_eq_channel][b].type != FILTER_FLAT)
            n++;
    }
    return n;
}

// Build the 8 display lines for the current page into oled_frame[].
static void oled_build_frame(void) {
    char buf[OLED_WIDTH / 6 + 1];
    int n_active = oled_count_active_bands();

    // Total pages for the EQ content; clamp the current page index.
    int n_pages = (n_active + (int)OLED_PAGE_BANDS - 1) / (int)OLED_PAGE_BANDS;
    if (n_pages < 1) n_pages = 1;
    if (oled_page_idx >= n_pages) oled_page_idx = 0;

    // Rows 0..1: title + source, rate + volume / mute (shared by all pages).
    snprintf(oled_frame[0], sizeof(oled_frame[0]), "DSPi %s", oled_source_name());
    if (user_mute || audio_state.mute) {
        snprintf(oled_frame[1], sizeof(oled_frame[1]), "MUTE");
    } else {
        oled_format_rate(audio_state.freq, buf, sizeof(buf));
        snprintf(oled_frame[1], sizeof(oled_frame[1]), "%s %+.1fdB",
                 buf, (double)master_volume_db);
    }

    // Row 2: EQ header with channel name, active band count and page position.
    char ch_name[PRESET_NAME_LEN];
    get_default_channel_name(oled_eq_channel, active_input_source, NULL, ch_name);
    snprintf(oled_frame[2], sizeof(oled_frame[2]), "EQ %s (%d) %d/%d",
             ch_name, n_active, oled_page_idx + 1, n_pages);

    // Rows 3..7: this page's slice of the non-flat bands.
    int r = 3;
    int first = oled_page_idx * OLED_PAGE_BANDS;
    int last  = first + OLED_PAGE_BANDS;
    for (int b = 0; b < MAX_BANDS && r < OLED_PAGES; b++) {
        if (b < first || b >= last) continue;
        const EqParamPacket *p = &filter_recipes[oled_eq_channel][b];
        if (p->type == FILTER_FLAT) continue;
        oled_format_freq(p->freq, buf, sizeof(buf));
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "%2d %s %5s %+.1f",
                 b + 1, oled_filter_type_label(p->type), buf, (double)p->gain_db);
        r++;
    }
    for (; r < OLED_PAGES; r++)
        oled_frame[r][0] = '\0';
}

// Rebuild + push the frame to the display if any line differs from the last
// rendered one.  Returns true when a flush was issued.
static bool oled_render_auto(void) {
    char prev[OLED_PAGES][OLED_WIDTH / 6 + 1];
    memcpy(prev, oled_frame, sizeof(prev));
    oled_build_frame();

    bool changed = memcmp(prev, oled_frame, sizeof(prev)) != 0;
    if (!changed) return false;

    ssd1306_clear(&oled_disp);
    for (int i = 0; i < OLED_PAGES; i++) {
        if (oled_frame[i][0] != '\0')
            ssd1306_draw_string(&oled_disp, 0, i * 8, 1, oled_frame[i]);
    }
    ssd1306_show(&oled_disp);
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void oled_init(void) {
    i2c_init(OLED_I2C_INSTANCE, OLED_I2C_BAUD);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    memset(&oled_disp, 0, sizeof(oled_disp));
    if (!ssd1306_init(&oled_disp, OLED_WIDTH, OLED_HEIGHT, OLED_I2C_ADDR,
                      OLED_I2C_INSTANCE)) {
        oled_auto = false;
        return;
    }

    oled_auto = true;
    oled_eq_channel = 0;
    oled_page_idx = 0;
    oled_next_page = make_timeout_time_ms(OLED_PAGE_DURATION_MS);
    oled_next_flush = make_timeout_time_ms(OLED_REFRESH_MS);
    oled_tick();
}

void oled_tick(void) {
    if (!oled_auto) return;

    // Advance the slideshow on a non-blocking timeout.
    if (time_reached(oled_next_page)) {
        oled_next_page = make_timeout_time_ms(OLED_PAGE_DURATION_MS);
        oled_page_idx++;
    }

    // Rebuild (and maybe flush) at most once per OLED_REFRESH_MS.
    if (!time_reached(oled_next_flush)) return;
    oled_next_flush = make_timeout_time_ms(OLED_REFRESH_MS);
    oled_render_auto();
}

void oled_set_eq_channel(uint8_t ch) {
    oled_eq_channel = (ch < NUM_CHANNELS) ? ch : 0;
}

void oled_set_page(uint8_t page) {
    oled_page_idx = page;
}

void oled_set_auto(bool on) {
    oled_auto = on;
    if (on) {
        oled_next_page = make_timeout_time_ms(OLED_PAGE_DURATION_MS);
        oled_next_flush = make_timeout_time_ms(OLED_REFRESH_MS);
    }
}

void oled_clear(void) {
    ssd1306_clear(&oled_disp);
}

void oled_text(uint8_t col, uint8_t row, const char *s) {
    if (row >= OLED_PAGES) return;
    oled_auto = false;          // custom mode until oled_set_auto(true)
    ssd1306_draw_string(&oled_disp, col, row * 8, 1, s);
}
