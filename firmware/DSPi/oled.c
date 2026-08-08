/*
 * oled.c — SSD1306 128x64 I²C OLED status screen with page slideshow.
 *
 * Thin adaptation of the daschr/pico-ssd1306 driver (MIT, David Schramm)
 * to this firmware's main-loop conventions.  The library owns the driver,
 * the builtin 8x5 font, framing and addressing; this file supplies content.
 *
 * CONTENT — pages (each a full 8-row frame):
 *   page 0..n-1  status + EQ bands (5 per page, slices of bands 1-10)
 *   page n       status + DSP effects status (crossfeed / loudness /
 *                psybass / leveller)
 * The EQ band pages come first (count depends on active bands) and the
 * effects page is always the last page, so the page index is
 * decoupled from the EQ band slice index (see oled_build_frame()).
 *
 *   row 0  "DSPi <source>"          USB / SPDIF / I2S / ADAT (inverted bar)
 *   row 1  "<rate>  <vol dB|MUTE>"  44.1k / 48.0k / 96.0k, master volume
 *   row 2  "EQ <channel> (N) p/q"   channel name + active band count + page
 *   row 3.. "<band> <type> <f> <g>"  up to 5 non-flat bands per page
 * All data comes from existing extern state (audio_state, active_input_source,
 * master_volume_db, user_mute, filter_recipes, get_default_channel_name) — the
 * DSP pipeline is not modified.  Everything is main-thread, so there is no
 * race with the audio path (which also runs on core 0).
 *
 * WRITE MODEL: slideshow auto-rotation is disabled (see oled_tick()); the
 * display shows the page selected by oled_set_page() and updates it in place.
 * The frame is pushed to the display only when a line actually changed
 * (compared against the last rendered frame), so the ~21 ms full-frame I²C
 * write happens only on real content changes (boot, source/rate/volume/EQ
 * edit) — never periodically — so the slideshow can't stall the audio main
 * loop and click the stream.  The 200 ms throttle additionally bounds how
 * often the frame is even rebuilt.
 *
 * oled_text()/oled_clear() remain as generic framebuffer primitives.  Calling
 * them switches the display to custom mode; the auto status screen resumes via
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

// font_8x5 is defined in font.h (included by ssd1306.c); reference it here
// via extern so both translation units share one definition.
extern const uint8_t font_8x5[];

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

// Number of EQ band pages needed for the selected channel.  Always >= 1 so
// the "all flat" state is still shown.
static int oled_eq_page_count(void) {
    int p = (oled_count_active_bands() + (int)OLED_PAGE_BANDS - 1)
            / (int)OLED_PAGE_BANDS;
    return p < 1 ? 1 : p;
}

// Total slideshow pages: EQ band pages followed by the effects status page.
static int oled_page_count(void) {
    return oled_eq_page_count() + 1;
}

// Rows 2..7 for the effects status page.  One line per DSP effect: name,
// ON/OFF and a short parameter summary so a glance shows the live state.
// `position`/`total` are the slideshow page position shown in the header.
static void oled_build_fx_rows(int position, int total) {
    int r = 3;

    snprintf(oled_frame[2], sizeof(oled_frame[2]), "FX EFFECTS %d/%d",
             position, total);

    // Crossfeed: preset name + effective crossover frequency.
    if (crossfeed_config.enabled) {
        const char *preset = "DFLT";
        float fc = 700.0f;
        switch (crossfeed_config.preset) {
            case CROSSFEED_PRESET_CHUMOY: preset = "CHUM"; fc = 700.0f; break;
            case CROSSFEED_PRESET_MEIER:  preset = "MEIR"; fc = 650.0f; break;
            case CROSSFEED_PRESET_CUSTOM: preset = "CUST"; fc = crossfeed_config.custom_fc; break;
            default: break;
        }
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "CF ON  %-4s %g",
                 preset, (double)fc);
    } else {
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "CF OFF");
    }
    r++;

    // Loudness: reference SPL + intensity.
    if (loudness_enabled) {
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "LD ON  %.0fdB %d%%",
                 (double)loudness_ref_spl, (int)loudness_intensity_pct);
    } else {
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "LD OFF");
    }
    r++;

    // Psybass: cutoff + drive.
    if (psybass_config.enabled) {
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "PB ON  %.0fHz %+.0fdB",
                 (double)psybass_config.cutoff_hz, (double)psybass_config.drive_db);
    } else {
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "PB OFF");
    }
    r++;

    // Leveller: amount + speed.
    if (leveller_config.enabled) {
        const char *speed = "SLOW";
        switch (leveller_config.speed) {
            case LEVELLER_SPEED_MEDIUM: speed = "MED";  break;
            case LEVELLER_SPEED_FAST:   speed = "FAST"; break;
            default: break;
        }
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "LV ON  %d%% %s",
                 (int)leveller_config.amount, speed);
    } else {
        snprintf(oled_frame[r], sizeof(oled_frame[r]), "LV OFF");
    }
    r++;

    for (; r < OLED_PAGES; r++)
        oled_frame[r][0] = '\0';
}

// Build the 8 display lines for the current page into oled_frame[].
static void oled_build_frame(void) {
    char buf[OLED_WIDTH / 6 + 1];
    int n_active = oled_count_active_bands();
    int n_eq = oled_eq_page_count();

    // Clamp the page index to the total slideshow (EQ pages + effects page).
    if (oled_page_idx >= oled_page_count())
        oled_page_idx = 0;

    // Rows 0..1: title + source, rate + volume / mute (shared by all pages).
    snprintf(oled_frame[0], sizeof(oled_frame[0]), "DSPi %s", oled_source_name());
    if (user_mute || audio_state.mute) {
        snprintf(oled_frame[1], sizeof(oled_frame[1]), "MUTE");
    } else {
        oled_format_rate(audio_state.freq, buf, sizeof(buf));
        snprintf(oled_frame[1], sizeof(oled_frame[1]), "%s %+.1fdB",
                 buf, (double)master_volume_db);
    }

    // Effects status page: always the last slideshow page.
    if (oled_page_idx >= n_eq) {
        oled_build_fx_rows(oled_page_idx + 1, oled_page_count());
        return;
    }

    // EQ page: header + this page's slice of the non-flat bands.
    char ch_name[PRESET_NAME_LEN];
    get_default_channel_name(oled_eq_channel, active_input_source, NULL, ch_name);
    snprintf(oled_frame[2], sizeof(oled_frame[2]), "EQ %s (%d) %d/%d",
             ch_name, n_active, oled_page_idx + 1, n_eq);

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

// Render `s` as black text on a filled white bar covering page `row`.
// Gives the slideshow title row a distinct header look (white bar with
// black glyphs) versus the normal content rows below.  Walks the same
// font_8x5 glyph layout as ssd1306_draw_char but clears the glyph pixels
// instead of setting them, over a row-wide white bar.
static void oled_draw_inverted_row(uint8_t row, const char *s) {
    const uint32_t y = row * 8u;
    ssd1306_draw_square(&oled_disp, 0, y, OLED_WIDTH, 8);
    uint32_t x = 0;
    for (const char *p = s; *p; p++) {
        uint8_t c = (uint8_t)*p;
        if (c < font_8x5[3] || c > font_8x5[4]) {
            x += font_8x5[1] + font_8x5[2];
            continue;
        }
        const uint8_t *g = &font_8x5[5 + (c - font_8x5[3]) * font_8x5[1]];
        for (uint32_t w = 0; w < font_8x5[1]; w++) {
            uint8_t line = g[w];
            for (int8_t j = 0; j < 8; j++, line >>= 1) {
                if (line & 1)
                    ssd1306_clear_pixel(&oled_disp, x + w, y + j);
            }
        }
        x += font_8x5[1] + font_8x5[2];
    }
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
        if (oled_frame[i][0] == '\0') continue;
        if (i == 0)
            oled_draw_inverted_row(0, oled_frame[0]);
        else
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

    // Slideshow auto-rotation is DISABLED for now: the display stays on the
    // page set by oled_set_page() (default page 0) so the periodic full-frame
    // I²C flush on page change (~21 ms per 1 KB @ 400 kHz) can't stall the
    // audio main loop and cause a click/pop in the stream.  The rebuild +
    // flush throttle below still live-updates content (source, rate, volume,
    // EQ/effect edits).  All page designs remain in oled_build_frame() and are
    // reachable manually via oled_set_page().  Re-enable rotation by
    // restoring the timeout advance below.
    //
    // if (time_reached(oled_next_page)) {
    //     oled_next_page = make_timeout_time_ms(OLED_PAGE_DURATION_MS);
    //     oled_page_idx++;
    // }

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
