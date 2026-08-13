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
 *   row 0  "RYNLABS DSPi"      brand (inverted bar, left) + active preset
 *                              name flush right (truncated; "P<n>" fallback)
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
#include "flash_storage.h"  // preset_get_active() for the header preset tag

// Bands shown per EQ page (rows 3..7 = 5 rows).
#define OLED_PAGE_BANDS     5u

static ssd1306_t oled_disp;
static bool oled_auto = true;       // auto slideshow on
static char oled_frame[OLED_PAGES][OLED_WIDTH / 6 + 1];  // last rendered lines
static char oled_preset_tag[12] = "";  // header right-aligned text (name or "P<n>")
static uint8_t oled_eq_channel = 0;
static uint8_t oled_page_idx = 0;   // current slideshow page
static absolute_time_t oled_next_page;
static absolute_time_t oled_next_flush;

// Preset-change flash overlay (see oled_flash_preset()).
#define OLED_FLASH_MS 2500u
static bool oled_flash_active = false;
static bool oled_flash_dirty = false;
static absolute_time_t oled_flash_until;
static char oled_flash_l1[OLED_WIDTH / 6 + 1];  // "LOADED P3" / "SAVED P3" / ...
static char oled_flash_l2[OLED_WIDTH / 6 + 1];  // preset name (<= 21 chars)

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

// Total slideshow pages: EQ Curve page (0) + EQ band pages (1..n) + DSP effects page (last).
static int oled_page_count(void) {
    return 1 + oled_eq_page_count() + 1;
}

// Row 0 header: brand text on the left, active preset name on the right.
// The name is truncated to the number of characters that fit after the brand
// (so a long name can never overlap it), and falls back to "P<n>" when the
// active slot has no name.
static void oled_build_header(void) {
    static const char brand[] = "RYNLABS DSPi";
    const uint8_t slot = preset_get_active();
    snprintf(oled_frame[0], sizeof(oled_frame[0]), "%s", brand);

    const size_t cw = font_8x5[1] + font_8x5[2];
    size_t max_r = (OLED_WIDTH - strlen(brand) * cw) / cw;
    if (max_r > sizeof(oled_preset_tag) - 1u)
        max_r = sizeof(oled_preset_tag) - 1u;

    char name[PRESET_NAME_LEN] = "";
    if (preset_get_name(slot, name) != PRESET_OK)
        name[0] = '\0';
    if (name[0] == '\0') {
        snprintf(oled_preset_tag, sizeof(oled_preset_tag), "P%u",
                 (unsigned)(slot + 1u));
    } else {
        name[max_r] = '\0';
        snprintf(oled_preset_tag, sizeof(oled_preset_tag), "%s", name);
    }
}

// Build the 8 display lines for the current page into oled_frame[].
static void oled_build_frame(void) {
    char buf[OLED_WIDTH / 6 + 1];
    int n_active = oled_count_active_bands();
    int n_eq = oled_eq_page_count();
    int total_pages = oled_page_count();

    // Clamp the page index
    if (oled_page_idx >= total_pages)
        oled_page_idx = 0;

    // Rows 0..1: title + source, rate + volume / mute (shared by all pages).
    // Row 0 header is fixed brand text; the active preset name rides on the
    // right edge, truncated to fit (see oled_build_header).  The input source
    // is not shown on the OLED.
    oled_build_header();
    if (user_mute || audio_state.mute) {
        snprintf(oled_frame[1], sizeof(oled_frame[1]), "MUTE");
    } else {
        // STATIC RATE CLAIM: always advertise 96 kHz / 24-bit on the OLED,
        // regardless of the rate actually applied to the pipeline.  Deliberate
        // cosmetic lie so the OLED matches what Android reports for the device
        // (its descriptor advertises up to 96000 Hz / 24-bit).
        //
        // To revert to the real applied rate, uncomment the two lines below and
        // delete the static snprintf:
        //   oled_format_rate(audio_state.freq, buf, sizeof(buf));
        //   snprintf(oled_frame[1], sizeof(oled_frame[1]), "%s %+.1fdB",
        //            buf, (double)master_volume_db);
        snprintf(oled_frame[1], sizeof(oled_frame[1]), "96 kHz 24 bit %+.1fdB",
                 (double)master_volume_db);
    }

    // Page 0: EQ Curve Graph (Header row 2 used for Curve title)
    if (oled_page_idx == 0) {
        char ch_name[PRESET_NAME_LEN];
        get_default_channel_name(oled_eq_channel, active_input_source, NULL, ch_name);
        snprintf(oled_frame[2], sizeof(oled_frame[2]), "CURVE %s (1/%d)", ch_name, total_pages);
        for (int r = 3; r < OLED_PAGES; r++) oled_frame[r][0] = '\0';
        return;
    }

    // Last Page: DSP Effects Status Page
    if (oled_page_idx == total_pages - 1) {
        oled_build_fx_rows(oled_page_idx + 1, total_pages);
        return;
    }

    // Page 1..n_eq: EQ Text Bands Pages
    int eq_sub_page = oled_page_idx - 1;
    char ch_name[PRESET_NAME_LEN];
    get_default_channel_name(oled_eq_channel, active_input_source, NULL, ch_name);
    snprintf(oled_frame[2], sizeof(oled_frame[2]), "EQ %s (%d) %d/%d",
             ch_name, n_active, oled_page_idx + 1, total_pages);

    int r = 3;
    int first = eq_sub_page * OLED_PAGE_BANDS;
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

#include <math.h>

// Calculate exact biquad frequency response magnitude in dB at frequency f_hz using precomputed active filters
static float oled_eval_eq_gain_db(float f_hz, const Filter *active_f, const EqParamPacket *active_p, int active_count) {
    if (f_hz <= 0.0f || active_count == 0) return 0.0f;
    float total_db = 0.0f;
    float fs = audio_state.freq > 0 ? (float)audio_state.freq : 48000.0f;
    float w0 = 2.0f * (float)M_PI * f_hz / fs;
    float cos_w = cosf(w0);
    float cos_2w = cosf(2.0f * w0);
    float sin_w = sinf(w0);
    float sin_2w = sinf(2.0f * w0);

    for (int i = 0; i < active_count; i++) {
        const Filter *f = &active_f[i];
#if PICO_RP2350
        float b0 = f->b0, b1 = f->b1, b2 = f->b2;
        float a1 = f->a1, a2 = f->a2;
#else
        float b0 = (float)f->b0 / (float)(1 << FILTER_SHIFT);
        float b1 = (float)f->b1 / (float)(1 << FILTER_SHIFT);
        float b2 = (float)f->b2 / (float)(1 << FILTER_SHIFT);
        float a1 = (float)f->a1 / (float)(1 << FILTER_SHIFT);
        float a2 = (float)f->a2 / (float)(1 << FILTER_SHIFT);
#endif

        float num_r = b0 + b1 * cos_w + b2 * cos_2w;
        float num_i = -(b1 * sin_w + b2 * sin_2w);
        float den_r = 1.0f + a1 * cos_w + a2 * cos_2w;
        float den_i = -(a1 * sin_w + a2 * sin_2w);

        float num_sq = num_r * num_r + num_i * num_i;
        float den_sq = den_r * den_r + den_i * den_i;

        if (den_sq > 1e-12f && num_sq > 1e-12f) {
            float mag_sq = num_sq / den_sq;
            total_db += 10.0f * log10f(mag_sq);
        }
    }
    return total_db;
}

// Render graphical EQ magnitude response curve into ssd1306 framebuffer.
static void oled_draw_eq_curve_overlay(ssd1306_t *disp) {
    // Pre-calculate coefficients for active bands ONCE to eliminate CPU overhead in loop
    Filter active_f[MAX_BANDS];
    EqParamPacket active_p[MAX_BANDS];
    int active_count = 0;
    float fs = audio_state.freq > 0 ? (float)audio_state.freq : 48000.0f;

    for (int b = 0; b < MAX_BANDS; b++) {
        const EqParamPacket *p = &filter_recipes[oled_eq_channel][b];
        if (p->type == FILTER_FLAT || p->freq <= 0.0f || p->bypass == 1) continue;
        Filter f;
        dsp_compute_coefficients((EqParamPacket*)p, &f, fs);
        if (f.bypass) continue;
        active_f[active_count] = f;
        active_p[active_count] = *p;
        active_count++;
    }

    // 1. Grid Lines Background (Area Y=22 to Y=55)
    const int y_top = 22;     // +6dB / +12dB Upper Bound
    const int y_zero = 39;    // 0dB Center Line
    const int y_bot = 56;     // -6dB / -12dB Lower Bound

    // Horizontal grid lines (Dotted upper, 0dB baseline, lower)
    for (int x = 0; x < OLED_WIDTH; x += 3) {
        ssd1306_draw_pixel(disp, x, y_top);
        ssd1306_draw_pixel(disp, x, y_zero);
        ssd1306_draw_pixel(disp, x, y_bot);
    }

    // Vertical grid lines (100Hz, 1kHz, 10kHz)
    const int x_100hz = 30;
    const int x_1khz  = 72;
    const int x_10khz = 114;

    for (int y = y_top; y <= y_bot; y += 3) {
        ssd1306_draw_pixel(disp, x_100hz, y);
        ssd1306_draw_pixel(disp, x_1khz, y);
        ssd1306_draw_pixel(disp, x_10khz, y);
    }

    // 2. Plot EQ Response Curve & Dynamic Visual Gain Boost
    const float gain_scale = 2.833f;

    int prev_y = y_zero;
    for (int x = 0; x < OLED_WIDTH; x++) {
        // Logarithmic scale mapping: 20Hz (x=0) -> 20kHz (x=127)
        float log_f = 1.30103f + ((float)x / 127.0f) * 3.0f;
        float f_hz = powf(10.0f, log_f);

        float gain_db = oled_eval_eq_gain_db(f_hz, active_f, active_p, active_count);

        // Map gain_db to Y pixels
        int y = y_zero - (int)(gain_db * gain_scale);
        if (y < y_top) y = y_top;
        if (y > y_bot) y = y_bot;

        // Draw solid curve line
        if (x == 0) {
            ssd1306_draw_pixel(disp, x, y);
        } else {
            ssd1306_draw_line(disp, x - 1, prev_y, x, y);
        }

        // Add vertical fill under active peaks
        if (x % 2 == 0) {
            if (y < y_zero) {
                for (int fill_y = y + 1; fill_y < y_zero; fill_y += 2) {
                    ssd1306_draw_pixel(disp, x, fill_y);
                }
            } else if (y > y_zero) {
                for (int fill_y = y_zero + 1; fill_y < y; fill_y += 2) {
                    ssd1306_draw_pixel(disp, x, fill_y);
                }
            }
        }

        prev_y = y;
    }

    // 3. Mark Active PEQ Center Points (Dots on the curve for each active band)
    for (int i = 0; i < active_count; i++) {
        const EqParamPacket *p = &active_p[i];
        float log_f0 = log10f(p->freq);
        int center_x = (int)(((log_f0 - 1.30103f) / 3.0f) * 127.0f);
        if (center_x >= 0 && center_x < OLED_WIDTH) {
            float gain_db = oled_eval_eq_gain_db(p->freq, active_f, active_p, active_count);
            int center_y = y_zero - (int)(gain_db * gain_scale);
            if (center_y < y_top) center_y = y_top;
            if (center_y > y_bot) center_y = y_bot;

            // Draw a small 3x3 dot marker on the band center
            ssd1306_draw_square(disp, center_x > 0 ? center_x - 1 : 0, center_y > y_top ? center_y - 1 : y_top, 3, 3);
        }
    }

    // 4. Bottom Frequency Axis Labels (Row 7, Y=57)
    ssd1306_draw_string(disp, 0, 57, 1, "20");
    ssd1306_draw_string(disp, 22, 57, 1, "100");
    ssd1306_draw_string(disp, 64, 57, 1, "1k");
    ssd1306_draw_string(disp, 100, 57, 1, "10k");
}

// Render `left` as black text on a filled white bar covering page `row`,
// with `right` black text flush against the right edge of the bar.
// Gives the slideshow title row a distinct header look (white bar with
// black glyphs) versus the normal content rows below.  Walks the same
// font_8x5 glyph layout as ssd1306_draw_char but clears the glyph pixels
// instead of setting them, over a row-wide white bar.
static void oled_draw_inverted_row(uint8_t row, const char *left, const char *right) {
    const uint32_t y = row * 8u;
    ssd1306_draw_square(&oled_disp, 0, y, OLED_WIDTH, 8);

    uint32_t x = 0;
    for (const char *p = left; *p; p++) {
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

    if (right && right[0]) {
        const uint32_t cw = font_8x5[1] + font_8x5[2];
        x = OLED_WIDTH - (uint32_t)strlen(right) * cw;
        for (const char *p = right; *p; p++) {
            uint8_t c = (uint8_t)*p;
            if (c < font_8x5[3] || c > font_8x5[4]) {
                x += cw;
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
            x += cw;
        }
    }
}

// Rebuild + push the frame to the display if any line or filter recipe differs.
static bool oled_render_auto(void) {
    char prev[OLED_PAGES][OLED_WIDTH / 6 + 1];
    char prev_tag[sizeof(oled_preset_tag)];
    static uint32_t prev_eq_hash = 0;
    memcpy(prev, oled_frame, sizeof(prev));
    memcpy(prev_tag, oled_preset_tag, sizeof(prev_tag));
    oled_build_frame();

    // Fast hash check of active channel filter recipes to detect EQ changes
    uint32_t current_eq_hash = 5381;
    uint8_t *rec_bytes = (uint8_t*)&filter_recipes[oled_eq_channel];
    for (size_t i = 0; i < sizeof(filter_recipes[oled_eq_channel]); i++) {
        current_eq_hash = ((current_eq_hash << 5) + current_eq_hash) + rec_bytes[i];
    }

    // Preset flash overlay lifecycle: show it exactly once when armed
    // (oled_flash_dirty), then force one final redraw when it expires so the
    // normal page content reappears without waiting for another change.
    bool flash_active  = oled_flash_active && !time_reached(oled_flash_until);
    bool flash_expired = oled_flash_active && time_reached(oled_flash_until);
    if (flash_expired) {
        oled_flash_active = false;
        oled_flash_l1[0] = '\0';
        oled_flash_l2[0] = '\0';
    }

    bool changed = (memcmp(prev, oled_frame, sizeof(prev)) != 0)
                   || (memcmp(prev_tag, oled_preset_tag, sizeof(prev_tag)) != 0)
                   || (current_eq_hash != prev_eq_hash)
                   || oled_flash_dirty || flash_expired;
    oled_flash_dirty = false;
    if (!changed) return false;

    prev_eq_hash = current_eq_hash;

    ssd1306_clear(&oled_disp);

    // Row 0 is always top Inverted Header (brand left, preset tag right).
    if (oled_frame[0][0] != '\0') {
        oled_draw_inverted_row(0, oled_frame[0], oled_preset_tag);
    }
    // Row 1 is rate / volume / mute
    if (oled_frame[1][0] != '\0') {
        ssd1306_draw_string(&oled_disp, 0, 8, 1, oled_frame[1]);
    }

    // Preset-change flash overlay: briefly show action + name over the page
    // content (rows 2..3), then fall through to the normal page once the
    // timer expires (flash_expired forces this redraw).
    if (flash_active) {
        if (oled_flash_l1[0] != '\0')
            ssd1306_draw_string(&oled_disp, 0, 16, 1, oled_flash_l1);
        if (oled_flash_l2[0] != '\0')
            ssd1306_draw_string(&oled_disp, 0, 24, 1, oled_flash_l2);
    } else
    // If Page 0, draw smooth continuous EQ Magnitude Curve in lower area
    if (oled_page_idx == 0) {
        oled_draw_eq_curve_overlay(&oled_disp);
    } else {
        // Normal text rows for remaining pages
        for (int i = 2; i < OLED_PAGES; i++) {
            if (oled_frame[i][0] == '\0') continue;
            ssd1306_draw_string(&oled_disp, 0, i * 8, 1, oled_frame[i]);
        }
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

void oled_flash_preset(uint8_t slot, uint8_t action, const char *name) {
    static const char *const label[3] = { "LOADED", "SAVED", "DELETED" };
    if (!oled_auto) return;
    if (slot >= PRESET_SLOTS) slot = preset_get_active();
    if (action > OLED_FLASH_DELETED) action = OLED_FLASH_LOADED;

    char namebuf[PRESET_NAME_LEN] = "";
    if (name == NULL) {
        if (preset_get_name(slot, namebuf) != PRESET_OK) namebuf[0] = '\0';
        name = namebuf;
    }
    if (name[0] == '\0') name = "(unnamed)";

    snprintf(oled_flash_l1, sizeof(oled_flash_l1), "%s P%u", label[action],
             (unsigned)(slot + 1u));
    snprintf(oled_flash_l2, sizeof(oled_flash_l2), "%s", name);
    oled_flash_active = true;
    oled_flash_dirty = true;
    oled_flash_until = make_timeout_time_ms(OLED_FLASH_MS);
    oled_next_flush = make_timeout_time_ms(0);   // render at the next tick
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
