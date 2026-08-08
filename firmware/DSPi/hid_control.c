/*
 * hid_control.c — AD1-style USB HID control interface for DSPi
 *
 * The Kiwi Ears AD1 exposes its PEQ/volume surface as a vendor HID interface
 * carrying 10-byte frames on Report ID 0x4B (see
 * kiwi_ears_ad1_protocol_spec.md):
 *
 *     byte 0    RegID       register address
 *     byte 1-3  Reserved    0x00
 *     byte 4    CMD         0x52 Read / 0x57 Write / 0x53 Commit-Flash
 *     byte 5    Reserved    0x00
 *     byte 6-9  Payload     Read: status (0x03 OK) or data; Write: data
 *
 * DSPi maps that register file onto its transport-neutral vendor command
 * surface via vendor_dispatch_get/set with CTRL_SOURCE_HID.  All dispatches
 * run from tud_hid_set_report_cb (main-loop context), matching the UART/I2C
 * transports' use of the same dispatcher.
 *
 * Register map (band N occupies two adjacent byte registers):
 *   0x24  EQ enable           0 = bypassed, 1 = active
 *   0x26+2N  band N gain+freq  p0/p1 gain int16 0.1 dB, p2/p3 freq u16 Hz
 *   0x27+2N  band N Q+type     p0/p1 Q u16 x1000, p2 filter type, p3 unused
 *   0x3B  preamp index         0..15, dB = (idx-13)*1.5
 *   0x40  channel select       0..NUM_CHANNELS-1 (applies to 0x26..0x3B)
 *   0x41  master volume        p0/p1 int16 0.1 dB (<= -1280 = mute)
 *   0x42  mute                 0 = unmuted, 1 = muted
 */

#include <string.h>

#include "tusb.h"
#include "hid_control.h"
#include "usb_descriptors.h"
#include "vendor_commands.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"       // audio_state.freq
#include "hardware/sync.h"   // save_and_disable_interrupts / restore_interrupts

// --- AD1 command codes ---
#define HID_CMD_READ    0x52
#define HID_CMD_WRITE   0x57
#define HID_CMD_COMMIT  0x53

// --- Register file ---
#define HID_REG_EQ_ENABLE   0x24
#define HID_REG_BAND_BASE   0x26   // band N gain+freq at +2N, Q+type at +2N+1
#define HID_REG_BAND_MAX    0x39   // band 9 Q+type
#define HID_REG_PREAMP      0x3B
#define HID_REG_CHANNEL     0x40
#define HID_REG_MASTER_VOL  0x41
#define HID_REG_MUTE        0x42

// --- Reply status byte ---
#define HID_STATUS_OK       0x03
#define HID_STATUS_ERROR    0x00

#define HID_PREAMP_BASE     13      // LUT index whose value is 0 dB
#define HID_PREAMP_STEP     1.5f    // dB per LUT step

// --- Reply queue ---
// tud_hid_n_report() copies the reply into TinyUSB's single internal buffer
// (p_epbuf->epin) and starts the EP-IN transfer; a second call while the EP
// is still busy fails usbd_edpt_claim() and silently drops the report.  The
// console often sends a WRITE followed immediately by a READ, so a single
// "last reply" slot loses the READ response.  Buffer replies in a small ring
// and drain them from the main loop (hid_control_tick) whenever the EP is
// idle.  _hid_reply is kept as the most-recent reply for the GET_REPORT
// control-transfer path.
#define HID_REPLY_Q_SLOTS  8

static uint8_t _hid_channel;        // channel select state (register 0x40)
static uint8_t _hid_reply[HID_CONTROL_ITF_BYTE_LEN];   // most-recent reply (GET path)
static bool    _hid_reply_valid;
static uint8_t _hid_reply_q[HID_REPLY_Q_SLOTS][HID_CONTROL_ITF_BYTE_LEN];
static volatile uint8_t _hid_reply_q_head;  // next reply to transmit
static volatile uint8_t _hid_reply_q_tail;  // next free slot

// --- Filter-type mapping (AD1 enum <-> DSPi FilterType) ---
static uint8_t ad1_type_to_dspi(uint8_t t) {
    switch (t) {
        case 0x00: return (uint8_t)FILTER_PEAKING;
        case 0x03: return (uint8_t)FILTER_LOWSHELF;
        case 0x04: return (uint8_t)FILTER_HIGHSHELF;
        case 0x10: return (uint8_t)FILTER_FLAT;
        case 0x11: return (uint8_t)FILTER_LOWPASS;
        case 0x12: return (uint8_t)FILTER_HIGHPASS;
        case 0x13: return (uint8_t)FILTER_NOTCH;
        case 0x14: return (uint8_t)FILTER_ALLPASS;
        case 0x15: return (uint8_t)FILTER_ALLPASS1;
        case 0x16: return (uint8_t)FILTER_LOWSHELF1;
        case 0x17: return (uint8_t)FILTER_HIGHSHELF1;
        case 0x18: return (uint8_t)FILTER_LINKWITZ_TRANSFORM;
        case 0x19: return (uint8_t)FILTER_LOWPASS1;
        case 0x1A: return (uint8_t)FILTER_HIGHPASS1;
        default:   return (uint8_t)FILTER_PEAKING;
    }
}

static uint8_t dspi_type_to_ad1(uint8_t t) {
    switch (t) {
        case FILTER_PEAKING:         return 0x00;
        case FILTER_LOWSHELF:        return 0x03;
        case FILTER_HIGHSHELF:       return 0x04;
        case FILTER_FLAT:            return 0x10;
        case FILTER_LOWPASS:         return 0x11;
        case FILTER_HIGHPASS:        return 0x12;
        case FILTER_NOTCH:           return 0x13;
        case FILTER_ALLPASS:         return 0x14;
        case FILTER_ALLPASS1:        return 0x15;
        case FILTER_LOWSHELF1:       return 0x16;
        case FILTER_HIGHSHELF1:      return 0x17;
        case FILTER_LINKWITZ_TRANSFORM: return 0x18;
        case FILTER_LOWPASS1:        return 0x19;
        case FILTER_HIGHPASS1:       return 0x1A;
        default:                     return 0x00;
    }
}

// --- Small numeric helpers ---
static int16_t hid_round_f32(float v) {
    if (v >= 0.0f) return (int16_t)(v + 0.5f);
    return (int16_t)(v - 0.5f);
}

static uint16_t hid_clamp_u16(int32_t v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return (uint16_t)v;
}

static uint8_t hid_clamp_u8(int32_t v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

// --- Dispatcher helpers ---
static bool hid_set_dispatch(uint8_t bRequest, uint16_t wValue,
                             const uint8_t *payload, uint16_t wLength) {
    return vendor_dispatch_set(CTRL_SOURCE_HID, bRequest, wValue, 0,
                               payload, wLength) == CTRL_DISPATCH_OK;
}

static bool hid_get_dispatch(uint8_t bRequest, uint16_t wValue,
                             uint16_t wLength, const uint8_t **rd, uint16_t *rl) {
    return vendor_dispatch_get(CTRL_SOURCE_HID, bRequest, wValue, 0,
                               wLength, rd, rl) == CTRL_DISPATCH_OK;
}

// Read one EQ scalar for the active channel (param: 0=type, 1=freq, 2=Q,
// 3=gain_db).  Response is 4 bytes LE.
static bool hid_get_eq_scalar(uint8_t ch, uint8_t band, uint8_t param, uint32_t *val) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    uint16_t wValue = (uint16_t)(((uint16_t)ch << 8) | ((uint16_t)band << 3) | param);
    if (!hid_get_dispatch(REQ_GET_EQ_PARAM, wValue, 4, &rd, &rl) || rl < 4)
        return false;
    memcpy(val, rd, 4);
    return true;
}

// Push a full band recipe through REQ_SET_EQ_PARAM (the handler validates the
// band index against channel_band_counts and latches pending_packet).
static bool hid_send_eq(uint8_t ch, uint8_t band, const EqParamPacket *r) {
    EqParamPacket out = *r;
    out.channel = ch;
    out.band    = band;
    out.bypass  = (out.bypass == 1) ? 1 : 0;
    return hid_set_dispatch(REQ_SET_EQ_PARAM, 0, (const uint8_t *)&out,
                            sizeof(EqParamPacket));
}

// --- Register handlers ---
static bool hid_reg_write_eq_enable(const uint8_t *p) {
    uint8_t v = (p[0] == 1) ? 0 : 1;   // enable -> inverse of bypass
    return hid_set_dispatch(REQ_SET_BYPASS, 0, &v, 1);
}

static bool hid_reg_read_eq_enable(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_GET_BYPASS, 0, 1, &rd, &rl) || rl < 1)
        return false;
    out[0] = (rd[0] == 1) ? 0 : 1;
    out[1] = out[2] = out[3] = 0;
    return true;
}

// The AD1 is a stereo device: its PEQ/preamp surface applies identically to
// the L and R channels.  DSPi carries the stereo pair as two input channels
// (USB1 L = ch 0, USB2 R = ch 1) which both feed the same stereo outputs in
// the matrix mixer, so mirror per-channel writes to the partner channel to
// keep the stereo image consistent.
static uint8_t hid_stereo_partner(uint8_t ch) {
    uint8_t p = ch ^ 1;                     // 0<->1, 2<->3, ...
    return (p < NUM_INPUT_CHANNELS) ? p : ch;
}

// Apply a band recipe to a channel synchronously, replicating what main.c's
// async consumer does for REQ_SET_EQ_PARAM (peq_qp_x512 -> filter_recipes ->
// dsp_compute_coefficients -> channel_bypassed recalc).  Only used for the
// stereo-partner mirror: the primary channel keeps going through the single
// REQ_SET_EQ_PARAM pending slot, and dispatching a second EQ write inside the
// same callback would overwrite the first pending_packet before the main loop
// can consume it.  Input channels (0/1) never hit the Core 1 fence, so no
// eq-worker sync is needed here.
static void hid_apply_band_sync(uint8_t ch, uint8_t band, const EqParamPacket *r) {
    if (ch >= NUM_CHANNELS || band >= channel_band_counts[ch])
        return;
    EqParamPacket local = *r;
    local.channel = ch;
    local.band    = band;
    local.bypass  = (local.bypass == 1) ? 1 : 0;
    // Mirror the LT target Qp from the primary channel (host writes ride the
    // primary channel's recipe; a per-band Qp does not belong to the partner).
    peq_qp_x512[ch][band] = peq_qp_x512[_hid_channel][band];
    filter_recipes[ch][band] = local;
    uint32_t flags = save_and_disable_interrupts();
    dsp_compute_coefficients(&local, &filters[ch][band], (float)audio_state.freq);
    bool all_bypassed = true;
    for (int b = 0; b < channel_band_counts[ch]; b++) {
        if (!filters[ch][b].bypass) { all_bypassed = false; break; }
    }
    channel_bypassed[ch] = all_bypassed;
    restore_interrupts(flags);
}

static bool hid_reg_write_band(uint8_t band, bool gain_freq, const uint8_t *p) {
    if (_hid_channel >= NUM_CHANNELS || band >= channel_band_counts[_hid_channel])
        return false;
    EqParamPacket r = filter_recipes[_hid_channel][band];   // preserve untouched fields
    if (gain_freq) {
        int16_t g10 = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        r.gain_db = (float)g10 / 10.0f;
        r.freq    = (float)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
    } else {
        uint16_t q1000 = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        r.Q    = (float)q1000 / 1000.0f;
        r.type = ad1_type_to_dspi(p[2]);
    }
    bool ok = hid_send_eq(_hid_channel, band, &r);
    uint8_t partner = hid_stereo_partner(_hid_channel);
    if (ok && partner != _hid_channel &&
        band < channel_band_counts[partner])
        hid_apply_band_sync(partner, band, &r);
    return ok;
}

static bool hid_reg_read_band(uint8_t band, bool gain_freq, uint8_t *out) {
    if (_hid_channel >= NUM_CHANNELS || band >= channel_band_counts[_hid_channel])
        return false;
    uint32_t v;
    if (gain_freq) {
        if (!hid_get_eq_scalar(_hid_channel, band, 3, &v)) return false;
        float gdb; memcpy(&gdb, &v, 4);
        int16_t g10 = hid_round_f32(gdb * 10.0f);
        out[0] = (uint8_t)(g10 & 0xFF);
        out[1] = (uint8_t)((g10 >> 8) & 0xFF);
        if (!hid_get_eq_scalar(_hid_channel, band, 1, &v)) return false;
        float fhz; memcpy(&fhz, &v, 4);
        uint16_t f = hid_clamp_u16((int32_t)(fhz + 0.5f));
        out[2] = (uint8_t)(f & 0xFF);
        out[3] = (uint8_t)(f >> 8);
    } else {
        if (!hid_get_eq_scalar(_hid_channel, band, 2, &v)) return false;
        float q; memcpy(&q, &v, 4);
        uint16_t q1000 = hid_clamp_u16(hid_round_f32(q * 1000.0f));
        out[0] = (uint8_t)(q1000 & 0xFF);
        out[1] = (uint8_t)(q1000 >> 8);
        if (!hid_get_eq_scalar(_hid_channel, band, 0, &v)) return false;
        out[2] = dspi_type_to_ad1((uint8_t)v);
        out[3] = 0;
    }
    return true;
}

static bool hid_reg_write_preamp(const uint8_t *p) {
    if (_hid_channel >= NUM_CHANNELS) return false;
    float db = ((float)(p[0] & 0x0F) - (float)HID_PREAMP_BASE) * HID_PREAMP_STEP;
    bool ok = hid_set_dispatch(REQ_SET_PREAMP_CH, _hid_channel,
                               (const uint8_t *)&db, sizeof(float));
    uint8_t partner = hid_stereo_partner(_hid_channel);
    if (ok && partner != _hid_channel)
        ok = hid_set_dispatch(REQ_SET_PREAMP_CH, partner,
                              (const uint8_t *)&db, sizeof(float));
    return ok;
}

static bool hid_reg_read_preamp(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (_hid_channel >= NUM_CHANNELS) return false;
    if (!hid_get_dispatch(REQ_GET_PREAMP_CH, _hid_channel, 4, &rd, &rl) || rl < 4)
        return false;
    float db; memcpy(&db, rd, 4);
    int32_t idx = hid_round_f32(db / HID_PREAMP_STEP) + HID_PREAMP_BASE;
    out[0] = hid_clamp_u8(idx);
    out[1] = out[2] = out[3] = 0;
    return true;
}

static bool hid_reg_write_channel(const uint8_t *p) {
    if (p[0] >= NUM_CHANNELS) return false;
    _hid_channel = p[0];
    return true;
}

static bool hid_reg_read_channel(uint8_t *out) {
    out[0] = _hid_channel;
    out[1] = out[2] = out[3] = 0;
    return true;
}

static bool hid_reg_write_master_vol(const uint8_t *p) {
    int16_t v10 = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    float db = (float)v10 / 10.0f;
    if (v10 <= -1280) db = -128.0f;   // mute sentinel
    return hid_set_dispatch(REQ_SET_MASTER_VOLUME, 0,
                            (const uint8_t *)&db, sizeof(float));
}

static bool hid_reg_read_master_vol(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_GET_MASTER_VOLUME, 0, 4, &rd, &rl) || rl < 4)
        return false;
    float db; memcpy(&db, rd, 4);
    int16_t v10 = hid_round_f32(db * 10.0f);
    if (v10 < -1280) v10 = -1280;
    if (v10 > 0)     v10 = 0;
    out[0] = (uint8_t)(v10 & 0xFF);
    out[1] = (uint8_t)((v10 >> 8) & 0xFF);
    out[2] = out[3] = 0;
    return true;
}

static bool hid_reg_write_mute(const uint8_t *p) {
    uint8_t v = (p[0] == 1) ? 1 : 0;
    return hid_set_dispatch(REQ_SET_USER_MUTE, 0, &v, 1);
}

static bool hid_reg_read_mute(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_GET_USER_MUTE, 0, 1, &rd, &rl) || rl < 1)
        return false;
    out[0] = rd[0];
    out[1] = out[2] = out[3] = 0;
    return true;
}

// --- Frame handling ---
static bool hid_reg_read(uint8_t reg, uint8_t *out) {
    if (reg == HID_REG_EQ_ENABLE)
        return hid_reg_read_eq_enable(out);
    if (reg >= HID_REG_BAND_BASE && reg <= HID_REG_BAND_MAX)
        return hid_reg_read_band((uint8_t)((reg - HID_REG_BAND_BASE) >> 1),
                                 (reg & 1) == 0, out);
    if (reg == HID_REG_PREAMP)
        return hid_reg_read_preamp(out);
    if (reg == HID_REG_CHANNEL)
        return hid_reg_read_channel(out);
    if (reg == HID_REG_MASTER_VOL)
        return hid_reg_read_master_vol(out);
    if (reg == HID_REG_MUTE)
        return hid_reg_read_mute(out);
    return false;
}

static bool hid_reg_write(uint8_t reg, const uint8_t *p) {
    if (reg == HID_REG_EQ_ENABLE)
        return hid_reg_write_eq_enable(p);
    if (reg >= HID_REG_BAND_BASE && reg <= HID_REG_BAND_MAX)
        return hid_reg_write_band((uint8_t)((reg - HID_REG_BAND_BASE) >> 1),
                                  (reg & 1) == 0, p);
    if (reg == HID_REG_PREAMP)
        return hid_reg_write_preamp(p);
    if (reg == HID_REG_CHANNEL)
        return hid_reg_write_channel(p);
    if (reg == HID_REG_MASTER_VOL)
        return hid_reg_write_master_vol(p);
    if (reg == HID_REG_MUTE)
        return hid_reg_write_mute(p);
    return false;
}

static bool hid_do_commit(void) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    // GET-style action: defers the flash write to the main loop, replies 1 byte.
    return hid_get_dispatch(REQ_SAVE_PARAMS, 0, 1, &rd, &rl) && rl >= 1;
}

// Queue one reply frame for the EP-IN interrupt pipe.
static void hid_reply_push(const uint8_t rep[HID_CONTROL_ITF_BYTE_LEN]) {
    // Keep the most-recent reply for the GET_REPORT control-transfer path.
    memcpy(_hid_reply, rep, HID_CONTROL_ITF_BYTE_LEN);
    _hid_reply_valid = true;

    uint8_t tail = _hid_reply_q_tail;
    uint8_t next = (uint8_t)((tail + 1) % HID_REPLY_Q_SLOTS);
    if (next == _hid_reply_q_head) {
        // Queue full: drop the oldest pending reply rather than the new one.
        _hid_reply_q_head = (uint8_t)((_hid_reply_q_head + 1) % HID_REPLY_Q_SLOTS);
    }
    memcpy(_hid_reply_q[tail], rep, HID_CONTROL_ITF_BYTE_LEN);
    _hid_reply_q_tail = next;
}

// Drain queued replies onto the HID interrupt IN endpoint.  Call from the main
// loop after tud_task(); each successful tud_hid_n_report() marks the EP busy
// (tud_hid_n_ready() goes false), so one report is handed off per call until
// the pipe is idle again.  Safe to call from tud_hid_set_report_cb as well.
void hid_control_tick(void) {
    while (_hid_reply_q_head != _hid_reply_q_tail) {
        if (!tud_hid_n_ready(0))
            return;   // EP busy: next report will be sent on a later tick
        if (!tud_hid_n_report(0, HID_RPT_ID,
                              _hid_reply_q[_hid_reply_q_head],
                              HID_CONTROL_ITF_BYTE_LEN))
            return;   // claim/xfer failed; retry later
        _hid_reply_q_head = (uint8_t)((_hid_reply_q_head + 1) % HID_REPLY_Q_SLOTS);
    }
}

bool hid_control_process_frame(const uint8_t *frame) {
    uint8_t rep[HID_CONTROL_ITF_BYTE_LEN] = {
        frame[0], 0x00, 0x00, 0x00,
        frame[4], 0x00,
        HID_STATUS_ERROR, 0x00, 0x00, 0x00
    };
    bool ok = false;

    switch (frame[4]) {
        case HID_CMD_READ:
            ok = hid_reg_read(frame[0], &rep[6]);
            break;
        case HID_CMD_WRITE:
            ok = hid_reg_write(frame[0], &frame[6]);
            if (ok) rep[6] = HID_STATUS_OK;
            break;
        case HID_CMD_COMMIT:
            ok = hid_do_commit();
            if (ok) rep[6] = HID_STATUS_OK;
            break;
        default:
            break;
    }

    hid_reply_push(rep);
    hid_control_tick();   // opportunistic immediate send when EP is idle
    return true;
}

// --- TinyUSB HID callbacks (compiled for all builds; the class is only
//     active when CFG_TUD_HID = 1) ---
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)itf;
    (void)report_id;
    if (report_type != HID_REPORT_TYPE_OUTPUT)
        return;
    if (bufsize < HID_CONTROL_ITF_BYTE_LEN)
        return;
    // Some hosts prepend the Report ID byte; accept both 10-byte (bare) and
    // 11-byte (ID-prefixed) frames.
    if (bufsize > HID_CONTROL_ITF_BYTE_LEN && buffer[0] == HID_RPT_ID)
        hid_control_process_frame(&buffer[1]);
    else
        hid_control_process_frame(buffer);
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)itf;
    (void)report_id;
    if (report_type != HID_REPORT_TYPE_INPUT)
        return 0;
    if (!_hid_reply_valid || reqlen < HID_CONTROL_ITF_BYTE_LEN)
        return 0;
    memcpy(buffer, _hid_reply, HID_CONTROL_ITF_BYTE_LEN);
    return HID_CONTROL_ITF_BYTE_LEN;
}
