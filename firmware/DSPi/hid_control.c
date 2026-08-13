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
 *   0x50..0x67  DSP effects    crossfeed / loudness / psybass / leveller
 *                              (v2 block, see rynlabs_dspi_hid_protocol_spec_v2.md)
 *                              global - NOT scoped by register 0x40 channel select
 *   0x80..0x87  presets        slot select / directory / active / name chunks /
 *                              action / startup (v3 block, see
 *                              rynlabs_dspi_hid_protocol_spec_v2.md §8.8).  Names are
 *                              32 bytes and ride the 4-byte payload in chunks; the
 *                              chunk index (0..7) lives in frame byte 1 and is
 *                              echoed back in reply byte 1.  Global - NOT scoped by
 *                              register 0x40 channel select.
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
#define HID_REG_EFF_BASE    0x50   // v2 DSP effects block (crossfeed .. leveller)
#define HID_REG_EFF_LAST    0x67

// --- Preset block (registers 0x80..0x87, v3) ---
#define HID_REG_PRESET_SLOT    0x80   // R/W byte: slot select 0..PRESET_SLOTS-1
#define HID_REG_PRESET_DIR_A   0x81   // R 4B: occupied u16 LE + startup + default_slot
#define HID_REG_PRESET_DIR_B   0x82   // R 4B: last_active + oc_mode + mv_mode
#define HID_REG_PRESET_ACTIVE  0x83   // R 1B: currently active slot
#define HID_REG_PRESET_NAME    0x84   // R 4B: name chunk (frame byte 1 = chunk 0..7)
#define HID_REG_PRESET_NAME_W  0x85   // W 4B: name chunk -> scratch (byte 1 = chunk)
#define HID_REG_PRESET_ACTION  0x86   // W 1B: 1=SAVE 2=LOAD 3=DELETE 4=APPLY_NAME
#define HID_REG_PRESET_STARTUP 0x87   // R/W 2B: [mode, default_slot] (R adds last_active)

#define HID_PRESET_CHUNKS      (PRESET_NAME_LEN / 4)   // 8 chunks of 4 bytes

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
static uint8_t _hid_preset_slot;    // preset slot select state (register 0x80)
static char    _hid_preset_name[PRESET_NAME_LEN];   // 32-byte name write scratch (0x85)
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

// --- Effects block (registers 0x50..0x67, v2) ---
// Bridges the four DSP effects (crossfeed, loudness, psybass, leveller) onto
// the transport-neutral vendor REQ surface.  All registers are GLOBAL: they are
// NOT scoped by the 0x40 channel select, unlike the PEQ block.  Scope clamping
// follows hid_expansion_plan.md §5: crossfeed 0x01, loudness/psybass 0x0003,
// leveller 0x03/0x03 (per byte).
typedef struct {
    uint8_t  set_req;   // vendor write REQ (0 = unsupported)
    uint8_t  get_req;   // vendor read REQ
    uint8_t  width;     // wire width: 1 = byte, 2 = u16 LE, 4 = f32 LE
    uint16_t clamp;     // write scope mask (0 = pass through)
} HidEffReg;

static const HidEffReg HID_EFF_REGS[0x68] = {
    [0x50] = { REQ_SET_CROSSFEED,          REQ_GET_CROSSFEED,         1, 0x0000 }, // crossfeed enable
    [0x51] = { REQ_SET_CROSSFEED_PRESET,   REQ_GET_CROSSFEED_PRESET,  1, 0x0000 }, // preset 0..3
    [0x52] = { REQ_SET_CROSSFEED_FREQ,     REQ_GET_CROSSFEED_FREQ,    4, 0x0000 }, // custom freq f32
    [0x53] = { REQ_SET_CROSSFEED_FEED,     REQ_GET_CROSSFEED_FEED,    4, 0x0000 }, // custom feed f32
    [0x54] = { REQ_SET_CROSSFEED_ITD,      REQ_GET_CROSSFEED_ITD,     1, 0x0000 }, // ITD enable
    [0x55] = { REQ_SET_CROSSFEED_OUTPUTS,  REQ_GET_CROSSFEED_OUTPUTS, 1, 0x0001 }, // output pair mask
    [0x56] = { REQ_SET_LOUDNESS,           REQ_GET_LOUDNESS,          1, 0x0000 }, // loudness enable
    [0x57] = { REQ_SET_LOUDNESS_REF,       REQ_GET_LOUDNESS_REF,      4, 0x0000 }, // ref SPL f32
    [0x58] = { REQ_SET_LOUDNESS_INTENSITY, REQ_GET_LOUDNESS_INTENSITY, 4, 0x0000 }, // intensity f32
    [0x59] = { REQ_SET_LOUDNESS_MASK,      REQ_GET_LOUDNESS_MASK,     2, 0x0003 }, // output mask u16
    [0x5A] = { REQ_SET_PSYBASS,            REQ_GET_PSYBASS,           1, 0x0000 }, // psybass enable
    [0x5B] = { REQ_SET_PSYBASS_CUTOFF,     REQ_GET_PSYBASS_CUTOFF,    4, 0x0000 }, // cutoff f32
    [0x5C] = { REQ_SET_PSYBASS_HARMONICS,  REQ_GET_PSYBASS_HARMONICS, 4, 0x0000 }, // harmonics f32
    [0x5D] = { REQ_SET_PSYBASS_DRIVE,      REQ_GET_PSYBASS_DRIVE,     4, 0x0000 }, // drive f32
    [0x5E] = { REQ_SET_PSYBASS_CHARACTER,  REQ_GET_PSYBASS_CHARACTER, 4, 0x0000 }, // character f32
    [0x5F] = { REQ_SET_PSYBASS_ORIGINAL,   REQ_GET_PSYBASS_ORIGINAL,  4, 0x0000 }, // original f32
    [0x60] = { REQ_SET_PSYBASS_MASK,       REQ_GET_PSYBASS_MASK,      2, 0x0003 }, // output mask u16
    [0x61] = { REQ_SET_LEVELLER_ENABLE,    REQ_GET_LEVELLER_ENABLE,   1, 0x0000 }, // leveller enable
    [0x62] = { REQ_SET_LEVELLER_AMOUNT,    REQ_GET_LEVELLER_AMOUNT,   4, 0x0000 }, // amount f32
    [0x63] = { REQ_SET_LEVELLER_SPEED,     REQ_GET_LEVELLER_SPEED,    1, 0x0000 }, // speed 0..2
    [0x64] = { REQ_SET_LEVELLER_MAX_GAIN,  REQ_GET_LEVELLER_MAX_GAIN, 4, 0x0000 }, // max gain f32
    [0x65] = { REQ_SET_LEVELLER_LOOKAHEAD, REQ_GET_LEVELLER_LOOKAHEAD, 1, 0x0000 }, // lookahead
    [0x66] = { REQ_SET_LEVELLER_GATE,      REQ_GET_LEVELLER_GATE,     4, 0x0000 }, // gate threshold f32
    [0x67] = { REQ_SET_LEVELLER_MASKS,     REQ_GET_LEVELLER_MASKS,    2, 0x0303 }, // det+apply masks
};

static bool hid_eff_write(uint8_t reg, const uint8_t *p) {
    if (reg < HID_REG_EFF_BASE || reg > HID_REG_EFF_LAST)
        return false;
    const HidEffReg *e = &HID_EFF_REGS[reg];
    if (e->set_req == 0 || e->width == 0)
        return false;
    uint8_t buf[2];
    switch (e->width) {
        case 1:
            buf[0] = (e->clamp != 0) ? (p[0] & (uint8_t)(e->clamp & 0xFF)) : p[0];
            return hid_set_dispatch(e->set_req, 0, buf, 1);
        case 2: {
            uint16_t v = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
            if (e->clamp) v &= e->clamp;
            buf[0] = (uint8_t)(v & 0xFF);
            buf[1] = (uint8_t)(v >> 8);
            return hid_set_dispatch(e->set_req, 0, buf, 2);
        }
        case 4:
            return hid_set_dispatch(e->set_req, 0, p, 4);
        default:
            return false;
    }
}

static bool hid_eff_read(uint8_t reg, uint8_t *out) {
    if (reg < HID_REG_EFF_BASE || reg > HID_REG_EFF_LAST)
        return false;
    const HidEffReg *e = &HID_EFF_REGS[reg];
    if (e->get_req == 0 || e->width == 0)
        return false;
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(e->get_req, 0, e->width, &rd, &rl) || rl < e->width)
        return false;
    out[0] = out[1] = out[2] = out[3] = 0;
    memcpy(out, rd, e->width);
    return true;
}

// --- Preset block (registers 0x80..0x87, v3) ---
// Bridges the 10-slot preset system (REQ_PRESET_*) onto the register file.
// All registers are GLOBAL — not scoped by the 0x40 channel select.  Names
// are PRESET_NAME_LEN (32) bytes, so they ride the 4-byte frame payload in
// chunks indexed by frame byte 1 (echoed back in reply byte 1).  SAVE/LOAD/
// DELETE are wValue-only SETs that live on the vendor GET path and return a
// 1-byte status; APPLY_NAME pushes the buffered 32-byte name through the SET
// path.  The vendor LOAD/SAVE/DELETE responses are fire-and-forget
// "accepted": the main loop defers the flash work, so the host confirms a
// LOAD by polling 0x83 and a SAVE/DELETE by re-reading 0x81.

static bool hid_reg_write_preset_slot(const uint8_t *p) {
    if (p[0] >= PRESET_SLOTS) return false;
    _hid_preset_slot = p[0];
    return true;
}

static bool hid_reg_read_preset_slot(uint8_t *out) {
    out[0] = _hid_preset_slot;
    out[1] = out[2] = out[3] = 0;
    return true;
}

// Directory bytes 0..3: occupied slot bitmask (u16 LE), startup_mode,
// default_slot.
static bool hid_reg_read_preset_dir_a(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_PRESET_GET_DIR, 0, 7, &rd, &rl) || rl < 7)
        return false;
    memcpy(out, rd, 4);
    return true;
}

// Directory bytes 4..6: last_active_slot, output_config_mode,
// master_volume_mode.
static bool hid_reg_read_preset_dir_b(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_PRESET_GET_DIR, 0, 7, &rd, &rl) || rl < 7)
        return false;
    out[0] = rd[4];
    out[1] = rd[5];
    out[2] = rd[6];
    out[3] = 0;
    return true;
}

static bool hid_reg_read_preset_active(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_PRESET_GET_ACTIVE, 0, 1, &rd, &rl) || rl < 1)
        return false;
    out[0] = rd[0];
    out[1] = out[2] = out[3] = 0;
    return true;
}

// Return one 4-byte chunk of the selected slot's name.  The chunk index
// (0..7) comes from frame byte 1 and selects the quarter of the 32-byte name.
static bool hid_reg_read_preset_name(uint8_t chunk, uint8_t *out) {
    if (_hid_preset_slot >= PRESET_SLOTS || chunk >= HID_PRESET_CHUNKS)
        return false;
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_PRESET_GET_NAME, _hid_preset_slot,
                          PRESET_NAME_LEN, &rd, &rl) || rl < PRESET_NAME_LEN)
        return false;
    memcpy(out, rd + (uint16_t)chunk * 4, 4);
    return true;
}

// Buffer one 4-byte chunk of the name to write (0x85); the assembled 32-byte
// name is committed to the selected slot via the 0x86 APPLY_NAME action.
static bool hid_reg_write_preset_name(uint8_t chunk, const uint8_t *p) {
    if (_hid_preset_slot >= PRESET_SLOTS || chunk >= HID_PRESET_CHUNKS)
        return false;
    memcpy(_hid_preset_name + (uint16_t)chunk * 4, p, 4);
    return true;
}

static bool hid_reg_write_preset_action(const uint8_t *p) {
    if (_hid_preset_slot >= PRESET_SLOTS)
        return false;
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    switch (p[0]) {
        case 0x01:  // SAVE selected slot (deferred to main loop)
            return hid_get_dispatch(REQ_PRESET_SAVE, _hid_preset_slot, 1, &rd, &rl)
                   && rl >= 1 && rd[0] == PRESET_OK;
        case 0x02:  // LOAD selected slot (deferred; poll 0x83 to confirm)
            return hid_get_dispatch(REQ_PRESET_LOAD, _hid_preset_slot, 1, &rd, &rl)
                   && rl >= 1 && rd[0] == PRESET_OK;
        case 0x03:  // DELETE selected slot (deferred; re-read 0x81 to confirm)
            return hid_get_dispatch(REQ_PRESET_DELETE, _hid_preset_slot, 1, &rd, &rl)
                   && rl >= 1 && rd[0] == PRESET_OK;
        case 0x04:  // APPLY buffered name to the selected slot
            return hid_set_dispatch(REQ_PRESET_SET_NAME, _hid_preset_slot,
                                    (const uint8_t *)_hid_preset_name,
                                    PRESET_NAME_LEN);
        default:
            return false;
    }
}

static bool hid_reg_write_preset_startup(const uint8_t *p) {
    return hid_set_dispatch(REQ_PRESET_SET_STARTUP, 0, p, 2);
}

static bool hid_reg_read_preset_startup(uint8_t *out) {
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (!hid_get_dispatch(REQ_PRESET_GET_STARTUP, 0, 3, &rd, &rl) || rl < 3)
        return false;
    memcpy(out, rd, 3);
    out[3] = 0;
    return true;
}

// --- Frame handling ---
static bool hid_reg_read(uint8_t reg, uint8_t sub, uint8_t *out) {
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
    if (reg >= HID_REG_EFF_BASE && reg <= HID_REG_EFF_LAST)
        return hid_eff_read(reg, out);
    if (reg >= HID_REG_PRESET_SLOT && reg <= HID_REG_PRESET_STARTUP) {
        switch (reg) {
            case HID_REG_PRESET_SLOT:    return hid_reg_read_preset_slot(out);
            case HID_REG_PRESET_DIR_A:   return hid_reg_read_preset_dir_a(out);
            case HID_REG_PRESET_DIR_B:   return hid_reg_read_preset_dir_b(out);
            case HID_REG_PRESET_ACTIVE:  return hid_reg_read_preset_active(out);
            case HID_REG_PRESET_NAME:    return hid_reg_read_preset_name(sub, out);
            case HID_REG_PRESET_STARTUP: return hid_reg_read_preset_startup(out);
            default:                     return false;
        }
    }
    return false;
}

static bool hid_reg_write(uint8_t reg, uint8_t sub, const uint8_t *p) {
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
    if (reg >= HID_REG_EFF_BASE && reg <= HID_REG_EFF_LAST)
        return hid_eff_write(reg, p);
    if (reg >= HID_REG_PRESET_SLOT && reg <= HID_REG_PRESET_STARTUP) {
        switch (reg) {
            case HID_REG_PRESET_SLOT:    return hid_reg_write_preset_slot(p);
            case HID_REG_PRESET_NAME_W:  return hid_reg_write_preset_name(sub, p);
            case HID_REG_PRESET_ACTION:  return hid_reg_write_preset_action(p);
            case HID_REG_PRESET_STARTUP: return hid_reg_write_preset_startup(p);
            default:                     return false;
        }
    }
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
        frame[0], frame[1], 0x00, 0x00,
        frame[4], 0x00,
        HID_STATUS_ERROR, 0x00, 0x00, 0x00
    };
    bool ok = false;

    switch (frame[4]) {
        case HID_CMD_READ:
            ok = hid_reg_read(frame[0], frame[1], &rep[6]);
            break;
        case HID_CMD_WRITE:
            ok = hid_reg_write(frame[0], frame[1], &frame[6]);
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
