/*
 * hid_control.h — AD1-style USB HID control interface for DSPi
 *
 * Bridges the KTMicro AD1 10-byte control-frame protocol (Report ID 0x4B)
 * onto DSPi's transport-neutral vendor command surface
 * (vendor_dispatch_get/set, CTRL_SOURCE_HID).  The register file and frame
 * layout follow kiwi_ears_ad1_protocol_spec.md.
 */

#ifndef HID_CONTROL_H
#define HID_CONTROL_H

#include "tusb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Process one 10-byte AD1 frame (CMD 0x52 Read / 0x57 Write / 0x53 Commit)
// and queue the 10-byte reply for the HID interrupt IN endpoint.  Call from
// tud_hid_set_report_cb only (main-loop / USB-task context).
bool hid_control_process_frame(const uint8_t *frame);

// Transmit any queued replies whose EP-IN slot is idle.  Call every main-loop
// iteration (after tud_task()) so back-to-back WRITE+READ responses are not
// dropped when the previous report is still in flight.
void hid_control_tick(void);

#ifdef __cplusplus
}
#endif

#endif // HID_CONTROL_H
