#ifndef DEFINES_RESP_H_
#define DEFINES_RESP_H_

#include "decaWaveModule.h"
#include "decadriver/deca_device_api.h"

/* Default communication configuration. */
static dwt_config_t config = {
    4,               /* Channel number */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_2048,   /* Preamble length. */
    DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
    17,              /* TX preamble code. Used in TX only. */
    17,              /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (2049 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/**
 * Message structure:
 * 	- byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *	- byte 2: sequence number, incremented for each new frame.
 *	- byte 3/4: PAN ID (0xDECA).
 *  - byte 5: source address
 *	- byte 6: destination address
 *  - byte 7: message type
 * 	- variable Payload data
 * 	- last 2 byte: CRC (set by the DW1000)
 */

/* Double-sided two-way ranging (DW-TWR) messages */
static uint8_t tx_poll_msg[10] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'D', MSG_TYPE_POLL, 0, 0};
static uint8_t tx_resp_msg[10] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'D', MSG_TYPE_RESP, 0, 0};
static uint8_t tx_final_msg[25] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'D', MSG_TYPE_FINAL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t tx_distance_msg[14] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'D', MSG_TYPE_DISTANCE, 0, 0, 0, 0, 0, 0};

#endif /* DEFINES_RESP_H_ */
