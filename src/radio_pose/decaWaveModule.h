#ifndef DECA_WAVE_MODULE_H_
#define DECA_WAVE_MODULE_H_

#include "decadriver/deca_device_api.h"
#include "rodos.h"
#include "stdint.h"

/* Index to access some of the fields in the frames involved in the process. */
// all messages
constexpr uint8_t ALL_MSG_SN_IDX = 2;         // sequence number
constexpr uint8_t ALL_MSG_SOURCE_ID_IDX = 5;  // source id
constexpr uint8_t ALL_MSG_DEST_ID_IDX = 6;    // destination id
constexpr uint8_t ALL_MSG_TYPE_IDX = 7;       // message type
constexpr uint8_t ALL_MSG_DEFAULT_LENGTH = 10;

// final message
constexpr uint8_t FINAL_MSG_TS_LEN = 5;                                                     // Number of bytes of a timestamp in the message
constexpr uint8_t FINAL_MSG_POLL_TX_TS_IDX = 8;                                             // poll_tx timestamp
constexpr uint8_t FINAL_MSG_RESP_RX_TS_IDX = FINAL_MSG_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN;   // resp_rx timestamp
constexpr uint8_t FINAL_MSG_FINAL_TX_TS_IDX = FINAL_MSG_RESP_RX_TS_IDX + FINAL_MSG_TS_LEN;  // final_tx timestamp
// distance message
constexpr uint8_t DISTANCE_MSG_DISTANCE_IDX = 8;  // distance

/* Message type identifiers. */
constexpr uint8_t MSG_TYPE_POLL = 0x20;
constexpr uint8_t MSG_TYPE_RESP = 0x21;
constexpr uint8_t MSG_TYPE_FINAL = 0x22;
constexpr uint8_t MSG_TYPE_DISTANCE = 0x23;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
constexpr uint8_t RX_BUF_LEN = 8 + 3 * FINAL_MSG_TS_LEN + 2;
static uint8_t rx_buffer[RX_BUF_LEN];

/* conversion factors from seconds to deca device time units and vice versa*/
constexpr double DWT_TO_SECONDS = 1.0 / 499.2e6 / 128.0;  // 15.65e-12 s
constexpr double SECONDS_TO_DWT = 499.2e6 * 128.0;        // 63.8976e9 device time units (dtu)

// This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature.
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
// This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function.
constexpr float RESP_RX_TO_FINAL_TX_DLY_S = 3.0e-3;
constexpr uint64_t RESP_RX_TO_FINAL_TX_DLY_DTU_U64 = (uint64_t)(RESP_RX_TO_FINAL_TX_DLY_S * SECONDS_TO_DWT);

// receive response timeout (not quite microseconds but 1.026 microseconds)
#define RESP_RX_TIMEOUT_UUS 60000

// Get temperature in °C
float getTemperature_C();

// speed of light in air [m/s]
constexpr double SPEED_OF_LIGHT = 299702547.2;

// antenna delays
constexpr uint16_t TX_ANT_DLY = 16436;
constexpr uint16_t RX_ANT_DLY = 16436;

/* Declaration of static functions. */
uint64_t get_tx_timestamp_u64(uint8_t spi_num);
uint64_t get_rx_timestamp_u64(uint8_t spi_num);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
// void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void final_msg_get_ts(const uint8_t *ts_field, uint64_t *ts);

uint64_t get_systemtime_deca(uint8_t spi_num);
double get_systemtime_seconds();

void init_decaWaveModule(uint8_t spi_num, dwt_config_t *conf);
void uwb_write(uint8_t spi_num, uint8_t sourceId, uint8_t destId, uint8_t *msg, int sizeOfMsg, uint8_t mode = DWT_START_TX_IMMEDIATE);
bool uwb_read(uint8_t spi_num, uint8_t *rx_buf);

#endif /* DECA_WAVE_MODULE_H_ */
