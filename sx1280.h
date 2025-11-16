#ifndef _SX1280_H
#define _SX1280_H

#include <linux/types.h>

// Constants.
#define SX1280_FREQ_XOSC_HZ 52000000

// Conversion macros.
#define SX1280_FREQ_HZ_TO_PLL(hz) ((u32) ((((u64) (hz) << 32) / SX1280_FREQ_XOSC_HZ) >> 14))

#pragma pack(push, 1)

enum sx1280_cmd {
  SX1280_CMD_GET_STATUS                 = 0xC0,
  SX1280_CMD_WRITE_REGISTER             = 0x18,
  SX1280_CMD_READ_REGISTER              = 0x19,
  SX1280_CMD_WRITE_BUFFER               = 0x1A,
  SX1280_CMD_READ_BUFFER                = 0x1B,
  SX1280_CMD_SET_SLEEP                  = 0x84,
  SX1280_CMD_SET_STANDBY                = 0x80,
  SX1280_CMD_SET_FS                     = 0xC1,
  SX1280_CMD_SET_TX                     = 0x83,
  SX1280_CMD_SET_RX                     = 0x82,
  SX1280_CMD_SET_RX_DUTY_CYCLE          = 0x94,
  SX1280_CMD_SET_CAD                    = 0xC5,
  SX1280_CMD_SET_TX_CONTINUOUS_WAVE     = 0xD1,
  SX1280_CMD_SET_TX_CONTINUOUS_PREAMBLE = 0xD2,
  SX1280_CMD_SET_PACKET_TYPE            = 0x8A,
  SX1280_CMD_GET_PACKET_TYPE            = 0x03,
  SX1280_CMD_SET_RF_FREQUENCY           = 0x86,
  SX1280_CMD_SET_TX_PARAMS              = 0x8E,
  SX1280_CMD_SET_CAD_PARAMS             = 0x88,
  SX1280_CMD_SET_BUFFER_BASE_ADDRESS    = 0x8F,
  SX1280_CMD_SET_MODULATION_PARAMS      = 0x8B,
  SX1280_CMD_SET_PACKET_PARAMS          = 0x8C,
  SX1280_CMD_GET_RX_BUFFER_STATUS       = 0x17,
  SX1280_CMD_GET_PACKET_STATUS          = 0x1D,
  SX1280_CMD_GET_RSSI_INST              = 0x1F,
  SX1280_CMD_SET_DIO_IRQ_PARAMS         = 0x8D,
  SX1280_CMD_GET_IRQ_STATUS             = 0x15,
  SX1280_CMD_CLR_IRQ_STATUS             = 0x97,
  SX1280_CMD_SET_REGULATOR_MODE         = 0x96,
  SX1280_CMD_SET_SAVE_CONTEXT           = 0xD5,
  SX1280_CMD_SET_AUTO_FS                = 0x9E,
  SX1280_CMD_SET_AUTO_TX                = 0x98,
  SX1280_CMD_SET_LONG_PREAMBLE          = 0x9B,
  SX1280_CMD_SET_UART_SPEED             = 0x9D,
  SX1280_CMD_SET_RANGING_ROLE           = 0xA3,
  SX1280_CMD_SET_ADVANCED_RANGING       = 0x9A
};

enum sx1280_mode {
  SX1280_MODE_GFSK    = 0x00,
  SX1280_MODE_LORA    = 0x01,
  SX1280_MODE_RANGING = 0x02,
  SX1280_MODE_FLRC    = 0x03,
  SX1280_MODE_BLE     = 0x04
};

enum sx1280_ramp_time {
  SX1280_RADIO_RAMP_02_US = 0x00,
  SX1280_RADIO_RAMP_04_US = 0x20,
  SX1280_RADIO_RAMP_06_US = 0x40,
  SX1280_RADIO_RAMP_08_US = 0x60,
  SX1280_RADIO_RAMP_10_US = 0x80,
  SX1280_RADIO_RAMP_12_US = 0xA0,
  SX1280_RADIO_RAMP_16_US = 0xC0,
  SX1280_RADIO_RAMP_20_US = 0xE0
};

enum sx1280_cad_symbol_num {
  SX1280_LORA_CAD_01_SYMBOL  = 0x00,
  SX1280_LORA_CAD_02_SYMBOLS = 0x20,
  SX1280_LORA_CAD_04_SYMBOLS = 0x40,
  SX1280_LORA_CAD_08_SYMBOLS = 0x60,
  SX1280_LORA_CAD_16_SYMBOLS = 0x80
};

enum sx1280_preamble_length {
  SX1280_PREAMBLE_LENGTH_04_BITS = 0x00,
  SX1280_PREAMBLE_LENGTH_08_BITS = 0x10,
  SX1280_PREAMBLE_LENGTH_12_BITS = 0x20,
  SX1280_PREAMBLE_LENGTH_16_BITS = 0x30,
  SX1280_PREAMBLE_LENGTH_20_BITS = 0x40,
  SX1280_PREAMBLE_LENGTH_24_BITS = 0x50,
  SX1280_PREAMBLE_LENGTH_28_BITS = 0x60,
  SX1280_PREAMBLE_LENGTH_32_BITS = 0x70
};

enum sx1280_sync_word_length {
  SX1280_SYNC_WORD_LEN_1_B = 0x00,
  SX1280_SYNC_WORD_LEN_2_B = 0x02,
  SX1280_SYNC_WORD_LEN_3_B = 0x04,
  SX1280_SYNC_WORD_LEN_4_B = 0x06,
  SX1280_SYNC_WORD_LEN_5_B = 0x08
};

enum sx1280_sync_word_combination {
  SX1280_RADIO_SELECT_SYNCWORD_OFF   = 0x00,
  SX1280_RADIO_SELECT_SYNCWORD_1     = 0x10,
  SX1280_RADIO_SELECT_SYNCWORD_2     = 0x20,
  SX1280_RADIO_SELECT_SYNCWORD_1_2   = 0x30,
  SX1280_RADIO_SELECT_SYNCWORD_3     = 0x40,
  SX1280_RADIO_SELECT_SYNCWORD_1_3   = 0x50,
  SX1280_RADIO_SELECT_SYNCWORD_2_3   = 0x60,
  SX1280_RADIO_SELECT_SYNCWORD_1_2_3 = 0x70
};

enum sx1280_gfsk_header_type {
  SX1280_RADIO_PACKET_FIXED_LENGTH    = 0x00,
  SX1280_RADIO_PACKET_VARIABLE_LENGTH = 0x20
};

enum sx1280_gfsk_crc_length {
  SX1280_RADIO_CRC_OFF     = 0x00,
  SX1280_RADIO_CRC_1_BYTE  = 0x10,
  SX1280_RADIO_CRC_2_BYTES = 0x20
};

enum sx1280_whitening {
  SX1280_WHITENING_ENABLE  = 0x00,
  SX1280_WHITENING_DISABLE = 0x08
};

struct sx1280_gfsk_packet_params {
  enum sx1280_preamble_length preamble_length;
  enum sx1280_sync_word_length sync_word_length;
  enum sx1280_sync_word_combination sync_word_match;
  enum sx1280_gfsk_header_type header_type;
  u8 payload_length;
  enum sx1280_gfsk_crc_length crc_length;
  enum sx1280_whitening whitening;
};

enum sx1280_ble_payload_length {
  SX1280_BLE_PAYLOAD_LENGTH_MAX_31_BYTES  = 0x00,
  SX1280_BLE_PAYLOAD_LENGTH_MAX_37_BYTES  = 0x20,
  SX1280_BLE_TX_TEST_MODE                 = 0x40,
  SX1280_BLE_PAYLOAD_LENGTH_MAX_255_BYTES = 0x80
};

enum sx1280_ble_crc_length {
  SX1280_BLE_CRC_OFF = 0x00,
  SX1280_BLE_CRC_3B  = 0x10
};

enum sx1280_ble_test_payload {
  SX1280_BLE_PRBS_9       = 0x00,
  SX1280_BLE_EYELONG_1_0  = 0x04,
  SX1280_BLE_EYESHORT_1_0 = 0x08,
  SX1280_BLE_PRBS_15      = 0x0C,
  SX1280_BLE_ALL_1        = 0x10,
  SX1280_BLE_ALL_0        = 0x14,
  SX1280_BLE_EYELONG_0_1  = 0x18,
  SX1280_BLE_EYESHORT_0_1 = 0x1C
};

struct sx1280_ble_packet_params {
  enum sx1280_ble_payload_length payload_length;
  enum sx1280_ble_crc_length crc_length;
  enum sx1280_ble_test_payload test_payload;
  enum sx1280_whitening whitening;
};

enum sx1280_flrc_sync_word_length {
  SX1280_FLRC_SYNC_WORD_NOSYNC = 0x00,
  SX1280_FLRC_SYNC_WORD_LEN_P32S = 0x04
};

enum sx1280_flrc_crc_length {
  SX1280_FLRC_CRC_OFF = 0x00,
  SX1280_FLRC_CRC_2_BYTE = 0x10,
  SX1280_FLRC_CRC_3_BYTE = 0x20,
  SX1280_FLRC_CRC_4_BYTE = 0x30
};

struct sx1280_flrc_packet_params {
  enum sx1280_preamble_length agc_preamble_length;
  enum sx1280_flrc_sync_word_length sync_word_length;
  enum sx1280_sync_word_combination sync_word_match;
  enum sx1280_gfsk_header_type header_type;
  u8 payload_length;
  enum sx1280_flrc_crc_length crc_length;
  enum sx1280_whitening whitening;
};

enum sx1280_lora_header_type {
  SX1280_EXPLICIT_HEADER = 0x00,
  SX1280_IMPLICIT_HEADER = 0x80,
};

enum sx1280_lora_crc {
  SX1280_LORA_CRC_ENABLE = 0x20,
  SX1280_LORA_CRC_DISABLE = 0x00
};

enum sx1280_lora_iq {
  SX1280_LORA_IQ_INVERTED = 0x00,
  SX1280_LORA_IQ_STD = 0x40
};

struct sx1280_lora_packet_params {
  u8 preamble_length;
  enum sx1280_lora_header_type header_type;
  u8 payload_length;
  enum sx1280_lora_crc crc;
  enum sx1280_lora_iq invert_iq;
};

union sx1280_packet_params {
  struct sx1280_ble_packet_params ble;
  struct sx1280_flrc_packet_params flrc;
  struct sx1280_gfsk_packet_params gfsk;
  struct sx1280_lora_packet_params lora;
  u8 raw[7];
};

enum sx1280_gfsk_bitrate_bandwidth {
  SX1280_GFSK_BLE_BR_2_000_BW_2_4 = 0x04,
  SX1280_GFSK_BLE_BR_1_600_BW_2_4 = 0x28,
  SX1280_GFSK_BLE_BR_1_000_BW_2_4 = 0x4C,
  SX1280_GFSK_BLE_BR_1_000_BW_1_2 = 0x45,
  SX1280_GFSK_BLE_BR_0_800_BW_2_4 = 0x70,
  SX1280_GFSK_BLE_BR_0_800_BW_1_2 = 0x69,
  SX1280_GFSK_BLE_BR_0_500_BW_1_2 = 0x8D,
  SX1280_GFSK_BLE_BR_0_500_BW_0_6 = 0x86,
  SX1280_GFSK_BLE_BR_0_400_BW_1_2 = 0xB1,
  SX1280_GFSK_BLE_BR_0_400_BW_0_6 = 0xAA,
  SX1280_GFSK_BLE_BR_0_250_BW_0_6 = 0xCE,
  SX1280_GFSK_BLE_BR_0_250_BW_0_3 = 0xC7,
  SX1280_GFSK_BLE_BR_0_125_BW_0_3 = 0xEF
};

enum sx1280_gfsk_modulation_index {
  SX1280_MOD_IND_0_35 = 0x00,
  SX1280_MOD_IND_0_50 = 0x01,
  SX1280_MOD_IND_0_75 = 0x02,
  SX1280_MOD_IND_1_00 = 0x03,
  SX1280_MOD_IND_1_25 = 0x04,
  SX1280_MOD_IND_1_50 = 0x05,
  SX1280_MOD_IND_1_75 = 0x06,
  SX1280_MOD_IND_2_00 = 0x07,
  SX1280_MOD_IND_2_25 = 0x08,
  SX1280_MOD_IND_2_50 = 0x09,
  SX1280_MOD_IND_2_75 = 0x0A,
  SX1280_MOD_IND_3_00 = 0x0B,
  SX1280_MOD_IND_3_25 = 0x0C,
  SX1280_MOD_IND_3_50 = 0x0D,
  SX1280_MOD_IND_3_75 = 0x0E,
  SX1280_MOD_IND_4_00 = 0x0F
};

enum sx1280_bandwidth_time {
  SX1280_BT_OFF = 0x00,
  SX1280_BT_1_0 = 0x10,
  SX1280_BT_0_5 = 0x20
};

struct sx1280_gfsk_modulation_params {
  enum sx1280_gfsk_bitrate_bandwidth bitrate_bandwidth;
  enum sx1280_gfsk_modulation_index modulation_index;
  enum sx1280_bandwidth_time bandwidth_time;
};

enum sx1280_flrc_bitrate_bandwidth {
  SX1280_FLRC_BR_1_300_BW_1_2 = 0x45,
  SX1280_FLRC_BR_1_000_BW_1_2 = 0x69,
  SX1280_FLRC_BR_0_650_BW_0_6 = 0x86,
  SX1280_FLRC_BR_0_520_BW_0_6 = 0xAA,
  SX1280_FLRC_BR_0_325_BW_0_3 = 0xC7,
  SX1280_FLRC_BR_0_260_BW_0_3 = 0xEB
};

enum sx1280_flrc_coding_rate {
  SX1280_FLRC_CR_1_2 = 0x00,
  SX1280_FLRC_CR_3_4 = 0x02,
  SX1280_FLRC_CR_1_1 = 0x04
};

struct sx1280_flrc_modulation_params {
  enum sx1280_flrc_bitrate_bandwidth bitrate_bandwidth;
  enum sx1280_flrc_coding_rate coding_rate;
  enum sx1280_bandwidth_time bandwidth_time;
};

enum sx1280_lora_spreading {
  SX1280_LORA_SF_5  = 0x50,
  SX1280_LORA_SF_6  = 0x60,
  SX1280_LORA_SF_7  = 0x70,
  SX1280_LORA_SF_8  = 0x80,
  SX1280_LORA_SF_9  = 0x90,
  SX1280_LORA_SF_10 = 0xA0,
  SX1280_LORA_SF_11 = 0xB0,
  SX1280_LORA_SF_12 = 0xC0
};

enum sx1280_lora_bandwidth {
  SX1280_LORA_BW_1600 = 0x0A,
  SX1280_LORA_BW_800  = 0x18,
  SX1280_LORA_BW_400  = 0x26,
  SX1280_LORA_BW_200  = 0x34
};

enum sx1280_lora_coding_rate {
  SX1280_LORA_CR_4_5    = 0x01,
  SX1280_LORA_CR_4_6    = 0x02,
  SX1280_LORA_CR_4_7    = 0x03,
  SX1280_LORA_CR_4_8    = 0x04,
  SX1280_LORA_CR_LI_4_5 = 0x05,
  SX1280_LORA_CR_LI_4_6 = 0x06,
  SX1280_LORA_CR_LI_4_8 = 0x07
};

struct sx1280_lora_modulation_params {
  enum sx1280_lora_spreading spreading;
  enum sx1280_lora_bandwidth bandwidth;
  enum sx1280_lora_coding_rate coding_rate;
};

union sx1280_modulation_params {
  struct sx1280_gfsk_modulation_params gfsk;
  struct sx1280_flrc_modulation_params flrc;
  struct sx1280_lora_modulation_params lora;
  u8 raw[3];
};

struct sx1280_packet_status_ble_gfsk_flrc {
  u8 rfu;
  u8 rssi_sync;
  u8 errors;
  u8 status;
  u8 sync;
};

struct sx1280_packet_status_lora {
  u8 rssi_sync;
  u8 snr_pkt;
};

union sx1280_packet_status {
  struct sx1280_packet_status_ble_gfsk_flrc ble_gfsk_flrc;
  struct sx1280_packet_status_lora lora;
  u8 raw[5];
};

#define SX1280_PREAMBLE_BITS(bits) (((bits) - 4) << 2)
#define SX1280_PREAMBLE_BITS_VALID(bits) ((bits) >= 4 && (bits) <= 32 && (bits) % 4 == 0)

#define SX1280_SYNC_WORD_BITS(bytes) (((bytes) - 1) * 2)
#define SX1280_SYNC_WORD_BITS_VALID(bytes) ((bytes) <= 5)

enum sx1280_standby {
  SX1280_STDBY_RC   = 0,
  SX1280_STDBY_XOSC = 1
};

#define SX1280_IRQ_TX_DONE (1 << 0)
#define SX1280_IRQ_RX_DONE (1 << 1)
#define SX1280_IRQ_SYNC_WORD_VALID (1 << 2)
#define SX1280_IRQ_SYNC_WORD_ERROR (1 << 3)
#define SX1280_IRQ_HEADER_VALID (1 << 4)
#define SX1280_IRQ_HEADER_ERROR (1 << 5)
#define SX1280_IRQ_CRC_ERROR (1 << 6)
#define SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE (1 << 7)
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARD (1 << 8)
#define SX1280_IRQ_RANGING_MASTER_RESULT_VALID (1 << 9)
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT (1 << 10)
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID (1 << 11)
#define SX1280_IRQ_CAD_DONE (1 << 12)
#define SX1280_IRQ_CAD_DETECTED (1 << 13)
#define SX1280_IRQ_RX_TX_TIMEOUT (1 << 14)
#define SX1280_IRQ_PREAMBLE_DETECTED (1 << 15)
#define SX1280_IRQ_ADVANCED_RANGING_DONE (1 << 15)

#define SX1280_STATUS_CIRCUIT_MODE(status) ((status) >> 5 & 0x7)
#define SX1280_STATUS_COMMAND_STATUS(status) ((status) >> 2 & 0x7)

#define SX1280_CIRCUIT_MODE_STDBY_RC   0x2
#define SX1280_CIRCUIT_MODE_STDBY_XOSC 0x3
#define SX1280_CIRCUIT_MODE_FS         0x4
#define SX1280_CIRCUIT_MODE_RX         0x5
#define SX1280_CIRCUIT_MODE_TX         0x6

#define SX1280_COMMAND_STATUS_TX_PROCESSED     0x1
#define SX1280_COMMAND_STATUS_DATA_AVAILABLE   0x2
#define SX1280_COMMAND_STATUS_TIMEOUT          0x3
#define SX1280_COMMAND_STATUS_PROCESSING_ERROR 0x4
#define SX1280_COMMAND_STATUS_EXEC_FAILURE     0x5
#define SX1280_COMMAND_STATUS_TX_DONE          0x6

// Register reading macros.
#define BITMASK(a, b) (((1 << ((b) - (a) + 1)) - 1) << (7 - (b)))
#define MASK(value, a, b) (((value) & BITMASK(a, b)) >> (7 - (b)))
#define PLACE(value, a, b) ((value) << (7 - (b)))

/**
 * SPI registers.
 */

enum sx1280_reg {
  SX1280_REG_FIRMWARE_VERSION               = 0x153,
  SX1280_REG_RX_GAIN                        = 0x891,
  SX1280_REG_MANUAL_GAIN_SETTING            = 0x895,
  SX1280_REG_LNA_GAIN_VALUE                 = 0x89E,
  SX1280_REG_LNA_GAIN_CONTROL               = 0x89F,
  SX1280_REG_SYNCH_PEAK_ATTENUATION         = 0x8C2,
  SX1280_REG_PAYLOAD_LENGTH                 = 0x901,
  SX1280_REG_LORA_HEADER_MODE               = 0x903,
  SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_3 = 0x912,
  SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_2 = 0x913,
  SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_1 = 0x914,
  SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_0 = 0x915,
  SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_3  = 0x916,
  SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_2  = 0x917,
  SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_1  = 0x918,
  SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_0  = 0x919,
  SX1280_REG_RANGING_FILTER_WINDOW_SIZE     = 0x91E,
  SX1280_REG_RESET_RANGING_FILTER           = 0x923,
  SX1280_REG_RANGING_RESULT_MUX             = 0x924,
  SX1280_REG_SF_ADDITIONAL_CONFIGURATION    = 0x925,
  SX1280_REG_RANGING_CALIBRATION_BYTE_2     = 0x92B,
  SX1280_REG_RANGING_CALIBRATION_BYTE_1     = 0x92C,
  SX1280_REG_RANGING_CALIBRATION_BYTE_0     = 0x92D,
  SX1280_REG_RANGING_ID_CHECK_LENGTH        = 0x931,
  SX1280_REG_FREQUENCY_ERROR_CORRECTION     = 0x93C,
  SX1280_REG_CAD_DET_PEAK                   = 0x942,
  SX1280_REG_LORA_SYNC_WORD_1               = 0x944,
  SX1280_REG_LORA_SYNC_WORD_2               = 0x945,
  SX1280_REG_HEADER_CRC                     = 0x954,
  SX1280_REG_CODING_RATE                    = 0x950,
  SX1280_REG_FEI_BYTE_2                     = 0x954,
  SX1280_REG_FEI_BYTE_1                     = 0x955,
  SX1280_REG_FEI_BYTE_0                     = 0x956,
  SX1280_REG_RANGING_RESULT_BYTE_2          = 0x961,
  SX1280_REG_RANGING_RESULT_BYTE_1          = 0x962,
  SX1280_REG_RANGING_RESULT_BYTE_0          = 0x963,
  SX1280_REG_RANGING_RSSI                   = 0x964,
  SX1280_REG_FREEZE_RANGING_RESULT          = 0x97F,
  SX1280_REG_PACKET_PREAMBLE_SETTINGS       = 0x9C1,
  SX1280_REG_WHITENING_INITIAL_VALUE        = 0x9C5,
  SX1280_REG_CRC_POLYNOMIAL_DEFINITION_MSB  = 0x9C6,
  SX1280_REG_CRC_POLYNOMIAL_DEFINITION_LSB  = 0x9C7,
  SX1280_REG_CRC_POLYNOMIAL_SEED_BYTE_2     = 0x9C7,
  SX1280_REG_CRC_POLYNOMIAL_SEED_BYTE_1     = 0x9C8,
  SX1280_REG_CRC_POLYNOMIAL_SEED_BYTE_0     = 0x9C9,
  SX1280_REG_CRC_MSB_INITIAL_VALUE          = 0x9C8,
  SX1280_REG_SYNC_ADDRESS_1_BYTE_4          = 0x9CE,
  SX1280_REG_SYNC_ADDRESS_1_BYTE_3          = 0x9CF,
  SX1280_REG_SYNC_ADDRESS_1_BYTE_2          = 0x9D0,
  SX1280_REG_SYNC_ADDRESS_1_BYTE_1          = 0x9D1,
  SX1280_REG_SYNC_ADDRESS_1_BYTE_0          = 0x9D2,
  SX1280_REG_SYNC_ADDRESS_2_BYTE_4          = 0x9D3,
  SX1280_REG_SYNC_ADDRESS_2_BYTE_3          = 0x9D4,
  SX1280_REG_SYNC_ADDRESS_2_BYTE_2          = 0x9D5,
  SX1280_REG_SYNC_ADDRESS_2_BYTE_1          = 0x9D6,
  SX1280_REG_SYNC_ADDRESS_2_BYTE_0          = 0x9D7,
  SX1280_REG_SYNC_ADDRESS_3_BYTE_4          = 0x9D8,
  SX1280_REG_SYNC_ADDRESS_3_BYTE_3          = 0x9D9,
  SX1280_REG_SYNC_ADDRESS_3_BYTE_2          = 0x9DA,
  SX1280_REG_SYNC_ADDRESS_3_BYTE_1          = 0x9DB,
  SX1280_REG_SYNC_ADDRESS_3_BYTE_0          = 0x9DC
};

#pragma pack(pop)

// Defaults.
#define SX1280_DEFAULT_RF_FREQ_HZ 2400000
#define SX1280_DEFAULT_STARTUP_TIMEOUT_US 2000
#define SX1280_DEFAULT_CRC_LENGTH SX1280_RADIO_CRC_2_BYTES

struct sx1280_ble_params {
  struct sx1280_gfsk_modulation_params modulation;
  struct sx1280_ble_packet_params packet;

  u32 access_address;
  u16 crc_seed;
};

struct sx1280_gfsk_params {
  struct sx1280_gfsk_modulation_params modulation;
  struct sx1280_gfsk_packet_params packet;

  u16 crc_seed;
  u16 crc_polynomial;
};

struct sx1280_flrc_params {
  struct sx1280_flrc_modulation_params modulation;
  struct sx1280_flrc_packet_params packet;

  u16 crc_seed;
};

struct sx1280_lora_params {
  struct sx1280_lora_modulation_params modulation;
  struct sx1280_lora_packet_params packet;
};

struct sx1280_ranging_params {
  struct sx1280_lora_modulation_params modulation;
  struct sx1280_lora_packet_params packet;

  u32 slave_address;
  u8 register_address_bit;
  u32 master_address;
  u16 calibration;
  u8 role;
};

enum sx1280_period_base {
  /* 15.625 us */
  SX1280_PERIOD_BASE_15_625_US = 0x00,

  /* 62.5 us */
  SX1280_PERIOD_BASE_62_500_US = 0x01,

  /* 1 ms */
  SX1280_PERIOD_BASE_1_MS = 0x02,

  /* 4 ms */
  SX1280_PERIOD_BASE_4_MS = 0x03
};

/**
 * struct sx1280_platform_data - Platform data for the SX1280 driver.
 *
 * @rf_freq - The frequency of the RF signal (in PLL steps, not Hertz).
 * @period_base - The base step of the timeout timer, defining the granularity.
 * @period_base_count - The number of time steps of the period base before a
 *                      timeout is registered.
 * @startup_timeout_us - The number of microseconds until a startup timeout is
 *                       registered.
 * @mode - The packet mode of the chip.
 * @busy_gpio - The legacy GPIO number corresponding to the busy pin.
 * @dio_gpios - The legacy GPIO numbers corresponding to the DIO pins.
 * @ble - Bluetooth Low Energy configuration.
 * @gfsk - Gaussian Frequency Shift Keying configuration.
 * @flrc - Fast Long Range Communication configuration.
 * @lora - LoRa configuration.
 * @ranging - Distance ranging configuration.
 */
struct sx1280_platform_data {
  enum sx1280_mode mode;
  enum sx1280_period_base period_base;
  u16 period_base_count;
  u8 power;
  enum sx1280_ramp_time ramp_time;
  u32 rf_freq;
  u32 startup_timeout_us;

  unsigned int busy_gpio;
  int dio_gpios[3];
  unsigned int reset_gpio;

  struct sx1280_ble_params ble;
  struct sx1280_gfsk_params gfsk;
  struct sx1280_flrc_params flrc;
  struct sx1280_lora_params lora;
  struct sx1280_ranging_params ranging;
};

#endif /* _SX1280_H */
