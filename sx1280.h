#ifndef _SX1280_H
#define _SX1280_H

#include <linux/types.h>

// Constants.
#define SX1280_FREQ_XOSC_HZ 52000000

// Conversion macros.
#define SX1280_FREQ_HZ_TO_PLL(hz) ((u32) ((((u64) (hz) << 32) / SX1280_FREQ_XOSC_HZ) >> 14))

#define SX1280_CMD_GET_STATUS                 0xC0
#define SX1280_CMD_WRITE_REGISTER             0x18
#define SX1280_CMD_READ_REGISTER              0x19
#define SX1280_CMD_WRITE_BUFFER               0x1A
#define SX1280_CMD_READ_BUFFER                0x1B
#define SX1280_CMD_SET_SLEEP                  0x84
#define SX1280_CMD_SET_STANDBY                0x80
#define SX1280_CMD_SET_FS                     0xC1
#define SX1280_CMD_SET_TX                     0x83
#define SX1280_CMD_SET_RX                     0x82
#define SX1280_CMD_SET_RX_DUTY_CYCLE          0x94
#define SX1280_CMD_SET_CAD                    0xC5
#define SX1280_CMD_SET_TX_CONTINUOUS_WAVE     0xD1
#define SX1280_CMD_SET_TX_CONTINUOUS_PREAMBLE 0xD2
#define SX1280_CMD_SET_PACKET_TYPE            0x8A
#define SX1280_CMD_GET_PACKET_TYPE            0x03
#define SX1280_CMD_SET_RF_FREQUENCY           0x86
#define SX1280_CMD_SET_TX_PARAMS              0x8E
#define SX1280_CMD_SET_CAD_PARAMS             0x88
#define SX1280_CMD_SET_BUFFER_BASE_ADDRESS    0x8F
#define SX1280_CMD_SET_MODULATION_PARAMS      0x8B
#define SX1280_CMD_SET_PACKET_PARAMS          0x8C
#define SX1280_CMD_GET_RX_BUFFER_STATUS       0x17
#define SX1280_CMD_GET_PACKET_STATUS          0x1D
#define SX1280_CMD_GET_RSSI_INST              0x1F
#define SX1280_CMD_SET_DIO_IRQ_PARAMS         0x8D
#define SX1280_CMD_GET_IRQ_STATUS             0x15
#define SX1280_CMD_CLR_IRQ_STATUS             0x97
#define SX1280_CMD_SET_REGULATOR_MODE         0x96
#define SX1280_CMD_SET_SAVE_CONTEXT           0xD5
#define SX1280_CMD_SET_AUTO_FS                0x9E
#define SX1280_CMD_SET_AUTO_TX                0x98
#define SX1280_CMD_SET_LONG_PREAMBLE          0x9B
#define SX1280_CMD_SET_UART_SPEED             0x9D
#define SX1280_CMD_SET_RANGING_ROLE           0xA3
#define SX1280_CMD_SET_ADVANCED_RANGING       0x9A

enum sx1280_mode {
  SX1280_MODE_GFSK    = 0x00,
  SX1280_MODE_LORA    = 0x01,
  SX1280_MODE_RANGING = 0x02,
  SX1280_MODE_FLRC    = 0x03,
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

#define SX1280_PREAMBLE_LENGTH_04_BITS 0x00
#define SX1280_PREAMBLE_LENGTH_08_BITS 0x10
#define SX1280_PREAMBLE_LENGTH_12_BITS 0x20
#define SX1280_PREAMBLE_LENGTH_16_BITS 0x30
#define SX1280_PREAMBLE_LENGTH_20_BITS 0x40
#define SX1280_PREAMBLE_LENGTH_24_BITS 0x50
#define SX1280_PREAMBLE_LENGTH_28_BITS 0x60
#define SX1280_PREAMBLE_LENGTH_32_BITS 0x70

#define SX1280_SYNC_WORD_LEN_1_B 0x00
#define SX1280_SYNC_WORD_LEN_2_B 0x02
#define SX1280_SYNC_WORD_LEN_3_B 0x04
#define SX1280_SYNC_WORD_LEN_4_B 0x06
#define SX1280_SYNC_WORD_LEN_5_B 0x08

#define SX1280_RADIO_SELECT_SYNCWORD_OFF   0x00
#define SX1280_RADIO_SELECT_SYNCWORD_1     0x10
#define SX1280_RADIO_SELECT_SYNCWORD_2     0x20
#define SX1280_RADIO_SELECT_SYNCWORD_1_2   0x30
#define SX1280_RADIO_SELECT_SYNCWORD_3     0x40
#define SX1280_RADIO_SELECT_SYNCWORD_1_3   0x50
#define SX1280_RADIO_SELECT_SYNCWORD_2_3   0x60
#define SX1280_RADIO_SELECT_SYNCWORD_1_2_3 0x70

#define SX1280_RADIO_PACKET_FIXED_LENGTH    0x00
#define SX1280_RADIO_PACKET_VARIABLE_LENGTH 0x20

#define SX1280_RADIO_CRC_OFF     0x00
#define SX1280_RADIO_CRC_1_BYTE  0x10
#define SX1280_RADIO_CRC_2_BYTES 0x20

#define SX1280_WHITENING_ENABLE  0x00
#define SX1280_WHITENING_DISABLE 0x08

struct sx1280_gfsk_packet_params {
  u8 preamble_length;
  u8 sync_word_length;
  u8 sync_word_match;
  u8 header_type;
  u8 payload_length;
  u8 crc_length;
  u8 whitening;
} __packed;

#define SX1280_FLRC_SYNC_WORD_NOSYNC   0x00
#define SX1280_FLRC_SYNC_WORD_LEN_P32S 0x04

#define SX1280_FLRC_CRC_OFF    0x00
#define SX1280_FLRC_CRC_2_BYTE 0x10
#define SX1280_FLRC_CRC_3_BYTE 0x20
#define SX1280_FLRC_CRC_4_BYTE 0x30

struct sx1280_flrc_packet_params {
  u8 agc_preamble_length;
  u8 sync_word_length;
  u8 sync_word_match;
  u8 header_type;
  u8 payload_length;
  u8 crc_length;
  u8 whitening;
} __packed;

#define SX1280_EXPLICIT_HEADER 0x00
#define SX1280_IMPLICIT_HEADER 0x80

#define SX1280_LORA_CRC_ENABLE  0x20
#define SX1280_LORA_CRC_DISABLE 0x00

#define SX1280_LORA_IQ_INVERTED 0x00
#define SX1280_LORA_IQ_STD      0x40

struct sx1280_lora_packet_params {
  u8 preamble_length;
  u8 header_type;
  u8 payload_length;
  u8 crc;
  u8 invert_iq;
} __packed;

union sx1280_packet_params {
  struct sx1280_flrc_packet_params flrc;
  struct sx1280_gfsk_packet_params gfsk;
  struct sx1280_lora_packet_params lora;
  u8 raw[7];
};

#define SX1280_GFSK_BR_2_000_BW_2_4 0x04
#define SX1280_GFSK_BR_1_600_BW_2_4 0x28
#define SX1280_GFSK_BR_1_000_BW_2_4 0x4C
#define SX1280_GFSK_BR_1_000_BW_1_2 0x45
#define SX1280_GFSK_BR_0_800_BW_2_4 0x70
#define SX1280_GFSK_BR_0_800_BW_1_2 0x69
#define SX1280_GFSK_BR_0_500_BW_1_2 0x8D
#define SX1280_GFSK_BR_0_500_BW_0_6 0x86
#define SX1280_GFSK_BR_0_400_BW_1_2 0xB1
#define SX1280_GFSK_BR_0_400_BW_0_6 0xAA
#define SX1280_GFSK_BR_0_250_BW_0_6 0xCE
#define SX1280_GFSK_BR_0_250_BW_0_3 0xC7
#define SX1280_GFSK_BR_0_125_BW_0_3 0xEF

#define SX1280_MOD_IND_0_35 0x00
#define SX1280_MOD_IND_0_50 0x01
#define SX1280_MOD_IND_0_75 0x02
#define SX1280_MOD_IND_1_00 0x03
#define SX1280_MOD_IND_1_25 0x04
#define SX1280_MOD_IND_1_50 0x05
#define SX1280_MOD_IND_1_75 0x06
#define SX1280_MOD_IND_2_00 0x07
#define SX1280_MOD_IND_2_25 0x08
#define SX1280_MOD_IND_2_50 0x09
#define SX1280_MOD_IND_2_75 0x0A
#define SX1280_MOD_IND_3_00 0x0B
#define SX1280_MOD_IND_3_25 0x0C
#define SX1280_MOD_IND_3_50 0x0D
#define SX1280_MOD_IND_3_75 0x0E
#define SX1280_MOD_IND_4_00 0x0F

#define SX1280_BT_OFF 0x00
#define SX1280_BT_1_0 0x10
#define SX1280_BT_0_5 0x20

struct sx1280_gfsk_modulation_params {
  u8 bitrate_bandwidth;
  u8 modulation_index;
  u8 bandwidth_time;
} __packed;

#define SX1280_FLRC_BR_1_300_BW_1_2 0x45
#define SX1280_FLRC_BR_1_000_BW_1_2 0x69
#define SX1280_FLRC_BR_0_650_BW_0_6 0x86
#define SX1280_FLRC_BR_0_520_BW_0_6 0xAA
#define SX1280_FLRC_BR_0_325_BW_0_3 0xC7
#define SX1280_FLRC_BR_0_260_BW_0_3 0xEB

#define SX1280_FLRC_CR_1_2 0x00
#define SX1280_FLRC_CR_3_4 0x02
#define SX1280_FLRC_CR_1_1 0x04

struct sx1280_flrc_modulation_params {
  u8 bitrate_bandwidth;
  u8 coding_rate;
  u8 bandwidth_time;
} __packed;

#define SX1280_LORA_SF_5  0x50
#define SX1280_LORA_SF_6  0x60
#define SX1280_LORA_SF_7  0x70
#define SX1280_LORA_SF_8  0x80
#define SX1280_LORA_SF_9  0x90
#define SX1280_LORA_SF_10 0xA0
#define SX1280_LORA_SF_11 0xB0
#define SX1280_LORA_SF_12 0xC0

#define SX1280_LORA_BW_1600 0x0A
#define SX1280_LORA_BW_800  0x18
#define SX1280_LORA_BW_400  0x26
#define SX1280_LORA_BW_200  0x34

#define SX1280_LORA_CR_4_5    0x01
#define SX1280_LORA_CR_4_6    0x02
#define SX1280_LORA_CR_4_7    0x03
#define SX1280_LORA_CR_4_8    0x04
#define SX1280_LORA_CR_LI_4_5 0x05
#define SX1280_LORA_CR_LI_4_6 0x06
#define SX1280_LORA_CR_LI_4_8 0x07

struct sx1280_lora_modulation_params {
  u8 spreading;
  u8 bandwidth;
  u8 coding_rate;
} __packed;

union sx1280_modulation_params {
  struct sx1280_gfsk_modulation_params gfsk;
  struct sx1280_flrc_modulation_params flrc;
  struct sx1280_lora_modulation_params lora;
  u8 raw[3];
};

struct sx1280_packet_status_gfsk_flrc {
  u8 rfu;
  u8 rssi_sync;
  u8 errors;
  u8 status;
  u8 sync;
} __packed;

struct sx1280_packet_status_lora {
  u8 rssi_sync;
  u8 snr;
} __packed;

union sx1280_packet_status {
  struct sx1280_packet_status_gfsk_flrc gfsk_flrc;
  struct sx1280_packet_status_lora lora;
  u8 raw[5];
};

#define SX1280_PREAMBLE_BITS(bits) (((bits) - 4) << 2)
#define SX1280_PREAMBLE_BITS_VALID(bits) ((bits) >= 4 && (bits) <= 32 && (bits) % 4 == 0)

#define SX1280_SYNC_WORD_BITS(bytes) (((bytes) - 1) * 2)
#define SX1280_SYNC_WORD_BITS_VALID(bytes) ((bytes) <= 5)

#define SX1280_STDBY_RC   0
#define SX1280_STDBY_XOSC 1

#define SX1280_IRQ_TX_DONE BIT(0)
#define SX1280_IRQ_RX_DONE BIT(1)
#define SX1280_IRQ_SYNC_WORD_VALID BIT(2)
#define SX1280_IRQ_SYNC_WORD_ERROR BIT(3)
#define SX1280_IRQ_HEADER_VALID BIT(4)
#define SX1280_IRQ_HEADER_ERROR BIT(5)
#define SX1280_IRQ_CRC_ERROR BIT(6)
#define SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE BIT(7)
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARD BIT(8)
#define SX1280_IRQ_RANGING_MASTER_RESULT_VALID BIT(9)
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT BIT(10)
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID BIT(11)
#define SX1280_IRQ_CAD_DONE BIT(12)
#define SX1280_IRQ_CAD_DETECTED BIT(13)
#define SX1280_IRQ_RX_TX_TIMEOUT BIT(14)
#define SX1280_IRQ_PREAMBLE_DETECTED BIT(15)
#define SX1280_IRQ_ADVANCED_RANGING_DONE BIT(15)

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

/* Registers */
#define SX1280_REG_FIRMWARE_VERSION               0x153
#define SX1280_REG_RX_GAIN                        0x891
#define SX1280_REG_MANUAL_GAIN_SETTING            0x895
#define SX1280_REG_LNA_GAIN_VALUE                 0x89E
#define SX1280_REG_LNA_GAIN_CONTROL               0x89F
#define SX1280_REG_SYNCH_PEAK_ATTENUATION         0x8C2
#define SX1280_REG_PAYLOAD_LENGTH                 0x901
#define SX1280_REG_LORA_HEADER_MODE               0x903
#define SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_3 0x912
#define SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_2 0x913
#define SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_1 0x914
#define SX1280_REG_RANGING_REQUEST_ADDRESS_BYTE_0 0x915
#define SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_3  0x916
#define SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_2  0x917
#define SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_1  0x918
#define SX1280_REG_RANGING_DEVICE_ADDRESS_BYTE_0  0x919
#define SX1280_REG_RANGING_FILTER_WINDOW_SIZE     0x91E
#define SX1280_REG_RESET_RANGING_FILTER           0x923
#define SX1280_REG_RANGING_RESULT_MUX             0x924
#define SX1280_REG_SF_ADDITIONAL_CONFIGURATION    0x925
#define SX1280_REG_RANGING_CALIBRATION_BYTE_2     0x92B
#define SX1280_REG_RANGING_CALIBRATION_BYTE_1     0x92C
#define SX1280_REG_RANGING_CALIBRATION_BYTE_0     0x92D
#define SX1280_REG_RANGING_ID_CHECK_LENGTH        0x931
#define SX1280_REG_FREQUENCY_ERROR_CORRECTION     0x93C
#define SX1280_REG_CAD_DET_PEAK                   0x942
#define SX1280_REG_LORA_SYNC_WORD_1               0x944
#define SX1280_REG_LORA_SYNC_WORD_2               0x945
#define SX1280_REG_HEADER_CRC                     0x954
#define SX1280_REG_CODING_RATE                    0x950
#define SX1280_REG_FEI_BYTE_2                     0x954
#define SX1280_REG_FEI_BYTE_1                     0x955
#define SX1280_REG_FEI_BYTE_0                     0x956
#define SX1280_REG_RANGING_RESULT_BYTE_2          0x961
#define SX1280_REG_RANGING_RESULT_BYTE_1          0x962
#define SX1280_REG_RANGING_RESULT_BYTE_0          0x963
#define SX1280_REG_RANGING_RSSI                   0x964
#define SX1280_REG_FREEZE_RANGING_RESULT          0x97F
#define SX1280_REG_PACKET_PREAMBLE_SETTINGS       0x9C1
#define SX1280_REG_WHITENING_INITIAL_VALUE        0x9C5
#define SX1280_REG_CRC_POLYNOMIAL_DEFINITION_MSB  0x9C6
#define SX1280_REG_CRC_POLYNOMIAL_DEFINITION_LSB  0x9C7
#define SX1280_REG_CRC_POLYNOMIAL_SEED_BYTE_2     0x9C7
#define SX1280_REG_CRC_POLYNOMIAL_SEED_BYTE_1     0x9C8
#define SX1280_REG_CRC_POLYNOMIAL_SEED_BYTE_0     0x9C9
#define SX1280_REG_CRC_MSB_INITIAL_VALUE          0x9C8
#define SX1280_REG_CRC_LSB_INITIAL_VALUE          0x9C9
#define SX1280_REG_SYNCH_ADDRESS_CONTROL          0x9CD
#define SX1280_REG_SYNC_ADDRESS_1_BYTE_4          0x9CE
#define SX1280_REG_SYNC_ADDRESS_1_BYTE_3          0x9CF
#define SX1280_REG_SYNC_ADDRESS_1_BYTE_2          0x9D0
#define SX1280_REG_SYNC_ADDRESS_1_BYTE_1          0x9D1
#define SX1280_REG_SYNC_ADDRESS_1_BYTE_0          0x9D2
#define SX1280_REG_SYNC_ADDRESS_2_BYTE_4          0x9D3
#define SX1280_REG_SYNC_ADDRESS_2_BYTE_3          0x9D4
#define SX1280_REG_SYNC_ADDRESS_2_BYTE_2          0x9D5
#define SX1280_REG_SYNC_ADDRESS_2_BYTE_1          0x9D6
#define SX1280_REG_SYNC_ADDRESS_2_BYTE_0          0x9D7
#define SX1280_REG_SYNC_ADDRESS_3_BYTE_4          0x9D8
#define SX1280_REG_SYNC_ADDRESS_3_BYTE_3          0x9D9
#define SX1280_REG_SYNC_ADDRESS_3_BYTE_2          0x9DA
#define SX1280_REG_SYNC_ADDRESS_3_BYTE_1          0x9DB
#define SX1280_REG_SYNC_ADDRESS_3_BYTE_0          0x9DC

/* Defaults */
#define SX1280_DEFAULT_RF_FREQ_HZ 2400000
#define SX1280_DEFAULT_STARTUP_TIMEOUT_US 2000
#define SX1280_DEFAULT_CRC_LENGTH SX1280_RADIO_CRC_2_BYTES

/* Limits */
#define SX1280_FLRC_PAYLOAD_LENGTH_MAX 127
#define SX1280_FLRC_PAYLOAD_LENGTH_MIN 6
#define SX1280_GFSK_PAYLOAD_LENGTH_MAX 255
#define SX1280_GFSK_PAYLOAD_LENGTH_MIN 0
#define SX1280_LORA_PAYLOAD_LENGTH_MAX 255
#define SX1280_LORA_PAYLOAD_LENGTH_MIN 1

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

  struct sx1280_gfsk_params gfsk;
  struct sx1280_flrc_params flrc;
  struct sx1280_lora_params lora;
  struct sx1280_ranging_params ranging;
};

#endif /* _SX1280_H */
