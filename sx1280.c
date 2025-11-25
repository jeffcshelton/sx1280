// SPDX-License-Identifier: GPL-2.0-only

/*
 * sx1280.c - Driver for the Semtech SX1280 RF transceiver.
 *
 * Maintained by: Jeff Shelton <jeff@shelton.one>
 *
 * Copyright (C) 2025 Jeff Shelton
 */

/*
 * TODO: Jam multiple Ethernet packets into one transmission.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <net/cfg80211.h>

// Constants.
#define SX1280_FREQ_XOSC_HZ 52000000

// Conversion macros.
#define SX1280_FREQ_HZ_TO_PLL(hz) ((u32) ((((u64) (hz) << 32) / SX1280_FREQ_XOSC_HZ) >> 14))
#define SX1280_FREQ_PLL_TO_HZ(pll) ((u32) ((((u64) (pll) << 14) * SX1280_FREQ_XOSC_HZ) >> 32))

enum sx1280_command {
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
  SX1280_CMD_SET_ADVANCED_RANGING       = 0x9A,
};

enum sx1280_mode {
  SX1280_MODE_GFSK    = 0x00,
  SX1280_MODE_LORA    = 0x01,
  SX1280_MODE_RANGING = 0x02,
  SX1280_MODE_FLRC    = 0x03,
};

static const char *sx1280_mode_etos(enum sx1280_mode mode) {
  switch (mode) {
  case SX1280_MODE_FLRC:
    return "flrc";
  case SX1280_MODE_GFSK:
    return "gfsk";
  case SX1280_MODE_LORA:
    return "lora";
  case SX1280_MODE_RANGING:
    return "ranging";
  }

  return ERR_PTR(-EINVAL);
}

enum sx1280_ramp_time {
  SX1280_RADIO_RAMP_02_US = 0x00,
  SX1280_RADIO_RAMP_04_US = 0x20,
  SX1280_RADIO_RAMP_06_US = 0x40,
  SX1280_RADIO_RAMP_08_US = 0x60,
  SX1280_RADIO_RAMP_10_US = 0x80,
  SX1280_RADIO_RAMP_12_US = 0xA0,
  SX1280_RADIO_RAMP_16_US = 0xC0,
  SX1280_RADIO_RAMP_20_US = 0xE0,
};

static int sx1280_ramp_time_etoi(enum sx1280_ramp_time ramp_time) {
  switch (ramp_time) {
  case SX1280_RADIO_RAMP_02_US: return 2;
  case SX1280_RADIO_RAMP_04_US: return 4;
  case SX1280_RADIO_RAMP_06_US: return 6;
  case SX1280_RADIO_RAMP_08_US: return 8;
  case SX1280_RADIO_RAMP_10_US: return 10;
  case SX1280_RADIO_RAMP_12_US: return 12;
  case SX1280_RADIO_RAMP_16_US: return 16;
  case SX1280_RADIO_RAMP_20_US: return 20;
  }
}

static int sx1280_ramp_time_itoe(int us) {
  switch (us) {
  case 2:  return SX1280_RADIO_RAMP_02_US;
  case 4:  return SX1280_RADIO_RAMP_04_US;
  case 6:  return SX1280_RADIO_RAMP_06_US;
  case 8:  return SX1280_RADIO_RAMP_08_US;
  case 10: return SX1280_RADIO_RAMP_10_US;
  case 12: return SX1280_RADIO_RAMP_12_US;
  case 16: return SX1280_RADIO_RAMP_16_US;
  case 20: return SX1280_RADIO_RAMP_20_US;
  default: return -EINVAL;
  }
}

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
  SX1280_PREAMBLE_LENGTH_32_BITS = 0x70,
};

inline static int sx1280_preamble_itoe(int bits) {
  if (bits < 4 || bits > 32 || bits % 4 != 0) {
    return -EINVAL;
  }

  return (bits - 4) << 2;
}

inline static int sx1280_preamble_etoi(enum sx1280_preamble_length len) {
  return (len >> 2) + 4;
}

enum sx1280_gfsk_sync_word_length {
  SX1280_SYNC_WORD_LEN_1_B = 0x00,
  SX1280_SYNC_WORD_LEN_2_B = 0x02,
  SX1280_SYNC_WORD_LEN_3_B = 0x04,
  SX1280_SYNC_WORD_LEN_4_B = 0x06,
  SX1280_SYNC_WORD_LEN_5_B = 0x08,
};

enum sx1280_sync_word_match {
  SX1280_RADIO_SELECT_SYNCWORD_OFF   = 0x00,
  SX1280_RADIO_SELECT_SYNCWORD_1     = 0x10,
  SX1280_RADIO_SELECT_SYNCWORD_2     = 0x20,
  SX1280_RADIO_SELECT_SYNCWORD_1_2   = 0x30,
  SX1280_RADIO_SELECT_SYNCWORD_3     = 0x40,
  SX1280_RADIO_SELECT_SYNCWORD_1_3   = 0x50,
  SX1280_RADIO_SELECT_SYNCWORD_2_3   = 0x60,
  SX1280_RADIO_SELECT_SYNCWORD_1_2_3 = 0x70,
};

enum sx1280_packet_type {
  SX1280_RADIO_PACKET_FIXED_LENGTH    = 0x00,
  SX1280_RADIO_PACKET_VARIABLE_LENGTH = 0x20,
};

enum sx1280_radio_crc {
  SX1280_RADIO_CRC_OFF     = 0x00,
  SX1280_RADIO_CRC_1_BYTE  = 0x10,
  SX1280_RADIO_CRC_2_BYTES = 0x20,
};

static int sx1280_radio_crc_etoi(enum sx1280_radio_crc crc) {
  return (int) (crc >> 4);
}

static int sx1280_radio_crc_itoe(int bytes) {
  if (bytes < 0 || bytes > 2) {
    return -EINVAL;
  }

  return bytes << 4;
}

enum sx1280_header_type {
  SX1280_EXPLICIT_HEADER = 0x00,
  SX1280_IMPLICIT_HEADER = 0x80,
};

enum sx1280_whitening {
  SX1280_WHITENING_ENABLE  = 0x00,
  SX1280_WHITENING_DISABLE = 0x08,
};

struct sx1280_gfsk_packet_params {
  enum sx1280_preamble_length preamble_length;
  enum sx1280_gfsk_sync_word_length sync_word_length;
  enum sx1280_sync_word_match sync_word_match;
  enum sx1280_packet_type packet_type;
  u8 payload_length;
  enum sx1280_radio_crc crc_length;
  enum sx1280_whitening whitening;
};

enum sx1280_flrc_sync_word_length {
  SX1280_FLRC_SYNC_WORD_NOSYNC   = 0x00,
  SX1280_FLRC_SYNC_WORD_LEN_P32S = 0x04,
};

enum sx1280_flrc_crc {
  SX1280_FLRC_CRC_OFF    = 0x00,
  SX1280_FLRC_CRC_2_BYTE = 0x10,
  SX1280_FLRC_CRC_3_BYTE = 0x20,
  SX1280_FLRC_CRC_4_BYTE = 0x30,
};

struct sx1280_flrc_packet_params {
  enum sx1280_preamble_length agc_preamble_length;
  enum sx1280_flrc_sync_word_length sync_word_length;
  enum sx1280_sync_word_match sync_word_match;
  enum sx1280_packet_type packet_type;
  u8 payload_length;
  enum sx1280_flrc_crc crc_length;
  enum sx1280_whitening whitening;
};

enum sx1280_lora_crc {
  SX1280_LORA_CRC_ENABLE  = 0x20,
  SX1280_LORA_CRC_DISABLE = 0x00,
};

enum sx1280_lora_iq {
  SX1280_LORA_IQ_INVERTED = 0x00,
  SX1280_LORA_IQ_STD      = 0x40,
};

struct sx1280_lora_packet_params {
  u8 preamble_length;
  enum sx1280_header_type header_type;
  u8 payload_length;
  enum sx1280_lora_crc crc;
  enum sx1280_lora_iq invert_iq;
};

struct sx1280_packet_params {
  enum sx1280_mode mode;

  union {
    struct sx1280_flrc_packet_params flrc;
    struct sx1280_gfsk_packet_params gfsk;
    struct sx1280_lora_packet_params lora;
  };
};

enum sx1280_fsk_bitrate_bandwidth {
  SX1280_FSK_BR_2_000_BW_2_4 = 0x04,
  SX1280_FSK_BR_1_600_BW_2_4 = 0x28,
  SX1280_FSK_BR_1_000_BW_2_4 = 0x4C,
  SX1280_FSK_BR_1_000_BW_1_2 = 0x45,
  SX1280_FSK_BR_0_800_BW_2_4 = 0x70,
  SX1280_FSK_BR_0_800_BW_1_2 = 0x69,
  SX1280_FSK_BR_0_500_BW_1_2 = 0x8D,
  SX1280_FSK_BR_0_500_BW_0_6 = 0x86,
  SX1280_FSK_BR_0_400_BW_1_2 = 0xB1,
  SX1280_FSK_BR_0_400_BW_0_6 = 0xAA,
  SX1280_FSK_BR_0_250_BW_0_6 = 0xCE,
  SX1280_FSK_BR_0_250_BW_0_3 = 0xC7,
  SX1280_FSK_BR_0_125_BW_0_3 = 0xEF,
};

inline static int sx1280_fsk_bitrate_etoi(
  enum sx1280_fsk_bitrate_bandwidth brbw
) {
  switch (brbw) {
  case SX1280_FSK_BR_2_000_BW_2_4:
    return 2000000;
  case SX1280_FSK_BR_1_600_BW_2_4:
    return 1600000;
  case SX1280_FSK_BR_1_000_BW_2_4:
  case SX1280_FSK_BR_1_000_BW_1_2:
    return 1000000;
  case SX1280_FSK_BR_0_800_BW_2_4:
  case SX1280_FSK_BR_0_800_BW_1_2:
    return 800000;
  case SX1280_FSK_BR_0_500_BW_1_2:
  case SX1280_FSK_BR_0_500_BW_0_6:
    return 500000;
  case SX1280_FSK_BR_0_400_BW_1_2:
  case SX1280_FSK_BR_0_400_BW_0_6:
    return 400000;
  case SX1280_FSK_BR_0_250_BW_0_6:
  case SX1280_FSK_BR_0_250_BW_0_3:
    return 250000;
  case SX1280_FSK_BR_0_125_BW_0_3:
    return 125000;
  }

  return -EINVAL;
}

inline static int sx1280_fsk_bandwidth_etoi(
  enum sx1280_fsk_bitrate_bandwidth brbw
) {
  switch (brbw) {
  case SX1280_FSK_BR_2_000_BW_2_4:
  case SX1280_FSK_BR_1_600_BW_2_4:
  case SX1280_FSK_BR_1_000_BW_2_4:
  case SX1280_FSK_BR_0_800_BW_2_4:
    return 2400000;
  case SX1280_FSK_BR_1_000_BW_1_2:
  case SX1280_FSK_BR_0_800_BW_1_2:
  case SX1280_FSK_BR_0_500_BW_1_2:
  case SX1280_FSK_BR_0_400_BW_1_2:
    return 1200000;
  case SX1280_FSK_BR_0_500_BW_0_6:
  case SX1280_FSK_BR_0_400_BW_0_6:
  case SX1280_FSK_BR_0_250_BW_0_6:
    return 600000;
  case SX1280_FSK_BR_0_250_BW_0_3:
  case SX1280_FSK_BR_0_125_BW_0_3:
    return 300000;
  }

  return -EINVAL;
}

enum sx1280_modulation_index {
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
  SX1280_MOD_IND_4_00 = 0x0F,
};

static const char *sx1280_modulation_index_etos(
  enum sx1280_modulation_index mod_index
) {
  switch (mod_index) {
  case SX1280_MOD_IND_0_35: return "0.35";
  case SX1280_MOD_IND_0_50: return "0.50";
  case SX1280_MOD_IND_0_75: return "0.75";
  case SX1280_MOD_IND_1_00: return "1.00";
  case SX1280_MOD_IND_1_25: return "1.25";
  case SX1280_MOD_IND_1_50: return "1.50";
  case SX1280_MOD_IND_1_75: return "1.75";
  case SX1280_MOD_IND_2_00: return "2.00";
  case SX1280_MOD_IND_2_25: return "2.25";
  case SX1280_MOD_IND_2_50: return "2.50";
  case SX1280_MOD_IND_2_75: return "2.75";
  case SX1280_MOD_IND_3_00: return "3.00";
  case SX1280_MOD_IND_3_25: return "3.25";
  case SX1280_MOD_IND_3_50: return "3.50";
  case SX1280_MOD_IND_3_75: return "3.75";
  case SX1280_MOD_IND_4_00: return "4.00";
  }

  return ERR_PTR(-EINVAL);
}

enum sx1280_bandwidth_time {
  SX1280_BT_OFF = 0x00,
  SX1280_BT_1_0 = 0x10,
  SX1280_BT_0_5 = 0x20,
};

static const char *sx1280_bandwidth_time_etos(enum sx1280_bandwidth_time bt) {
  switch (bt) {
  case SX1280_BT_OFF: return "off";
  case SX1280_BT_0_5: return "0.5";
  case SX1280_BT_1_0: return "1.0";
  }

  return ERR_PTR(-EINVAL);
}

struct sx1280_gfsk_modulation_params {
  enum sx1280_fsk_bitrate_bandwidth bitrate_bandwidth;
  enum sx1280_modulation_index modulation_index;
  enum sx1280_bandwidth_time bandwidth_time;
};

enum sx1280_flrc_bitrate_bandwidth {
  SX1280_FLRC_BR_1_300_BW_1_2 = 0x45,
  SX1280_FLRC_BR_1_000_BW_1_2 = 0x69,
  SX1280_FLRC_BR_0_650_BW_0_6 = 0x86,
  SX1280_FLRC_BR_0_520_BW_0_6 = 0xAA,
  SX1280_FLRC_BR_0_325_BW_0_3 = 0xC7,
  SX1280_FLRC_BR_0_260_BW_0_3 = 0xEB,
};

enum sx1280_flrc_coding_rate {
  SX1280_FLRC_CR_1_2 = 0x00,
  SX1280_FLRC_CR_3_4 = 0x02,
  SX1280_FLRC_CR_1_1 = 0x04,
};

struct sx1280_flrc_modulation_params {
  enum sx1280_flrc_bitrate_bandwidth bitrate_bandwidth;
  enum sx1280_flrc_coding_rate coding_rate;
  enum sx1280_bandwidth_time bandwidth_time;
};

enum sx1280_lora_spreading_factor {
  SX1280_LORA_SF_5  = 0x50,
  SX1280_LORA_SF_6  = 0x60,
  SX1280_LORA_SF_7  = 0x70,
  SX1280_LORA_SF_8  = 0x80,
  SX1280_LORA_SF_9  = 0x90,
  SX1280_LORA_SF_10 = 0xA0,
  SX1280_LORA_SF_11 = 0xB0,
  SX1280_LORA_SF_12 = 0xC0,
};

enum sx1280_lora_bandwidth {
  SX1280_LORA_BW_1600 = 0x0A,
  SX1280_LORA_BW_800  = 0x18,
  SX1280_LORA_BW_400  = 0x26,
  SX1280_LORA_BW_200  = 0x34,
};

enum sx1280_lora_coding_rate {
  SX1280_LORA_CR_4_5    = 0x01,
  SX1280_LORA_CR_4_6    = 0x02,
  SX1280_LORA_CR_4_7    = 0x03,
  SX1280_LORA_CR_4_8    = 0x04,
  SX1280_LORA_CR_LI_4_5 = 0x05,
  SX1280_LORA_CR_LI_4_6 = 0x06,
  SX1280_LORA_CR_LI_4_8 = 0x07,
};

struct sx1280_lora_modulation_params {
  enum sx1280_lora_spreading_factor spreading_factor;
  enum sx1280_lora_bandwidth bandwidth;
  enum sx1280_lora_coding_rate coding_rate;
};

struct sx1280_modulation_params {
  enum sx1280_mode mode;

  union {
    struct sx1280_flrc_modulation_params flrc;
    struct sx1280_gfsk_modulation_params gfsk;
    struct sx1280_lora_modulation_params lora;
  };
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

#define SX1280_IRQ_TX_DONE                       BIT(0)
#define SX1280_IRQ_RX_DONE                       BIT(1)
#define SX1280_IRQ_SYNC_WORD_VALID               BIT(2)
#define SX1280_IRQ_SYNC_WORD_ERROR               BIT(3)
#define SX1280_IRQ_HEADER_VALID                  BIT(4)
#define SX1280_IRQ_HEADER_ERROR                  BIT(5)
#define SX1280_IRQ_CRC_ERROR                     BIT(6)
#define SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE   BIT(7)
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARD BIT(8)
#define SX1280_IRQ_RANGING_MASTER_RESULT_VALID   BIT(9)
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT        BIT(10)
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID   BIT(11)
#define SX1280_IRQ_CAD_DONE                      BIT(11)
#define SX1280_IRQ_CAD_DETECTED                  BIT(13)
#define SX1280_IRQ_RX_TX_TIMEOUT                 BIT(14)
#define SX1280_IRQ_PREAMBLE_DETECTED             BIT(15)
#define SX1280_IRQ_ADVANCED_RANGING_DONE         BIT(15)

/* GetPacketStatus status flags */
#define SX1280_PACKET_STATUS_STATUS_RX_NO_ACK BIT(5)
#define SX1280_PACKET_STATUS_STATUS_PKT_SENT  BIT(0)

/* GetPacketStatus errors flags */
#define SX1280_PACKET_STATUS_ERROR_SYNC_ERROR       BIT(6)
#define SX1280_PACKET_STATUS_ERROR_LENGTH_ERROR     BIT(5)
#define SX1280_PACKET_STATUS_ERROR_CRC_ERROR        BIT(4)
#define SX1280_PACKET_STATUS_ERROR_ABORT_ERROR      BIT(3)
#define SX1280_PACKET_STATUS_ERROR_HEADER_RECEIVED  BIT(2)
#define SX1280_PACKET_STATUS_ERROR_PACKET_RECEIVED  BIT(1)
#define SX1280_PACKET_STATUS_ERROR_PACKET_CTRL_BUSY BIT(0)

/* Status masks */
#define SX1280_STATUS_CIRCUIT_MODE_MASK GENMASK(7, 5)
#define SX1280_STATUS_COMMAND_STATUS_MASK GENMASK(4, 2)

/* Circuit modes (field of GetStatus) */
enum sx1280_circuit_mode {
  SX1280_CIRCUIT_MODE_STDBY_RC   = 0x2,
  SX1280_CIRCUIT_MODE_STDBY_XOSC = 0x3,
  SX1280_CIRCUIT_MODE_FS         = 0x4,
  SX1280_CIRCUIT_MODE_RX         = 0x5,
  SX1280_CIRCUIT_MODE_TX         = 0x6,
};

/* Command statuses (field of GetStatus) */
enum sx1280_command_status {
  SX1280_COMMAND_STATUS_TX_PROCESSED     = 0x1,
  SX1280_COMMAND_STATUS_DATA_AVAILABLE   = 0x2,
  SX1280_COMMAND_STATUS_TIMEOUT          = 0x3,
  SX1280_COMMAND_STATUS_PROCESSING_ERROR = 0x4,
  SX1280_COMMAND_STATUS_EXEC_FAILURE     = 0x5,
  SX1280_COMMAND_STATUS_TX_DONE          = 0x6,
};

/* Registers */
enum sx1280_register {
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
  SX1280_REG_CRC_LSB_INITIAL_VALUE          = 0x9C9,
  SX1280_REG_SYNCH_ADDRESS_CONTROL          = 0x9CD,
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
  SX1280_REG_SYNC_ADDRESS_3_BYTE_0          = 0x9DC,
};

/* Limits */
#define SX1280_FLRC_PAYLOAD_LENGTH_MAX 127
#define SX1280_FLRC_PAYLOAD_LENGTH_MIN 6
#define SX1280_GFSK_PAYLOAD_LENGTH_MAX 255
#define SX1280_GFSK_PAYLOAD_LENGTH_MIN 0
#define SX1280_LORA_PAYLOAD_LENGTH_MAX 255
#define SX1280_LORA_PAYLOAD_LENGTH_MIN 1

struct sx1280_flrc_params {
  u8 crc_seed[2];
  struct sx1280_flrc_modulation_params modulation;
  struct sx1280_flrc_packet_params packet;
};

struct sx1280_gfsk_params {
  u8 crc_polynomial[2];
  u8 crc_seed[2];
  struct sx1280_gfsk_modulation_params modulation;
  struct sx1280_gfsk_packet_params packet;
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
 * @busy_gpio - The legacy GPIO number corresponding to the BUSY pin.
 * @dio_gpios - The legacy GPIO numbers corresponding to the DIO pins.
 * @reset_gpio - The legacy GPIO number corresponding to the NRESET pin/
 */
struct sx1280_platform_data {
  unsigned int busy_gpio;
  int dio_gpios[3];
  unsigned int reset_gpio;
};

/**
 * struct sx1280_config - Configuration data for the SX1280 driver.
 *
 * Note that every field in an instance of the config should be represented in
 * the form that the chip expects to receive, not necessarily human-readable
 * form.
 *
 * @mode - The packet type of the transceiver: GFSK, FLRC, LoRa, or Ranging.
 * TODO
 */
struct sx1280_config {
  enum sx1280_mode mode;
  enum sx1280_period_base period_base;
  u16 period_base_count;
  u8 power;
  enum sx1280_ramp_time ramp_time;
  u32 freq;
  u8 sync_words[3][5];

  struct sx1280_flrc_params flrc;
  struct sx1280_gfsk_params gfsk;
  struct sx1280_lora_params lora;
  struct sx1280_ranging_params ranging;
};

static const struct sx1280_config sx1280_default_config = {
  .mode = SX1280_MODE_GFSK,
  .period_base = SX1280_PERIOD_BASE_1_MS,
  .period_base_count = 1000,
  .power = 18, /* 0 dBm */
  .ramp_time = SX1280_RADIO_RAMP_08_US,
  .freq = SX1280_FREQ_HZ_TO_PLL(2400000000), /* 2.4 GHz */
  .sync_words = {
    { 0x12, 0xAD, 0x34, 0xCD, 0x56 },
    { 0xD3, 0x91, 0xD3, 0x91, 0xD3 },
    { 0xAA, 0xF0, 0x05, 0x3C, 0x81 },
  },
  .flrc = {
    .crc_seed = { 0xFF, 0xFF },
    .modulation = {
      .bandwidth_time = SX1280_BT_1_0,
      .bitrate_bandwidth = SX1280_FLRC_BR_1_300_BW_1_2,
      .coding_rate = SX1280_FLRC_CR_3_4,
    },
    .packet = {
      .agc_preamble_length = SX1280_PREAMBLE_LENGTH_08_BITS,
      .crc_length = SX1280_FLRC_CRC_2_BYTE,
      .packet_type = SX1280_RADIO_PACKET_VARIABLE_LENGTH,
      .payload_length = SX1280_FLRC_PAYLOAD_LENGTH_MAX,
      .sync_word_length = SX1280_FLRC_SYNC_WORD_LEN_P32S,
      .sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_1,
      .whitening = SX1280_WHITENING_ENABLE,
    },
  },
  .gfsk = {
    .crc_polynomial = { 0x10, 0x21 },
    .crc_seed = { 0xFF, 0xFF },
    .modulation = {
      .bandwidth_time = SX1280_BT_0_5,
      .bitrate_bandwidth = SX1280_FSK_BR_2_000_BW_2_4,
      .modulation_index = SX1280_MOD_IND_0_50,
    },
    .packet = {
      .crc_length = SX1280_RADIO_CRC_2_BYTES,
      .packet_type = SX1280_RADIO_PACKET_VARIABLE_LENGTH,
      .payload_length = SX1280_GFSK_PAYLOAD_LENGTH_MAX,
      .preamble_length = SX1280_PREAMBLE_LENGTH_08_BITS,
      .sync_word_length = SX1280_SYNC_WORD_LEN_4_B,
      .sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_1,
      .whitening = SX1280_WHITENING_ENABLE,
    },
  },
  .lora = {
    .modulation = {
      .bandwidth = SX1280_LORA_BW_1600,
      .coding_rate = SX1280_LORA_CR_4_7,
      .spreading_factor = SX1280_LORA_SF_12,
    },
    .packet = {
      .crc = SX1280_LORA_CRC_ENABLE,
      .header_type = SX1280_EXPLICIT_HEADER,
      .invert_iq = SX1280_LORA_IQ_STD,
      .payload_length = SX1280_LORA_PAYLOAD_LENGTH_MAX,
      .preamble_length = SX1280_PREAMBLE_LENGTH_08_BITS,
    },
  },
  .ranging = {},
};

#define SX1280_BUSY_TIMEOUT_US 100000

/* The private, internal structure for the SX1280 driver. */
struct sx1280_priv {
  /* Devices */
  struct net_device *netdev;
  struct spi_device *spi;

  /* GPIOs + IRQs */
  struct gpio_desc *busy;
  struct gpio_desc *dio;
  struct gpio_desc *reset;
  int dio_index;
  int irq;

  /*
   * The current configuration of the SX1280.
   */
  struct sx1280_config cfg;

  /* The packet waiting to be transmitted. */
  struct sk_buff *tx_skb;

  struct workqueue_struct *xmit_queue;
  struct work_struct tx_work;

  /*
   * Mutex that locks all uninterruptible operations.
   *
   * This mutex should be locked before every transaction involving the chip,
   * especially in IRQ handlers.
   */
  struct mutex lock;

  /*
   * Whether the chip is idle, meaning that it is not currently performing an
   * operation that must not be interrupted, such as transmitting a packet.
   *
   * This is different than just waiting for the BUSY pin to be pulled low,
   * which is required before every SPI transaction with the chip.
   */
  bool idle;

  /*
   * Wait queue for all operations that require the chip to be idle, such as
   * setting mode and packet parameters. All waiters in the queue will be
   * triggered at once and must hold a lock.
   */
  wait_queue_head_t idle_wait;

  /*
   * Whether the device / driver is fully initialized.
   *
   * Used to gate the IRQ handler so that it doesn't receive spurious interrupts
   * during setup, causing the device / driver to enter an invalid state.
   */
  bool initialized;

#ifdef DEBUG
  struct delayed_work status_check;
#endif
};

/****************
* SPI Functions *
****************/

/**
 * Waits for the BUSY pin to be pulled low, so a SPI transfer can begin.
 *
 * For short waits, which are expected in the vast majority of cases, this
 * function quickly busy-loops. Once the time has surpassed 50 us, it starts
 * sleeping for longer periods before ultimately timing out.
 *
 * @context - process & locked
 */
static int sx1280_wait_busy(struct sx1280_priv *priv) {
  ktime_t start = ktime_get();
  s64 wait = 0;

  while (gpiod_get_value_cansleep(priv->busy)) {
    wait = ktime_us_delta(ktime_get(), start);

    if (wait < 50) {
      cpu_relax();
    } else if (wait < SX1280_BUSY_TIMEOUT_US) {
      usleep_range(20, 40);
    } else {
      return -ETIMEDOUT;
    }
  }

  return 0;
}

/**
 * Performs an arbitrary SPI transaction with the SX1280, after first waiting
 * for BUSY = 0 (this is necessary for every transaction).
 *
 * @context - process & locked
 */
static int sx1280_transfer(
  struct sx1280_priv *priv,
  struct spi_transfer *xfers,
  unsigned int num_xfers
) {
  int err;
  if (
    (err = sx1280_wait_busy(priv))
    || (err = spi_sync_transfer(priv->spi, xfers, num_xfers))
    || (sx1280_wait_busy(priv)) /* TODO: consider removing */
  ) {
    return err;
  }

  return 0;
}

static int sx1280_write(
  struct sx1280_priv *priv,
  void *buf,
  size_t len
) {
  int err;
  if (!(err = sx1280_wait_busy(priv))) {
    err = spi_write(priv->spi, buf, len);
  }

  return err;
}

/**
 * @context process & locked
 */
static int sx1280_get_status(struct sx1280_priv *priv, u8 *status) {
  int err;
  u8 tx[2] = { SX1280_CMD_GET_STATUS };
  u8 rx[2];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx),
  };

  if ((err = sx1280_transfer(priv, &xfer, 1))) {
    dev_err(&priv->spi->dev, "GetStatus failed: %d\n", err);
    return err;
  }

  if (status) {
    *status = rx[1];
  }

  return 0;
}

/**
 * @context process & locked
 */
static int sx1280_write_register(
  struct sx1280_priv *priv,
  u16 addr,
  const u8 *data,
  size_t len
) {
  int err;
  u8 cmd_tx[] = {
    SX1280_CMD_WRITE_REGISTER,
    addr >> 8,
    addr & 0xFF
  };

  struct spi_transfer xfers[] = {
    {
      .tx_buf = cmd_tx,
      .len = ARRAY_SIZE(cmd_tx),
      .cs_change = 0,
},
    {
      .tx_buf = data,
      .len = len,
    }
  };

  if ((err = sx1280_transfer(priv, xfers, ARRAY_SIZE(xfers)))) {
    dev_err(&priv->spi->dev, "WriteRegister failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_read_register(
  struct sx1280_priv *priv,
  u16 addr,
  u8 *data,
  size_t len
) {
  int err;
  u8 cmd_tx[] = {
    SX1280_CMD_READ_REGISTER,
    addr >> 8,
    addr & 0xFF,
    0
  };

  struct spi_transfer xfers[] = {
    {
      .tx_buf = cmd_tx,
      .len = ARRAY_SIZE(cmd_tx),
      .cs_change = 0
    },
    {
      .rx_buf = data,
      .len = len
    }
  };

  if ((err = sx1280_transfer(priv, xfers, ARRAY_SIZE(xfers)))) {
    dev_err(&priv->spi->dev, "ReadRegister failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_write_buffer(
  struct sx1280_priv *priv,
  u8 offset,
  const u8 *data,
  size_t len
) {
  int err;
  u8 cmd_tx[] = {
    SX1280_CMD_WRITE_BUFFER,
    offset
  };

  struct spi_transfer xfers[] = {
    {
      .tx_buf = cmd_tx,
      .len = ARRAY_SIZE(cmd_tx),
      .cs_change = 0
    },
    {
      .tx_buf = data,
      .len = len
    }
  };

  if ((err = sx1280_transfer(priv, xfers, ARRAY_SIZE(xfers)))) {
    dev_err(&priv->spi->dev, "WriteBuffer failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_read_buffer(
  struct sx1280_priv *priv,
  u8 offset,
  u8 *data,
  size_t len
) {
  int err;
  u8 cmd_tx[3] = { SX1280_CMD_READ_BUFFER, offset };

  struct spi_transfer xfers[] = {
    {
      .tx_buf = cmd_tx,
      .len = ARRAY_SIZE(cmd_tx),
      .cs_change = 0
    },
    {
      .rx_buf = data,
      .len = len
    }
  };

  if ((err = sx1280_transfer(priv, xfers, ARRAY_SIZE(xfers)))) {
    dev_err(&priv->spi->dev, "ReadBuffer failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_sleep(
  struct sx1280_priv *priv,
  bool save_buffer,
  bool save_ram
) {
  int err;
  u8 sleep_config = (save_buffer << 1) | save_ram;
  u8 tx[] = { SX1280_CMD_SET_SLEEP, sleep_config };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetSleep failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_standby(struct sx1280_priv *priv, u8 mode) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_STANDBY, mode };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetStandby failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_fs(struct sx1280_priv *priv) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_FS };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetFs failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_tx(
  struct sx1280_priv *priv,
  u8 period_base,
  u16 period_base_count
) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_TX,
    period_base,
    period_base_count >> 8,
    period_base_count & 0xFF
  };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetTx failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_rx(
  struct sx1280_priv *priv,
  u8 period_base,
  u16 period_base_count
) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_RX,
    period_base,
    period_base_count >> 8,
    period_base_count & 0xFF
  };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetRx failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_rx_duty_cycle(
  struct sx1280_priv *priv,
  u8 period_base,
  u16 rx_period_base_count,
  u16 sleep_period_base_count
) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_RX_DUTY_CYCLE,
    period_base,
    rx_period_base_count >> 8,
    rx_period_base_count & 0xFF,
    sleep_period_base_count >> 8,
    sleep_period_base_count & 0xFF
  };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_dbg(&priv->spi->dev, "SetRxDutyCycle failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_long_preamble(struct sx1280_priv *priv, bool enable) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_LONG_PREAMBLE, enable };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetLongPreamble failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_cad(struct sx1280_priv *priv) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_CAD };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetCad failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_tx_continuous_wave(struct sx1280_priv *priv) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_TX_CONTINUOUS_WAVE };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetTxContinuousWave failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_tx_continuous_preamble(struct sx1280_priv *priv) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_TX_CONTINUOUS_PREAMBLE };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetTxContinuousPreamble failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_auto_tx(struct sx1280_priv *priv, u16 time) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_AUTO_TX,
    time >> 8,
    time & 0xFF
  };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetAutoTx failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_auto_fs(struct sx1280_priv *priv, bool enable) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_AUTO_FS, enable };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetAutoFs failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_packet_type(
  struct sx1280_priv *priv,
  enum sx1280_mode packet_type
) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_PACKET_TYPE, (u8) packet_type };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetPacketType failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_get_packet_type(
  struct sx1280_priv *priv,
  enum sx1280_mode *packet_type
) {
  int err;
  u8 tx[3] = { SX1280_CMD_GET_PACKET_TYPE };
  u8 rx[3];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  if ((err = sx1280_transfer(priv, &xfer, 1))) {
    dev_err(&priv->spi->dev, "GetPacketType failed: %d\n", err);
    return err;
  } else if (rx[2] > 0x04) {
    dev_err(&priv->spi->dev, "GetPacketType returned invalid value: %d\n", rx[2]);
    return -EINVAL;
  }

  if (packet_type) {
    *packet_type = (enum sx1280_mode) rx[2];
  }

  return 0;
}

static int sx1280_set_rf_frequency(
  struct sx1280_priv *priv,
  u32 freq
) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_RF_FREQUENCY,
    freq >> 16,
    (freq >> 8) & 0xFF,
    freq & 0xFF
  };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetRfFrequency failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_tx_params(
  struct sx1280_priv *priv,
  u8 power,
  enum sx1280_ramp_time ramp_time
) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_TX_PARAMS, power, (u8) ramp_time };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetTxParams failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_cad_params(
  struct sx1280_priv *priv,
  enum sx1280_cad_symbol_num cad_symbol_num
) {
  int err;
  u8 tx[] = { SX1280_CMD_SET_CAD_PARAMS, (u8) cad_symbol_num };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetCadParams failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_buffer_base_address(
  struct sx1280_priv *priv,
  u8 tx_base_addr,
  u8 rx_base_addr
) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_BUFFER_BASE_ADDRESS,
    tx_base_addr,
    rx_base_addr
  };

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetBufferBaseAddress failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_modulation_params(
  struct sx1280_priv *priv,
  struct sx1280_modulation_params params
) {
  int err;
  u8 tx[4] = { SX1280_CMD_SET_MODULATION_PARAMS };

  switch (params.mode) {
  case SX1280_MODE_FLRC:
    tx[1] = (u8) params.flrc.bitrate_bandwidth;
    tx[2] = (u8) params.flrc.coding_rate;
    tx[3] = (u8) params.flrc.bandwidth_time;
    break;
  case SX1280_MODE_GFSK:
    tx[1] = (u8) params.gfsk.bitrate_bandwidth;
    tx[2] = (u8) params.gfsk.modulation_index;
    tx[3] = (u8) params.gfsk.bandwidth_time;
    break;
  case SX1280_MODE_LORA:
  case SX1280_MODE_RANGING:
    tx[1] = (u8) params.lora.spreading_factor;
    tx[2] = (u8) params.lora.bandwidth;
    tx[3] = (u8) params.lora.coding_rate;
    break;
  default:
    return -EINVAL;
  }

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetModulationParams failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_packet_params(
  struct sx1280_priv *priv,
  struct sx1280_packet_params params
) {
  int err;
  u8 tx[8] = {
    SX1280_CMD_SET_PACKET_PARAMS
  };

  switch (params.mode) {
  case SX1280_MODE_FLRC:
    tx[1] = (u8) params.flrc.agc_preamble_length;
    tx[2] = (u8) params.flrc.sync_word_length;
    tx[3] = (u8) params.flrc.sync_word_match;
    tx[4] = (u8) params.flrc.packet_type;
    tx[5] = (u8) params.flrc.payload_length;
    tx[6] = (u8) params.flrc.crc_length;
    tx[7] = (u8) params.flrc.whitening;
    break;
  case SX1280_MODE_GFSK:
    tx[1] = (u8) params.gfsk.preamble_length;
    tx[2] = (u8) params.gfsk.sync_word_length;
    tx[3] = (u8) params.gfsk.sync_word_match;
    tx[4] = (u8) params.gfsk.packet_type;
    tx[5] = (u8) params.gfsk.payload_length;
    tx[6] = (u8) params.gfsk.crc_length;
    tx[7] = (u8) params.gfsk.whitening;
    break;
  case SX1280_MODE_LORA:
  case SX1280_MODE_RANGING:
    tx[1] = (u8) params.lora.preamble_length;
    tx[2] = (u8) params.lora.header_type;
    tx[3] = (u8) params.lora.payload_length;
    tx[4] = (u8) params.lora.crc;
    tx[5] = (u8) params.lora.invert_iq;
    break;
  default:
    return -EINVAL;
  }

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetPacketParams: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_get_rx_buffer_status(
  struct sx1280_priv *priv,
  u8 *rx_payload_len,
  u8 *rx_start_buffer_ptr
) {
  int err;
  u8 tx[4] = { SX1280_CMD_GET_RX_BUFFER_STATUS };
  u8 rx[4];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  if ((err = sx1280_transfer(priv, &xfer, 1))) {
    dev_err(&priv->spi->dev, "GetRxBufferStatus failed: %d\n", err);
    return err;
  }

  if (rx_payload_len) {
    *rx_payload_len = rx[2];
  }

  if (rx_start_buffer_ptr) {
    *rx_start_buffer_ptr = rx[3];
  }

  return 0;
}

static int sx1280_get_packet_status(
  struct sx1280_priv *priv,
  union sx1280_packet_status *packet_status
) {
  int err;
  u8 tx[7] = { SX1280_CMD_GET_PACKET_STATUS };
  u8 rx[7];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  if ((err = sx1280_transfer(priv, &xfer, 1))) {
    dev_err(&priv->spi->dev, "GetPacketStatus failed: %d\n", err);
    return err;
  }

  memcpy(&packet_status->raw, &rx[2], 5);
  return 0;
}

static int sx1280_get_rssi_inst(struct sx1280_priv *priv, u8 *rssi_inst) {
  int err;
  u8 tx[3] = { SX1280_CMD_GET_RSSI_INST };
  u8 rx[3];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  if ((err = sx1280_transfer(priv, &xfer, 1))) {
    dev_err(&priv->spi->dev, "GetRssiInst failed: %d\n", err);
    return err;
  }

  if (rssi_inst) {
    *rssi_inst = rx[2];
  }

  return 0;
}

/** Configures the DIO pins to act as interrupts according to their masks. */
static int sx1280_set_dio_irq_params(
  struct sx1280_priv *priv,
  u16 irq_mask,
  u16 dio_mask[3]
) {
  int err;
  u8 tx[] = {
    SX1280_CMD_SET_DIO_IRQ_PARAMS,
    irq_mask >> 8,
    irq_mask & 0xFF,
    dio_mask[0] >> 8,
    dio_mask[0] & 0xFF,
    dio_mask[1] >> 8,
    dio_mask[1] & 0xFF,
    dio_mask[2] >> 8,
    dio_mask[2] & 0xFF
  };

  if ((err = sx1280_write(priv, tx, sizeof(tx)))) {
    dev_err(&priv->spi->dev, "SetDioIrqParams failed: %d\n", err);
    return err;
  }

  return 0;
}

/** Gets the current state of the IRQ register. */
static int sx1280_get_irq_status(struct sx1280_priv *priv, u16 *irq_status) {
  int err;
  u8 tx[4] = { SX1280_CMD_GET_IRQ_STATUS };
  u8 rx[4];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx),
  };

  if ((err = sx1280_transfer(priv, &xfer, 1))) {
    dev_err(&priv->spi->dev, "GetIrqStatus failed: %d\n", err);
    return err;
  }

  if (irq_status) {
    /* The IRQ status is returned in big-endian format. */
    *irq_status = ((u16) rx[2] << 8) | rx[3];
  }

  return 0;
}

/** Clears flags in the IRQ register according to the mask. */
static int sx1280_clear_irq_status(struct sx1280_priv *priv, u16 irq_mask) {
  int err;
  u8 tx[] = {
    SX1280_CMD_CLR_IRQ_STATUS,
    irq_mask >> 8,
    irq_mask & 0xFF
  };

  if ((err = sx1280_write(priv, tx, sizeof(tx)))) {
    dev_err(&priv->spi->dev, "ClearIrqStatus failed: %d\n", err);
    return err;
  }

  return 0;
}

/*******************
* Driver functions *
*******************/

static int sx1280_open(struct net_device *netdev) {
  netdev_dbg(
    netdev,
    "ndo_open called by process: %s (pid %d)\n",
    current->comm,
    current->pid
  );

  if (current->parent) {
    netdev_dbg(
      netdev,
      "  parent process: %s (pid %d)\n",
      current->parent->comm,
      current->parent->pid
    );
  }

  netif_carrier_on(netdev);
  netif_start_queue(netdev);
  return 0;
}

static int sx1280_stop(struct net_device *netdev) {
  netdev_dbg(
    netdev,
    "ndo_stop called by process: %s (pid %d)\n",
    current->comm,
    current->pid
  );

  if (current->parent) {
    netdev_dbg(
      netdev,
      "  parent process: %s (pid %d)\n",
      current->parent->comm,
      current->parent->pid
    );
  }

  netif_stop_queue(netdev);
  netif_carrier_off(netdev);
  return 0;
}

/**
 * Called to transmit a single packet buffer.
 * @context atomic | process
 */
static netdev_tx_t sx1280_xmit(struct sk_buff *skb, struct net_device *netdev) {
  struct sx1280_priv *priv = netdev_priv(netdev);

  /*
   * Since the netdev is configured as an Ethernet device, the SKB is guaranteed
   * to be wrapped with an Ethernet header.
   */
  struct ethhdr *ethh = (struct ethhdr *) skb->data;
  u16 protocol = ntohs(ethh->h_proto);

  /* Log information about the packet for debugging. */
  netdev_dbg(netdev, "xmit: proto=0x%04x, len=%d\n", protocol, skb->len);
  netdev_dbg(netdev, "  mac: src=%pM, dst=%pM\n", ethh->h_source, ethh->h_dest);

  void *body = skb->data + sizeof(struct ethhdr);

  /* If the protocol is a */
  switch (protocol) {
  case ETH_P_IP:;
    struct iphdr *iph = (struct iphdr *) body;
    netdev_dbg(
      netdev,
      "  ipv4: src=%pI4, dst=%pI4\n",
      &iph->saddr,
      &iph->daddr
    );

    break;
  case ETH_P_IPV6:;
    struct ipv6hdr *ip6h = (struct ipv6hdr *) body;
    netdev_dbg(
      netdev,
      "  ipv6: src=%pI6c, dst=%pI6c\n",
      &ip6h->saddr,
      &ip6h->daddr
    );

    break;
  case ETH_P_ARP:;
    struct arphdr *arph = (struct arphdr *) body;
    netdev_dbg(
      netdev,
      "  arp: proto=0x%04x, op=%u\n",
      ntohs(arph->ar_pro),
      ntohs(arph->ar_op)
    );

    break;
  }

  /*
   * Stop the packet queue, applying backpressure to the kernel networking stack
   * that allows the driver to send one packet at a time. Packets that arrive in
   * the intervening time will be queued by the networking stack.
   *
   * Once the corresponding packet has been sent and the chip is ready for a new
   * one, `netif_wake_queue` is called to tell the kernel that it is permitted
   * to call `sx1280_xmit` once again.
   */
  netif_stop_queue(netdev);

  /*
   * Check if there is already a packet being transmitted.
   *
   * Generally, the kernel will not call `ndo_start_xmit` if the packet queue is
   * stopped. However, if there are packets in flight before the queue was
   * stopped, they will still arrive here. In that case, apply backpressure to
   * the networking stack.
   */
  if (priv->tx_skb) {
    netdev_err(netdev, "packet transmission requested after queue frozen\n");
    return NETDEV_TX_BUSY;
  }

  /* Queue the work so that it can be performed in a non-atomic context. */
  priv->tx_skb = skb;
  queue_work(priv->xmit_queue, &priv->tx_work);
  return NETDEV_TX_OK;
}

static void sx1280_tx_work(struct work_struct *work) {
  struct sx1280_priv *priv = container_of(work, struct sx1280_priv, tx_work);
  struct net_device *netdev = priv->netdev;
  struct sk_buff *skb = priv->tx_skb;

  if (!skb) {
    netdev_warn(netdev, "transmission queued without packet skb\n");
    return;
  }

  mutex_lock(&priv->lock);

  struct sx1280_packet_params params = { .mode = priv->cfg.mode };
  switch (params.mode) {
  case SX1280_MODE_FLRC:
    params.flrc = priv->cfg.flrc.packet;

    /* TODO: Pad FLRC packets less than 6 bytes. */
    if (skb->len < 6 || skb->len > 127) {
      netdev_warn(netdev, "invalid FLRC packet size: %d bytes\n", skb->len);
      goto drop;
    }

    params.flrc.payload_length = skb->len;
    break;
  case SX1280_MODE_GFSK:
    params.gfsk = priv->cfg.gfsk.packet;

    if (skb->len > 255) {
      netdev_warn(netdev, "invalid GFSK packet size: %d bytes\n", skb->len);
      goto drop;
    }

    params.gfsk.payload_length = skb->len;
    break;
  case SX1280_MODE_LORA:
    params.lora = priv->cfg.lora.packet;

    if (skb->len < 1 || skb->len > 255) {
      netdev_warn(netdev, "invalid LoRa packet size: %d bytes\n", skb->len);
      goto drop;
    }

    params.lora.payload_length = skb->len;
    break;
  case SX1280_MODE_RANGING:
    /* Packets can't be sent in ranging mode. */
    netdev_warn(netdev, "packet transmission requested in ranging mode\n");
    goto drop;
  }

  /* Write packet data and packet parameters onto the chip. */
  if (
    sx1280_set_standby(priv, SX1280_STDBY_RC) /* TODO: remove if not necessary */
    || sx1280_write_buffer(priv, 0x00, skb->data, skb->len)
    || sx1280_set_packet_params(priv, params)
    || sx1280_set_tx(priv, priv->cfg.period_base, priv->cfg.period_base_count)
  ) {
    goto drop;
  }

  priv->idle = false;
  mutex_unlock(&priv->lock);
  return;

drop:
  netdev->stats.tx_dropped++;
  dev_kfree_skb(skb);
  mutex_unlock(&priv->lock);
  netif_start_queue(priv->netdev);
  netdev_err(netdev, "dropped invalid tx packet\n");
}

#ifdef DEBUG
static void sx1280_check_status(struct work_struct *work) {
  struct sx1280_priv *priv = container_of(
    work,
    struct sx1280_priv,
    status_check.work
  );

  struct spi_device *spi = priv->spi;

  u8 status;
  enum sx1280_mode packet_type;
  u8 rx_len;
  u8 rx_start;
  union sx1280_packet_status packet_status;
  u8 rssi_inst;
  u16 irq_status;

  ktime_t start = ktime_get();

  mutex_lock(&priv->lock);

  /* Get all statuses throughout the chip. */
  sx1280_get_status(priv, &status);
  sx1280_get_packet_type(priv, &packet_type);
  sx1280_get_rx_buffer_status(priv, &rx_len, &rx_start);
  sx1280_get_packet_status(priv, &packet_status);
  sx1280_get_rssi_inst(priv, &rssi_inst);
  sx1280_get_irq_status(priv, &irq_status);

  dev_dbg(&spi->dev, "status check:\n");
  dev_dbg(&spi->dev, "  status=0x%02x\n", status);
  dev_dbg(&spi->dev, "  mode=%u\n", packet_type);
  dev_dbg(&spi->dev, "  rx_start=0x%02x, rx_len=%u\n", rx_start, rx_len);
  dev_dbg(&spi->dev, "  pkt_status=%*ph\n", 5, packet_status.raw);
  dev_dbg(&spi->dev, "  rssi_inst=%u\n", rssi_inst);
  dev_dbg(&spi->dev, "  irq=0x%04x\n", irq_status);

  ktime_t end = ktime_get();
  s64 time = ktime_to_us(ktime_sub(end, start));
  dev_dbg(&spi->dev, "  time=%lld us\n", time);

  mutex_unlock(&priv->lock);
  schedule_delayed_work(&priv->status_check, 5 * HZ);
}

#endif

/**
 * Sets the chip into continuous RX mode to listen for packets.
 * @context process & locked
 */
static int sx1280_listen(struct sx1280_priv *priv) {
  int err;

  struct sx1280_packet_params packet_params = { .mode = priv->cfg.mode };
  switch (packet_params.mode) {
  case SX1280_MODE_FLRC:
    packet_params.flrc = priv->cfg.flrc.packet;
    packet_params.flrc.payload_length = SX1280_FLRC_PAYLOAD_LENGTH_MAX;
    break;
  case SX1280_MODE_GFSK:
    packet_params.gfsk = priv->cfg.gfsk.packet;
    packet_params.gfsk.payload_length = SX1280_GFSK_PAYLOAD_LENGTH_MAX;
    break;
  case SX1280_MODE_LORA:
    packet_params.lora = priv->cfg.lora.packet;
    packet_params.lora.payload_length = SX1280_LORA_PAYLOAD_LENGTH_MAX;
    break;
  case SX1280_MODE_RANGING:
    /* TODO */
    break;
  }

  if (
    (err = sx1280_set_packet_params(priv, packet_params))
    || (err = sx1280_set_rx(priv, SX1280_PERIOD_BASE_15_625_US, 0xFFFF))
  ) {
    dev_err(&priv->spi->dev, "failed to transition to listen\n");
  }

  return err;
}

/**
 * Threaded interrupt handler for DIO interrupt requests.
 * @context process
 */
static irqreturn_t sx1280_irq(int irq, void *dev_id) {
  int err;

  struct sx1280_priv *priv = (struct sx1280_priv *) dev_id;
  struct net_device *netdev = priv->netdev;
  struct spi_device *spi = priv->spi;

  mutex_lock(&priv->lock);

  /*
   * The SX1280 can give spurious interrupts during reset, and these should be
   * ignored.
   */
  u16 mask;
  if (
    !priv->initialized
    || (err = sx1280_get_irq_status(priv, &mask))
  ) {
    goto unlock;
  }

  dev_dbg(&spi->dev, "interrupt: mask=0x%04x\n", mask);

  /*
   * Handle interrupts specified by the mask.
   */

  if ((mask & SX1280_IRQ_TX_DONE) || (mask & SX1280_IRQ_RX_TX_TIMEOUT)) {
    /* Update netdev stats. */
    if (mask & SX1280_IRQ_TX_DONE) {
      netdev->stats.tx_packets++;
      netdev->stats.tx_bytes += priv->tx_skb->len;
    } else {
      /*
       * Rx cannot timeout because it's constantly listening.
       * Any timeouts must be Tx, so the packet should be dropped in this case.
       */
      netdev->stats.tx_dropped++;
      netdev_warn(netdev, "tx timeout (packet dropped)\n");
    }

    /* Free the previous Tx packet in preparation for the next. */
    dev_kfree_skb(priv->tx_skb);
    priv->tx_skb = NULL;

    /* Allow for changing of packet parameters if queued from userspace. */
    priv->idle = true;
    wake_up_all(&priv->idle_wait);

    /* Switch back into listening. */
    err = sx1280_listen(priv);

    /* Restart the Tx packet queue. */
    netif_wake_queue(netdev);
  }

  if ((mask & SX1280_IRQ_RX_DONE) || (mask & SX1280_IRQ_SYNC_WORD_VALID)) {
    union sx1280_packet_status status;
    if ((err = sx1280_get_packet_status(priv, &status))) {
      goto rx_error;
    }

    /* TODO: Set the RSSI to be publicly accessible. */
    switch (priv->cfg.mode) {
    case SX1280_MODE_FLRC:
    case SX1280_MODE_GFSK:
      netdev_dbg(
        netdev,
        "rx: rssi_sync=0x%02x, errors=0x%02x, status=0x%02x, sync=0x%02x\n",
        status.gfsk_flrc.rssi_sync,
        status.gfsk_flrc.errors,
        status.gfsk_flrc.status,
        status.gfsk_flrc.sync
      );

      break;
    case SX1280_MODE_LORA:
      netdev_dbg(
        netdev,
        "rx: rssi=%d, snr=%d\n",
        status.lora.rssi_sync,
        status.lora.snr
      );

      break;
    case SX1280_MODE_RANGING:
      netdev_err(priv->netdev, "received packet in ranging mode\n");
      err = -EIO;
      goto rx_error;
    }

    /* Check errors after checking packet status for accurate debugging. */
    /* TODO: maybe change this for efficiency after debugging */
    if (
      (mask & SX1280_IRQ_SYNC_WORD_ERROR)
      || (mask & SX1280_IRQ_HEADER_ERROR)
      || (mask & SX1280_IRQ_CRC_ERROR)
    ) {
      netdev_dbg(netdev, "rx error: mask=0x%04x\n", mask);
      netdev->stats.rx_errors += 1;
      goto rx_error;
    }

    u8 payload_start, payload_len;

    /*
     * Get the start and length of the received packet.
     *
     * The start should always be the same due to how the buffer is partitioned
     * in setup, but length has to be fetched so it might as well use the
     * offset provided.
     */
    err = sx1280_get_rx_buffer_status(priv, &payload_len, &payload_start);
    if (err) {
      goto rx_error;
    }

    /* Allocate an SKB to hold the packet data and pass it to userspace. */
    struct sk_buff *skb = dev_alloc_skb((unsigned int) payload_len);
    if (!skb) {
      netdev_err(netdev, "failed to allocate SKB for RX packet\n");
      goto rx_error;
    }

    /* Read the RX packet data directly into the SKB. */
    void *rx_data = skb_put(skb, (unsigned int) payload_len);
    err = sx1280_read_buffer(
      priv,
      payload_start,
      (u8 *) rx_data,
      (size_t) payload_len
    );

    if (err) {
      goto rx_error;
    }

    netdev_dbg(netdev, "rx: %*ph\n", payload_len, rx_data);

    skb->dev = netdev;
    skb->protocol = eth_type_trans(skb, netdev);
    skb->ip_summed = CHECKSUM_NONE;

    /* Update netdev stats. */
    netdev->stats.rx_packets++;
    netdev->stats.rx_bytes += payload_len;

    netif_rx(skb);
  }

rx_error:

  /* Acknowledge all interrupts. */
  sx1280_clear_irq_status(priv, 0xFFFF);

unlock:
  mutex_unlock(&priv->lock);
  return IRQ_HANDLED;
}

// static int sx1280_parse_dt_lora(struct sx1280_priv *priv) {
//   /* Convenience definitions for conciseness. */
//   struct sx1280_platform_data *pdata = &priv->pdata;
//   struct sx1280_lora_modulation_params *mod = &pdata->lora.modulation;
//   struct sx1280_lora_packet_params *pkt = &pdata->lora.packet;
//   struct device *dev = &priv->spi->dev;
//
//   u32 spreading_factor = 12;
//   u32 bandwidth_khz = 1600;
//   const char *coding_rate = "4/7";
//   bool disable_li;
//   u32 preamble_bits = 8;
//   bool implicit_header;
//   bool disable_crc;
//   bool invert_iq;
//
//   struct device_node *child = of_get_child_by_name(dev->of_node, "lora");
//
//   if (child) {
//     of_property_read_u32(child, "spreading-factor", &spreading_factor);
//     of_property_read_u32(child, "bandwidth-khz", &bandwidth_khz);
//     of_property_read_string(child, "coding-rate", &coding_rate);
//     disable_li = of_property_read_bool(child, "disable-long-interleaving");
//     of_property_read_u32(child, "preamble-bits", &preamble_bits);
//     implicit_header = of_property_read_bool(child, "implicit-header");
//     disable_crc = of_property_read_bool(child, "disable-crc");
//     invert_iq = of_property_read_bool(child, "invert-iq");
//
//     of_node_put(child);
//   }
//
//   if (spreading_factor < 5 || spreading_factor > 12) {
//     dev_err(dev, "Property lora.spreading-factor out of range.\n");
//     return -EINVAL;
//   }
//
//   mod->spreading = spreading_factor << 4;
//
//   switch (bandwidth_khz) {
//   case 1600:
//     mod->bandwidth = SX1280_LORA_BW_1600;
//     break;
//   case 800:
//     mod->bandwidth = SX1280_LORA_BW_800;
//     break;
//   case 400:
//     mod->bandwidth = SX1280_LORA_BW_400;
//     break;
//   case 200:
//     mod->bandwidth = SX1280_LORA_BW_200;
//     break;
//   default:
//     dev_err(dev, "Invalid value for lora.bandwidth-khz.\n");
//     return -EINVAL;
//   }
//
//   /* Coding rate parsing. */
//   if (
//     strlen(coding_rate) != 3
//     || coding_rate[0] != '4'
//     || coding_rate[1] != '/'
//     || coding_rate[2] < '5'
//     || coding_rate[2] > '8'
//   ) {
//     dev_err(dev, "Invalid lora.coding-rate. Must be 4/5, 4/6, 4/7, or 4/8.\n");
//     return -EINVAL;
//   }
//
//   if (disable_li) {
//     mod->coding_rate = coding_rate[2] - '4';
//   } else {
//     switch (coding_rate[2]) {
//     case '5':
//       mod->coding_rate = SX1280_LORA_CR_LI_4_5;
//       break;
//     case '6':
//       mod->coding_rate = SX1280_LORA_CR_LI_4_6;
//       break;
//     case '7':
//       /*
//        * 4/7 LI coding is not available.
//        * Default to using 4/7 coding without LI.
//        */
//       mod->coding_rate = SX1280_LORA_CR_4_7;
//       break;
//     case '8':
//       mod->coding_rate = SX1280_LORA_CR_LI_4_8;
//       break;
//     }
//   }
//
//   /* Preamble length (bits). */
//   u32 pb_mant = preamble_bits;
//   u8 pb_exp = 0;
//
//   /*
//    * Reduce the preamble bit count into a mantissa and exponent, where both
//    * are allowed to be [1..15].
//    */
//   while ((pb_mant & 1) == 0) {
//     pb_mant >>= 1;
//     pb_exp++;
//   }
//
//   if (
//     pb_mant == 0
//     || pb_mant > 15
//     || pb_exp == 0
//     || pb_exp > 15
//   ) {
//     dev_err(dev, "Invalid value for lora.preamble-bits.\n");
//     return -EINVAL;
//   }
//
//   /*
//    * The format used by the SX1280 to encode preamble length is a strange
//    * choice. It uses bits 7-4 to encode the exponent and bits 3-0 for the
//    * mantissa of the preamble length, in bits. Both components must be in the
//    * range [1..15].
//    *
//    * This increases the maximum preamble length to 491,520 over the 255 that
//    * would ordinarily be representable, but 255 is itself an excessively large
//    * preamble size (8 is typical).
//    */
//   pkt->preamble_length = (pb_exp << 4) | (u8) pb_mant;
//
//   /* Header type. */
//   pkt->header_type = implicit_header
//     ? SX1280_IMPLICIT_HEADER
//     : SX1280_EXPLICIT_HEADER;
//
//   /* CRC enabling. */
//   pkt->crc = disable_crc
//     ? SX1280_LORA_CRC_DISABLE
//     : SX1280_LORA_CRC_ENABLE;
//
//   /* IQ inversion. */
//   pkt->invert_iq = invert_iq
//     ? SX1280_LORA_IQ_INVERTED
//     : SX1280_LORA_IQ_STD;
//
//   return 0;
// }

// static int sx1280_parse_dt_ranging(struct sx1280_priv *priv) {
//   return 0;
// }

// static int sx1280_parse_dt(struct sx1280_priv *priv) {
//   int err;
//   struct sx1280_platform_data *pdata = &priv->pdata;
//   struct device *dev = &priv->spi->dev;
//   struct device_node *node = dev->of_node;
//
//   /* Default values for top-level device tree properties. */
//   const char *mode = "gfsk";
//   s32 power_dbm = 0;
//   u32 ramp_time_us = 20;
  // u32 rf_freq_hz = 2400000000;
  // u32 startup_timeout_us = 10000;
  // u32 tx_timeout_us = 1000000;
  //
  // /* Overwrite the default values if they are specified. */
  // of_property_read_string(node, "mode", &mode);
  // of_property_read_s32(node, "power-dbm", &power_dbm);
  // of_property_read_u32(node, "rf-freq-hz", &rf_freq_hz);
  // of_property_read_u32(node, "startup-timeout-us", &startup_timeout_us);
  // of_property_read_u32(node, "tx-timeout-us", &tx_timeout_us);
  //
  // // if (strcmp(mode, "flrc") == 0) {
  // //   pdata->mode = SX1280_MODE_FLRC;
  // // } else if (strcmp(mode, "gfsk") == 0) {
  // //   pdata->mode = SX1280_MODE_GFSK;
  // // } else if (strcmp(mode, "lora") == 0) {
  // //   pdata->mode = SX1280_MODE_LORA;
  // // } else if (strcmp(mode, "ranging") == 0) {
  // //   pdata->mode = SX1280_MODE_RANGING;
  // // } else {
  // //   dev_err(dev, "Invalid value for mode.\n");
  // //   return -EINVAL;
  // // }
  //
  // /* Power */
  // if (power_dbm < -18 || power_dbm > 13) {
  //   dev_err(dev, "Invalid value for power-dbm.\n");
  //   return -EINVAL;
  // }
  //
  // pdata->power = power_dbm + 18;
  //
  //
  // /* Ramp time */
  // if (
  //   ramp_time_us < 2
  //   || ramp_time_us > 20
  //   || ramp_time_us % 2 != 0
  //   || ramp_time_us == 14
  // ) {
  //   dev_err(dev, "Invalid value for ramp-time-us.\n");
  //   return -EINVAL;
  // }
  //
  // if (ramp_time_us <= 12) {
  //   pdata->ramp_time = (ramp_time_us - 2) << 4;
  // } else {
  //   pdata->ramp_time = (ramp_time_us + 8) << 3;
  // }
  //
  // /*
  //  * Convert the Hz frequency into the fixed-point representation that the chip
  //  * natively understands.
  //  */
  // // pdata->rf_freq = SX1280_FREQ_HZ_TO_PLL(rf_freq_hz);
  //
  // /* Startup timeout */
  // pdata->startup_timeout_us = startup_timeout_us;
  //
  // /*
  //  * Convert the timeout and period base into nanoseconds so that an integer
  //  * division can be performed for maximum precision within constraints.
  //  */
  // u64 timeout_ns = (u64) tx_timeout_us * 1000;
  // u64 period_base_ns;
  //
  // if (tx_timeout_us < 1024000) {
  //   pdata->period_base = SX1280_PERIOD_BASE_15_625_US;
  //   period_base_ns = 15625;
  // } else if (tx_timeout_us < 4096000) {
  //   pdata->period_base = SX1280_PERIOD_BASE_62_500_US;
  //   period_base_ns = 62500;
  // } else if (tx_timeout_us < 6553600) {
  //   pdata->period_base = SX1280_PERIOD_BASE_1_MS;
  //   period_base_ns = 1000000;
  // } else if (tx_timeout_us < 262144000) {
  //   pdata->period_base = SX1280_PERIOD_BASE_4_MS;
  //   period_base_ns = 4000000;
  // } else {
  //   dev_err(dev, "Invalid value for timeout-us.\n");
  //   return -EINVAL;
  // }
  //
  // /*
  //  * Perform a ceiling division to determine the cycle count.
  //  *
  //  * The timeout specified is always a minimum bound on the timeout that could
  //  * also be affected by outside factors such as kernel timing, timer precision,
  //  * etc. When a timeout is specified, it's crucial that the driver guarantees
  //  * that _at least_ that amount of time has passed before timing out.
  //  */
  // pdata->period_base_count = (u16) (timeout_ns / period_base_ns)
  //   + (timeout_ns % period_base_ns != 0);
  //
  // /*
  //  * Parse all sub-nodes of the device tree, containing the properties
  //  * configuring each of the different packet modes of the SX1280.
  //  */
  // if (
  //   (err = sx1280_parse_dt_flrc(priv))
//     || (err = sx1280_parse_dt_gfsk(priv))
//     || (err = sx1280_parse_dt_lora(priv))
//     || (err = sx1280_parse_dt_ranging(priv))
//   ) {
//     return err;
//   }
//
//   return 0;
// }

/**
 * Parses busy GPIO and DIO GPIOs.
 * @param priv - The internal SX1280 driver structure.
 * @param dt - Whether a device tree configuration or platform data is used.
 */
static int sx1280_setup_gpios(struct sx1280_priv *priv) {
  int err;
  struct device *dev = &priv->spi->dev;

  /*
   * 1. Configure the busy pin GPIO.
   * 2. Configure the DIO1, DIO2, DIO3 GPIOs.
   */

  struct sx1280_platform_data *pdata = dev_get_platdata(dev);
  priv->dio = NULL;

  if (!pdata) {
    /*
     * If a device tree is used, then the GPIOs are directly registered with the
     * SPI device and freed upon the SPI device being unregistered.
     */

    priv->busy = devm_gpiod_get(dev, "busy", GPIOD_IN);
    if (IS_ERR(priv->busy)) {
      dev_err(dev, "Failed to configure GPIO for busy pin.\n");
      return PTR_ERR(priv->busy);
    }

    for (int i = 0; i < 3; i++) {
      struct gpio_desc *dio = devm_gpiod_get_index(dev, "dio", i, GPIOD_IN);

      if (IS_ERR(dio)) {
        err = PTR_ERR(dio);

        if (err == -ENOENT) {
          dev_warn(dev, "Optional DIO%d not specified.\n", i + 1);
        } else {
          dev_err(dev, "Failed to configure GPIO for DIO%d.\n", i + 1);
          return err;
        }
      } else {
        priv->dio_index = i + 1;
        priv->dio = dio;
        break;
      }
    }

    priv->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(priv->reset)) {
      dev_err(dev, "Failed to configure GPIO for the reset pin.\n");
      return PTR_ERR(priv->reset);
    }
  } else {
    /*
     * If platform data is used, the GPIOs are passed as integers.
     * This is not the preferred way for GPIOs to be specified in device trees
     * anymore, so the conversion of GPIOs into descriptors must be done
     * separately.
     */

    err = -EINVAL;

    priv->busy = gpio_to_desc(pdata->busy_gpio);
    if (!priv->busy || (err = gpiod_direction_input(priv->busy))) {
      dev_err(dev, "Failed to configure GPIO for busy pin.\n");
      return err;
    }

    for (int i = 0; i < 3; i++) {
      int gpio_num = pdata->dio_gpios[i];
      if (gpio_num < 0) {
        continue;
      }

      struct gpio_desc *dio = gpio_to_desc((unsigned int) gpio_num);

      if (!dio || (err = gpiod_direction_input(dio))) {
        dev_err(dev, "Failed to configure GPIO for DIO%d.\n", i);
        return err;
      }

      priv->dio_index = i + 1;
      priv->dio = dio;
      break;
    }

    priv->reset = gpio_to_desc(pdata->reset_gpio);
    if (!priv->reset || (err = gpiod_direction_output(priv->reset, 1))) {
      dev_err(dev, "Failed to configure GPIO for reset.\n");
      return err;
    }
  }

  /* Check that at least one DIO was set. */
  if (!priv->dio) {
    dev_err(dev, "no DIOs are set in the configuration\n");
    return -EINVAL;
  }

  priv->irq = gpiod_to_irq(priv->dio);
  if ((err = priv->irq) < 0) {
    dev_err(dev, "failed to register IRQ for DIO\n");
    return err;
  }

  /*
   * Register the DIO IRQs to their interrupt handlers.
   * TODO: Change this to split interrupts across all DIOs.
   */
  err = devm_request_threaded_irq(
    dev,
    priv->irq,
    NULL,
    sx1280_irq,
    IRQF_TRIGGER_RISING | IRQF_ONESHOT,
    "sx1280_irq",
    priv
  );

  if (err) {
    dev_err(dev, "Failed to set IRQ handler.\n");
    return err;
  }

  return 0;
}

/**
 * Performs a hardware reset by toggling the NRESET pin and waiting for BUSY.
 * @param priv - The internal SX1280 driver structure.
 */
static int sx1280_reset(struct sx1280_priv *priv) {
  int err;

  netdev_dbg(priv->netdev, "resetting hardware\n");

  /* Toggle NRESET. */
  gpiod_set_value_cansleep(priv->reset, 1);
  usleep_range(100, 150);
  gpiod_set_value_cansleep(priv->reset, 0);

#ifdef DEBUG
  ktime_t start = ktime_get();
#endif

  /* Wait for BUSY = 0. */
  if ((err = sx1280_wait_busy(priv))) {
    netdev_err(priv->netdev, "failed to reset, timeout exceeded\n");
    return err;
  }

#ifdef DEBUG
  ktime_t end = ktime_get();
  s64 reset_time = ktime_to_us(ktime_sub(end, start));
  netdev_dbg(priv->netdev, "reset completed in %lld us\n", reset_time);
#endif

  return 0;
}

/**
 * Performs the chip setup.
 * @context - process & pre-lock
 * @param priv - The internal SX1280 driver structure.
 */
static int sx1280_setup(struct sx1280_priv *priv) {
  struct spi_device *spi = priv->spi;
  struct sx1280_config *cfg = &priv->cfg;
  int err;

  dev_dbg(&spi->dev, "starting setup\n");

  /* Reset the chip and check its status after reset. */
  u8 status;
  if (
    (err = sx1280_reset(priv))
    || (err = sx1280_get_status(priv, &status))
  ) {
    return err;
  }

  dev_dbg(&spi->dev, "status: 0x%02x\n", status);

  /* Extract circuit mode and command status and check for valid values. */
  u8 circuit_mode = FIELD_GET(SX1280_STATUS_CIRCUIT_MODE_MASK, status);
  u8 command_status = FIELD_GET(SX1280_STATUS_COMMAND_STATUS_MASK, status);

  if (circuit_mode != SX1280_CIRCUIT_MODE_STDBY_RC) {
    dev_err(&spi->dev, "unexpected circuit mode: 0x%02x.\n", circuit_mode);
    return -EIO;
  } else if (
    command_status == SX1280_COMMAND_STATUS_TIMEOUT
    || command_status == SX1280_COMMAND_STATUS_PROCESSING_ERROR
    || command_status == SX1280_COMMAND_STATUS_EXEC_FAILURE
  ) {
    dev_err(&spi->dev, "unexpected command status: 0x%02x\n", command_status);
    return -EIO;
  }

  /*
   * Extract the modulation and packet params from the platform data, depending
   * on the mode that the chip is being commanded into.
   */
  struct sx1280_modulation_params mod_params = {
    .mode = SX1280_MODE_GFSK,
    .gfsk = cfg->gfsk.modulation,
  };

  if (
    (err = sx1280_set_rf_frequency(priv, cfg->freq))

    /*
     * Set the Tx and Rx buffer base addresses to 0x0.
     * This allows the chip to use the full 256-byte data buffer.
     * The size of the data buffer also restricts the MTU to 256 bytes.
     *
     * Since the chip supports half-duplex, the data must be sent/read before
     * performing another operation, but otherwise will not be overwritten.
     */
    || (err = sx1280_set_buffer_base_address(priv, 0x0, 0x0))
    || (err = sx1280_set_modulation_params(priv, mod_params))
    || (
      err = sx1280_write_register(
        priv,
        SX1280_REG_SYNC_ADDRESS_1_BYTE_4,
        (u8 *) cfg->sync_words,
        15
      )
    ) || (
      err = sx1280_write_register(
        priv,
        SX1280_REG_CRC_POLYNOMIAL_DEFINITION_MSB,
        cfg->gfsk.crc_polynomial,
        2
      )
    ) || (
      err = sx1280_write_register(
        priv,
        SX1280_REG_CRC_MSB_INITIAL_VALUE,
        cfg->gfsk.crc_seed,
        2
      )
    ) || (err = sx1280_set_tx_params(priv, cfg->power, cfg->ramp_time))
    // || (err = sx1280_set_auto_fs(priv, true)) TODO
  ) {
    return err;
  }

  priv->idle = true;
  return 0;
}

static const struct net_device_ops sx1280_netdev_ops = {
  .ndo_open = sx1280_open,
  .ndo_start_xmit = sx1280_xmit,
  .ndo_stop = sx1280_stop,
};

/**
 * Acquires the shared lock by waiting until idle, so that the chip
 * configuration can be changed. The wait times out after 1 second.
 *
 * NOTE: This function acquires the lock but does not release it unless it fails
 * and returns an error code. The shared lock must be released by the caller.
 *
 * @context - process & unlocked
 */
static int sx1280_acquire_idle(struct sx1280_priv *priv, bool locked) {
  int err;

  if (!locked && (err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  while (!priv->idle) {
    mutex_unlock(&priv->lock);

    if (
      (err = wait_event_interruptible(priv->idle_wait, priv->idle))
      || (err = mutex_lock_interruptible(&priv->lock))
    ) {
      return err;
    }
  }

  return 0;
}

/**
 * Acquires the shared lock no matter the mode, but additionally waits for idle
 * if the mode is the one specified.
 *
 * Note that the mode must still be checked after this function exits, as the
 * mode may have changed since released and re-acquiring the mutex while waiting
 * for idle.
 */
static int sx1280_acquire_idle_if_mode(
  struct sx1280_priv *priv,
  enum sx1280_mode mode,
  bool locked
) {
  int err;

  /* The mutex must be locked before accessing priv->cfg.mode. */
  if (
    (!locked && (err = mutex_lock_interruptible(&priv->lock)))
    || (priv->cfg.mode == mode && (err = sx1280_acquire_idle(priv, true)))
  ) {
    return err;
  }

  return 0;
}

/**
 * Acquires the shared lock by waiting until idle before switching into standby
 * mode, so that the chip configuration can be changed. The wait times out after
 * 1 second.
 *
 * NOTE: This function acquires the lock but does not release it unless it fails
 * and returns an error code. The shared lock must be released by the caller.
 *
 * @context - process & unlocked
 */
static int sx1280_acquire_stdby(struct sx1280_priv *priv, bool locked) {
  int err;

  if ((err = sx1280_acquire_idle(priv, locked))) {
    return err;
  }

  if ((err = sx1280_set_standby(priv, SX1280_STDBY_XOSC))) {
    mutex_unlock(&priv->lock);
    return err;
  }

  return 0;
}

/*********/
/* sysfs */
/*********/

static ssize_t mode_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  const char *mode_str = sx1280_mode_etos(priv->cfg.mode);
  int count = sprintf(buf, "%s\n", mode_str);

  mutex_unlock(&priv->lock);
  return count;
}

static ssize_t mode_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  enum sx1280_mode new_mode;
  if (sysfs_streq(buf, "flrc")) {
    new_mode = SX1280_MODE_FLRC;
  } else if (sysfs_streq(buf, "gfsk")) {
    new_mode = SX1280_MODE_GFSK;
  } else if (sysfs_streq(buf, "lora")) {
    new_mode = SX1280_MODE_LORA;
  } else if (sysfs_streq(buf, "ranging")) {
    new_mode = SX1280_MODE_RANGING;
  } else {
    return -EINVAL;
  }

  if ((err = sx1280_acquire_stdby(priv, false))) {
    return err;
  }

  err = sx1280_set_packet_type(priv, new_mode);
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t tx_power_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  int count = sprintf(buf, "%d\n", (s32) priv->cfg.power - 18);
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

/**
 *
 * @context - process
 */
static ssize_t tx_power_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  s32 power_dbm;
  if ((err = kstrtos32(buf, 10, &power_dbm))) {
    return err;
  }

  if (power_dbm < -18 || power_dbm > 13) {
    return -EINVAL;
  }

  if ((err = sx1280_acquire_idle(priv, false))) {
    return err;
  }

  priv->cfg.power = (u32) (power_dbm + 18);
  mutex_unlock(&priv->lock);
  return count;
}

/**
 * @context - process
 */
static ssize_t busy_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  int value = gpiod_get_value_cansleep(priv->busy);
  if (value < 0) {
    return value;
  }

  return sprintf(buf, "%d\n", value);
}

static ssize_t ramp_time_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }
  
  int ramp_time_us = sx1280_ramp_time_etoi(priv->cfg.ramp_time);
  int count = sprintf(buf, "%d\n", ramp_time_us);

  mutex_unlock(&priv->lock);
  return count;
}

static ssize_t ramp_time_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  u32 ramp_time_us;
  if ((err = kstrtou32(buf, 10, &ramp_time_us))) {
    return err;
  }

  int ramp_time = sx1280_ramp_time_itoe(ramp_time_us);
  if (ramp_time < 0) {
    return ramp_time;
  }

  if ((err = sx1280_acquire_idle(priv, false))) {
    return err;
  }

  priv->cfg.ramp_time = (enum sx1280_ramp_time) ramp_time;
  mutex_unlock(&priv->lock);
  return count;
}

static ssize_t frequency_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  u32 freq_hz = SX1280_FREQ_PLL_TO_HZ(priv->cfg.freq);
  int count = sprintf(buf, "%u\n", freq_hz);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t frequency_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  u32 freq_hz;
  if ((err = kstrtou32(buf, 10, &freq_hz))) {
    return err;
  }

  if (freq_hz < 2400000000 || freq_hz > 2500000000) {
    return -EINVAL;
  }

  u32 freq_pll = SX1280_FREQ_HZ_TO_PLL(freq_hz);
  
  if ((err = sx1280_acquire_idle(priv, false))) {
    return err;
  }

  if ((err = sx1280_set_rf_frequency(priv, freq_pll))) {
    goto fail;
  }

  priv->cfg.freq = freq_pll;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_bandwidth_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  u8 brbw = priv->cfg.gfsk.modulation.bitrate_bandwidth;
  int bandwidth_hz = sx1280_fsk_bandwidth_etoi(brbw);
  if (bandwidth_hz < 0) {
    goto fail;
  }

  int count = sprintf(buf, "%d\n", bandwidth_hz);

fail:
  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_bandwidth_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  u32 bandwidth_hz;
  if (
    (err = kstrtou32(buf, 10, &bandwidth_hz))
    || (err = mutex_lock_interruptible(&priv->lock))
  ) {
    return err;
  }

  u8 brbw = priv->cfg.gfsk.modulation.bitrate_bandwidth;
  int bitrate = sx1280_fsk_bitrate_etoi(brbw);
  if (bitrate < 0) {
    err = bitrate;
    goto fail;
  }

  /*
   * Pick the bandwidth-bitrate combination that must respect the bandwidth just
   * chosen, and attempts to respect the existing bitrate. If the bitrate cannot
   * be respected, then pick the highest bitrate for that bandwidth.
   */
  enum sx1280_fsk_bitrate_bandwidth new_brbw;
  switch (bandwidth_hz) {
  case 2400000:
    switch (bitrate) {
    case 1600000:
      new_brbw = SX1280_FSK_BR_1_600_BW_2_4;
      break;
    case 1000000:
      new_brbw = SX1280_FSK_BR_1_000_BW_2_4;
      break;
    case 800000:
      new_brbw = SX1280_FSK_BR_0_800_BW_2_4;
      break;
    default:
      new_brbw = SX1280_FSK_BR_2_000_BW_2_4;
      break;
    }

    break;
  case 1200000:
    switch (bitrate) {
    case 800000:
      new_brbw = SX1280_FSK_BR_0_800_BW_1_2;
      break;
    case 500000:
      new_brbw = SX1280_FSK_BR_0_500_BW_1_2;
      break;
    case 400000:
      new_brbw = SX1280_FSK_BR_0_400_BW_1_2;
      break;
    default:
      new_brbw = SX1280_FSK_BR_1_000_BW_1_2;
      break;
    }

    break;
  case 600000:
    switch (bitrate) {
    case 400000:
      new_brbw = SX1280_FSK_BR_0_400_BW_0_6;
      break;
    case 250000:
      new_brbw = SX1280_FSK_BR_0_250_BW_0_6;
      break;
    default:
      new_brbw = SX1280_FSK_BR_0_500_BW_0_6;
      break;
    }

    break;
  case 300000:
    new_brbw = bitrate == 125000
      ? SX1280_FSK_BR_0_125_BW_0_3
      : SX1280_FSK_BR_0_250_BW_0_3;

    break;
  default:
    err = -EINVAL;
    goto fail;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, true))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_modulation_params mod_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.modulation,
    };
    mod_params.gfsk.bitrate_bandwidth = new_brbw;

    if ((err = sx1280_set_modulation_params(priv, mod_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.modulation.bitrate_bandwidth = new_brbw;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_bitrate_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  u8 brbw = priv->cfg.gfsk.modulation.bitrate_bandwidth;
  int bitrate = sx1280_fsk_bitrate_etoi(brbw);
  int count = sprintf(buf, "%d\n", bitrate);

  mutex_unlock(&priv->lock);
  return err ? err : count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_bitrate_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  u32 bitrate;
  if (
    (err = kstrtou32(buf, 10, &bitrate))
    || (err = mutex_lock_interruptible(&priv->lock))
  ) {
    return err;
  }

  enum sx1280_fsk_bitrate_bandwidth brbw =
    priv->cfg.gfsk.modulation.bitrate_bandwidth;
  int bandwidth_hz = sx1280_fsk_bandwidth_etoi(brbw);

  /*
   * Pick the bandwidth-bitrate combination that must respect the bitrate just
   * chosen, and attempts to respect the existing bandwidth. If the bandwidth
   * cannot be respected, then pick the highest bandwidth for that bitrate.
   */
  enum sx1280_fsk_bitrate_bandwidth new_brbw;
  switch (bitrate) {
  case 2000000:
    new_brbw = SX1280_FSK_BR_2_000_BW_2_4;
    break;
  case 1600000:
    new_brbw = SX1280_FSK_BR_1_600_BW_2_4;
    break;
  case 1000000:
    new_brbw = bandwidth_hz == 1200000
      ? SX1280_FSK_BR_1_000_BW_1_2
      : SX1280_FSK_BR_1_000_BW_2_4;
    break;
  case 800000:
    new_brbw = bandwidth_hz == 1200000
      ? SX1280_FSK_BR_0_800_BW_1_2
      : SX1280_FSK_BR_0_800_BW_2_4;
    break;
  case 500000:
    new_brbw = bandwidth_hz == 600000
      ? SX1280_FSK_BR_0_500_BW_0_6
      : SX1280_FSK_BR_0_500_BW_1_2;
    break;
  case 400000:
    new_brbw = bandwidth_hz == 600000
      ? SX1280_FSK_BR_0_400_BW_0_6
      : SX1280_FSK_BR_0_400_BW_1_2;
    break;
  case 250000:
    new_brbw = bandwidth_hz == 300000
      ? SX1280_FSK_BR_0_250_BW_0_3
      : SX1280_FSK_BR_0_250_BW_0_6;
    break;
  case 125000:
    new_brbw = SX1280_FSK_BR_0_125_BW_0_3;
    break;
  default:
    err = -EINVAL;
    goto fail;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, true))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_modulation_params mod_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.modulation,
    };
    mod_params.gfsk.bitrate_bandwidth = new_brbw;

    if ((err = sx1280_set_modulation_params(priv, mod_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.modulation.bitrate_bandwidth = new_brbw;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_bandwidth_time_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  const char *bt = sx1280_bandwidth_time_etos(
    priv->cfg.gfsk.modulation.bandwidth_time
  );

  int count = sprintf(buf, "%s\n", bt);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_bandwidth_time_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  u8 bt;
  if (sysfs_streq(buf, "off")) {
    bt = SX1280_BT_OFF;
  } else if (sysfs_streq(buf, "0.5")) {
    bt = SX1280_BT_0_5;
  } else if (sysfs_streq(buf, "1.0")) {
    bt = SX1280_BT_1_0;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_modulation_params mod_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.modulation,
    };
    mod_params.gfsk.bandwidth_time = bt;

    if ((err = sx1280_set_modulation_params(priv, mod_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.modulation.bandwidth_time = bt;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_crc_bytes_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  int crc_bytes = sx1280_radio_crc_etoi(priv->cfg.gfsk.packet.crc_length);
  int count = sprintf(buf, "%d\n", crc_bytes);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_crc_bytes_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  u32 crc_bytes;
  if ((err = kstrtou32(buf, 10, &crc_bytes))) {
    return err;
  }

  int crc_length = sx1280_radio_crc_itoe(crc_bytes);
  if (crc_length < 0) {
    return crc_length;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_packet_params packet_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.packet,
    };
    packet_params.gfsk.crc_length = crc_length;

    if ((err = sx1280_set_packet_params(priv, packet_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.packet.crc_length = crc_length;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_crc_polynomial_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  int count = sprintf(buf, "%*phN\n", 2, priv->cfg.gfsk.crc_polynomial);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_crc_polynomial_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if (count != 5 && count != 4) {
    return -EINVAL;
  }

  u8 crc_polynomial[2];
  if ((err = hex2bin(crc_polynomial, buf, 2))) {
    return err;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  err = sx1280_write_register(
    priv,
    SX1280_REG_CRC_POLYNOMIAL_DEFINITION_MSB,
    crc_polynomial,
    2
  );
  if (err) {
    goto fail;
  }

  memcpy(priv->cfg.gfsk.crc_polynomial, crc_polynomial, 2);
  
fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_crc_seed_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  int count = sprintf(buf, "%*phN\n", 2, priv->cfg.gfsk.crc_seed);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_crc_seed_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if (count != 5 && count != 4) {
    return -EINVAL;
  }

  u8 crc_seed[2];
  if ((err = hex2bin(crc_seed, buf, 2))) {
    return err;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  err = sx1280_write_register(
    priv,
    SX1280_REG_CRC_POLYNOMIAL_DEFINITION_MSB,
    crc_seed,
    2
  );
  if (err) {
    goto fail;
  }

  memcpy(priv->cfg.gfsk.crc_seed, crc_seed, 2);

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_whitening_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  bool whitening = priv->cfg.gfsk.packet.whitening == SX1280_WHITENING_ENABLE;
  int count = sprintf(buf, "%d\n", whitening);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_whitening_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);
  
  bool whitening_bool;
  if ((err = kstrtobool(buf, &whitening_bool))) {
    return err;
  }

  enum sx1280_whitening whitening = whitening_bool
    ? SX1280_WHITENING_ENABLE
    : SX1280_WHITENING_DISABLE;

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_packet_params packet_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.packet,
    };
    packet_params.gfsk.whitening = whitening;

    if ((err = sx1280_set_packet_params(priv, packet_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.packet.whitening = whitening;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_modulation_index_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  const char *mod_index = sx1280_modulation_index_etos(
    priv->cfg.gfsk.modulation.modulation_index
  );

  int count = sprintf(buf, "%s\n", mod_index);

  mutex_unlock(&priv->lock);
  return count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_modulation_index_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);
  
  u8 mod_index;
  if (sysfs_streq(buf, "0.35")) {
    mod_index = SX1280_MOD_IND_0_35;
  } else if (sysfs_streq(buf, "0.50")) {
    mod_index = SX1280_MOD_IND_0_50;
  } else if (sysfs_streq(buf, "0.75")) {
    mod_index = SX1280_MOD_IND_0_75;
  } else if (sysfs_streq(buf, "1.00")) {
    mod_index = SX1280_MOD_IND_1_00;
  } else if (sysfs_streq(buf, "1.25")) {
    mod_index = SX1280_MOD_IND_1_25;
  } else if (sysfs_streq(buf, "1.50")) {
    mod_index = SX1280_MOD_IND_1_50;
  } else if (sysfs_streq(buf, "1.75")) {
    mod_index = SX1280_MOD_IND_1_75;
  } else if (sysfs_streq(buf, "2.00")) {
    mod_index = SX1280_MOD_IND_2_00;
  } else if (sysfs_streq(buf, "2.25")) {
    mod_index = SX1280_MOD_IND_2_25;
  } else if (sysfs_streq(buf, "2.50")) {
    mod_index = SX1280_MOD_IND_2_50;
  } else if (sysfs_streq(buf, "2.75")) {
    mod_index = SX1280_MOD_IND_2_75;
  } else if (sysfs_streq(buf, "3.00")) {
    mod_index = SX1280_MOD_IND_3_00;
  } else if (sysfs_streq(buf, "3.25")) {
    mod_index = SX1280_MOD_IND_3_25;
  } else if (sysfs_streq(buf, "3.50")) {
    mod_index = SX1280_MOD_IND_3_50;
  } else if (sysfs_streq(buf, "3.75")) {
    mod_index = SX1280_MOD_IND_3_75;
  } else if (sysfs_streq(buf, "4.00")) {
    mod_index = SX1280_MOD_IND_4_00;
  } else {
    return -EINVAL;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_modulation_params mod_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.modulation,
    };
    mod_params.gfsk.modulation_index = mod_index;

    if ((err = sx1280_set_modulation_params(priv, mod_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.modulation.modulation_index = mod_index;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_preamble_bits_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  int bits = sx1280_preamble_etoi(priv->cfg.gfsk.packet.preamble_length);
  int count = sprintf(buf, "%d\n", bits);

  mutex_unlock(&priv->lock);
  return err ? err : count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_preamble_bits_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  int preamble_bits;
  if ((err = kstrtoint(buf, 10, &preamble_bits))) {
    return err;
  }

  int preamble_enum = sx1280_preamble_itoe(preamble_bits);
  if (preamble_enum < 0) {
    return preamble_enum;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_packet_params packet_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.packet,
    };
    packet_params.gfsk.preamble_length = preamble_enum;

    if ((err = sx1280_set_packet_params(priv, packet_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.packet.preamble_length = preamble_enum;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_sync_word_length_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  int bytes;
  switch (priv->cfg.gfsk.packet.sync_word_length) {
  case SX1280_SYNC_WORD_LEN_1_B: bytes = 1; break;
  case SX1280_SYNC_WORD_LEN_2_B: bytes = 2; break;
  case SX1280_SYNC_WORD_LEN_3_B: bytes = 3; break;
  case SX1280_SYNC_WORD_LEN_4_B: bytes = 4; break;
  case SX1280_SYNC_WORD_LEN_5_B: bytes = 5; break;
  }

  int count = sprintf(buf, "%d\n", bytes);

  mutex_unlock(&priv->lock);
  return err ? err : count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_sync_word_length_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  int sync_word_bytes;
  if ((err = kstrtoint(buf, 10, &sync_word_bytes))) {
    return err;
  }

  enum sx1280_gfsk_sync_word_length sync_word_length;
  switch (sync_word_bytes) {
  case 1: sync_word_length = SX1280_SYNC_WORD_LEN_1_B; break;
  case 2: sync_word_length = SX1280_SYNC_WORD_LEN_2_B; break;
  case 3: sync_word_length = SX1280_SYNC_WORD_LEN_3_B; break;
  case 4: sync_word_length = SX1280_SYNC_WORD_LEN_4_B; break;
  case 5: sync_word_length = SX1280_SYNC_WORD_LEN_5_B; break;
  default: return -EINVAL;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_packet_params packet_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.packet,
    };
    packet_params.gfsk.sync_word_length = sync_word_length;

    if ((err = sx1280_set_packet_params(priv, packet_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.packet.sync_word_length = sync_word_length;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

static ssize_t gfsk_sync_word_match_show(
  struct device *dev,
  struct device_attribute *attr,
  char *buf
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  if ((err = mutex_lock_interruptible(&priv->lock))) {
    return err;
  }

  const char *mask;
  switch (priv->cfg.gfsk.packet.sync_word_match) {
  case SX1280_RADIO_SELECT_SYNCWORD_OFF  : mask = "000"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_1    : mask = "100"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_2    : mask = "010"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_1_2  : mask = "110"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_3    : mask = "001"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_1_3  : mask = "101"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_2_3  : mask = "011"; break;
  case SX1280_RADIO_SELECT_SYNCWORD_1_2_3: mask = "111"; break;
  }

  int count = sprintf(buf, "%s\n", mask);

  mutex_unlock(&priv->lock);
  return err ? err : count;
}

/**
 *
 * @context - process
 */
static ssize_t gfsk_sync_word_match_store(
  struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count
) {
  int err;
  struct net_device *netdev = to_net_dev(dev);
  struct sx1280_priv *priv = netdev_priv(netdev);

  enum sx1280_sync_word_match sync_word_match;
  if (sysfs_streq(buf, "000") || sysfs_streq(buf, "off")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_OFF;
  } else if (sysfs_streq(buf, "100")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_1;
  } else if (sysfs_streq(buf, "010")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_2;
  } else if (sysfs_streq(buf, "110")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_1_2;
  } else if (sysfs_streq(buf, "001")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_3;
  } else if (sysfs_streq(buf, "101")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_1_3;
  } else if (sysfs_streq(buf, "011")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_2_3;
  } else if (sysfs_streq(buf, "111")) {
    sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_1_2_3;
  } else {
    return -EINVAL;
  }

  if ((err = sx1280_acquire_idle_if_mode(priv, SX1280_MODE_GFSK, false))) {
    return err;
  }

  if (priv->cfg.mode == SX1280_MODE_GFSK) {
    struct sx1280_packet_params packet_params = {
      .mode = SX1280_MODE_GFSK,
      .gfsk = priv->cfg.gfsk.packet,
    };
    packet_params.gfsk.sync_word_match = sync_word_match;

    if ((err = sx1280_set_packet_params(priv, packet_params))) {
      goto fail;
    }
  }

  priv->cfg.gfsk.packet.sync_word_match = sync_word_match;

fail:
  mutex_unlock(&priv->lock);
  return err ? err : count;
}

/* GFSK-specific device attributes */
static struct device_attribute dev_attr_gfsk_bandwidth =
  __ATTR(bandwidth, 0644, gfsk_bandwidth_show, gfsk_bandwidth_store);
static struct device_attribute dev_attr_gfsk_bandwidth_time =
  __ATTR(bandwidth_time, 0644, gfsk_bandwidth_time_show, gfsk_bandwidth_time_store);
static struct device_attribute dev_attr_gfsk_bitrate =
  __ATTR(bitrate, 0644, gfsk_bitrate_show, gfsk_bitrate_store);
static struct device_attribute dev_attr_gfsk_crc_bytes =
  __ATTR(crc_bytes, 0644, gfsk_crc_bytes_show, gfsk_crc_bytes_store);
static struct device_attribute dev_attr_gfsk_crc_polynomial =
  __ATTR(crc_polynomial, 0644, gfsk_crc_polynomial_show, gfsk_crc_polynomial_store);
static struct device_attribute dev_attr_gfsk_crc_seed =
  __ATTR(crc_seed, 0644, gfsk_crc_seed_show, gfsk_crc_seed_store);
static struct device_attribute dev_attr_gfsk_modulation_index =
  __ATTR(modulation_index, 0644, gfsk_modulation_index_show, gfsk_modulation_index_store);
static struct device_attribute dev_attr_gfsk_preamble_bits =
  __ATTR(preamble_bits, 0644, gfsk_preamble_bits_show, gfsk_preamble_bits_store);
static struct device_attribute dev_attr_gfsk_sync_word_length =
  __ATTR(sync_word_length, 0644, gfsk_sync_word_length_show, gfsk_sync_word_length_store);
static struct device_attribute dev_attr_gfsk_sync_word_match =
  __ATTR(sync_word_match, 0644, gfsk_sync_word_match_show, gfsk_sync_word_match_store);
static struct device_attribute dev_attr_gfsk_whitening =
  __ATTR(whitening, 0644, gfsk_whitening_show, gfsk_whitening_store);

static struct attribute *sx1280_flrc_attrs[] = {
  NULL,
};

static struct attribute_group sx1280_flrc_group = {
  .attrs = sx1280_flrc_attrs,
  .name = "flrc",
};

static struct attribute *sx1280_gfsk_attrs[] = {
  &dev_attr_gfsk_bandwidth.attr,
  &dev_attr_gfsk_bandwidth_time.attr,
  &dev_attr_gfsk_bitrate.attr,
  &dev_attr_gfsk_crc_bytes.attr,
  &dev_attr_gfsk_crc_polynomial.attr,
  &dev_attr_gfsk_crc_seed.attr,
  &dev_attr_gfsk_modulation_index.attr,
  &dev_attr_gfsk_preamble_bits.attr,
  &dev_attr_gfsk_sync_word_length.attr,
  &dev_attr_gfsk_sync_word_match.attr,
  &dev_attr_gfsk_whitening.attr,
  NULL,
};

static struct attribute_group sx1280_gfsk_group = {
  .attrs = sx1280_gfsk_attrs,
  .name = "gfsk",
};

static struct attribute *sx1280_lora_attrs[] = {
  NULL,
};

static struct attribute_group sx1280_lora_group = {
  .attrs = sx1280_lora_attrs,
  .name = "lora",
};

static DEVICE_ATTR_RO(busy);
static DEVICE_ATTR_RW(frequency);
static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_RW(tx_power);

static struct attribute *sx1280_attrs[] = {
  &dev_attr_busy.attr,
  &dev_attr_frequency.attr,
  &dev_attr_mode.attr,
  &dev_attr_tx_power.attr,
  NULL,
};

static const struct attribute_group sx1280_attr_group = {
  .attrs = sx1280_attrs
};

static const struct attribute_group *sx1280_groups[] = {
  &sx1280_attr_group,
  &sx1280_flrc_group,
  &sx1280_gfsk_group,
  &sx1280_lora_group,
  NULL,
};

/**
 * The core probe function for the SX1280.
 * @param spi - The SPI device wired to the SX1280.
 * @returns 0 on success, error code otherwise.
 */
static int sx1280_probe(struct spi_device *spi) {
  int err;

  /* Allocate a new net device (without registering, yet). */
  struct net_device *netdev = alloc_etherdev(sizeof(struct sx1280_priv));
  if (!netdev) {
    dev_err(&spi->dev, "failed to alloc etherdev\n");
    return -ENOMEM;
  }

  /* Set properties of the net device. */
  netdev->netdev_ops = &sx1280_netdev_ops;
  netdev->mtu = SX1280_GFSK_PAYLOAD_LENGTH_MAX;
  strcpy(netdev->name, "radio%d");
  SET_NETDEV_DEV(netdev, &spi->dev);
  eth_hw_addr_random(netdev);

  /* Create and register the priv structure. */
  struct sx1280_priv *priv = netdev_priv(netdev);
  priv->cfg = sx1280_default_config;
  priv->initialized = false;
  priv->netdev = netdev;
  priv->spi = spi;
  mutex_init(&priv->lock);
  init_waitqueue_head(&priv->idle_wait);

  /*
   * Parse GPIOs according to whether a device tree or platform data is used.
   */
  if ((err = sx1280_setup_gpios(priv))) {
    dev_err(&spi->dev, "failed to configure GPIOs\n");
    goto error_free;
  }

  /* Define SPI settings according to SX1280 datasheet. */
  spi_set_drvdata(spi, priv);
  spi->bits_per_word = 8;
  spi->max_speed_hz = 5000000;  /* TODO: change to 18 MHz */
  spi->mode = 0;                 /* CPOL = 0, CPHA = 0 */

  /* Apply the SPI settings above and handle errors. */
  if ((err = spi_setup(spi))) {
    dev_err(&spi->dev, "failed to apply SPI settings\n");
    goto error_free;
  }

  if ((err = sx1280_setup(priv))) {
    dev_err(&spi->dev, "failed to set up the SX1280.\n");
    goto error_free;
  }

  /* Map all IRQs to the in-use DIO. */
  u16 irq_mask[3] = { 0 };
  irq_mask[priv->dio_index - 1] = 0xFFFF;
  if ((err = sx1280_set_dio_irq_params(priv, 0xFFFF, irq_mask))) {
    goto error_free;
  }

  netdev_dbg(netdev, "configured DIO%d as IRQ", priv->dio_index);

  /* Initialize the work queue and work items for packet transmission. */
  priv->xmit_queue = alloc_workqueue(
    "%s",
    WQ_MEM_RECLAIM,
    0,
    netdev_name(netdev)
  );

  INIT_WORK(&priv->tx_work, sx1280_tx_work);
  mutex_lock(&priv->lock);

  /*
   * Register the new net device.
   * The first one will appear as interface radio0.
   */
  if ((err = register_netdev(netdev))) {
    dev_err(&spi->dev, "failed to register net device\n");
    goto error_wq;
  }

  dev_info(
    &spi->dev,
    "SX1280 interface device initialized: %s\n",
    netdev->name
  );

  if ((err = sysfs_create_groups(&netdev->dev.kobj, sx1280_groups))) {
    netdev_err(netdev, "failed to create sysfs entries\n");
    goto error_wq;
  }

  /*
   * Set into continuous RX mode. Constantly look for packets and only switch to
   * TX when a packet is queued by userspace.
   */
  if ((err = sx1280_listen(priv))) {
    goto error_wq;
  }

  /*
   * Mark the SX1280 as fully initialized.
   * This activates the IRQ handler.
   */
  priv->initialized = true;
  mutex_unlock(&priv->lock);

#ifdef DEBUG
  INIT_DELAYED_WORK(&priv->status_check, sx1280_check_status);
  // schedule_delayed_work(&priv->status_check, 5 * HZ);
#endif

  dev_dbg(&spi->dev, "%s is listening for packets\n", netdev->name);
  return 0;

error_wq:
  mutex_unlock(&priv->lock);
  destroy_workqueue(priv->xmit_queue);
error_free:
  free_netdev(netdev);
  return err;
}

static void sx1280_remove(struct spi_device *spi) {
  struct sx1280_priv *priv = spi_get_drvdata(spi);

  /* TODO: Potentially need to free GPIOs for platform data instances. */

  mutex_lock(&priv->lock);

#ifdef DEBUG
  cancel_delayed_work_sync(&priv->status_check);
#endif

  cancel_work_sync(&priv->tx_work);
  destroy_workqueue(priv->xmit_queue);
  unregister_netdev(priv->netdev);
  free_netdev(priv->netdev);
  mutex_unlock(&priv->lock);
}

static const struct of_device_id sx1280_of_match[] = {
  {
    .compatible = "semtech,sx1280",
  },
  { },
};

MODULE_DEVICE_TABLE(of, sx1280_of_match);

static struct spi_driver sx1280_spi = {
  .driver = {
    .name = "sx1280",
    .of_match_table = sx1280_of_match,
    .owner = THIS_MODULE,
  },
  .probe = sx1280_probe,
  .remove = sx1280_remove,
};

module_spi_driver(sx1280_spi);

MODULE_AUTHOR("Jeff Shelton <jeff@shelton.one>");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");
