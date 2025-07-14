#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/of.h>

#include "sx1280.h"

enum sx1280_state {
  SX1280_STATE_RX,
  SX1280_STATE_TX,
  SX1280_STATE_WAIT_ACK,
  SX1280_STATE_SEND_ACK
};

/* The private, internal structure for the SX1280 driver. */
struct sx1280_priv {
  struct sx1280_platform_data pdata;
  struct net_device *netdev;
  struct spi_device *spi;

  struct gpio_desc *busy;
  struct gpio_desc *dios[3];
  int irqs[3];

  /* The packet mode that the chip is currently in. */
  enum sx1280_mode mode;

  /* The state machine state that the protocol is currently in. */
  enum sx1280_state state;

  /* The packet waiting to be transmitted. */
  struct sk_buff *tx_skb;

  struct workqueue_struct *xmit_queue;
  struct work_struct tx_work;

  spinlock_t lock;
};

/**
 * SPI functions.
 */

static int sx1280_get_status(struct spi_device *spi, u8 *status) {
  u8 tx[] = { SX1280_CMD_GET_STATUS };
  u8 rx[1];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx),
  };

  int err = spi_sync_transfer(spi, &xfer, 1);
  if (err) {
    return err;
  }

  if (status) {
    *status = rx[0];
  }

  return 0;
}

static int sx1280_write_register(
  struct spi_device *spi,
  u16 addr,
  const u8 *data,
  size_t len
) {
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

  return spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
}

static int sx1280_read_register(
  struct spi_device *spi,
  u16 addr,
  u8 *data,
  size_t len
) {
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

  return spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
}

static int sx1280_write_buffer(
  struct spi_device *spi,
  u8 offset,
  const u8 *data,
  size_t len
) {
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

  return spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
}

static int sx1280_read_buffer(
  struct spi_device *spi,
  u8 offset,
  u8 *data,
  size_t len
) {
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

  return spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
}

static int sx1280_set_sleep(
  struct spi_device *spi,
  bool save_buffer,
  bool save_ram
) {
  u8 sleep_config = (save_buffer << 1) | save_ram;
  u8 tx[] = { SX1280_CMD_SET_SLEEP, sleep_config };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_standby(
  struct spi_device *spi,
  u8 mode
) {
  u8 tx[] = { SX1280_CMD_SET_STANDBY, mode };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_fs(struct spi_device *spi) {
  u8 tx[] = { SX1280_CMD_SET_FS };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_tx(
  struct spi_device *spi,
  u8 period_base,
  u16 period_base_count
) {
  u8 tx[] = {
    SX1280_CMD_SET_TX,
    period_base,
    period_base_count >> 8,
    period_base_count & 0xFF
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_rx(
  struct spi_device *spi,
  u8 period_base,
  u16 period_base_count
) {
  u8 tx[] = {
    SX1280_CMD_SET_RX,
    period_base,
    period_base_count >> 8,
    period_base_count & 0xFF
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_rx_duty_cycle(
  struct spi_device *spi,
  u8 period_base,
  u16 rx_period_base_count,
  u16 sleep_period_base_count
) {
  u8 tx[] = {
    SX1280_CMD_SET_RX_DUTY_CYCLE,
    period_base,
    rx_period_base_count >> 8,
    rx_period_base_count & 0xFF,
    sleep_period_base_count >> 8,
    sleep_period_base_count & 0xFF
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_long_preamble(struct spi_device *spi, bool enable) {
  u8 tx[] = { SX1280_CMD_SET_LONG_PREAMBLE, enable };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_cad(struct spi_device *spi) {
  u8 tx[] = { SX1280_CMD_SET_CAD };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_tx_continuous_wave(struct spi_device *spi) {
  u8 tx[] = { SX1280_CMD_SET_TX_CONTINUOUS_WAVE };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_tx_continuous_preamble(struct spi_device *spi) {
  u8 tx[] = { SX1280_CMD_SET_TX_CONTINUOUS_PREAMBLE };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_auto_tx(struct spi_device *spi, u16 time) {
  u8 tx[] = {
    SX1280_CMD_SET_AUTO_TX,
    time >> 8,
    time & 0xFF
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_auto_fs(struct spi_device *spi, bool enable) {
  u8 tx[] = { SX1280_CMD_SET_AUTO_FS, enable };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_packet_type(
  struct spi_device *spi,
  enum sx1280_mode packet_type
) {
  u8 tx[] = { SX1280_CMD_SET_PACKET_TYPE, (u8) packet_type };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_get_packet_type(
  struct spi_device *spi,
  enum sx1280_mode *packet_type
) {
  u8 tx[3] = { SX1280_CMD_GET_PACKET_TYPE };
  u8 rx[3];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  int err = spi_sync_transfer(spi, &xfer, 1);
  if (err) {
    return err;
  } else if (rx[2] > 0x04) {
    dev_err(&spi->dev, "Device returned invalid packet type: %d\n", rx[2]);
    return -EINVAL;
  }

  if (packet_type) {
    *packet_type = (enum sx1280_mode) rx[2];
  }

  return 0;
}

static int sx1280_set_rf_frequency(
  struct spi_device *spi,
  u32 freq
) {
  u8 tx[] = {
    SX1280_CMD_SET_RF_FREQUENCY,
    freq >> 16,
    (freq >> 8) & 0xFF,
    freq & 0xFF
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_tx_params(
  struct spi_device *spi,
  u8 power,
  enum sx1280_ramp_time ramp_time
) {
  u8 tx[] = {
    SX1280_CMD_SET_TX_PARAMS,
    power,
    (u8) ramp_time
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_cad_params(
  struct spi_device *spi,
  enum sx1280_cad_symbol_num cad_symbol_num
) {
  u8 tx[] = { SX1280_CMD_SET_CAD_PARAMS, (u8) cad_symbol_num };
  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_buffer_base_address(
  struct spi_device *spi,
  u8 tx_base_addr,
  u8 rx_base_addr
) {
  u8 tx[] = {
    SX1280_CMD_SET_BUFFER_BASE_ADDRESS,
    tx_base_addr,
    rx_base_addr
  };

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_modulation_params(
  struct spi_device *spi,
  union sx1280_modulation_params params
) {
  u8 tx[4] = { SX1280_CMD_SET_MODULATION_PARAMS };
  memcpy(&tx[1], params.raw, 3);

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_set_packet_params(
  struct spi_device *spi,
  union sx1280_packet_params params
) {
  u8 tx[8] = { SX1280_CMD_SET_PACKET_PARAMS };
  memcpy(&tx[1], params.raw, 7);

  return spi_write(spi, tx, ARRAY_SIZE(tx));
}

static int sx1280_get_rx_buffer_status(
  struct spi_device *spi,
  u8 *rx_payload_len,
  u8 *rx_start_buffer_ptr
) {
  u8 tx[4] = { SX1280_CMD_GET_RX_BUFFER_STATUS };
  u8 rx[4];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  int err = spi_sync_transfer(spi, &xfer, 1);
  if (err) {
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
  struct spi_device *spi,
  union sx1280_packet_status *packet_status
) {
  u8 tx[7] = { SX1280_CMD_GET_PACKET_STATUS };

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = packet_status->raw,
    .len = ARRAY_SIZE(packet_status->raw)
  };

  return spi_sync_transfer(spi, &xfer, 1);
}

static int sx1280_get_rssi_inst(struct spi_device *spi, u8 *rssi_inst) {
  u8 tx[3] = { SX1280_CMD_GET_RSSI_INST };
  u8 rx[3];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx)
  };

  int err = spi_sync_transfer(spi, &xfer, 1);
  if (err) {
    return err;
  }

  if (rssi_inst) {
    *rssi_inst = rx[2];
  }

  return 0;
}

// Configures the DIO pins to act as interrupts according to their masks.
static int sx1280_set_dio_irq_params(
  struct spi_device *spi,
  u16 irq_mask,
  u16 dio_mask[3]
) {
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

  return spi_write(spi, tx, sizeof(tx));
}

// Gets the current state of the IRQ register.
static int sx1280_get_irq_status(struct spi_device *spi, u16 *irq_status) {
  u8 tx[4] = { SX1280_CMD_GET_IRQ_STATUS };
  u8 rx[4];

  struct spi_transfer xfer = {
    .tx_buf = tx,
    .rx_buf = rx,
    .len = ARRAY_SIZE(rx),
  };

  int err = spi_sync_transfer(spi, &xfer, 1);
  if (err) {
    return err;
  }

  // The IRQ status is returned in big-endian format.
  *irq_status = ((u16) rx[2] << 8) | rx[3];
  return 0;
}

// Clears flags in the IRQ register according to the mask.
static int sx1280_clear_irq_status(struct spi_device *spi, u16 irq_mask) {
  u8 tx[] = {
    SX1280_CMD_CLR_IRQ_STATUS,
    irq_mask >> 8,
    irq_mask & 0xFF
  };

  return spi_write(spi, tx, sizeof(tx));
}

/**
 * Waits for the GPIO to be pulled externally to the specified level within a
 * specified maximum number of microseconds.
 */
static int sx1280_wait_on_gpio(
  struct gpio_desc *gpio,
  int value,
  u32 timeout_us
) {
  u32 elapsed_us = 0;

  // Perform a measurement of the GPIO every 500 us and check the value.
  do {
    int meas = gpiod_get_value_cansleep(gpio);

    if (meas < 0) {
      return meas;
    } else if (meas == value) {
      return 0;
    }

    usleep_range(500, 1000);
    elapsed_us += 500;
  } while (elapsed_us < timeout_us);

  // If the loop has fallen through, the loop timed out.
  return -ETIMEDOUT;
}

/**
 * Driver functions.
 */

static int sx1280_open(struct net_device *dev) {
  netif_start_queue(dev);
  return 0;
}

static int sx1280_stop(struct net_device *dev) {
  netif_stop_queue(dev);
  return 0;
}

static netdev_tx_t sx1280_xmit(struct sk_buff *skb, struct net_device *dev) {
  struct sx1280_priv *priv = netdev_priv(dev);

  // Stop the packet queue, applying backpressure to the kernel networking stack
  // that allows the driver to send one packet at a time. Packets that arrive in
  // the intervening time will be queued by the networking stack.
  //
  // Once the corresponding packet has been sent and the chip is ready for a new
  // one, `netif_wake_queue` is called to tell the kernel that it is permitted
  // to call `sx1280_xmit` once again.
  netif_stop_queue(dev);

  // Check if there is already a packet being transmitted.
  //
  // This is not supposed to happen, as the kernel should not call `sx1280_xmit`
  // after the queue is stopped. If it does happen, backpressure is applied and
  // a warning is printed.
  if (priv->tx_skb) {
    dev_warn(&dev->dev, "Packet transmission requested after queue frozen.");
    return NETDEV_TX_BUSY;
  }

  // Queue the work so that it can be performed in a non-atomic context.
  priv->tx_skb = skb;
  queue_work(priv->xmit_queue, &priv->tx_work);
  return NETDEV_TX_OK;
}

static void sx1280_tx_work(struct work_struct *work) {
  struct sx1280_priv *priv = container_of(work, struct sx1280_priv, tx_work);
  struct spi_device *spi = priv->spi;
  struct sk_buff *skb = priv->tx_skb;
  int err;

  if (!skb) {
    dev_warn(&priv->netdev->dev, "Transmission queued without packet SKB.");
    return;
  }

  if ((err = sx1280_wait_on_gpio(priv->busy, 0, 10000))) {
    dev_err(&priv->spi->dev, "Wait for busy pin timed out.");
    return;
  }

  // Nullify the Tx SKB in preparation for the next packet.
  //
  // Here, the driver does not yet wake the queue because the transmission is
  // not finished.
  priv->tx_skb = NULL;

  // Check that the chip is not in ranging mode.
  // Packets can't be sent in ranging mode.
  if (priv->mode == SX1280_MODE_RANGING) {
    dev_warn(&priv->netdev->dev, "Packet transmission requested in ranging mode.");
    dev_consume_skb_any(skb);
    return;
  }

  // Write the packet data from the SKB into the chip's data buffer.
  //
  // The buffer is a maximum of 256 bytes long.
  // The SKB is restricted to at most 256 bytes by the configuration in
  // `sx1280_probe` that specifies the MTU as 256 bytes.
  if ((err = sx1280_write_buffer(spi, 0, skb->data, skb->len))) {
    dev_err(&spi->dev, "Failed to write packet data to buffer.");
    return;
  }

  err = sx1280_set_tx(
    spi,
    priv->pdata.period_base,
    priv->pdata.period_base_count
  );

  if (err) {
    dev_err(&spi->dev, "Failed to transmit packet.");
    return;
  }

  // Free the SKB.
  dev_consume_skb_any(skb);

  // Restart the packet queue so that xmit may be called again.
  netif_start_queue(priv->netdev);
}

static irqreturn_t sx1280_done_irq(int irq, void *dev_id) {
  struct sx1280_priv *priv = dev_id;

  // Acknowledge the interrupt.
  sx1280_clear_irq_status(priv->spi, 0xFF);
  return IRQ_HANDLED;
}

static irqreturn_t sx1280_timeout_irq(int irq, void *dev_id) {
  return IRQ_HANDLED;
}

static irqreturn_t sx1280_error_irq(int irq, void *dev_id) {
  return IRQ_HANDLED;
}

#define CONCAT32(a, b) (((u32) (a) << 16) | (u32) (b))
#define MATCH2(a, b, v) case CONCAT32(a, b): mod->bitrate_bandwidth = v; break;

static int sx1280_parse_dt_ble(
  struct device *dev,
  struct sx1280_platform_data *pdata
) {
  u32 max_payload_bytes = 37;
  u32 crc_bytes = 3;
  const char *test_payload = "prbs9";
  bool disable_whitening = false;
  u32 access_address = 0;
  u16 crc_seed = 0;
  s32 power_dbm = 13;
  u32 ramp_time_us = 2;

  struct device_node *child = of_get_child_by_name(dev->of_node, "ble");

  if (child) {
    of_property_read_u32(child, "max-payload-bytes", &max_payload_bytes);
    of_property_read_u32(child, "crc-bytes", &crc_bytes);
    of_property_read_string(child, "test-payload", &test_payload);
    disable_whitening = of_property_read_bool(child, "disable-whitening");
    of_property_read_u32(child, "access-address", &access_address);
    of_property_read_u16(child, "crc-seed", &crc_seed);
    of_property_read_s32(child, "power-dbm", &power_dbm);
    of_property_read_u32(child, "ramp-time-us", &ramp_time_us);

    of_node_put(child);
  }

  struct sx1280_ble_params *ble = &pdata->ble;
  struct sx1280_gfsk_modulation_params *mod = &ble->modulation;
  struct sx1280_ble_packet_params *pkt = &ble->packet;

  // Modulation variables are fully determined for BLE.
  *mod = (struct sx1280_gfsk_modulation_params) {
    .bitrate_bandwidth = SX1280_GFSK_BLE_BR_1_000_BW_1_2,
    .modulation_index = SX1280_MOD_IND_0_50,
    .bandwidth_time = SX1280_BT_0_5
  };

  /**
   * Payload length
   */

  switch (max_payload_bytes) {
  case 31:
    pkt->payload_length = SX1280_BLE_PAYLOAD_LENGTH_MAX_31_BYTES;
    break;
  case 37:
    pkt->payload_length = SX1280_BLE_PAYLOAD_LENGTH_MAX_37_BYTES;
    break;
  case 63:
    pkt->payload_length = SX1280_BLE_TX_TEST_MODE;
    break;
  case 255:
    pkt->payload_length = SX1280_BLE_PAYLOAD_LENGTH_MAX_255_BYTES;
    break;
  default:
    dev_err(dev, "Invalid value for ble.max-payload-bytes.");
    return -EINVAL;
  }

  /**
   * CRC
   */

  switch (crc_bytes) {
  case 0:
    pkt->crc_length = SX1280_BLE_CRC_OFF;
    break;
  case 3:
    pkt->crc_length = SX1280_BLE_CRC_3B;
    break;
  default:
    dev_err(dev, "Invalid value for ble.crc-bytes.");
    return -EINVAL;
  }

  /**
   * Test payload
   */

  if (strcmp(test_payload, "prbs9") == 0) {
    pkt->test_payload = SX1280_BLE_PRBS_9;
  } else if (strcmp(test_payload, "eyelong10") == 0) {
    pkt->test_payload = SX1280_BLE_EYELONG_1_0;
  } else if (strcmp(test_payload, "eyeshort10") == 0) {
    pkt->test_payload = SX1280_BLE_EYESHORT_1_0;
  } else if (strcmp(test_payload, "prbs15") == 0) {
    pkt->test_payload = SX1280_BLE_PRBS_15;
  } else if (strcmp(test_payload, "all1") == 0) {
    pkt->test_payload = SX1280_BLE_ALL_1;
  } else if (strcmp(test_payload, "all0") == 0) {
    pkt->test_payload = SX1280_BLE_ALL_0;
  } else if (strcmp(test_payload, "eyelong01") == 0) {
    pkt->test_payload = SX1280_BLE_EYELONG_0_1;
  } else if (strcmp(test_payload, "eyeshort01") == 0) {
    pkt->test_payload = SX1280_BLE_EYESHORT_0_1;
  } else {
    dev_err(dev, "Invalid value for ble.test-payload.");
    return -EINVAL;
  }

  /**
   * Whitening
   */

  pkt->whitening = disable_whitening
    ? SX1280_WHITENING_DISABLE
    : SX1280_WHITENING_ENABLE;

  /**
   * Access address
   */

  ble->access_address = access_address;

  /**
   * CRC seed
   */

  ble->crc_seed = crc_seed;

  /**
   * Power
   */

  if (power_dbm < -18 || power_dbm > 13) {
    dev_err(dev, "Invalid value for ble.power-dbm.");
    return -EINVAL;
  }

  ble->power = (u8) (power_dbm + 18);

  /**
   * Ramp time
   */

  if (
    ramp_time_us < 2
    || ramp_time_us > 20
    || ramp_time_us % 2 != 0
    || ramp_time_us == 14
  ) {
    dev_err(dev, "Invalid value for ble.ramp-time-us.");
    return -EINVAL;
  }

  if (ramp_time_us <= 12) {
    ble->ramp_time = (ramp_time_us - 2) << 4;
  } else {
    ble->ramp_time = (ramp_time_us + 8) << 3;
  }

  return 0;
}

static int sx1280_parse_dt_flrc(
  struct device *dev,
  struct sx1280_platform_data *pdata
) {
  u32 bitrate_kbs = 1300;
  const char *coding_rate = "3/4";
  const char *bt = "1.0";
  u32 preamble_bits = 8;
  u32 sync_word_bits = 32;
  u32 sync_word_match[3] = { 0, 0, 0 };
  bool fixed_length = false;
  u32 max_payload_bytes = 127;
  u32 crc_bytes = 2;
  u32 crc_seed = 0;
  bool disable_whitening = false;
  u32 sync_words[3] = { 0, 0, 0 };

  struct device_node *child = of_get_child_by_name(dev->of_node, "flrc");

  if (child) {
    of_property_read_u32(child, "bitrate-kbs", &bitrate_kbs);
    of_property_read_string(child, "coding-rate", &coding_rate);
    of_property_read_string(child, "bt", &bt);
    of_property_read_u32(child, "preamble-bits", &preamble_bits);
    of_property_read_u32(child, "sync-word-bytes", &sync_word_bits);
    of_property_read_u32_array(child, "sync-word-match", sync_word_match, 3);
    fixed_length = of_property_read_bool(child, "fixed-length");
    of_property_read_u32(child, "max-payload-bytes", &max_payload_bytes);
    of_property_read_u32(child, "crc-bytes", &crc_bytes);
    of_property_read_u32(child, "crc-seed", &crc_seed);
    disable_whitening = of_property_read_bool(child, "disable-whitening");
    of_property_read_variable_u32_array(child, "sync-words", sync_words, 0, 3);

    of_node_put(child);
  }

  // Convenience variables for simple member accesses.
  struct sx1280_flrc_params *flrc = &pdata->flrc;
  struct sx1280_flrc_modulation_params *mod = &flrc->modulation;
  struct sx1280_flrc_packet_params *pkt = &flrc->packet;

  switch (bitrate_kbs) {
  case 1300:
    mod->bitrate_bandwidth = SX1280_FLRC_BR_1_300_BW_1_2;
    break;
  case 1000:
    mod->bitrate_bandwidth = SX1280_FLRC_BR_1_000_BW_1_2;
    break;
  case 650:
    mod->bitrate_bandwidth = SX1280_FLRC_BR_0_650_BW_0_6;
    break;
  case 520:
    mod->bitrate_bandwidth = SX1280_FLRC_BR_0_520_BW_0_6;
    break;
  case 325:
    mod->bitrate_bandwidth = SX1280_FLRC_BR_0_325_BW_0_3;
    break;
  case 260:
    mod->bitrate_bandwidth = SX1280_FLRC_BR_0_260_BW_0_3;
    break;
  default:
    dev_err(dev, "Invalid value for flrc.bitrate-kbs.");
    return -EINVAL;
  }

  /**
   * Coding rate
   */

  if (strcmp(coding_rate, "1/2") == 0) {
    mod->coding_rate = SX1280_FLRC_CR_1_2;
  } else if (strcmp(coding_rate, "3/4") == 0) {
    mod->coding_rate = SX1280_FLRC_CR_3_4;
  } else if (strcmp(coding_rate, "1/1") == 0) {
    mod->coding_rate = SX1280_FLRC_CR_1_1;
  } else {
    dev_err(dev, "Invalid value for flrc.coding-rate.");
    return -EINVAL;
  }

  /**
   * Bandwidth-time
   */

  if (strcmp(bt, "off") == 0) {
    mod->bandwidth_time = SX1280_BT_OFF;
  } else if (strcmp(bt, "1.0") == 0) {
    mod->bandwidth_time = SX1280_BT_1_0;
  } else if (strcmp(bt, "0.5") == 0) {
    mod->bandwidth_time = SX1280_BT_0_5;
  } else {
    dev_err(dev, "Invalid value for flrc.bt.");
    return -EINVAL;
  }

  /**
   * Preamble length (bits)
   */

  if (
    preamble_bits % 4 != 0
    || preamble_bits > 32
    || preamble_bits < 4
  ) {
    dev_err(dev, "Invalid value for flrc.preamble-bits.");
    return -EINVAL;
  }

  pkt->agc_preamble_length = (enum sx1280_preamble_length) (preamble_bits << 2);

  /**
   * Sync word length (bits)
   */

  switch (sync_word_bits) {
  case 0:
    pkt->sync_word_length = SX1280_FLRC_SYNC_WORD_NOSYNC;
    break;
  case 32:
    pkt->sync_word_length = SX1280_FLRC_SYNC_WORD_LEN_P32S;
    break;
  default:
    dev_err(dev, "Invalid value for flrc.sync-word-bits.");
    return -EINVAL;
  }

  /**
   * Sync word combination
   */

  pkt->sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_OFF;

  for (int i = 0; i < 2; i++) {
    if (sync_word_match[i] > 1) {
      dev_err(dev, "Invalid value for flrc.sync-word-match.");
      return -EINVAL;
    }

    pkt->sync_word_match |= sync_word_match[i] << i;
  }

  /**
   * Header type
   */

  pkt->header_type = fixed_length
    ? SX1280_RADIO_PACKET_FIXED_LENGTH
    : SX1280_RADIO_PACKET_VARIABLE_LENGTH;

  /**
   * Payload length
   */

  if (max_payload_bytes < 6 || max_payload_bytes > 127) {
    dev_err(dev, "Invalid value for flrc.max-payload-bytes.");
    return -EINVAL;
  }

  pkt->payload_length = max_payload_bytes;

  /**
   * CRC bytes
   */

  switch (crc_bytes) {
  case 0:
    pkt->crc_length = SX1280_FLRC_CRC_OFF;
    break;
  case 2:
    pkt->crc_length = SX1280_FLRC_CRC_2_BYTE;
    break;
  case 3:
    pkt->crc_length = SX1280_FLRC_CRC_3_BYTE;
    break;
  case 4:
    pkt->crc_length = SX1280_FLRC_CRC_4_BYTE;
    break;
  default:
    dev_err(dev, "Invalid value for flrc.crc-bytes.");
    return -EINVAL;
  }

  /**
   * Whitening
   */

  pkt->whitening = disable_whitening
    ? SX1280_WHITENING_DISABLE
    : SX1280_WHITENING_ENABLE;

  /**
   * CRC seed
   */

  flrc->crc_seed = crc_seed;

  return 0;
}

static int sx1280_parse_dt_gfsk(
  struct device *dev,
  struct sx1280_platform_data *pdata
) {
  // Populate defaults for GFSK mode.
  u16 bitrate_kbs = 2000;
  u16 bandwidth_khz = 2400;
  u32 mod_index = 200;
  const char *bt = "1.0";
  u32 preamble_bits = 8;
  u32 sync_word_bytes = 2;
  u32 sync_word_match[3] = { 0, 0, 0 };
  bool fixed_length = false;
  u8 max_payload_bytes = 255;
  u32 crc_bytes = 2;
  bool disable_whitening = false;
  u32 sync_words[3] = { 0, 0, 0 };
  u32 crc_seed = 0;
  u32 crc_polynomial = 0;
  s32 power_dbm = 13;
  u32 ramp_time_us = 2;

  struct device_node *child = of_get_child_by_name(dev->of_node, "gfsk");

  if (child) {
    // TODO: Handle errors other than being not present.
    of_property_read_u16(child, "bitrate-kbs", &bitrate_kbs);
    of_property_read_u16(child, "bandwidth-khz", &bandwidth_khz);
    of_property_read_u32(child, "modulation-index", &mod_index);
    of_property_read_string(child, "bt", &bt);
    of_property_read_u32(child, "preamble-bits", &preamble_bits);
    of_property_read_u32(child, "sync-word-bytes", &sync_word_bytes);
    of_property_read_u32_array(child, "sync-word-match", sync_word_match, 3);
    fixed_length = of_property_read_bool(child, "fixed-length");
    of_property_read_u8(child, "max-payload-bytes", &max_payload_bytes);
    of_property_read_u32(child, "crc-bytes", &crc_bytes);
    disable_whitening = of_property_read_bool(child, "disable-whitening");
    of_property_read_variable_u32_array(child, "sync-words", sync_words, 0, 3);
    of_property_read_u32(child, "crc-seed", &crc_seed);
    of_property_read_u32(child, "crc-polynomial", &crc_polynomial);
    of_property_read_s32(child, "power-dbm", &power_dbm);
    of_property_read_u32(child, "ramp-time-us", &ramp_time_us);

    of_node_put(child);
  }

  struct sx1280_gfsk_params *gfsk = &pdata->gfsk;
  struct sx1280_gfsk_modulation_params *mod = &gfsk->modulation;
  struct sx1280_gfsk_packet_params *pkt = &gfsk->packet;

  /**
   * Bitrate-bandwidth
   */

  u32 brbw = CONCAT32(bitrate_kbs, bandwidth_khz);

  switch (brbw) {
    MATCH2(2000, 2400, SX1280_GFSK_BLE_BR_2_000_BW_2_4)
    MATCH2(1600, 2400, SX1280_GFSK_BLE_BR_1_600_BW_2_4)
    MATCH2(1000, 2400, SX1280_GFSK_BLE_BR_1_000_BW_2_4)
    MATCH2(1000, 1200, SX1280_GFSK_BLE_BR_1_000_BW_1_2)
    MATCH2(800, 2400, SX1280_GFSK_BLE_BR_0_800_BW_2_4)
    MATCH2(800, 1200, SX1280_GFSK_BLE_BR_0_800_BW_1_2)
    MATCH2(500, 1200, SX1280_GFSK_BLE_BR_0_500_BW_1_2)
    MATCH2(500, 600, SX1280_GFSK_BLE_BR_0_500_BW_0_6)
    MATCH2(400, 1200, SX1280_GFSK_BLE_BR_0_400_BW_1_2)
    MATCH2(400, 600, SX1280_GFSK_BLE_BR_0_400_BW_0_6)
    MATCH2(250, 600, SX1280_GFSK_BLE_BR_0_250_BW_0_6)
    MATCH2(250, 300, SX1280_GFSK_BLE_BR_0_250_BW_0_3)
    MATCH2(125, 300, SX1280_GFSK_BLE_BR_0_125_BW_0_3)
  default:
    dev_err(
      dev,
      "Invalid combination of gfsk.bitrate-kbs and gfsk.bandwidth-khz."
    );
    return -EINVAL;
  }

  /**
   * Modulation index
   */

  if (
    (mod_index % 25 != 0 && mod_index != 35)
    || mod_index > 400
  ) {
    dev_err(dev, "Invalid value for gfsk.modulation-index.");
    return -EINVAL;
  }

  mod->modulation_index =
    (enum sx1280_gfsk_modulation_index) (mod_index / 25 - 1);

  /**
   * Bandwidth-time
   */

  if (strcmp(bt, "off") == 0) {
    mod->bandwidth_time = SX1280_BT_OFF;
  } else if (strcmp(bt, "1.0") == 0) {
    mod->bandwidth_time = SX1280_BT_1_0;
  } else if (strcmp(bt, "0.5") == 0) {
    mod->bandwidth_time = SX1280_BT_0_5;
  } else {
    dev_err(dev, "Invalid value for gfsk.bt.");
    return -EINVAL;
  }

  /**
   * Preamble length (bits)
   */

  if (
    preamble_bits % 4 != 0
    || preamble_bits > 32
    || preamble_bits < 4
  ) {
    dev_err(dev, "Invalid value for gfsk.preamble-bits.");
    return -EINVAL;
  }

  pkt->preamble_len = (enum sx1280_preamble_length) (preamble_bits << 2);

  /**
   * Sync word length
   */

  if (sync_word_bytes < 1 || sync_word_bytes > 5) {
    dev_err(dev, "Invalid value for gfsk.sync-word-bytes.");
    return -EINVAL;
  }

  pkt->sync_word_len = ((sync_word_bytes - 1) * 2);

  /**
   * Sync word combination
   */

  pkt->sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_OFF;

  for (int i = 0; i < 2; i++) {
    if (sync_word_match[i] > 1) {
      dev_err(dev, "Invalid value for gfsk.sync-word-match.");
      return -EINVAL;
    }

    pkt->sync_word_match |= sync_word_match[i] << i;
  }

  /**
   * Header type
   */

  pkt->header_type = fixed_length
    ? SX1280_RADIO_PACKET_FIXED_LENGTH
    : SX1280_RADIO_PACKET_VARIABLE_LENGTH;

  /**
   * Payload length
   */

  pkt->payload_len = max_payload_bytes;

  /**
   * CRC bytes
   */

  pkt->crc_len = (enum sx1280_gfsk_crc_length) (crc_bytes << 4);

  /**
   * Whitening
   */

  pkt->whitening = disable_whitening
    ? SX1280_WHITENING_DISABLE
    : SX1280_WHITENING_ENABLE;

  /**
   * CRC
   */

  gfsk->crc_seed = crc_seed;
  gfsk->crc_polynomial = crc_polynomial;

  /**
   * Power
   */

  if (power_dbm < -18 || power_dbm > 13) {
    dev_err(dev, "Invalid value for power-dbm.");
    return -EINVAL;
  }

  gfsk->power = (u8) (power_dbm + 18);

  /**
   * Ramp time
   */

  if (
    ramp_time_us < 2
    || ramp_time_us > 20
    || ramp_time_us % 2 != 0
    || ramp_time_us == 14
  ) {
    dev_err(dev, "Invalid value for ramp-time-us.");
    return -EINVAL;
  }

  if (ramp_time_us <= 12) {
    gfsk->ramp_time = (ramp_time_us - 2) << 4;
  } else {
    gfsk->ramp_time = (ramp_time_us + 8) << 3;
  }

  return 0;
}

static int sx1280_parse_dt_lora(
  struct device *dev,
  struct sx1280_platform_data *pdata
) {
  u32 spreading_factor = 12;
  u32 bandwidth_khz = 1600;
  const char *coding_rate = "4/8";
  bool disable_li;
  u32 preamble_bits = 8;
  bool implicit_header;
  u8 max_payload_bytes = 255;
  bool disable_crc;
  bool invert_iq;

  struct device_node *child = of_get_child_by_name(dev->of_node, "lora");

  if (child) {
    of_property_read_u32(child, "spreading-factor", &spreading_factor);
    of_property_read_u32(child, "bandwidth-khz", &bandwidth_khz);
    of_property_read_string(child, "coding-rate", &coding_rate);
    disable_li = of_property_read_bool(child, "disable-long-interleaving");
    of_property_read_u32(child, "preamble-bits", &preamble_bits);
    implicit_header = of_property_read_bool(child, "implicit-header");
    of_property_read_u8(child, "max-payload-bytes", &max_payload_bytes);
    disable_crc = of_property_read_bool(child, "disable-crc");
    invert_iq = of_property_read_bool(child, "invert-iq");

    of_node_put(child);
  }

  struct sx1280_lora_modulation_params *mod = &pdata->lora.modulation;
  struct sx1280_lora_packet_params *pkt = &pdata->lora.packet;

  if (spreading_factor < 5 || spreading_factor > 12) {
    dev_err(dev, "Property lora.spreading-factor out of range.");
    return -EINVAL;
  }

  mod->spreading = (enum sx1280_lora_spreading) (spreading_factor << 4);

  switch (bandwidth_khz) {
  case 1600:
    mod->bandwidth = SX1280_LORA_BW_1600;
    break;
  case 800:
    mod->bandwidth = SX1280_LORA_BW_800;
    break;
  case 400:
    mod->bandwidth = SX1280_LORA_BW_400;
    break;
  case 200:
    mod->bandwidth = SX1280_LORA_BW_200;
    break;
  default:
    dev_err(dev, "Invalid value for bandwidth-khz.");
    return -EINVAL;
  }

  /**
   * Coding rate parsing.
   */

  if (
    strlen(coding_rate) != 3
    || coding_rate[0] != '4'
    || coding_rate[1] != '/'
    || coding_rate[2] < '5'
    || coding_rate[2] > '8'
  ) {
    dev_err(dev, "Invalid coding-rate. Must be 4/5, 4/6, 4/7, or 4/8.");
    return -EINVAL;
  }

  if (disable_li) {
    mod->coding_rate = (enum sx1280_lora_coding_rate) (coding_rate[2] - '4');
  } else {
    switch (coding_rate[2]) {
    case '5':
      mod->coding_rate = SX1280_LORA_CR_LI_4_5;
      break;
    case '6':
      mod->coding_rate = SX1280_LORA_CR_LI_4_6;
      break;
    case '7':
      // 4/7 LI coding is not available.
      // Default to using 4/7 coding without LI.
      mod->coding_rate = SX1280_LORA_CR_4_7;
      break;
    case '8':
      mod->coding_rate = SX1280_LORA_CR_LI_4_8;
      break;
    }
  }

  /**
   * Preamble length (bits).
   */

  u32 pb_mant = preamble_bits;
  u8 pb_exp = 0;

  // Reduce the preamble bit count into a mantissa and exponent, where both
  // are allowed to be [1..15].
  while ((pb_mant & 1) == 0) {
    pb_mant >>= 1;
    pb_exp++;
  }

  if (
    pb_mant == 0
    || pb_mant > 15
    || pb_exp == 0
    || pb_exp > 15
  ) {
    dev_err(dev, "Invalid value for preamble-bits.");
    return -EINVAL;
  }

  // The format used by the SX1280 to encode preamble length is a strange
  // choice. It uses bits 7-4 to encode the exponent and bits 3-0 for the
  // mantissa of the preamble length, in bits. Both components must be in the
  // range [1..15].
  //
  // This increases the maximum preamble length to 491,520 over the 255 that
  // would ordinarily be representable, but 255 is itself an excessively large
  // preamble size (8 is typical).
  pkt->preamble_len = (pb_exp << 4) | (u8) pb_mant;

  /**
   * Header type.
   */

  pkt->header_type = implicit_header
    ? SX1280_IMPLICIT_HEADER
    : SX1280_EXPLICIT_HEADER;

  /**
   * Payload length.
   */

  if (
    max_payload_bytes == 0
    || (mod->coding_rate == SX1280_LORA_CR_LI_4_8 && max_payload_bytes > 253)
  ) {
    dev_err(dev, "Invalid value for max-payload-bytes.");
    return -EINVAL;
  }

  pkt->payload_len = max_payload_bytes;

  /**
   * CRC enabling.
   */

  pkt->crc = disable_crc
    ? SX1280_LORA_CRC_DISABLE
    : SX1280_LORA_CRC_ENABLE;

  /**
   * IQ inversion.
   */

  pkt->invert_iq = invert_iq
    ? SX1280_LORA_IQ_INVERTED
    : SX1280_LORA_IQ_STD;

  return 0;
}

static int sx1280_parse_dt_ranging(
  struct device *dev,
  struct sx1280_platform_data *pdata
) {
  return 0;
}

static int sx1280_parse_dt(
  struct device *dev,
  struct sx1280_platform_data *pdata
) {
  struct device_node *node = dev->of_node;
  int err;

  // Default values for top-level device tree properties.
  const char *mode = "lora";
  u32 rf_freq_hz = 2400000000;
  u32 startup_timeout_us = 2000;
  u32 tx_timeout_us = 1000;

  // Overwrite the default values if they are specified.
  of_property_read_string(node, "mode", &mode);
  of_property_read_u32(node, "rf-freq-hz", &rf_freq_hz);
  of_property_read_u32(node, "startup-timeout-us", &startup_timeout_us);
  of_property_read_u32(node, "tx-timeout-us", &tx_timeout_us);

  if (strcmp(mode, "ble") == 0) {
    pdata->mode = SX1280_MODE_BLE;
  } else if (strcmp(mode, "flrc") == 0) {
    pdata->mode = SX1280_MODE_FLRC;
  } else if (strcmp(mode, "gfsk") == 0) {
    pdata->mode = SX1280_MODE_GFSK;
  } else if (strcmp(mode, "lora") == 0) {
    pdata->mode = SX1280_MODE_LORA;
  } else if (strcmp(mode, "ranging") == 0) {
    pdata->mode = SX1280_MODE_RANGING;
  } else {
    dev_err(dev, "Invalid value for mode.");
    return -EINVAL;
  }

  // Convert the Hz frequency into the fixed-point representation that the chip
  // natively understands.
  pdata->rf_freq = SX1280_FREQ_HZ_TO_PLL(rf_freq_hz);

  // Convert the timeout and period base into nanoseconds so that an integer
  // division can be performed for maximum precision within constraints.
  u32 timeout_ns = tx_timeout_us * 1000;
  u32 period_base_ns;

  if (tx_timeout_us < 1024000) {
    pdata->period_base = SX1280_PERIOD_BASE_15_625_US;
    period_base_ns = 15625;
  } else if (tx_timeout_us < 4096000) {
    pdata->period_base = SX1280_PERIOD_BASE_62_500_US;
    period_base_ns = 62500;
  } else if (tx_timeout_us < 6553600) {
    pdata->period_base = SX1280_PERIOD_BASE_1_MS;
    period_base_ns = 1000000;
  } else if (tx_timeout_us < 262144000) {
    pdata->period_base = SX1280_PERIOD_BASE_4_MS;
    period_base_ns = 4000000;
  } else {
    dev_err(dev, "Invalid value for timeout-us.");
    return -EINVAL;
  }

  // Perform a ceiling division to determine the cycle count.
  //
  // The timeout specified is always a minimum bound on the timeout that could
  // also be affected by outside factors such as kernel timing, timer precision,
  // etc. When a timeout is specified, it's crucial that the driver guarantees
  // that _at least_ that amount of time has passed before timing out.
  pdata->period_base_count = (timeout_ns / period_base_ns)
    + (timeout_ns % period_base_ns != 0);

  // Parse all sub-nodes of the device tree, containing the properties
  // configuring each of the different packet modes of the SX1280.
  if (
    (err = sx1280_parse_dt_ble(dev, pdata))
    || (err = sx1280_parse_dt_flrc(dev, pdata))
    || (err = sx1280_parse_dt_gfsk(dev, pdata))
    || (err = sx1280_parse_dt_lora(dev, pdata))
    || (err = sx1280_parse_dt_ranging(dev, pdata))
  ) {
    return err;
  }

  return 0;
}

/**
 * Performs the chip setup.
 */
static int sx1280_setup(struct sx1280_priv *priv, enum sx1280_mode mode) {
  struct spi_device *spi = priv->spi;
  int err;

  // Switch the chip into STDBY_RC mode.
  //
  // While it is always the case that after hard reset, the chip will go to
  // STDBY_RC, the driver cannot guarantee the state of the chip at this point
  // as it has not yet had exclusive control over the SPI line.
  sx1280_set_standby(spi, SX1280_STDBY_RC);

  // Set the packet type.
  sx1280_set_packet_type(spi, mode);

  // Set the RF frequency.
  u32 rf_freq = SX1280_DEFAULT_RF_FREQ_HZ;
  err = of_property_read_u32(spi->dev.of_node, "freq-hz", &rf_freq);
  if (err) {
    dev_info(&spi->dev, "freq-hz not defined in device tree. Defaulting to 2.4 GHz.");
  }

  sx1280_set_rf_frequency(spi, rf_freq);

  // Set the Tx and Rx buffer base addresses to 0x0.
  // This allows the chip to use the full 256-byte data buffer.
  // The size of the data buffer also restricts the MTU to 256 bytes.
  //
  // Since the chip supports half-duplex, the data must be sent/read before
  // performing another operation, but otherwise will not be overwritten.
  sx1280_set_buffer_base_address(spi, 0x0, 0x0);

  // Extract the modulation and packet params from the platform data, depending
  // on the mode that the chip is being commanded into.
  union sx1280_modulation_params mod_params;
  union sx1280_packet_params pkt_params;

  switch (mode) {
  case SX1280_MODE_BLE:
    mod_params.gfsk = priv->pdata.ble.modulation;
    pkt_params.ble = priv->pdata.ble.packet;
    break;
  case SX1280_MODE_FLRC:
    mod_params.flrc = priv->pdata.flrc.modulation;
    pkt_params.flrc = priv->pdata.flrc.packet;
    break;
  case SX1280_MODE_GFSK:
    mod_params.gfsk = priv->pdata.gfsk.modulation;
    pkt_params.gfsk = priv->pdata.gfsk.packet;
    break;
  case SX1280_MODE_LORA:
    mod_params.lora = priv->pdata.lora.modulation;
    pkt_params.lora = priv->pdata.lora.packet;
    break;
  case SX1280_MODE_RANGING:
    mod_params.lora = priv->pdata.ranging.modulation;
    pkt_params.lora = priv->pdata.ranging.packet;
    break;
  }

  // Set modulation and packet parameters.
  // These may be changed later by ioctl calls.
  sx1280_set_modulation_params(spi, mod_params);
  sx1280_set_packet_params(spi, pkt_params);

  // Write the sync words.
  // TODO: Allow customization.
  u8 sync_word[5] = { 0 };
  sx1280_write_register(spi, SX1280_REG_SYNC_ADDRESS_1_BYTE_4, sync_word, 5);

  return 0;
}

static const struct net_device_ops sx1280_netdev_ops = {
  .ndo_open = sx1280_open,
  .ndo_start_xmit = sx1280_xmit,
  .ndo_stop = sx1280_stop,
};

/**
 * The core probe function for the SX1280.
 * @param spi - The SPI device wired to the SX1280.
 * @returns 0 on success, error code otherwise.
 */
static int sx1280_probe(struct spi_device *spi) {
  int err;

  // Allocate a new net device (without registering, yet).
  struct net_device *netdev = alloc_netdev(
    sizeof(struct sx1280_priv),
    "sx%d",
    NET_NAME_UNKNOWN,

    // TODO: Evaluate whether this is the right setup handler.
    ether_setup
  );

  if (!netdev) {
    return -ENOMEM;
  }

  // Set properties of the net device.
  netdev->netdev_ops = &sx1280_netdev_ops;
  SET_NETDEV_DEV(netdev, &spi->dev);

  // Create and register the priv structure.
  struct sx1280_priv *priv = netdev_priv(netdev);
  priv->mode = SX1280_MODE_GFSK;
  priv->netdev = netdev;
  priv->spi = spi;
  spin_lock_init(&priv->lock);

  // Attempt to get legacy platform data.
  // Otherwise, parse the device tree.
  struct sx1280_platform_data *legacy_platform = dev_get_platdata(&spi->dev);
  if (legacy_platform) {
    priv->pdata = *legacy_platform;

    // If platform data is used, the GPIOs are passed as integers.
    // This is not the preferred way for GPIOs to be specified in device trees
    // anymore, so the conversion of GPIOs into descriptors must be done
    // separately.

    priv->busy = gpio_to_desc(legacy_platform->busy_gpio);
    err = -EINVAL;
    if (!priv->busy || (err = gpiod_direction_input(priv->busy))) {
      dev_err(&spi->dev, "Failed to configure GPIO for busy pin.");
      goto err_platform;
    }

    for (int i = 0; i < 3; i++) {
      struct gpio_desc *dio = gpio_to_desc(legacy_platform->dio_gpios[i]);

      err = -EINVAL;
      if (!dio || (err = gpiod_direction_input(dio))) {
        dev_err(&spi->dev, "Failed to configure GPIO for DIO%d.", i);
        goto err_platform;
      }

      priv->dios[i] = dio;
    }
  } else {
    if ((err = sx1280_parse_dt(&spi->dev, &priv->pdata))) {
      dev_err(&spi->dev, "Failed to parse device tree.");
      goto err_platform;
    }

    // If a device tree is used, then the GPIOs are directly registered with the
    // SPI device and freed upon the SPI device being unregistered.

    priv->busy = devm_gpiod_get(&spi->dev, "busy", GPIOD_IN);
    if (IS_ERR(priv->busy)) {
      dev_err(&spi->dev, "Failed to configure GPIO for busy pin.");
      err = PTR_ERR(priv->busy);
      goto err_platform;
    }

    for (int i = 0; i < 3; i++) {
      struct gpio_desc *dio = devm_gpiod_get_index(
        &spi->dev,
        "dio-map",
        i,
        GPIOD_IN
      );

      if (IS_ERR(dio)) {
        dev_err(&spi->dev, "Failed to configure GPIO for DIO%d.", i);
        err = PTR_ERR(dio);
        goto err_platform;
      }

      priv->dios[i] = dio;
    }
  }

  // Request IRQs for each DIO.
  for (int i = 0; i < ARRAY_SIZE(priv->dios); i++) {
    int irq = gpiod_to_irq(priv->dios[i]);
    if (irq < 0) {
      dev_err(&spi->dev, "Failed to register IRQ for DIO%d.", i);
      goto err_irq;
    }

    priv->irqs[i] = irq;
  }

  // Register the DIO IRQs to their interrupt handlers.

#define DIO_IRQ(i, f, n) devm_request_threaded_irq( \
  &spi->dev, \
  priv->irqs[i], \
  NULL, \
  f, \
  IRQF_TRIGGER_RISING | IRQF_ONESHOT, \
  n, \
  priv \
)

  if (
    (err = DIO_IRQ(0, sx1280_done_irq, "sx1280_done"))
    || (err = DIO_IRQ(1, sx1280_timeout_irq, "sx1280_timeout"))
    || (err = DIO_IRQ(2, sx1280_error_irq, "sx1280_error"))
  ) {
    dev_err(&spi->dev, "Failed to request IRQ.");
    goto err_irq;
  }

  // Define SPI settings according to SX1280 datasheet.
  spi_set_drvdata(spi, priv);
  spi->bits_per_word = 8;
  spi->max_speed_hz = 18000000;  // 18 MHz
  spi->mode = 0;                 // CPOL = 0, CPHA = 0

  // Apply the SPI settings above and handle errors.
  if ((err = spi_setup(spi))) {
    dev_err(&spi->dev, "Failed to apply SPI settings.");
    goto err_spi;
  }

  // Wait until the chip is not busy to initiate setup.
  err = sx1280_wait_on_gpio(priv->busy, 0, priv->pdata.startup_timeout_us);
  if (err) {
    dev_err(&spi->dev, "Startup sequence timed out.");
    goto err_busy;
  }

  if ((err = sx1280_setup(priv, SX1280_MODE_LORA))) {
    goto err_setup;
  }

  // Instruct the chip to use the specified DIO as IRQ.
  // TODO: Add other interrupt classes as necessary.
  sx1280_set_dio_irq_params(
    spi,
    SX1280_IRQ_RX_DONE,
    (u16[]) {
      SX1280_IRQ_RX_DONE | SX1280_IRQ_TX_DONE,
      SX1280_IRQ_RX_TX_TIMEOUT,
      0
    }
  );

  // Initialize the work queue and work items for packet transmission.
  priv->xmit_queue = alloc_workqueue(
    "%s",
    WQ_MEM_RECLAIM,
    0,
    netdev_name(netdev)
  );

  INIT_WORK(&priv->tx_work, sx1280_tx_work);

  // Register the new net device.
  // The first one will appear as interface lora0.
  if ((err = register_netdev(netdev))) {
    goto err_register;
  }

  dev_info(&spi->dev, "SX1280 interface device initialized: %s\n", netdev->name);
  return 0;

err_register:
  destroy_workqueue(priv->xmit_queue);
err_spi:
err_irq:
err_busy:
err_setup:
err_platform:
  free_netdev(netdev);
  return err;
}

static void sx1280_remove(struct spi_device *spi) {
  struct sx1280_priv *priv = spi_get_drvdata(spi);

  cancel_work_sync(&priv->tx_work);
  destroy_workqueue(priv->xmit_queue);
  unregister_netdev(priv->netdev);
  free_netdev(priv->netdev);
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
