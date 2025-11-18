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
#include <net/cfg80211.h>

#include "sx1280.h"

struct sx1280_config {

};

/* The private, internal structure for the SX1280 driver. */
struct sx1280_priv {
  struct sx1280_platform_data pdata;
  struct net_device *netdev;
  struct spi_device *spi;

  struct gpio_desc *busy;
  struct gpio_desc *dio;
  struct gpio_desc *reset;
  int dio_index;
  int irq;

  /* The packet mode that the chip is currently in. */
  enum sx1280_mode mode;

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
    } else if (wait < 10000) { /* TODO: no magic numbers */
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
  if (!(err = sx1280_wait_busy(priv))) {
    err = spi_sync_transfer(priv->spi, xfers, num_xfers);
  }

  return err;
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

static int sx1280_set_standby(
  struct sx1280_priv *priv,
  u8 mode
) {
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
  union sx1280_modulation_params params
) {
  int err;
  u8 tx[4] = {
    SX1280_CMD_SET_MODULATION_PARAMS,
  };


  memcpy(&tx[1], params.raw, 3);

  if ((err = sx1280_write(priv, tx, ARRAY_SIZE(tx)))) {
    dev_err(&priv->spi->dev, "SetModulationParams failed: %d\n", err);
    return err;
  }

  return 0;
}

static int sx1280_set_packet_params(
  struct sx1280_priv *priv,
  union sx1280_packet_params params
) {
  int err;
  u8 tx[8] = { SX1280_CMD_SET_PACKET_PARAMS };
  memcpy(&tx[1], params.raw, 7);

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

  union sx1280_packet_params params;
  switch (priv->mode) {
  case SX1280_MODE_FLRC:
    params.flrc = priv->pdata.flrc.packet;

    /* TODO: Pad FLRC packets less than 6 bytes. */
    if (skb->len < 6 || skb->len > 127) {
      netdev_warn(netdev, "invalid FLRC packet size: %d bytes\n", skb->len);
      goto drop;
    }

    params.flrc.payload_length = skb->len;
    break;
  case SX1280_MODE_GFSK:
    params.gfsk = priv->pdata.gfsk.packet;

    if (skb->len > 255) {
      netdev_warn(netdev, "invalid GFSK packet size: %d bytes\n", skb->len);
      goto drop;
    }

    params.gfsk.payload_length = skb->len;
    break;
  case SX1280_MODE_LORA:
    params.lora = priv->pdata.lora.packet;

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
    || sx1280_set_tx(priv, priv->pdata.period_base, priv->pdata.period_base_count)
  ) {
    goto drop;
  }

  u8 buffer[256];
  sx1280_read_buffer(priv, 0x00, buffer, skb->len);
  netdev_dbg(netdev, "tx: %*ph\n", skb->len, buffer);

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
  schedule_delayed_work(&priv->status_check, HZ);
}

#endif

/**
 * Sets the chip into continuous RX mode to listen for packets.
 * @context process & locked
 */
static int sx1280_listen(struct sx1280_priv *priv) {
  int err;
  union sx1280_packet_params params;
  params.gfsk = priv->pdata.gfsk.packet;
  params.gfsk.payload_length = 255; /* expand to other modes */

  if (
    (err = sx1280_set_packet_params(priv, params))
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

  dev_info(&spi->dev, "interrupt: mask=0x%04x\n", mask);

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

    /* Switch back into listening. */
    err = sx1280_listen(priv);

    /* Restart the Tx packet queue. */
    netif_start_queue(netdev);
  }

  if ((mask & SX1280_IRQ_RX_DONE) || (mask & SX1280_IRQ_SYNC_WORD_VALID)) {
    union sx1280_packet_status status;
    if ((err = sx1280_get_packet_status(priv, &status))) {
      goto rx_error;
    }

    /* TODO: Set the RSSI to be publicly accessible. */
    switch (priv->mode) {
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
      // goto rx_error;
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

  if (mask & SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE) {

  }

  if (mask & SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARD) {

  }

  if (mask & SX1280_IRQ_RANGING_MASTER_RESULT_VALID) {

  }

  if (mask & SX1280_IRQ_RANGING_MASTER_TIMEOUT) {

  }

  if (mask & SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID) {

  }

  if (mask & SX1280_IRQ_CAD_DONE) {

  }

  if (mask & SX1280_IRQ_CAD_DETECTED) {

  }

  if (mask & SX1280_IRQ_PREAMBLE_DETECTED) {
    /* If in ranging mode, the mask is instead for "Advanced Ranging Done". */
    if (priv->mode == SX1280_MODE_RANGING) {

    }
  }

  /* Acknowledge all interrupts. */
  sx1280_clear_irq_status(priv, 0xFFFF);

unlock:
  mutex_unlock(&priv->lock);
  return IRQ_HANDLED;
}

#define CONCAT32(a, b) (((u32) (a) << 16) | (u32) (b))
#define MATCH2(a, b, v) case CONCAT32(a, b): mod->bitrate_bandwidth = v; break;

static int sx1280_parse_dt_flrc(struct sx1280_priv *priv) {
  /* Convenience definitions for conciseness. */
  struct sx1280_platform_data *pdata = &priv->pdata;
  struct sx1280_flrc_params *flrc = &pdata->flrc;
  struct sx1280_flrc_modulation_params *mod = &flrc->modulation;
  struct sx1280_flrc_packet_params *pkt = &flrc->packet;
  struct device *dev = &priv->spi->dev;

  u32 bitrate_kbs = 1300;
  const char *coding_rate = "3/4";
  const char *bt = "1.0";
  u32 preamble_bits = 8;
  u32 sync_word_bits = 32;
  u32 sync_word_match[3] = { 0, 0, 0 };
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
    of_property_read_u32(child, "crc-bytes", &crc_bytes);
    of_property_read_u32(child, "crc-seed", &crc_seed);
    disable_whitening = of_property_read_bool(child, "disable-whitening");
    of_property_read_variable_u32_array(child, "sync-words", sync_words, 0, 3);

    of_node_put(child);
  }

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
    dev_err(dev, "Invalid value for flrc.bitrate-kbs.\n");
    return -EINVAL;
  }

  /* Coding rate */
  if (strcmp(coding_rate, "1/2") == 0) {
    mod->coding_rate = SX1280_FLRC_CR_1_2;
  } else if (strcmp(coding_rate, "3/4") == 0) {
    mod->coding_rate = SX1280_FLRC_CR_3_4;
  } else if (strcmp(coding_rate, "1/1") == 0) {
    mod->coding_rate = SX1280_FLRC_CR_1_1;
  } else {
    dev_err(dev, "Invalid value for flrc.coding-rate.\n");
    return -EINVAL;
  }

  /* Bandwidth-time */
  if (strcmp(bt, "off") == 0) {
    mod->bandwidth_time = SX1280_BT_OFF;
  } else if (strcmp(bt, "1.0") == 0) {
    mod->bandwidth_time = SX1280_BT_1_0;
  } else if (strcmp(bt, "0.5") == 0) {
    mod->bandwidth_time = SX1280_BT_0_5;
  } else {
    dev_err(dev, "Invalid value for flrc.bt.\n");
    return -EINVAL;
  }

  /* Preamble length (bits) */
  if (
    preamble_bits % 4 != 0
    || preamble_bits > 32
    || preamble_bits < 4
  ) {
    dev_err(dev, "Invalid value for flrc.preamble-bits.\n");
    return -EINVAL;
  }

  pkt->agc_preamble_length = preamble_bits << 2;

  /* Sync word length (bits) */
  switch (sync_word_bits) {
  case 0:
    pkt->sync_word_length = SX1280_FLRC_SYNC_WORD_NOSYNC;
    break;
  case 32:
    pkt->sync_word_length = SX1280_FLRC_SYNC_WORD_LEN_P32S;
    break;
  default:
    dev_err(dev, "Invalid value for flrc.sync-word-bits.\n");
    return -EINVAL;
  }

  /* Sync word combination */
  pkt->sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_OFF;

  for (int i = 0; i < 2; i++) {
    if (sync_word_match[i] > 1) {
      dev_err(dev, "Invalid value for flrc.sync-word-match.\n");
      return -EINVAL;
    }

    pkt->sync_word_match |= sync_word_match[i] << i;
  }

  /*
   * Header type is necessarily configured as variable length because fixed
   * length is incompatible with general-purpose packet transmission using the
   * Linux networking stack, which may pass packets of any size.
   */
  pkt->header_type = SX1280_RADIO_PACKET_VARIABLE_LENGTH;

  /* CRC bytes */
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
    dev_err(dev, "Invalid value for flrc.crc-bytes.\n");
    return -EINVAL;
  }

  /* Whitening */
  pkt->whitening = disable_whitening
    ? SX1280_WHITENING_DISABLE
    : SX1280_WHITENING_ENABLE;

  /* CRC seed */
  flrc->crc_seed = crc_seed;

  return 0;
}

#define SX1280_DEFAULT_GFSK_BITRATE_KBS 2000
#define SX1280_DEFAULT_GFSK_BANDWIDTH_KHZ 2400
#define SX1280_DEFAULT_GFSK_MOD_IND 50
#define SX1280_DEFAULT_GFSK_BT "0.5"
#define SX1280_DEFAULT_GFSK_CRC_SEED 0xFF
#define SX1280_DEFAULT_GFSK_CRC_POLYNOMIAL 0x1021

static int sx1280_parse_dt_gfsk(struct sx1280_priv *priv) {
  /* Convenience definitions for conciseness. */
  struct sx1280_platform_data *pdata = &priv->pdata;
  struct sx1280_gfsk_params *gfsk = &pdata->gfsk;
  struct sx1280_gfsk_modulation_params *mod = &gfsk->modulation;
  struct sx1280_gfsk_packet_params *pkt = &gfsk->packet;
  struct device *dev = &priv->spi->dev;

  /* Populate defaults for GFSK mode. */
  u16 bitrate_kbs = 2000;
  u16 bandwidth_khz = 2400;
  u32 mod_index = 50;
  const char *bt = "0.5";
  u32 preamble_bits = 32;
  u32 sync_word_bytes = 5;
  u32 sync_word_match[3] = { 1, 0, 0 };
  bool fixed_length = false;
  u32 crc_bytes = 2;
  bool disable_whitening = true; /* TODO: default to false */
  u32 sync_words[3] = { 0, 0, 0 };
  u16 crc_seed = SX1280_DEFAULT_GFSK_CRC_SEED;
  u16 crc_polynomial = SX1280_DEFAULT_GFSK_CRC_POLYNOMIAL;

  struct device_node *child = of_get_child_by_name(dev->of_node, "gfsk");

  if (child) {
    /* TODO: Handle errors other than being not present. */
    of_property_read_u16(child, "bitrate-kbs", &bitrate_kbs);
    of_property_read_u16(child, "bandwidth-khz", &bandwidth_khz);
    of_property_read_u32(child, "modulation-index", &mod_index);
    of_property_read_string(child, "bt", &bt);
    of_property_read_u32(child, "preamble-bits", &preamble_bits);
    of_property_read_u32(child, "sync-word-bytes", &sync_word_bytes);
    of_property_read_u32_array(child, "sync-word-match", sync_word_match, 3);
    fixed_length = of_property_read_bool(child, "fixed-length");
    of_property_read_u32(child, "crc-bytes", &crc_bytes);
    // disable_whitening = of_property_read_bool(child, "disable-whitening");
    of_property_read_variable_u32_array(child, "sync-words", sync_words, 0, 3);
    of_property_read_u16(child, "crc-seed", &crc_seed);
    of_property_read_u16(child, "crc-polynomial", &crc_polynomial);

    of_node_put(child);
  }

  /* Bitrate-bandwidth */
  u32 brbw = CONCAT32(bitrate_kbs, bandwidth_khz);

  switch (brbw) {
    MATCH2(2000, 2400, SX1280_GFSK_BR_2_000_BW_2_4)
    MATCH2(1600, 2400, SX1280_GFSK_BR_1_600_BW_2_4)
    MATCH2(1000, 2400, SX1280_GFSK_BR_1_000_BW_2_4)
    MATCH2(1000, 1200, SX1280_GFSK_BR_1_000_BW_1_2)
    MATCH2(800, 2400, SX1280_GFSK_BR_0_800_BW_2_4)
    MATCH2(800, 1200, SX1280_GFSK_BR_0_800_BW_1_2)
    MATCH2(500, 1200, SX1280_GFSK_BR_0_500_BW_1_2)
    MATCH2(500, 600, SX1280_GFSK_BR_0_500_BW_0_6)
    MATCH2(400, 1200, SX1280_GFSK_BR_0_400_BW_1_2)
    MATCH2(400, 600, SX1280_GFSK_BR_0_400_BW_0_6)
    MATCH2(250, 600, SX1280_GFSK_BR_0_250_BW_0_6)
    MATCH2(250, 300, SX1280_GFSK_BR_0_250_BW_0_3)
    MATCH2(125, 300, SX1280_GFSK_BR_0_125_BW_0_3)
  default:
    dev_err(
      dev,
      "Invalid combination of gfsk.bitrate-kbs and gfsk.bandwidth-khz.\n"
    );
    return -EINVAL;
  }

  /* Modulation index */
  if (
    (mod_index % 25 != 0 && mod_index != 35)
    || mod_index > 400
  ) {
    dev_err(dev, "Invalid value for gfsk.modulation-index.\n");
    return -EINVAL;
  }

  mod->modulation_index = mod_index / 25 - 1;

  /* Bandwidth-time */
  if (strcmp(bt, "off") == 0) {
    mod->bandwidth_time = SX1280_BT_OFF;
  } else if (strcmp(bt, "1.0") == 0) {
    mod->bandwidth_time = SX1280_BT_1_0;
  } else if (strcmp(bt, "0.5") == 0) {
    mod->bandwidth_time = SX1280_BT_0_5;
  } else {
    dev_err(dev, "Invalid value for gfsk.bt.\n");
    return -EINVAL;
  }

  /* Preamble length (bits) */
  if (
    preamble_bits % 4 != 0
    || preamble_bits > 32
    || preamble_bits < 4
  ) {
    dev_err(dev, "Invalid value for gfsk.preamble-bits.\n");
    return -EINVAL;
  }

  pkt->preamble_length = (preamble_bits << 2) - 0x10;

  /* Sync word length */
  if (sync_word_bytes < 1 || sync_word_bytes > 5) {
    dev_err(dev, "Invalid value for gfsk.sync-word-bytes.\n");
    return -EINVAL;
  }

  pkt->sync_word_length = ((sync_word_bytes - 1) * 2);

  /* Sync word combination */
  pkt->sync_word_match = SX1280_RADIO_SELECT_SYNCWORD_OFF;

  for (int i = 0; i < 2; i++) {
    if (sync_word_match[i] > 1) {
      dev_err(dev, "Invalid value for gfsk.sync-word-match.\n");
      return -EINVAL;
    }

    pkt->sync_word_match |= sync_word_match[i] << (i + 4);
  }

  /* Header type */
  pkt->header_type = SX1280_RADIO_PACKET_VARIABLE_LENGTH;

  /* CRC bytes */
  pkt->crc_length = crc_bytes << 4;

  /* Whitening */
  pkt->whitening = disable_whitening
    ? SX1280_WHITENING_DISABLE
    : SX1280_WHITENING_ENABLE;

  /* CRC */
  gfsk->crc_seed = crc_seed;
  gfsk->crc_polynomial = crc_polynomial;

  return 0;
}

static int sx1280_parse_dt_lora(struct sx1280_priv *priv) {
  /* Convenience definitions for conciseness. */
  struct sx1280_platform_data *pdata = &priv->pdata;
  struct sx1280_lora_modulation_params *mod = &pdata->lora.modulation;
  struct sx1280_lora_packet_params *pkt = &pdata->lora.packet;
  struct device *dev = &priv->spi->dev;

  u32 spreading_factor = 12;
  u32 bandwidth_khz = 1600;
  const char *coding_rate = "4/7";
  bool disable_li;
  u32 preamble_bits = 8;
  bool implicit_header;
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
    disable_crc = of_property_read_bool(child, "disable-crc");
    invert_iq = of_property_read_bool(child, "invert-iq");

    of_node_put(child);
  }

  if (spreading_factor < 5 || spreading_factor > 12) {
    dev_err(dev, "Property lora.spreading-factor out of range.\n");
    return -EINVAL;
  }

  mod->spreading = spreading_factor << 4;

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
    dev_err(dev, "Invalid value for lora.bandwidth-khz.\n");
    return -EINVAL;
  }

  /* Coding rate parsing. */
  if (
    strlen(coding_rate) != 3
    || coding_rate[0] != '4'
    || coding_rate[1] != '/'
    || coding_rate[2] < '5'
    || coding_rate[2] > '8'
  ) {
    dev_err(dev, "Invalid lora.coding-rate. Must be 4/5, 4/6, 4/7, or 4/8.\n");
    return -EINVAL;
  }

  if (disable_li) {
    mod->coding_rate = coding_rate[2] - '4';
  } else {
    switch (coding_rate[2]) {
    case '5':
      mod->coding_rate = SX1280_LORA_CR_LI_4_5;
      break;
    case '6':
      mod->coding_rate = SX1280_LORA_CR_LI_4_6;
      break;
    case '7':
      /*
       * 4/7 LI coding is not available.
       * Default to using 4/7 coding without LI.
       */
      mod->coding_rate = SX1280_LORA_CR_4_7;
      break;
    case '8':
      mod->coding_rate = SX1280_LORA_CR_LI_4_8;
      break;
    }
  }

  /* Preamble length (bits). */
  u32 pb_mant = preamble_bits;
  u8 pb_exp = 0;

  /*
   * Reduce the preamble bit count into a mantissa and exponent, where both
   * are allowed to be [1..15].
   */
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
    dev_err(dev, "Invalid value for lora.preamble-bits.\n");
    return -EINVAL;
  }

  /*
   * The format used by the SX1280 to encode preamble length is a strange
   * choice. It uses bits 7-4 to encode the exponent and bits 3-0 for the
   * mantissa of the preamble length, in bits. Both components must be in the
   * range [1..15].
   *
   * This increases the maximum preamble length to 491,520 over the 255 that
   * would ordinarily be representable, but 255 is itself an excessively large
   * preamble size (8 is typical).
   */
  pkt->preamble_length = (pb_exp << 4) | (u8) pb_mant;

  /* Header type. */
  pkt->header_type = implicit_header
    ? SX1280_IMPLICIT_HEADER
    : SX1280_EXPLICIT_HEADER;

  /* CRC enabling. */
  pkt->crc = disable_crc
    ? SX1280_LORA_CRC_DISABLE
    : SX1280_LORA_CRC_ENABLE;

  /* IQ inversion. */
  pkt->invert_iq = invert_iq
    ? SX1280_LORA_IQ_INVERTED
    : SX1280_LORA_IQ_STD;

  return 0;
}

static int sx1280_parse_dt_ranging(struct sx1280_priv *priv) {
  return 0;
}

static int sx1280_parse_dt(struct sx1280_priv *priv) {
  int err;
  struct sx1280_platform_data *pdata = &priv->pdata;
  struct device *dev = &priv->spi->dev;
  struct device_node *node = dev->of_node;

  /* Default values for top-level device tree properties. */
  const char *mode = "gfsk";
  s32 power_dbm = 0;
  u32 ramp_time_us = 20;
  u32 rf_freq_hz = 2400000000;
  u32 startup_timeout_us = 10000;
  u32 tx_timeout_us = 1000000;

  /* Overwrite the default values if they are specified. */
  of_property_read_string(node, "mode", &mode);
  of_property_read_s32(node, "power-dbm", &power_dbm);
  of_property_read_u32(node, "rf-freq-hz", &rf_freq_hz);
  of_property_read_u32(node, "startup-timeout-us", &startup_timeout_us);
  of_property_read_u32(node, "tx-timeout-us", &tx_timeout_us);

  if (strcmp(mode, "flrc") == 0) {
    pdata->mode = SX1280_MODE_FLRC;
  } else if (strcmp(mode, "gfsk") == 0) {
    pdata->mode = SX1280_MODE_GFSK;
  } else if (strcmp(mode, "lora") == 0) {
    pdata->mode = SX1280_MODE_LORA;
  } else if (strcmp(mode, "ranging") == 0) {
    pdata->mode = SX1280_MODE_RANGING;
  } else {
    dev_err(dev, "Invalid value for mode.\n");
    return -EINVAL;
  }

  /* Power */
  if (power_dbm < -18 || power_dbm > 13) {
    dev_err(dev, "Invalid value for power-dbm.\n");
    return -EINVAL;
  }

  pdata->power = power_dbm + 18;


  /* Ramp time */
  if (
    ramp_time_us < 2
    || ramp_time_us > 20
    || ramp_time_us % 2 != 0
    || ramp_time_us == 14
  ) {
    dev_err(dev, "Invalid value for ramp-time-us.\n");
    return -EINVAL;
  }

  if (ramp_time_us <= 12) {
    pdata->ramp_time = (ramp_time_us - 2) << 4;
  } else {
    pdata->ramp_time = (ramp_time_us + 8) << 3;
  }

  /*
   * Convert the Hz frequency into the fixed-point representation that the chip
   * natively understands.
   */
  pdata->rf_freq = SX1280_FREQ_HZ_TO_PLL(rf_freq_hz);

  /* Startup timeout */
  pdata->startup_timeout_us = startup_timeout_us;

  /*
   * Convert the timeout and period base into nanoseconds so that an integer
   * division can be performed for maximum precision within constraints.
   */
  u64 timeout_ns = (u64) tx_timeout_us * 1000;
  u64 period_base_ns;

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
    dev_err(dev, "Invalid value for timeout-us.\n");
    return -EINVAL;
  }

  /*
   * Perform a ceiling division to determine the cycle count.
   *
   * The timeout specified is always a minimum bound on the timeout that could
   * also be affected by outside factors such as kernel timing, timer precision,
   * etc. When a timeout is specified, it's crucial that the driver guarantees
   * that _at least_ that amount of time has passed before timing out.
   */
  pdata->period_base_count = (u16) (timeout_ns / period_base_ns)
    + (timeout_ns % period_base_ns != 0);

  /*
   * Parse all sub-nodes of the device tree, containing the properties
   * configuring each of the different packet modes of the SX1280.
   */
  if (
    (err = sx1280_parse_dt_flrc(priv))
    || (err = sx1280_parse_dt_gfsk(priv))
    || (err = sx1280_parse_dt_lora(priv))
    || (err = sx1280_parse_dt_ranging(priv))
  ) {
    return err;
  }

  return 0;
}

/**
 * Parses busy GPIO and DIO GPIOs.
 * @param priv - The internal SX1280 driver structure.
 * @param dt - Whether a device tree configuration or platform data is used.
 */
static int sx1280_parse_gpios(struct sx1280_priv *priv, bool dt) {
  int err;
  struct device *dev = &priv->spi->dev;

  dev_dbg(dev, "sx1280_parse_gpios");

  /*
   * 1. Configure the busy pin GPIO.
   * 2. Configure the DIO1, DIO2, DIO3 GPIOs.
   */

  priv->dio = NULL;

  if (dt) {
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

    priv->busy = gpio_to_desc(priv->pdata.busy_gpio);
    if (!priv->busy || (err = gpiod_direction_input(priv->busy))) {
      dev_err(dev, "Failed to configure GPIO for busy pin.\n");
      return err;
    }

    for (int i = 0; i < 3; i++) {
      int gpio_num = priv->pdata.dio_gpios[i];
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

    priv->reset = gpio_to_desc(priv->pdata.reset_gpio);
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
 * @param mode - The mode in which to put the SX1280.
 */
static int sx1280_setup(struct sx1280_priv *priv, enum sx1280_mode mode) {
  struct spi_device *spi = priv->spi;
  struct sx1280_platform_data *pdata = &priv->pdata;
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
  u8 circuit_mode = SX1280_STATUS_CIRCUIT_MODE(status);
  u8 command_status = SX1280_STATUS_COMMAND_STATUS(status);
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
  union sx1280_modulation_params mod_params;
  union sx1280_packet_params pkt_params;

  switch (mode) {
  case SX1280_MODE_FLRC:
    mod_params.flrc = pdata->flrc.modulation;
    pkt_params.flrc = pdata->flrc.packet;
    break;
  case SX1280_MODE_GFSK:
    mod_params.gfsk = pdata->gfsk.modulation;
    pkt_params.gfsk = pdata->gfsk.packet;
    break;
  case SX1280_MODE_LORA:
    mod_params.lora = pdata->lora.modulation;
    pkt_params.lora = pdata->lora.packet;
    break;
  case SX1280_MODE_RANGING:
    mod_params.lora = pdata->ranging.modulation;
    pkt_params.lora = pdata->ranging.packet;
    break;
  }

  /* TODO: Allow customization */
  u8 sync_word[5] = { 0xD3, 0x91, 0xD3, 0x91, 0xD3 };
  u16 sync_word_addr = SX1280_REG_SYNC_ADDRESS_1_BYTE_4;

  u8 crc_poly[2] = {
    priv->pdata.gfsk.crc_polynomial >> 8,
    priv->pdata.gfsk.crc_polynomial & 0xFF
  };
  u16 crc_poly_addr = SX1280_REG_CRC_POLYNOMIAL_DEFINITION_MSB;

  u8 crc_seed[2] = {
    priv->pdata.gfsk.crc_seed >> 8,
    priv->pdata.gfsk.crc_seed & 0xFF
  };
  u16 crc_seed_addr = SX1280_REG_CRC_MSB_INITIAL_VALUE;

  if (
    (err = sx1280_set_packet_type(priv, mode))
    || (err = sx1280_set_rf_frequency(priv, pdata->rf_freq))

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
    || (err = sx1280_write_register(priv, sync_word_addr, sync_word, 5))
    || (err = sx1280_write_register(priv, crc_poly_addr, crc_poly, 2))
    || (err = sx1280_write_register(priv, crc_seed_addr, crc_seed, 2))
    || (err = sx1280_set_tx_params(priv, pdata->power, pdata->ramp_time))
    // || (err = sx1280_set_auto_fs(priv, true))
  ) {
    return err;
  }

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

  /* Allocate a new net device (without registering, yet). */
  struct net_device *netdev = alloc_etherdev(sizeof(struct sx1280_priv));
  if (!netdev) {
    dev_err(&spi->dev, "failed to alloc etherdev\n");
    return -ENOMEM;
  }

  /* Set properties of the net device. */
  netdev->netdev_ops = &sx1280_netdev_ops;
  strcpy(netdev->name, "radio%d");
  SET_NETDEV_DEV(netdev, &spi->dev);
  eth_hw_addr_random(netdev);

  /* Create and register the priv structure. */
  struct sx1280_priv *priv = netdev_priv(netdev);
  priv->initialized = false;
  priv->mode = SX1280_MODE_GFSK; /* default mode after reset */
  priv->netdev = netdev;
  priv->spi = spi;
  mutex_init(&priv->lock);

  /*
   * Attempt to get legacy platform data.
   * Otherwise, parse the device tree.
   */
  struct sx1280_platform_data *legacy_platform = dev_get_platdata(&spi->dev);
  if (legacy_platform) {
    priv->pdata = *legacy_platform;
  } else {
    if ((err = sx1280_parse_dt(priv))) {
      dev_err(&spi->dev, "failed to parse device tree\n");
      goto error_free;
    }
  }

  /*
   * Parse GPIOs according to whether a device tree or platform data is used.
   */
  if ((err = sx1280_parse_gpios(priv, !legacy_platform))) {
    dev_err(&spi->dev, "failed to configure GPIOs\n");
    goto error_free;
  }

  /* Set netdev settings according to device tree. */
  /* TODO: Allow packet size and mode to be changed from userspace. */
  switch (priv->pdata.mode) {
  case SX1280_MODE_FLRC:
    netdev->mtu = SX1280_FLRC_PAYLOAD_LENGTH_MAX;
    break;
  case SX1280_MODE_GFSK:
    netdev->mtu = SX1280_GFSK_PAYLOAD_LENGTH_MAX;
    break;
  case SX1280_MODE_LORA:
    netdev->mtu = SX1280_LORA_PAYLOAD_LENGTH_MAX;
    break;
  case SX1280_MODE_RANGING:
    /* Packet transmission is not permitted in ranging mode. */
    netdev->mtu = 0;
    break;
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

  if ((err = sx1280_setup(priv, priv->pdata.mode))) {
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
  // schedule_delayed_work(&priv->status_check, 10 * HZ);
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
