/* VAN bus driver for TSS463 VAN Controller with SPI Interface
 *
 * Copyright 2018 Danny Milosavljevic <dannym@scratchpost.org>
 *
 * Based on CAN bus driver for HI3110 CAN Controller with SPI Interface
 * Copyright 2016 Timesys Corporation
 *
 * Based on Microchip 251x CAN Controller (mcp251x) Linux kernel driver
 * Copyright 2009 Christian Pellegrin EVOL S.r.l.
 * Copyright 2007 Raymarine UK, Ltd. All Rights Reserved.
 * Copyright 2006 Arcom Control Systems Ltd.
 *
 * Based on CAN bus driver for the CCAN controller written by
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix
 * - Simon Kallweit, intefo AG
 * Copyright 2007
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

/* Sender: If we do not want an ACK and must not get one.
   Receiver: If an ACK was requested by the sender (FYI only) */
#define CANFD_DRAK 0x20

/* "Reply" request, f.e. read-register. */
#define CANFD_RNW 0x40

/* TODO: Support EXT some more. */
/* Notes on "Reply" requests:

We send a Reply request: RNW=1, RTR=1, CHTx=0, CHRx=0 [somewhat like "transmit"].
	Afterwards, CHRx=1, but CHTx may be unchanged (if there was an in-frame reply).
	A Reply request potentially needs both Rx and Tx of one channel for the same frame.

----------------------------------- or ------------------------------------------------------------

We wait for a Reply request: RNW=1, RTR=0, CHTx=1, CHRx=0 [like "receive"]
	Afterwards, CHRx=1.

We immediately reply: RNW=1, RTR=0, CHTx=0, CHRx=0 (in-frame!) [somewhat like "send"]
	Afterwards, CHTx=1, CHRx=1.

We reply later: RNW=1, RTR=0, CHTx=0, CHRx=1 [like "send"]
	Afterwards, CHTx=1, CHRx unchanged.

*/

#define TSS463AA_TX_ECHO_SKB_MAX 1

static int tss463aa_enable_dma = 1; /* Enable SPI DMA. Default: 1 (On) */
module_param(tss463aa_enable_dma, int, 0444);
MODULE_PARM_DESC(tss463aa_enable_dma, "Enable SPI DMA. Default: 1 (On)");

enum tss463aa_model {
	CAN_TSS463AA_TSS463AA = 0x4631,
};

/* TODO: Allow setting CAN FD "data" bit rate (which must be at least as high as the arbitration bitrate) */

/* Note: These contain the sizes of the driver's SPI buffers.
The chip buffer can't hold more than 31 Byte (per payload message).
But this here needs to fit "address" and "control" as well. */

#define TSS463AA_RX_BUF_LEN 33
#define TSS463AA_TX_BUF_LEN 33

#define TSS463AA_CHANNEL0_OFFSET 0x10
#define TSS463AA_CHANNEL_SIZE 0x08
#define TSS463AA_CHANNEL_COUNT 14

struct tss463aa_priv {
	struct can_priv can; /* must be first member */
	struct net_device *net;
	struct spi_device *spi;
	enum tss463aa_model model;
	__u32 xtal_clock_frequency;
	/* Because of the shared buffer, each channel can only either send or receive at the same time.
	In order to reduce complexity, just have each channel do either send or receive at all times.
	As an exception, allow RNW RTR channels to SHORTLY receive - even though they usually transmit (this does count as "listening").
	This flag array mostly exists because we don't want to have races checking the actual CHRx - also, the CHRx can intermittently change while listeningchannels is a constant */
	bool listeningchannels[TSS463AA_CHANNEL_COUNT];
	bool immediate_reply_channels[TSS463AA_CHANNEL_COUNT];
	/* Both we and the chip fiddle with the CHRx register bits.  When a message arrives, it will change from 0 to 1.
	   But also if we don't want to handle a message, it stays 1.  The ISR can't distinguish which it is.
	   So introduce our values of CHRx here for the ISR to check. */
	bool our_CHRxs[TSS463AA_CHANNEL_COUNT];

	struct mutex tss463aa_lock;

	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;

	struct spi_transfer t_lead;
	struct spi_transfer t_address;
	struct spi_transfer t_control;
	struct spi_transfer t_body;

	struct sk_buff *tx_skb;
	int tx_len;

	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct restart_work;

	int force_quit;
	int after_suspend;
#define TSS463AA_AFTER_SUSPEND_UP 1
#define TSS463AA_AFTER_SUSPEND_DOWN 2
#define TSS463AA_AFTER_SUSPEND_POWER 4
#define TSS463AA_AFTER_SUSPEND_RESTART 8
	int restart_tx;
	struct regulator *power;
	struct clk *clk;
	struct gpio_desc *reset;
	bool aa55_sync;
};

/* Note: assert XTAL < (MAX_UDELAY_MS 1000 us = 5000 us) for udelay to work. */
#define XTAL_us(cycles) DIV_ROUND_UP(((cycles)*1000000), priv->xtal_clock_frequency)

static int __must_check tss463aa_hw_set_up_spi_trans(struct spi_device *spi)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	struct spi_transfer t_lead = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = 0,
		.delay_usecs = XTAL_us(4),
	};
	struct spi_transfer t_address = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.tx_dma = priv->spi_tx_dma,
		.rx_dma = priv->spi_rx_dma,
		.len = 1,
		.delay_usecs = XTAL_us(8),
	};
	struct spi_transfer t_control = {
		.tx_buf = priv->spi_tx_buf ? priv->spi_tx_buf + 1 : NULL,
		.rx_buf = priv->spi_rx_buf ? priv->spi_rx_buf + 1 : NULL,
		.tx_dma = tss463aa_enable_dma ? priv->spi_tx_dma + 1 : 0,
		.rx_dma = tss463aa_enable_dma ? priv->spi_rx_dma + 1 : 0,
		.len = 1,
		.delay_usecs = XTAL_us(15),
	};
	struct spi_transfer t_body = {
		.tx_buf = priv->spi_tx_buf ? priv->spi_tx_buf + 2 : NULL,
		.rx_buf = priv->spi_rx_buf ? priv->spi_rx_buf + 2 : NULL,
		.tx_dma = tss463aa_enable_dma ? priv->spi_tx_dma + 2 : 0,
		.rx_dma = tss463aa_enable_dma ? priv->spi_rx_dma + 2 : 0,
		.len = 0,
		.delay_usecs = XTAL_us(12),
	};
	memcpy(&priv->t_lead, &t_lead, sizeof(t_lead));
	memcpy(&priv->t_address, &t_address, sizeof(t_address));
	memcpy(&priv->t_control, &t_control, sizeof(t_control));
	memcpy(&priv->t_body, &t_body, sizeof(t_body));
}

/**
 * tss463aa_hw_spi_trans - Synchronously transfers an SPI message with
 * TSS463AA-specific timing.
 *
 * The data to send is supposed to be in priv->spi_tx_buf.
 * The data received (full duplex) is going to be in priv->spi_rx_buf.
 * LEN is the length of sent data (equal to the length of the rx buffer).
 *
 * The return value is 0 if the transfer was successful and -errno
 * otherwise.
 */
static int __must_check tss463aa_hw_spi_trans(struct spi_device *spi, int len)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	BUG_ON(len < 2);

	struct spi_message m;
	int ret;

	spi_message_init(&m);
	if (tss463aa_enable_dma)
		m.is_dma_mapped = 1;
	priv->t_body.len = len - 2;
	spi_message_add_tail(&priv->t_lead, &m);
	spi_message_add_tail(&priv->t_address, &m);
	spi_message_add_tail(&priv->t_control, &m);
	spi_message_add_tail(&priv->t_body, &m);
	ret = spi_sync(spi, &m);
	if (ret) {
		dev_err(&spi->dev, "SPI transfer failed: ret = %d\n", ret);
	} else if (priv->spi_tx_buf[0] == 0 && priv->spi_tx_buf[1] == 0) {
		/* Reset 1 */
	} else if (priv->spi_tx_buf[0] == 0xFF && priv->spi_tx_buf[1] == 0xFF) {
		/* Reset 2 */
	} else if (!priv->aa55_sync) {
	} else if (priv->spi_rx_buf[0] == 0xAA && priv->spi_rx_buf[1] == 0x55) {
	} else {
		dev_err(&spi->dev, "chip is out of sync\n");
		ret = -EIO;
	}
	return ret;
}

/* Postcondition: Chip is in IDLE mode. */
static int __must_check tss463aa_hw_reset(struct spi_device *spi)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	int ret;
	dev_dbg(&spi->dev, "tss463aa_hw_reset\n");

	if (priv->reset) {
		gpiod_set_value(priv->reset, 0);
		udelay(XTAL_us(12)); /* FIXME: How long? */
		gpiod_set_value(priv->reset, 1);
		udelay(XTAL_us(12));
	}

	/* Perform soft reset and set up Motorola SPI mode */

	priv->spi_tx_buf[0] = 0;
	priv->spi_tx_buf[1] = 0;
	ret = tss463aa_hw_spi_trans(spi, 2);
	if (ret)
		dev_err(&spi->dev, "soft reset failed.\n");
	return ret;
}

#define TSS463AA_REGISTER_READ 0x60
#define TSS463AA_REGISTER_WRITE 0xE0

static u8 __must_check tss463aa_hw_read_u8(struct spi_device *spi, u8 reg)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	u8 val = 0;
	int ret;

	priv->spi_tx_buf[0] = reg;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_READ;
	priv->spi_tx_buf[2] = 0xFF; /* dummy value */
	ret = tss463aa_hw_spi_trans(spi, 3);
	if (ret) {
		dev_err(&spi->dev, "hw_read failed.\n");
		/* FIXME: Handle error. */
		return 0;
	}
	val = priv->spi_rx_buf[2];
	return val;
}

static int __must_check tss463aa_hw_write_u8(struct spi_device *spi, u8 reg, u8 val)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	int ret;

	priv->spi_tx_buf[0] = reg;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_WRITE;
	priv->spi_tx_buf[2] = val;
	ret = tss463aa_hw_spi_trans(spi, 3);
	if (ret) {
		dev_err(&spi->dev, "hw_write failed.\n");
		return ret;
	}
	/* Note: priv->spi_rx_buf[2] == 0xFF */
	return 0;
}

/** Reads register REG, masks it with KEEP, enables the bits in bitmask ENABLE and writes the result back. */
static int __must_check tss463aa_hw_fiddle_u8(struct spi_device *spi, u8 reg, unsigned keep, u8 enable) {
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	u8 val = 0;
	int ret;

	priv->spi_tx_buf[0] = reg;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_READ;
	priv->spi_tx_buf[2] = 0xFF; /* dummy value */
	ret = tss463aa_hw_spi_trans(spi, 3);
	if (ret) {
		dev_err(&spi->dev, "hw_read failed.\n");
		return ret;
	}
	val = priv->spi_rx_buf[2];
	val &= keep;
	val |= enable;
	return tss463aa_hw_write_u8(spi, reg, val);
}

#define TSS463AA_CHANNELFIELD1_RTR BIT(0)
#define TSS463AA_CHANNELFIELD1_RNW BIT(1)
#define TSS463AA_CHANNELFIELD1_RAK BIT(2)
#define TSS463AA_CHANNELFIELD1_EXT BIT(3)
#define TSS463AA_CHANNELFIELD1_IDTL_SHIFT 4
#define TSS463AA_CHANNELFIELD1_IDTL_MASK 0xF0

#define TSS463AA_CHANNELFIELD2_DRAK BIT(7)
#define TSS463AA_CHANNELFIELD2_MSGPOINTER_SHIFT 0
#define TSS463AA_CHANNELFIELD2_MSGPOINTER_MASK 0x7F

#define TSS463AA_CHANNELFIELD3_CHRX BIT(0)
#define TSS463AA_CHANNELFIELD3_CHTX BIT(1)
#define TSS463AA_CHANNELFIELD3_CHER BIT(2)
#define TSS463AA_CHANNELFIELD3_MSGLEN_SHIFT 3
#define TSS463AA_CHANNELFIELD3_MSGLEN_MASK 0xF8

#define TSS463AA_CHANNELFIELD7_IDM_MASK 0xF0
#define TSS463AA_CHANNELFIELD7_IDM_SHIFT 4

/* Reads the channel ID and the setup from the channel at CHANNEL_OFFSET.

The channel ID is stored into out_id, which is not optional.
The channel setup is stored into out_setup, if that is provided.
*/
static int __must_check tss463aa_hw_read_id(struct spi_device *spi, u8 channel_offset, u16* out_id, u8* out_setup)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	int ret;

	priv->spi_tx_buf[0] = channel_offset;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_READ;
	priv->spi_tx_buf[2] = 0xFF;
	priv->spi_tx_buf[3] = 0xFF;
	ret = tss463aa_hw_spi_trans(spi, 4);
	if (ret)
		return ret;
	*out_id = (priv->spi_rx_buf[2] << 4) |
	          ((priv->spi_rx_buf[3] & TSS463AA_CHANNELFIELD1_IDTL_MASK) >>
	           TSS463AA_CHANNELFIELD1_IDTL_SHIFT);
	if (out_setup)
		*out_setup = priv->spi_rx_buf[3] & 0x0F;
	return 0;
}

static int __must_check tss463aa_hw_set_channel_up(struct spi_device *spi, u8 offset, u16 idtag, u16 idmask, bool CHTx, bool CHRx, u8 msgpointer, u8 msglen, bool ext, bool rak, bool rnw, bool rtr, bool drak)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);

	priv->spi_tx_buf[0] = offset;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_WRITE;
	priv->spi_tx_buf[2] = idtag >> 4;
	priv->spi_tx_buf[3] = (idtag << TSS463AA_CHANNELFIELD1_IDTL_SHIFT) |
	                      (ext ? TSS463AA_CHANNELFIELD1_EXT : 0) |
	                      (rak ? TSS463AA_CHANNELFIELD1_RAK : 0) |
	                      (rnw ? TSS463AA_CHANNELFIELD1_RNW : 0) |
	                      (rtr ? TSS463AA_CHANNELFIELD1_RTR : 0);
	priv->spi_tx_buf[4] = (drak ? TSS463AA_CHANNELFIELD2_DRAK : 0) |
	                      (msgpointer << TSS463AA_CHANNELFIELD2_MSGPOINTER_SHIFT);
	priv->spi_tx_buf[5] = (CHTx ? TSS463AA_CHANNELFIELD3_CHTX : 0) |
	                      (CHRx ? TSS463AA_CHANNELFIELD3_CHRX : 0) |
	                      (msglen << TSS463AA_CHANNELFIELD3_MSGLEN_SHIFT); /* Note: Clears error, too. */
	if (tss463aa_hw_spi_trans(spi, 6))
		return -EIO;
	priv->spi_tx_buf[0] = offset + 6;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_WRITE;
	priv->spi_tx_buf[2] = idmask >> 4;
	priv->spi_tx_buf[3] = idmask << TSS463AA_CHANNELFIELD7_IDM_SHIFT;
	if (tss463aa_hw_spi_trans(spi, 4))
		return -EIO;

	return 0;
}


#define TSS463AA_COMMAND 3
#define TSS463AA_COMMAND_SLEEP BIT(6)
#define TSS463AA_COMMAND_IDLE BIT(5)
#define TSS463AA_COMMAND_ACTIVATE BIT(4)
#define TSS463AA_COMMAND_REAR BIT(3)
#define TSS463AA_COMMAND_MSDC BIT(0)

/* Note: This IMMEDIATELY stops the clock.
 * Note: To wake up, call tss463aa_reset. */
static int __must_check tss463aa_hw_sleep(struct spi_device *spi)
{
	int ret = tss463aa_hw_write_u8(spi, TSS463AA_COMMAND, TSS463AA_COMMAND_SLEEP);
	/* TODO: Poll line status to see whether it was done already? */
	if (ret)
		dev_warn(&spi->dev, "could not send chip to sleep.\n");
	return ret;
}

/* Matches a channel we can send messages on. */
static u8 __must_check tss463aa_hw_find_tx_channel(struct spi_device *spi, bool ext, bool rnw, bool rtr)
{
	u8 channel_offset;
	u8 channel;
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	/* Note:
	The datasheet claims that the RNW and RTR fields are not "compared",
	but the EXT field is.

	But the following has been observed (with TX disconnected!):

	All messages received by a channel with RNW=0, RTR=1 have RNW=0, RTR=0.
	All messages received by a channel with RNW=1, RTR=0 have RNW=1, RTR=0.
	All messages received by a channel with RNW=1, RTR=1 have RNW=1, RTR=0.

	So apparently the RNW is somehow taken into account, because why else
	were other messages filtered out?
	*/
	/* TODO: Try to find and prefer an exact ID match. */
	for (channel_offset = TSS463AA_CHANNEL0_OFFSET, channel = 0;
	     channel < TSS463AA_CHANNEL_COUNT;
	     channel_offset += TSS463AA_CHANNEL_SIZE, ++channel) {
		u8 idthcmd = tss463aa_hw_read_u8(spi, channel_offset + 1);
		bool channel_ext = (idthcmd & TSS463AA_CHANNELFIELD1_EXT) != 0;
		bool channel_rnw = (idthcmd & TSS463AA_CHANNELFIELD1_RNW) != 0;
		u8 status = tss463aa_hw_read_u8(spi, channel_offset + 3);
		bool need_receiver_free = priv->listeningchannels[channel] && rnw;
		if (channel_ext != ext || channel_rnw != rnw) /* Note: NOT RTR */
			continue;
		if ((status & TSS463AA_CHANNELFIELD3_CHTX) == 0) /* busy */
			continue;

		/* This is racy - but a sanity check only. */
		BUG_ON(need_receiver_free && (status & TSS463AA_CHANNELFIELD3_CHRX) == 0);
		return channel;
	}
	return TSS463AA_CHANNEL_COUNT;
}

/* Precondition: Buffer is not occupied */
static int __must_check tss463aa_hw_tx_frame(struct spi_device *spi, u8 channel_offset, u8 *buf, unsigned len)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);

	u8 msgpointer = 0x80 | ((tss463aa_hw_read_u8(priv->spi, channel_offset + 2) & TSS463AA_CHANNELFIELD2_MSGPOINTER_MASK) >> TSS463AA_CHANNELFIELD2_MSGPOINTER_SHIFT);
	u8 msglen = (tss463aa_hw_read_u8(priv->spi, channel_offset + 3) & TSS463AA_CHANNELFIELD3_MSGLEN_MASK) >> TSS463AA_CHANNELFIELD3_MSGLEN_SHIFT;
	if (msglen == 0) {
		dev_err(&spi->dev, "cannot transmit message via this channel - no buffer space is available.\n");
		return -EIO;
	}

	priv->spi_tx_buf[0] = msgpointer;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_WRITE;
	priv->spi_tx_buf[2] = 0; /* dummy */
	if (len > TSS463AA_TX_BUF_LEN - 3) {
		len = TSS463AA_TX_BUF_LEN - 3;
		dev_err(&spi->dev, "truncated a message since it was too long.\n");
	}
	if (len > msglen - 1) /* 1: The dummy above */ {
		len = msglen - 1;
		dev_err(&spi->dev, "truncated a message since it was too long.\n");
	}
	memcpy(priv->spi_tx_buf + 3, buf, len);
	return tss463aa_hw_spi_trans(spi, len + 3);
}

/* Precondition: Buffer is not occupied */
static int __must_check tss463aa_tx(struct spi_device *spi, struct canfd_frame *frame)
{
	u16 idt;
	u8 channel;
	u8 channel_offset;
	u8 ret;
	u8 keep = TSS463AA_CHANNELFIELD3_CHRX |
	          TSS463AA_CHANNELFIELD3_CHER;
	bool ext = true;
	bool rtr = (frame->can_id & CAN_RTR_FLAG) != 0;
	bool rnw = (frame->flags & CANFD_RNW) != 0;
	bool rak = (frame->flags & CANFD_DRAK) == 0;
	u8 len1 = frame->len + 1; /* includes the status dummy */
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	if ((frame->can_id & CAN_EFF_FLAG) != 0)
		idt = frame->can_id & CAN_EFF_MASK;
	else
		idt = frame->can_id & CAN_SFF_MASK;

	channel = tss463aa_hw_find_tx_channel(spi, ext, rnw, rtr);
	if (channel >= TSS463AA_CHANNEL_COUNT) {
		dev_err(&spi->dev, "cannot transmit message to %X since no channels are free.\n", idt);
		/* FIXME: Retry */
		return -EBUSY;
	}
	channel_offset = TSS463AA_CHANNEL0_OFFSET + channel * TSS463AA_CHANNEL_SIZE;

	ret = tss463aa_hw_write_u8(spi, channel_offset, idt >> 4);
	if (ret)
		return ret;

	ret = tss463aa_hw_tx_frame(spi, channel_offset, frame->data, frame->len);
	if (ret)
		return ret;

	ret = tss463aa_hw_write_u8(spi, channel_offset + 1,
	                           (idt << TSS463AA_CHANNELFIELD1_IDTL_SHIFT) |
	                           (ext ? TSS463AA_CHANNELFIELD1_EXT : 0) |
	                           (rtr ? TSS463AA_CHANNELFIELD1_RTR : 0) |
	                           (rnw ? TSS463AA_CHANNELFIELD1_RNW : 0) |
	                           (rak ? TSS463AA_CHANNELFIELD1_RAK : 0));
	if (ret)
		return ret;

	/* Transmit */
	if (rnw) { /* "Reply" request (either sent or received): Allow receiving, once. */
		priv->our_CHRxs[channel] = false;
		keep &= ~TSS463AA_CHANNELFIELD3_CHRX;
	}
	ret = tss463aa_hw_fiddle_u8(spi, channel_offset + 3, keep, len1 << TSS463AA_CHANNELFIELD3_MSGLEN_SHIFT);
	return ret;
}

/* Precondition: There is a message to be received already. */
static int __must_check tss463aa_hw_rx_frame(struct spi_device *spi, u8 channel_offset)
{
	int ret;
	struct tss463aa_priv *priv = spi_get_drvdata(spi);

	u8 msgpointer = 0x80 | ((tss463aa_hw_read_u8(priv->spi, channel_offset + 2) & TSS463AA_CHANNELFIELD2_MSGPOINTER_MASK) >> TSS463AA_CHANNELFIELD2_MSGPOINTER_SHIFT);
	u8 msglen = (tss463aa_hw_read_u8(priv->spi, channel_offset + 3) & TSS463AA_CHANNELFIELD3_MSGLEN_MASK) >> TSS463AA_CHANNELFIELD3_MSGLEN_SHIFT;
	u8 len = TSS463AA_RX_BUF_LEN - 2;
	if (msglen <= len)
		len = msglen;
	else {
		dev_warn(&spi->dev, "received message was too long - ignored.\n");
		return -E2BIG;
	}
	if (len < 1) {
		dev_warn(&spi->dev, "channel cannot receive even the status - buffer space is not available.\n");
		return -E2BIG;
	}

	priv->spi_tx_buf[0] = msgpointer;
	priv->spi_tx_buf[1] = TSS463AA_REGISTER_READ;
	memset(priv->spi_tx_buf + 2, 0, len);
	ret = tss463aa_hw_spi_trans(spi, len + 2);
	if (ret == 0) {
		u8 status = priv->spi_rx_buf[2];
		u8 r_payloadlen = status & 0x1F;
		u8 payloadlen = len - 1;
		if (r_payloadlen > payloadlen) { /* ??? */
			dev_warn(&spi->dev, "buffer occupation was reported as greater than buffer size - impossible.  Cutting occupation to 0.\n");
			priv->spi_rx_buf[2] = status &~ 0x1F;
		}
	}
	return ret;
}

/* Precondition: There is a message to be received already. */
static int __must_check tss463aa_hw_rx(struct spi_device *spi, u8 channel_offset, u16 id)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	u8* buf;
	int ret;
	struct sk_buff *skb;
	struct canfd_frame *frame;

	ret = tss463aa_hw_rx_frame(spi, channel_offset);
	if (ret)
		return ret;
	buf = priv->spi_rx_buf + 2;

	u8 dlen = buf[0] & 0x1F;
	if (dlen > CANFD_MAX_DLEN) {
		dev_warn(&spi->dev, "received message was too long - ignored.\n");
		return -E2BIG;
	}
	skb = alloc_canfd_skb(priv->net, &frame);
	if (!skb) {
		priv->net->stats.rx_dropped++;
		return -ENOMEM;
	}
	frame->len = dlen;
	memcpy(frame->data, buf + 1, dlen);

	if (buf[0] & 0x20)
		frame->can_id |= CAN_RTR_FLAG;
	if (buf[0] & 0x40)
		frame->flags |= CANFD_RNW;
	if ((buf[0] & 0x80) == 0)
		frame->flags |= CANFD_DRAK;
	/* Note: A combination RNW = 0 (write) && RTR = 1 will never happen (VAN standard). */

	frame->can_id = id | CAN_EFF_FLAG; /* Note: CAN IDs have 29 bits. */

	priv->net->stats.rx_packets++;
	priv->net->stats.rx_bytes += frame->len;

	can_led_event(priv->net, CAN_LED_EVENT_RX);

	netif_rx_ni(skb);

	return 0;
}

#define TSS463AA_LINE_CONTROL 0
#define TSS463AA_LINE_CONTROL_IVRX BIT(0)
#define TSS463AA_LINE_CONTROL_IVTX BIT(1)
#define TSS463AA_LINE_CONTROL_PC BIT(3)
#define TSS463AA_LINE_CONTROL_CD_SHIFT 4
#define TSS463AA_LINE_CONTROL_CD_MASK 0xF0

static const struct can_bittiming_const tss463aa_canfd_nominal_bittiming_const = {
	.name = KBUILD_MODNAME,
	/* FIXME fix those values: */
	.tseg1_min = 1,
	.tseg1_max = 1, /* FIXME */
	.tseg2_min = 1,
	.tseg2_max = 1, /* FIXME */
	.sjw_max = 1, /* FIXME */
	.brp_min = 1,
	.brp_max = 192,
	.brp_inc = 1,
};

/* Precondition: Device is down */
static int __must_check tss463aa_set_bittiming(struct net_device *dev)
{
	struct tss463aa_priv *priv = netdev_priv(dev);
	struct spi_device *spi = priv->spi;
	struct can_bittiming* timing = &priv->can.bittiming;
	__u32 CD;

	netdev_dbg(dev, "set_bittiming!\n");
	//timing->bitrate; /* bits/second */
	/* TODO: One day, support fractional divisors */
	/* Note: prop_seg = tseg1 / 2 */
	/* Note: phase_seg1 = tseg1 - prop_seg */
	/* Note: phase_seg2 = tseg2 */
#if 0
	if (timing->prop_seg + timing->phase_seg1 + timing->phase_seg2 + 1 != 16) {
		netdev_err(dev, "%s: Sum of all the timings has to be 16 TQ.\n", __func__);
		return -EINVAL;
	}
#endif
	if (timing->prop_seg*2 + timing->phase_seg2 + 1 != 16) {
		netdev_err(dev, "%s: Sum of the timings has to be 16 TQ.\n", __func__);
		return -EINVAL;
	}

	/* Note: Max. VAN transfer rate: 1 Mbit/s */

	CD = ilog2(timing->brp);
	if (CD >= 8 || (1 << CD) != timing->brp)
		return -EINVAL;

	return tss463aa_hw_fiddle_u8(spi, TSS463AA_LINE_CONTROL, ~TSS463AA_LINE_CONTROL_CD_MASK, CD << TSS463AA_LINE_CONTROL_CD_SHIFT);
}

#define TSS463AA_TRANSMISSION_CONTROL 1
#define TSS463AA_TRANSMISSION_CONTROL_MT BIT(0)
#define TSS463AA_TRANSMISSION_CONTROL_VER_SHIFT 1
#define TSS463AA_TRANSMISSION_CONTROL_VER_MASK 14
#define TSS463AA_TRANSMISSION_CONTROL_MR_SHIFT 4
#define TSS463AA_TRANSMISSION_CONTROL_MR_MASK 112

#define TSS463AA_DIAGNOSTIC_CONTROL 2
#define TSS463AA_DIAGNOSTIC_CONTROL_ESDC BIT(0)
#define TSS463AA_DIAGNOSTIC_CONTROL_ETIP BIT(1)
#define TSS463AA_DIAGNOSTIC_CONTROL_M_SHIFT 2
#define TSS463AA_DIAGNOSTIC_CONTROL_M_MASK 12
#define TSS463AA_DIAGNOSTIC_CONTROL_SDC_SHIFT 4
#define TSS463AA_DIAGNOSTIC_CONTROL_SDC_MASK 240

/* Precondition: TX not busy. */
static netdev_tx_t __must_check tss463aa_hard_start_xmit(struct sk_buff *skb, struct net_device *net)
{
	struct tss463aa_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	if (priv->tx_skb || priv->tx_len) {
		dev_err(&spi->dev, "hard_xmit called while tx busy\n");
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(net);
	priv->tx_skb = skb;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}

static void tss463aa_clean(struct net_device *net)
{
	struct tss463aa_priv *priv = netdev_priv(net);

	if (priv->tx_skb || priv->tx_len)
		net->stats.tx_errors++;
	if (priv->tx_skb)
		dev_kfree_skb(priv->tx_skb);
	if (priv->tx_len)
		can_free_echo_skb(priv->net, 0);
	priv->tx_skb = NULL;
	priv->tx_len = 0;
}

static int __must_check tss463aa_set_mode(struct net_device *net, enum can_mode mode)
{
	struct tss463aa_priv *priv = netdev_priv(net);

	switch (mode) {
	case CAN_MODE_START: /* Restart, starting from bus-off */
		tss463aa_clean(net);
		/* We have to delay work since SPI I/O may sleep */
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		priv->restart_tx = 1;
		if (priv->can.restart_ms == 0) /* no automatic restart */
			priv->after_suspend = TSS463AA_AFTER_SUSPEND_RESTART;
		queue_work(priv->wq, &priv->restart_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

#define TSS463AA_INTE 0x0A
#define TSS463AA_INTE_RST BIT(7) /* reset */
#define TSS463AA_INTE_TXERR BIT(4)
#define TSS463AA_INTE_TXOK BIT(3)
#define TSS463AA_INTE_RXERR BIT(2)
#define TSS463AA_INTE_RXOK BIT(1)
#define TSS463AA_INTE_RXNOK BIT(0) /* with no RAK */
#define TSS463AA_INTE_MASK 31

#define TSS463AA_LINE_STATUS 4
#define TSS463AA_LINE_STATUS_RXG BIT(0) /* receiving */
#define TSS463AA_LINE_STATUS_TXG BIT(1) /* transmitting */
#define TSS463AA_LINE_STATUS_SBA_SHIFT 2
#define TSS463AA_LINE_STATUS_SBA_MASK 12

#define TSS463AA_LINE_STATUS_SBA_RXD0 0 /* Differential communication */
#define TSS463AA_LINE_STATUS_SBA_RXD2 1 /* Fault on DATA (RXD1) */
#define TSS463AA_LINE_STATUS_SBA_RXD1 2 /* Fault on inv DATA (RXD2) */
#define TSS463AA_LINE_STATUS_SBA_MAJOR_ERROR 3 /* Major error */

#define TSS463AA_LINE_STATUS_SC BIT(4) /* error anywhen mark */
#define TSS463AA_LINE_STATUS_IDG BIT(5) /* idling */
#define TSS463AA_LINE_STATUS_SPG BIT(6) /* sleeping */

#define TSS463AA_INTERRUPT_STATUS 9
#define TSS463AA_INTERRUPT_STATUS_MASK 0x9f
#define TSS463AA_INTERRUPT_STATUS_RNOK BIT(0) /* reception without ACK OK */
#define TSS463AA_INTERRUPT_STATUS_ROK BIT(1) /* reception OK */
#define TSS463AA_INTERRUPT_STATUS_RE BIT(2) /* reception error */
#define TSS463AA_INTERRUPT_STATUS_TOK BIT(3) /* transmission OK */
#define TSS463AA_INTERRUPT_STATUS_TE BIT(4) /* transmission error */
#define TSS463AA_INTERRUPT_STATUS_RST BIT(7) /* reset */

#define TSS463AA_INTERRUPT_RESET 0x0b
#define TSS463AA_INTERRUPT_RESET_MASK 0x9f

static int __must_check tss463aa_activate(struct spi_device *spi)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	int ret;

	ret = tss463aa_hw_write_u8(spi, TSS463AA_INTE, TSS463AA_INTE_RST |
	                                TSS463AA_INTE_TXERR | TSS463AA_INTE_TXOK |
	                                TSS463AA_INTE_RXERR | TSS463AA_INTE_RXOK |
	                                TSS463AA_INTE_RXNOK);
	if (ret)
		return ret;

	/* Start transmitting/receiving on the VAN bus */
	ret = tss463aa_hw_write_u8(spi, TSS463AA_COMMAND,
	                           (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
	                           ? TSS463AA_COMMAND_IDLE
	                           : TSS463AA_COMMAND_ACTIVATE);
	if (ret)
		return ret;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

/* Given a DT node and channel number, sets up the TSS463AA accordingly (also taking into account CAN).
Precondition: Device is not up. */
static int __must_check tss463aa_set_channel_up_from_dt(struct tss463aa_priv *priv, __u8 channel, struct device_node *dt_node)
{
	struct spi_device *spi = priv->spi;
	if (channel >= TSS463AA_CHANNEL_COUNT) {
		return -EPERM;
	}
	if (dt_node && of_device_is_available(dt_node)) {
		u8 msgpointer = 0;
		u8 msglen = 0;
		u16 idtag = 0;
		u16 idmask = 0xFFF;

		bool ext = !of_property_read_bool(dt_node, "tss463aa,disable-recessive-ext");
		bool listener = of_property_read_bool(dt_node, "tss463aa,listener");
		/*
		RNW RTR CHTx CHRx Meaning
		0   0   0    ?    Transmit Message
		0   1   ?    0    Receive Message
		1   1   0    0    Reply Request
		1   0   0    0    Immediate Reply Message [transmit in-frame reply]
		1   0   0    1    Deferred reply [transmits immediately]
		1   0   1    0    Reply request detector probe

		?   ?   1    1    Inactive
		*/
		bool rnw = of_property_read_bool(dt_node, "tss463aa,rnw");
		bool rtr = of_property_read_bool(dt_node, "tss463aa,remote-transmission-request");
		bool CHRx = !listener; /* CHRx: RX done */
		bool CHTx = true; /* CHTx: TX done */
		bool immediate_reply = of_property_read_bool(dt_node, "tss463aa,immediate-reply");

		bool rak = of_property_read_bool(dt_node, "tss463aa,request-ack");

		bool drak = (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) != 0;

		if (immediate_reply) {
			CHRx = false;
			CHTx = false;
			if (!listener)
				dev_warn(&spi->dev, "channel %u: Set up to immediately reply but not listen (via devicetree).  Hint: Try adding 'tss463aa,listener'.\n", channel);
			listener = true;
		}

		priv->listeningchannels[channel] = listener;
		priv->immediate_reply_channels[channel] = immediate_reply;
		priv->our_CHRxs[channel] = CHRx;

		if (of_property_read_u8(dt_node, "tss463aa,link", &msgpointer) == 0) {
			msglen = 0;
			if (msgpointer > TSS463AA_CHANNEL_COUNT) {
				dev_err(&spi->dev, "channel %u: invalid 'tss463aa,link' in devicetree.\n", channel);
				return -EINVAL;
			}
		} else {
			if (of_property_read_u8(dt_node, "tss463aa,msgpointer", &msgpointer)) {
				dev_err(&spi->dev, "channel %u: missing 'tss463aa,msgpointer' in devicetree.\n", channel);
				return -EINVAL;
			}
			if (msgpointer > 127) {
				dev_err(&spi->dev, "channel %u: invalid 'tss463aa,msgpointer' in devicetree.\n", channel);
				return -EINVAL;
			}
			if (of_property_read_u8(dt_node, "tss463aa,msglen", &msglen)) {
				dev_err(&spi->dev, "channel %u: missing 'tss463aa,msglen' in devicetree.\n", channel);
				return -EINVAL;
			}
			if (msglen >= 32 || msglen >= TSS463AA_RX_BUF_LEN || msglen >= TSS463AA_TX_BUF_LEN || msglen == 0) { /* FIXME: Fix one-off ? */
				dev_err(&spi->dev, "channel %u: invalid 'tss463aa,msglen' in devicetree.\n", channel);
				return -EINVAL;
			}
		}

		if (of_property_read_u16(dt_node, "tss463aa,idtag", &idtag)) {
			dev_err(&spi->dev, "channel %u: missing 'tss463aa,idtag' in devicetree.\n", channel);
			return -EINVAL;
		}
		if (idtag > 0xFFF) {
			dev_err(&spi->dev, "channel %u: invalid 'tss463aa,idtag' in devicetree.\n", channel);
			return -EINVAL;
		}
		if (of_property_read_u16(dt_node, "tss463aa,idmask", &idmask)) {
			dev_err(&spi->dev, "channel %u: missing 'tss463aa,idmask' in devicetree.\n", channel);
			return -EINVAL;
		}
		if (idmask > 0xFFF) {
			dev_err(&spi->dev, "channel %u: invalid 'tss463aa,idmask' in devicetree.\n", channel);
			return -EINVAL;
		}

		return tss463aa_hw_set_channel_up(spi, TSS463AA_CHANNEL0_OFFSET + TSS463AA_CHANNEL_SIZE * channel, idtag, idmask, CHTx, CHRx, msgpointer, msglen, ext, rak, rnw, rtr, drak);
	} else {
		bool drak = (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) != 0;
		priv->listeningchannels[channel] = false;
		priv->immediate_reply_channels[channel] = false;
		priv->our_CHRxs[channel] = true;
		return tss463aa_hw_set_channel_up(spi, TSS463AA_CHANNEL0_OFFSET + TSS463AA_CHANNEL_SIZE * channel, 0, 0, true, true, 0x7F, 0, false/*ext*/, false, true, true, drak);
	}
	return 0;
}

static int tss463aa_set_channels_up_from_dt(struct spi_device *spi, struct device_node *dt_node)
{
	struct device_node *channels_node;
	struct device_node *channel_node;
	int ret;
	struct tss463aa_priv *priv = spi_get_drvdata(spi);

	channels_node = of_get_child_by_name(dt_node, "tss463aa,channels");
	if (channels_node) {
		u8 i = 0;
		for_each_child_of_node(channels_node, channel_node) {
			if (i >= TSS463AA_CHANNEL_COUNT) {
				dev_err(&spi->dev, "too many channels in device tree\n");
				break;
			}
			ret = tss463aa_set_channel_up_from_dt(priv, i++, channel_node);
			if (ret)
				return ret;
		}
	} else { /* default setup */
		bool drak = (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) != 0;
		int ret = tss463aa_hw_set_channel_up(spi, 0, 0, 0, true, false, 0, 31, true/*ext*/, false, false, true, drak);
		if (ret)
			return ret;
		ret = tss463aa_hw_set_channel_up(spi, 1, 0, 0, true, true, 32, 31, true/*ext*/, true, false, false, drak);
		if (ret)
			return ret;
		priv->our_CHRxs[0] = false;
		priv->our_CHRxs[1] = true;
	}
	return 0;
}

/* Given a DT node, sets up the TSS463AA accordingly.
Precondition: Device is not up. */
static int __must_check tss463aa_set_up_from_dt(struct spi_device *spi, struct device_node *dt_node)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	__u8 transmission_retry_count = 0;
	__u32 M;
	int ret;
	/* SDC period should be longer than the max frame length on the bus.
	Let's take 512 bits to be sure. */
	u8 SDC = 3;

	/* Set up Line Control settings */

	ret = tss463aa_hw_fiddle_u8(spi, TSS463AA_LINE_CONTROL,
	                            ~(TSS463AA_LINE_CONTROL_IVRX |
	                              TSS463AA_LINE_CONTROL_IVTX |
	                              TSS463AA_LINE_CONTROL_PC),
	                            (of_property_read_bool(dt_node, "tss463aa,inverted-rx") ?
	                             TSS463AA_LINE_CONTROL_IVRX : 0) |
	                            (of_property_read_bool(dt_node, "tss463aa,invert-tx") ?
	                             TSS463AA_LINE_CONTROL_IVTX : 0) |
	                            /* PC line control enables internal loopback */
	                            ((of_property_read_bool(dt_node, "tss463aa,pulse-coded-modulation") || (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)) ?
	                             TSS463AA_LINE_CONTROL_PC : 0));
	if (ret)
		return ret;

	/* Set up Transmission Control settings */

	ret = of_property_read_u8(dt_node, "tss463aa,transmission-retry-count", &transmission_retry_count);
	if (ret)
		return ret;
	if (transmission_retry_count > (TSS463AA_TRANSMISSION_CONTROL_MR_MASK >> TSS463AA_TRANSMISSION_CONTROL_MR_SHIFT)) {
		dev_warn(&spi->dev, "Value for 'transmission retry count' is invalid. Clamping.\n");
		transmission_retry_count = TSS463AA_TRANSMISSION_CONTROL_MR_MASK >> TSS463AA_TRANSMISSION_CONTROL_MR_SHIFT;
	}
	ret = tss463aa_hw_fiddle_u8(spi, TSS463AA_TRANSMISSION_CONTROL,
	                            ~(TSS463AA_TRANSMISSION_CONTROL_MT |
	                              TSS463AA_TRANSMISSION_CONTROL_MR_MASK),
	                            (of_property_read_bool(dt_node, "tss463aa,autonomous") ?
	                             TSS463AA_TRANSMISSION_CONTROL_MT : 0) |
	                            (transmission_retry_count << TSS463AA_TRANSMISSION_CONTROL_MR_SHIFT));
	if (ret)
		return ret;

	/* Set up Diagnostic Control settings */

	if (of_property_read_u32(dt_node, "tss463aa,diagnostic-mode", &M))
		M = 3; /* automatic selection */
	if (M > 3) {
		dev_warn(&spi->dev, "Value of 'tss463aa,diagnostic-mode' is invalid. Clamping.\n");
		return -EINVAL;
	}

	priv->aa55_sync = !of_property_read_bool(dt_node, "tss463aa,crystal-clock");

	ret = tss463aa_hw_fiddle_u8(spi, TSS463AA_DIAGNOSTIC_CONTROL,
	                            ~(TSS463AA_DIAGNOSTIC_CONTROL_ESDC |
	                              TSS463AA_DIAGNOSTIC_CONTROL_ETIP |
	                              TSS463AA_DIAGNOSTIC_CONTROL_M_MASK |
	                              TSS463AA_DIAGNOSTIC_CONTROL_SDC_MASK),
	                            (of_property_read_bool(dt_node, "tss463aa,disable-system-diagnosis") ?
	                             0 : TSS463AA_DIAGNOSTIC_CONTROL_ESDC) |
	                            (of_property_read_bool(dt_node, "tss463aa,disable-transmission-diagnosis") ?
	                             0 : TSS463AA_DIAGNOSTIC_CONTROL_ETIP) |
	                            (M << TSS463AA_DIAGNOSTIC_CONTROL_M_SHIFT) |
	                            (SDC << TSS463AA_DIAGNOSTIC_CONTROL_SDC_SHIFT));
	if (ret)
		return ret;

	return tss463aa_set_channels_up_from_dt(spi, dt_node);
}

static int __must_check tss463aa_hw_clear_channels(struct spi_device *spi, bool drak)
{
	u8 channel_offset;
	for (channel_offset = TSS463AA_CHANNEL0_OFFSET;
	     channel_offset < TSS463AA_CHANNEL0_OFFSET + TSS463AA_CHANNEL_COUNT * TSS463AA_CHANNEL_SIZE;
	     channel_offset += TSS463AA_CHANNEL_SIZE) {
		int ret = tss463aa_hw_set_channel_up(spi, channel_offset, 0, 0, true, true, 0x7F, 0, false/*ext*/, false, true, true, drak);
		if (ret)
			return ret;
	}
	return 0;
}

static int __must_check tss463aa_clear_channels(struct spi_device *spi)
{
	u8 channel;
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	bool drak = (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) != 0;
	int ret = tss463aa_hw_clear_channels(spi, drak);
	for (channel = 0; channel < TSS463AA_CHANNEL_COUNT; ++channel) {
		priv->our_CHRxs[channel] = true;
		priv->listeningchannels[channel] = false;
		priv->immediate_reply_channels[channel] = false;
	}
	return ret;
}

static int __must_check tss463aa_setup(struct spi_device *spi)
{
	int ret = tss463aa_clear_channels(spi);
	if (ret)
		return ret;
	return tss463aa_set_up_from_dt(spi, spi->dev.of_node);
}

static int __must_check tss463aa_power_enable(struct spi_device *spi, struct regulator *reg, int enable)
{
	if (IS_ERR_OR_NULL(reg))
		return 0;

	if (enable) {
		int ret = regulator_enable(reg);
		if (ret)
			dev_err(&spi->dev, "could not enable chip power.\n");
		return ret;
	} else {
		int ret = regulator_disable(reg);
		if (ret)
			dev_err(&spi->dev, "could not disable chip power.\n");
		return ret;
	}
}

static int __must_check tss463aa_stop(struct net_device *net)
{
	struct tss463aa_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	int ret;

	close_candev(net);

	/* FIXME: Wait until the chip has finished transfers? */

	priv->force_quit = 1;
	free_irq(spi->irq, priv);
	destroy_workqueue(priv->wq);
	priv->wq = NULL;

	mutex_lock(&priv->tss463aa_lock);

	/* Disable transmission&reception, disable interrupts and clear flags. */
	ret = tss463aa_clear_channels(spi);
	if (ret)
		netdev_err(net, "could not clear channels.\n");
	ret = tss463aa_hw_write_u8(spi, TSS463AA_INTE, 0x0);
	if (ret)
		netdev_err(net, "could not stop.\n");
	ret = tss463aa_hw_write_u8(spi, TSS463AA_INTERRUPT_RESET, TSS463AA_INTERRUPT_RESET_MASK);
	if (ret)
		netdev_err(net, "could not stop.\n");

	tss463aa_clean(net);

	tss463aa_hw_sleep(spi);

	priv->can.state = CAN_STATE_STOPPED;

	mutex_unlock(&priv->tss463aa_lock);

	can_led_event(net, CAN_LED_EVENT_STOP);

	return 0;
}

static void tss463aa_tx_work_handler(struct work_struct *ws)
{
	struct tss463aa_priv *priv = container_of(ws, struct tss463aa_priv, tx_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	struct canfd_frame *frame;

	mutex_lock(&priv->tss463aa_lock);
	if (priv->tx_skb) {
		if (priv->can.state == CAN_STATE_BUS_OFF) {
			tss463aa_clean(net);
		} else {
			int ret;
			frame = (struct canfd_frame *)priv->tx_skb->data;
			ret = tss463aa_tx(spi, frame);
			if (ret == -EBUSY) {
				/* See interrupt handler for the case where it retries. */
			} else if (ret) { /* error */
				/* TODO: Try again later queue_work(priv->wq, &priv->tx_work); */
				/* FIXME: Handle error here. */
			} else {
				priv->tx_len = 1 + frame->len; /* FIXME */
				can_put_echo_skb(priv->tx_skb, net, 0);
				priv->tx_skb = NULL;
			}
		}
	}
	mutex_unlock(&priv->tss463aa_lock);
}

static void tss463aa_restart_work_handler(struct work_struct *ws)
{
	struct tss463aa_priv *priv = container_of(ws, struct tss463aa_priv,
						restart_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;

	mutex_lock(&priv->tss463aa_lock);
	if (priv->after_suspend) {
		/* FIXME: Check for errors */
		tss463aa_hw_reset(spi); /* wake it up */
		tss463aa_setup(spi);
		if (priv->after_suspend & TSS463AA_AFTER_SUSPEND_RESTART) {
			tss463aa_activate(spi);
		} else if (priv->after_suspend & TSS463AA_AFTER_SUSPEND_UP) {
			netif_device_attach(net);
			tss463aa_clean(net);
			tss463aa_activate(spi);
			netif_wake_queue(net);
		} else {
			tss463aa_hw_sleep(spi);
		}
		priv->after_suspend = 0;
		priv->force_quit = 0;
	}

	if (priv->restart_tx) {
		priv->restart_tx = 0;
		/* FIXME: Check for errors */
		tss463aa_hw_reset(spi);
		tss463aa_setup(spi);
		tss463aa_clean(net);
		tss463aa_activate(spi);
		netif_wake_queue(net);
	}
	mutex_unlock(&priv->tss463aa_lock);
}

#define TSS463AA_LAST_ERROR 7
#define TSS463AA_LAST_ERROR_BOC BIT(6) /* RX: Buffer occupied */
#define TSS463AA_LAST_ERROR_BOV BIT(5) /* RX: Buffer overflow */
#define TSS463AA_LAST_ERROR_FCSE BIT(3) /* RX: Frame check sequence error */
#define TSS463AA_LAST_ERROR_ACKE BIT(2) /* TX: Collision on ACK */
#define TSS463AA_LAST_ERROR_CV BIT(1) /* RX: Manchester code violation or a physical violation */
#define TSS463AA_LAST_ERROR_FV BIT(0) /* RX: Collision on ACK */

#define TSS463AA_TRANSMISSION_STATUS 5
#define TSS463AA_TRANSMISSION_STATUS_NRT_MASK 0xF0
#define TSS463AA_TRANSMISSION_STATUS_NRT_SHIFT 4
#define TSS463AA_TRANSMISSION_STATUS_IDT_MASK 0x0F
#define TSS463AA_TRANSMISSION_STATUS_IDT_SHIFT 4

/* Note: Updates CF in place. */
static void tss463aa_update_can_error(struct tss463aa_priv *priv, struct can_frame *cf, int transmission)
{
	struct spi_device *spi = priv->spi;
	u8 last_error = tss463aa_hw_read_u8(spi, TSS463AA_LAST_ERROR);
	if (last_error & TSS463AA_LAST_ERROR_BOV) {
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	}
	if (last_error & TSS463AA_LAST_ERROR_FCSE) {
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] = CAN_ERR_PROT_FORM;
	}
	if (last_error & TSS463AA_LAST_ERROR_ACKE) {
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] |= CAN_ERR_PROT_FORM|CAN_ERR_PROT_TX;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
	}
	if (last_error & TSS463AA_LAST_ERROR_FV) {
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] |= CAN_ERR_PROT_FORM;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
	}
	if (last_error & TSS463AA_LAST_ERROR_CV) {
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] |= CAN_ERR_PROT_FORM;
	}

	if (transmission) {
		u8 transmission_status = tss463aa_hw_read_u8(spi, TSS463AA_TRANSMISSION_STATUS);
		u8 idt = ((transmission_status & TSS463AA_TRANSMISSION_STATUS_IDT_MASK) >> TSS463AA_TRANSMISSION_STATUS_IDT_SHIFT);
		u8 nrt = ((transmission_status & TSS463AA_TRANSMISSION_STATUS_NRT_MASK) >> TSS463AA_TRANSMISSION_STATUS_NRT_SHIFT);
		/* FIXME: Check whether this still works when TE is asserted! */
		cf->data[5] = idt; /* controller specific */
		cf->data[6] = nrt; /* controller specific */
	}

	/* TODO: CAN_STATE_ERROR_WARNING if degraded */
	/* TODO: Bus error
	priv->can.can_stats.bus_error++;
	stats->rx_errors++;
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	cf->data[2] |= CAN_ERR_PROT_BIT;
	cf->data[2] |= CAN_ERR_PROT_FORM;
	cf->data[2] |= CAN_ERR_PROT_STUFF;
	cf->data[3] = ecc & ECC_SEG; error location
	cf->data[2] |= CAN_ERR_PROT_TX; error during transmission
	*/
	/* TODO: Arbitrarion lost
	priv->can.can_stats.arbitration_lost++;
	stats->tx_errors++;
	cf->can_id |= CAN_ERR_LOSTARB;
	cf->data[0] = alc & 0x1f;
	*/
}

static void tss463aa_can_error(struct tss463aa_priv *priv, int transmission)
{
	struct sk_buff *skb;
	struct can_frame *cf;
	struct net_device *net = priv->net;
	skb = alloc_can_err_skb(net, &cf);
	if (!skb) {
		netdev_err(priv->net, "could not allocate CAN error frame.\n");
		return;
	}

	tss463aa_update_can_error(priv, cf, transmission);

	netif_rx_ni(skb);
}

static void tss463aa_update_can_state(struct tss463aa_priv *priv, enum can_state new_state) {
	struct net_device *net = priv->net;

	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state rx_state, tx_state;
	u8 rxerr, txerr;

	skb = alloc_can_err_skb(net, &cf);
	if (!skb)
		return;

	txerr = 1; /* FIXME */
	rxerr = 1; /* FIXME */
	cf->data[6] = txerr;
	cf->data[7] = rxerr;
	tx_state = new_state;
	rx_state = new_state;
	can_change_state(net, cf, tx_state, rx_state);
	netif_rx_ni(skb);
}

/* Note: Runs in its own thread */
static irqreturn_t tss463aa_can_ist(int irq, void *dev_id)
{
	struct tss463aa_priv *priv = dev_id;
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	u8 intf;
	u8 line_status;
	u8 channel_offset;

	mutex_lock(&priv->tss463aa_lock);

	intf = tss463aa_hw_read_u8(spi, TSS463AA_INTERRUPT_STATUS) & TSS463AA_INTERRUPT_STATUS_MASK;
	if (!intf) {
		mutex_unlock(&priv->tss463aa_lock);
		return IRQ_NONE;
	}

	while (!priv->force_quit) {
		enum can_state new_state;

		intf = tss463aa_hw_read_u8(spi, TSS463AA_INTERRUPT_STATUS) & TSS463AA_INTERRUPT_STATUS_MASK;
		if (intf == 0)
			break;

		tss463aa_hw_write_u8(spi, TSS463AA_INTERRUPT_RESET, TSS463AA_INTERRUPT_RESET_MASK); /* clear pending interrupt */

		/* There's no interrupt for line_status, so take what we can get. */

		line_status = tss463aa_hw_read_u8(spi, TSS463AA_LINE_STATUS);

		/* Update CAN state */
		switch ((line_status & TSS463AA_LINE_STATUS_SBA_MASK) >> TSS463AA_LINE_STATUS_SBA_SHIFT) {
		case TSS463AA_LINE_STATUS_SBA_RXD0:
			new_state = CAN_STATE_ERROR_ACTIVE;
			break;
		case TSS463AA_LINE_STATUS_SBA_RXD1:
			new_state = CAN_STATE_ERROR_WARNING;
			break;
		case TSS463AA_LINE_STATUS_SBA_RXD2:
			new_state = CAN_STATE_ERROR_WARNING;
			break;
		case TSS463AA_LINE_STATUS_SBA_MAJOR_ERROR:
			new_state = CAN_STATE_ERROR_PASSIVE;
			break;
		}

		if ((intf & TSS463AA_INTERRUPT_STATUS_RST) != 0) {
			dev_dbg(&spi->dev, "chip reset happened.  Glitch?\n");
			new_state = CAN_STATE_BUS_OFF;
			/* TODO: can_restart(net); */
		}

		if (new_state != priv->can.state) {
			tss463aa_update_can_state(priv, new_state);
			if (new_state == CAN_STATE_BUS_OFF) {
				can_bus_off(net);
				if (priv->can.restart_ms == 0) { /* no automatic restart */
					priv->force_quit = 1;
					tss463aa_hw_sleep(spi);
					break;
				}
			}
		}

		if (intf & (TSS463AA_INTERRUPT_STATUS_RNOK | TSS463AA_INTERRUPT_STATUS_ROK | TSS463AA_INTERRUPT_STATUS_TOK | TSS463AA_INTERRUPT_STATUS_TE | TSS463AA_INTERRUPT_STATUS_RE)) {
			u8 channel;
			for (channel_offset = TSS463AA_CHANNEL0_OFFSET, channel = 0;
			     channel < TSS463AA_CHANNEL_SIZE;
			     ++channel, channel_offset += TSS463AA_CHANNEL_SIZE) {
				u8 channel_status = tss463aa_hw_read_u8(spi, channel_offset + 3);
				if (unlikely((channel_status & TSS463AA_CHANNELFIELD3_CHER) != 0)) {
					int ret;
					/* TODO: Get error (if possible) */
					dev_warn(&spi->dev, "channel %u logged an error. Clearing it.\n", channel);
					channel_status &= ~TSS463AA_CHANNELFIELD3_CHER;
					ret = tss463aa_hw_write_u8(spi, channel_offset + 3, channel_status);
					if (ret)
						dev_err(&spi->dev, "could not clear error.\n");
				}
				if ((channel_status & TSS463AA_CHANNELFIELD3_CHRX) != 0 && !priv->our_CHRxs[channel]) { /* RX occupied */
					// TODO: Check TSS463AA_INTERRUPT_STATUS_RNOK | TSS463AA_INTERRUPT_STATUS_ROK ?

					u16 id = 0;
					u8 setup = 0;
					int ret;
					ret = tss463aa_hw_read_id(spi, channel_offset, &id, &setup);
					if (ret) {
						dev_err(&spi->dev, "could not read channel setup.\n");
					} else {
						ret = tss463aa_hw_rx(spi, channel_offset, id);
						if (ret)
							dev_err(&spi->dev, "could not read message.\n");
					}
					if ((setup & TSS463AA_CHANNELFIELD1_RNW) != 0) {
						priv->our_CHRxs[channel] = true; /* Disable reception. */
						/* "Reply" requests don't receive automatically. */
						if ((setup & (TSS463AA_CHANNELFIELD1_RNW | TSS463AA_CHANNELFIELD1_RTR)) == (TSS463AA_CHANNELFIELD1_RNW | TSS463AA_CHANNELFIELD1_RTR)) { /* RNW, RTR. So a Reply request. */
							/* If there was an in-frame reply by another module,
							   it's possible that CHTx=0 because the TX message
							   was never finished. This would mean we'd lose
							   our ability to transmit.
							   Restore it here and lose the TX message.
							   FIXME: Remove if possible. */
							channel_status |= TSS463AA_CHANNELFIELD3_CHTX;
							tss463aa_hw_write_u8(spi, channel_offset + 3, channel_status);
						}
					} else if (priv->listeningchannels[channel]) {
						if (priv->immediate_reply_channels[channel]) {
							/* Load the immediate reply again */
							tss463aa_hw_write_u8(spi, channel_offset + 3, channel_status &~ (TSS463AA_CHANNELFIELD3_CHRX | TSS463AA_CHANNELFIELD3_CHTX));
						} else {
							/* Allow receiving another message. */
							tss463aa_hw_write_u8(spi, channel_offset + 3, channel_status &~ TSS463AA_CHANNELFIELD3_CHRX);
						}
					}
				} else if ((channel_status & TSS463AA_CHANNELFIELD3_CHTX) != 0) {
					/* Record previous transmission stats */
					if (priv->tx_len) {
						net->stats.tx_packets++;
						net->stats.tx_bytes += priv->tx_len - 1;
						can_led_event(net, CAN_LED_EVENT_TX);
						if (priv->tx_len) {
							can_get_echo_skb(net, 0);
							priv->tx_len = 0;
						}
						/* Allow new transmission */
						if (!priv->tx_skb)
							netif_wake_queue(net);
					}
					if (priv->tx_skb) { /* We were busy before and a message is still pending. */
						/* Note: netif_stop_queue(net); */
						queue_work(priv->wq, &priv->tx_work);
					}
				}
			}
		}

		if (intf & (TSS463AA_INTERRUPT_STATUS_TE | TSS463AA_INTERRUPT_STATUS_RE))
			tss463aa_can_error(priv, (intf & TSS463AA_INTERRUPT_STATUS_TE) != 0);
	}
	mutex_unlock(&priv->tss463aa_lock);
	return IRQ_HANDLED;
}

static int __must_check tss463aa_open(struct net_device *net)
{
	struct tss463aa_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW; /* FIXME */
	int ret;

	ret = open_candev(net);
	if (ret)
		return ret;

	mutex_lock(&priv->tss463aa_lock);

	priv->force_quit = 0;
	priv->tx_skb = NULL;
	priv->tx_len = 0;

	ret = request_threaded_irq(spi->irq, NULL, tss463aa_can_ist,
				   flags, KBUILD_MODNAME, priv);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
		goto out_close;
	}

	priv->wq = alloc_workqueue("tss463aa_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	if (!priv->wq) {
		ret = -ENOMEM;
		goto out_free_irq;
	}
	INIT_WORK(&priv->tx_work, tss463aa_tx_work_handler);
	INIT_WORK(&priv->restart_work, tss463aa_restart_work_handler);

	ret = tss463aa_hw_reset(spi);
	if (ret)
		goto out_free_wq;

	ret = tss463aa_setup(spi);
	if (ret)
		goto out_free_wq;

	ret = tss463aa_activate(spi);
	if (ret)
		goto out_free_wq;

	can_led_event(net, CAN_LED_EVENT_OPEN);
	netif_wake_queue(net);
	mutex_unlock(&priv->tss463aa_lock);

	return 0;

 out_free_wq:
	destroy_workqueue(priv->wq);
 out_free_irq:
	free_irq(spi->irq, priv);
	tss463aa_hw_sleep(spi);
 out_close:
	close_candev(net);
	mutex_unlock(&priv->tss463aa_lock);
	return ret;
}

static void tss463aa_get_stats64(struct net_device *net, struct rtnl_link_stats64 *stats) {
	netdev_stats_to_stats64(stats, &net->stats);
}

#define TSS463AAIOREAR SIOCDEVPRIVATE

static void tss463aa_ndo_tx_timeout(struct net_device *net)
{
	struct tss463aa_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	net->stats.tx_dropped++;
	netif_trans_update(net);
	/* FIXME: Set CHER for the channel of the affected packet (good luck). */
	netif_start_queue(net);
}

static int tss463aa_ndo_ioctl(struct net_device *net, struct ifreq *req, int cmd)
{
	int ret;
	struct tss463aa_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	/* FIXME: Check that device is not sleeping/suspended. */
	switch (cmd) {
	case TSS463AAIOREAR:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		mutex_lock(&priv->tss463aa_lock);
		ret = tss463aa_hw_write_u8(spi, TSS463AA_COMMAND,
		                           TSS463AA_COMMAND_REAR |
		                           ((priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		                            ? TSS463AA_COMMAND_IDLE
		                            : TSS463AA_COMMAND_ACTIVATE));
		mutex_unlock(&priv->tss463aa_lock);
		return ret;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops tss463aa_netdev_ops = {
	.ndo_open = tss463aa_open,
	.ndo_stop = tss463aa_stop,
	.ndo_start_xmit = tss463aa_hard_start_xmit,
	.ndo_get_stats64 = tss463aa_get_stats64,
	.ndo_do_ioctl = tss463aa_ndo_ioctl,
	.ndo_tx_timeout = tss463aa_ndo_tx_timeout,
};

static const struct of_device_id tss463aa_of_match[] = {
	{
		.compatible	= "atmel,tss463aa",
		.data		= (void *)CAN_TSS463AA_TSS463AA,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, tss463aa_of_match);

static const struct spi_device_id tss463aa_id_table[] = {
	{
		.name		= "tss463aa",
		.driver_data	= (kernel_ulong_t)CAN_TSS463AA_TSS463AA,
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, tss463aa_id_table);

static int tss463aa_can_probe(struct spi_device *spi)
{
	struct device_node *dt_node = spi->dev.of_node;
	const struct of_device_id *of_id;
	struct net_device *net;
	struct tss463aa_priv *priv;
	struct clk *clk;
	unsigned long freq;
	int ret;

	if (!dt_node) {
		dev_err(&spi->dev, "Missing device tree node.\n");
		return -EINVAL;
	}
	of_id = of_match_node(tss463aa_of_match, dt_node); /* FIXME: Or of_match_device(tss463aa_of_match, &spi->dev);*/
	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&spi->dev, "no chip clock source defined\n");
		return PTR_ERR(clk);
	}
	freq = clk_get_rate(clk);
	if (freq < 4000) {
		dev_err(&spi->dev, "XTAL clock frequency is so low we can't calibrate our delays anymore.\n");
		return -EPERM;
	}

	/* Sanity check */
	if (freq > 8000000)
		return -ERANGE;

	/* Allocate can/net device */
	net = alloc_candev(sizeof(struct tss463aa_priv), TSS463AA_TX_ECHO_SKB_MAX); /* FIXME */
	if (!net)
		return -ENOMEM;

	ret = clk_prepare_enable(clk);
	if (ret)
		goto out_free;

	net->netdev_ops = &tss463aa_netdev_ops;
	net->watchdog_timeo = 1 * HZ; /* 1 s (way too long) */
	net->flags |= IFF_ECHO;

	priv = netdev_priv(net);
	priv->reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	priv->xtal_clock_frequency = freq;
	priv->can.state = CAN_STATE_STOPPED;
	priv->can.bittiming_const = &tss463aa_canfd_nominal_bittiming_const;
	priv->can.do_set_bittiming = tss463aa_set_bittiming;
	/* priv->can.do_set_data_bittiming */
	priv->can.do_set_mode = tss463aa_set_mode;
	priv->can.clock.freq = freq /* / 16*/; /* Note: TS/s */
	priv->can.ctrlmode_supported = CAN_CTRLMODE_FD | CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_LOOPBACK;
	can_set_static_ctrlmode(net, CAN_CTRLMODE_FD_NON_ISO);

	if (of_id)
		priv->model = (enum tss463aa_model)of_id->data;
	else
		priv->model = spi_get_device_id(spi)->driver_data;
	priv->net = net;
	priv->clk = clk;

	spi_set_drvdata(spi, priv);

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		goto out_clk;

	priv->power = devm_regulator_get_optional(&spi->dev, "vdd");
	if (PTR_ERR(priv->power) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		goto out_clk;
	}

	ret = tss463aa_power_enable(spi, priv->power, 1);
	if (ret)
		goto out_clk;

	priv->spi = spi;
	mutex_init(&priv->tss463aa_lock);

	/* If requested, allocate DMA buffers */
	if (tss463aa_enable_dma) {
		spi->dev.coherent_dma_mask = ~0;

		/* Minimum coherent DMA allocation is PAGE_SIZE, so allocate
		 * that much and share it between Tx and Rx DMA buffers.
		 */
		priv->spi_tx_buf = dmam_alloc_coherent(&spi->dev,
						       PAGE_SIZE,
						       &priv->spi_tx_dma,
						       GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (priv->spi_tx_buf + (PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
							(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			tss463aa_enable_dma = 0;
		}
	}
	if (!tss463aa_enable_dma) {
		priv->spi_tx_dma = 0;
		priv->spi_rx_dma = 0;
		priv->spi_tx_buf = devm_kzalloc(&spi->dev, TSS463AA_TX_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_probe;
		}
		priv->spi_rx_buf = devm_kzalloc(&spi->dev, TSS463AA_RX_BUF_LEN,
						GFP_KERNEL);

		if (!priv->spi_rx_buf) {
			ret = -ENOMEM;
			goto error_probe;
		}
	}
	ret = tss463aa_hw_set_up_spi_trans(spi);
	if (ret)
		goto error_probe;

	SET_NETDEV_DEV(net, &spi->dev);

	ret = tss463aa_hw_reset(spi);
	if (ret) {
		if (ret == -ENODEV)
			dev_err(&spi->dev, "Cannot initialize %x. Wrong wiring?\n",
				priv->model);
		goto error_probe;
	}

	ret = register_candev(net);
	if (ret)
		goto error_probe;

	devm_can_led_init(net);
	netdev_info(net, "%x successfully initialized.\n", priv->model);

	return 0;

 error_probe:
	tss463aa_power_enable(spi, priv->power, 0);

 out_clk:
	if (!IS_ERR(clk))
		clk_disable_unprepare(clk);

 out_free:
	free_candev(net);

	dev_err(&spi->dev, "Probe failed, err=%d\n", -ret);
	return ret;
}

static int tss463aa_can_remove(struct spi_device *spi)
{
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	unregister_candev(net);

	tss463aa_power_enable(spi, priv->power, 0);

	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

	free_candev(net);

	return 0;
}

static int __maybe_unused tss463aa_can_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct tss463aa_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	priv->force_quit = 1;
	disable_irq(spi->irq);

	/* Note: at this point neither IST nor workqueues are running.
	 * open/stop cannot be called anyway so locking is not needed
	 */
	if (netif_running(net)) {
		netif_device_detach(net);

		tss463aa_hw_sleep(spi);
		priv->after_suspend = TSS463AA_AFTER_SUSPEND_UP;
	} else {
		priv->after_suspend = TSS463AA_AFTER_SUSPEND_DOWN;
	}

	if (!IS_ERR_OR_NULL(priv->power)) {
		regulator_disable(priv->power);
		priv->after_suspend |= TSS463AA_AFTER_SUSPEND_POWER;
	}

	return 0;
}

static int __maybe_unused tss463aa_can_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct tss463aa_priv *priv = spi_get_drvdata(spi);

	if (priv->after_suspend & TSS463AA_AFTER_SUSPEND_POWER)
		tss463aa_power_enable(spi, priv->power, 1);

	if (priv->after_suspend & TSS463AA_AFTER_SUSPEND_UP) {
		queue_work(priv->wq, &priv->restart_work);
	} else {
		priv->after_suspend = 0;
	}

	priv->force_quit = 0;
	enable_irq(spi->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(tss463aa_can_pm_ops, tss463aa_can_suspend, tss463aa_can_resume);

static struct spi_driver tss463aa_can_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = tss463aa_of_match,
		.pm = &tss463aa_can_pm_ops,
	},
	.id_table = tss463aa_id_table, /* FIXME ??? */
	.probe = tss463aa_can_probe,
	.remove = tss463aa_can_remove,
};

module_spi_driver(tss463aa_can_driver);

MODULE_AUTHOR("Danny Milosavljevic <dannym@scratchpost.org>");
MODULE_DESCRIPTION("Atmel TSS463 VAN driver");
MODULE_LICENSE("GPL v2");

