/*
 * holtic_hi3593.c - HoltIC HI-3593 driver
 *
 * The HI-3593 from Holt Integrated Circuits is a CMOS integrated circuit
 * for interfacing a Serial Peripheral Interface (SPI) enabled microcontroller
 * to the ARINC 429 serial bus.
 * This device has two receivers and single transmitter
 *
 * Copyright (C) 2015 Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/of_gpio.h>

#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include <linux/arinc429/dev.h>

#define DRV_NAME "hi3593"

/* Default ARINC429 clock is 1MHz, but chip allows using clocks in 1-30MHz range
 * by using internal counter configured via ACLK DIV register */
#define DEFAULT_ARINC429_CLK_FREQ 1000000

#define RX_FIFO_SIZE 32
#define TX_FIFO_SIZE 32

#define NUM_LABELS 256

/* Instruction opcodes */
/*------------------------------------------------------------------------------
 *					# Data
 *	ID			Value	bytes	Description
 *------------------------------------------------------------------------------*/
#define CMD_NOP_ZERO		0x00	/*  0	No operation			*/
#define CMD_MASTER_RESET	0x04	/*  0	Master Reset			*/
#define CMD_WRITE_TCR		0x08	/*  1	Write Transmit
						Control Register		*/
#define CMD_WRITE_MSG		0x0C	/*  4	Write ARINC 429 message
						to Transmit FIFO		*/
#define CMD_WRITE_R1_CR		0x10	/*  1	Write Receiver 1
						Control Register		*/
#define CMD_WRITE_R1_LABELS	0x14	/*  32	Write label values to Receiver 1
						label memory. Starting with
label 0xFF, consecutively set or reset each label in descending order.
For example, if the first data byte is programmed to 10110010 then labels
FF, FD FC and F9 will be set and FE, FB, FA and F8 will be reset.	*/

#define CMD_WRITE_R1_PLMR	0x18	/*  3	Write Receiver 1 Priority-Label
						Match Registers. The data field
consists of three eight-bit labels. The first data byte is written to
P-L filter #3, the second to P-L filter #2, and the last byte to filter #1	*/

#define CMD_WRITE_R2_CR		0x24	/*  1	Write Receiver 2
						Control Register		*/
#define CMD_WRITE_R2_LABELS	0x28	/*  32	Write label values to Receiver 2
						label memory. Starting with
label 0xFF, consecutively set or reset each label in descending order.
For example, if the first data byte is programmed to 10110010 then labels
FF, FD FC and F9 will be set and FE, FB, FA and F8 will be reset.		*/

#define CMD_WRITE_R2_PLMR	0x2C	/*  3	Write Receiver 2 Priority-Label
						Match Registers. The data field
consists of three eight-bit labels. The first eight bits is written to
P-L filter #3, the second to P-L filter #2, and the last byte to filter #1	*/

#define CMD_WRITE_FLAG		0x34	/*  1	Write Flag / Interrupt
						Assignment Register		*/
#define CMD_WRITE_ACLK_DIV	0x38	/*  1	Write ACLK Division Register	*/
#define CMD_TX			0x40	/*  0	Transmit current contents of
						Transmit FIFO if Transmit Control
						Register bit 5 (TMODE) is a "0"	*/
#define CMD_SOFT_RESET		0x44	/*  0	Software Reset. Clears the
						Transmit and Receive FIFOs and
						the Priority-Label Registers	*/
#define CMD_SET_R1_LABELS	0x48	/*  0	Set all bits in Receiver 1 label
						memory to a "1"			*/
#define CMD_SET_R2_LABELS	0x4C	/*  0	Set all bits in Receiver 2 label
						memory to a "1"			*/
#define CMD_READ_TSR		0x80	/*  1	Read Transmit Status Register	*/
#define CMD_READ_TCR		0x84	/*  1	Read Transmit Control Register	*/
#define CMD_READ_R1_STATUS	0x90	/*  1	Read Receiver 1 Status Register	*/
#define CMD_READ_R1_CTRL	0x94	/*  1	Read Receiver 1
						Control Register 		*/
#define CMD_READ_R1_LABELS	0x98	/*  32	Read label values from Receiver 1
						label memory. */
#define CMD_READ_R1_PLMR	0x9C	/*  3	Read Receiver 1 Priority-Label
						Match Registers.		*/
#define CMD_READ_R1_MSG		0xA0	/*  4	Read one ARINC 429 message from
						the Receiver 1 FIFO		*/
#define CMD_READ_R1_PLR1	0xA4	/*  3	Read Receiver 1 Priority-Label
						Register #1, ARINC429 bytes 2,3
						& 4 (bits 9 - 32)		*/
#define CMD_READ_R1_PLR2	0xA8	/*  3	Read Receiver 1 Priority-Label
						Register #2, ARINC429 bytes 2,3
						& 4 (bits 9 - 32)		*/
#define CMD_READ_R1_PLR3	0xAC	/*  3	Read Receiver 1 Priority-Label
						Register #3, ARINC429 bytes 2,3
						& 4 (bits 9 - 32)		*/
#define CMD_READ_R2_STATUS	0xB0	/*  1	Read Receiver 2 Status Register	*/
#define CMD_READ_R2_CTRL	0xB4	/*  1	Read Receiver 2
						Control Register		*/
#define CMD_READ_R2_LABELS	0xB8	/*  32	Read label values from
						Receiver 2 label memory.	*/
#define CMD_READ_R2_PLMR	0xBC	/*  3	Read Receiver 2 Priority-Label
						Match Registers.		*/
#define CMD_READ_R2_MSG		0xC0	/*  4	Read one ARINC 429 message from
						the Receiver 2 FIFO		*/
#define CMD_READ_R2_PLR1	0xC4	/*  3	Read Receiver 2 Priority-Label
						Register #1, ARINC429 bytes 2,3
						& 4 (bits 9 - 32)		*/
#define CMD_READ_R2_PLR2	0xC8	/*  3	Read Receiver 2 Priority-Label
						Register #2, ARINC429 bytes 2,3
						& 4 (bits 9 - 32)		*/
#define CMD_READ_R2_PLR3	0xCC	/*  3	Read Receiver 2 Priority-Label
						Register #3, ARINC429 bytes 2,3
						& 4 (bits 9 - 32)		*/
#define CMD_READ_FLAG		0xD0	/*  1	Read Flag / Interrupt Assignment
						Register			*/
#define CMD_READ_ACLK_DIV	0xD4	/*  1	Read ACLK Division Register	*/
#define CMD_NOP_ONES		0xFF	/*  0	No operation.			*/

/* FLAG / Interrupt assignment register bits */
#define R1_FLAG_FIFO_EMPTY	(0)	/* FIFO empty */
#define R1_FLAG_FIFO_FULL	(1)	/* FIFO contains 32 messages */
#define R1_FLAG_FIFO_HALF_FULL	(2)	/* FIFO contains 16 messages */
#define R1_FLAG_FIFO_NOT_EMPTY	(3)	/* FIFO is not empty */
#define R1_INT_MSG_RECEIVED	(0 << 2) /* any valid message received */
#define R1_INT_PL1_MSG_RECEIVED	(1 << 2) /* Message in Priority Label mailbox #1 */
#define R1_INT_PL2_MSG_RECEIVED	(2 << 2) /* Message in Priority Label mailbox #2 */
#define R1_INT_PL3_MSG_RECEIVED	(3 << 2) /* Message in Priority Label mailbox #3 */
#define R2_FLAG_FIFO_EMPTY	(0 << 4) /* same as above for second receiver */
#define R2_FLAG_FIFO_FULL	(1 << 4)
#define R2_FLAG_FIFO_HALF_FULL	(2 << 4)
#define R2_FLAG_FIFO_NOT_EMPTY	(3 << 4)
#define R2_INT_MSG_RECEIVED	(0 << 6)
#define R2_INT_PL1_MSG_RECEIVED	(1 << 6)
#define R2_INT_PL2_MSG_RECEIVED	(2 << 6)
#define R2_INT_PL3_MSG_RECEIVED	(3 << 6)

/* Receiver control register bits */
#define RECEIVER_FLIP			(1 << 7)
#define RECEIVER_SD9			(1 << 6)
#define RECEIVER_SD10			(1 << 5)
#define RECEIVER_SDON			(1 << 4)
#define RECEIVER_PARITY			(1 << 3)
#define RECEIVER_LABEL_FILTERS_ENABLED	(1 << 2)
#define RECEIVER_PRIORITY_MATCH_ON	(1 << 1)
#define RECEIVER_RATE			(1 << 0)

/* Receiver status register bits */
#define RECEIVER_STATUS_PL3		(1 << 5)
#define RECEIVER_STATUS_PL2		(1 << 4)
#define RECEIVER_STATUS_PL1		(1 << 3)
#define RECEIVER_STATUS_FIFO_FULL	(1 << 2)
#define RECEIVER_STATUS_FIFO_HALF_FULL	(1 << 1)
#define RECEIVER_STATUS_FIFO_EMPTY	(1 << 0)

/* Transmitter control register bits */
#define TRANSMITTER_HIZ			(1 << 7)
#define TRANSMITTER_FLIP		(1 << 6)
#define TRANSMITTER_MODE		(1 << 5)
#define TRANSMITTER_SELF_TEST		(1 << 4)
#define TRANSMITTER_ODD_EVEN		(1 << 3)
#define TRANSMITTER_PARITY		(1 << 2)
#define TRANSMITTER_RATE		(1 << 0)

/* Transmitter status bits */
#define TRANSMITTER_STATUS_FULL		(1 << 2)
#define TRANSMITTER_STATUS_HALF_FULL	(1 << 1)
#define TRANSMITTER_STATUS_EMPTY	(1 << 0)


/* channel types */
enum channel_type {
	FIRST_CHANNEL,

	RECEIVER_1 = FIRST_CHANNEL,
	RECEIVER_2,
	NUM_RECEIVERS,

	TRANSMITTER = NUM_RECEIVERS,

	NUM_CHANNELS
};

const char const* channel_type_names[] = {
	__stringify(RECEIVER_1),
	__stringify(RECEIVER_2),
	__stringify(TRANSMITTER)
};

/* opcodes to write channel Control Register */
static const u8 write_cr_cmds[NUM_CHANNELS] =
	{CMD_WRITE_R1_CR, CMD_WRITE_R2_CR, CMD_WRITE_TCR};
/* opcodes to read channel Status Register */
static const u8 read_sr_cmds[NUM_CHANNELS] =
	{CMD_READ_R1_STATUS, CMD_READ_R2_STATUS, CMD_READ_TSR};
/* opcodes to receive/send message over channel */
static const u8 io_msg_cmds[NUM_CHANNELS] =
	{CMD_READ_R1_MSG, CMD_READ_R2_MSG, CMD_WRITE_MSG};

/* label flip register bits for each channel */
static const u8 label_flip_bits[NUM_CHANNELS] =
	{RECEIVER_FLIP, RECEIVER_FLIP, TRANSMITTER_FLIP};
/* rate register bits for each channel */
static const u8 rate_bits[NUM_CHANNELS] =
	{RECEIVER_RATE, RECEIVER_RATE, TRANSMITTER_RATE};
/* parity register bits for each channel */
static const u8 parity_bits[NUM_CHANNELS] =
	{RECEIVER_PARITY, RECEIVER_PARITY, TRANSMITTER_PARITY};


/* ARINC429 bus specifies two rates HIGH and LOW */
enum channel_rate {
	RATE_HIGH,
	RATE_LOW
};

/* time interval to wait (in ms) before re-enabling single message RX interrupt
 * calculated as 16 messages * 32 bits * ARINC bit time + word gap bit time
 * (bit time is in clocks) */
#define HIGH_RATE_DELAY		((16 * 32 * 10 + 40) / 1000)
#define LOW_RATE_DELAY		((16 * 32 * 80 + 320) / 1000)

struct hi3593_priv;

/* HoltIC HI-3953 single channel private data
 *
 * @arinc429 - common ARINC429 interface private data
 * @ndev: network device private data
 * @adev: HI-3593 chip specific private data
 * @type: channel type
 * @rate: channel rate
 * @work: workitem for RX or TX message processing
 * @irq_enable_timer: timeout to re-enable single message RX interrupt
 */
struct hi3593_channel_priv {
	struct arinc429_priv	arinc429;	/* must be the first member */
	struct net_device	*ndev;
	struct hi3593_priv	*adev;

	enum channel_type	type;
	enum channel_rate	rate;

	struct work_struct	work;

	struct timer_list	irq_enable_timer;
};

/* HoltIC HI-3953 chip specific private data
 *
 * @spi: spi device
 * @dev_lock: lock protecting this structure
 * @active_channels: amount of opened channels
 * @channels: channels private data
 * @ctrl_regs: cached control registers for each channel
 * @flag_iar: cached FLAG/Interrupt Assignment Register
 * @r1_label_filter: label filter bitmap in direct order (from 0000 to 0377)
 * @r2_label_filter: same as above for second receiver, do not forget to
                     to reverse byte order befor writing into HW as it takes
                     label bitmap from 0377 to 0000
 * @hw_reset: external master reset GPIO pin
 * @r1_int: R1INT interrupt GPIO pin
 * @r1_flag: R1FLAG interrupt GPIO pin
 * @r2_int: R2INT interrupt GPIO pin
 * @r2_flag: R2FLAG interrupt GPIO pin
 * @tx_full: TFULL interrupt GPIO pin
 * @tx_empty: TFEMPTY interrupt GPIO pin
 * @freq: ARINC429 bus reference clock
 * @txq: TX queue
 */
struct hi3593_priv {
	struct spi_device		*spi;
	struct mutex			dev_lock;
	int				active_channels;
	struct hi3593_channel_priv	*channels[NUM_CHANNELS];
	u8				ctrl_regs[NUM_CHANNELS];
	u8				flag_iar;
	DECLARE_BITMAP(			r1_label_filter, NUM_LABELS);
	DECLARE_BITMAP(			r2_label_filter, NUM_LABELS);
	int				hw_reset;
	int				r1_int;
	int				r1_flag;
	int				r2_int;
	int				r2_flag;
	int				tx_full;
	int				tx_empty;
	u32				freq;
	struct sk_buff_head		txq;
};

static void __hi3593_rx_buf_until_empty(struct hi3593_channel_priv *chan);

static inline bool hi3593_is_transmitter(enum channel_type channel_type)
{
	return channel_type == TRANSMITTER;
}

static inline bool hi3593_is_receiver(enum channel_type channel_type)
{
	return !(hi3593_is_transmitter(channel_type));
}

static inline int __hi3593_write_ctrl_reg(struct hi3593_priv *adev,
		enum channel_type channel_type, u8 val)
{
	u8 tx_buf[2];
	int ret;

	dev_vdbg(&adev->spi->dev,
		"%s: %s CTRL = %#x\n",
		__func__, channel_type_names[channel_type], val);

	tx_buf[0] = write_cr_cmds[channel_type];
	tx_buf[1] = val;

	ret = spi_write(adev->spi, tx_buf, sizeof(tx_buf));

	/* update cached copy after successfull HW configuration */
	if (!ret)
		adev->ctrl_regs[channel_type] = val;

	return ret;
}

static inline int __hi3593_set_ctrl_reg_bits(struct hi3593_priv *adev,
		enum channel_type channel_type, u8 bits)
{
	return __hi3593_write_ctrl_reg(adev, channel_type,
			adev->ctrl_regs[channel_type] | bits);
}

static inline int __hi3593_clear_ctrl_reg_bits(struct hi3593_priv *adev,
		enum channel_type channel_type, u8 bits)
{
	return __hi3593_write_ctrl_reg(adev, channel_type,
			adev->ctrl_regs[channel_type] & ~bits);
}

static void hi3593_tx_work_handler(struct work_struct *ws)
{
	struct hi3593_channel_priv *chan = container_of(ws,
			struct hi3593_channel_priv, work);
	struct hi3593_priv *adev = chan->adev;
	struct spi_device *spi = adev->spi;
	struct sk_buff *txb;
	bool last = skb_queue_empty(&adev->txq);

	netdev_vdbg(chan->ndev, "%s: enter\n", __func__);

	while (!last) {
		txb = skb_dequeue(&adev->txq);
		last = skb_queue_empty(&adev->txq);
		if (txb) {
			struct net_device *ndev = txb->dev;
			struct arinc429_frame *af =
				(struct arinc429_frame *)txb->data;
			u8 tx_buf[5] = { CMD_WRITE_MSG, 0, 0, 0, 0 };
			int ret;

			tx_buf[1] = af->data[0];
			tx_buf[2] = af->data[1];
			tx_buf[3] = af->data[2];
			tx_buf[4] = af->label;

			netdev_vdbg(ndev,
				"tx message label: %#o, data: %#x %02x %02x\n",
				af->label, af->data[0], af->data[1], af->data[2]);

			/* Load the TX buffer into the hardware. */
			ret = spi_write(spi, tx_buf, sizeof(tx_buf));
			if (ret) {
				ndev->stats.tx_errors++;
			} else {
				ndev->stats.tx_packets++;
				ndev->stats.tx_bytes += sizeof(*af);
			}

			dev_kfree_skb(txb);

			netdev_vdbg(ndev, "packet sent, label: %#o ret: %d\n",
				af->label, ret);
		}
	}
}

static void hi3593_rx_work_handler(struct work_struct *ws)
{
	struct hi3593_channel_priv *chan = container_of(ws,
			struct hi3593_channel_priv, work);
	int irq;

	netdev_vdbg(chan->ndev, "%s: enter\n", __func__);

	__hi3593_rx_buf_until_empty(chan);

	switch (chan->type) {
	case RECEIVER_1:
		irq = gpio_to_irq(chan->adev->r1_int);
		break;
	case RECEIVER_2:
		irq = gpio_to_irq(chan->adev->r2_int);
		break;
	default:
		BUG();
		break;
	}

	enable_irq(irq);

	netdev_vdbg(chan->ndev, "%s: irq %d enabled\n", __func__, irq);
}

static int hi3593_master_reset(struct hi3593_priv *adev)
{
	int ret = 0;

	dev_dbg(&adev->spi->dev, "%s: enter\n", __func__);

	/* MR must be pulsed high for 1us to bring the part to
	 * its completely reset state. */

	if (gpio_is_valid(adev->hw_reset)) {
		gpio_set_value_cansleep(adev->hw_reset, 1);
		udelay(1);
		gpio_set_value_cansleep(adev->hw_reset, 0);
	} else {
		/* perform master reset via SPI command */
		u8 tx_buf = CMD_MASTER_RESET;
		ret = spi_write(adev->spi, &tx_buf, 1);
		udelay(1);
	}

	return ret;
}

static int hi3593_set_clock(struct spi_device *spi, u32 desired_freq)
{
	dev_dbg(&spi->dev, "%s: enter\n", __func__);

	if (desired_freq != DEFAULT_ARINC429_CLK_FREQ) {
		u8 tx_buf[2] = { CMD_WRITE_ACLK_DIV, 0x02 };
		u32 freq = 2 * DEFAULT_ARINC429_CLK_FREQ;

		/* HI-3593 clk should be even multiple of 1 MHz (up to 30 MHz) */
		do {
			if (freq == desired_freq) {
				dev_dbg(&spi->dev,
					"setting clock to %u Hz\n", freq);
				return spi_write(spi, tx_buf, 2);
			}

			freq += 2 * DEFAULT_ARINC429_CLK_FREQ;
			tx_buf[1] += 2;
		} while (freq <= 30 * DEFAULT_ARINC429_CLK_FREQ);

		return -EINVAL;
	}

	return 0;
}

static inline void hi3593_restart_timer(struct timer_list *timer,
		unsigned int ms)
{
	u32 wait_jiffies = msecs_to_jiffies(ms);

	mod_timer(timer, jiffies + wait_jiffies);
}

static void
hi3593_enable_timer(unsigned long arg)
{
	struct hi3593_channel_priv *chan = (struct hi3593_channel_priv *)arg;
	int irq;

	netdev_vdbg(chan->ndev, "%s: enter\n", __func__);

	if (chan->type == RECEIVER_1)
		irq = gpio_to_irq(chan->adev->r1_int);
	else if (chan->type == RECEIVER_2)
		irq = gpio_to_irq(chan->adev->r2_int);
	else
		BUG();

	schedule_work(&chan->work);
}

/* Reads single message from RX FIFO, make sure that FIFO contains
 * a message. HW does not report error on try to read empty FIFO.
 */
static void __hi3593_rx_buf(struct hi3593_channel_priv *chan)
{
	struct spi_device *spi = chan->adev->spi;
	struct sk_buff *skb;
	struct arinc429_frame *af;
	u8 rx_buf[4] = {0};
	int ret;
	
	netdev_vdbg(chan->ndev, "%s: enter\n", __func__);

	ret = spi_write_then_read(spi, &io_msg_cmds[chan->type], 1, rx_buf, 4);
	if (ret) {
		chan->ndev->stats.rx_errors++;
		netdev_err(chan->ndev, "unable to read message\n");
		return;
	}

	skb = alloc_arinc429_skb(chan->ndev, &af);
	if (skb == NULL) {
		netdev_err(chan->ndev, "unable to allocate skb\n");
		chan->ndev->stats.rx_dropped++;
		return;
	}

	af->data[0] = rx_buf[0];
	af->data[1] = rx_buf[1];
	af->data[2] = rx_buf[2];
	af->label = rx_buf[3];

	netif_rx_ni(skb);

	chan->ndev->stats.rx_packets++;
	chan->ndev->stats.rx_bytes += sizeof(*af);

	netdev_vdbg(chan->ndev,
		"received message label: %#o, data: %#x %02x %02x\n",
		af->label, af->data[0], af->data[1], af->data[2]);
}

static void __hi3593_rx_buf_until_empty(struct hi3593_channel_priv *chan)
{
	u8 status;
	int ret;

	while (true) {
		ret = spi_write_then_read(chan->adev->spi,
				&read_sr_cmds[chan->type], 1, &status, 1);
		if (ret) {
			netdev_err(chan->ndev, "unable to get Receiver status\n");
			return;
		}

		if (status & RECEIVER_STATUS_FIFO_EMPTY)
			return;

		__hi3593_rx_buf(chan);
	}
}

static irqreturn_t hi3593_rx_irq(int irq, void *dev_id)
{
	struct hi3593_channel_priv *chan = dev_id;

	netdev_vdbg(chan->ndev, "%s: enter\n", __func__);

	del_timer(&chan->irq_enable_timer);

	/* IRQ handler will grab all pending messages,
	 * no need for work execution this time */
	cancel_work_sync(&chan->work);

	disable_irq_nosync(irq);

	__hi3593_rx_buf_until_empty(chan);

	hi3593_restart_timer(&chan->irq_enable_timer,
		chan->rate == RATE_HIGH ? HIGH_RATE_DELAY : LOW_RATE_DELAY);

	return IRQ_HANDLED;
}

static irqreturn_t hi3593_rx_flag_irq(int irq, void *dev_id)
{
	struct hi3593_channel_priv *chan = dev_id;
	u8 msg_count = 0;
	u8 status;
	u8 i;
	int ret;

	netdev_vdbg(chan->ndev, "%s: enter\n", __func__);

	del_timer(&chan->irq_enable_timer);

	/* IRQ handler will grab all pending messages,
	 * no need for RX work execution this time */
	cancel_work_sync(&chan->work);

	ret = spi_write_then_read(chan->adev->spi,
			&read_sr_cmds[chan->type], 1, &status, 1);
	if (ret) {
		netdev_err(chan->ndev, "unable to get Receiver status\n");
		goto exit;
	}

	netdev_vdbg(chan->ndev, "%s: receiver status: %#x\n",
		__func__, status);

	if (status & RECEIVER_STATUS_FIFO_FULL) {
		msg_count = RX_FIFO_SIZE;
	} else 	if (status & RECEIVER_STATUS_FIFO_HALF_FULL) {
		msg_count = RX_FIFO_SIZE / 2;
	} else if (!(status & RECEIVER_STATUS_FIFO_EMPTY)) {
		/* there is no indication of how many messages are in RX FIFO.
		 * It is known that at least one is there, so set count to one
		 * and fall back to one-by-one read until FIFO becomes empty */
		msg_count = 1;
	} else
		goto exit;

	if (likely(msg_count > 1))
		for (i = 0; i < msg_count; i++)
			__hi3593_rx_buf(chan);
	else
		__hi3593_rx_buf_until_empty(chan);

exit:
	hi3593_restart_timer(&chan->irq_enable_timer,
		chan->rate == RATE_HIGH ? HIGH_RATE_DELAY : LOW_RATE_DELAY);

	return IRQ_HANDLED;
}

static irqreturn_t hi3593_tx_full_irq(int irq, void *dev_id)
{
	struct hi3593_channel_priv *chan = dev_id;
	struct hi3593_priv *adev = chan->adev;

	dev_vdbg(&chan->adev->spi->dev, "%s: enter\n", __func__);

	netif_stop_queue(chan->ndev);

	disable_irq_nosync(irq);

	enable_irq(gpio_to_irq(adev->tx_empty));

	return IRQ_HANDLED;
}

static irqreturn_t hi3593_tx_empty_irq(int irq, void *dev_id)
{
	struct hi3593_channel_priv *chan = dev_id;
	struct hi3593_priv *adev = chan->adev;

	dev_vdbg(&chan->adev->spi->dev, "%s: enter\n", __func__);

	netif_wake_queue(chan->ndev);

	disable_irq_nosync(irq);

	enable_irq(gpio_to_irq(adev->tx_full));

	return IRQ_HANDLED;
}

/* A little helper to prepare label filters bitmap for HW */
static inline void hi3593_reverse_bytes(u8 *dst,
		const u8 *src, unsigned int n)
{
	unsigned int i;
	src += n - 1;
	for (i = 0; i < n; i++)
		*dst++ = *src--;
}

static int
__hi3593_configure_label_filters(struct hi3593_priv *adev,
		enum channel_type channel, const unsigned long *filters)
{
	u8 reg[NUM_RECEIVERS] = {CMD_WRITE_R1_LABELS, CMD_WRITE_R2_LABELS};
	u8 data_buf[NUM_LABELS / BITS_PER_BYTE];
	struct spi_transfer cmd = {0};
	struct spi_transfer data = {0};
	struct spi_message m;

	cmd.tx_buf = &reg[channel];
	cmd.len = 1;

	/* HW takes filter bitmap in reversed byte order, starting from label 0377 */
	hi3593_reverse_bytes(data_buf, (const u8*)filters, sizeof(data_buf));

	data.tx_buf = data_buf;
	data.len = sizeof(data_buf);

	spi_message_init(&m);
	spi_message_add_tail(&cmd, &m);
	spi_message_add_tail(&data, &m);
	return spi_sync(adev->spi, &m);
}

static int __hi3593_hw_default_config(struct hi3593_priv *adev) {
	u8 tx_buf[2] = {CMD_WRITE_FLAG, adev->flag_iar};
	enum channel_type ch;
	int ret;

	ret = hi3593_master_reset(adev);
	if (ret)
		return ret;

	/* receiver flags and interrupt settings */
	ret = spi_write(adev->spi, tx_buf, sizeof(tx_buf));
	if (ret)
		return ret;

	/* setup label filters if any */
	if (!bitmap_empty(adev->r1_label_filter, NUM_LABELS)) {
		ret = __hi3593_configure_label_filters(adev,
				RECEIVER_1, adev->r1_label_filter);
		if (ret)
			return ret;
		adev->ctrl_regs[RECEIVER_1] |= RECEIVER_LABEL_FILTERS_ENABLED;
	}
	if (!bitmap_empty(adev->r2_label_filter, NUM_LABELS)) {
		ret = __hi3593_configure_label_filters(adev,
				RECEIVER_2, adev->r2_label_filter);
		if (ret)
			return ret;
		adev->ctrl_regs[RECEIVER_2] |= RECEIVER_LABEL_FILTERS_ENABLED;
	}

	/* write control regs if values provided via device tree */
	for (ch = FIRST_CHANNEL; ch < NUM_CHANNELS; ch++) {
		if (adev->ctrl_regs[ch]) {
			ret = __hi3593_write_ctrl_reg(adev,
				ch, adev->ctrl_regs[ch]);
			if (ret)
				return ret;
		}
	}

	return ret;
}

static inline void __hi3593_update_channel_rate(
		struct hi3593_channel_priv *chan)
{
	if (chan->adev->ctrl_regs[chan->type] & rate_bits[chan->type])
		chan->rate = RATE_LOW;
	else
		chan->rate = RATE_HIGH;
}

static int hi3593_open(struct net_device *ndev)
{
	struct hi3593_channel_priv *chan = netdev_priv(ndev);
	struct hi3593_priv *adev = chan->adev;
	struct device *dev = &adev->spi->dev;
	int ret;

	netdev_dbg(ndev, "%s: enter\n", __func__);

	/* common open */
	ret = open_arinc429dev(ndev);
	if (ret)
		goto err;

	mutex_lock(&adev->dev_lock);

	/* perform intial setup on first open */
	if (!adev->active_channels) {
		ret = __hi3593_hw_default_config(adev);
		if (ret)
			goto err_unlock;
	}

	switch (chan->type) {
	case RECEIVER_1:
		if (!gpio_is_valid(adev->r1_int) || 
		    !gpio_is_valid(adev->r1_flag)) {
			netdev_err(ndev, "GPIO lines for R1 IRQs are invalid\n");
			ret = -EINVAL;
			goto err_unlock;
		}

		ret = devm_request_threaded_irq(dev,
			gpio_to_irq(adev->r1_int),
			NULL,
			hi3593_rx_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			DRV_NAME "-rx1",
			chan);
		if (ret) {
			netdev_err(ndev, "failed to acquire r1-int\n");
			goto err_unlock;
		}

		ret = devm_request_threaded_irq(dev,
			gpio_to_irq(adev->r1_flag),
			NULL,
			hi3593_rx_flag_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
			DRV_NAME "-rx1-flag",
			chan);
		if (ret) {
			netdev_err(ndev, "failed to acquire r1-flag\n");
			goto err_unlock;
		}

		init_timer(&chan->irq_enable_timer);
		chan->irq_enable_timer.data = (unsigned long)chan;
		chan->irq_enable_timer.function = hi3593_enable_timer;

		INIT_WORK(&chan->work, hi3593_rx_work_handler);

		break;

	case RECEIVER_2:
		if (!gpio_is_valid(adev->r2_int) || 
		    !gpio_is_valid(adev->r2_flag)) {
			netdev_err(ndev, "GPIO lines for R2 IRQs are invalid\n");
			ret = -EINVAL;
			goto err_unlock;
		}

		ret = devm_request_threaded_irq(dev,
			gpio_to_irq(adev->r2_int),
			NULL,
			hi3593_rx_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			DRV_NAME "-rx2",
			chan);
		if (ret) {
			netdev_err(ndev, "failed to acquire r2-int\n");
			goto err_unlock;
		}
		ret = devm_request_threaded_irq(dev,
			gpio_to_irq(adev->r2_flag),
			NULL,
			hi3593_rx_flag_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
			DRV_NAME "-rx2-flag",
			chan);
		if (ret) {
			netdev_err(ndev, "failed to acquire r2-flag\n");
			goto err_unlock;
		}

		init_timer(&chan->irq_enable_timer);
		chan->irq_enable_timer.data = (unsigned long)chan;
		chan->irq_enable_timer.function = hi3593_enable_timer;

		INIT_WORK(&chan->work, hi3593_rx_work_handler);

		break;

	case TRANSMITTER:
		if (!gpio_is_valid(adev->tx_full) || 
		    !gpio_is_valid(adev->tx_empty)) {
			netdev_err(ndev, "GPIO lines for TX IRQs are invalid\n");
			ret = -EINVAL;
			goto err_unlock;
		}

		ret = devm_request_threaded_irq(dev,
			gpio_to_irq(adev->tx_full),
			NULL,
			hi3593_tx_full_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_RISING,
			DRV_NAME "-tx-full",
			chan);
		if (ret) {
			netdev_err(ndev, "failed to acquire tx-full\n");
			goto err_unlock;
		}

		ret = devm_request_threaded_irq(dev,
			gpio_to_irq(adev->tx_empty),
			NULL,
			hi3593_tx_empty_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_RISING,
			DRV_NAME "-tx-empty",
			chan);
		if (ret) {
			netdev_err(ndev, "failed to acquire tx-empty\n");
			goto err_unlock;
		}

		/* start with TX empty interrupt disabled.
		 * In case TX FIFO becomes full this irq will be switched on
		 * and flip off TX full. */
		disable_irq(gpio_to_irq(adev->tx_empty));

		INIT_WORK(&chan->work, hi3593_tx_work_handler);
		break;

	default:
		BUG();
		break;
	}

	__hi3593_update_channel_rate(chan);

	adev->active_channels++;

	mutex_unlock(&adev->dev_lock);

	netif_start_queue(ndev);

	return 0;

err_unlock:
	mutex_unlock(&chan->adev->dev_lock);

err:
	return ret;
}

static int hi3593_close(struct net_device *ndev)
{
	struct hi3593_channel_priv *chan = netdev_priv(ndev);
	struct hi3593_priv *adev = chan->adev;
	struct device *dev = &adev->spi->dev;

	netdev_dbg(ndev, "%s: enter\n", __func__);

	netif_stop_queue(ndev);

	close_arinc429dev(ndev);

	mutex_lock(&adev->dev_lock);

	switch (chan->type) {
	case RECEIVER_1:
		flush_work(&chan->work);

		devm_free_irq(dev, gpio_to_irq(adev->r1_int), chan);
		devm_free_irq(dev, gpio_to_irq(adev->r1_flag), chan);

		del_timer_sync(&chan->irq_enable_timer);

		break;

	case RECEIVER_2:
		flush_work(&chan->work);

		devm_free_irq(dev, gpio_to_irq(adev->r2_int), chan);
		devm_free_irq(dev, gpio_to_irq(adev->r2_flag), chan);

		del_timer_sync(&chan->irq_enable_timer);

		break;

	case TRANSMITTER:
		flush_work(&chan->work);

		/* ensure any queued tx buffers are dumped */
		while (!skb_queue_empty(&adev->txq)) {
			struct sk_buff *txb = skb_dequeue(&adev->txq);

			netdev_dbg(ndev, "%s: freeing txb %p\n", __func__, txb);

			dev_kfree_skb(txb);
		}

		devm_free_irq(dev, gpio_to_irq(adev->tx_full), chan);
		devm_free_irq(dev, gpio_to_irq(adev->tx_empty), chan);

		break;

	default:
		BUG();
		break;
	}

	adev->active_channels--;

	mutex_unlock(&adev->dev_lock);

	return 0;
}

static int hi3593_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	int ret = NETDEV_TX_OK;

	netdev_vdbg(dev, "%s: enter\n", __func__);

	/* discard transmit frame if this is RX channel */
	if (chan->type != TRANSMITTER) {
		netdev_dbg(dev,
			"%s: attempted to transmit on RX channel\n", __func__);

		dev->stats.tx_dropped++;

		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (arinc429_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	skb_queue_tail(&adev->txq, skb);

	schedule_work(&chan->work);

	return ret;
}

static const struct net_device_ops hi3593_netdev_ops = {
	.ndo_open               = hi3593_open,
	.ndo_stop               = hi3593_close,
	.ndo_start_xmit         = hi3593_start_xmit,
};

/* sysfs properties */

static ssize_t
hi3593_flip_label_bits_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;

	return sprintf(buf, "%s\n",
		(adev->ctrl_regs[chan->type] & label_flip_bits[chan->type]) ?
		"on" : "off");
}

static ssize_t
hi3593_flip_label_bits_set(struct device *d,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	int ret = -EINVAL;

	if (!strncmp(buf, "on", 2)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_set_ctrl_reg_bits(adev,
				chan->type, label_flip_bits[chan->type]);
		mutex_unlock(&adev->dev_lock);
	} else if (!strncmp(buf, "off", 3)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_clear_ctrl_reg_bits(adev,
				chan->type, label_flip_bits[chan->type]);
		mutex_unlock(&adev->dev_lock);
	}

	return ret < 0 ? ret : count;
}

static DEVICE_ATTR(flip_label_bits, S_IRUGO | S_IWUSR,
	hi3593_flip_label_bits_get, hi3593_flip_label_bits_set);


static ssize_t
hi3593_rate_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);

	return sprintf(buf, "%s\n", (chan->rate == RATE_HIGH) ? "high" : "low");
}

static ssize_t
hi3593_rate_set(struct device *d,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	int ret = -EINVAL;

	if (!strncmp(buf, "low", 3)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_set_ctrl_reg_bits(adev,
				chan->type, rate_bits[chan->type]);
		if (!ret)
			chan->rate = RATE_LOW;
		mutex_unlock(&adev->dev_lock);
	} else if (!strncmp(buf, "high", 4)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_clear_ctrl_reg_bits(adev,
				chan->type, rate_bits[chan->type]);
		if (!ret)
			chan->rate = RATE_HIGH;
		mutex_unlock(&adev->dev_lock);
	}
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(rate, S_IRUGO | S_IWUSR,
	hi3593_rate_get, hi3593_rate_set);

static ssize_t
hi3593_parity_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;

	return sprintf(buf, "%s\n",
		(adev->ctrl_regs[chan->type] & parity_bits[chan->type]) ?
		"on" : "off");
}

static ssize_t
hi3593_parity_set(struct device *d,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	int ret = -EINVAL;

	if (!strncmp(buf, "on", 2)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_set_ctrl_reg_bits(adev,
				chan->type, parity_bits[chan->type]);
		mutex_unlock(&adev->dev_lock);
	} else if (!strncmp(buf, "off", 3)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_clear_ctrl_reg_bits(adev,
				chan->type, parity_bits[chan->type]);
		mutex_unlock(&adev->dev_lock);
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(parity, S_IRUGO | S_IWUSR,
	hi3593_parity_get, hi3593_parity_set);

static ssize_t
hi3593_label_filter_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;

	if (hi3593_is_transmitter(chan->type))
		return sprintf(buf, "N/A\n");

	return sprintf(buf, "%s\n",
		(adev->ctrl_regs[chan->type] & RECEIVER_LABEL_FILTERS_ENABLED) ?
			"on" : "off");
}

static ssize_t
hi3593_label_filter_set(struct device *d,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	int ret = -EINVAL;

	if (hi3593_is_transmitter(chan->type))
		return ret;

	if (!strncmp(buf, "on", 2)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_set_ctrl_reg_bits(adev,
			chan->type, RECEIVER_LABEL_FILTERS_ENABLED);
		mutex_unlock(&adev->dev_lock);
	} else if (!strncmp(buf, "off", 3)) {
		mutex_lock(&adev->dev_lock);
		ret = __hi3593_clear_ctrl_reg_bits(adev,
			chan->type, RECEIVER_LABEL_FILTERS_ENABLED);
		mutex_unlock(&adev->dev_lock);
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(label_filter, S_IRUGO | S_IWUSR,
	hi3593_label_filter_get, hi3593_label_filter_set);

static ssize_t
hi3593_label_filter_bitmap_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	const unsigned long *p[NUM_RECEIVERS] = {
		adev->r1_label_filter, adev->r2_label_filter};

	if (hi3593_is_transmitter(chan->type))
		return sprintf(buf, "N/A\n");

	return bitmap_print_to_pagebuf(true, buf, p[chan->type], NUM_LABELS);
}

static ssize_t
hi3593_label_filter_bitmap_set(struct device *d,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	unsigned long *p[NUM_RECEIVERS] = {
		adev->r1_label_filter, adev->r2_label_filter};
	DECLARE_BITMAP(bits, NUM_LABELS);
	int ret = -EINVAL;

	if (hi3593_is_transmitter(chan->type))
		return ret;

	ret = bitmap_parselist(buf, bits, NUM_LABELS);
	if (ret)
		return ret;

	mutex_lock(&adev->dev_lock);

	ret = __hi3593_configure_label_filters(adev, chan->type, bits);
	if (!ret)
		/* update cached copy */
		bitmap_copy(p[chan->type], bits, NUM_LABELS);

	mutex_unlock(&adev->dev_lock);

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(label_filter_bitmap, S_IRUGO | S_IWUSR,
	hi3593_label_filter_bitmap_get, hi3593_label_filter_bitmap_set);

static ssize_t
hi3593_tx_odd_even_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;

	if (hi3593_is_receiver(chan->type))
		return sprintf(buf, "N/A\n");

	return sprintf(buf, "%s\n",
			(adev->ctrl_regs[TRANSMITTER] & TRANSMITTER_ODD_EVEN) ?
				"even" : "odd");
}

static ssize_t
hi3593_tx_odd_even_set(struct device *d,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	int ret = -EINVAL;

	if (chan->type != TRANSMITTER)
		return ret;

	if (!strncmp(buf, "even", 4)) {
		mutex_lock(&adev->dev_lock);

		ret = __hi3593_set_ctrl_reg_bits(adev,
				TRANSMITTER, TRANSMITTER_ODD_EVEN);

		mutex_unlock(&adev->dev_lock);
	} else if (!strncmp(buf, "odd", 3)) {
		mutex_lock(&adev->dev_lock);

		ret = __hi3593_clear_ctrl_reg_bits(adev,
				TRANSMITTER, TRANSMITTER_ODD_EVEN);

		mutex_unlock(&adev->dev_lock);
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(tx_odd_even, S_IRUGO | S_IWUSR,
	hi3593_tx_odd_even_get, hi3593_tx_odd_even_set);

static ssize_t
hi3593_dump_regs_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	struct hi3593_priv *adev = chan->adev;
	struct {
		char *name;
		u8 reg;
		u8 val;
	} reg_data[] = {
		{"R1 CTRL", CMD_READ_R1_CTRL, 0},
		{"R2 CTRL", CMD_READ_R2_CTRL, 0},
		{"FLAG", CMD_READ_FLAG, 0},
		{"TX CTRL", CMD_READ_TCR, 0},
		{"ACLK DIV", CMD_READ_ACLK_DIV, 0},
	};
	DECLARE_BITMAP(bits, NUM_LABELS);
	DECLARE_BITMAP(reversed, NUM_LABELS);
	u8 cmd = CMD_READ_R1_LABELS;
	ssize_t count = 0;
	int i;
	int ret;

	for (i = 0; i < 5; i++) {
		ret = spi_write_then_read(adev->spi,
			&reg_data[i].reg, 1,
			&reg_data[i].val, 1);

		if (ret)
			continue;

		ret = sprintf(buf, "%s : %#x\n",
				reg_data[i].name, reg_data[i].val);
		buf += ret;
		count += ret;
	}

	if (!spi_write_then_read(adev->spi, &cmd, 1, (u8*)bits, 32)) {
		ret = sprintf(buf, "R1 LABELS:\n");
		buf += ret;
		count += ret;
		hi3593_reverse_bytes((u8*)reversed,
			(const u8*)bits, NUM_LABELS / BITS_PER_BYTE);
		ret = bitmap_print_to_pagebuf(false, buf, reversed, NUM_LABELS);
		buf += ret;
		count += ret;
		ret = sprintf(buf, "\n");
		buf += ret;
		count += ret;
	}

	cmd = CMD_READ_R2_LABELS;
	if (!spi_write_then_read(adev->spi, &cmd, 1, (u8*)bits, 32)) {
		ret = sprintf(buf, "R2 LABELS:\n");
		buf += ret;
		count += ret;
		hi3593_reverse_bytes((u8*)reversed,
			(const u8*)bits, NUM_LABELS / BITS_PER_BYTE);
		ret = bitmap_print_to_pagebuf(false, buf, reversed, NUM_LABELS);
		buf += ret;
		count += ret;
		ret = sprintf(buf, "\n");
		buf += ret;
		count += ret;
	}

	return count;
}

static DEVICE_ATTR(dump_regs, S_IRUSR,
	hi3593_dump_regs_get, NULL);

static ssize_t
hi3593_channel_type_get(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct net_device *dev = to_net_dev(d);
	struct hi3593_channel_priv *chan = netdev_priv(dev);
	return sprintf(buf, "%s\n", channel_type_names[chan->type]);
}

static DEVICE_ATTR(channel_type, S_IRUSR,
	hi3593_channel_type_get, NULL);

static struct attribute *hi3593_sysfs_entries[] = {
	&dev_attr_flip_label_bits.attr,
	&dev_attr_rate.attr,
	&dev_attr_parity.attr,
	&dev_attr_label_filter.attr,
	&dev_attr_label_filter_bitmap.attr,
	&dev_attr_tx_odd_even.attr,
	&dev_attr_dump_regs.attr,
	&dev_attr_channel_type.attr,
	NULL
};

static struct attribute_group hi3593_attribute_group = {
	.name = "arinc429",
	.attrs = hi3593_sysfs_entries,
};

static struct hi3593_channel_priv*
hi3593_alloc_channel(struct hi3593_priv *adev, enum channel_type type)
{
	struct net_device *dev;
	struct hi3593_channel_priv *chan;
	int ret;

	dev_dbg(&adev->spi->dev, "%s: enter\n", __func__);

	dev = alloc_arinc429dev(sizeof(*chan), 1);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	chan = netdev_priv(dev);

	chan->adev = adev;
	chan->ndev = dev;
	chan->type = type;

	/* FIXME: clock freq is not used in ARINC protocol driver
	in CAN framework it was used to calculate bit timing. In ARINC theres is
	only RATE (low-speed and high-speed) */
	chan->arinc429.clock.freq = chan->adev->freq;

	/* TODO: add more callbacks into ARINC framework to manage:
	 - rate
	 - label filters
	 - priority matching
	 - parity
	 - label bits flip

	 As an alternative to callbacks - use IOCTLs on dev/sock (?)
	 */

	/* init sysfs attributes */
	dev->sysfs_groups[0] = &hi3593_attribute_group;

	SET_NETDEV_DEV(dev, &adev->spi->dev);
	dev->netdev_ops = &hi3593_netdev_ops;

	ret = register_arinc429dev(dev);
	if (ret) {
		free_arinc429dev(dev);
		dev_err(&adev->spi->dev, "registering failed (ret=%d)\n", ret);
		return ERR_PTR(ret);
	}

	dev_info(&adev->spi->dev, "device registered\n");

	return chan;
}

static void hi3593_free_channel(struct hi3593_channel_priv *chan)
{
	if (!IS_ERR_OR_NULL(chan)) {
		unregister_arinc429dev(chan->ndev);
		free_arinc429dev(chan->ndev);
	}
}

/* Parses Device Tree data for the chip.
 * Supported properties:
 *   arinc-frequency - ARINC429 clock frequency, (1,2,4,6..30 MHz)
 *   r1_ctrl - initial value for Receiver 1 control register
 *   r2_ctrl - initial value for Receiver 2 control register
 *   tx_ctrl - initial value for Transmitter control register
 *   flag_iar - initial value for FLAG / Interrupt Assignment register
 *   r1_lf - label filters bitmap list for Receiver 1
 *   r2_lf - label filters bitmap list for Receiver 2
 *   master_reset-gpio - external HW reset pin
 *   r1_int-gpio - Receiver 1 interrupt (R1INT)
 *   r1_flag-gpio - Receiver 1 FLAG interrupt (R1FLAG)
 *   r2_int-gpio - Receiver 2 interrupt (R2INT)
 *   r2_flag-gpio - Receiver 2 FLAG interrupt (R2FLAG)
 *   tx_full-gpio - Transmitter FIFO full interrupt (TFULL)
 *   tx_empty-gpio - Transmitter FIFO empty (TFEMPTY)
 *
 * Sample configuration:
 * arinc-frequency = <1000000>;
 * r1_ctrl = /bits/ 8 <0x88>;
 * r2_ctrl = /bits/ 8 <0x88>;
 * tx_ctrl = /bits/ 8 <0x60>;
 * flag_iar = /bits/ 8 <0x22>;
 * r1_lf = "241,255";
 * master_reset-gpio = <&gpio3 12 GPIO_ACTIVE_HIGH>;
 * r1_int-gpio = <&gpio3 3 GPIO_ACTIVE_HIGH>;
 * r1_flag-gpio = <&gpio3 7 GPIO_ACTIVE_HIGH>;
 * r2_int-gpio = <&gpio3 4 GPIO_ACTIVE_HIGH>;
 * r2_flag-gpio = <&gpio3 8 GPIO_ACTIVE_HIGH>;
 * tx_full-gpio = <&gpio4 1 GPIO_ACTIVE_HIGH>;
 * tx_empty-gpio = <&gpio4 0 GPIO_ACTIVE_HIGH>;
 */
static int hi3593_probe_dt(struct hi3593_priv *adev)
{
	struct device *dev = &adev->spi->dev;
	struct device_node *node = dev->of_node;
	const char* s;
	u8 val;
	int ret;

	if (!node) {
		dev_err(dev, "Device does not have associated DT data\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "arinc-frequency", &adev->freq);
	if (ret < 0) {
		if (ret == -EINVAL) {
			adev->freq = DEFAULT_ARINC429_CLK_FREQ;
		} else {
			dev_err(dev,
				"Failed to get arinc-frequency node, err: %d\n",
				ret);
			return ret;
		}
	}
	dev_dbg(dev, "selected ARINC429 clock frequency: %u\n", adev->freq);

	/* read pre-defined control register values */
	ret = of_property_read_u8(node, "r1_ctrl", &val);
	if (!ret) {
		adev->ctrl_regs[RECEIVER_1] = val;
		dev_dbg(dev, "set Receiver1 Control Reg to %#x\n", val);
	}
	ret = of_property_read_u8(node, "r2_ctrl", &val);
	if (!ret) {
		adev->ctrl_regs[RECEIVER_2] = val;
		dev_dbg(dev, "set Receiver2 Control Reg to %#x\n", val);

	}
	ret = of_property_read_u8(node, "tx_ctrl", &val);
	if (!ret) {
		adev->ctrl_regs[TRANSMITTER] = val;
		dev_dbg(dev, "set Transmitter Control Reg to %#x\n", val);
	}
	ret = of_property_read_u8(node, "flag_iar", &val);
	if (!ret) {
		adev->flag_iar = val;
		dev_dbg(dev, "set FLAG/IAR to %#x\n", val);
	}

	ret = of_property_read_string(node, "r1_lf", &s);
	if (!ret) {
		ret = bitmap_parselist(s, adev->r1_label_filter, NUM_LABELS);
		if (ret) {
			dev_err(dev,
				"error parsing label filters from list: %s\n", s);
			bitmap_zero(adev->r1_label_filter, NUM_LABELS);
		} else
			dev_dbg(dev, "label filter: %*pbl\n",
				NUM_LABELS, adev->r1_label_filter);
	}
	ret = of_property_read_string(node, "r2_lf", &s);
	if (!ret) {
		ret = bitmap_parselist(s, adev->r2_label_filter, NUM_LABELS);
		if (ret) {
			dev_err(dev,
				"error parsing label filters from list: %s\n", s);
			bitmap_zero(adev->r2_label_filter, NUM_LABELS);
		} else
			dev_dbg(dev, "label filter: %*pbl\n",
				NUM_LABELS, adev->r2_label_filter);
	}

	adev->hw_reset = of_get_named_gpio(node, "master_reset-gpio", 0);
	if (gpio_is_valid(adev->hw_reset))
		if (devm_gpio_request_one(dev, adev->hw_reset,
				GPIOF_OUT_INIT_LOW, "HI-3593 Reset"))
			adev->hw_reset = -EINVAL;

	adev->r1_int = of_get_named_gpio(node, "r1_int-gpio", 0);
	if (gpio_is_valid(adev->r1_int))
		if (devm_gpio_request_one(dev, adev->r1_int,
				GPIOF_IN, "HI-3593 Receiver 1 interrupt pin"))
			adev->r1_int = -EINVAL;

	adev->r1_flag = of_get_named_gpio(node, "r1_flag-gpio", 0);
	if (gpio_is_valid(adev->r1_flag))
		if (devm_gpio_request_one(dev, adev->r1_flag,
				GPIOF_IN, "HI-3593 Receiver 1 flag pin"))
			adev->r1_flag = -EINVAL;

	adev->r2_int = of_get_named_gpio(node, "r2_int-gpio", 0);
	if (gpio_is_valid(adev->r2_int))
		if (devm_gpio_request_one(dev, adev->r2_int,
				GPIOF_IN, "HI-3593 Receiver 2 interrupt pin"))
			adev->r2_int = -EINVAL;

	adev->r2_flag = of_get_named_gpio(node, "r2_flag-gpio", 0);
	if (gpio_is_valid(adev->r2_flag))
		if (devm_gpio_request_one(dev, adev->r2_flag,
				GPIOF_IN, "HI-3593 Receiver 2 flag pin"))
			adev->r2_flag = -EINVAL;

	adev->tx_full = of_get_named_gpio(node, "tx_full-gpio", 0);
	if (gpio_is_valid(adev->tx_full))
		if (devm_gpio_request_one(dev, adev->tx_full,
				GPIOF_IN, "HI-3593 Transmit FIFO full"))
			adev->tx_full = -EINVAL;

	adev->tx_empty = of_get_named_gpio(node, "tx_empty-gpio", 0);
	if (gpio_is_valid(adev->tx_empty))
		if (devm_gpio_request_one(dev, adev->tx_empty,
				GPIOF_IN, "HI-3593 Transmit FIFO empty"))
			adev->tx_empty = -EINVAL;

	return 0;
}

/* Executes built-in self test procedure */
static int hi3593_self_test(struct hi3593_priv *adev)
{
	u8 tx_buf[5] = { CMD_WRITE_MSG, 0x12, 0x34, 0x56, 0x78 };
	u8 rx_buf[4] = {0};
	u8 r2_reply[4] = {0xed, 0xcb, 0xa9, 0x87};
	u8 old_regs[NUM_CHANNELS];
	u8 plm_cmd[4] = {CMD_WRITE_R1_PLMR, 0, 0, 0x78};
	u8 cmd;
	int ret;

	/* backup current reg values */
	memcpy(old_regs, adev->ctrl_regs, NUM_CHANNELS);

	dev_dbg(&adev->spi->dev, "%s: enter\n", __func__);

	/* enable internal loopback */
	ret = __hi3593_write_ctrl_reg(adev, TRANSMITTER,
			TRANSMITTER_MODE | TRANSMITTER_SELF_TEST);
	if (ret)
		return ret;

	dev_dbg(&adev->spi->dev, "%s: self test enabled\n", __func__);

	/* if ARINC429 bus has active transmitter connected to our RX channels
	 * FIFO will be filled by this data before we got our self test packet.
	 * set priority matching and our test labels for R1 and R2, perform
	 * soft reset to clear FIFO and execute self test. */
	__hi3593_write_ctrl_reg(adev, RECEIVER_1, RECEIVER_PRIORITY_MATCH_ON);
	__hi3593_write_ctrl_reg(adev, RECEIVER_2, RECEIVER_PRIORITY_MATCH_ON);

	/* set Receiver 1 match labels */
	spi_write(adev->spi, plm_cmd, sizeof(plm_cmd));

	/* set Receiver 2 match labels */
	plm_cmd[0] = CMD_WRITE_R2_PLMR;
	plm_cmd[3] = 0x87;
	spi_write(adev->spi, plm_cmd, sizeof(plm_cmd));

	cmd = CMD_SOFT_RESET;
	spi_write(adev->spi, &cmd, 1);

	/* If Transmit Control Register bit SELFTEST is equal '1', the
	 * transmitter serial output data is internally looped-back into the
	 * receiver 1. The data will appear inverted (compliment) on receiver 2.
	 * Data passes unmodified from transmitter to receiver 1. Setting
	 * Transmit Control register bit SELFTEST to '1' forces TXAOUT and
	 * TXBOUT to the Null state to prevent self-test data from appearing on
	 * theARINC 429 bus. */

	ret = spi_write(adev->spi, tx_buf, sizeof(tx_buf));
	if (ret)
		return ret;

	dev_dbg(&adev->spi->dev, "%s: TX data: %#x %02x %02x %02x\n",
		__func__, tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	/* let HW settle. Experimentally found that bits appear on RX in ~220usec */
	udelay(300);

	/* read back first receiver and compare with source data */
	cmd = CMD_READ_R1_MSG;
	ret = spi_write_then_read(adev->spi, &cmd, 1, rx_buf, 4);
	if (ret)
		return ret;

	dev_dbg(&adev->spi->dev, "%s: R1 data: %#x %02x %02x %02x\n",
		__func__, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
	if (memcmp(&tx_buf[1], rx_buf, 4))
		return -EINVAL;

	cmd = CMD_READ_R2_MSG;
	ret = spi_write_then_read(adev->spi, &cmd, 1, rx_buf, 4);
	if (ret)
		return ret;
	dev_dbg(&adev->spi->dev, "%s: R2 data: %#x %02x %02x %02x\n",
		__func__, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	if (memcmp(r2_reply, rx_buf, 4))
		return -EINVAL;

	/* restore control regs values */
	memcpy(adev->ctrl_regs, old_regs, NUM_CHANNELS);

	dev_dbg(&adev->spi->dev, "%s: self test completed\n", __func__);
	return 0;
}


static int hi3593_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct hi3593_priv *adev;
	int ret;
	enum channel_type ch;

	dev_dbg(dev, "%s: enter\n", __func__);

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	mutex_init(&adev->dev_lock);

	adev->spi = spi;

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	dev_set_drvdata(dev, adev);

	ret = hi3593_probe_dt(adev);
	if (ret)
		return ret;

	if (adev->freq != DEFAULT_ARINC429_CLK_FREQ)
		if (hi3593_set_clock(adev->spi, adev->freq)) {
			dev_err(dev, "Unable to set freq: %d\n", adev->freq);
			return -EINVAL;
		}

	/* There is no ID regs on this chip and we can't use status registers
	 *  as well. If data flows on ARINC429, their state is changed from default.
	 * Let's do chip verification by executng self-test.
	 * Do not bother to clean-up FIFOs after test,
	 * on first network device open call HW will be reset */
	if (hi3593_self_test(adev))
		return -ENODEV;

	for (ch = FIRST_CHANNEL; ch < NUM_CHANNELS; ch++) {
		adev->channels[ch] = hi3593_alloc_channel(adev, ch);
		if (IS_ERR(adev->channels[ch])) {
			ret = PTR_ERR(adev->channels[ch]);
			goto cleanup;
		}
	}

	skb_queue_head_init(&adev->txq);

	return 0;

cleanup:
	for (ch = FIRST_CHANNEL; ch < NUM_CHANNELS; ch++)
		hi3593_free_channel(adev->channels[ch]);

	return ret;
}

static int hi3593_spi_remove(struct spi_device *spi)
{
	struct hi3593_priv *adev = dev_get_drvdata(&spi->dev);
	enum channel_type ch;

	dev_dbg(&spi->dev, "%s: enter\n", __func__);

	for (ch = FIRST_CHANNEL; ch < NUM_CHANNELS; ch++)
		hi3593_free_channel(adev->channels[ch]);

	return 0;
}

static const struct spi_device_id hi3593_id[] = {
	{ DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, hi3593_id);

static struct spi_driver hi3593_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = hi3593_spi_probe,
	.remove = hi3593_spi_remove,
	.id_table = hi3593_id,
};
module_spi_driver(hi3593_driver);

MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("HoltIC HI-3593 ARINC429 driver");
MODULE_LICENSE("GPL v2");

