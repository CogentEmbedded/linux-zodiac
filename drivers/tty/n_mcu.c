/*
 *  n_mcu.c - microcontroller unit communication line discipline
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
/*
 * This driver implement single instance line discipline with simple
 * UART protocol using following entities:
 *  - message to MCU => ACK response
 *  - event from MCU => event ACK
 *
 * Frame structure:
 * <STX> <DATA> <CHECKSUM> <ETX>
 * Where:
 * - STX - is start of transmission character
 * - ETX - end of transmission
 * - DATA - payload
 * - CHECKSUM - checksum calculated on <DATA>
 *
 * If <DATA> or <CHECKSUM> contain one of control characters, then it is
 * escaped using <DLE> control code. Added <DLE> does not participate in
 * checksum calculation.
 * 
 * Client code should configure line discipline to use checksum (if needed) and
 * get/set function pointers to send data, and receive unsolicited notifications.
 * 
 */
/* #define DEBUG */

#include <asm/unaligned.h>
#include <linux/crc-ccitt.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>

#include <linux/n_mcu.h>


#define DRIVER_NAME		"n_mcu"

#define STX			0x02
#define ETX			0x03
#define DLE			0x10

#define STX_SIZE		1
#define ETX_SIZE		1
#define MAX_SERIAL_BUF_SIZE	(STX_SIZE + 2 * N_MCU_MAX_CMD_SIZE + ETX_SIZE)

#define MAX_RETRIES		3
#define CMD_TIMEOUT		100

enum parser_state {
	EXPECT_SOF,
	EXPECT_DATA,
	EXPECT_ESCAPED_DATA
};

struct n_mcu_priv {
	struct tty_struct	*tty;
	wait_queue_head_t	wait;
	bool			opened;

	atomic_t		transfer_in_progress;
	bool			transfer_failed;

	u8			tx_frame[MAX_SERIAL_BUF_SIZE];
	size_t			tx_frame_size;

	u8			rx_data[MAX_SERIAL_BUF_SIZE];
	size_t			rx_data_size;
	atomic_t		rx_ready;

	enum parser_state	parser_state;

	struct completion	cmd_complete;

	void 			*callback_cookie;
	event_callback_t	event;

	void	(*add_checksum)(const u8 const *data, size_t len,
				u8 *out, size_t *out_idx);
	bool	(*validate_checksum)(const u8 const *data, size_t len);

	void	(*response)(struct n_mcu_cmd *);
};

static struct n_mcu_priv n_mcu_priv;
static DEFINE_MUTEX(n_mcu_lock);
static DEFINE_SPINLOCK(n_mcu_tx_lock);

static void n_mcu_put_data(const u8 const *data,
		size_t len, u8 *out, size_t *out_idx);
static void n_mcu_handle_event(void);
static int n_mcu_event_response(struct n_mcu_cmd *cmd);


static void n_mcu_add_checksum_ccitt_false(const u8 const *data,
		size_t len, u8 *out, size_t *out_idx)
{
	u16 crc = 0xffff;
	u8 tmp[2];

	pr_debug("%s: enter\n", __func__);

	crc = crc_ccitt_false(crc, data, len);

	put_unaligned_be16(crc, tmp);

	n_mcu_put_data(tmp, sizeof(tmp), out, out_idx);
}

static bool n_mcu_validate_checksum_ccitt_false(const u8 const *data,
		size_t len)
{
	u16 crc = 0xffff;

	pr_debug("%s: enter\n", __func__);

	/* Expect at least one data byte with CRC */
	if (unlikely(len <= 2))
		return false;

	crc = crc_ccitt_false(crc, data, len - 2);

	return crc == get_unaligned_be16(&data[len - 2]);
}

static inline u8 n_mcu_checksum_8b2c(const u8 const *data,
		size_t len)
{
	int i;
	u8 checksum = 0;

	pr_debug("%s: enter\n", __func__);

	/* calculate 8-bit 2's complement from buffer */
	for (i = 0; i < len; i++)
		checksum += data[i];

	return 1 + ~checksum;
}

static void n_mcu_add_checksum_8b2c(const u8 const *data,
		size_t len, u8 *out, size_t *out_idx)
{
	u8 checksum = 0;

	pr_debug("%s: enter\n", __func__);

	checksum = n_mcu_checksum_8b2c(data, len);

	n_mcu_put_data(&checksum, sizeof(checksum), out, out_idx);
}

static bool n_mcu_validate_checksum_8b2c(const u8 const *data,
		size_t len)
{
	pr_debug("%s: enter\n", __func__);

	/* Expect at least one data byte with checksum */
	if (unlikely(len <= 1))
		return false;

	return data[len - 1] == n_mcu_checksum_8b2c(data, len - 1);
}

static int n_mcu_set_checksum(unsigned int type)
{
	int ret = 0;

	pr_debug("%s: enter, going to set checksum type: %d\n", __func__, type);

	mutex_lock(&n_mcu_lock);
	switch (type) {
	case N_MCU_CHECKSUM_NONE:
		n_mcu_priv.add_checksum = NULL;
		n_mcu_priv.validate_checksum = NULL;
		break;
	case N_MCU_CHECKSUM_CCITT_FALSE:
		n_mcu_priv.add_checksum = n_mcu_add_checksum_ccitt_false;
		n_mcu_priv.validate_checksum = n_mcu_validate_checksum_ccitt_false;
		break;
	case N_MCU_CHECKSUM_8B2C:
		n_mcu_priv.add_checksum = n_mcu_add_checksum_8b2c;
		n_mcu_priv.validate_checksum = n_mcu_validate_checksum_8b2c;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&n_mcu_lock);

	return ret;
}

static void n_mcu_put_data(const u8 const *in,
		size_t in_size, u8 *out, size_t *out_idx)
{
	int i;
	for (i = 0; i < in_size; i++) {
		switch (in[i]) {
		case STX:
		case ETX:
		case DLE:
			out[(*out_idx)++] = DLE;
			break;
		}
		out[(*out_idx)++] = in[i];
	}
}

static int n_mcu_prepare_frame(struct n_mcu_priv *priv,
		const u8 *data, size_t size)
{
	size_t i = 0;

	pr_debug("%s: enter\n", __func__);

	if (size > N_MCU_MAX_CMD_SIZE)
		return -EINVAL;

	priv->tx_frame[i++] = STX;

	n_mcu_put_data(data, size, priv->tx_frame, &i);

	if (priv->add_checksum)
		priv->add_checksum(data, size, priv->tx_frame, &i);

	priv->tx_frame[i++] = ETX;

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "frame data: ", DUMP_PREFIX_OFFSET,
			16, 1, priv->tx_frame, i, true);
#endif

	priv->tx_frame_size = i;
	return 0;
}

static void n_mcu_handle_msg(struct n_mcu_priv *priv)
{
	pr_debug("%s: enter\n", __func__);

	if (priv->validate_checksum &&
	    !priv->validate_checksum(priv->rx_data, priv->rx_data_size)) {
		n_mcu_priv.transfer_failed = true;
		goto out;
	}

	pr_debug("%s: checksum ok\n", __func__);

	/* this is cmd response */
	if (atomic_read(&n_mcu_priv.transfer_in_progress)) {
		if (!completion_done(&priv->cmd_complete))
			complete(&priv->cmd_complete);
		goto out;
	}

	/* otherwise it is event from MCU */
	n_mcu_handle_event();
out:
	atomic_set(&n_mcu_priv.transfer_in_progress, 0);
	atomic_set(&n_mcu_priv.rx_ready, 1);
}

static void n_mcu_parse_data(struct n_mcu_priv *priv,
		const u8 const *data, int count)
{
	int i;
	for (i = 0; i < count; i++) {
		switch (priv->parser_state) {
		case EXPECT_SOF:
			if (data[i] != STX)
				break;

			if (wait_event_interruptible(priv->wait,
				atomic_read(&priv->rx_ready)))
				return;

			priv->rx_data_size = 0;
			priv->parser_state = EXPECT_DATA;
			break;

		case EXPECT_DATA:
			if (data[i] == DLE) {
				priv->parser_state = EXPECT_ESCAPED_DATA;
				break;
			}

			if (unlikely(data[i] == STX)) {
				pr_warn("unexpected STX, incomplete frame discarded\n");
				priv->rx_data_size = 0;
				break;
			}

			if (data[i] != ETX) {
				priv->rx_data[priv->rx_data_size++] = data[i];
				break;
			}

			/* ETX, disable further receiving until handled */

			atomic_set(&priv->rx_ready, 0);

			priv->parser_state = EXPECT_SOF;

			n_mcu_handle_msg(&n_mcu_priv);

			break;

		case EXPECT_ESCAPED_DATA:
			priv->rx_data[priv->rx_data_size++] = data[i];
			priv->parser_state = EXPECT_DATA;
			break;

		default:
			BUG();
		}
	}
}

int n_mcu_send_frame(struct n_mcu_priv *priv)
{
	u8 *frame;
	ssize_t to_write;

	pr_debug("%s: enter\n", __func__);

	frame = priv->tx_frame;
	to_write = priv->tx_frame_size;

	set_bit(TTY_DO_WRITE_WAKEUP, &priv->tty->flags);
	while (to_write) {
		ssize_t ret = priv->tty->ops->write(priv->tty, frame, to_write);
		if (ret < 0)
			return ret;

		to_write -= ret;
		frame += ret;
	};

	return 0;
}

static int n_mcu_execute_cmd(struct n_mcu_cmd* cmd)
{
	int retry_count = MAX_RETRIES;
	int ret;

	pr_debug("%s: enter\n", __func__);

start:
	if (wait_event_interruptible(n_mcu_priv.wait,
		   !atomic_read(&n_mcu_priv.transfer_in_progress)))
		return -ERESTART;

	spin_lock(&n_mcu_tx_lock);

	if (atomic_cmpxchg(&n_mcu_priv.transfer_in_progress, 0, 1)) {
		spin_unlock(&n_mcu_tx_lock);
		goto start;
	}

	ret = n_mcu_prepare_frame(&n_mcu_priv, cmd->data, cmd->size);
	if (ret)
		goto out;

	reinit_completion(&n_mcu_priv.cmd_complete);
	n_mcu_priv.transfer_failed = false;

	ret = n_mcu_send_frame(&n_mcu_priv);
	if (ret)
		goto out;

	spin_unlock(&n_mcu_tx_lock);

	if (wait_for_completion_timeout(&n_mcu_priv.cmd_complete,
					msecs_to_jiffies(CMD_TIMEOUT))) {

		if (!n_mcu_priv.transfer_failed) {
			cmd->size = n_mcu_priv.rx_data_size;
			memcpy(cmd->data, n_mcu_priv.rx_data,
			       n_mcu_priv.rx_data_size);
			goto out;
		}
		/* else it is failed due to checksum validation,
		 * fallback to retry loop */
	}

	atomic_set(&n_mcu_priv.transfer_in_progress, 0);
	atomic_set(&n_mcu_priv.rx_ready, 1);

	if (--retry_count)
		goto start;

	return -ETIMEDOUT;

out:
	if (ret) {
		atomic_set(&n_mcu_priv.transfer_in_progress, 0);
		atomic_set(&n_mcu_priv.rx_ready, 1);
		spin_unlock(&n_mcu_tx_lock);
	}

	return ret;
}

static int n_mcu_execute_cmd_no_response(struct n_mcu_cmd* cmd)
{
	int ret;

	pr_debug("%s: enter\n", __func__);

	/* No wait for completion of previous transfer.
	 * This is special function to send high priority commands in atomic
	 * context, e.g. HW reset from kernel restart handler */

	spin_lock(&n_mcu_tx_lock);

	if (atomic_cmpxchg(&n_mcu_priv.transfer_in_progress, 0, 1))
		pr_warn("%s: previous transfer not completed\n", __func__);

	ret = n_mcu_prepare_frame(&n_mcu_priv, cmd->data, cmd->size);
	if (ret)
		goto err;

	n_mcu_priv.transfer_failed = false;

	ret = n_mcu_send_frame(&n_mcu_priv);

err:
	if (ret) {
		atomic_set(&n_mcu_priv.transfer_in_progress, 0);
		atomic_set(&n_mcu_priv.rx_ready, 1);
	}
	spin_unlock(&n_mcu_tx_lock);
	return ret;
}

static void n_mcu_handle_event(void)
{
	struct n_mcu_cmd cmd;

	pr_debug("%s: enter\n", __func__);

start:
	if (wait_event_interruptible(n_mcu_priv.wait,
		!atomic_read(&n_mcu_priv.transfer_in_progress)))
		return;

	spin_lock(&n_mcu_tx_lock);

	if (atomic_cmpxchg(&n_mcu_priv.transfer_in_progress, 0, 1)) {
		spin_unlock(&n_mcu_tx_lock);
		goto start;
	}

	cmd.size = n_mcu_priv.rx_data_size;
	memcpy(cmd.data, n_mcu_priv.rx_data, n_mcu_priv.rx_data_size);

	atomic_set(&n_mcu_priv.transfer_in_progress, 0);
	atomic_set(&n_mcu_priv.rx_ready, 1);

	spin_unlock(&n_mcu_tx_lock);

	if (n_mcu_priv.event)
		n_mcu_priv.event(n_mcu_priv.callback_cookie, &cmd);

}

static int n_mcu_event_response(struct n_mcu_cmd *cmd)
{
	int ret;

	pr_debug("%s: enter\n", __func__);
start:
	if (wait_event_interruptible(n_mcu_priv.wait,
		   !atomic_read(&n_mcu_priv.transfer_in_progress)))
		return -ERESTART;

	spin_lock(&n_mcu_tx_lock);

	if (atomic_cmpxchg(&n_mcu_priv.transfer_in_progress, 0, 1)) {
		spin_unlock(&n_mcu_tx_lock);
		goto start;
	}

	ret = n_mcu_prepare_frame(&n_mcu_priv, cmd->data, cmd->size);
	if (ret)
		goto out;

	ret = n_mcu_send_frame(&n_mcu_priv);
	if (ret)
		goto out;

out:
	atomic_set(&n_mcu_priv.transfer_in_progress, 0);
	atomic_set(&n_mcu_priv.rx_ready, 1);

	spin_unlock(&n_mcu_tx_lock);

	return ret;
}

static int n_mcu_configure_ops(struct n_mcu_priv *priv,
		struct n_mcu_ops* ops)
{
	pr_debug("%s: enter\n", __func__);

	mutex_lock(&n_mcu_lock);

	/* provide cmd execution calls to client */
	ops->cmd = n_mcu_execute_cmd;
	ops->cmd_no_response = n_mcu_execute_cmd_no_response;
	ops->event_response = n_mcu_event_response;

	/* callbacks from client */
	priv->event = ops->event;
	priv->callback_cookie = ops->callback_cookie;

	mutex_unlock(&n_mcu_lock);

	return 0;
}

static int n_mcu_open(struct tty_struct *tty)
{
	int ret = -EEXIST;

	pr_debug("%s: enter\n", __func__);

	mutex_lock(&n_mcu_lock);

	if (!n_mcu_priv.opened) {
		memset(&n_mcu_priv, 0, sizeof(struct n_mcu_priv));

		n_mcu_priv.tty = tty_kref_get(tty);

		if (n_mcu_priv.tty == NULL) {
			ret = -ENODEV;
			goto unlock;
		}

		n_mcu_priv.opened = true;
		tty->receive_room = MAX_SERIAL_BUF_SIZE;
		tty->disc_data = &n_mcu_priv;

		tty_driver_flush_buffer(tty);

		init_waitqueue_head(&n_mcu_priv.wait);
		init_completion(&n_mcu_priv.cmd_complete);

		atomic_set(&n_mcu_priv.rx_ready, 1);

		ret = 0;

	}
unlock:
	mutex_unlock(&n_mcu_lock);
	return ret;
}

static void n_mcu_close(struct tty_struct *tty)
{
	struct n_mcu_priv *priv = tty->disc_data;

	pr_debug("%s: enter\n", __func__);

	if (!priv)
		return;

	mutex_lock(&n_mcu_lock);

	tty_driver_flush_buffer(tty);

	wake_up_interruptible_all(&n_mcu_priv.wait);

	tty_kref_put(n_mcu_priv.tty);

	n_mcu_priv.opened = false;
	tty->disc_data = NULL;

	mutex_unlock(&n_mcu_lock);

}

static int n_mcu_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct n_mcu_priv *priv = tty->disc_data;

	pr_debug("%s: enter\n", __func__);

	if (!priv)
		return -EINVAL;

	switch (cmd) {
	case N_MCU_SET_CHECKSUM_TYPE:
		return n_mcu_set_checksum(arg);
	case N_MCU_CONFIGURE_OPS:
		return n_mcu_configure_ops(priv, (struct n_mcu_ops*)arg);
	default:
		return n_tty_ioctl_helper(tty, file, cmd, arg);
	}
}

static void n_mcu_receive(struct tty_struct *tty, const u8 *data,
		char *flags, int count)
{
	struct n_mcu_priv *priv = tty->disc_data;

	pr_debug("%s: enter\n", __func__);

	WARN_ON(priv->tty != n_mcu_priv.tty);

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "n_mcu_receive: data: ", DUMP_PREFIX_OFFSET,
			16, 1, data, count, true);
#endif

	n_mcu_parse_data(priv, data, count);
}

static struct tty_ldisc_ops n_mcu_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= DRIVER_NAME,
	.open		= n_mcu_open,
	.close		= n_mcu_close,
	.ioctl		= n_mcu_ioctl,
	.receive_buf	= n_mcu_receive,
};

static int __init n_mcu_init(void)
{
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = tty_register_ldisc(N_MCU, &n_mcu_ldisc);
	if (ret < 0)
		pr_err("%s: registration failed: %d\n", __func__, ret);

	return ret;
}

static void __exit n_mcu_exit(void)
{
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = tty_unregister_ldisc(N_MCU);
	if (ret < 0)
		pr_err("%s: unregistration failed: %d\n", __func__, ret);
}

module_init(n_mcu_init);
module_exit(n_mcu_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_ALIAS_LDISC(N_MCU);
MODULE_DESCRIPTION("Generic microcontroller unit ldisc driver");
