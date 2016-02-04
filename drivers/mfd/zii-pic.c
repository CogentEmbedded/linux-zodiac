/*
 * zii-pic.c - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
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

#define DEBUG
#define VERBOSE_DEBUG

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/intel-hex.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/serial_core.h>
#include <linux/syscalls.h>
#include <linux/uart_slave.h>

#include "zii-pic-niu.h"
#include "zii-pic-mezz.h"
#include "zii-pic-esb.h"
#include "zii-pic-rdu.h"
#include "zii-pic-rdu2.h"

#define DRIVER_NAME		"zii-pic-mfd"

static const char zii_pic_fw[] = "zii_pic.hex";

/* timeout to wait before device become ready to be opened via /dev/tty* */
#define PROBE_DEFER_TIMEOUT	500

struct zii_pic_fw_data {
	unsigned char address[4];
	unsigned char size;
	unsigned char payload[56];
} __packed;

typedef int (*zii_pic_firmware_action_t)(struct zii_pic_mfd *adev,
		struct zii_pic_fw_data *data);
typedef int (*zii_pic_firmware_eof_t)(struct zii_pic_mfd *adev,
		const struct firmware *firmware);

union zii_pic_cmd_data_set_wdt {
	struct {
		u8	get;
		u8	enable;
		u8	timeout;
	};
	u8	buf[3];
} __packed;


const static struct of_device_id zii_pic_mfd_dt_ids[] = {
	{ .compatible = "zii,pic-niu",  .data = (const void *)PIC_HW_ID_NIU},
	{ .compatible = "zii,pic-mezz", .data = (const void *)PIC_HW_ID_MEZZ},
	{ .compatible = "zii,pic-esb", .data = (const void *)PIC_HW_ID_ESB},
	{ .compatible = "zii,pic-rdu",  .data = (const void *)PIC_HW_ID_RDU},
	{ .compatible = "zii,pic-rdu2",	.data = (const void *)PIC_HW_ID_RDU2},
	{}
};

static const struct mfd_cell zii_pic_devices[] = {
	{
		.of_compatible = "zii,pic-main-eeprom",
		.name = ZII_PIC_NAME_MAIN_EEPROM,
	},
	{
		.of_compatible = "zii,pic-dds-eeprom",
		.name = ZII_PIC_NAME_DDS_EEPROM,
	},
	{
		.of_compatible = "zii,pic-watchdog",
		.name = ZII_PIC_NAME_WATCHDOG,
	},
	{
		.of_compatible = "zii,pic-hwmon",
		.name = ZII_PIC_NAME_HWMON,
	},
	{
		.of_compatible = "zii,pic-pwrbutton",
		.name = ZII_PIC_NAME_PWRBUTTON,
	},
	{
		.of_compatible = "zii,pic-backlight",
		.name = ZII_PIC_NAME_BACKLIGHT,
	},
};

static int zii_pic_process_firmware(struct zii_pic_mfd *adev,
		const struct firmware *firmware,
		zii_pic_firmware_action_t action,
		zii_pic_firmware_eof_t eof);


static int zii_pic_device_added(struct uart_slave *slave)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(&slave->dev);

	pr_debug("%s: enter\n", __func__);

	/* this callback is executed when device is _really_ added by core code
	 * but /dev/tty* is not yet ready to be used by open syscall.
	 */

	schedule_delayed_work(&adev->state_work,
		msecs_to_jiffies(PROBE_DEFER_TIMEOUT));

	return 0;
}

static void zii_pic_event_handler(void *cookie, struct n_mcu_cmd *event)
{
	struct zii_pic_mfd *adev = cookie;

	pr_debug("%s: enter\n", __func__);

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "event data: ", DUMP_PREFIX_OFFSET,
			16, 1, event->data, event->size, true);
#endif

	if (adev->hw_ops.event_handler)
		adev->hw_ops.event_handler(adev, event);
}

int zii_pic_mcu_cmd(struct zii_pic_mfd *adev,
		enum zii_pic_cmd_id id, const u8 * const data, u8 data_size)
{
	struct n_mcu_cmd mcu_cmd;
	u8 ack_id;
	int ret;

	pr_debug("%s: enter\n", __func__);

	if (unlikely(!adev->cmd[id].cmd_id)) {
		pr_warn("%s: command: %d is not implemented\n", __func__, id);
		return 0;
	}

	if (unlikely(data_size != adev->cmd[id].data_len))
		return -EINVAL;

	mcu_cmd.timeout = ZII_PIC_MCU_DEFAULT_CMD_TIMEOUT;
	mcu_cmd.size = 2 + adev->cmd[id].data_len;
	mcu_cmd.data[0] = adev->cmd[id].cmd_id;
	mcu_cmd.data[1] = ack_id = atomic_inc_return(&adev->cmd_seqn);

	/* cmd specific data */
	if (data_size) {
		memcpy(&mcu_cmd.data[2], data, data_size);
	}

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "cmd data: ", DUMP_PREFIX_OFFSET,
			16, 1, mcu_cmd.data, mcu_cmd.size, true);
#endif

	ret = adev->mcu_ops.cmd(&mcu_cmd);
	if (ret)
		return ret;

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "response data: ", DUMP_PREFIX_OFFSET,
			16, 1, mcu_cmd.data, mcu_cmd.size, true);
#endif

	/* check if it is our response */
	if (ack_id != mcu_cmd.data[1])
		return -EAGAIN;

	/* now cmd data contains response */
	if (adev->cmd[id].response_handler)
		/* do not pass header and checksum into handler */
		return adev->cmd[id].response_handler(
				adev,
				&mcu_cmd.data[2],
				mcu_cmd.size - 2 - adev->checksum_size);

	return 0;
}

int zii_pic_mcu_cmd_no_response(struct zii_pic_mfd *adev,
		enum zii_pic_cmd_id id, const u8 * const data, u8 data_size)
{
	struct n_mcu_cmd mcu_cmd;

	pr_debug("%s: enter\n", __func__);

	if (unlikely(!adev->cmd[id].cmd_id))
		return -ENOENT;

	if (unlikely(data_size != adev->cmd[id].data_len))
		return -EINVAL;

	mcu_cmd.size = 2 + adev->cmd[id].data_len;
	mcu_cmd.data[0] = adev->cmd[id].cmd_id;
	mcu_cmd.data[1] = atomic_inc_return(&adev->cmd_seqn);

	/* cmd specific data */
	if (data_size) {
		memcpy(&mcu_cmd.data[2], data, data_size);
	}

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "cmd data: ", DUMP_PREFIX_OFFSET,
			16, 1, mcu_cmd.data, mcu_cmd.size, true);
#endif

	return adev->mcu_ops.cmd_no_response(&mcu_cmd);
}

static int zii_pic_mcu_bl_cmd(struct zii_pic_mfd *adev,
		enum zii_pic_bl_cmd_id id, u32 timeout,
		const u8 * const data, u8 data_size,
		u8 **response, u8 *response_size)
{
	struct n_mcu_cmd mcu_cmd;
	u8 ack_id;
	int ret;

#ifdef VERBOSE_DEBUG
	pr_debug("%s: enter\n", __func__);
#endif
	if (unlikely(!adev->cmd[ZII_PIC_CMD_BOOTLOADER].cmd_id)) {
		pr_warn("%s: bootloader command is not implemented\n", __func__);
		return 0;
	}

	mcu_cmd.timeout = timeout;
	/* cmd id + ack + bl cmd id + data */
	mcu_cmd.size = 3 + data_size;
	mcu_cmd.data[0] = adev->cmd[ZII_PIC_CMD_BOOTLOADER].cmd_id;
	mcu_cmd.data[1] = ack_id = atomic_inc_return(&adev->cmd_seqn);
	mcu_cmd.data[2] = id;

	/* cmd specific data */
	if (data_size) {
		memcpy(&mcu_cmd.data[3], data, data_size);
	}

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "cmd data: ", DUMP_PREFIX_OFFSET,
			16, 1, mcu_cmd.data, mcu_cmd.size, true);
#endif

	ret = adev->mcu_ops.cmd(&mcu_cmd);
	if (ret)
		return ret;

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "response data: ", DUMP_PREFIX_OFFSET,
			16, 1, mcu_cmd.data, mcu_cmd.size, true);
#endif
	/* check if it is our response */
	if (ack_id != mcu_cmd.data[1])
		return -EAGAIN;

	if (response_size) {
		*response_size = mcu_cmd.size - 3 - adev->checksum_size;
		if (response) {
			u8 *p = kmalloc(*response_size, GFP_KERNEL);
			if (!p)
				return -ENOMEM;
			memcpy(p, &mcu_cmd.data[3], *response_size);
			*response = p;
		}
	}

	return 0;
}

static void zii_pic_get_reset_reason(struct zii_pic_mfd *adev)
{
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_RESET_REASON, NULL, 0);
	if (ret) {
		pr_warn("Failed to get reset reason (err = %d)\n", ret);
		adev->reset_reason = 0xFF;
	}
}

static inline int zii_pic_get_versions(struct zii_pic_mfd *adev)
{
	if (adev->hw_ops.get_versions)
		return adev->hw_ops.get_versions(adev);
	return 0;
}

static inline int zii_pic_get_status(struct zii_pic_mfd *adev)
{
	if (adev->hw_ops.get_status)
		return adev->hw_ops.get_status(adev);
	return 0;
}

static int zii_pic_configure(struct zii_pic_mfd *adev)
{
	struct ktermios ktermios;
	int checksum_type;
	int ret;

	pr_debug("%s: enter\n", __func__);

	ktermios = adev->tty->termios;
	ktermios.c_cflag = CS8 | CLOCAL | CREAD;
	ktermios.c_iflag = IGNPAR;
	ktermios.c_oflag = 0;
	ktermios.c_lflag = 0;
	ktermios.c_cc[VMIN]=0;
	ktermios.c_cc[VTIME]=100;

	tty_termios_encode_baud_rate(&ktermios, adev->baud, adev->baud);

	tty_set_termios(adev->tty, &ktermios);

	ret = tty_set_ldisc(adev->tty, N_MCU);
	if (ret) {
		pr_err("%s: unable to set ldisc, ret: %d\n", __func__, ret);
		return ret;
	}

	switch (adev->hw_id) {
	case PIC_HW_ID_NIU:
		zii_pic_niu_init(adev);
		checksum_type = N_MCU_CHECKSUM_CCITT_FALSE;
		break;

	case PIC_HW_ID_MEZZ:
		zii_pic_mezz_init(adev);
		checksum_type = N_MCU_CHECKSUM_CCITT_FALSE;
		break;

	case PIC_HW_ID_ESB:
		zii_pic_esb_init(adev);
		checksum_type = N_MCU_CHECKSUM_CCITT_FALSE;
		break;

	case PIC_HW_ID_RDU:
		zii_pic_rdu_init(adev);
		checksum_type = N_MCU_CHECKSUM_8B2C;
		break;

	case PIC_HW_ID_RDU2:
		zii_pic_rdu2_init(adev);
		checksum_type = N_MCU_CHECKSUM_CCITT_FALSE;
		break;

	default:
		BUG();
		break;
	}

	ret = sys_ioctl(adev->port_fd, N_MCU_SET_CHECKSUM_TYPE, checksum_type);
	if (ret)
		return ret;

	ret = sys_ioctl(adev->port_fd, N_MCU_CONFIGURE_OPS,
			(unsigned long)&adev->mcu_ops);
	if (ret)
		return ret;

	if (unlikely(!adev->mcu_ops.cmd || !adev->mcu_ops.cmd_no_response))
		BUG();

	ret = zii_pic_get_status(adev);
	if (ret)
		return ret;

	ret = zii_pic_get_versions(adev);
	if (ret)
		return ret;

	zii_pic_get_reset_reason(adev);

	ret = mfd_add_devices(adev->dev, -1, zii_pic_devices,
			ARRAY_SIZE(zii_pic_devices), NULL, 0, NULL);
	if (ret)
		return ret;

	return 0;
}

static void zii_pic_state_work(struct work_struct *work)
{
	struct zii_pic_mfd *adev =
		container_of(work, struct zii_pic_mfd, state_work.work);
	struct uart_slave *slave = container_of(adev->dev, struct uart_slave, dev);
	int ret;

	pr_debug("%s: enter\n", __func__);

	switch(adev->state) {
	case PIC_STATE_UNKNOWN:
	{
		char *tty_dev_name = kasprintf(GFP_KERNEL, "/dev/%s",
					dev_name(slave->tty_dev));
		if (!tty_dev_name) {
			pr_err("%s: not enough memory for name", __func__);
			return;
		}
		adev->port_fd = sys_open(
				(const char __user *) tty_dev_name,
				O_RDWR | O_NOCTTY, 0);
		if (adev->port_fd < 0) {
			pr_err("Warning: unable to open serial port: %ld\n",
				adev->port_fd);

			/* device is not ready yet */
			schedule_delayed_work(&adev->state_work,
				msecs_to_jiffies(PROBE_DEFER_TIMEOUT));
		}
		kfree(tty_dev_name);
		break;
	}
	case PIC_STATE_OPENED:
		ret = zii_pic_configure(adev);
		if (ret) {
			sys_close(adev->port_fd);
			adev->state = PIC_STATE_UNKNOWN;
			pr_err("%s: unable to configure PIC, ret: %d\n",
				__func__, ret);
			break;
		}
		adev->state = PIC_STATE_CONFIGURED;

		break;

	case PIC_STATE_CONFIGURED:
		break;

	default:
		BUG();
	}
}

static int zii_pic_uart_open(struct tty_struct *tty, struct file *filp)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(tty->dev->parent);
	int ret;

	pr_debug("%s: enter\n", __func__);

	/* Do not allow opening more than once */
	/* TODO: make sure this is _our_ open call, and not coming from user space */
	if (adev->state != PIC_STATE_UNKNOWN)
		return -EBUSY;

	if (!adev->uart_open)
		return -ENODEV;

	ret = adev->uart_open(tty, filp);
	if (ret) {
		pr_err("%s: unable to open uart port\n", __func__);
		return ret;
	}

	adev->state = PIC_STATE_OPENED;
	adev->tty = tty;

	/* Continue with port initialization and device probing.
	 * Impossible to set speed and ldisc here as tty_lock already held
	 */
	schedule_delayed_work(&adev->state_work, 0);

	return 0;
}

static int zii_pic_query_device(struct zii_pic_mfd *adev)
{
	u8 *response;
	u8 response_size;
	int ret;
	pr_debug("%s: enter\n", __func__);

	ret = zii_pic_mcu_bl_cmd(adev,
		ZII_PIC_BL_CMD_QUERY_DEVICE, 1000,
		NULL, 0,
		&response, &response_size);

	if (ret)
		return ret;

	/* TODO: do we need to process response ? */
	kfree(response);

	return 0;
}

static int zii_pic_erase_device(struct zii_pic_mfd *adev)
{
	pr_debug("%s: enter\n", __func__);

	return zii_pic_mcu_bl_cmd(adev,
			ZII_PIC_BL_CMD_ERASE_APP, 7000,
			NULL, 0,
			NULL, NULL);
}

static int zii_pic_verify_firmware_chunk(struct zii_pic_mfd *adev,
		struct zii_pic_fw_data *data)
{
	struct zii_pic_fw_data read_data = {
		.address = {
			data->address[0], data->address[1],
			data->address[2], data->address[3]
		},
		.size = data->size
	};
	u8 *response;
	u8 response_size;
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = zii_pic_mcu_bl_cmd(adev,
		ZII_PIC_BL_CMD_READ_APP, 5000,
		(const u8 * const)&read_data,
		sizeof(read_data.address) + sizeof(read_data.size),
		&response, &response_size);

	if (ret)
		return ret;

	if (memcmp(response, data, response_size)) {
		print_hex_dump(KERN_ERR, "error verifying block: ", DUMP_PREFIX_OFFSET,
				16, 1, response, response_size, true);
		print_hex_dump(KERN_ERR, "fw block: ", DUMP_PREFIX_OFFSET,
				16, 1, data, data->size + 5, true);
		ret = -EIO;
	}

	kfree(response);

	return ret;
}

static int zii_pic_verify_firmware_eof(struct zii_pic_mfd *adev,
		const struct firmware *firmware)
{
	/* Everything is fine so far, send complete command to PIC */
	pr_debug("%s: going to commit changes\n", __func__);
	return zii_pic_mcu_bl_cmd(adev,
		ZII_PIC_BL_CMD_PROGRAM_COMPLETE, 2000,
		NULL, 0, NULL, NULL);
}

static int zii_pic_upload_firmware_chunk(struct zii_pic_mfd *adev,
		struct zii_pic_fw_data *data)
{
	return zii_pic_mcu_bl_cmd(adev,
		ZII_PIC_BL_CMD_PROGRAM_DEVICE, 1000,
		(const u8 * const)data,
		sizeof(data->address) + sizeof(data->size) + data->size,
		NULL, NULL);
}

static int zii_pic_upload_firmware_eof(struct zii_pic_mfd *adev,
		const struct firmware *firmware)
{
	pr_debug("%s: starting firmware verification\n", __func__);
	return zii_pic_process_firmware(adev, firmware,
			zii_pic_verify_firmware_chunk,
			zii_pic_verify_firmware_eof);
}

static int zii_pic_process_firmware(struct zii_pic_mfd *adev,
		const struct firmware *firmware,
		zii_pic_firmware_action_t action,
		zii_pic_firmware_eof_t eof)
{
	unsigned char addr_has_changed = 0;
	int total_read_bytes = 0;
	int percent = 0;
	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	for (total_read_bytes = 0; total_read_bytes < firmware->size; ) {
		struct zii_pic_fw_data data;
		unsigned char chunk_data[64];
		u32 chunk_address;
		int read_bytes = 0;

		read_bytes = parse_hex_line(
				(unsigned char *)(firmware->data + total_read_bytes),
				(unsigned char *)&chunk_address,
				chunk_data,
				&data.size,
				&addr_has_changed);

		if (read_bytes <= 0)
			break;

		/* detect the end of file */
		total_read_bytes += read_bytes;

		if (total_read_bytes * 100 / firmware->size > percent)
			pr_debug("%s: progress: %d%%, %d from %d\n", __func__,
				++percent, total_read_bytes, firmware->size);

		if (total_read_bytes == firmware->size) {
			pr_debug("%s: completed\n", __func__);
			if (eof)
				ret = eof(adev, firmware);
			break;
		}

		if (!addr_has_changed) {
			*(u32*)data.address = be32_to_cpu(chunk_address) / 2;
			memcpy(data.payload, chunk_data, data.size);
			ret = action(adev, &data);
			if (ret < 0)
				break;
		}
	}

	pr_debug("%s: processed %d bytes, exit code: %d\n", __func__,
		 total_read_bytes, ret);

	return ret;
}

static ssize_t zii_pic_show_fw_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);

	pr_debug("%s: enter\n", __func__);

	return snprintf(buf, PAGE_SIZE,
			"%d.%d.%d.%c%c\n",
			adev->firmware_version.hw,
			adev->firmware_version.major,
			adev->firmware_version.minor,
			adev->firmware_version.letter_1,
			adev->firmware_version.letter_2);
}

static DEVICE_ATTR(part_number_firmware, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_fw_version, NULL);

static ssize_t zii_pic_show_bl_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);

	pr_debug("%s: enter\n", __func__);

	return snprintf(buf, PAGE_SIZE,
			"%d.%d.%d.%c%c\n",
			adev->bootloader_version.hw,
			adev->bootloader_version.major,
			adev->bootloader_version.minor,
			adev->bootloader_version.letter_1,
			adev->bootloader_version.letter_2);
}
static DEVICE_ATTR(part_number_bootloader, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_bl_version, NULL);

static ssize_t zii_pic_show_reset_reason(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", adev->reset_reason);
}

static DEVICE_ATTR(reset_reason, S_IRUSR | S_IRGRP,
		zii_pic_show_reset_reason, NULL);


static ssize_t zii_pic_show_boot_source(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);

	if (adev->hw_ops.get_boot_source) {
		int ret;
		ret = adev->hw_ops.get_boot_source(adev);
		if (ret)
			return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", adev->boot_source);
}

static ssize_t zii_pic_store_boot_source(struct device *dev,
		struct device_attribute *attr,  const char *buf, size_t count)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);
	u8 boot_src;
	int ret;

	ret = kstrtou8(buf, 0, &boot_src);
	if (ret)
		return ret;

	if (boot_src > PIC_BOOT_SRC_LAST)
		return -EINVAL;

	if (!adev->hw_ops.set_boot_source)
		return -ENOTSUPP;

	ret = adev->hw_ops.set_boot_source(adev, boot_src);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(boot_source, S_IRUSR | S_IWUSR | S_IRGRP,
		zii_pic_show_boot_source, zii_pic_store_boot_source);

static ssize_t zii_pic_store_update_firmware(struct device *dev,
		struct device_attribute *attr,  const char *buf, size_t count)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);
	const struct firmware *firmware = NULL;
	int ret;

	/* TODO: what would be proper locking for the PIC?
	 * - atomic
	 * - mutex
	 * - prevent any other commands execution? */
	if (atomic_read(&adev->fw_update_started))
		return -EBUSY;

	atomic_set(&adev->fw_update_started, 1);

	/* TODO: is it needed to have specific parsing of input parameters?
	 * Maybe interpret written data as file name inside /lib/firmware ? */

	ret = request_firmware(&firmware, zii_pic_fw, dev);
	if (ret)
		goto out;

	ret = zii_pic_mcu_cmd_no_response(adev, ZII_PIC_CMD_JMP_TO_BOOTLOADER,
			NULL, 0);
	if (ret)
		goto out;

	/* wait until bootloader is ready to accept commands */
	mdelay(3000);

	ret = zii_pic_watchdog_disable(adev->dev);
	if (ret)
		return ret;

	ret = zii_pic_query_device(adev);
	if (ret)
		goto out;

	ret = zii_pic_erase_device(adev);
	if (ret)
		goto out;

	ret = zii_pic_process_firmware(adev, firmware,
			zii_pic_upload_firmware_chunk,
			zii_pic_upload_firmware_eof);
	if (ret)
		goto out;

	pr_debug("%s: firware updated successfully\n", __func__);

out:
	release_firmware(firmware);
	atomic_set(&adev->fw_update_started, 0);

	return count;
}
static DEVICE_ATTR(update_firmware, S_IWUSR,
		NULL, zii_pic_store_update_firmware);

static struct attribute *zii_pic_dev_attrs[] = {
	&dev_attr_part_number_firmware.attr,
	&dev_attr_part_number_bootloader.attr,
	&dev_attr_reset_reason.attr,
	&dev_attr_boot_source.attr,
	&dev_attr_update_firmware.attr,
	NULL
};

static const struct attribute_group zii_pic_dev_attr_group = {
	.attrs = zii_pic_dev_attrs,
};

static int zii_pic_create_sysfs_groups(struct device *dev,
			struct zii_pic_mfd *adev)
{
	const struct attribute_group **p;
	int num_groups;
	int ret;

	pr_debug("%s: enter\n", __func__);

	/* start from one to count NULL terminator itself */
	for (p = dev->groups, num_groups = 1; p && *p;
			num_groups++, p++);

	/* add place for our attributes and copy original group */
	p = kcalloc(num_groups + 1, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p[--num_groups] = &zii_pic_dev_attr_group;
	while(num_groups--)
		p[num_groups] = dev->groups[num_groups];

	ret = sysfs_create_groups(&dev->kobj, p);
	if (ret) {
		kfree(p);
		return ret;
	}
	adev->groups = dev->groups = p;

	return 0;
}

static int zii_pic_mfd_probe(struct device *dev)
{
	struct uart_slave *slave = container_of(dev, struct uart_slave, dev);
	struct zii_pic_mfd *adev;
	const struct of_device_id *id;
	u32 baud;
	int ret;

	pr_debug("%s: enter\n", __func__);

	if (dev->parent == NULL)
		return -ENODEV;

	id = of_match_device(zii_pic_mfd_dt_ids, dev);
	if (!id)
		return -ENODEV;

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->dev = dev;
	adev->hw_id = (enum zii_pic_hw_id)id->data;

	if (of_property_read_u32(dev->of_node, "current-speed", &baud))
		adev->baud = ZII_PIC_DEFAULT_BAUD_RATE;
	else
		adev->baud = baud;
	adev->mcu_ops.event = zii_pic_event_handler;
	adev->mcu_ops.callback_cookie = adev;
	slave->device_added = zii_pic_device_added;

	adev->uart_open = slave->ops.open;
	slave->ops.open = zii_pic_uart_open;

	ret = zii_pic_create_sysfs_groups(dev, adev);
	if (ret)
		return ret;

	dev_set_drvdata(dev, adev);

	INIT_DELAYED_WORK(&adev->state_work, zii_pic_state_work);

	ret = uart_slave_add_tty(slave);
	if (ret)
		return ret;

	return 0;
}

static int zii_pic_mfd_remove(struct device *dev)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(dev);

	pr_debug("%s: enter\n", __func__);

	sysfs_remove_groups(&dev->kobj, adev->groups);
	kfree(adev->groups);

	cancel_delayed_work_sync(&adev->state_work);

	return 0;
}

int zii_pic_watchdog_enable(struct device *pic_dev)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	union zii_pic_cmd_data_set_wdt data;
	int ret;

	pr_debug("%s: enter\n", __func__);

	data.get = 0;
	data.enable = 1;
	data.timeout = adev->watchdog_timeout;

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_SW_WDT_SET,
			      data.buf, sizeof(data));
	if (ret)
		pr_err("Failed to enable watchdog (err = %d)\n", ret);
	else
		adev->watchdog_enabled = 1;

	return ret;
}

int zii_pic_watchdog_disable(struct device *pic_dev)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	union zii_pic_cmd_data_set_wdt data;
	int ret;

	pr_debug("%s: enter\n", __func__);

	data.get = 0;
	data.enable = 0;
	data.timeout = 0;

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_SW_WDT_SET,
			      data.buf, sizeof(data));
	if (ret)
		pr_err("Failed to disable watchdog (err = %d)\n", ret);
	else
		adev->watchdog_enabled = 0;

	return ret;
}

int zii_pic_watchdog_get_status(struct device *pic_dev)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	u8 data = 1;
	int ret;

	pr_debug("%s: enter\n", __func__);

	/* Special case for NIU HW as firmware does not respond properly
	 * Assume watchdog is enabled and has default timeout
	 */
	if (adev->hw_id == PIC_HW_ID_NIU) {
		adev->watchdog_enabled = 1;
		adev->watchdog_timeout = ZII_PIC_WDT_DEFAULT_TIMEOUT;
		return 0;
	}

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_SW_WDT_GET,
				&data, sizeof(data));

	if (ret)
		pr_err("Failed to get watchdog status (err = %d)\n", ret);

	return ret;
}

int zii_pic_watchdog_ping(struct device *pic_dev)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_PET_WDT, NULL, 0);
	if (ret)
		pr_err("Failed to pet the dog (err = %d)\n", ret);

	return ret;
}

int zii_pic_watchdog_set_timeout(struct device *pic_dev,
		unsigned int timeout)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	union zii_pic_cmd_data_set_wdt data;
	int ret;

	pr_debug("%s: enter\n", __func__);

	if (timeout < ZII_PIC_WDT_MIN_TIMEOUT ||
	    timeout > ZII_PIC_WDT_MAX_TIMEOUT)
		return -EINVAL;

	data.get = 0;
	data.enable = adev->watchdog_enabled;
	data.timeout = timeout;

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_SW_WDT_SET,
			data.buf, sizeof(data));
	if (ret)
		pr_err("Failed to set watchdog timeout (err = %d)\n", ret);
	else
		adev->watchdog_timeout = timeout;

	return ret;
}

void zii_pic_watchdog_reset(struct device *pic_dev, bool hw_recovery)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	int ret;

	pr_debug("%s: enter\n", __func__);

	if (!adev->hw_ops.reset)
		return;

	/* There is no way to get normal processing of response for this
	 * command due to disabled scheduling when reset handler is called.
	 * So repeat until it is successfully executed or watchdog
	 * triggers reset */
	for (;;) {
		ret = adev->hw_ops.reset(adev);
		if (ret)
			pr_emerg("%s: failed to reset (err = %d)\n",
				__func__, ret);

		/* PIC firmware waits for 500 ms before resetting */
		mdelay(550);

		pr_emerg("%s: reset cmd timed out, repeat\n", __func__);
	}
}

int zii_pic_hwmon_read_sensor(struct device *pic_dev,
			      enum zii_pic_sensor id, int *val)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);

	pr_debug("%s: enter\n", __func__);

	if (!adev->hw_ops.read_sensor)
		return -ENOTSUPP;

	return adev->hw_ops.read_sensor(adev, id, val);
}

static int zii_pic_eeprom_read_page(struct zii_pic_mfd *adev,
		enum zii_pic_eeprom_type type, u16 page)
{
	switch(type) {
	case MAIN_EEPROM:
	{
		u8 cmd_data[3] = {1, page & 0xFF, page >> 8};
		return zii_pic_mcu_cmd(adev, ZII_PIC_CMD_EEPROM_READ,
				cmd_data, sizeof(cmd_data));
	}
	case DDS_EEPROM:
	{
		u8 cmd_data[2] = {1, page & 0xFF};
		return zii_pic_mcu_cmd(adev, ZII_PIC_CMD_DDS_EEPROM_READ,
				cmd_data, sizeof(cmd_data));
	}
	default:
		BUG();
	}
	return 0;
}

static int zii_pic_eeprom_write_page(struct zii_pic_mfd *adev,
		enum zii_pic_eeprom_type type, u16 page, const u8 *data)
{
	switch(type) {
	case MAIN_EEPROM:
	{
		u8 cmd_data[35] = {0, page & 0xFF, page >> 8};
		memcpy(&cmd_data[3], data, ZII_PIC_EEPROM_PAGE_SIZE);
		return zii_pic_mcu_cmd(adev, ZII_PIC_CMD_EEPROM_WRITE,
				cmd_data, sizeof(cmd_data));
	}
	case DDS_EEPROM:
	{
		u8 cmd_data[34] = {0, page & 0xFF};
		memcpy(&cmd_data[2], data, ZII_PIC_EEPROM_PAGE_SIZE);
		return zii_pic_mcu_cmd(adev, ZII_PIC_CMD_DDS_EEPROM_WRITE,
				cmd_data, sizeof(cmd_data));
	}
	default:
		BUG();
	}
	return 0;
}

int zii_pic_eeprom_read(struct device *pic_dev,
		enum zii_pic_eeprom_type type, u16 reg,
		void *val, size_t val_size)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	int page = reg >> 5;
	int offset = reg & 0x1f;
	size_t bytes_left = val_size;
	int ret;

	pr_debug("%s: enter\n", __func__);

	do {
		int count = bytes_left > ZII_PIC_EEPROM_PAGE_SIZE ?
			ZII_PIC_EEPROM_PAGE_SIZE : bytes_left;

		if (unlikely(count + offset > ZII_PIC_EEPROM_PAGE_SIZE))
			count -= offset;

		ret = zii_pic_eeprom_read_page(adev, type, page);
		if (ret)
			return ret;

		pr_debug("%s: returning %d bytes from %d\n", __func__, count, val_size);
		memcpy(val, adev->eeprom_page + offset, count);

		page++;
		offset = 0;
		bytes_left -= count;
		val += count;
	} while (bytes_left);

	return 0;
}

int zii_pic_eeprom_write(struct device *pic_dev,
		enum zii_pic_eeprom_type type, u16 reg,
		const void *data, size_t size)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	int page = reg >> 5;
	int offset = reg & 0x1f;
	size_t bytes_left = size;
	int ret;

	pr_debug("%s: enter\n", __func__);

	do {
		int count = bytes_left > ZII_PIC_EEPROM_PAGE_SIZE ?
			ZII_PIC_EEPROM_PAGE_SIZE : bytes_left;

		if (unlikely(count + offset > ZII_PIC_EEPROM_PAGE_SIZE))
			count -= offset;

		if (count == ZII_PIC_EEPROM_PAGE_SIZE)
			ret = zii_pic_eeprom_write_page(adev, type, page, data);
		else {
			ret = zii_pic_eeprom_read_page(adev, type, page);
			if (ret)
				return ret;

			memcpy(adev->eeprom_page + offset, data, count);

			ret = zii_pic_eeprom_write_page(adev, type,
					page, adev->eeprom_page);
		}
		if (ret)
			return ret;

		page++;
		offset = 0;
		bytes_left -= count;
		data += count;

	} while(bytes_left);

	return 0;
}

int zii_pic_register_pwrbutton_callback(struct device *pic_dev,
		zii_pic_pwrbutton_callback_t callback,
		void *pwrbutton)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);

	pr_debug("%s: enter\n", __func__);

	adev->pwrbutton_event = callback;
	adev->pwrbutton = pwrbutton;

	return 0;
}

int zii_pic_backlight_set(struct device *pic_dev, int intensity)
{
	struct zii_pic_mfd *adev = dev_get_drvdata(pic_dev);
	u8 data[3] = {0, 0, 0};

	pr_debug("%s: enter\n", __func__);

	if (intensity < 0 || intensity > 100)
		return -EINVAL;

	data[0] = intensity;
	if (intensity)
		data[0] |= 0x80;

	return zii_pic_mcu_cmd(adev, ZII_PIC_CMD_BACKLIGHT, data, sizeof(data));
}



static struct device_driver zii_pic_mfd_drv = {
	.name			= DRIVER_NAME,
	.owner			= THIS_MODULE,
	.of_match_table		= zii_pic_mfd_dt_ids,
	.probe			= zii_pic_mfd_probe,
	.remove			= zii_pic_mfd_remove,
};

/*
 * Module init
 */
static int __init zii_pic_mfd_init_module(void)
{
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = uart_slave_driver_register(&zii_pic_mfd_drv);

	return ret;
}

static void __exit zii_pic_mfd_exit_module(void)
{
	pr_debug("%s: enter\n", __func__);

	driver_unregister(&zii_pic_mfd_drv);
}

module_init(zii_pic_mfd_init_module);
module_exit(zii_pic_mfd_exit_module);

MODULE_DEVICE_TABLE(of, zii_pic_mfd_dt_ids);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC MCU core driver");
