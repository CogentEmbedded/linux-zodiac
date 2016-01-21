/*
 * zii-pic-niu.c - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 * (NIU board specific code)
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

#include <linux/kernel.h>
#include <linux/device.h>

#include "zii-pic-niu.h"

struct zii_pic_cmd_desc zii_pic_niu_cmds[ZII_PIC_CMD_COUNT] = {
	/* ZII_PIC_CMD_GET_STATUS */
	{0x10, 0, zii_pic_niu_process_status_response},
	/* ZII_PIC_CMD_SW_WDT_SET */
	{0x1C, 3, NULL},
	/* ZII_PIC_CMD_SW_WDT_GET */
	{0x1C, 1, zii_pic_niu_process_watchdog_state},
	/* ZII_PIC_CMD_PET_WDT */
	{0x1D, 0, NULL},
	/* ZII_PIC_CMD_RESET  */
	{0x1E, 1, NULL},
	/* ZII_PIC_CMD_HW_RECOVERY_RESET  */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_RESET_REASON */
	{0x1F, 0, zii_pic_niu_process_reset_reason},
	/* ZII_PIC_CMD_GET_28V_READING */
	{0x1A, 0, zii_pic_niu_process_28v},
	/* ZII_PIC_CMD_GET_12V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_5V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_3V3_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_TEMPERATURE */
	{0x19, 0, zii_pic_niu_process_temperature},
	/* ZII_PIC_CMD_EEPROM_READ */
	{0x20, 3, zii_pic_niu_process_eeprom_read},
	/* ZII_PIC_CMD_EEPROM_WRITE */
	{0x20, 35, zii_pic_niu_process_eeprom_write},
	/* ZII_PIC_CMD_GET_FIRMWARE_VERSION */
	{0x11, 0, zii_pic_niu_process_firmware_version},
	/* ZII_PIC_CMD_GET_BOOTLOADER_VERSION */
	{0x12, 0, zii_pic_niu_process_bootloader_version},
	/* ZII_PIC_CMD_DDS_EEPROM_READ */
	{0,    0, NULL},
	/* ZII_PIC_CMD_DDS_EEPROM_WRITE */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_BOOT_SOURCE */
	{0x14, 1, zii_pic_niu_process_get_boot_source},
	/* ZII_PIC_CMD_SET_BOOT_SOURCE */
	{0x14, 1, NULL},
	/* ZII_PIC_CMD_LCD_BOOT_ENABLE */
	{0,    0, NULL},
	/* ZII_PIC_CMD_BACKLIGHT */
	{0,    0, NULL},
};

int zii_pic_niu_process_status_response(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	struct zii_pic_niu_status_info *status =
			(struct zii_pic_niu_status_info*)data;
	int boot_device;

	pr_debug("%s: enter\n", __func__);

	/* bad response */
	if (size != sizeof(*status))
		return -EINVAL;

	memcpy(&adev->bootloader_version, &data[0], 6);
	memcpy(&adev->firmware_version, &data[6], 6);

	pr_debug("Bootloader: %d.%d.%d%c%c\n", status->bl_part_num_hw,
			status->bl_part_num_major_ver,
			status->bl_part_num_minor_ver,
			status->bl_part_num_letter_1,
			status->bl_part_num_letter_2);

	pr_debug("Firmware: %d.%d.%d%c%c\n", status->fw_part_num_hw,
			status->fw_part_num_major_ver,
			status->fw_part_num_minor_ver,
			status->fw_part_num_letter_1,
			status->fw_part_num_letter_2);

	pr_debug("Watchdog is %s\n", (status->general_status & 1) ? "active" : "not active");
	pr_debug("Mode: %s\n", (status->general_status & 2) ? "Bootloader" : "Firmware");

	pr_debug("NIU flag: %#x\n", status->niu_flag_num);
	pr_debug("PIC flag: %#x\n", status->pic_flag_num);
	pr_debug("ETC: %d\n", status->etc_reading);
	pr_debug("Temperature: %d.%d Celsius\n", status->temp_reading >> 1, status->temp_reading & 1 ? 5 : 0);
	pr_debug("Host GPIO status: %d\n", status->host_gpio_status & 1);
	pr_debug("Voltage: %#x\n", status->voltage_28_reading);

	pr_debug("I2C devices:\n");
	pr_debug("\tTemp sensor: %c\n", status->i2c_device_status & 1 ? 'Y' : 'N');
	pr_debug("\tETC: %c\n", status->i2c_device_status & 2 ? 'Y' : 'N');
	pr_debug("\tEEPROM: %c\n", status->i2c_device_status & 4 ? 'Y' : 'N');
	pr_debug("\tI/O Expander: %c\n", status->i2c_device_status & 8 ? 'Y' : 'N');

	boot_device = status->general_status >> 2 & 0x03;
	switch(boot_device) {
	case 0:
		pr_debug("Boot device: SD\n");
		break;
	case 1:
		pr_debug("Boot device: eMMC\n");
		break;
	case 2:
		pr_debug("Boot device: NOR Flash\n");
		break;
	}
	return 0;
}

int zii_pic_niu_process_watchdog_state(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response */
	if (size != 2)
		return -EINVAL;

	adev->watchdog_enabled = data[0];
	adev->watchdog_timeout = data[1];

	pr_debug("Watchdog state: %d, timeout: %d\n", data[0], data[1]);

	return 0;
}

int zii_pic_niu_process_reset_reason(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 1)
		return -EINVAL;

	adev->reset_reason = *data;

	return 0;
}

int zii_pic_niu_process_28v(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to mV  */
	adev->sensor_28v = zii_pic_f88_to_int(data);

	return 0;
}

int zii_pic_niu_process_12v(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to mV  */
	adev->sensor_12v = zii_pic_f88_to_int(data);

	return 0;
}

int zii_pic_niu_process_5v(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to mV  */
	adev->sensor_5v = zii_pic_f88_to_int(data);

	return 0;
}

int zii_pic_niu_process_3v3(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to mV  */
	adev->sensor_3v3 = zii_pic_f88_to_int(data);

	return 0;
}

int zii_pic_niu_process_temperature(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to millidegree Celsius */
	adev->temperature = (*(u16*)data) * 500;

	return 0;
}

int zii_pic_niu_process_firmware_version(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != sizeof(struct zii_pic_version))
		return -EINVAL;

	memcpy(&adev->firmware_version, data, size);

	return 0;
}
int zii_pic_niu_process_bootloader_version(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != sizeof(struct zii_pic_version))
		return -EINVAL;

	memcpy(&adev->bootloader_version, data, size);

	return 0;
}

int zii_pic_niu_process_eeprom_read(struct zii_pic_mfd *adev,
				u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2 + ZII_PIC_EEPROM_PAGE_SIZE)
		return -EINVAL;

	/* check operation status */
	if (!data[1])
		return -EIO;

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "EEPROM data: ", DUMP_PREFIX_OFFSET,
			16, 1, &data[2], ZII_PIC_EEPROM_PAGE_SIZE, true);
#endif

	memcpy(adev->eeprom_page, &data[2], ZII_PIC_EEPROM_PAGE_SIZE);

	return 0;
}

int zii_pic_niu_process_eeprom_write(struct zii_pic_mfd *adev,
				u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* check operation status */
	if (!data[1])
		return -EIO;

	return 0;
}

int zii_pic_niu_process_get_boot_source(struct zii_pic_mfd *adev,
			u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);
	adev->boot_source = *data;
	return 0;
}

int zii_pic_niu_hwmon_read_sensor(struct zii_pic_mfd *adev,
			enum zii_pic_sensor id, int *val)
{
	int ret;

	switch (id) {
	case ZII_PIC_SENSOR_28V:
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_28V_READING, NULL, 0);
		if (ret)
			break;
		*val = adev->sensor_28v;
		break;

	case ZII_PIC_SENSOR_12V:
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_12V_READING, NULL, 0);
		if (ret)
			break;
		*val = adev->sensor_12v;
		break;

	case ZII_PIC_SENSOR_5V:
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_5V_READING, NULL, 0);
		if (ret)
			break;
		*val = adev->sensor_5v;
		break;

	case ZII_PIC_SENSOR_3V3:
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_3V3_READING, NULL, 0);
		if (ret)
			break;
		*val = adev->sensor_3v3;
		break;

	case ZII_PIC_SENSOR_TEMPERATURE:
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_TEMPERATURE, NULL, 0);
		if (ret)
			break;
		*val = adev->temperature;
		break;

	default:
		BUG();
	}
	return ret;
}

