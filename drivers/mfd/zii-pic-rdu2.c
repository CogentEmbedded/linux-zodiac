/*
 * zii-pic-rdu2.c - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 * (RDU2 board specific code)
 *
 * Copyright (C) 2016 Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/nvmem-consumer.h>

#include "zii-pic-niu.h"
#include "zii-pic-rdu.h"
#include "zii-pic-rdu2.h"

/* Main RDU EEPROM has same command/response structure */
#define zii_pic_rdu2_process_reset_reason	zii_pic_rdu_process_reset_reason
#define zii_pic_rdu2_process_temperature	zii_pic_niu_process_temperature
#define zii_pic_rdu2_process_eeprom_read	zii_pic_niu_process_eeprom_read
#define zii_pic_rdu2_process_eeprom_write	zii_pic_niu_process_eeprom_write
#define zii_pic_rdu2_process_dds_eeprom_read	zii_pic_rdu_process_dds_eeprom_read
#define zii_pic_rdu2_process_dds_eeprom_write	zii_pic_rdu_process_dds_eeprom_write
#define zii_pic_rdu2_process_get_boot_source	zii_pic_niu_process_get_boot_source
#define zii_pic_rdu2_process_firmware_version	zii_pic_niu_process_firmware_version
#define zii_pic_rdu2_process_bootloader_version	zii_pic_niu_process_bootloader_version

struct zii_pic_cmd_desc zii_pic_rdu2_cmds[ZII_PIC_CMD_COUNT] = {
	/* ZII_PIC_CMD_GET_STATUS */
	{0,    0, NULL},
	/* ZII_PIC_CMD_SW_WDT_SET */
	{0xA1, 3, NULL},
	/* ZII_PIC_CMD_SW_WDT_GET */
	{0,    0, NULL},
	/* ZII_PIC_CMD_PET_WDT    */
	{0xA2, 0, NULL},
	/* ZII_PIC_CMD_RESET  */
	{0xA7, 2, NULL},
	/* ZII_PIC_CMD_HW_RECOVERY_RESET  */
	{0xA7, 2, NULL},
	/* ZII_PIC_CMD_GET_RESET_REASON */
	{0xA8, 0, zii_pic_rdu2_process_reset_reason},
	/* ZII_PIC_CMD_GET_28V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_12V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_5V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_3V3_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_VOLTAGE */
	{0x22, 1, zii_pic_rdu2_process_voltage_response},
	/* ZII_PIC_CMD_GET_CURRENT */
	{0x23, 1, zii_pic_rdu2_process_current_response},
	/* ZII_PIC_CMD_GET_TEMPERATURE */
	{0x24, 1, zii_pic_rdu2_process_temperature},
	/* ZII_PIC_CMD_EEPROM_READ */
	{0xA4, 3, zii_pic_rdu2_process_eeprom_read},
	/* ZII_PIC_CMD_EEPROM_WRITE */
	{0xA4, 35, zii_pic_rdu2_process_eeprom_write},
	/* ZII_PIC_CMD_GET_FIRMWARE_VERSION */
	{0x20,  0, zii_pic_rdu2_process_firmware_version},
	/* ZII_PIC_CMD_GET_BOOTLOADER_VERSION */
	{0x21,  0, zii_pic_rdu2_process_bootloader_version},
	/* ZII_PIC_CMD_DDS_EEPROM_READ */
	{0xA3, 2, zii_pic_rdu2_process_dds_eeprom_read},
	/* ZII_PIC_CMD_DDS_EEPROM_WRITE */
	{0xA3, 34, zii_pic_rdu2_process_dds_eeprom_write},
	/* ZII_PIC_CMD_GET_BOOT_SOURCE */
	{0x23, 2, zii_pic_rdu2_process_get_boot_source},
	/* ZII_PIC_CMD_SET_BOOT_SOURCE */
	{0x23, 2, NULL},
	/* ZII_PIC_CMD_LCD_BOOT_ENABLE */
	{0xBE, 0, NULL},
	/* ZII_PIC_CMD_BACKLIGHT */
	{0xA6, 3, NULL},
	/* ZII_PIC_CMD_JMP_TO_BOOTLOADER */
	{0xB0, 0, NULL},
	/* ZII_PIC_CMD_BOOTLOADER */
	{0xB1, 0xff, NULL},
};

int zii_pic_rdu2_process_voltage_response(struct zii_pic_mfd *adev,
					u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to mV  */
	adev->sensor_voltage = zii_pic_f88_to_int(data);
	return 0;
}

int zii_pic_rdu2_process_current_response(struct zii_pic_mfd *adev,
					u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* convert to mA  */
	adev->sensor_current = zii_pic_f88_to_int(data);
	return 0;
}

static int zii_pic_rdu2_reset(struct zii_pic_mfd *adev)
{
	u8 data[2] = {1, 0};
	return zii_pic_mcu_cmd_no_response(adev, ZII_PIC_CMD_RESET,
					   data, sizeof(data));
}

static int zii_pic_rdu2_hwmon_read_sensor(struct zii_pic_mfd *adev,
			enum zii_pic_sensor id, int *val)
{
	int ret = 0;
	pr_debug("%s: enter\n", __func__);

	switch (id) {
	case ZII_PIC_SENSOR_28V:
	case ZII_PIC_SENSOR_12V:
	case ZII_PIC_SENSOR_5V:
	case ZII_PIC_SENSOR_3V3:
		return -EINVAL;

	case ZII_PIC_SENSOR_RMB_3V3_PMIC:
	case ZII_PIC_SENSOR_RMB_3V3_MCU:
	case ZII_PIC_SENSOR_RMB_5V_MAIN:
	case ZII_PIC_SENSOR_RMB_12V_MAIN:
	case ZII_PIC_SENSOR_RMB_28V_FIL:
	case ZII_PIC_SENSOR_RMB_28V_HOTSWAP:
	case ZII_PIC_SENSOR_DEB_1V8:
	case ZII_PIC_SENSOR_DEB_3V3:
	case ZII_PIC_SENSOR_DEB_28V_DEB:
	case ZII_PIC_SENSOR_DEB_28V_RDU:
	{
		u8 data = id - ZII_PIC_SENSOR_RMB_3V3_PMIC;
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_VOLTAGE,
				&data, sizeof(data));
		if (ret)
			return ret;

		*val = adev->sensor_voltage;
		break;
	}

	case ZII_PIC_SENSOR_TEMPERATURE:
	case ZII_PIC_SENSOR_TEMPERATURE_2:
	{
		u8 data = id - ZII_PIC_SENSOR_TEMPERATURE;
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_TEMPERATURE,
				&data, sizeof(data));
		if (ret)
			return ret;

		*val = adev->sensor_temperature;
		break;
	}

	case ZII_PIC_SENSOR_BACKLIGHT_CURRENT:
		return -EINVAL;

	case ZII_PIC_SENSOR_RMB_28V_CURRENT:
	{
		u8 data = 0;
		ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_CURRENT,
				&data, sizeof(data));
		if (ret)
			return ret;
	
		*val = adev->sensor_current;
		break;
	}

	default:
		BUG();
	}

	return ret;
}

int zii_pic_rdu2_init(struct zii_pic_mfd *adev)
{
	adev->cmd = zii_pic_rdu2_cmds;
	adev->checksum_size = 2;

	adev->hw_ops.event_handler = NULL;
	adev->hw_ops.get_status = NULL;
	adev->hw_ops.get_versions = zii_pic_niu_get_versions;
	adev->hw_ops.get_boot_source = zii_pic_niu_get_boot_source;
	adev->hw_ops.set_boot_source = zii_pic_niu_set_boot_source;
	adev->hw_ops.reset = zii_pic_rdu2_reset;
	/* recovery reset command is the same as on RDU */
	adev->hw_ops.recovery_reset = zii_pic_rdu_recovery_reset;
	adev->hw_ops.read_sensor = zii_pic_rdu2_hwmon_read_sensor;

	return 0;
}

