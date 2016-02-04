/*
 *  zii-pic-esb.c - Multifunction core driver for Zodiac Inflight Infotainment
 *  PIC MCU that is connected via dedicated UART port
 * (ESB board specific code)
 *
 * Copyright (C) 2015-2016 Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
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
#include "zii-pic-esb.h"

struct zii_pic_cmd_desc zii_pic_esb_cmds[ZII_PIC_CMD_COUNT] = {
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
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_12V_READING */
	{0x2C, 0, zii_pic_niu_process_12v},
	/* ZII_PIC_CMD_GET_5V_READING */
	{0x2E, 0, zii_pic_niu_process_5v},
	/* ZII_PIC_CMD_GET_3V3_READING */
	{0x2F, 0, zii_pic_niu_process_3v3},
	/* ZII_PIC_CMD_GET_VOLTAGE */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_CURRENT */
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
	{0x14, 2, zii_pic_niu_process_get_boot_source},
	/* ZII_PIC_CMD_SET_BOOT_SOURCE */
	{0x14, 2, NULL},
	/* ZII_PIC_CMD_LCD_BOOT_ENABLE */
	{0,    0, NULL},
	/* ZII_PIC_CMD_BACKLIGHT */
	{0,    0, NULL},
	/* ZII_PIC_CMD_JMP_TO_BOOTLOADER */
	{0x29, 0, NULL},
	/* ZII_PIC_CMD_BOOTLOADER */
	{0x2A, 0xff, NULL},
};

int zii_pic_esb_init(struct zii_pic_mfd *adev)
{
	adev->cmd = zii_pic_esb_cmds;
	adev->checksum_size = 2;

	adev->hw_ops.event_handler = NULL;
	adev->hw_ops.get_status = NULL;
	adev->hw_ops.get_versions = zii_pic_niu_get_versions;
	adev->hw_ops.get_boot_source = zii_pic_niu_get_boot_source;
	adev->hw_ops.set_boot_source = zii_pic_niu_set_boot_source;
	adev->hw_ops.reset = zii_pic_niu_reset;
	adev->hw_ops.recovery_reset = NULL;
	adev->hw_ops.read_sensor = zii_pic_niu_hwmon_read_sensor;

	return 0;
}
