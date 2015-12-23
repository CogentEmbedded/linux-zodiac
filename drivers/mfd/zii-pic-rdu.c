/*
 * zii-pic-rdu.c - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 * (RDU board specific code)
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

#include <linux/kernel.h>
#include <linux/device.h>

#include "zii-pic-rdu.h"

struct pic_cmd_desc zii_pic_rdu_cmds[ZII_PIC_CMD_COUNT] = {
	/* ZII_PIC_CMD_GET_STATUS */
	{0xA0, 2, zii_pic_rdu_process_status_response},
	/* ZII_PIC_CMD_SW_WDT_SET */
	{0xA1, 5},
	/* ZII_PIC_CMD_SW_WDT_GET */
	{0,    0},
	/* ZII_PIC_CMD_PET_WDT    */
	{0xA2, 2},
	/* ZII_PIC_CMD_RESET  */
	{0xA7, 3},
	/* ZII_PIC_CMD_HW_RECOVERY_RESET  */
	{0xA7, 4},
	/* ZII_PIC_CMD_GET_RESET_REASON */
	{0xA8, 2, zii_pic_rdu_process_reset_reason},
	/* ZII_PIC_CMD_GET_28V_READING */
	{0, 0},
	/* ZII_PIC_CMD_GET_12V_READING */
	{0, 0},
	/* ZII_PIC_CMD_GET_5V_READING */
	{0, 0},
	/* ZII_PIC_CMD_GET_3V3_READING */
	{0, 0},
	/* ZII_PIC_CMD_GET_TEMPERATURE */
	{0, 0},
};

int zii_pic_rdu_process_status_response(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	struct pic_rdu_status_info *status =
			(struct pic_rdu_status_info*)data;

	/* bad response, ignore */
	if (size != sizeof(*status))
		return -EINVAL;

	return 0;
}

int zii_pic_rdu_process_reset_reason(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	static enum zii_pic_reset_reason reason_map[] = {
		ZII_PIC_RESET_POWER_OFF,	/* 0x00 */
		ZII_PIC_RESET_HW_WATCHDOG,	/* 0x01 */
		ZII_PIC_RESET_SW_WATCHDOG,	/* 0x02 */
		ZII_PIC_RESET_VOLTAGE,		/* 0x03 */
		ZII_PIC_RESET_HOST_REQUEST,	/* 0x04 */
		ZII_PIC_RESET_TEMPERATURE,	/* 0x05 */
		ZII_PIC_RESET_USER_REQUEST,	/* 0x06 */
		ZII_PIC_RESET_ILLEGAL_CONFIGURATION_WORD, /* 0x07 */
		ZII_PIC_RESET_ILLEGAL_INSTRUCTION, /* 0x08 */
		ZII_PIC_RESET_ILLEGAL_TRAP,	/* 0x09 */
		ZII_PIC_RESET_UNKNOWN		/* 0x0A */
	};

	/* bad response, ignore */
	if (size != 1 || *data > ARRAY_SIZE(reason_map))
		return -EINVAL;

	adev->reset_reason = reason_map[*data];

	return 0;
}
