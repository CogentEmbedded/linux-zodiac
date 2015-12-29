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
	{0xA0, 0, zii_pic_rdu_process_status_response},
	/* ZII_PIC_CMD_SW_WDT_SET */
	{0xA1, 3, NULL},
	/* ZII_PIC_CMD_SW_WDT_GET */
	{0,    0, NULL},
	/* ZII_PIC_CMD_PET_WDT    */
	{0xA2, 0, NULL},
	/* ZII_PIC_CMD_RESET  */
	{0xA7, 1, NULL},
	/* ZII_PIC_CMD_HW_RECOVERY_RESET  */
	{0xA7, 2, NULL},
	/* ZII_PIC_CMD_GET_RESET_REASON */
	{0xA8, 0, zii_pic_rdu_process_reset_reason},
	/* ZII_PIC_CMD_GET_28V_READING */
	{0, 0, NULL},
	/* ZII_PIC_CMD_GET_12V_READING */
	{0, 0, NULL},
	/* ZII_PIC_CMD_GET_5V_READING */
	{0, 0, NULL},
	/* ZII_PIC_CMD_GET_3V3_READING */
	{0, 0, NULL},
	/* ZII_PIC_CMD_GET_TEMPERATURE */
	{0, 0, NULL},
	/* ZII_PIC_CMD_GET_FIRMWARE_VERSION */
	{0, 0, NULL},
	/* ZII_PIC_CMD_GET_BOOTLOADER_VERSION */
	{0, 0, NULL},
};

int zii_pic_rdu_process_status_response(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	struct pic_rdu_status_info *status =
			(struct pic_rdu_status_info*)data;

	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != sizeof(*status))
		return -EINVAL;

	memcpy(&adev->bootloader_version, &data[0], 6);
	memcpy(&adev->firmware_version, &data[6], 6);

	return 0;
}

int zii_pic_rdu_process_reset_reason(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

#if 0
	/* bad response, ignore */
	if (size != 1)
		return -EINVAL;
#endif
	adev->reset_reason = *data;

	return 0;
}
