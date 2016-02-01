/*
 * zii-pic-rdu.h - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 * (RDU board specific code)
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

#ifndef _LINUX_ZII_PIC_RDU_H_
#define _LINUX_ZII_PIC_RDU_H_

#include "zii-pic-core.h"

#define		ZII_PIC_RDU_EVENT_BUTTON_PRESS		0xE0
#define 	ZII_PIC_RDU_RESPONSE_BUTTON_PRESS	0xE1
#define		ZII_PIC_RDU_EVENT_ORIENTATION		0xE2
#define 	ZII_PIC_RDU_RESPONSE_ORIENTATION	0xE3

struct zii_pic_rdu_status_info_common {
	struct zii_pic_version bl;
	struct zii_pic_version fw;

	u16	rf;
	u16	df;
	u8	pf;
	u8	po;
	u32	ec;
	u16	t1;
	u16	t2;
	u8	bk[3];
	u8	cs;
	u8	hs;
	u8	v[2];
	u8	ib;
	u8	ps;
	u8	gs;
} __packed;

struct zii_pic_rdu_status_info {
	struct zii_pic_rdu_status_info_common common;

	/* FIXME: On RDU with FW 2.3.0.bx size of status structure is one
	 * byte less, need to confirm which leftover byte is missing */
	u8	wf;
	u8	pl;
	u8	dt;
	u8	pp;

}__packed;

int zii_pic_rdu_process_status_response(struct zii_pic_mfd *adev,
					u8 *data, u8 size);

int zii_pic_rdu_process_reset_reason(struct zii_pic_mfd *adev,
					u8 *data, u8 size);

int zii_pic_rdu_process_dds_eeprom_read(struct zii_pic_mfd *adev,
				u8 *data, u8 size);

int zii_pic_rdu_process_dds_eeprom_write(struct zii_pic_mfd *adev,
				u8 *data, u8 size);

void zii_pic_rdu_event_handler(struct zii_pic_mfd *adev,
		struct n_mcu_cmd *event);

int zii_pic_rdu_hwmon_read_sensor(struct zii_pic_mfd *adev,
			enum zii_pic_sensor id, int *val);

int zii_pic_rdu_set_boot_source(struct zii_pic_mfd *adev,
		enum zii_pic_boot_source source);

int zii_pic_rdu_init(struct zii_pic_mfd *adev);

int zii_pic_rdu_recovery_reset(struct zii_pic_mfd *adev);

extern struct zii_pic_cmd_desc zii_pic_rdu_cmds[ZII_PIC_CMD_COUNT];

#endif /* _LINUX_ZII_PIC_RDU_H_ */
