/*
 * zii-pic-niu.h - Multifunction core driver for Zodiac Inflight Infotainment
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
#ifndef _LINUX_ZII_PIC_NIU_H_
#define _LINUX_ZII_PIC_NIU_H_

#include "zii-pic-core.h"

struct zii_pic_niu_status_info {
	u8	bl_part_num_hw;
	u16	bl_part_num_major_ver;
	u8	bl_part_num_minor_ver;
	u8	bl_part_num_letter_1;
	u8	bl_part_num_letter_2;
	u8	fw_part_num_hw;
	u16	fw_part_num_major_ver;
	u8	fw_part_num_minor_ver;
	u8	fw_part_num_letter_1;
	u8	fw_part_num_letter_2;
	u8	niu_flag_num;
	u8	pic_flag_num;
	u32	etc_reading;
	u16	temp_reading;
	u8	host_gpio_status;
	u16	voltage_28_reading;
	u8	i2c_device_status;
	u8	general_status;
}__packed;

int zii_pic_niu_process_status_response(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_watchdog_state(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_reset_reason(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_28v(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_12v(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_5v(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_3v3(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_temperature(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_firmware_version(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_bootloader_version(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_eeprom_read(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_eeprom_write(struct zii_pic_mfd *adev,
				u8 *data, u8 size);
int zii_pic_niu_process_get_boot_source(struct zii_pic_mfd *adev,
			u8 *data, u8 size);

int zii_pic_niu_hwmon_read_sensor(struct zii_pic_mfd *adev,
			enum zii_pic_sensor id, int *val);

extern struct zii_pic_cmd_desc zii_pic_niu_cmds[ZII_PIC_CMD_COUNT];

#endif /* _LINUX_ZII_PIC_NIU_H_ */
