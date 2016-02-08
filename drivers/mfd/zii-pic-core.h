/*
 * zii-pic-core.h - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
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
#ifndef _LINUX_ZII_PIC_CORE_H_
#define _LINUX_ZII_PIC_CORE_H_

#include <linux/tty.h>
#include <linux/zii-pic.h>
#include <linux/n_mcu.h>

#define ZII_PIC_MCU_DEFAULT_CMD_TIMEOUT	100
#define ZII_PIC_EEPROM_PAGE_SIZE	32
#define ZII_PIC_BL_DATA_SIZE		56

enum zii_pic_state {
	PIC_STATE_UNKNOWN,
	PIC_STATE_OPENED,
	PIC_STATE_CONFIGURED
};

enum zii_pic_hw_id {
	PIC_HW_ID_NIU,
	PIC_HW_ID_MEZZ,
	PIC_HW_ID_ESB,
	PIC_HW_ID_RDU,
	PIC_HW_ID_RDU2
};

enum zii_pic_boot_source {
	PIC_BOOT_SRC_FIRST,
	PIC_BOOT_SRC_SD = PIC_BOOT_SRC_FIRST,
	PIC_BOOT_SRC_EMMC,
	PIC_BOOT_SRC_NOR,
	PIC_BOOT_SRC_LAST = PIC_BOOT_SRC_NOR
};

struct zii_pic_version {
	u8	hw;
	u16	major;
	u8	minor;
	u8	letter_1;
	u8	letter_2;
} __packed;

struct zii_pic_mfd;
typedef int (*zii_pic_handler_t)(struct zii_pic_mfd *adev,
					u8 *data, u8 size);
/* */
struct zii_pic_cmd_desc {
	u8			cmd_id;
	u8			data_len; /* without header (cmd id, ack) and CRC */
	zii_pic_handler_t	response_handler;
};

/* Interface to HW-specific implementation for several features */
struct zii_pic_hw_ops {
	void (*event_handler)(struct zii_pic_mfd *adev,
			struct n_mcu_cmd *event);
	int (*get_status)(struct zii_pic_mfd *adev);
	int (*get_versions)(struct zii_pic_mfd *adev);

	int (*get_boot_source)(struct zii_pic_mfd *adev);
	int (*set_boot_source)(struct zii_pic_mfd *adev,
			enum zii_pic_boot_source boot_src);

	int (*reset)(struct zii_pic_mfd *adev);
	int (*recovery_reset)(struct zii_pic_mfd *adev);
	int (*read_sensor)(struct zii_pic_mfd *adev,
			enum zii_pic_sensor id, int *val);
};

/*
 * @cmd_seqn: PIC command sequence number
 */
struct zii_pic_mfd {
	struct device			*dev;
	const struct attribute_group	**groups;
	enum zii_pic_hw_id		hw_id;
	u8				checksum_size;

	atomic_t			cmd_seqn;
	struct zii_pic_cmd_desc		*cmd;
	struct zii_pic_hw_ops		hw_ops;
	struct n_mcu_ops		mcu_ops;

	enum zii_pic_state		state;
	long				port_fd;
	struct delayed_work		state_work;
	struct tty_struct		*tty;
	speed_t				baud;

	u8				watchdog_timeout;
	u8				watchdog_enabled;
	u8				reset_reason;
	u8				boot_source;

	u8				orientation;
	bool				stowed;

	int				sensor_28v;
	int				sensor_12v;
	int				sensor_5v;
	int				sensor_3v3;
	int				sensor_temperature;
	int				sensor_temperature_2;
	int				sensor_current;
	int				sensor_voltage;

	struct zii_pic_version		bootloader_version;
	struct zii_pic_version		firmware_version;
	u32				firmware_start_address;
	u32				firmware_end_address;

	u8				eeprom_page[ZII_PIC_EEPROM_PAGE_SIZE];

	int (*uart_open)(struct tty_struct * tty, struct file * filp);
	int (*init)(struct zii_pic_mfd *adev);

	zii_pic_pwrbutton_callback_t	pwrbutton_event;
	void				*pwrbutton;

	atomic_t			fw_update_state;
	int				fw_update_progress;
};

/* Convert 8.8 fixed point value multiplied by 1000 to integer.
 * Used to represent sensor values as mV and mA
 */
static inline int zii_pic_f88_to_int(u8 *data)
{
	return data[1] * 1000 + (data[0] * 1000 >> 8);
}

int zii_pic_mcu_cmd(struct zii_pic_mfd *adev,
		enum zii_pic_cmd_id id, const u8 * const data, u8 data_size);
int zii_pic_mcu_cmd_no_response(struct zii_pic_mfd *adev,
		enum zii_pic_cmd_id id, const u8 * const data, u8 data_size);


#endif /* _LINUX_ZII_PIC_CORE_H_ */
