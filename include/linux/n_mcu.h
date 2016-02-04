/*
 *  n_mcu.h - microcontroller unit communication line discipline
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
#ifndef _LINUX_N_MCU_H_
#define _LINUX_N_MCU_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define N_MCU_IO			'M'

#define N_MCU_CHECKSUM_NONE		0 /* Checksum is not used */
#define N_MCU_CHECKSUM_CCITT_FALSE	1 /* CCITT-FALSE CRC16 */
#define N_MCU_CHECKSUM_8B2C		2 /* 8-bit 2's complement */

#define N_MCU_MAX_CMD_SIZE		64
struct n_mcu_cmd {
	size_t	size;
	u32	timeout;
	__u8	data[N_MCU_MAX_CMD_SIZE];
	/* Nasty workaround for misbehaving firmware:
	 * bootloader query command returns more than 64 bytes as response */
	__u8	guard[4];
};

typedef void (*event_callback_t)(void *, struct n_mcu_cmd*);

struct n_mcu_ops {
	int (*cmd)(struct n_mcu_cmd *);
	int (*cmd_no_response)(struct n_mcu_cmd *);
	int (*event_response)(struct n_mcu_cmd *);

	event_callback_t event;
	void *callback_cookie;
};

#define N_MCU_SET_CHECKSUM_TYPE	_IOW (N_MCU_IO, 0, unsigned int)
#define N_MCU_CONFIGURE_OPS	_IOWR(N_MCU_IO, 1, struct n_mcu_ops*)

#endif /* _LINUX_N_MCU_H_ */
