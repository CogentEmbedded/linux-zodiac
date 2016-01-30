/*
 *  zii-pic-mezz.h - Multifunction core driver for Zodiac Inflight Infotainment
 *  PIC MCU that is connected via dedicated UART port
 * (MEZZ board specific code)
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

#ifndef _LINUX_ZII_PIC_MEZZ_H_
#define _LINUX_ZII_PIC_MEZZ_H_

#include "zii-pic-core.h"

int zii_pic_mezz_init(struct zii_pic_mfd *adev);

extern struct zii_pic_cmd_desc zii_pic_mezz_cmds[ZII_PIC_CMD_COUNT];

#endif /* _LINUX_ZII_PIC_MEZZ_H_ */
