/*
 * INTEL HEX format parser
 *
 * Code extracted from drivers/media/usb/as102/as102_fw.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ctype.h>
#include <linux/kernel.h>

#include <linux/intel-hex.h>


static unsigned char atohx(unsigned char *dst, char *src)
{
	unsigned char value = 0;

	char msb = tolower(*src) - '0';
	char lsb = tolower(*(src + 1)) - '0';

	if (msb > 9)
		msb -= 7;
	if (lsb > 9)
		lsb -= 7;

	*dst = value = ((msb & 0xF) << 4) | (lsb & 0xF);
	return value;
}

/*
 * Parse INTEL HEX firmware file to extract address and data.
 */
int parse_hex_line(unsigned char *in_data, unsigned char *addr,
		unsigned char *out_data, unsigned char *out_length,
		unsigned char *addr_has_changed) {

	int count = 0;
	unsigned char *src, dst;

	if (*in_data++ != ':') {
		pr_err("invalid firmware file\n");
		return -EFAULT;
	}

	/* locate end of line */
	for (src = in_data; *src != '\n'; src += 2) {
		atohx(&dst, src);
		/* parse line to split addr / data */
		switch (count) {
		case 0:
			*out_length = dst;
			break;
		case 1:
			addr[2] = dst;
			break;
		case 2:
			addr[3] = dst;
			break;
		case 3:
			/* check if data is an address */
			if (dst == 0x04)
				*addr_has_changed = 1;
			else
				*addr_has_changed = 0;
			break;
		case  4:
		case  5:
			if (*addr_has_changed)
				addr[(count - 4)] = dst;
			else
				out_data[(count - 4)] = dst;
			break;
		default:
			out_data[(count - 4)] = dst;
			break;
		}
		count++;
	}

	/* return read value + ':' + '\n' */
	return (count * 2) + 2;
}
