#ifndef _LINUX_N_MCU_H_
#define _LINUX_N_MCU_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define N_MCU_IO 'M'

#define N_MCU_CHECKSUM_NONE	0 /* Checksum is not calculated */
#define N_MCU_CHECKSUM_CRC16	1 /* CCITT CRC16 */
#define N_MCU_CHECKSUM_8B2C	2 /* 8-bit 2's complement */

#define N_MCU_MAX_CMD_SIZE	64
struct n_mcu_cmd {
	size_t	size;
	__u8	data[N_MCU_MAX_CMD_SIZE];
} __packed;

struct n_mcu_ops {
	int (*cmd)(struct n_mcu_cmd *);
	int (*cmd_no_response)(struct n_mcu_cmd *);
	void (*event)(struct n_mcu_cmd *);
};

#define N_MCU_SET_CHECKSUM_TYPE	_IOW (N_MCU_IO, 0, unsigned int)
#define N_MCU_CONFIGURE_OPS	_IOWR(N_MCU_IO, 1, struct n_mcu_ops*)

#endif /* _LINUX_N_MCU_H_ */
