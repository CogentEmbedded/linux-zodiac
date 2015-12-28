#ifndef _LINUX_ZII_PIC_CORE_H_
#define _LINUX_ZII_PIC_CORE_H_

#include <linux/tty.h>
#include <linux/zii-pic.h>
#include <linux/n_mcu.h>

enum pic_state {
	PIC_STATE_UNKNOWN,
	PIC_STATE_OPENED,
	PIC_STATE_CONFIGURED
};

enum pic_hw_id {
	PIC_HW_ID_NIU,
	PIC_HW_ID_MEZZ,
	PIC_HW_ID_ESB,
	PIC_HW_ID_RDU
};

struct pic_version {
	u8	hw;
	u16	major;
	u8	minor;
	u8	letter_1;
	u8	letter_2;
} __packed;

/*
 * @cmd_seqn: PIC command sequence number
 */
struct pic_cmd_desc;
struct zii_pic_mfd {
	struct device			*dev;
	const struct attribute_group	**groups;
	enum pic_hw_id			hw_id;
	u8				checksum_size;

	atomic_t			cmd_seqn;
	struct pic_cmd_desc		*cmd;
	struct n_mcu_ops		mcu_ops;

	enum pic_state			state;
	long				port_fd;
	struct delayed_work		state_work;
	struct tty_struct		*tty;
	speed_t				baud;

	/* cached watchdog state */
	u8				watchdog_timeout;
	u8				watchdog_enabled;
	u8				reset_reason;

	/* sensors, missing on RDU? */
	int				sensor_28v;
	int				sensor_12v;
	int				sensor_5v;
	int				sensor_3v3;
	int				temperature;

	struct pic_version		bootloader_version;
	struct pic_version		firmware_version;

	int (*uart_open)(struct tty_struct * tty, struct file * filp);
};

typedef int (*zii_pic_handler_t)(struct zii_pic_mfd *adev,
					u8 *data, u8 size);
struct pic_cmd_desc {
	u8			cmd_id;
	u8			data_len; /* without header (cmd id, ack) and CRC */
	zii_pic_handler_t	response_handler;
};

/* common response handlers */

#endif /* _LINUX_ZII_PIC_CORE_H_ */
