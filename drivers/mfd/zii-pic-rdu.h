#ifndef _LINUX_ZII_PIC_RDU_H_
#define _LINUX_ZII_PIC_RDU_H_

#include "zii-pic-core.h"

/* TODO: fix this struct */
struct pic_rdu_status_info {
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

int zii_pic_rdu_process_status_response(struct zii_pic_mfd *adev,
					u8 *data, u8 size);

int zii_pic_rdu_process_reset_reason(struct zii_pic_mfd *adev,
					u8 *data, u8 size);

extern struct pic_cmd_desc zii_pic_rdu_cmds[ZII_PIC_CMD_COUNT];

#endif /* _LINUX_ZII_PIC_RDU_H_ */
