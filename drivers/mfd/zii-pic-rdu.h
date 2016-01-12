#ifndef _LINUX_ZII_PIC_RDU_H_
#define _LINUX_ZII_PIC_RDU_H_

#include "zii-pic-core.h"

#define		PIC_RDU_EVENT_BUTTON_PRESS	0xE0
#define 	PIC_RDU_RESPONSE_BUTTON_PRESS	0xE1
#define		PIC_RDU_EVENT_ORIENTATION	0xE2
#define 	PIC_RDU_RESPONSE_ORIENTATION	0xE3


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

extern struct pic_cmd_desc zii_pic_rdu_cmds[ZII_PIC_CMD_COUNT];

#endif /* _LINUX_ZII_PIC_RDU_H_ */
