/*
 * zii-pic-rdu.c - Multifunction core driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 * (RDU board specific code)
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

/* #define DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/nvmem-consumer.h>

#include "zii-pic-niu.h"
#include "zii-pic-rdu.h"

#define PIC_RDU_ORIENTATION_MASK	0x7
#define PIC_RDU_STOWAGE_MASK		0x80

/* Main RDU EEPROM has same command/response structure */
#define zii_pic_rdu_process_eeprom_read		zii_pic_niu_process_eeprom_read
#define zii_pic_rdu_process_eeprom_write	zii_pic_niu_process_eeprom_write


struct zii_pic_cmd_desc zii_pic_rdu_cmds[ZII_PIC_CMD_COUNT] = {
	/* ZII_PIC_CMD_GET_STATUS */
	{0xA0, 0, zii_pic_rdu_process_status_response},
	/* ZII_PIC_CMD_SW_WDT_SET */
	{0xA1, 3, NULL},
	/* ZII_PIC_CMD_SW_WDT_GET */
	{0,    0, NULL},
	/* ZII_PIC_CMD_PET_WDT    */
	{0xA2, 0, NULL},
	/* ZII_PIC_CMD_RESET  */
	{0xA7, 1, NULL},
	/* ZII_PIC_CMD_HW_RECOVERY_RESET  */
	{0xA7, 2, NULL},
	/* ZII_PIC_CMD_GET_RESET_REASON */
	{0xA8, 0, zii_pic_rdu_process_reset_reason},
	/* ZII_PIC_CMD_GET_28V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_12V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_5V_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_3V3_READING */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_TEMPERATURE */
	{0,    0, NULL},
	/* ZII_PIC_CMD_EEPROM_READ */
	{0xA4, 3, zii_pic_rdu_process_eeprom_read},
	/* ZII_PIC_CMD_EEPROM_WRITE */
	{0xA4, 35, zii_pic_rdu_process_eeprom_write},
	/* ZII_PIC_CMD_GET_FIRMWARE_VERSION */
	{0,    0, NULL},
	/* ZII_PIC_CMD_GET_BOOTLOADER_VERSION */
	{0,    0, NULL},
	/* ZII_PIC_CMD_DDS_EEPROM_READ */
	{0xA3, 2, zii_pic_rdu_process_dds_eeprom_read},
	/* ZII_PIC_CMD_DDS_EEPROM_WRITE */
	{0xA3, 34, zii_pic_rdu_process_dds_eeprom_write},
	/* ZII_PIC_CMD_GET_BOOT_SOURCE */
	{0,    0, NULL},
	/* ZII_PIC_CMD_SET_BOOT_SOURCE */
	{0,    0, NULL},
};

int zii_pic_rdu_process_status_response(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	struct zii_pic_rdu_status_info *status =
			(struct zii_pic_rdu_status_info*)data;

	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != sizeof(*status))
		return -EINVAL;

	memcpy(&adev->bootloader_version, &data[0], 6);
	memcpy(&adev->firmware_version, &data[6], 6);

	adev->sensor_28v = zii_pic_f88_to_int(status->v);

	adev->temperature = status->t1 * 500;
	adev->temperature_2 = status->t2 * 500;

	pr_debug("%s: backlight state: %s, brightness: %d\n", __func__,
		(status->bk[0] & 0x80) ? "On" : "Off", status->bk[0] & 0x7f);
	adev->backlight_current = status->bk[1] | status->bk[2] << 8;

	adev->boot_source = (status->gs >> 2) & 0x03;
	return 0;
}

int zii_pic_rdu_process_reset_reason(struct zii_pic_mfd *adev,
		u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	adev->reset_reason = *data;

	return 0;
}

int zii_pic_rdu_process_dds_eeprom_read(struct zii_pic_mfd *adev,
				u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2 + ZII_PIC_EEPROM_PAGE_SIZE)
		return -EINVAL;

	/* check operation status */
	if (!data[1])
		return -EIO;

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "DDS EEPROM data: ", DUMP_PREFIX_OFFSET,
			16, 1, &data[2], ZII_PIC_EEPROM_PAGE_SIZE, true);
#endif

	memcpy(adev->eeprom_page, &data[2], ZII_PIC_EEPROM_PAGE_SIZE);

	return 0;
}

int zii_pic_rdu_process_dds_eeprom_write(struct zii_pic_mfd *adev,
				u8 *data, u8 size)
{
	pr_debug("%s: enter\n", __func__);

	/* bad response, ignore */
	if (size != 2)
		return -EINVAL;

	/* check operation status */
	if (!data[1])
		return -EIO;

	return 0;
}

void zii_pic_rdu_event_handler(struct zii_pic_mfd *adev,
		struct n_mcu_cmd *event)
{
	pr_debug("%s: enter\n", __func__);

	switch (event->data[0]) {
	case ZII_PIC_RDU_EVENT_BUTTON_PRESS:
		event->data[0] = ZII_PIC_RDU_RESPONSE_BUTTON_PRESS;
		if (adev->pwrbutton_event)
			adev->pwrbutton_event(adev->pwrbutton,
				event->data[2]);
		break;

	case ZII_PIC_RDU_EVENT_ORIENTATION:
		event->data[0] = ZII_PIC_RDU_RESPONSE_ORIENTATION;
		adev->orientation = event->data[2] & PIC_RDU_ORIENTATION_MASK;
		adev->stowed = !(event->data[2] & PIC_RDU_STOWAGE_MASK);
		break;
	default:
		BUG();
	}

	event->size = 2;

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "event response data: ", DUMP_PREFIX_OFFSET,
			16, 1, event->data, event->size, true);
#endif

	if (adev->mcu_ops.event_response)
		adev->mcu_ops.event_response(event);
}


int zii_pic_rdu_hwmon_read_sensor(struct zii_pic_mfd *adev,
			enum zii_pic_sensor id, int *val)
{
	int ret;

	ret = zii_pic_mcu_cmd(adev, ZII_PIC_CMD_GET_STATUS, NULL, 0);
	if (ret)
		return ret;

	switch (id) {
	case ZII_PIC_SENSOR_28V:
		*val = adev->sensor_28v;
		break;

	case ZII_PIC_SENSOR_12V:
		*val = adev->sensor_12v;
		break;

	case ZII_PIC_SENSOR_5V:
		*val = adev->sensor_5v;
		break;

	case ZII_PIC_SENSOR_3V3:
		*val = adev->sensor_3v3;
		break;

	case ZII_PIC_SENSOR_TEMPERATURE:
		*val = adev->temperature;
		break;

	case ZII_PIC_SENSOR_TEMPERATURE_2:
		*val = adev->temperature_2;
		break;

	case ZII_PIC_SENSOR_BACKLIGHT_CURRENT:
		*val = adev->backlight_current;
		break;

	default:
		BUG();
	}

	return ret;
}

int zii_pic_rdu_set_boot_source(struct zii_pic_mfd *adev,
		enum zii_pic_boot_source source)
{
	struct nvmem_cell *cell;
	void *value;
	size_t cell_len;
	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	/* On RDU it is accessible only via EEPROM */
	cell = devm_nvmem_cell_get(adev->dev, "boot_source");
	if (IS_ERR(cell)) {
		pr_warn("%s: failed to get boot source NVMEM cell\n", __func__);
		return PTR_ERR(cell);
	}

	value = nvmem_cell_read(cell, &cell_len);
	if (IS_ERR(value)) {
		pr_warn("%s: failed to get boot source from EEPROM\n", __func__);
		ret = PTR_ERR(cell);
		goto out;
	}

	if (*(enum zii_pic_boot_source*)value != source) {
		u8 data =  source;
		pr_debug("%s: updating boot source from %d to %d\n", __func__,
				*(enum zii_pic_boot_source*)value, source);
		ret = nvmem_cell_write(cell, &data, sizeof(data));
		if (ret < 0)
			pr_err("%s: failed to save boot source to EEPROM\n",
				__func__);
		else {
			pr_debug("%s: bytes written to cell: %d\n", __func__, ret);
			ret = 0;
		}
	}

out:
	devm_nvmem_cell_put(adev->dev, cell);

	return ret;
}

