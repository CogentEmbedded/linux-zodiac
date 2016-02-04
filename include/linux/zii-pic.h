#ifndef _LINUX_ZII_PIC_H_
#define _LINUX_ZII_PIC_H_

#define ZII_PIC_NAME_WATCHDOG		"pic-watchdog"
#define ZII_PIC_NAME_HWMON		"pic-hwmon"
#define ZII_PIC_NAME_EEPROM		"pic-eeprom"
#define ZII_PIC_NAME_MAIN_EEPROM	"pic-main-eeprom"
#define ZII_PIC_NAME_DDS_EEPROM		"pic-dds-eeprom"
#define ZII_PIC_NAME_PWRBUTTON		"pic-pwrbutton"
#define ZII_PIC_NAME_BACKLIGHT		"pic-backlight"

#define ZII_PIC_DEFAULT_BAUD_RATE	57600

/* sequential list of all commands on all HW variants */
enum zii_pic_cmd_id {
	ZII_PIC_CMD_GET_STATUS,

	/* Watchdog and reset */
	ZII_PIC_CMD_SW_WDT_SET,
	ZII_PIC_CMD_SW_WDT_GET,
	ZII_PIC_CMD_PET_WDT,
	ZII_PIC_CMD_RESET,
	ZII_PIC_CMD_HW_RECOVERY_RESET,
	ZII_PIC_CMD_GET_RESET_REASON,

	/* HWMON sensors */
	ZII_PIC_CMD_GET_28V_READING,
	ZII_PIC_CMD_GET_12V_READING,
	ZII_PIC_CMD_GET_5V_READING,
	ZII_PIC_CMD_GET_3V3_READING,
	ZII_PIC_CMD_GET_VOLTAGE,
	ZII_PIC_CMD_GET_CURRENT,
	ZII_PIC_CMD_GET_TEMPERATURE,

	/* Main EEPROM */
	ZII_PIC_CMD_EEPROM_READ,
	ZII_PIC_CMD_EEPROM_WRITE,

	/* Versions */
	ZII_PIC_CMD_GET_FIRMWARE_VERSION,
	ZII_PIC_CMD_GET_BOOTLOADER_VERSION,

	/* DDS EEPROM */
	ZII_PIC_CMD_DDS_EEPROM_READ,
	ZII_PIC_CMD_DDS_EEPROM_WRITE,

	/* Boot source */
	ZII_PIC_CMD_GET_BOOT_SOURCE,
	ZII_PIC_CMD_SET_BOOT_SOURCE,

	/* LCD Backlight */
	ZII_PIC_CMD_LCD_BOOT_ENABLE,
	ZII_PIC_CMD_BACKLIGHT,

	/* PIC Bootloader commands */
	ZII_PIC_CMD_JMP_TO_BOOTLOADER,
	ZII_PIC_CMD_BOOTLOADER,

	/* Add new command IDs here */

	/* last one to get amount of supported commands */
	ZII_PIC_CMD_COUNT
};

/* PIC Bootloader commands */
enum zii_pic_bl_cmd_id {
	ZII_PIC_BL_CMD_QUERY_DEVICE = 0xA1,
	/* Not implemented according to ICD */
	/* ZII_PIC_BL_CMD_UNLOCK_CONFIG = 0xA2, */
	ZII_PIC_BL_CMD_ERASE_APP = 0xA3,
	ZII_PIC_BL_CMD_PROGRAM_DEVICE = 0xA4,
	ZII_PIC_BL_CMD_PROGRAM_COMPLETE = 0xA5,
	ZII_PIC_BL_CMD_READ_APP = 0xA6,
	ZII_PIC_BL_CMD_RESET_DEVICE = 0xA7,
	ZII_PIC_BL_CMD_LAUNCH_APP = 0xA8,

	/* TODO: add PCU specific commands if needed */
};

/* Watchdog access API */

#define ZII_PIC_WDT_DEFAULT_TIMEOUT	180
#define ZII_PIC_WDT_MIN_TIMEOUT 	60
/* BUG: inside PIC firmware, it has MAX 300, it does not fit into u8 */
#define ZII_PIC_WDT_MAX_TIMEOUT		255

int zii_pic_watchdog_enable(struct device *pic_dev);
int zii_pic_watchdog_disable(struct device *pic_dev);
int zii_pic_watchdog_get_status(struct device *pic_dev);
int zii_pic_watchdog_ping(struct device *pic_dev);
int zii_pic_watchdog_set_timeout(struct device *pic_dev,
				unsigned int timeout);
void zii_pic_watchdog_reset(struct device *pic_dev, bool hw_recovery);

/* HWMON API */
enum zii_pic_sensor {
	ZII_PIC_SENSOR_FIRST,

	ZII_PIC_SENSOR_28V = ZII_PIC_SENSOR_FIRST,
	ZII_PIC_SENSOR_12V,
	ZII_PIC_SENSOR_5V,
	ZII_PIC_SENSOR_3V3,
	ZII_PIC_SENSOR_RMB_3V3_PMIC,
	ZII_PIC_SENSOR_RMB_3V3_MCU,
	ZII_PIC_SENSOR_RMB_5V_MAIN,
	ZII_PIC_SENSOR_RMB_12V_MAIN,
	ZII_PIC_SENSOR_RMB_28V_FIL,
	ZII_PIC_SENSOR_RMB_28V_HOTSWAP,
	ZII_PIC_SENSOR_DEB_1V8,
	ZII_PIC_SENSOR_DEB_3V3,
	ZII_PIC_SENSOR_DEB_28V_DEB,
	ZII_PIC_SENSOR_DEB_28V_RDU,

	ZII_PIC_SENSOR_TEMPERATURE,
	ZII_PIC_SENSOR_TEMPERATURE_2,

	ZII_PIC_SENSOR_BACKLIGHT_CURRENT,
	ZII_PIC_SENSOR_RMB_28V_CURRENT,

	ZII_PIC_SENSORS_COUNT
};

int zii_pic_hwmon_read_sensor(struct device *pic_dev,
				enum zii_pic_sensor id, int *val);

/* EEPROM API */
enum zii_pic_eeprom_type {
	MAIN_EEPROM,
	DDS_EEPROM
};

int zii_pic_eeprom_read(struct device *pic_dev,
		enum zii_pic_eeprom_type type, u16 reg,
		void *val, size_t val_size);

int zii_pic_eeprom_write(struct device *pic_dev,
		enum zii_pic_eeprom_type type, u16 reg,
		const void *data, size_t size);

/* Input events */
typedef void (*zii_pic_pwrbutton_callback_t)(void *pwrbutton,
		bool state);

int zii_pic_register_pwrbutton_callback(struct device *pic_dev,
		zii_pic_pwrbutton_callback_t callback,
		void *pwrbutton);

/* LCD Backlight */
int zii_pic_backlight_set(struct device *pic_dev, int intensity);

#endif /* _LINUX_ZII_PIC_H_ */
