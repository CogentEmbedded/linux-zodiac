#ifndef _LINUX_ZII_PIC_H_
#define _LINUX_ZII_PIC_H_

#define ZII_PIC_NAME_WATCHDOG		"pic-watchdog"
#define ZII_PIC_NAME_HWMON		"pic-hwmon"
#define ZII_PIC_NAME_EEPROM		"pic-eeprom"
#define ZII_PIC_NAME_MAIN_EEPROM	"pic-main-eeprom"
#define ZII_PIC_NAME_DDS_EEPROM		"pic-dds-eeprom"

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

	/* Add new command IDs here */

	/* last one to get amount of supported commands */
	ZII_PIC_CMD_COUNT
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

	ZII_PIC_SENSOR_TEMPERATURE,
	ZII_PIC_SENSOR_TEMPERATURE_2,

	ZII_PIC_SENSOR_BACKLIGHT_CURRENT,

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


#endif /* _LINUX_ZII_PIC_H_ */
