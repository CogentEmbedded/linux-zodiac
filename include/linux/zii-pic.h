#ifndef _LINUX_ZII_PIC_H_
#define _LINUX_ZII_PIC_H_

#define ZII_PIC_DRVNAME_WATCHDOG	"pic-watchdog"
#define ZII_PIC_DRVNAME_HWMON		"pic-hwmon"

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

	/* Board specific variants */
	ZII_PIC_CMD_GET_FIRMWARE_VERSION,
	ZII_PIC_CMD_GET_BOOTLOADER_VERSION,

	/* last one to get amount of supported commands */
	ZII_PIC_CMD_COUNT
};

enum zii_pic_event_id {
	ZII_PIC_EVENT_X,

	/* last one to get amount of supported events */
	ZII_PIC_EVENT_COUNT
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

	ZII_PIC_SENSORS_COUNT
};

int zii_pic_hwmon_read_sensor(struct device *pic_dev,
				enum zii_pic_sensor id, int *val);


#endif /* _LINUX_ZII_PIC_H_ */
