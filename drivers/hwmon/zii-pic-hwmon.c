/*
 * zii-pic-hwmon.c - HWMON driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
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

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/zii-pic.h>

static const char * const input_names[] = {
	[ZII_PIC_SENSOR_28V]			= "28V",
	[ZII_PIC_SENSOR_12V]			= "12V",
	[ZII_PIC_SENSOR_5V]			= "5V",
	[ZII_PIC_SENSOR_3V3]			= "3V3",
	[ZII_PIC_SENSOR_RMB_3V3_PMIC]		= "RMB_3V3_PMIC",
	[ZII_PIC_SENSOR_RMB_3V3_MCU]		= "RMB_3V3_MCU",
	[ZII_PIC_SENSOR_RMB_5V_MAIN]		= "RMB_5V_MAIN",
	[ZII_PIC_SENSOR_RMB_12V_MAIN]		= "RMB_12V_MAIN",
	[ZII_PIC_SENSOR_RMB_28V_FIL]		= "RMB_28V_FIL",
	[ZII_PIC_SENSOR_RMB_28V_HOTSWAP]	= "RMB_28V_HOTSWAP",
	[ZII_PIC_SENSOR_DEB_1V8]		= "DEB_1V8",
	[ZII_PIC_SENSOR_DEB_3V3]		= "DEB_3V3",
	[ZII_PIC_SENSOR_DEB_28V_DEB]		= "DEB_28V_DEB",
	[ZII_PIC_SENSOR_DEB_28V_RDU]		= "DEB_28V_RDU",
	[ZII_PIC_SENSOR_TEMPERATURE]		= "TEMPERATURE",
	[ZII_PIC_SENSOR_TEMPERATURE_2]		= "TEMPERATURE_2",
	[ZII_PIC_SENSOR_BACKLIGHT_CURRENT]	= "BACKLIGHT_CURRENT",
	[ZII_PIC_SENSOR_RMB_28V_CURRENT]	= "RMB_28V_CURRENT"
};

struct zii_pic_hwmon {
	struct device		*pic_dev;
	struct attribute	*attrs[];
};

static ssize_t zii_pic_read_sensor(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct zii_pic_hwmon *hwmon = dev_get_drvdata(dev);
	int idx = to_sensor_dev_attr(attr)->index;
	int val = 0;
	int ret;

	pr_debug("%s: enter\n", __func__);

	switch (idx) {
	case ZII_PIC_SENSOR_28V:
	case ZII_PIC_SENSOR_12V:
	case ZII_PIC_SENSOR_5V:
	case ZII_PIC_SENSOR_3V3:
	case ZII_PIC_SENSOR_RMB_3V3_PMIC:
	case ZII_PIC_SENSOR_RMB_3V3_MCU:
	case ZII_PIC_SENSOR_RMB_5V_MAIN:
	case ZII_PIC_SENSOR_RMB_12V_MAIN:
	case ZII_PIC_SENSOR_RMB_28V_FIL:
	case ZII_PIC_SENSOR_RMB_28V_HOTSWAP:
	case ZII_PIC_SENSOR_DEB_1V8:
	case ZII_PIC_SENSOR_DEB_3V3:
	case ZII_PIC_SENSOR_DEB_28V_DEB:
	case ZII_PIC_SENSOR_DEB_28V_RDU:
	case ZII_PIC_SENSOR_TEMPERATURE:
	case ZII_PIC_SENSOR_TEMPERATURE_2:
	case ZII_PIC_SENSOR_BACKLIGHT_CURRENT:
	case ZII_PIC_SENSOR_RMB_28V_CURRENT:
		ret = zii_pic_hwmon_read_sensor(hwmon->pic_dev, idx, &val);
		if (ret)
			return ret;
		break;

	default:
		BUG();
	}

	return sprintf(buf, "%d\n", val);
}

static ssize_t zii_pic_show_label(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	int idx = to_sensor_dev_attr(attr)->index;

	pr_debug("%s: enter\n", __func__);

	return sprintf(buf, "%s\n", input_names[idx]);
}

/* Voltage sensors */
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_28V);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_12V);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_5V);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_3V3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_3V3_PMIC);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_3V3_MCU);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_5V_MAIN);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_12V_MAIN);
static SENSOR_DEVICE_ATTR(in8_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_28V_FIL);
static SENSOR_DEVICE_ATTR(in9_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_28V_HOTSWAP);
static SENSOR_DEVICE_ATTR(in10_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_DEB_1V8);
static SENSOR_DEVICE_ATTR(in11_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_DEB_3V3);
static SENSOR_DEVICE_ATTR(in12_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_DEB_28V_DEB);
static SENSOR_DEVICE_ATTR(in13_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_DEB_28V_RDU);

/* Voltage sensor labels */
static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_28V);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_12V);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_5V);
static SENSOR_DEVICE_ATTR(in3_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_3V3);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_3V3_PMIC);
static SENSOR_DEVICE_ATTR(in5_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_3V3_MCU);
static SENSOR_DEVICE_ATTR(in6_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_5V_MAIN);
static SENSOR_DEVICE_ATTR(in7_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_12V_MAIN);
static SENSOR_DEVICE_ATTR(in8_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_28V_FIL);
static SENSOR_DEVICE_ATTR(in9_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_28V_HOTSWAP);
static SENSOR_DEVICE_ATTR(in10_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_DEB_1V8);
static SENSOR_DEVICE_ATTR(in11_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_DEB_3V3);
static SENSOR_DEVICE_ATTR(in12_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_DEB_28V_DEB);
static SENSOR_DEVICE_ATTR(in13_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_DEB_28V_RDU);

/* Temperature sensors */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_TEMPERATURE);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_TEMPERATURE_2);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_TEMPERATURE);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_TEMPERATURE_2);

/* Current sensors */
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_BACKLIGHT_CURRENT);
static SENSOR_DEVICE_ATTR(curr2_input, S_IRUGO, zii_pic_read_sensor,
			NULL, ZII_PIC_SENSOR_RMB_28V_CURRENT);
static SENSOR_DEVICE_ATTR(curr1_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_BACKLIGHT_CURRENT);
static SENSOR_DEVICE_ATTR(curr2_label, S_IRUGO, zii_pic_show_label,
			NULL, ZII_PIC_SENSOR_RMB_28V_CURRENT);

static struct attribute *zii_pic_hwmon_sensors[] = {
	[ZII_PIC_SENSOR_28V]	= &sensor_dev_attr_in0_input.dev_attr.attr,
	[ZII_PIC_SENSOR_12V]	= &sensor_dev_attr_in1_input.dev_attr.attr,
	[ZII_PIC_SENSOR_5V]	= &sensor_dev_attr_in2_input.dev_attr.attr,
	[ZII_PIC_SENSOR_3V3]	= &sensor_dev_attr_in3_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_3V3_PMIC]	= &sensor_dev_attr_in4_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_3V3_MCU]	= &sensor_dev_attr_in5_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_5V_MAIN]	= &sensor_dev_attr_in6_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_12V_MAIN]	= &sensor_dev_attr_in7_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_28V_FIL]	= &sensor_dev_attr_in8_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_28V_HOTSWAP] = &sensor_dev_attr_in9_input.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_1V8]	= &sensor_dev_attr_in10_input.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_3V3]	= &sensor_dev_attr_in11_input.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_28V_DEB]	= &sensor_dev_attr_in12_input.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_28V_RDU]	= &sensor_dev_attr_in13_input.dev_attr.attr,
	[ZII_PIC_SENSOR_TEMPERATURE]	= &sensor_dev_attr_temp1_input.dev_attr.attr,
	[ZII_PIC_SENSOR_TEMPERATURE_2]	= &sensor_dev_attr_temp2_input.dev_attr.attr,
	[ZII_PIC_SENSOR_BACKLIGHT_CURRENT] = &sensor_dev_attr_curr1_input.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_28V_CURRENT] = &sensor_dev_attr_curr2_input.dev_attr.attr
};

static struct attribute *zii_pic_hwmon_labels[] = {
	[ZII_PIC_SENSOR_28V]	= &sensor_dev_attr_in0_label.dev_attr.attr,
	[ZII_PIC_SENSOR_12V]	= &sensor_dev_attr_in1_label.dev_attr.attr,
	[ZII_PIC_SENSOR_5V]	= &sensor_dev_attr_in2_label.dev_attr.attr,
	[ZII_PIC_SENSOR_3V3]	= &sensor_dev_attr_in3_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_3V3_PMIC]	= &sensor_dev_attr_in4_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_3V3_MCU]	= &sensor_dev_attr_in5_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_5V_MAIN]	= &sensor_dev_attr_in6_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_12V_MAIN]	= &sensor_dev_attr_in7_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_28V_FIL]	= &sensor_dev_attr_in8_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_28V_HOTSWAP] = &sensor_dev_attr_in9_label.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_1V8]	= &sensor_dev_attr_in10_label.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_3V3]	= &sensor_dev_attr_in11_label.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_28V_DEB]	= &sensor_dev_attr_in12_label.dev_attr.attr,
	[ZII_PIC_SENSOR_DEB_28V_RDU]	= &sensor_dev_attr_in13_label.dev_attr.attr,
	[ZII_PIC_SENSOR_TEMPERATURE] = &sensor_dev_attr_temp1_label.dev_attr.attr,
	[ZII_PIC_SENSOR_TEMPERATURE_2] = &sensor_dev_attr_temp2_label.dev_attr.attr,
	[ZII_PIC_SENSOR_BACKLIGHT_CURRENT] = &sensor_dev_attr_curr1_label.dev_attr.attr,
	[ZII_PIC_SENSOR_RMB_28V_CURRENT] = &sensor_dev_attr_curr2_label.dev_attr.attr
};

static struct attribute_group zii_pic_hwmon_group;

__ATTRIBUTE_GROUPS(zii_pic_hwmon);

static int zii_pic_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic_hwmon *hwmon;
	struct device *hwmon_dev;
	struct device_node *node = dev->of_node;
	const char *sensor_name;
	int count, i, attr_idx = 0;

	pr_debug("%s: enter\n", __func__);

	count = of_property_count_strings(node, "sensors");
	if (count <= 0) {
		pr_err("%s: available sensor list is not provided\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: sensors count: %d\n", __func__, count);

	/* add one more for NULL-terminator */
	count++;

	hwmon = devm_kzalloc(dev, sizeof(struct zii_pic_hwmon) +
			count * sizeof(struct attribute *), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->pic_dev = pdev->dev.parent;

	/* prepare attribute group */
	for (i = 0; i < count; i++) {
		enum zii_pic_sensor j;

		if (of_property_read_string_index(node, "sensors",
				i, &sensor_name))
			continue;
		for (j = ZII_PIC_SENSOR_FIRST; j < ZII_PIC_SENSORS_COUNT; j++) {
			if (strcmp(sensor_name, input_names[j]))
				continue;

			pr_debug("%s: added: %s\n", __func__, sensor_name);
			hwmon->attrs[attr_idx++] = zii_pic_hwmon_sensors[j];
			hwmon->attrs[attr_idx++] = zii_pic_hwmon_labels[j];
			break;
		}
	}
	zii_pic_hwmon_group.attrs = hwmon->attrs;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, "zii_pic_hwmon",
					hwmon, zii_pic_hwmon_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct platform_driver zii_pic_hwmon_driver = {
	.probe = zii_pic_hwmon_probe,
	.driver = {
		.name = ZII_PIC_NAME_HWMON,
	},
};

module_platform_driver(zii_pic_hwmon_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC HWMON driver");
MODULE_ALIAS("platform:zii-pic-hwmon");
