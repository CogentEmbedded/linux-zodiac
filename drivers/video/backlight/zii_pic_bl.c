/*
 * zii_pic_bl.c - LCD Backlight driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 *
 * Copyright (C) 2016 Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
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

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/zii-pic.h>

struct zii_pic_backlight {
	struct device		*pic_dev;
};

static int zii_pic_bl_update_status(struct backlight_device *bd)
{
	struct zii_pic_backlight *adev = dev_get_drvdata(&bd->dev);
	int intensity = bd->props.brightness;

	pr_debug("%s: enter, intensity: %d, power: %d\n", __func__, intensity, bd->props.power);

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;

	return zii_pic_backlight_set(adev->pic_dev, intensity);
}

static const struct backlight_ops zii_pic_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status  = zii_pic_bl_update_status,
};

static struct backlight_properties zii_pic_bl_props = {
	.type = BACKLIGHT_RAW,
	.max_brightness = 100,
	.brightness = 50,
};

static const struct of_device_id zii_pic_bl_of_match[] = {
	{ .compatible = "zii,pic-backlight" },
	{}
};

static int zii_pic_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
	struct zii_pic_backlight *adev;

	pr_debug("%s: enter: dev->parent: %p\n", __func__, pdev->dev.parent);

	if (!pdev->dev.of_node)
		return -ENODEV;

	if (!pdev->dev.parent)
		return -EINVAL;

	adev = devm_kzalloc(&pdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->pic_dev = pdev->dev.parent;

	bd = devm_backlight_device_register(&pdev->dev, ZII_PIC_NAME_BACKLIGHT,
					    &pdev->dev, adev, &zii_pic_bl_ops,
					    &zii_pic_bl_props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);
	backlight_update_status(bd);

	return 0;
}

static struct platform_driver zii_pic_backlight_driver = {
	.probe		= zii_pic_backlight_probe,
	.driver		= {
		.name	= ZII_PIC_NAME_BACKLIGHT,
		.of_match_table = zii_pic_bl_of_match,
	},
};
module_platform_driver(zii_pic_backlight_driver);

MODULE_DEVICE_TABLE(of, zii_pic_bl_of_match);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC LCD Backlight driver");
MODULE_ALIAS("platform:zii-pic-backlight");

