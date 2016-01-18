/*
 *  zii-pic-pwrbutton.c - Power button driver on Zodiac Inflight Infotainment
 *  PIC MCU that is connected via dedicated UART port
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
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/zii-pic.h>

static void zii_pic_pwrbutton_report(void *pwrbutton, bool state)
{
	struct input_dev *idev = pwrbutton;

	pr_debug("%s: enter, state: %s\n", __func__, state ? "down" : "up");

	input_report_key(idev, KEY_POWER, state);
	input_sync(idev);
}

static int zii_pic_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *idev;
	int ret;

	pr_debug("%s: enter: dev->parent: %p\n", __func__, pdev->dev.parent);

	if (!pdev->dev.parent)
		return -EINVAL;

	idev = devm_input_allocate_device(&pdev->dev);
	if (!idev)
		return -ENOMEM;

	idev->name = ZII_PIC_NAME_PWRBUTTON;
	idev->dev.parent = &pdev->dev;

	input_set_capability(idev, EV_KEY, KEY_POWER);
	input_set_drvdata(idev, pdev->dev.parent);

	ret = input_register_device(idev);
	if (ret)
		return ret;

	ret = zii_pic_register_pwrbutton_callback(pdev->dev.parent,
			zii_pic_pwrbutton_report,
			idev);
	if (ret)
		return ret;

	return 0;
}


static struct platform_driver zii_pic_pwrbutton_driver = {
	.probe		= zii_pic_pwrbutton_probe,
	.driver		= {
		.name	= ZII_PIC_NAME_PWRBUTTON,
	},
};
module_platform_driver(zii_pic_pwrbutton_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC Power Button driver");
MODULE_ALIAS("platform:zii-pic-pwrbutton");
