/*
 * zii-pic-eeprom.c - EEPROM driver for Zodiac Inflight Infotainment
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

#define DEBUG

#include <linux/device.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/zii-pic.h>

struct zii_pic_eeprom {
	struct device *pic_dev;
	struct nvmem_device *nvmem;
};

static struct nvmem_config econfig = {
	.name = "zii-pic-eeprom",
	.owner = THIS_MODULE,
};

static int zii_pic_eeprom_bus_read(void *context,
		const void *reg, size_t reg_size,
		void *val, size_t val_size)
{
	struct device *dev = context;
	struct zii_pic_eeprom *adev = dev_get_drvdata(dev);

	pr_debug("%s: enter\n", __func__);

	if (reg_size != 2)
		return -ENOTSUPP;

	return zii_pic_eeprom_read(adev->pic_dev, *(u16*)reg, val, val_size);
}

static int zii_pic_eeprom_bus_write(void *context,
		const void *data, size_t count)
{
	struct device *dev = context;
	struct zii_pic_eeprom *adev = dev_get_drvdata(dev);
	const u16 *p = data;

	pr_debug("%s: enter\n", __func__);

	return zii_pic_eeprom_write(adev->pic_dev, *p, p+1, count - sizeof(*p));
}

static struct regmap_bus zii_pic_eeprom_bus = {
	.read = zii_pic_eeprom_bus_read,
	.write = zii_pic_eeprom_bus_write,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static struct regmap_config zii_pic_eeprom_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0x3FFF,
};

static int zii_pic_eeprom_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	struct zii_pic_eeprom *adev;

	pr_debug("%s: enter\n", __func__);

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	regmap = devm_regmap_init(dev, &zii_pic_eeprom_bus, dev,
				  &zii_pic_eeprom_regmap_config);
	if (IS_ERR(regmap)) {
		pr_err("%s: regmap init failed (err = %ld)\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	adev->pic_dev = pdev->dev.parent;

	econfig.dev = dev;
	adev->nvmem = nvmem_register(&econfig);
	if (IS_ERR(adev->nvmem)) {
		pr_err("%s: nvmem register failed (err = %ld)\n",
			__func__, PTR_ERR(adev->nvmem));
		return PTR_ERR(adev->nvmem);
	}

	platform_set_drvdata(pdev, adev);

	return 0;
}

static int zii_pic_eeprom_remove(struct platform_device *pdev)
{
	struct nvmem_device *nvmem = platform_get_drvdata(pdev);

	return nvmem_unregister(nvmem);
}

static struct platform_driver zii_pic_eeprom_driver = {
	.probe = zii_pic_eeprom_probe,
	.remove = zii_pic_eeprom_remove,
	.driver = {
		.name = ZII_PIC_DRVNAME_EEPROM,
	},
};
module_platform_driver(zii_pic_eeprom_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC EEPROM driver");
