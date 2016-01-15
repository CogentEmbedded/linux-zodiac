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

/* #define DEBUG */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/zii-pic.h>

struct zii_pic_eeprom {
	struct device *pic_dev;
	struct nvmem_device *nvmem;
	enum zii_pic_eeprom_type eeprom_type;
};

struct zii_pic_eeprom_info {
	const char 			*name;
	enum zii_pic_eeprom_type	eeprom_type;
	struct regmap_config*		regmap_config;
};

static struct nvmem_config eeprom_config = {
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

	return zii_pic_eeprom_read(adev->pic_dev, adev->eeprom_type,
			*(u16*)reg, val, val_size);
}

static int zii_pic_eeprom_bus_write(void *context,
		const void *data, size_t count)
{
	struct device *dev = context;
	struct zii_pic_eeprom *adev = dev_get_drvdata(dev);
	const u16 *p = data;

	pr_debug("%s: enter\n", __func__);

	return zii_pic_eeprom_write(adev->pic_dev,  adev->eeprom_type,
			*p, p+1, count - sizeof(*p));
}

static struct regmap_bus zii_pic_eeprom_bus = {
	.read = zii_pic_eeprom_bus_read,
	.write = zii_pic_eeprom_bus_write,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static struct regmap_config zii_pic_main_eeprom_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0x3FFF,
};

static struct regmap_config zii_pic_dds_eeprom_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0x1FFF,
};

static const struct zii_pic_eeprom_info zii_pic_main_eeprom_info = {
	.name = ZII_PIC_NAME_MAIN_EEPROM,
	.eeprom_type = MAIN_EEPROM,
	.regmap_config = &zii_pic_main_eeprom_regmap_config,
};

static const struct zii_pic_eeprom_info zii_pic_dds_eeprom_info = {
	.name = ZII_PIC_NAME_DDS_EEPROM,
	.eeprom_type = DDS_EEPROM,
	.regmap_config = &zii_pic_dds_eeprom_regmap_config,
};

static const struct of_device_id zii_pic_eeprom_of_match[] = {
	{ .compatible = "zii,pic-main-eeprom", .data = &zii_pic_main_eeprom_info},
	{ .compatible = "zii,pic-dds-eeprom", .data = &zii_pic_dds_eeprom_info},
	{}
};

static int zii_pic_eeprom_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic_eeprom *adev;
	struct regmap *regmap;
	const struct of_device_id *id;
	struct zii_pic_eeprom_info *info;

	pr_debug("%s: enter: node %s\n", __func__, dev->of_node->name);

	id = of_match_device(zii_pic_eeprom_of_match, dev);
	if (!id)
		return -ENODEV;

	info = (struct zii_pic_eeprom_info *)id->data;

	regmap = devm_regmap_init(dev, &zii_pic_eeprom_bus,
			dev, info->regmap_config);
	if (IS_ERR(regmap)) {
		pr_err("%s: regmap init failed (err = %ld)\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->pic_dev = dev->parent;
	adev->eeprom_type = info->eeprom_type;

	eeprom_config.name = info->name;
	eeprom_config.dev = dev;
	adev->nvmem = nvmem_register(&eeprom_config);
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
		.name = ZII_PIC_NAME_EEPROM,
		.of_match_table = zii_pic_eeprom_of_match,
	},
};
module_platform_driver(zii_pic_eeprom_driver);

MODULE_DEVICE_TABLE(of, zii_pic_eeprom_of_match);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC EEPROM driver");
