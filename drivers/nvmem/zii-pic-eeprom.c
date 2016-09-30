/*
 * zii-pic-eeprom.c - EEPROM driver for Zodiac Inflight Infotainment
 * PIC MCU that is connected via dedicated UART port
 *
 * Copyright (C) 2015 Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
 *
 * Ported to 4.7+ kernels by Nikita Yushchenko <nikita.yoush@cogentembedded.com>
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
#include <linux/zii-pic.h>

struct zii_pic_eeprom {
	struct device *pic_dev;
	enum zii_pic_eeprom_type type;
	struct nvmem_device *nvmem;
};

static int zii_pic_eeprom_reg_read(void *priv, unsigned int offset,
		void *val, size_t bytes)
{
	struct zii_pic_eeprom *adev = priv;

	pr_debug("%s: enter\n", __func__);

	return zii_pic_eeprom_read(adev->pic_dev, adev->type,
			offset, val, bytes);
}

static int zii_pic_eeprom_reg_write(void *priv, unsigned int offset,
		void *val, size_t bytes)
{
	struct zii_pic_eeprom *adev = priv;

	pr_debug("%s: enter\n", __func__);

	return zii_pic_eeprom_write(adev->pic_dev, adev->type,
			offset, val, bytes);
}

static const struct of_device_id zii_pic_eeprom_of_match[] = {
	{ .compatible = "zii,pic-main-eeprom", .data = (void *)MAIN_EEPROM },
	{ .compatible = "zii,pic-dds-eeprom", .data = (void *)DDS_EEPROM },
	{}
};

static int zii_pic_eeprom_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic_eeprom *adev;
	const struct of_device_id *id;
	struct nvmem_config config;

	pr_debug("%s: enter: node %s\n", __func__, dev->of_node->name);

	id = of_match_device(zii_pic_eeprom_of_match, dev);
	if (!id)
		return -ENODEV;

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;
	platform_set_drvdata(pdev, adev);

	adev->pic_dev = dev->parent;
	adev->type = (enum zii_pic_eeprom_type) id->data;

	memset(&config, 0, sizeof(config));
	config.owner = THIS_MODULE;
	config.dev = dev;
	config.priv = adev;
	config.name = zii_pic_eeprom_name(adev->pic_dev, adev->type);
	config.size = zii_pic_eeprom_size(adev->pic_dev, adev->type);
	config.reg_read = zii_pic_eeprom_reg_read;
	config.reg_write = zii_pic_eeprom_reg_write;
	config.word_size = 1;
	config.stride = 1;

	adev->nvmem = nvmem_register(&config);
	if (IS_ERR(adev->nvmem)) {
		pr_err("%s: nvmem register failed (err = %ld)\n",
			__func__, PTR_ERR(adev->nvmem));
		return PTR_ERR(adev->nvmem);
	}

	return 0;
}

static int zii_pic_eeprom_remove(struct platform_device *pdev)
{
	struct zii_pic_eeprom *adev = platform_get_drvdata(pdev);

	return nvmem_unregister(adev->nvmem);
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
