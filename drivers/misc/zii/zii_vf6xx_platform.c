/*
 * Copyright 2016 Zodiac Inflight Innovations.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/gpio/consumer.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>



struct zii_vf6xx_platform_data {
	struct gpio_desc *gpiod_bits[4];
	unsigned plat_value;
};

enum zii_vf6xx_platform_types {
	ZII_PLATFORM_VF610_DEV  = 0x01,
	ZII_PLATFORM_VF610_AIB  = 0x02,
	ZII_PLATFORM_VF610_SPU3 = 0x03,
	ZII_PLATFORM_VF610_CFU1 = 0x04
};

struct zii_vf6xx_platform_data zii_vf6xx_plat_data;
struct proc_dir_entry *zii_vf6xx_proc_dir;


static int
board_type_show(struct seq_file *m, void *v)
{
	char *type_str;
	switch(zii_vf6xx_plat_data.plat_value) {
	case ZII_PLATFORM_VF610_DEV:
		/* Vybrid Dev Board */
		type_str = "VYBRID_DEV";
		break;
	case ZII_PLATFORM_VF610_AIB:
		/* SCU4 AIB */
		type_str = "SCU4 AIB";
		break;
	case ZII_PLATFORM_VF610_SPU3:
		/* SPU3 SSMB */
		type_str = "SPU3";
		break;
	case ZII_PLATFORM_VF610_CFU1:
		/* CFU1 */
		type_str = "CFU1";
		break;
	default:
		/* UNKNOWN */
		/* As of 8/12/2016 there are known HW issues
		 *  setting to SPU3 as the default for now
		 */
		type_str = "SPU3";
		break;
	}

	seq_printf(m, "%s(%02x)\n", type_str, zii_vf6xx_plat_data.plat_value);
	return 0;
}

static int
board_type_open(struct inode *inode, struct file *file)
{
	return single_open(file, board_type_show, NULL);
}


static const struct file_operations board_type_fops = {
    .owner      = THIS_MODULE,
    .open       = board_type_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};


static int zii_vf6xx_get_control_gpios(struct platform_device *pdev)
{
	int ret, x;

	/* gpios for system_type */
	for(x = 0; x < ARRAY_SIZE(zii_vf6xx_plat_data.gpiod_bits); x++) {
		zii_vf6xx_plat_data.gpiod_bits[x] = devm_gpiod_get_index(&pdev->dev, "platform_id", x, GPIOD_IN);
		if (IS_ERR(zii_vf6xx_plat_data.gpiod_bits[x])) {
			dev_err(&pdev->dev, "unable to claim system_type[%d] gpio\n", x);
			return PTR_ERR(zii_vf6xx_plat_data.gpiod_bits[x]);
		}

		ret = gpiod_direction_input(zii_vf6xx_plat_data.gpiod_bits[x]);
		if (ret)
			return ret;
	}

	return 0;
}

static int zii_vf6xx_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = zii_vf6xx_get_control_gpios(pdev);
	if(ret)
		return ret;

	/* Get the value of the GPIOs */
	zii_vf6xx_plat_data.plat_value  =   gpiod_get_value(zii_vf6xx_plat_data.gpiod_bits[0]);
	zii_vf6xx_plat_data.plat_value |=  (gpiod_get_value(zii_vf6xx_plat_data.gpiod_bits[1]) << 1);
	zii_vf6xx_plat_data.plat_value |=  (gpiod_get_value(zii_vf6xx_plat_data.gpiod_bits[2]) << 2);
	zii_vf6xx_plat_data.plat_value |=  (gpiod_get_value(zii_vf6xx_plat_data.gpiod_bits[3]) << 3);

	zii_vf6xx_proc_dir = proc_mkdir( "rave", NULL );
	proc_create("board_type", 0, zii_vf6xx_proc_dir, &board_type_fops);

	return ret;
}

static int zii_vf6xx_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id zii_vf6xx_dt_ids[] = {
	{ .compatible = "zii,zii_vf6xx_platform"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zii_vf6xx_dt_ids);

static struct platform_driver zii_vf6xx_driver = {
	.driver = {
		.name = "zii-vf6xx-plat",
		.owner = THIS_MODULE,
		.of_match_table = zii_vf6xx_dt_ids,
	},
	.probe = zii_vf6xx_probe,
	.remove = zii_vf6xx_remove,
};
module_platform_driver(zii_vf6xx_driver);

MODULE_AUTHOR("Jeff White <jeff.white@zii.aero>");
MODULE_DESCRIPTION("Zii_VF6XX_ driver");
MODULE_LICENSE("GPL v2");
