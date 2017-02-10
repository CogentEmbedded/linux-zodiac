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



struct zii_mx6_platform_data {
	struct gpio_desc *gpiod_bits[4];
	unsigned plat_value;
};

enum zii_mx6_platform_types {
	ZII_PLATFORM_RDU2_MX6Q = 0x00
};

struct zii_mx6_platform_data zii_mx6_plat_data;
struct proc_dir_entry *zii_mx6_proc_dir;


static int
board_type_show(struct seq_file *m, void *v)
{
	char *type_str;
	switch(zii_mx6_plat_data.plat_value) {
	case ZII_PLATFORM_RDU2_MX6Q:
		/* RDU2 with MX6Q */
		type_str = "RDU2";
		break;
	default:
		/* UNKNOWN */
		type_str = "UNKNOWN";
		break;
	}

	seq_printf(m, "%s(%02x)\n", type_str, zii_mx6_plat_data.plat_value);
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


static int zii_mx6_get_control_gpios(struct platform_device *pdev)
{
	int ret, x;

	/* gpios for system_type */
	for(x = 0; x < ARRAY_SIZE(zii_mx6_plat_data.gpiod_bits); x++) {
		zii_mx6_plat_data.gpiod_bits[x] = devm_gpiod_get_index(&pdev->dev, "platform_id", x, GPIOD_IN);
		if (IS_ERR(zii_mx6_plat_data.gpiod_bits[x])) {
			dev_err(&pdev->dev, "unable to claim system_type[%d] gpio\n", x);
			return PTR_ERR(zii_mx6_plat_data.gpiod_bits[x]);
		}

		ret = gpiod_direction_input(zii_mx6_plat_data.gpiod_bits[x]);
		if (ret)
			return ret;
	}

	return 0;
}

static int zii_mx6_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = zii_mx6_get_control_gpios(pdev);
	if(ret)
		return ret;

	/* Get the value of the GPIOs */
	zii_mx6_plat_data.plat_value  =   gpiod_get_value(zii_mx6_plat_data.gpiod_bits[0]);
	zii_mx6_plat_data.plat_value |=  (gpiod_get_value(zii_mx6_plat_data.gpiod_bits[1]) << 1);
	zii_mx6_plat_data.plat_value |=  (gpiod_get_value(zii_mx6_plat_data.gpiod_bits[2]) << 2);
	zii_mx6_plat_data.plat_value |=  (gpiod_get_value(zii_mx6_plat_data.gpiod_bits[3]) << 3);

	zii_mx6_proc_dir = proc_mkdir( "rave", NULL );
	proc_create("board_type", 0, zii_mx6_proc_dir, &board_type_fops);

	return ret;
}

static int zii_mx6_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id zii_mx6_dt_ids[] = {
	{ .compatible = "zii,zii_mx6_platform"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zii_mx6_dt_ids);

static struct platform_driver zii_mx6_driver = {
	.driver = {
		.name = "zii-mx6-plat",
		.owner = THIS_MODULE,
		.of_match_table = zii_mx6_dt_ids,
	},
	.probe = zii_mx6_probe,
	.remove = zii_mx6_remove,
};
module_platform_driver(zii_mx6_driver);

MODULE_AUTHOR("Jeff White <jeff.white@zii.aero>");
MODULE_DESCRIPTION("Zii_i.MX6_ driver");
MODULE_LICENSE("GPL v2");
