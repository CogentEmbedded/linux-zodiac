/*
 * Export Nameplate information in /sys
 *
 * Copyright (C) 2015 Andrew Lunn
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>

typedef enum {
	NP_INT,
	NP_STRING,
} np_type;

struct attrib {
	struct list_head list;
	struct proc_dir_entry *proc_file;
	struct bin_attribute attr;
	struct nvmem_cell *cell;
	np_type type;
};

struct proc_dir_entry *proc_dir;

static LIST_HEAD(attribs);

static ssize_t np_read(struct file *filp,
					   struct kobject *kobj,
					   struct bin_attribute *attr,
					   char *buf, loff_t pos, size_t count)
{
	struct attrib *attrib = attr->private;
	void *value;
	size_t cell_len;
	int ret = 0;

	value = nvmem_cell_read(attrib->cell, &cell_len);
	if (IS_ERR(value))
		return PTR_ERR(value);

	if (attrib->type == NP_STRING) {
		if (pos >= cell_len) {
			ret = 0;
			goto out;
		}

		if (pos + count > cell_len)
			count = cell_len - pos;

		memcpy(buf, value + pos, count);
		ret = count;
	}
	if (attrib->type == NP_INT) {
		u8 u8;
		u32 u32;
		size_t len;
		char string[128];

		switch (cell_len) {
		case 1:
			memcpy(&u8, value, 1);
			len = sprintf(string, "0x%02X", u8);
			break;
		case 4:
			memcpy(&u32, value, 4);
			len = sprintf(string, "0x%08X", u32);
			break;
		default:
			ret = -EINVAL;
			goto out;
		}

		if (pos >= len) {
			ret = 0;
			goto out;
		}

		if (pos + count > len)
			count = len - pos;

		memcpy(buf, string + pos, count);
		ret = count;
	}

out:
	kfree(value);
	return ret;
}

static const struct bin_attribute bin_attr = {
	.attr	= {
		.mode	= S_IRUGO,
	},
	.read	= np_read,
};

static int zii_np_remove(struct platform_device *pdev)
{
	struct attrib *attrib;

	list_for_each_entry(attrib, &attribs, list) {
		device_remove_bin_file(&pdev->dev, &attrib->attr);
		if (attrib->proc_file)
			proc_remove(attrib->proc_file);
	}

	if (proc_dir)
		proc_remove(proc_dir);

	return 0;
}

static int np_show(struct seq_file *m, void *v)
{
	void *value;
	int ret = 0;
	size_t cell_len;
	struct attrib *attrib = m->private;

	value = nvmem_cell_read(attrib->cell, &cell_len);

	if (attrib->type == NP_STRING) {
		memcpy(m->buf + m->count, value, cell_len);
		m->count += cell_len;
	}
	if (attrib->type == NP_INT) {
		u8 u8;
		u32 u32;

		switch (cell_len) {
		case 1:
			memcpy(&u8, value, 1);
			seq_printf(m, "0x%02X", u8);
			break;
		case 4:
			memcpy(&u32, value, 4);
			seq_printf(m, "0x%08X", u32);
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
	}

out:
	kfree(value);

	return 0;
}

static int np_open(struct inode *inode, struct file *file)
{
	return single_open(file, np_show, PDE_DATA(inode));
}

static const struct file_operations np_fops = {
	.owner      = THIS_MODULE,
	.open       = np_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int zii_np_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct nvmem_cell *cell;
	struct attrib *attrib;
	const char *name;
	const char *proc_path;
	int i = 0;
	int ret;

	ret = of_property_read_string(np, "proc-path", &proc_path);
	if (!ret) {
		proc_dir = proc_mkdir(proc_path, NULL);
		if (proc_dir == NULL)
			return -ENOMEM;
	}

	for (;;) {
		ret = of_property_read_string_index(np, "strings", i, &name);
		if (ret)
			break;
		cell = devm_nvmem_cell_get(dev, name);
		if (IS_ERR(cell)) {
			dev_warn(dev, "Unable to get nvmem cell %s\n", name);
			ret = PTR_ERR(cell);
			goto out;
		}
		attrib = devm_kzalloc(dev, sizeof(*attrib), GFP_KERNEL);
		if (!attrib) {
			ret = -ENOMEM;
			goto out;
		}
		INIT_LIST_HEAD(&attrib->list);
		attrib->attr = bin_attr;
		sysfs_bin_attr_init(&attrib->attr);
		attrib->attr.attr.name = name;
		attrib->attr.private = attrib;
		attrib->cell = cell;
		attrib->type = NP_STRING;

		ret = device_create_bin_file(dev, &attrib->attr);
		if (ret)
			goto out;

		if (proc_dir)
			attrib->proc_file = proc_create_data(name, S_IRUGO,
					proc_dir, &np_fops, attrib);

		list_add(&attrib->list, &attribs);
		i++;
	}

	i = 0;

	for (;;) {
		ret = of_property_read_string_index(np, "integers", i, &name);
		if (ret)
			break;
		cell = devm_nvmem_cell_get(dev, name);
		if (IS_ERR(cell)) {
			dev_warn(dev, "Unable to get nvmem cell %s\n", name);
			ret = PTR_ERR(cell);
			goto out;
		}
		attrib = devm_kzalloc(dev, sizeof(*attrib), GFP_KERNEL);
		if (!attrib) {
			ret = -ENOMEM;
			goto out;
		}
		INIT_LIST_HEAD(&attrib->list);
		attrib->attr = bin_attr;
		sysfs_bin_attr_init(&attrib->attr);
		attrib->attr.attr.name = name;
		attrib->attr.private = attrib;
		attrib->cell = cell;
		attrib->type = NP_INT;

		ret = device_create_bin_file(dev, &attrib->attr);
		if (ret)
			goto out;

		if (proc_dir)
			attrib->proc_file = proc_create_data(name, S_IRUGO,
					proc_dir, &np_fops, attrib);

		list_add(&attrib->list, &attribs);
		i++;
	}

	return 0;
out:
	zii_np_remove(pdev);

	return ret;
}

static const struct of_device_id zii_np_of_match_table[] = {
	{
		.compatible = "zii,nameplate"
	},
	{}
};
MODULE_DEVICE_TABLE(of, orion_spi_of_match_table);

static struct platform_driver zii_np_driver = {
	.driver = {
		.name	= "zii-nameplate",
		.of_match_table = zii_np_of_match_table,
	},
	.probe		= zii_np_probe,
	.remove		= zii_np_remove,
};

module_platform_driver(zii_np_driver);

MODULE_DESCRIPTION("ZII Nameplate driver");
MODULE_AUTHOR("Andrew Lunn <andrew@lunn.ch>");
MODULE_LICENSE("GPL");
