/*
 * uart slave core - device bus for uart slaves
 *
 * Copyright (C) 2015 NeilBrown <neil@brown.name>
 *
 *    This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

/*
 * A "uart-slave" is a device permanently attached to a particular
 * wired to a UART.
 * A uart-slave has two particular roles.
 * Firstly it can intercept any tty_operations to provide extra control
 * of the device.  For example it might intercept "open" and "close"
 * in order to power the device up and down.  It might intercept
 * "hangup" to toggle a reset line on the device.
 *
 * Secondly it appears as a parent of the tty in the device model, so
 * that any attributes it presents are visible to udev when the tty
 * is added.  This allows udev to start appropriate handlers such as
 * hciattach or inputattach.
 *
 * uart-slave devices must be described in devicetree as a child node
 * of the node which described the attaching UART.
 *
 * If such a child is present, the tty device will not be registered
 * until the slave device is fully probed and initialized.
 */
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/uart_slave.h>

static int uart_slave_match(struct device *dev, struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static void uart_slave_release(struct device *dev)
{
	struct uart_slave *slave =
		container_of(dev, struct uart_slave, dev);

	if (!slave->finalized) {
		put_device(slave->tty_dev);
		tty_driver_kref_put(slave->tty_drv);
	}
	of_node_put(dev->of_node);
	kfree(slave);
}

struct bus_type uart_slave_bus_type = {
	.name		= "uart-slave",
	.match		= uart_slave_match,
};

static int uart_slave_tty_dev_match(struct device *dev, void *tty)
{
	struct uart_slave *slave =
		container_of(dev, struct uart_slave, dev);

	return slave->tty_dev == tty;
}

static int uart_slave_add_dev(struct device *class_dev,
		struct class_interface *class_intf)
{
	struct uart_slave *slave;
	struct device *dev;
	int ret = 0;

	dev = bus_find_device(&uart_slave_bus_type, NULL,
			class_dev, uart_slave_tty_dev_match);
	if (!dev)
		return 0;

	slave = container_of(dev, struct uart_slave, dev);
	if (slave->device_added)
		ret = slave->device_added(slave);

	put_device(dev);

	return ret;
}

static struct class_interface uart_slave_interface = {
	.add_dev = &uart_slave_add_dev,
};

int uart_slave_register(struct device *parent,
			struct device *tty, struct tty_driver *drv)
{
	struct device_node *node, *found = NULL;
	struct uart_slave *slave;
	int retval;

	if (!parent || !parent->of_node)
		return -ENODEV;

	for_each_available_child_of_node(parent->of_node, node) {
		if (!of_get_property(node, "compatible", NULL))
			continue;
		if (found) {
			dev_err(parent, "Multiple connected children found - non registered");
			return -ENODEV;
		}
		found = node;
	}
	if (!found)
		return -ENODEV;

	slave = kzalloc(sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return -ENOMEM;

	slave->dev.bus = &uart_slave_bus_type;
	slave->dev.parent = parent;
	slave->dev.release = uart_slave_release;
	slave->dev.of_node = of_node_get(found);
	dev_set_name(&slave->dev, "%s", found->name);
	slave->tty_dev = tty;
	slave->tty_drv = tty_driver_kref_get(drv);
	slave->ops = *drv->ops;
	retval = device_register(&slave->dev);
	if (retval)
		put_device(&slave->dev);
	return retval;
}
EXPORT_SYMBOL(uart_slave_register);

void uart_slave_activate(struct tty_struct *tty)
{
	struct device *parent = NULL;
	if (tty->dev)
		parent = tty->dev->parent;
	if (parent &&
	    parent->bus == &uart_slave_bus_type)
	{
		struct uart_slave *dev =
			container_of(parent, struct uart_slave, dev);
		tty->ops = &dev->ops;
	}
}
EXPORT_SYMBOL(uart_slave_activate);

int uart_slave_add_tty(struct uart_slave *slave)
{
	int retval;
	if (slave->finalized)
		return -EBUSY;
	slave->tty_dev->parent = &slave->dev;
	retval = tty_device_add(slave->tty_drv,
				slave->tty_dev);
	/* If that succeeded, the tty now holds a reference to
	 * the slave through ->parent, so that ref we hold
	 * can be dropped, as can our ref on the tty driver.
	 */
	slave->finalized = true;
	tty_driver_kref_put(slave->tty_drv);
	put_device(&slave->dev);
	return retval;
}
EXPORT_SYMBOL(uart_slave_add_tty);

int uart_slave_driver_register(struct device_driver *drv)
{
	drv->bus = &uart_slave_bus_type;
	return driver_register(drv);
}
EXPORT_SYMBOL(uart_slave_driver_register);

static int __init uart_slave_init(void)
{
	int ret;

	ret = bus_register(&uart_slave_bus_type);
	if (ret)
		goto exit;

	uart_slave_interface.class = tty_class;
	ret = class_interface_register(&uart_slave_interface);
	if (ret) {
		bus_unregister(&uart_slave_bus_type);
		pr_err("%s: unable to register class interface\n", __func__);
	}

exit:
	return ret;
}

static void __exit uart_slave_exit(void)
{
	class_interface_unregister(&uart_slave_interface);

	bus_unregister(&uart_slave_bus_type);
}

postcore_initcall(uart_slave_init);
module_exit(uart_slave_exit);
MODULE_AUTHOR("NeilBrown <neil@brown.name>");
MODULE_DESCRIPTION("UART-slave core support.");
MODULE_LICENSE("GPL v2");
