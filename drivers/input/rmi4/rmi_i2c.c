/*
 * Copyright (c) 2011-2015 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/rmi.h>
#include <linux/of.h>
#include "rmi_driver.h"

#define BUFFER_SIZE_INCREMENT 32

/**
 * struct rmi_i2c_xport - stores information for i2c communication
 *
 * @xport: The transport interface structure
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 *
 * @tx_buf: Buffer used for transmitting data to the sensor over i2c.
 * @tx_buf_size: Size of the buffer
 */
struct rmi_i2c_xport {
	struct rmi_transport_dev xport;
	struct i2c_client *client;

	struct mutex page_mutex;
	int page;

	int irq;

	u8 *tx_buf;
	size_t tx_buf_size;
};

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

/*
 * rmi_set_page - Set RMI page
 * @xport: The pointer to the rmi_transport_dev struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the transport
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_i2c_xport *rmi_i2c, u8 page)
{
	struct i2c_client *client = rmi_i2c->client;
	u8 txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;

	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, retval);
		return (retval < 0) ? retval : -EIO;
	}

	rmi_i2c->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_transport_dev *xport, u16 addr,
			       const void *buf, size_t len)
{
	struct rmi_i2c_xport *rmi_i2c =
		container_of(xport, struct rmi_i2c_xport, xport);
	struct i2c_client *client = rmi_i2c->client;
	size_t tx_size = len + 1;
	int retval;

	mutex_lock(&rmi_i2c->page_mutex);

	if (!rmi_i2c->tx_buf || rmi_i2c->tx_buf_size < tx_size) {
		if (rmi_i2c->tx_buf)
			devm_kfree(&client->dev, rmi_i2c->tx_buf);
		rmi_i2c->tx_buf_size = tx_size + BUFFER_SIZE_INCREMENT;
		rmi_i2c->tx_buf = devm_kzalloc(&client->dev,
					       rmi_i2c->tx_buf_size,
					       GFP_KERNEL);
		if (!rmi_i2c->tx_buf) {
			rmi_i2c->tx_buf_size = 0;
			retval = -ENOMEM;
			goto exit;
		}
	}

	rmi_i2c->tx_buf[0] = addr & 0xff;
	memcpy(rmi_i2c->tx_buf + 1, buf, len);

	if (RMI_I2C_PAGE(addr) != rmi_i2c->page) {
		retval = rmi_set_page(rmi_i2c, RMI_I2C_PAGE(addr));
		if (retval)
			goto exit;
	}

	retval = i2c_master_send(client, rmi_i2c->tx_buf, tx_size);
	if (retval == tx_size)
		retval = 0;
	else if (retval >= 0)
		retval = -EIO;

exit:
	rmi_dbg(RMI_DEBUG_XPORT, &client->dev,
		"write %zd bytes at %#06x: %d (%*ph)\n",
		len, addr, retval, (int)len, buf);

	mutex_unlock(&rmi_i2c->page_mutex);
	return retval;
}

static int rmi_i2c_read_block(struct rmi_transport_dev *xport, u16 addr,
			      void *buf, size_t len)
{
	struct rmi_i2c_xport *rmi_i2c =
		container_of(xport, struct rmi_i2c_xport, xport);
	struct i2c_client *client = rmi_i2c->client;
	u8 addr_offset = addr & 0xff;
	int retval;
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.len	= sizeof(addr_offset),
			.buf	= &addr_offset,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		},
	};

	mutex_lock(&rmi_i2c->page_mutex);

	if (RMI_I2C_PAGE(addr) != rmi_i2c->page) {
		retval = rmi_set_page(rmi_i2c, RMI_I2C_PAGE(addr));
		if (retval)
			goto exit;
	}

	retval = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (retval == ARRAY_SIZE(msgs))
		retval = 0; /* success */
	else if (retval >= 0)
		retval = -EIO;

exit:
	rmi_dbg(RMI_DEBUG_XPORT, &client->dev,
		"read %zd bytes at %#06x: %d (%*ph)\n",
		len, addr, retval, (int)len, buf);

	mutex_unlock(&rmi_i2c->page_mutex);
	return retval;
}

static const struct rmi_transport_ops rmi_i2c_ops = {
	.write_block	= rmi_i2c_write_block,
	.read_block	= rmi_i2c_read_block,
};

static irqreturn_t rmi_i2c_irq(int irq, void *dev_id)
{
	struct rmi_i2c_xport *rmi_i2c = dev_id;
	struct rmi_device *rmi_dev = rmi_i2c->xport.rmi_dev;
	int ret;

	ret = rmi_process_interrupt_requests(rmi_dev);
	if (ret)
		rmi_dbg(RMI_DEBUG_XPORT, &rmi_dev->dev,
			"Failed to process interrupt request: %d\n", ret);

	return IRQ_HANDLED;
}

static int rmi_i2c_init_irq(struct i2c_client *client)
{
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int irq_flags = rmi_i2c->xport.pdata.irq_flags;
	int ret;

	if (!irq_flags)
		irq_flags = IRQF_TRIGGER_LOW;

	ret = request_threaded_irq(rmi_i2c->irq, NULL, rmi_i2c_irq,
			irq_flags | IRQF_ONESHOT, client->name, rmi_i2c);
	if (ret < 0) {
		dev_warn(&client->dev, "Failed to register interrupt %d\n",
			rmi_i2c->irq);

		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rmi_i2c_of_match[] = {
	{ .compatible = "syna,rmi4-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, rmi_i2c_of_match);
#endif

static int rmi_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct rmi_device_platform_data *pdata;
	struct rmi_device_platform_data *client_pdata =
					dev_get_platdata(&client->dev);
	struct rmi_i2c_xport *rmi_i2c;
	int retval;

	rmi_i2c = devm_kzalloc(&client->dev, sizeof(struct rmi_i2c_xport),
				GFP_KERNEL);
	if (!rmi_i2c)
		return -ENOMEM;

	pdata = &rmi_i2c->xport.pdata;

	if (!client->dev.of_node && client_pdata)
		*pdata = *client_pdata;

	if (client->irq > 0)
		rmi_i2c->irq = client->irq;

	rmi_dbg(RMI_DEBUG_XPORT, &client->dev, "Probing %s.\n",
			dev_name(&client->dev));
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"adapter does not support required functionality.\n");
		return -ENODEV;
	}

	rmi_i2c->client = client;
	mutex_init(&rmi_i2c->page_mutex);

	rmi_i2c->xport.dev = &client->dev;
	rmi_i2c->xport.proto_name = "i2c";
	rmi_i2c->xport.ops = &rmi_i2c_ops;

	i2c_set_clientdata(client, rmi_i2c);

	/*
	 * Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 */
	retval = rmi_set_page(rmi_i2c, 0);
	if (retval) {
		dev_err(&client->dev, "Failed to set page select to 0.\n");
		return retval;
	}

	retval = rmi_register_transport_device(&rmi_i2c->xport);
	if (retval) {
		dev_err(&client->dev, "Failed to register transport driver at 0x%.2X.\n",
			client->addr);
		return retval;
	}

	retval = rmi_i2c_init_irq(client);
	if (retval < 0)
		return retval;

	dev_info(&client->dev, "registered rmi i2c driver at %#04x.\n",
			client->addr);
	return 0;
}

static int rmi_i2c_remove(struct i2c_client *client)
{
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);

	rmi_unregister_transport_device(&rmi_i2c->xport);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rmi_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	ret = rmi_driver_suspend(rmi_i2c->xport.rmi_dev);
	if (ret)
		dev_warn(dev, "Failed to resume device: %d\n", ret);

	disable_irq(rmi_i2c->irq);
	if (device_may_wakeup(&client->dev)) {
		ret = enable_irq_wake(rmi_i2c->irq);
		if (!ret)
			dev_warn(dev, "Failed to enable irq for wake: %d\n",
				ret);
	}
	return ret;
}

static int rmi_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	enable_irq(rmi_i2c->irq);
	if (device_may_wakeup(&client->dev)) {
		ret = disable_irq_wake(rmi_i2c->irq);
		if (!ret)
			dev_warn(dev, "Failed to disable irq for wake: %d\n",
				ret);
	}

	ret = rmi_driver_resume(rmi_i2c->xport.rmi_dev);
	if (ret)
		dev_warn(dev, "Failed to resume device: %d\n", ret);

	return ret;
}
#endif

#ifdef CONFIG_PM
static int rmi_i2c_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	ret = rmi_driver_suspend(rmi_i2c->xport.rmi_dev);
	if (ret)
		dev_warn(dev, "Failed to resume device: %d\n", ret);

	disable_irq(rmi_i2c->irq);

	return 0;
}

static int rmi_i2c_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	enable_irq(rmi_i2c->irq);

	ret = rmi_driver_resume(rmi_i2c->xport.rmi_dev);
	if (ret)
		dev_warn(dev, "Failed to resume device: %d\n", ret);

	return 0;
}
#endif

static const struct dev_pm_ops rmi_i2c_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(rmi_i2c_suspend, rmi_i2c_resume)
	SET_RUNTIME_PM_OPS(rmi_i2c_runtime_suspend, rmi_i2c_runtime_resume,
			   NULL)
};

static const struct i2c_device_id rmi_id[] = {
	{ "rmi4_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi4_i2c",
		.pm	= &rmi_i2c_pm,
		.of_match_table = of_match_ptr(rmi_i2c_of_match),
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= rmi_i2c_remove,
};

module_i2c_driver(rmi_i2c_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
