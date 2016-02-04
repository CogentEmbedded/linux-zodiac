/*
 *  zii-pic-wdt.c - Watchdog driver on Zodiac Inflight Infotainment
 *  PIC MCU that is connected via dedicated UART port
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

#include <linux/zii-pic.h>

struct zii_pic_wdt {
	struct watchdog_device	wdt;
	struct device		*pic_dev;
	struct notifier_block	restart_handler;
};

static int zii_pic_wdt_start(struct watchdog_device *wdt_dev)
{
	struct zii_pic_wdt *adev = container_of(wdt_dev,
						struct zii_pic_wdt,
						wdt);
	pr_debug("%s: enter\n", __func__);

	return zii_pic_watchdog_enable(adev->pic_dev);
}

static int zii_pic_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct zii_pic_wdt *adev = container_of(wdt_dev,
						struct zii_pic_wdt,
						wdt);
	pr_debug("%s: enter\n", __func__);

	return zii_pic_watchdog_disable(adev->pic_dev);
}

static int zii_pic_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct zii_pic_wdt *adev = container_of(wdt_dev,
						struct zii_pic_wdt,
						wdt);
	pr_debug("%s: enter\n", __func__);


	return zii_pic_watchdog_ping(adev->pic_dev);
}

static int zii_pic_wdt_set_timeout(struct watchdog_device *wdt_dev,
				   unsigned int timeout)
{
	struct zii_pic_wdt *adev = container_of(wdt_dev,
						struct zii_pic_wdt,
						wdt);
	int ret;

	pr_debug("%s: enter\n", __func__);

	ret = zii_pic_watchdog_set_timeout(adev->pic_dev, timeout);
	if (!ret)
		adev->wdt.timeout = timeout;

	return ret;
}

static const struct watchdog_info zii_pic_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "ZII PIC Watchdog",
};

static const struct watchdog_ops zii_pic_wdt_ops = {
	.owner = THIS_MODULE,
	.start = zii_pic_wdt_start,
	.stop = zii_pic_wdt_stop,
	.ping = zii_pic_wdt_ping,
	.set_timeout = zii_pic_wdt_set_timeout,
};

static int zii_pic_wdt_restart_handler(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	struct zii_pic_wdt *adev = container_of(this,
						struct zii_pic_wdt,
						restart_handler);

#ifdef DEBUG
	pr_emerg("%s: enter\n", __func__);
#endif
	/* HACK: workaround to let reset command pass through UART to PIC.
	 * Reason: in machine_restart() all IRQs are disabled, and after
	 * command is written to UART buffer it remains there, as no TX interrupt
	 * is handled by IMX serial driver
	 */
	local_irq_enable();

	zii_pic_watchdog_reset(adev->pic_dev, false);

	return NOTIFY_DONE;
}

static const struct of_device_id zii_pic_wdt_of_match[] = {
	{ .compatible = "zii,pic-watchdog" },
	{}
};

static int zii_pic_wdt_probe(struct platform_device *pdev)
{
	struct zii_pic_wdt *adev;
	struct nvmem_cell *cell;
	int ret;

	pr_debug("%s: enter: dev->parent: %p\n", __func__, pdev->dev.parent);

	if (!pdev->dev.of_node)
		return -ENODEV;

	if (!pdev->dev.parent)
		return -EINVAL;

	adev = devm_kzalloc(&pdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->pic_dev = pdev->dev.parent;
	ret = zii_pic_watchdog_get_status(adev->pic_dev);
	if (ret)
		return ret;

	adev->wdt.info = &zii_pic_wdt_info;
	adev->wdt.ops = &zii_pic_wdt_ops;
	adev->wdt.min_timeout = ZII_PIC_WDT_MIN_TIMEOUT;
	adev->wdt.max_timeout = ZII_PIC_WDT_MAX_TIMEOUT;
	adev->wdt.status = WATCHDOG_NOWAYOUT_INIT_STATUS;

	cell = devm_nvmem_cell_get(&pdev->dev, "wdt_timeout");
	if (IS_ERR(cell)) {
		pr_warn("%s: unable to get WDT Timeout from EEPROM, err: %ld\n", __func__, PTR_ERR(cell));
		adev->wdt.timeout = ZII_PIC_WDT_DEFAULT_TIMEOUT;
	} else {
		void *value;
		size_t cell_len;

		value = nvmem_cell_read(cell, &cell_len);
		if (!IS_ERR(value)) {
			adev->wdt.timeout = *(u16*)value;

			pr_debug("Using WDT timeout from EEPROM: %d\n",
				adev->wdt.timeout);
		}
		devm_nvmem_cell_put(&pdev->dev, cell);
	}

	watchdog_set_drvdata(&adev->wdt, adev);
	dev_set_drvdata(&pdev->dev, adev);

	ret = watchdog_register_device(&adev->wdt);
	if (ret)
		return ret;

	adev->restart_handler.notifier_call = zii_pic_wdt_restart_handler;
	adev->restart_handler.priority = 255;
	ret = register_restart_handler(&adev->restart_handler);
	if (ret)
		pr_err("Failed to register restart handler (err = %d)\n", ret);

	return ret;
}

static int zii_pic_wdt_remove(struct platform_device *pdev)
{
	struct zii_pic_wdt *adev = dev_get_drvdata(&pdev->dev);

	pr_debug("%s: enter\n", __func__);

	unregister_restart_handler(&adev->restart_handler);

	watchdog_unregister_device(&adev->wdt);

	return 0;
}

static struct platform_driver zii_pic_wdt_driver = {
	.probe = zii_pic_wdt_probe,
	.remove = zii_pic_wdt_remove,
	.driver = {
		.name = ZII_PIC_NAME_WATCHDOG,
		.of_match_table = zii_pic_wdt_of_match,
	},
};

module_platform_driver(zii_pic_wdt_driver);

MODULE_DEVICE_TABLE(of, zii_pic_wdt_of_match);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC Watchdog driver");
MODULE_ALIAS("platform:zii-pic-watchdog");
