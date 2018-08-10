/*
 * Driver for Aquantia PHY
 *
 * Author: Shaohui Xie <Shaohui.Xie@freescale.com>
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/hwmon.h>
#include <linux/phy.h>
#include <linux/mdio.h>
#include <linux/mtd/spi-nor.h>

#define PHY_ID_AQ1202	0x03a1b445
#define PHY_ID_AQ2104	0x03a1b460
#define PHY_ID_AQR105	0x03a1b4a2
#define PHY_ID_AQR106	0x03a1b4d0
#define PHY_ID_AQR107	0x03a1b4e0
#define PHY_ID_AQCS109	0x03a1b5c2
#define PHY_ID_AQR405	0x03a1b4b0

#define PHY_AQR_FEATURES	(SUPPORTED_10000baseT_Full | \
				 SUPPORTED_1000baseT_Full | \
				 SUPPORTED_100baseT_Full | \
				 PHY_DEFAULT_FEATURES)

#define MDIO_AN_TX_VEND_STATUS1			0xc800
#define MDIO_AN_TX_VEND_STAUTS1_FULL_DUPLEX	BIT(0)
#define MDIO_AN_TX_VEND_STAUTS1_10BASET		(0x0 << 1)
#define MDIO_AN_TX_VEND_STAUTS1_100BASETX	(0x1 << 1)
#define MDIO_AN_TX_VEND_STAUTS1_1000BASET	(0x2 << 1)
#define MDIO_AN_TX_VEND_STAUTS1_10GBASET	(0x3 << 1)
#define MDIO_AN_TX_VEND_STAUTS1_2500BASET	(0x4 << 1)
#define MDIO_AN_TX_VEND_STAUTS1_5000BASET	(0x5 << 1)
#define MDIO_AN_TX_VEND_STAUTS1_RATE_MASK	(0x7 << 1)
#define MDIO_AN_TX_VEND_INT_STATUS2		0xcc01
#define MDIO_AN_TX_VEND_INT_MASK2		0xd401

/* PHY XS System Interface Connection Status */
#define MDIO_XS_SYSIF_STATUS			0xe812
#define MDIO_XS_SYSIF_MODE_SHIFT		3
#define MDIO_XS_SYSIF_MODE_MASK			0x1f
#define MDIO_XS_SYSIF_MODE_BACKPLANE_KR		0
#define MDIO_XS_SYSIF_MODE_BACKPLANE_KX		1
#define MDIO_XS_SYSIF_MODE_XFI			2
#define MDIO_XS_SYSIF_MODE_USXGMII		3
#define MDIO_XS_SYSIF_MODE_XAUI			4
#define MDIO_XS_SYSIF_MODE_XAUI_PAUSE		5
#define MDIO_XS_SYSIF_MODE_SGMII		6
#define MDIO_XS_SYSIF_MODE_RXAUI		7
#define MDIO_XS_SYSIF_MODE_MAC			8
#define MDIO_XS_SYSIF_MODE_OFF			9
#define MDIO_XS_SYSIF_MODE_OCSGMII		10

/* Vendor specific 1, MMIO_MMD_VEND1 */
#define VEND1_GLOBAL_INT_STD_MASK		0xff00
#define VEND1_GLOBAL_INT_STD_MASK_ALL		BIT(0x0)
#define VEND1_GLOBAL_INT_STD_MASK_GBE		BIT(0x6)
#define VEND1_GLOBAL_INT_STD_MASK_AN2		BIT(0x7)
#define VEND1_GLOBAL_INT_STD_MASK_AN1		BIT(0x8)
#define VEND1_GLOBAL_INT_STD_MASK_PHY_XS2	BIT(0x9)
#define VEND1_GLOBAL_INT_STD_MASK_PHY_XS1	BIT(0xa)
#define VEND1_GLOBAL_INT_STD_MASK_PCS3		BIT(0xb)
#define VEND1_GLOBAL_INT_STD_MASK_PCS3		BIT(0xb)
#define VEND1_GLOBAL_INT_STD_MASK_PCS2		BIT(0xc)
#define VEND1_GLOBAL_INT_STD_MASK_PCS1		BIT(0xd)
#define VEND1_GLOBAL_INT_STD_MASK_PMA2		BIT(0xe)
#define VEND1_GLOBAL_INT_STD_MASK_PMA1		BIT(0xf)

#define VEND1_GLOBAL_INT_VEND_MASK		0xff01
#define VEND1_GLOBAL_INT_VEND_MASK_GLOBAL3	BIT(0x0)
#define VEND1_GLOBAL_INT_VEND_MASK_GLOBAL2	BIT(0x1)
#define VEND1_GLOBAL_INT_VEND_MASK_GLOBAL1	BIT(0x2)
#define VEND1_GLOBAL_INT_VEND_MASK_GBE		BIT(0xb)
#define VEND1_GLOBAL_INT_VEND_MASK_AN		BIT(0xc)
#define VEND1_GLOBAL_INT_VEND_MASK_PHY_XS	BIT(0xd)
#define VEND1_GLOBAL_INT_VEND_MASK_PCS		BIT(0xe)
#define VEND1_GLOBAL_INT_VEND_MASK_PMA		BIT(0xf)

#define VEND1_THERMAL_PROV_HIGH_TEMP_FAIL	0xc421
#define VEND1_THERMAL_PROV_LOW_TEMP_FAIL	0xc422
#define VEND1_THERMAL_PROV_HIGH_TEMP_WARN	0xc423
#define VEND1_THERMAL_PROV_LOW_TEMP_WARN	0xc424
#define VEND1_THERMAL_STAT1			0xc820
#define VEND1_THERMAL_STAT2			0xc821
#define VEND1_THERMAL_STAT2_VALID		BIT(0)
#define VEND1_GENERAL_STAT1			0xc830
#define VEND1_GENERAL_STAT1_HIGH_TEMP_FAIL	BIT(0xe)
#define VEND1_GENERAL_STAT1_LOW_TEMP_FAIL	BIT(0xd)
#define VEND1_GENERAL_STAT1_HIGH_TEMP_WARN	BIT(0xc)
#define VEND1_GENERAL_STAT1_LOW_TEMP_WARN	BIT(0xb)

struct aqr_priv {
	struct device *hwmon_dev;
	char *hwmon_name;

	struct spi_nor *nvr;
	struct mutex nvr_lock;
	u16 global_nvr_prov_w0,
	    global_nvr_prov_w1,
	    global_nvr_prov_w2,
	    global_control_w1;
};

static int aqr_config_aneg(struct phy_device *phydev)
{
	phydev->supported = PHY_AQR_FEATURES;
	phydev->advertising = phydev->supported;

	return 0;
}

static int aqr_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		err = phy_write_mmd(phydev, MDIO_MMD_AN,
				    MDIO_AN_TX_VEND_INT_MASK2, 1);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1,
				    VEND1_GLOBAL_INT_STD_MASK,
				    VEND1_GLOBAL_INT_STD_MASK_ALL);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1,
				    VEND1_GLOBAL_INT_VEND_MASK,
				    VEND1_GLOBAL_INT_VEND_MASK_GLOBAL3 |
				    VEND1_GLOBAL_INT_VEND_MASK_AN);
	} else {
		err = phy_write_mmd(phydev, MDIO_MMD_AN,
				    MDIO_AN_TX_VEND_INT_MASK2, 0);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1,
				    VEND1_GLOBAL_INT_STD_MASK, 0);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1,
				    VEND1_GLOBAL_INT_VEND_MASK, 0);
	}

	return err;
}

static int aqr_ack_interrupt(struct phy_device *phydev)
{
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_AN,
			   MDIO_AN_TX_VEND_INT_STATUS2);
	return (reg < 0) ? reg : 0;
}

static int aqr_read_status(struct phy_device *phydev)
{
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	if (reg & MDIO_STAT1_LSTATUS)
		phydev->link = 1;
	else
		phydev->link = 0;

	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_TX_VEND_STATUS1);
	mdelay(10);
	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_TX_VEND_STATUS1);

	switch (reg & MDIO_AN_TX_VEND_STAUTS1_RATE_MASK) {
	case MDIO_AN_TX_VEND_STAUTS1_10GBASET:
		phydev->speed = SPEED_10000;
		break;
	case MDIO_AN_TX_VEND_STAUTS1_5000BASET:
		phydev->speed = SPEED_50000;
		break;
	case MDIO_AN_TX_VEND_STAUTS1_2500BASET:
		phydev->speed = SPEED_2500;
		break;
	case MDIO_AN_TX_VEND_STAUTS1_1000BASET:
		phydev->speed = SPEED_1000;
		break;
	case MDIO_AN_TX_VEND_STAUTS1_100BASETX:
		phydev->speed = SPEED_100;
		break;
	case MDIO_AN_TX_VEND_STAUTS1_10BASET:
		phydev->speed = SPEED_10;
		break;
	default:
		phydev->speed = SPEED_UNKNOWN;
		break;
	}

	if (reg & MDIO_AN_TX_VEND_STAUTS1_FULL_DUPLEX)
		phydev->duplex = DUPLEX_FULL;
	else
		phydev->duplex = DUPLEX_HALF;

	reg = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MDIO_XS_SYSIF_STATUS);

	switch ((reg >> MDIO_XS_SYSIF_MODE_SHIFT) & MDIO_XS_SYSIF_MODE_MASK) {
	case MDIO_XS_SYSIF_MODE_BACKPLANE_KR:
		phydev->interface = PHY_INTERFACE_MODE_10GKR;
		break;
	case MDIO_XS_SYSIF_MODE_SGMII:
		phydev->interface = PHY_INTERFACE_MODE_SGMII;
		break;
	case MDIO_XS_SYSIF_MODE_XAUI:
		phydev->interface = PHY_INTERFACE_MODE_XAUI;
		break;
	default:
		phydev->interface = PHY_INTERFACE_MODE_NA;
	}

	return 0;
}

#ifdef CONFIG_HWMON
static umode_t aqr_hwmon_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_min_alarm:
	case hwmon_temp_max_alarm:
	case hwmon_temp_lcrit_alarm:
	case hwmon_temp_crit_alarm:
	case hwmon_temp_min:
	case hwmon_temp_max:
	case hwmon_temp_lcrit:
	case hwmon_temp_crit:
		return 0444;
	default:
		return 0;
	}
}

static int aqr_hwmon_get(struct phy_device *phydev, int reg, long *value)
{
	int temp = phy_read_mmd(phydev, MDIO_MMD_VEND1, reg);

	if (temp < 0)
		return temp;
	if (temp > 0x8000)
		temp -= 0x10000;

	*value = temp * 1000 / 256;

	return 0;
}

static int aqr_hwmon_status1(struct phy_device *phydev, int bit, long *value)
{
	int reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, VEND1_GENERAL_STAT1);

	if (reg < 0)
		return reg;

	*value = !!(reg & bit);

	return 0;
}

static int aqr_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *value)
{
	struct phy_device *phydev = dev_get_drvdata(dev);
	int reg;

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_input:
		reg = phy_read_mmd(phydev, MDIO_MMD_VEND1,
				   VEND1_THERMAL_STAT2);
		if (reg < 0)
			return reg;
		if (!(reg & VEND1_THERMAL_STAT2_VALID))
			return -EIO;

		return aqr_hwmon_get(phydev, VEND1_THERMAL_STAT1, value);

	case hwmon_temp_lcrit:
		return aqr_hwmon_get(phydev, VEND1_THERMAL_PROV_LOW_TEMP_FAIL,
				     value);

	case hwmon_temp_min:
		return aqr_hwmon_get(phydev, VEND1_THERMAL_PROV_LOW_TEMP_WARN,
				     value);

	case hwmon_temp_max:
		return aqr_hwmon_get(phydev, VEND1_THERMAL_PROV_HIGH_TEMP_WARN,
				     value);

	case hwmon_temp_crit:
		return aqr_hwmon_get(phydev, VEND1_THERMAL_PROV_HIGH_TEMP_FAIL,
				     value);

	case hwmon_temp_lcrit_alarm:
		return aqr_hwmon_status1(phydev,
					 VEND1_GENERAL_STAT1_LOW_TEMP_FAIL,
					 value);

	case hwmon_temp_min_alarm:
		return aqr_hwmon_status1(phydev,
					 VEND1_GENERAL_STAT1_LOW_TEMP_WARN,
					 value);

	case hwmon_temp_max_alarm:
		return aqr_hwmon_status1(phydev,
					 VEND1_GENERAL_STAT1_HIGH_TEMP_WARN,
					 value);

	case hwmon_temp_crit_alarm:
		return aqr_hwmon_status1(phydev,
					 VEND1_GENERAL_STAT1_HIGH_TEMP_FAIL,
					 value);
	default:
		return -EOPNOTSUPP;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops aqr_hwmon_ops = {
	.is_visible = aqr_hwmon_is_visible,
	.read = aqr_hwmon_read,
};

static u32 aqr_hwmon_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0,
};

static const struct hwmon_channel_info aqr_hwmon_chip = {
	.type = hwmon_chip,
	.config = aqr_hwmon_chip_config,
};

static u32 aqr_hwmon_temp_config[] = {
	HWMON_T_INPUT |
	HWMON_T_MAX | HWMON_T_MIN |
	HWMON_T_MAX_ALARM | HWMON_T_MIN_ALARM |
	HWMON_T_CRIT | HWMON_T_LCRIT |
	HWMON_T_CRIT_ALARM | HWMON_T_LCRIT_ALARM,
	0,
};

static const struct hwmon_channel_info aqr_hwmon_temp = {
	.type = hwmon_temp,
	.config = aqr_hwmon_temp_config,
};

static const struct hwmon_channel_info *aqr_hwmon_info[] = {
	&aqr_hwmon_chip,
	&aqr_hwmon_temp,
	NULL,
};

static const struct hwmon_chip_info aqr_hwmon_chip_info = {
	.ops = &aqr_hwmon_ops,
	.info = aqr_hwmon_info,
};

static int aqr_hwmon_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct aqr_priv *priv = dev_get_drvdata(&phydev->mdio.dev);
	int i, j;

	priv->hwmon_name = devm_kstrdup(dev, dev_name(dev), GFP_KERNEL);
	if (!priv->hwmon_name)
		return -ENODEV;

	for (i = j = 0; priv->hwmon_name[i]; i++) {
		if (isalnum(priv->hwmon_name[i])) {
			if (i != j)
				priv->hwmon_name[j] = priv->hwmon_name[i];
			j++;
		}
	}
	priv->hwmon_name[j] = '\0';

	priv->hwmon_dev = devm_hwmon_device_register_with_info(dev,
				priv->hwmon_name, phydev,
				&aqr_hwmon_chip_info, NULL);

	return PTR_ERR_OR_ZERO(priv->hwmon_dev);
}
#else
static inline int aqr_hwmon_config(struct phy_device *phydev, bool enable)
{
	return 0;
}

static int aqr_hwmon_probe(struct phy_device *phydev)
{
	return 0;
}
#endif

static int aqr_nvr_wait_ready(struct phy_device *phydev)
{
	unsigned long timeout = jiffies + HZ / 10;
	int ret;

	do {
		ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0100);
		if (ret < 0)
			return ret;
		if (!(ret & BIT(8)))
			return 0;
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int aqr_nvr_rx(struct phy_device *phydev, u8 opcode,
		u8 addr_size, u32 addr,
		u8 dummy_size,
		u32 data_size, u8 *data)
{
	int ret;
	u16 val_p, val_i;

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0xc450);
	if (ret < 0)
		return ret;
	val_p = ret;

	if (WARN_ON(addr_size > 3))
		return -EIO;
	val_p &= ~GENMASK(1, 0);
	val_p |= addr_size;

	if (WARN_ON(dummy_size > 4))
		return -EIO;
	val_p &= ~GENMASK(6, 4);
	val_p |= (dummy_size << 4);

	val_p &= ~GENMASK(10, 8);
	val_p |= (min_t(u16, 4, data_size) << 8);

	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xc450, val_p);
	if (ret < 0)
		return ret;

	if (addr_size) {
		ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0103,
				addr & 0xffff);
		if (ret < 0)
			return ret;
	}
	if (addr_size > 2) {
		ret = __phy_modify_mmd(phydev, MDIO_MMD_VEND1, 0x0102,
				0xff, (addr >> 16) & 0xff);
		if (ret < 0)
			return ret;
	}

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0100);
	if (ret < 0)
		return ret;
	val_i = ret;

	val_i &= (BIT(11) | BIT(9));	/* clear all but unused bits */
	val_i |= BIT(15);		/* start op */
	val_i |= opcode;

	if (data_size > 4) {
		for (; data_size > 4; data_size -= 4) {
			ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0100,
					val_i | BIT(10));	/* use burst */
			if (ret < 0)
				return ret;
			ret = aqr_nvr_wait_ready(phydev);
			if (ret < 0)
				return ret;
			ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0105);
			if (ret < 0)
				return ret;
			*data++ = ret & 0xff;
			*data++ = ret >> 8;
			ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0104);
			if (ret < 0)
				return ret;
			*data++ = ret & 0xff;
			*data++ = ret >> 8;
		}

		if (data_size != 4) {
			val_p &= ~GENMASK(10, 8);
			val_p |= (data_size << 8);
			ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xc450,
					val_p);
			if (ret < 0)
				return ret;
		}
	}

	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0100, val_i);
	if (ret < 0)
		return ret;
	ret = aqr_nvr_wait_ready(phydev);
	if (ret < 0)
		return ret;
	if (data_size) {
		ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0105);
		if (ret < 0)
			return ret;
		*data++ = ret & 0xff;
		if (data_size > 1)
			*data++ = ret >> 8;
	}
	if (data_size > 2) {
		ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0104);
		if (ret < 0)
			return ret;
		*data++ = ret & 0xff;
		if (data_size > 3)
			*data++ = ret >> 8;
	}

	return 0;
}

static int aqr_nvr_tx(struct phy_device *phydev, u8 opcode,
		u8 addr_size, u32 addr,
		u32 data_size, const u8 *data)
{
	int ret;
	u16 val_p, val_i, val_d;

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0xc450);
	if (ret < 0)
		return ret;
	val_p = ret;

	if (WARN_ON(addr_size > 3))
		return -EIO;
	val_p &= ~GENMASK(1, 0);
	val_p |= addr_size;

	val_p &= ~GENMASK(6, 4);	/* no dummy bytes for tx */

	val_p &= ~GENMASK(10, 8);
	val_p |= (min_t(u16, 4, data_size) << 8);

	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xc450, val_p);
	if (ret < 0)
		return ret;

	if (addr_size) {
		ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0103,
				addr & 0xffff);
		if (ret < 0)
			return ret;
	}
	if (addr_size > 2) {
		ret = __phy_modify_mmd(phydev, MDIO_MMD_VEND1, 0x0102,
				0xff, (addr >> 16) & 0xff);
		if (ret < 0)
			return ret;
	}

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0x0100);
	if (ret < 0)
		return ret;
	val_i = ret;

	val_i &= (BIT(11) | BIT(9));	/* clear all but unused bits */
	val_i |= BIT(15);		/* start op */
	val_i |= BIT(14);		/* write */
	val_i |= opcode;

	if (data_size > 4) {
		for (; data_size > 4; data_size -= 4, data += 4) {
			ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0105,
					data[0] | (data[1] << 8));
			if (ret < 0)
				return ret;
			ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0104,
					data[2] | (data[3] << 8));
			if (ret < 0)
				return ret;

			ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0100,
					val_i | BIT(10));	/* use burst */
			if (ret < 0)
				return ret;
			ret = aqr_nvr_wait_ready(phydev);
			if (ret < 0)
				return ret;
		}

		if (data_size != 4) {
			val_p &= ~GENMASK(10, 8);
			val_p |= (data_size << 8);
			ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xc450,
					val_p);
			if (ret < 0)
				return ret;
		}
	}

	val_d = data[0];
	if (data_size > 1)
		val_d |= (data[1] << 8);
	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0105, val_d);
	if (ret < 0)
		return ret;
	if (data_size > 2) {
		val_d = data[2];
		if (data_size > 3)
			val_d |= (data[3] << 8);
		ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0104, val_d);
		if (ret < 0)
			return ret;
	}

	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x0100, val_i);
	if (ret < 0)
		return ret;

	return aqr_nvr_wait_ready(phydev);
}

static int aqr_nvr_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct phy_device *phydev =
		container_of(nor->dev, typeof(*phydev), mdio.dev);
	int ret;

	if (WARN_ON(len < 0))
		return -EIO;

	if (WARN_ON((nor->read_dummy % 8)))
		return -EIO;

	mutex_lock(&phydev->mdio.bus->mdio_lock);
	ret = aqr_nvr_rx(phydev, opcode, 0, 0, nor->read_dummy / 8, len, buf);
	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	return ret;
}

static int aqr_nvr_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct phy_device *phydev =
		container_of(nor->dev, typeof(*phydev), mdio.dev);
	int ret;

	if (WARN_ON(len < 0))
		return -EIO;

	mutex_lock(&phydev->mdio.bus->mdio_lock);
	if (len == 0)
		ret = aqr_nvr_rx(phydev, opcode, 0, 0, 0, 0, NULL);
	else
		ret = aqr_nvr_tx(phydev, opcode, 0, 0, len, buf);
	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	return ret;
}

static ssize_t aqr_nvr_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *read_buf)
{
	struct phy_device *phydev =
		container_of(nor->dev, typeof(*phydev), mdio.dev);
	int ret;

	if (WARN_ON((nor->read_dummy % 8)))
		return -EIO;

	mutex_lock(&phydev->mdio.bus->mdio_lock);
	ret = aqr_nvr_rx(phydev, nor->read_opcode,
			nor->addr_width, from,
			nor->read_dummy / 8,
			len, read_buf);
	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	return ret ? ret : len;
}

static ssize_t aqr_nvr_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *write_buf)
{
	struct phy_device *phydev =
		container_of(nor->dev, typeof(*phydev), mdio.dev);
	int ret;

	mutex_lock(&phydev->mdio.bus->mdio_lock);
	ret = aqr_nvr_tx(phydev, nor->program_opcode,
			nor->addr_width, to, len, write_buf);
	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	return ret ? ret : len;
}

static int aqr_nvr_attach(struct device *dev)
{
	struct phy_device *phydev =
		container_of(dev, typeof(*phydev), mdio.dev);
	struct aqr_priv *priv = dev_get_drvdata(dev);
	const struct spi_nor_hwcaps nvr_caps = {
		.mask = SNOR_HWCAPS_READ | SNOR_HWCAPS_PP,
	};
	int ret;

	mutex_lock(&priv->nvr_lock);

	if (priv->nvr) {
		ret = 0;
		goto out;
	}

	/* Before accessing flash, need to
	 * (1) save value of register [1e.c001] that control PHY's CPU
	 * (2) save value of registers [1e.c450 .. 1e.c452] that control
	 *     NVR interface (keeping existing values could be required
	 *     for PHY to be able to read from flash)
	 * (3) halt PHY's CPU
	 * (4) reset PHY but preserve PHY's registers
	 * (5) setup flash to be controlled from registers
	 *
	 * This is more or less same as Aquantia API does
	 */

	mutex_lock(&phydev->mdio.bus->mdio_lock);

	/* (1) */

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0xc001);
	if (ret < 0)
		goto out;
	priv->global_control_w1 = ret;

	/* (2) */

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0xc450);
	if (ret < 0)
		goto out;
	priv->global_nvr_prov_w0 = ret;

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0xc451);
	if (ret < 0)
		goto out;
	priv->global_nvr_prov_w1 = ret;

	ret = __phy_read_mmd(phydev, MDIO_MMD_VEND1, 0xc452);
	if (ret < 0)
		goto out;
	priv->global_nvr_prov_w2 = ret;

	/* (3) */

	/* stall phy's processor */
	ret = __phy_set_bits_mmd(phydev, MDIO_MMD_VEND1, 0xc001,
			BIT(0) | BIT(6));
	if (ret < 0)
		goto out_restore;

	/* (4) */

	/* disable daisy chain */
	ret = __phy_set_bits_mmd(phydev, MDIO_MMD_VEND1, 0xc452, BIT(0));
	if (ret < 0)
		goto out_restore;

	/* force reset of daisy chain machinery */

	/* lock out reset of register map */
	ret = __phy_set_bits_mmd(phydev, MDIO_MMD_VEND1, 0xc006, BIT(14));
	if (ret < 0)
		goto out_restore;

	/* do reset */
	ret = __phy_set_bits_mmd(phydev, MDIO_MMD_VEND1, 0x0000, BIT(15));
	if (ret < 0)
		goto out_restore;
	udelay(100);
	ret = __phy_clear_bits_mmd(phydev, MDIO_MMD_VEND1, 0x0000, BIT(15));
	if (ret < 0)
		goto out_restore;

	/* unlock reset of register map */
	ret = __phy_clear_bits_mmd(phydev, MDIO_MMD_VEND1, 0xc006, BIT(14));
	if (ret < 0)
		goto out_restore;

	/* (5) */

	ret = __phy_set_bits_mmd(phydev, MDIO_MMD_VEND1, 0xc451, BIT(8));
	if (ret < 0)
		goto out_restore;

	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	priv->nvr = kzalloc(sizeof(*priv->nvr), GFP_KERNEL);
	if (!priv->nvr) {
		ret = -ENOMEM;
		goto out_relock;
	}

	priv->nvr->dev = dev;
	priv->nvr->mtd.name = "aqr-nvr";

	priv->nvr->read_reg = aqr_nvr_read_reg;
	priv->nvr->write_reg = aqr_nvr_write_reg;
	priv->nvr->read = aqr_nvr_read;
	priv->nvr->write = aqr_nvr_write;

	ret = spi_nor_scan(priv->nvr, NULL, &nvr_caps);
	if (ret < 0)
		goto out_free;

	ret = mtd_device_register(&priv->nvr->mtd, NULL, 0);
	if (ret < 0)
		goto out_free;

	mutex_unlock(&priv->nvr_lock);
	return 0;

out_free:
	kfree(priv->nvr);
	priv->nvr = NULL;
out_relock:
	mutex_lock(&phydev->mdio.bus->mdio_lock);
out_restore:
	__phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc452, priv->global_nvr_prov_w2);
	__phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc451, priv->global_nvr_prov_w1);
	__phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc450, priv->global_nvr_prov_w0);
	__phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc001, priv->global_control_w1);
out:
	mutex_unlock(&phydev->mdio.bus->mdio_lock);
	mutex_unlock(&priv->nvr_lock);
	return ret;
}

static int aqr_nvr_detach(struct device *dev)
{
	struct phy_device *phydev =
		container_of(dev, typeof(*phydev), mdio.dev);
	struct aqr_priv *priv = dev_get_drvdata(dev);
	int ret, ret2;

	mutex_lock(&priv->nvr_lock);

	if (!priv->nvr) {
		ret = 0;
		goto out;
	}

	mtd_device_unregister(&priv->nvr->mtd);

	kfree(priv->nvr);
	priv->nvr = NULL;

	mutex_lock(&phydev->mdio.bus->mdio_lock);

	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc452, priv->global_nvr_prov_w2);

	ret2 = __phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc451, priv->global_nvr_prov_w1);
	if (ret2 && !ret)
		ret = ret2;

	ret2 = __phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc450, priv->global_nvr_prov_w0);
	if (ret2 && !ret)
		ret = ret2;

	ret2 = __phy_write_mmd(phydev, MDIO_MMD_VEND1,
			0xc001, priv->global_control_w1);
	if (ret2 && !ret)
		ret = ret2;

	mutex_unlock(&phydev->mdio.bus->mdio_lock);
out:
	mutex_unlock(&priv->nvr_lock);
	return ret;
}

static ssize_t nvr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aqr_priv *priv = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&priv->nvr_lock);
	if (priv->nvr)
		ret = sprintf(buf, "mtd%d\n", priv->nvr->mtd.index);
	else
		ret = sprintf(buf, "(none)\n");
	mutex_unlock(&priv->nvr_lock);

	return ret;
}

static ssize_t nvr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char tmpbuf[16], *tmp;
	ssize_t ret;

	strlcpy(tmpbuf, buf, sizeof(tmpbuf));
	tmp = strstrip(tmpbuf);

	if (!strcmp(tmp, "attach"))
		ret = aqr_nvr_attach(dev);
	else if (!strcmp(tmp, "detach"))
		ret = aqr_nvr_detach(dev);
	else
		ret = -EINVAL;

	return ret ? ret : count;
}

static DEVICE_ATTR_RW(nvr);

static int aqr_probe(struct phy_device *phydev)
{
	struct aqr_priv *priv;
	u32 mmd_mask = MDIO_DEVS_PMAPMD | MDIO_DEVS_AN;
	int ret;

	if (!phydev->is_c45 ||
	    (phydev->c45_ids.devices_in_package & mmd_mask) != mmd_mask)
		return -ENODEV;

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(&phydev->mdio.dev, priv);

	ret = aqr_hwmon_probe(phydev);
	if (ret)
		return ret;

	ret = device_create_file(&phydev->mdio.dev, &dev_attr_nvr);
	if (ret)
		return ret;
	mutex_init(&priv->nvr_lock);

	return 0;
}

static void aqr_remove(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct aqr_priv *priv = dev_get_drvdata(dev);

	if (priv->nvr)
		aqr_nvr_detach(dev);

	device_remove_file(dev, &dev_attr_nvr);
}

static struct phy_driver aqr_driver[] = {
{
	.phy_id		= PHY_ID_AQ1202,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQ1202",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
{
	.phy_id		= PHY_ID_AQ2104,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQ2104",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
{
	.phy_id		= PHY_ID_AQR105,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR105",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
{
	.phy_id		= PHY_ID_AQR106,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR106",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
{
	.phy_id		= PHY_ID_AQR107,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR107",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.probe          = aqr_probe,
	.remove		= aqr_remove,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
{
	.phy_id		= PHY_ID_AQCS109,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQCS109",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.probe          = aqr_probe,
	.remove		= aqr_remove,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
{
	.phy_id		= PHY_ID_AQR405,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR405",
	.features	= PHY_AQR_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aqr_config_aneg,
	.config_intr	= aqr_config_intr,
	.ack_interrupt	= aqr_ack_interrupt,
	.read_status	= aqr_read_status,
},
};

module_phy_driver(aqr_driver);

static struct mdio_device_id __maybe_unused aqr_tbl[] = {
	{ PHY_ID_AQ1202, 0xfffffff0 },
	{ PHY_ID_AQ2104, 0xfffffff0 },
	{ PHY_ID_AQR105, 0xfffffff0 },
	{ PHY_ID_AQR106, 0xfffffff0 },
	{ PHY_ID_AQR107, 0xfffffff0 },
	{ PHY_ID_AQCS109, 0xfffffff0 },
	{ PHY_ID_AQR405, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, aqr_tbl);

MODULE_DESCRIPTION("Aquantia PHY driver");
MODULE_AUTHOR("Shaohui Xie <Shaohui.Xie@freescale.com>");
MODULE_LICENSE("GPL v2");
