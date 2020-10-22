/*
 * Fujitsu FRAM(Ferroelectronic RAM) SPI Interface driver (mb85rs series)
 *
 * Copyright (C) 2018 JongHo Kim <furmuwon@gmail.com>
 *
 * Based on at25.c
 *    Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

#define FUJITSU_MID 0x04

#define PID_MB85RS16   0x01  /* Density 16Kbit */
#define PID_MB85RS64V  0x03  /* Density 64Kbit */
#define PID_MB85RS128B 0x04  /* Density 128Kbit */
#define PID_MB85RS256B 0x05  /* Density 256Kbit */
#define PID_MB85RS512T 0x26  /* Density 512Kbit */
#define PID_MB85RS1MT  0x27  /* Density 1Mbit */
#define PID_MB85RS2MT  0x28  /* Density 2Mbit */

#define MB85RS_OP_WRSR  0x1  /* Write Status Register  */
#define MB85RS_OP_WRITE 0x2  /* Write Memory Code */
#define MB85RS_OP_READ  0x3  /* Read Memory Code */
#define MB85RS_OP_WRDI  0x4  /* Reset Write Enable Latch */
#define MB85RS_OP_RDSR  0x5  /* Read Status Register */
#define MB85RS_OP_WREN  0x6  /* Set Write Enable Latch */
#define MB85RS_OP_RDID  0x9f /* Read Device ID */

#define MB85RS_BLOCKPROTECT_NONE     0x0
#define MB85RS_BLOCKPROTECT_QUARTER  0x1
#define MB85RS_BLOCKPROTECT_HALF     0x2
#define MB85RS_BLOCKPROTECT_ALL      0x3

struct mb85rs_data {
	struct spi_device *spi;
	struct mutex lock;
	struct bin_attribute bin;
	int addr_len;
	int buf_sz;
	int wp_gpio;
};

static ssize_t mb85_fram_read(struct mb85rs_data *mb85rs, char *buf,
				loff_t off, size_t count)
{
	int err;
	struct spi_transfer t[2];
	struct spi_message m;
	struct spi_device *spi = mb85rs->spi;
	u8 cmd_buffer[4];
	u8 *cmd_addr = cmd_buffer;

	if (unlikely(off >= (mb85rs->bin.size)))
		return -EFBIG;
	if ((off + count) > (mb85rs->bin.size))
		count = (mb85rs->bin.size) - off;
	if (unlikely(!count))
		return count;

	spi_message_init(&m);
	memset(t, 0, sizeof t);

	*cmd_addr++ = MB85RS_OP_READ;
	switch(mb85rs->addr_len) {
	case 3:
		*cmd_addr++ = (off >> 16) & 0xff;
	case 2:
	default:
		*cmd_addr++ = (off >> 8) & 0xff;
		*cmd_addr++ = off & 0xff;
	}

	t[0].tx_buf = cmd_buffer;
	t[0].len = mb85rs->addr_len + 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&mb85rs->lock);
	err = spi_sync(spi, &m);
	if (err < 0) {
		dev_err(&spi->dev, "cmd:READ err:%d\n", err);
	}
	mutex_unlock(&mb85rs->lock);
	return err ? err : count;
}

static ssize_t mb85rs_bin_read(struct file *filp, struct kobject *kobj,
				      struct bin_attribute *bin_attr,
				      char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct mb85rs_data *mb85rs;
	dev = container_of(kobj, struct device, kobj);
	mb85rs = dev_get_drvdata(dev);
	return mb85_fram_read(mb85rs, buf, off, count);
}

static ssize_t mb85rs_fram_write(struct mb85rs_data *mb85rs, const char *buf,
					loff_t off, size_t count)
{
	int err = 0;
	int retries = 0;
	ssize_t written = 0;
	u32 addr_word;
	u8 cmd;
	u8 sr;
	u8 *trans_buf;
	u8 *cmd_addr;
	u32 segment;
	struct spi_device *spi = mb85rs->spi;

	if (unlikely(off >= (mb85rs->bin.size)))
		return -EFBIG;
	if ((off + count) > (mb85rs->bin.size))
		count = (mb85rs->bin.size) - off;
	if (unlikely(!count))
		return count;

	trans_buf = kzalloc(mb85rs->buf_sz + mb85rs->addr_len + 1, GFP_KERNEL);
	if (!trans_buf)
		return -ENOMEM;

	mutex_lock(&mb85rs->lock);
	do {
		cmd_addr = trans_buf;
		addr_word = off;
		cmd = MB85RS_OP_WREN;
		err = spi_write(spi, &cmd, 1);
		if (err < 0) {
			dev_err(&spi->dev, "cmd:WREN err:%d\n", err);
			break;
		}

		while (1) {
			cmd = MB85RS_OP_RDSR;
			err = spi_write_then_read(mb85rs->spi, &cmd, 1, &sr, 1);
			if (err < 0) {
				dev_err(&spi->dev, "cmd:RDSR err:%d\n", err);
				break;
			}

			if ((sr & 0x2)) break;

			if (retries++ > 3) {
				dev_err(&spi->dev, "Not set WEL bit\n");
				err = -ETIMEDOUT;
				break;
			}
			msleep(10);
		}

		*cmd_addr++ = MB85RS_OP_WRITE;
		switch(mb85rs->addr_len) {
		case 3:
			*cmd_addr++ = (addr_word >> 16) & 0xff;
		case 2:
		default:
			*cmd_addr++ = (addr_word >> 8) & 0xff;
			*cmd_addr++ = addr_word & 0xff;
		}

		segment = mb85rs->buf_sz - (addr_word % mb85rs->buf_sz);
		if (segment > count)
			segment = count;
		memcpy(cmd_addr, buf, segment);
		err = spi_write(spi, trans_buf, segment + mb85rs->addr_len + 1);
		if (err < 0) {
			dev_err(&spi->dev, "cmd:WRITE err:%d\n", err);
			break;
		}
		cmd = MB85RS_OP_WRDI;
		err = spi_write(spi, &cmd, 1);
		if (err < 0) {
			dev_err(&spi->dev, "cmd:WRDI err:%d\n", err);
			break;
		}
		off += segment;
		buf += segment;
		count -= segment;
		written += segment;
	} while (count > 0);

	mutex_unlock(&mb85rs->lock);

	kfree(trans_buf);
	return written ? written : err;
}

static ssize_t mb85rs_bin_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct mb85rs_data *mb85rs;
	dev = container_of(kobj, struct device, kobj);
	mb85rs = dev_get_drvdata(dev);
	return mb85rs_fram_write(mb85rs, buf, off, count);
}

static int mb85rs_set_block_protect(struct spi_device *spi, u8 bp)
{
	int err;
	u8 status_reg = 0;
	u8 cmd = MB85RS_OP_RDSR;
	u8 cmd_ws[2];
	err = spi_write_then_read(spi, &cmd, 1, &status_reg, 1);
	if (err < 0) {
		dev_err(&spi->dev, "cmd:RDSR err:%d\n", err);
		return err;
	}

	cmd = MB85RS_OP_WREN;
	err = spi_write(spi, &cmd, 1);
	if (err < 0) {
		dev_err(&spi->dev, "cmd:WREN err:%d\n", err);
		return err;
	}

	cmd_ws[0] = MB85RS_OP_WRSR;
	cmd_ws[1] = (status_reg & ~(0xc)) | (bp << 2);
	err = spi_write(spi, cmd_ws, 2);
	if (err < 0) {
		dev_err(&spi->dev, "cmd:WRSR err:%d\n", err);
		return err;
	}

	cmd = MB85RS_OP_WRDI;
	err = spi_write(spi, &cmd, 1);
	if (err < 0) {
		dev_err(&spi->dev, "cmd:WRDI err:%d\n", err);
	}

	return err;
}

static u8 mb85rs_get_blk_protect(struct spi_device *spi)
{
	int err;
	u8 status_reg = 0;
	u8 cmd = MB85RS_OP_RDSR;
	err = spi_write_then_read(spi, &cmd, 1, &status_reg, 1);
	if (err < 0) {
		dev_err(&spi->dev, "cmd:RDSR err:%d\n", err);
		return -1;
	}
	return ((status_reg & 0xc) >> 2);
}

static u32 mb85rs_readid(struct spi_device *spi, u8 *mid, u8 *pid1, u8 *pid2)
{
	int err;
	u8 cmd = MB85RS_OP_RDID;
	u8 buf[4] = {0};
	err = spi_write_then_read(spi, &cmd, 1, buf, 4);
	*mid = buf[0];
	*pid1 = buf[2];
	*pid2 = buf[3];
	return err;
}

static void mb85rs_write_protect(struct mb85rs_data *mb85rs, int enable)
{
	if (enable)
		gpio_set_value(mb85rs->wp_gpio, 0);
	else
		gpio_set_value(mb85rs->wp_gpio, 1);
}

static int mb85rs_dt_read(struct device *dev, int *pwp_gpio)
{
	struct gpio_desc *wp_gpiod = devm_gpiod_get(dev, "wp", GPIOD_OUT_LOW);

	if (!wp_gpiod)
		return -ENODEV;
	*pwp_gpio = desc_to_gpio(wp_gpiod);
	gpiod_put(wp_gpiod);
	dev_warn(dev, "WP_GPIO = %d\n", *pwp_gpio);
	return 0;
}

static int mb85rs_probe(struct spi_device *spi)
{
	struct mb85rs_data *mb85rs = NULL;
	int err;
	u8 mid, pid1, pid2;
	int wp_gpio;
	u8 bp;

	if (!spi->dev.platform_data) {
		err = mb85rs_dt_read(&spi->dev, &wp_gpio);
		if (err) {
			dev_err(&spi->dev, "need the platform data(wp gpio)\n");
			return err;
		}
	} else {
		wp_gpio = *(int *)spi->dev.platform_data;
	}

	err = gpio_request(wp_gpio, "mb85rs_wp");
	if (err) {
		dev_err(&spi->dev, "failed req gpio %d\n", wp_gpio);
		goto fail;
	}

	mb85rs = kzalloc(sizeof *mb85rs, GFP_KERNEL);
	if (!mb85rs) {
		dev_err(&spi->dev, "failed to memory alloc\n");
		err = -ENOMEM;
		goto fail;
	}

	mb85rs->wp_gpio = wp_gpio;
	gpio_direction_output(mb85rs->wp_gpio, 0);

	err = mb85rs_readid(spi, &mid, &pid1, &pid2);
	if (err) {
		dev_err(&spi->dev, "read id error %d\n", err);
		err = -ENXIO;
		goto fail;
	}

	if (mid != FUJITSU_MID){
		dev_err(&spi->dev, "wrong fugitsu manufacturer id 0x%x\n", mid);
		err = -ENXIO;
		goto fail;
	}

	mb85rs->buf_sz = 1024;

	switch(pid1) {
	case PID_MB85RS16:
		mb85rs->bin.size = SZ_2K;
		mb85rs->addr_len = 2;
		break;
	case PID_MB85RS64V:
		mb85rs->bin.size = SZ_8K;
		mb85rs->addr_len = 2;
		break;
	case PID_MB85RS128B:
		mb85rs->bin.size = SZ_16K;
		mb85rs->addr_len = 2;
		break;
	case PID_MB85RS256B:
		mb85rs->bin.size = SZ_32K;
		mb85rs->addr_len = 2;
		break;
	case PID_MB85RS512T:
		mb85rs->bin.size = SZ_64K;
		mb85rs->addr_len = 2;
		break;
	case PID_MB85RS1MT:
		mb85rs->bin.size = SZ_128K;
		mb85rs->addr_len = 3;
		break;
	case PID_MB85RS2MT:
		mb85rs->bin.size = SZ_256K;
		mb85rs->addr_len = 3;
		break;
		break;
	default :
		dev_err(&spi->dev, "wrong pid1 0x%x\n", pid1);
		err = -ENXIO;
		goto fail;
	}

	mutex_init(&mb85rs->lock);
	mb85rs->spi = spi_dev_get(spi);
	spi_set_drvdata(spi, mb85rs);
	sysfs_bin_attr_init(&mb85rs->bin);
	mb85rs->bin.attr.name = "mb85rs_fram";
	mb85rs->bin.attr.mode = 0666;
	mb85rs->bin.read = mb85rs_bin_read;
	mb85rs->bin.write = mb85rs_bin_write;

	err = sysfs_create_bin_file(&spi->dev.kobj, &mb85rs->bin);
	if (err)
		goto fail;

	mb85rs_write_protect(mb85rs, 0);

	bp = mb85rs_get_blk_protect(mb85rs->spi);
	if (bp != MB85RS_BLOCKPROTECT_NONE) {
		/* force release block protect */
		mb85rs_set_block_protect(mb85rs->spi, MB85RS_BLOCKPROTECT_NONE);
	}

	dev_info(&spi->dev, "probe mb85rs spi fram %d bytes", mb85rs->bin.size);
	return 0;
fail:
	dev_err(&spi->dev, "probe err %d\n", err);
	kfree(mb85rs);
	return err;
}

static int mb85rs_remove(struct spi_device *spi)
{
	struct mb85rs_data *mb85rs;
	mb85rs = spi_get_drvdata(spi);
	sysfs_remove_bin_file(&spi->dev.kobj, &mb85rs->bin);
	kfree(mb85rs);
	return 0;
}

static struct spi_driver mb85rs_spi_driver = {
	.driver = {
		.name = "mb85rs",
		.owner = THIS_MODULE,
	},
	.probe = mb85rs_probe,
	.remove = mb85rs_remove,
};

module_spi_driver(mb85rs_spi_driver);

MODULE_DESCRIPTION("Driver for Fujitsu mb85rs series FRAM");
MODULE_AUTHOR("JongHo Kim <furmuwon@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:mb85rs");
