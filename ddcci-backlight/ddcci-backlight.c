/*
 *  DDC/CI monitor backlight driver
 *
 *  Copyright (c) 2015 Christoph Grenz
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/fb.h>

#include <linux/ddcci.h>


#define DDCCI_COMMAND_READ	0x01	/* read ctrl value */
#define DDCCI_REPLY_READ	0x02	/* read ctrl value reply */
#define DDCCI_COMMAND_WRITE	0x03	/* write ctrl value */
#define DDCCI_COMMAND_SAVE	0x0c	/* save current settings */

#define DDCCI_MONITOR_LUMINANCE	0x10
#define DDCCI_MONITOR_BACKLIGHT	0x13
#define DDCCI_MONITOR_BL_WHITE		0x6B

struct ddcci_monitor_drv_data {
	struct ddcci_device *device;
	struct backlight_device *bl_dev;
	struct device *fb_dev;
	unsigned char used_vcp;
};

static int ddcci_monitor_writectrl(struct ddcci_device *device,
				   unsigned char ctrl, unsigned short value)
{
	unsigned char buf[4];
	int ret;

	buf[0] = DDCCI_COMMAND_WRITE;
	buf[1] = ctrl;
	buf[2] = (value >> 8);
	buf[3] = (value & 255);

	ret = ddcci_device_write(device, true, buf, sizeof(buf));

	return ret;
}

static int ddcci_monitor_readctrl(struct ddcci_device *device,
				  unsigned char ctrl, unsigned short *value,
				  unsigned short *maximum)
{
	int ret;
	unsigned char buf[10];

	buf[0] = DDCCI_COMMAND_READ;
	buf[1] = ctrl;

	ret = ddcci_device_writeread(device, true, buf, 2, sizeof(buf));
	if (ret < 0)
		return ret;

	if (ret == 0)
		return -ENOTSUPP;

	if (ret == 8 && buf[0] == DDCCI_REPLY_READ && buf[2] == ctrl) {
		if (value)
			*value = buf[6] * 256 + buf[7];

		if (maximum)
			*maximum = buf[4] * 256 + buf[5];

		if (buf[1] == 1)
			return -ENOTSUPP;
		if (buf[1] != 0)
			return -EIO;
		return 0;
	}

	return -EIO;
}

static int ddcci_backlight_check_fb(struct backlight_device *bl,
				   struct fb_info *info)
{
	struct ddcci_monitor_drv_data *drv_data = bl_get_data(bl);

	return drv_data->fb_dev == NULL || drv_data->fb_dev == info->dev;
}

static int ddcci_backlight_update_status(struct backlight_device *bl)
{
	struct ddcci_monitor_drv_data *drv_data = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int ret;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	ret = ddcci_monitor_writectrl(drv_data->device, drv_data->used_vcp,
				      brightness);
	if (ret > 0)
		ret = 0;
	return ret;
}

static int ddcci_backlight_get_brightness(struct backlight_device *bl)
{
	unsigned short value = 0, maxval = 0;
	int ret;
	struct ddcci_monitor_drv_data *drv_data = bl_get_data(bl);

	ret = ddcci_monitor_readctrl(drv_data->device, drv_data->used_vcp,
				     &value, &maxval);
	if (ret < 0)
		return ret;

	bl->props.brightness = value;
	bl->props.max_brightness = maxval;
	ret = value;

	return ret;
}

static const struct backlight_ops ddcci_backlight_ops = {
	.options	= 0,
	.update_status	= ddcci_backlight_update_status,
	.get_brightness = ddcci_backlight_get_brightness,
	.check_fb	= ddcci_backlight_check_fb,
};

static const char *ddcci_monitor_vcp_name(unsigned char vcp)
{
	switch (vcp) {
		case DDCCI_MONITOR_BL_WHITE:
			return "backlight";
		case DDCCI_MONITOR_LUMINANCE:
			return "luminance";
		default:
			return "???";
	}
}

static int ddcci_monitor_probe(struct ddcci_device *dev,
			       const struct ddcci_device_id *id)
{
	struct ddcci_monitor_drv_data *drv_data;
	struct backlight_properties props;
	struct backlight_device *bl = NULL;
	int ret = 0;
	unsigned short brightness = 0, max_brightness = 0;

	dev_dbg(&dev->dev, "probing monitor backlight device\n");

	/* Initialize driver data structure */
	drv_data = devm_kzalloc(&dev->dev, sizeof(struct ddcci_monitor_drv_data),
				GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;
	drv_data->device = dev;

	/* Try getting backlight level */
	ret = ddcci_monitor_readctrl(drv_data->device, DDCCI_MONITOR_BL_WHITE,
				     &brightness, &max_brightness);
	if (ret < 0) {
		if (ret == -ENOTSUPP)
			dev_info(&dev->dev,
				 "monitor does not support reading backlight level\n");
		else
			goto err_free;
	} else {
		drv_data->used_vcp = DDCCI_MONITOR_BL_WHITE;
	}

	if (!drv_data->used_vcp) {
		/* Try getting luminance */
		ret = ddcci_monitor_readctrl(drv_data->device, DDCCI_MONITOR_LUMINANCE,
					     &brightness, &max_brightness);
		if (ret < 0) {
			if (ret == -ENOTSUPP)
				dev_info(&dev->dev,
					"monitor does not support reading luminance\n");
			else
				goto err_free;
		} else {
			drv_data->used_vcp = DDCCI_MONITOR_LUMINANCE;
		}
		drv_data->used_vcp = DDCCI_MONITOR_LUMINANCE;
	}

	if (!drv_data->used_vcp)
		goto err_free;

	/* Create brightness device */
	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = max_brightness;
	props.brightness = brightness;
	bl = devm_backlight_device_register(&dev->dev, dev_name(&dev->dev),
					    &dev->dev, drv_data,
					    &ddcci_backlight_ops, &props);
	drv_data->bl_dev = bl;
	if (IS_ERR(bl)) {
		dev_err(&dev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}
	dev_info(&dev->dev, "registered %s as backlight device %s\n",
		 ddcci_monitor_vcp_name(drv_data->used_vcp),
		 dev_name(&dev->dev));

	goto end;
err_free:
	devm_kfree(&dev->dev, drv_data);
end:
	return ret;
}

static int ddcci_monitor_remove(struct ddcci_device *dev)
{
	dev_dbg(&dev->dev, "removing device\n");
	return 0;
}

static struct ddcci_device_id ddcci_monitor_idtable[] = {
	{ "monitor", DDCCI_ANY_ID, DDCCI_ANY_ID, DDCCI_ANY_ID, DDCCI_ANY_ID, 0 },
	{}
};

static struct ddcci_driver ddcci_backlight_driver = {
	.driver = {
		.name	= "ddcci-backlight",
		.owner	= THIS_MODULE,
	},

	.id_table	= ddcci_monitor_idtable,
	.probe		= ddcci_monitor_probe,
	.remove		= ddcci_monitor_remove,
};

module_ddcci_driver(ddcci_backlight_driver);

MODULE_AUTHOR("Christoph Grenz");
MODULE_DESCRIPTION("DDC/CI generic monitor backlight driver");
MODULE_VERSION("0.2");
MODULE_LICENSE("GPL");

MODULE_ALIAS("ddcci:monitor-*-*-*-*");


