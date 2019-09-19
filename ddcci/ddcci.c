/*
 *  DDC/CI sub-bus driver
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
#include <asm-generic/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/sem.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <linux/ddcci.h>

#define DDCCI_RECV_BUFFER_SIZE 130
#define DEVICE_NAME "ddcci"
#define DDCCI_MAX_CAP_CHUNKS 200

static unsigned int delay = 60;
static unsigned short autoprobe_addrs[127] = {0xF0, 0xF2, 0xF4, 0xF6, 0xF8};
static int autoprobe_addr_count = 5;

static dev_t ddcci_cdev_first;
static dev_t ddcci_cdev_next;
static dev_t ddcci_cdev_end;
static DEFINE_SEMAPHORE(core_lock);

struct bus_type ddcci_bus_type;
EXPORT_SYMBOL_GPL(ddcci_bus_type);

/* Internal per-i2c-client driver data */
struct ddcci_bus_drv_data {
	unsigned long quirks;
	struct i2c_client *i2c_dev;
	struct semaphore sem;
	unsigned char recv_buffer[DDCCI_RECV_BUFFER_SIZE];
};

/* Write a message to the DDC/CI bus using i2c_smbus_write_byte() */
static int __ddcci_write_bytewise(struct i2c_client *client, unsigned char addr,
				  bool p_flag, const unsigned char *buf,
				  unsigned char len)
{
	int ret = 0;
	unsigned char outer_addr = (unsigned char)(client->addr << 1);
	unsigned xor = outer_addr; /* initial xor value */

	/* Consistency checks */
	if (len > 127)
		return -EINVAL;

	/* Special case: sender to 0x6E is always 0x51 */
	if (addr == DDCCI_DEFAULT_DEVICE_ADDR) {
		addr = DDCCI_HOST_ADDR_ODD;
	} else {
		/* When sending the odd address is used */
		addr = addr | 1;
	}

	/* first byte: sender address */
	xor ^= addr;
	ret = i2c_smbus_write_byte(client, addr);
	if (ret < 0)
		return ret;

	/* second byte: protocol flag and message size */
	xor ^= ((p_flag << 7) | len);
	ret = i2c_smbus_write_byte(client, (p_flag << 7)|len);
	if (ret < 0)
		return ret;

	/* send payload */
	while (len--) {
		xor ^= (*buf);
		ret = i2c_smbus_write_byte(client, (*buf));
		if (ret < 0)
			return ret;
		buf++;
	}

	/* send checksum */
	ret = i2c_smbus_write_byte(client, xor);
	return ret;
}

/* Write a message to the DDC/CI bus using i2c_master_send() */
static int __ddcci_write_block(struct i2c_client *client, unsigned char addr,
			       unsigned char *sendbuf, bool p_flag,
			       const unsigned char *data, unsigned char len)
{
	unsigned char outer_addr = (unsigned char)(client->addr << 1);
	unsigned xor = outer_addr;	/* initial xor value */
	unsigned char *ptr = sendbuf;

	/* Consistency checks */
	if (len > 127)
		return -EINVAL;

	/* Special case: sender to 0x6E is always 0x51 */
	if (addr == DDCCI_DEFAULT_DEVICE_ADDR) {
		addr = DDCCI_HOST_ADDR_ODD;
	} else {
		/* When sending the odd address is used */
		addr = addr | 1;
	}

	/* first byte: sender address */
	xor ^= addr;
	*(ptr++) = addr;
	/* second byte: protocol flag and message size */
	xor ^= ((p_flag << 7) | len);
	*(ptr++) = (p_flag << 7)|len;
	/* payload */
	while (len--) {
		xor ^= (*data);
		*(ptr++) = (*data);
		data++;
	}
	/* checksum */
	(*ptr) = xor;

	/* Send it */
	return i2c_master_send(client, sendbuf, ptr - sendbuf + 1);
}

/*
 * Write a message to the DDC/CI bus.
 *
 * You must hold the bus semaphore when calling this function.
 */
static int ddcci_write(struct i2c_client *client, unsigned char addr,
		       bool p_flag, const unsigned char *data,
		       unsigned char len)
{
	struct ddcci_bus_drv_data *drv_data;
	unsigned char *sendbuf;
	int ret;

	drv_data = i2c_get_clientdata(client);


	pr_debug("sending to %d:%02x:%02x: %*ph\n", client->adapter->nr,
		 client->addr << 1, addr, len, data);
	if (drv_data->quirks & DDCCI_QUIRK_WRITE_BYTEWISE) {
		ret = __ddcci_write_bytewise(client, addr, p_flag, data, len);
	} else {
		sendbuf = drv_data->recv_buffer;
		ret = __ddcci_write_block(client, addr, sendbuf, p_flag, data, len);
	}

	return ret;
}

/*
 * Read a response from the DDC/CI bus with headers directly into a buffer.
 * Always check for DDCCI_QUIRK_SKIP_FIRST_BYTE when using this function.
 * The returned length contains the whole unmodified response.
 * If -EMSGSIZE is returned, the buffer contains the response up to `len`.
 * If any other negative error code is returned, the buffer content is
 * unspecified.
 */
static int __ddcci_read(struct i2c_client *client, unsigned char addr,
			bool p_flag, unsigned long quirks, unsigned char *buf,
			unsigned char len)
{
	int i, payload_len, packet_length, ret;
	unsigned char xor = DDCCI_HOST_ADDR_EVEN;

	/* Consistency checks */
	if (len < 3)
		return -EINVAL;

	/* Read frame */
	ret = i2c_master_recv(client, buf, len);
	if (ret < 0)
		goto out_err;
	packet_length = ret;

	/* Skip first byte if quirk active */
	if ((quirks & DDCCI_QUIRK_SKIP_FIRST_BYTE) && ret > 0 && len > 0) {
		ret--;
		len--;
		buf++;
	}

	/* If answer too short (= incomplete) break out */
	if (ret < 3) {
		ret = -EIO;
		goto out_err;
	}

	/* validate first byte */
	if (unlikely(buf[0] != addr)) {
		ret = (buf[0] == '\0') ? -EAGAIN : -EIO;
		goto out_err;
	}

	/* validate second byte (protocol flag) */
	if (unlikely((buf[1] & 0x80) != (p_flag << 7))) {
		if (!p_flag || !(quirks & DDCCI_QUIRK_NO_PFLAG)) {
			ret = -EIO;
			goto out_err;
		}
	}

	/* get and check payload length */
	payload_len = buf[1] & 0x7F;
	if (3+payload_len > packet_length)
		return -EBADMSG;
	if (3+payload_len > len)
		return -EMSGSIZE;

	/* calculate checksum */
	for (i = 0; i < 3+payload_len; i++)
		xor ^= buf[i];

	/* verify checksum */
	if (xor != 0) {
		dev_err(&client->dev, "invalid DDC/CI response, corrupted data - xor is 0x%02x, length 0x%02x\n",
			xor, payload_len);
		ret = -EBADMSG;
		goto out_err;
	}

	/* return result */
	ret = payload_len+3+((quirks & DDCCI_QUIRK_SKIP_FIRST_BYTE)?1:0);

out_err:
	return ret;
}

/*
 * Read a response from the DDC/CI bus
 *
 * You must hold the bus semaphore when calling this function.
 */
static int ddcci_read(struct i2c_client *client, unsigned char addr,
		      bool p_flag, unsigned char *buf, unsigned char len)
{
	struct ddcci_bus_drv_data *drv_data;
	unsigned char *recvbuf;
	int ret;

	drv_data = i2c_get_clientdata(client);
	recvbuf = drv_data->recv_buffer;

	/* Read frame */
	ret = __ddcci_read(client, addr, p_flag,
		drv_data->quirks, recvbuf, DDCCI_RECV_BUFFER_SIZE);
	if (ret < 0)
		return ret;

	if (drv_data->quirks & DDCCI_QUIRK_SKIP_FIRST_BYTE)
		recvbuf++;

	/* return result */
	if (buf) {
		if (ret > 3) {
			ret = ret-3;
			/* copy to caller buffer */
			memcpy(buf, &recvbuf[2], (ret < len) ? ret : len);

			if (ret > len) {
				/* if message was truncated, return -EMSGSIZE */
				pr_debug("received from %d:%02x:%02x: [%u/%u] %*ph ...\n",
					 client->adapter->nr, client->addr << 1,
					 addr, ret, len, len, buf);
				ret = -EMSGSIZE;
			} else {
				pr_debug("received from %d:%02x:%02x: [%u/%u] %*ph\n",
					 client->adapter->nr, client->addr << 1,
					 addr, ret, len, ret, buf);
			}
		}
	}
	if (!(drv_data->quirks & DDCCI_QUIRK_WRITE_BYTEWISE)) {
		/* second read to clear buffers, needed on some devices */
		__ddcci_read(client, addr, true, drv_data->quirks, recvbuf, 1);
	}
	return ret;
}

/* Request the capability string for a device and put it into buf */
static int ddcci_get_caps(struct i2c_client *client, unsigned char addr,
			  unsigned char *buf, unsigned int len)
{
	int result = 0, counter = 0, offset = 0;
	unsigned char cmd[3] = { DDCCI_COMMAND_CAPS, 0x00, 0x00};
	unsigned char *chunkbuf = kzalloc(35, GFP_KERNEL);

	if (!chunkbuf)
		return -ENOMEM;

	do {
		/* Send command */
		result = ddcci_write(client, addr, true, cmd, sizeof(cmd));
		if (result < 0)
			goto err_free;
		msleep(delay);
		/* read result chunk */
		result = ddcci_read(client, addr, true, chunkbuf,
				    (len > 32) ? 35 : len+3);
		if (result < 0)
			goto err_free;

		if (result > 0) {
			/* check chunk header */
			if (chunkbuf[0] != DDCCI_REPLY_CAPS) {
				result = -EIO;
				goto err_free;
			}
			if (chunkbuf[1] != cmd[1] || chunkbuf[2] != cmd[2]) {
				result = -EIO;
				goto err_free;
			}
			memcpy(buf, chunkbuf+3, result-3);

			counter++;
			/* adjust offset, etc. */
			offset += result-3;
			len -= result-3;
			buf += result-3;
			cmd[1] = offset >> 8;
			cmd[2] = offset & 0xFF;
			/* Another superfluous read to make some devices happy... */
			ddcci_read(client, addr, true, NULL, 2);
		}
	} while (result > 3 && counter < DDCCI_MAX_CAP_CHUNKS);

	kfree(chunkbuf);
	return offset+result-3;
err_free:
	kfree(chunkbuf);
	return result;
}

/*
 * Request the device identification and put it into buf.
 *
 * Also detects all communication quirks and sets the corresponding flags
 * in the ddcci_bus_drv_data structure associated with client.
 *
 * The identification command will fail on most DDC devices, as it is optional
 * to support, but even the "failed" response suffices to detect quirks.
 */
static int ddcci_identify_device(struct i2c_client *client, unsigned char addr,
				 unsigned char *buf, unsigned char len)
{
	int i, payload_len, ret = -ENODEV;
	unsigned long quirks;
	unsigned char cmd[2] = { DDCCI_COMMAND_ID, 0x00 };
	unsigned char *buffer;
	unsigned char xor = DDCCI_HOST_ADDR_EVEN;
	struct ddcci_bus_drv_data *bus_drv_data;

	bus_drv_data = i2c_get_clientdata(client);
	quirks = bus_drv_data->quirks;
	buffer = bus_drv_data->recv_buffer;

	/* Send Identification command */
	if (!(quirks & DDCCI_QUIRK_WRITE_BYTEWISE)) {
		ret = __ddcci_write_block(client, addr, buffer, true, cmd, 1);
		dev_dbg(&client->dev,
			"[%02x:%02x] writing identification command in block mode: %d\n",
			client->addr << 1, addr, ret);
		if ((ret == -ENXIO)
		    && i2c_check_functionality(client->adapter,
					       I2C_FUNC_SMBUS_WRITE_BYTE)) {
			quirks |= DDCCI_QUIRK_WRITE_BYTEWISE;
			dev_info(&client->dev,
				"DDC/CI bus quirk detected: writes must be done bytewise\n");
			/* Some devices need writing twice after a failed blockwise write */
			__ddcci_write_bytewise(client, addr, true, cmd, 2);
			msleep(delay);
		}
	}
	if (ret < 0 && (quirks & DDCCI_QUIRK_WRITE_BYTEWISE)) {
		ret = __ddcci_write_bytewise(client, addr, true, cmd, 2);
		dev_dbg(&client->dev,
			"[%02x:%02x] writing identification command in bytewise mode: %d\n",
			client->addr << 1, addr, ret);
	}
	if (ret < 0)
		return -ENODEV;

	/* Wait */
	msleep(delay);

	/* Receive response */
	ret = i2c_master_recv(client, buffer, DDCCI_RECV_BUFFER_SIZE);
	if (ret < 0) {
		dev_dbg(&client->dev,
			"[%02x:%02x] receiving identification response resulted in errno %d\n",
			client->addr << 1, addr, ret);
		return ret;
	}

	if (ret == 0) {
		dev_dbg(&client->dev,
			"[%02x:%02x] no identification response received\n",
			client->addr << 1, addr);
		return ret;
	}

	/* Skip first byte if quirk already active */
	if (quirks & DDCCI_QUIRK_SKIP_FIRST_BYTE && ret > 1) {
		dev_dbg(&client->dev,
			"[%02x:%02x] doubled first byte quirk in effect\n",
			client->addr << 1, addr);
		ret--;
		buffer++;
	}

	/* If answer too short (= incomplete) break out */
	if (ret < 3) {
		dev_dbg(&client->dev,
			"[%02x:%02x] identification response is too short (%d bytes)\n",
			client->addr << 1, addr, ret);
		return -EIO;
	}

	/* validate first byte */
	if (buffer[0] != addr) {
		dev_dbg(&client->dev,
			"[%02x:%02x] identification response: %*ph\n",
			client->addr << 1, addr, (ret > 32 ? 32 : ret), buffer);

		dev_dbg(&client->dev,
			"[%02x:%02x] identification response invalid (expected first byte %02x, got %02x)\n",
			client->addr << 1, addr, addr, buffer[0]);
		return -ENODEV;
	}

	/* Check if first byte is doubled (QUIRK_SKIP_FIRST_BYTE) */
	if (!(quirks & DDCCI_QUIRK_SKIP_FIRST_BYTE)) {
		if (buffer[0] == buffer[1]) {
			quirks |= DDCCI_QUIRK_SKIP_FIRST_BYTE;
			dev_info(&client->dev,
				"DDC/CI bus quirk detected: doubled first byte on read\n");
			ret--;
			buffer++;
			if (ret < 3)
				return -EIO;
		}
	}

	/* validate second byte (protocol flag) */
	if ((buffer[1] & 0x80) != 0x80 && !(quirks & DDCCI_QUIRK_NO_PFLAG)) {
		dev_info(&client->dev,
			"DDC/CI bus quirk detected: device omits protocol flag on responses\n");
		quirks |= DDCCI_QUIRK_NO_PFLAG;
	}

	/* get and check payload length */
	payload_len = buffer[1] & 0x7F;
	if (3+payload_len > ret) {
		dev_dbg(&client->dev,
			"[%02x:%02x] identification response: %*ph ...\n",
			client->addr << 1, addr, ret, buffer);
		dev_dbg(&client->dev,
			"[%02x:%02x] identification response was truncated (expected %d bytes, got %d)\n",
			client->addr << 1, addr, 3+payload_len, ret);
		return -EBADMSG;
	}

	dev_dbg(&client->dev,
		"[%02x:%02x] identification response: %*ph\n",
		client->addr << 1, addr, 3+payload_len, buffer);

	/* calculate checksum */
	for (i = 0; i < 3+payload_len; i++)
		xor ^= buffer[i];

	/* verify checksum */
	if (xor != 0) {
		dev_err(&client->dev,
			"[%02x:%02x] invalid DDC/CI response, corrupted data - xor is 0x%02x, length 0x%02x\n",
			client->addr << 1, addr, xor, payload_len);
		return -EBADMSG;
	}

	/* save quirks */
	bus_drv_data->quirks = quirks;

	/* return result */
	if (payload_len <= len) {
		ret = payload_len;
		memcpy(buf, &buffer[2], payload_len);
	} else {
		ret = -EMSGSIZE;
		memcpy(buf, &buffer[2], len);
	}
	return ret;
}

/* Character device */

/* Data structure for an open file handle */
struct ddcci_fp_data {
	struct ddcci_device *dev;
	bool exclusive;
	unsigned char buffer[129];
};

/* Called when the character device is opened */
static int ddcci_cdev_open(struct inode *inode, struct file *filp)
{
	struct ddcci_device *dev = container_of(inode->i_cdev,
						struct ddcci_device, cdev);
	struct ddcci_fp_data *fp_data = NULL;

	fp_data = kzalloc(sizeof(struct ddcci_fp_data), GFP_KERNEL);

	if (!fp_data)
		return -ENOMEM;

	fp_data->exclusive = filp->f_flags & O_EXCL;

	if (fp_data->exclusive) {
		if (down_write_trylock(&dev->cdev_sem) == 0) {
			kfree(fp_data);
			return -EBUSY;
		}
	} else {
		if (down_read_trylock(&dev->cdev_sem) == 0) {
			kfree(fp_data);
			return -EBUSY;
		}
	}

	fp_data->dev = dev;
	filp->private_data = fp_data;

	return 0;
}

/* Called when the character device is closed */
static int ddcci_cdev_close(struct inode *inode, struct file *filp)
{
	struct ddcci_fp_data *fp_data = filp->private_data;
	struct ddcci_device *dev = fp_data->dev;

	if (fp_data->exclusive)
		up_write(&dev->cdev_sem);
	else
		up_read(&dev->cdev_sem);

	filp->private_data = NULL;
	kfree(fp_data);
	return 0;
}

/* Called when reading from the character device */
static ssize_t ddcci_cdev_read(struct file *filp, char __user *buffer,
			       size_t length, loff_t *offset)
{
	struct ddcci_fp_data *fp_data = filp->private_data;
	struct ddcci_device *dev = fp_data->dev;
	unsigned char *buf = fp_data->buffer;
	const bool nonblocking = (filp->f_flags & O_NONBLOCK) != 0;
	int ret = 0;

	if ((filp->f_mode & FMODE_READ) == 0)
		return -EBADF;

	/* Lock mutex */
	if (nonblocking) {
		if (down_trylock(&dev->bus_drv_data->sem))
			return -EAGAIN;
	} else {
		if (down_interruptible(&dev->bus_drv_data->sem))
			return -ERESTARTSYS;
	}

	/* Execute read */
	ret = ddcci_read(dev->bus_drv_data->i2c_dev, dev->inner_addr, true, buf,
			 length);

	if (ret > 0) {
		/* Copy data from user space */
		if (copy_to_user(buffer, buf, ret)) {
			ret = -EFAULT;
			goto out;
		}
	}

out:
	up(&dev->bus_drv_data->sem);
	return ret;
}

/* Called when writing to the character device */
static ssize_t ddcci_cdev_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *offset)
{
	struct ddcci_fp_data *fp_data = filp->private_data;
	struct ddcci_device *dev = fp_data->dev;
	unsigned char *buf = fp_data->buffer;
	const bool nonblocking = (filp->f_flags & O_NONBLOCK) != 0;
	int ret = 0;

	if ((filp->f_mode & FMODE_WRITE) == 0)
		return -EBADF;

	if (count > 127)
		return -EINVAL;

	/* Lock mutex */
	if (nonblocking) {
		if (down_trylock(&dev->bus_drv_data->sem))
			return -EAGAIN;
	} else {
		if (down_interruptible(&dev->bus_drv_data->sem))
			return -ERESTARTSYS;
	}

	if (count > 0) {
		/* Copy data from user space */
		if (copy_from_user(buf, buffer, count)) {
			ret = -EFAULT;
			goto err_out;
		}

		/* Execute write */
		ret = ddcci_write(dev->bus_drv_data->i2c_dev, dev->inner_addr,
				  true, buf, count);
	}

	if (ret >= 0) {
		msleep(delay);
		up(&dev->bus_drv_data->sem);
		return count;
	}

err_out:
	up(&dev->bus_drv_data->sem);
	return ret;
}

/* Called when seeking the character device */
static loff_t ddcci_cdev_seek(struct file *filp, loff_t offset, int anchor)
{
	return -EINVAL;
}

static const struct file_operations ddcci_fops = {
	.owner = THIS_MODULE,
	.read = ddcci_cdev_read,
	.write = ddcci_cdev_write,
	.open = ddcci_cdev_open,
	.release = ddcci_cdev_close,
	.llseek = ddcci_cdev_seek
};

/* Set up the character device for a DDC/CI device */
static int ddcci_setup_char_device(struct ddcci_device *device)
{
	int ret = -EINVAL;

	/* Check if free minor exists */
	if (ddcci_cdev_next == ddcci_cdev_end) {
		dev_err(&device->dev, "no free major/minor\n");
		ret = -ENFILE;
		goto out;
	}

	/* Initialize rwsem */
	init_rwsem(&device->cdev_sem);

	/* Initialize character device node */
	cdev_init(&device->cdev, &ddcci_fops);
	device->cdev.owner = THIS_MODULE;

	/* Publish char device */
	device->dev.devt = ddcci_cdev_next;
	ret = cdev_add(&device->cdev, ddcci_cdev_next, 1);
	if (ret) {
		device->dev.devt = 0;
		goto out;
	}

	ddcci_cdev_next++;
out:
	return ret;
}

/* sysfs attributes */

static ssize_t ddcci_attr_capabilities_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;
	size_t len;

	if (likely(device != NULL)) {
		len = device->capabilities_len;
		if (unlikely(len > PAGE_SIZE))
			len = PAGE_SIZE;
		if (len == 0) {
			ret = len;
		} else {
			memcpy(buf, device->capabilities, len);
			if (likely(len < PAGE_SIZE)) {
				buf[len] = '\n';
				ret = len+1;
			}
		}
	}

	return ret;
}

static ssize_t ddcci_attr_prot_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;
	size_t len;

	if (likely(device != NULL)) {
		len = strlen(device->prot);
		strncpy(buf, device->prot, PAGE_SIZE);
		if (len == 0) {
			ret = len;
		} else if (likely(len < PAGE_SIZE)) {
			buf[len] = '\n';
			ret = len+1;
		} else {
			ret = PAGE_SIZE;
		}
	}
	return ret;
}

static ssize_t ddcci_attr_type_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;
	size_t len;

	if (likely(device != NULL)) {
		len = strlen(device->type);
		strncpy(buf, device->type, PAGE_SIZE);
		if (len == 0) {
			ret = len;
		} else if (likely(len < PAGE_SIZE)) {
			buf[len] = '\n';
			ret = len+1;
		} else {
			ret = PAGE_SIZE;
		}
	}
	return ret;
}

static ssize_t ddcci_attr_model_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;
	size_t len;

	if (likely(device != NULL)) {
		len = strlen(device->model);
		strncpy(buf, device->model, PAGE_SIZE);
		if (len == 0) {
			ret = len;
		} else if (likely(len < PAGE_SIZE)) {
			buf[len] = '\n';
			ret = len+1;
		} else {
			ret = PAGE_SIZE;
		}
	}
	return ret;
}

static ssize_t ddcci_attr_vendor_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;
	size_t len;

	if (likely(device != NULL)) {
		len = strlen(device->vendor);
		strncpy(buf, device->vendor, PAGE_SIZE);
		if (len == 0) {
			ret = len;
		} else if (likely(len < PAGE_SIZE)) {
			buf[len] = '\n';
			ret = len+1;
		} else {
			ret = PAGE_SIZE;
		}
	}
	return ret;
}

static ssize_t ddcci_attr_module_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;
	size_t len;

	if (likely(device != NULL)) {
		len = strlen(device->module);
		strncpy(buf, device->module, PAGE_SIZE);
		if (len == 0) {
			ret = len;
		} else if (likely(len < PAGE_SIZE)) {
			buf[len] = '\n';
			ret = len+1;
		} else {
			ret = PAGE_SIZE;
		}
	}
	return ret;
}

static ssize_t ddcci_attr_serial_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct ddcci_device *device = ddcci_verify_device(dev);
	ssize_t ret = -ENOENT;

	if (likely(device != NULL))
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", device->device_number);

	return ret;
}

static DEVICE_ATTR(capabilities, S_IRUGO, ddcci_attr_capabilities_show, NULL);
static DEVICE_ATTR(idProt, S_IRUGO, ddcci_attr_prot_show, NULL);
static DEVICE_ATTR(idType, S_IRUGO, ddcci_attr_type_show, NULL);
static DEVICE_ATTR(idModel, S_IRUGO, ddcci_attr_model_show, NULL);
static DEVICE_ATTR(idVendor, S_IRUGO, ddcci_attr_vendor_show, NULL);
static DEVICE_ATTR(idModule, S_IRUGO, ddcci_attr_module_show, NULL);
static DEVICE_ATTR(idSerial, S_IRUGO, ddcci_attr_serial_show, NULL);

static struct attribute *ddcci_char_device_attrs[] = {
	&dev_attr_capabilities.attr,
	&dev_attr_idProt.attr,
	&dev_attr_idType.attr,
	&dev_attr_idModel.attr,
	&dev_attr_idVendor.attr,
	&dev_attr_idModule.attr,
	&dev_attr_idSerial.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ddcci_char_device);

/* DDC/CI bus */

static int ddcci_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct ddcci_device	*device = to_ddcci_device(dev);

	if (add_uevent_var(env, "MODALIAS=%s%s-%s-%s-%s-%s",
			   DDCCI_MODULE_PREFIX,
			   device->prot,
			   device->type,
			   device->model,
			   device->vendor,
			   device->module
		))
		return -ENOMEM;

	if (device->prot[0])
		if (add_uevent_var(env, "DDCCI_PROT=%s", device->prot))
			return -ENOMEM;

	if (device->type[0])
		if (add_uevent_var(env, "DDCCI_TYPE=%s", device->type))
			return -ENOMEM;

	if (device->model[0])
		if (add_uevent_var(env, "DDCCI_MODEL=%s", device->model))
			return -ENOMEM;

	if (device->vendor[0]) {
		if (add_uevent_var(env, "DDCCI_VENDOR=%s", device->vendor))
			return -ENOMEM;

		if (add_uevent_var(env, "DDCCI_MODULE=%s", device->module))
			return -ENOMEM;

		if (add_uevent_var(env, "DDCCI_UNIQ=%d", device->device_number))
			return -ENOMEM;
	}

	return 0;
}

static void ddcci_device_release(struct device *dev)
{
	struct ddcci_device *device = to_ddcci_device(dev);
	struct ddcci_driver *driver;

	/* Notify driver */
	if (dev->driver) {
		driver = to_ddcci_driver(dev->driver);
		if (driver->remove)
			driver->remove(device);
	}

	/* Teardown chardev */
	if (dev->devt) {
		down(&core_lock);
		if (device->cdev.dev == ddcci_cdev_next-1)
			ddcci_cdev_next--;
		cdev_del(&device->cdev);
		up(&core_lock);
	}

	/* Free capability string */
	if (device->capabilities) {
		device->capabilities_len = 0;
		kfree(device->capabilities);
	}
	/* Free device */
	kfree(device);
}

static char *ddcci_devnode(struct device *dev,
			 umode_t *mode, kuid_t *uid, kgid_t *gid)
{
	struct ddcci_device *device;

	device = to_ddcci_device(dev);
	return kasprintf(GFP_KERNEL, "bus/ddcci/%d/display",
			 device->i2c_client->adapter->nr);
}

static char *ddcci_dependent_devnode(struct device *dev,
			 umode_t *mode, kuid_t *uid, kgid_t *gid)
{
	struct ddcci_device *device;

	device = to_ddcci_device(dev);
	if (device->flags & DDCCI_FLAG_EXTERNAL) {
		if (device->outer_addr == device->inner_addr)
			return kasprintf(GFP_KERNEL, "bus/ddcci/%d/e%02x",
					 device->i2c_client->adapter->nr,
					 device->outer_addr);
		else
			return kasprintf(GFP_KERNEL, "bus/ddcci/%d/e%02x%02x",
					 device->i2c_client->adapter->nr,
					 device->outer_addr, device->inner_addr);
	} else {
		return kasprintf(GFP_KERNEL, "bus/ddcci/%d/i%02x",
				 device->i2c_client->adapter->nr,
				 device->inner_addr);
	}
}

/* Device type for main DDC/CI devices*/
static struct device_type ddcci_device_type = {
	.name	= "ddcci-device",
	.uevent		= ddcci_device_uevent,
	.groups		= ddcci_char_device_groups,
	.release	= ddcci_device_release,
	.devnode	= ddcci_devnode
};

/* Device type for dependent DDC/CI devices*/
static struct device_type ddcci_dependent_type = {
	.name	= "ddcci-dependent-device",
	.uevent		= ddcci_device_uevent,
	.groups		= ddcci_char_device_groups,
	.release	= ddcci_device_release,
	.devnode	= ddcci_dependent_devnode
};

/**
 * ddcci_verify_device - return parameter as ddcci_device, or NULL
 * @dev: device, probably from some driver model iterator
 */
struct ddcci_device *ddcci_verify_device(struct device *dev)
{
	if (unlikely(!dev))
		return NULL;
	return (dev->type == &ddcci_device_type
		|| dev->type == &ddcci_dependent_type)
			? to_ddcci_device(dev)
			: NULL;
}
EXPORT_SYMBOL(ddcci_verify_device);

/**
 * ddcci_quirks - Get quirks for DDC/CI device
 * @dev: Target DDC/CI device
 */
unsigned long ddcci_quirks(struct ddcci_device *dev)
{
	if (unlikely(WARN_ON(!dev)))
		return ~0L;
	if (unlikely(WARN_ON(!dev->bus_drv_data)))
		return ~0L;
	return dev->bus_drv_data->quirks;
}
EXPORT_SYMBOL(ddcci_quirks);

/**
 * ddcci_register_driver - register DDC/CI driver
 * @owner: the owning module
 * @driver: the driver to register
 */
int ddcci_register_driver(struct module *owner, struct ddcci_driver *driver)
{
	int ret;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!ddcci_bus_type.p)))
		return -EAGAIN;

	pr_debug("registering driver [%s]\n", driver->driver.name);

	/* add the driver to the list of ddcci drivers in the driver core */
	driver->driver.owner = owner;
	driver->driver.bus = &ddcci_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	ret = driver_register(&driver->driver);
	if (ret)
		return ret;

	pr_debug("driver [%s] registered\n", driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(ddcci_register_driver);

/**
 * ddcci_del_driver - unregister DDC/CI driver
 * @driver: the driver being unregistered
 */
void ddcci_del_driver(struct ddcci_driver *driver)
{
	driver_unregister(&driver->driver);
	pr_debug("driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(ddcci_del_driver);

/**
 * ddcci_device_write - Write a message to a DDC/CI device
 * @dev: Target DDC/CI device
 * @p_flag: Protocol flag, true for standard control messages
 * @data: Data that will be written to the device
 * @length: How many bytes to write
 *
 * Writes the message to the device and sleeps (see module parameter 'delay')
 */
int ddcci_device_write(struct ddcci_device *dev, bool p_flag,
		       unsigned char *data, unsigned char length)
{
	int ret;

	if (down_interruptible(&dev->bus_drv_data->sem))
		return -EAGAIN;

	ret = ddcci_write(dev->bus_drv_data->i2c_dev, dev->inner_addr, p_flag, data, length);
	msleep(delay);
	up(&dev->bus_drv_data->sem);
	return ret;
}
EXPORT_SYMBOL(ddcci_device_write);

/**
 * ddcci_device_read - Read a response from a DDC/CI device
 * @dev: Target DDC/CI device
 * @p_flag: Protocol flag, must match the corresponding write
 * @buffer: Where to store data read from the device
 * @length: Buffer size
 */
int ddcci_device_read(struct ddcci_device *dev, bool p_flag,
		      unsigned char *buffer, unsigned char length)
{
	int ret;

	if (down_interruptible(&dev->bus_drv_data->sem))
		return -EAGAIN;

	ret = ddcci_read(dev->bus_drv_data->i2c_dev, dev->inner_addr, p_flag, buffer, length);
	up(&dev->bus_drv_data->sem);
	return ret;
}
EXPORT_SYMBOL(ddcci_device_read);

/**
 * ddcci_device_writeread - Write a message to a device and read the response
 * @dev: Target DDC/CI device
 * @p_flag: Protocol flag, true for standard control messages
 * @buffer: Buffer used for write and read
 * @length: How many bytes to write
 * @maxlength: Buffer size on read
 *
 * Writing, sleeping and reading are done without releasing the DDC/CI bus.
 * This provides atomicity in respect to other DDC/CI accesses on the same bus.
 */
int ddcci_device_writeread(struct ddcci_device *dev, bool p_flag,
			   unsigned char *buffer, unsigned char length,
			   unsigned char maxlength)
{
	int ret;

	if (down_interruptible(&dev->bus_drv_data->sem))
		return -EAGAIN;

	ret = ddcci_write(dev->bus_drv_data->i2c_dev, dev->inner_addr, p_flag, buffer, length);
	if (ret < 0)
		goto err;
	msleep(delay);
	ret = ddcci_read(dev->bus_drv_data->i2c_dev, dev->inner_addr, p_flag, buffer, maxlength);
err:
	up(&dev->bus_drv_data->sem);
	return ret;
}
EXPORT_SYMBOL(ddcci_device_writeread);

#define IS_ANY_ID(x) (((x)[0] == -1) && ((x)[7] == -1))

/* Check if any device id in the array matches the device and return the matching id */
static const struct ddcci_device_id *ddcci_match_id(const struct ddcci_device_id *id,
						    const struct ddcci_device *device)
{
	while (id->prot[0] || id->type[0] || id->model[0] || id->vendor[0] || id->module[0]) {
		if ((IS_ANY_ID(id->prot) || (strcmp(device->prot, id->prot) == 0))
		 && (IS_ANY_ID(id->type) || (strcmp(device->type, id->type) == 0))
		 && (IS_ANY_ID(id->model) || (strcmp(device->model, id->model) == 0))
		 && (IS_ANY_ID(id->vendor) || (strcmp(device->vendor, id->vendor) == 0))
		 && (IS_ANY_ID(id->module) || (strcmp(device->module, id->module) == 0))) {
			return id;
		}
		id++;
	}
	return NULL;
}

static int ddcci_device_match(struct device *dev, struct device_driver *drv)
{
	struct ddcci_device	*device = ddcci_verify_device(dev);
	struct ddcci_driver	*driver;

	if (!device)
		return 0;

	driver = to_ddcci_driver(drv);
	/* match on an id table if there is one */
	if (driver->id_table)
		return ddcci_match_id(driver->id_table, device) != NULL;

	return 0;
}

static int ddcci_device_probe(struct device *dev)
{
	struct ddcci_device	*device = ddcci_verify_device(dev);
	struct ddcci_driver	*driver;
	const struct ddcci_device_id *id;
	int ret = 0;

	if (!device)
		return -EINVAL;
	driver = to_ddcci_driver(dev->driver);

	id = ddcci_match_id(driver->id_table, device);
	if (!id)
		return -ENODEV;

	if (driver->probe)
		ret = driver->probe(device, id);

	return ret;
}

static int ddcci_device_remove(struct device *dev)
{
	struct ddcci_device	*device = ddcci_verify_device(dev);
	struct ddcci_driver	*driver;
	int ret = 0;

	if (!device)
		return -EINVAL;
	driver = to_ddcci_driver(dev->driver);

	if (driver->remove)
		ret = driver->remove(device);

	return ret;
}

/**
 * DDCCI bus type structure
 */
struct bus_type ddcci_bus_type = {
	.name		= "ddcci",
	.match		= ddcci_device_match,
	.probe		= ddcci_device_probe,
	.remove		= ddcci_device_remove
};

/* Main I2C driver */

/* Get a pointer to the closing parenthesis */
static char *ddcci_capstr_tok(const char *s, int depth)
{
	const char *ptr = s;
	char *end;

	if (s == NULL || s[0] == '\0')
		return NULL;

	while ((end = strpbrk(ptr, "()"))) {
		if (!end || depth == INT_MAX)
			return NULL;
		if (*end == '(')
			depth++;
		else if (depth > 0)
			depth--;
		else
			break;
		ptr = end+1;
	}
	return end;
}

/**
 * ddcci_find_capstr_item - Search capability string for a tag
 * @capabilities: Capability string to search
 * @tag: Tag to find
 * @length: Buffer for the length of the found tag value (optional)
 *
 * Return a pointer to the start of the tag value (directly after the '(') on
 * success and write the length of the value (excluding the ')') into `length`.
 *
 * If the tag is not found or another error occurs, an ERR_PTR is returned
 * and `length` stays untouched.
 */
const char *ddcci_find_capstr_item(const char *capabilities, const char *tag,
				    ptrdiff_t *length)
{
	const char *src = capabilities, *ptr;
	ptrdiff_t len;
	int taglen = strlen(tag);

	/* Check length of requested tag */
	if (unlikely(taglen <= 0 || taglen > 65535))
		return ERR_PTR(-EINVAL);

	/* Find tag */
	while (src && (strncmp(src+1, tag, taglen) != 0 || src[1+taglen] != '('))
		src = ddcci_capstr_tok(src+1, -1);
	if (!src || src[0] == '\0')
		return ERR_PTR(-ENOENT);

	/* Locate end of value */
	src += taglen+2;
	ptr = ddcci_capstr_tok(src, 0);
	if (unlikely(!ptr))
		return ERR_PTR(-EOVERFLOW);

	/* Check length of tag data */
	len = ptr-src;
	if (unlikely(len < 0 || len > 65535))
		return ERR_PTR(-EMSGSIZE);

	/* Return pointer and length */
	if (likely(length != NULL))
		*length = len;
	return src;
}
EXPORT_SYMBOL(ddcci_find_capstr_item);

/* Search the capability string for a tag and copy the value to dest */
static int ddcci_cpy_capstr_item(char *dest, const char *src,
				  const char *tag, size_t maxlen)
{
	const char *ptr;
	ptrdiff_t len;

	/* Find tag */
	ptr = ddcci_find_capstr_item(src, tag, &len);
	if (IS_ERR(ptr)) {
		return PTR_ERR(ptr);
	}

	/* Copy value */
	memcpy(dest, ptr, (len < maxlen) ? len : maxlen);
	return 0;
}

/* Fill fields in device by parsing the capability string */
static int ddcci_parse_capstring(struct ddcci_device *device)
{
	const char *capstr = device->capabilities;
	int ret = 0;

	if (!capstr)
		return -EINVAL;

	/* capability string start with a paren */
	if (capstr[0] != '(')
		return -EINVAL;

	/* get prot(...) */
	ret = ddcci_cpy_capstr_item(device->prot, capstr, "prot", 8);
	if (ret) {
		if (ret == -ENOENT) {
			dev_warn(&device->dev, "malformed capability string: no protocol tag");
			memset(device->prot, 0, 8);
		} else {
			return ret;
		}
	}

	/* get type(...) */
	ret = ddcci_cpy_capstr_item(device->type, capstr, "type", 8);
	if (ret) {
		if (ret == -ENOENT) {
			dev_warn(&device->dev, "malformed capability string: no type tag");
			memset(device->type, 0, 8);
		} else {
			return ret;
		}
	}

	/* and then model(...) */
	ret = ddcci_cpy_capstr_item(device->model, capstr, "model", 8);
	if (ret) {
		if (ret == -ENOENT) {
			dev_warn(&device->dev, "malformed capability string: no model tag");
			memset(device->model, 0, 8);
		} else {
			return ret;
		}
	}

	/* if there is no protocol tag */
	if (!device->prot[0]) {
		/* and no type tag: give up. */
		if (!device->type[0])
			return -ENOENT;

		/* Assume protocol "monitor" if type is "LCD" or "CRT" */
		if (strncasecmp(device->type, "LCD", 8) == 0
		 || strncasecmp(device->type, "CRT", 8) == 0) {
			memcpy(device->prot, "monitor", 7);
		}
	}

	/* skip the rest for now */

	return 0;
}

/* Probe for a device on an inner address and create a ddcci_device for it */
static int ddcci_detect_device(struct i2c_client *client, unsigned char addr,
			       int dependent)
{
	int ret;
	unsigned char outer_addr = client->addr << 1;
	unsigned char *buffer = NULL;
	struct ddcci_bus_drv_data *drv_data = i2c_get_clientdata(client);
	struct ddcci_device *device = NULL;

	down(&drv_data->sem);

	/* Allocate buffer big enough for any capability string */
	buffer = kmalloc(16384, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto end;
	}

	/* Allocate device struct */
	device = kzalloc(sizeof(struct ddcci_device), GFP_KERNEL);
	if (!device) {
		ret = -ENOMEM;
		goto end;
	}

	/* Initialize device */
	device_initialize(&device->dev);
	device->dev.parent = &client->dev;
	device->dev.bus = &ddcci_bus_type;
	device->outer_addr = outer_addr;
	device->inner_addr = addr;
	device->bus_drv_data = drv_data;
	device->i2c_client = client;

	if (!dependent) {
		device->dev.type = &ddcci_device_type;
		ret = dev_set_name(&device->dev, "ddcci%d", client->adapter->nr);
	} else if (outer_addr == dependent) {
		/* Internal dependent device */
		device->dev.type = &ddcci_dependent_type;
		device->flags = DDCCI_FLAG_DEPENDENT;
		ret = dev_set_name(&device->dev, "ddcci%di%02x", client->adapter->nr, addr);
	} else if (outer_addr == addr) {
		/* External dependent device */
		device->dev.type = &ddcci_dependent_type;
		device->flags = DDCCI_FLAG_DEPENDENT | DDCCI_FLAG_EXTERNAL;
		ret = dev_set_name(&device->dev, "ddcci%de%02x", client->adapter->nr, addr);
	} else {
		/* Dependent device of external dependent device
		   Just in case something like this exists */
		device->dev.type = &ddcci_dependent_type;
		device->flags = DDCCI_FLAG_DEPENDENT | DDCCI_FLAG_EXTERNAL;
		ret = dev_set_name(&device->dev, "ddcci%de%02x%02x", client->adapter->nr, outer_addr, addr);
	}

	if (ret)
		goto err_free;

	/* Read identification and check for quirks */
	ret = ddcci_identify_device(client, addr, buffer, 29);
	if (ret < 0)
		goto err_free;
	if (ret == 29 && buffer[0] == DDCCI_REPLY_ID) {
		memcpy(device->vendor, &buffer[7], 8);
		memcpy(device->module, &buffer[17], 8);
		device->device_number = be32_to_cpu(*(__force __be32 *)&buffer[18]);
	}

	/* Read capabilities */
	ret = ddcci_get_caps(client, addr, buffer, 16384);
	if (ret > 0) {
		device->capabilities = kzalloc(ret+1, GFP_KERNEL);
		if (!device->capabilities) {
			ret = -ENOMEM;
			goto err_free;
		}
		device->capabilities_len = ret;
		memcpy(device->capabilities, buffer, ret);

		ret = ddcci_parse_capstring(device);
		if (ret) {
			dev_err(&device->dev, "malformed capability string: \"%s\" errno %d\n", device->capabilities, ret);
			ret = -EINVAL;
			goto err_free;
		}
	}

	/* Found a device if either identification or capabilities succeeded */
	if (!device->capabilities && device->vendor[0] == '\0') {
		dev_dbg(&client->dev,
			"[%02x:%02x] got neither valid identification nor capability data\n",
			client->addr << 1, addr);
		ret = -ENODEV;
		goto err_free;
	}

	/* Setup chardev */
	down(&core_lock);
	ret = ddcci_setup_char_device(device);
	if (ret)
		goto err_free;

	/* Add device */
	pr_debug("found device at %d:%02x:%02x\n", client->adapter->nr, outer_addr, addr);
	ret = device_add(&device->dev);
	if (ret)
		goto err_free;

	goto end;
err_free:
	put_device(&device->dev);
end:
	kfree(buffer);
	up(&drv_data->sem);
	up(&core_lock);
	return ret;
}

/* I2C detect function: check if a main or external dependent device exists */
static int ddcci_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int ret;
	unsigned char outer_addr;
	unsigned char inner_addr;
	unsigned char buf[32];
	unsigned char cmd[2] = { DDCCI_COMMAND_ID, 0x00 };

	/* Check for i2c_master_* functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_debug("i2c adapter %d unsuitable: no i2c_master functionality\n", client->adapter->nr);
		return -ENODEV;
	}

	/* send Identification Request command */
	outer_addr = client->addr << 1;
	inner_addr = (outer_addr == DDCCI_DEFAULT_DEVICE_ADDR) ? DDCCI_HOST_ADDR_ODD : outer_addr | 1;
	pr_debug("detecting %d:%02x\n", client->adapter->nr, outer_addr);

	ret = __ddcci_write_block(client, inner_addr, buf, true, cmd, 2);

	if (ret == -ENXIO) {
		if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_BYTE))
			return -ENODEV;
		pr_debug("i2c write failed with ENXIO, trying bytewise writing\n");
		ret = __ddcci_write_bytewise(client, inner_addr, true, cmd, 2);
		if (ret == 0) {
			msleep(delay);
			ret = __ddcci_write_bytewise(client, inner_addr, true, cmd, 2);
		}
	}

	if (ret < 0)
		return -ENODEV;

	/* wait for device */
	msleep(delay);
	/* receive answer */
	ret = i2c_master_recv(client, buf, 32);
	if (ret < 3) {
		pr_debug("detection failed: no answer\n");
		return -ENODEV;
	}

	/* check response starts with outer addr */
	if (buf[0] != outer_addr) {
		pr_debug("detection failed: invalid identification response (%02x != %02x)\n", buf[0], outer_addr);
		pr_debug("received message was %*ph \n", ret, buf);
		return -ENODEV;
	}

	pr_debug("detected %d:%02x\n", client->adapter->nr, outer_addr);

	/* set device type */
	strlcpy(info->type, (outer_addr == DDCCI_DEFAULT_DEVICE_ADDR) ? "ddcci" : "ddcci-dependent", I2C_NAME_SIZE);

	return 0;
}

/* I2C probe function */
static int ddcci_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i, ret = -ENODEV, tmp;
	unsigned char main_addr, addr;
	struct ddcci_bus_drv_data *drv_data;

	/* Initialize driver data structure */
	drv_data = devm_kzalloc(&client->dev, sizeof(struct ddcci_bus_drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;
	drv_data->i2c_dev = client;
	sema_init(&drv_data->sem, 1);

	/* Set i2c client data */
	i2c_set_clientdata(client, drv_data);

	if (id->driver_data == 0) {
		/* Core device, probe at 0x6E */
		main_addr = DDCCI_DEFAULT_DEVICE_ADDR;
		dev_dbg(&client->dev, "probing core device [%02x]\n",
			client->addr << 1);
		ret = ddcci_detect_device(client, main_addr, 0);
		if (ret) {
			dev_info(&client->dev, "core device [%02x] probe failed: %d\n",
				 client->addr << 1, ret);
			if (ret == -EIO)
				ret = -ENODEV;
			goto err_free;
		}

		/* Detect internal dependent devices */
		dev_dbg(&client->dev, "probing internal dependent devices\n");
		for (i = 0; i < autoprobe_addr_count; ++i) {
			addr = (unsigned short)autoprobe_addrs[i];
			if ((addr & 1) == 0 && addr != main_addr) {
				tmp = ddcci_detect_device(client, addr, main_addr);
				if (tmp < 0 && tmp != -ENODEV) {
					dev_info(&client->dev, "internal dependent device [%02x:%02x] probe failed: %d\n",
						 client->addr << 1, addr, ret);
				}
			}
		}
	} else if (id->driver_data == 1) {
		/* External dependent device */
		main_addr = client->addr << 1;
		dev_dbg(&client->dev, "probing external dependent device [%02x]\n", main_addr);
		ret = ddcci_detect_device(client, main_addr, -1);
		if (ret) {
			dev_info(&client->dev, "external dependent device [%02x] probe failed: %d\n",
				 main_addr, ret);
			if (ret == -EIO)
				ret = -ENODEV;
			goto err_free;
		}
	} else {
		dev_warn(&client->dev,
			 "probe() called with invalid i2c device id\n");
		ret = -EINVAL;
	}

	goto end;
err_free:
	devm_kfree(&client->dev, drv_data);
end:
	return ret;
}

/*
 * Callback for bus_find_device() used in ddcci_remove()
 *
 * Find next device on i2c_client not flagged with
 * DDCCI_FLAG_REMOVED and flag it.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,3,0)
static int ddcci_remove_helper(struct device *dev, const void *p)
#else
static int ddcci_remove_helper(struct device *dev, void *p)
#endif
{
	struct ddcci_device *device;

	device = ddcci_verify_device(dev);
	if (!device || device->flags & DDCCI_FLAG_REMOVED)
		return 0;

	if (!p || (dev->parent == p)) {
		device->flags |= DDCCI_FLAG_REMOVED;
		wmb();
		return 1;
	}

	return 0;
}

/* I2C driver remove callback: unregister all subdevices */
static int ddcci_remove(struct i2c_client *client)
{
	struct ddcci_bus_drv_data *drv_data = i2c_get_clientdata(client);
	struct device *dev;

	down(&drv_data->sem);
	while (1) {
		dev = bus_find_device(&ddcci_bus_type, NULL, client,
				      ddcci_remove_helper);
		if (!dev)
			break;
		device_unregister(dev);
		put_device(dev);
	}
	up(&drv_data->sem);
	return 0;
}

/*
 * I2C driver device identification table.
 */
static const struct i2c_device_id ddcci_idtable[] = {
	{ "ddcci", 0 },
	{ "ddcci-dependent", 1 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ddcci_idtable);

/*
 * I2C driver description structure
 */
static struct i2c_driver ddcci_driver = {
	.driver = {
		.name	= "ddcci",
		.owner	= THIS_MODULE,
	},

	.id_table	= ddcci_idtable,
	.probe		= ddcci_probe,
	.remove		= ddcci_remove,
	.class		= I2C_CLASS_DDC,
	.detect		= ddcci_detect,
	.address_list	= I2C_ADDRS(
		DDCCI_DEFAULT_DEVICE_ADDR>>1
	),
};

/*
 * Module initialization function. Called when the module is inserted or
 * (if builtin) at boot time.
 */
static int __init ddcci_module_init(void)
{
	int ret;

	pr_debug("initializing ddcci driver\n");
	/* Allocate a device number region for the character devices */
	ret = alloc_chrdev_region(&ddcci_cdev_first, 0, 128, DEVICE_NAME);
	if (ret < 0) {
		pr_err("failed to register device region: error %d\n", ret);
		goto err_chrdevreg;
	}
	ddcci_cdev_next = ddcci_cdev_first;
	ddcci_cdev_end = MKDEV(MAJOR(ddcci_cdev_first), MINOR(ddcci_cdev_first)+128);

	/* Register bus */
	ret = bus_register(&ddcci_bus_type);
	if (ret) {
		pr_err("failed to register bus 'ddcci'\n");
		goto err_busreg;
	}

	/* Register I2C driver */
	ret = i2c_add_driver(&ddcci_driver);
	if (ret) {
		pr_err("failed to register i2c driver\n");
		goto err_drvreg;
	}

	pr_debug("ddcci driver initialized\n");

	return 0;

err_drvreg:
	bus_unregister(&ddcci_bus_type);
err_busreg:
	unregister_chrdev_region(ddcci_cdev_first, 128);
err_chrdevreg:
	return ret;
}

/*
 * Module clean-up function. Called when the module is removed.
 */
static void __exit ddcci_module_exit(void)
{
	struct device *dev;

	while (1) {
		dev = bus_find_device(&ddcci_bus_type, NULL, NULL, ddcci_remove_helper);
		if (!dev)
			break;
		device_unregister(dev);
		put_device(dev);
	}

	i2c_del_driver(&ddcci_driver);
	bus_unregister(&ddcci_bus_type);
	unregister_chrdev_region(ddcci_cdev_first, 128);
}

/* Let the kernel know the calls for module init and exit */
module_init(ddcci_module_init);
module_exit(ddcci_module_exit);

/* Module parameter description */
module_param(delay, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(delay, "default delay after bus writes (in ms, default 60)");
module_param_array(autoprobe_addrs, ushort, &autoprobe_addr_count, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(autoprobe_addrs, "internal dependent device addresses to autoprobe");

/* Module description */
MODULE_AUTHOR("Christoph Grenz");
MODULE_DESCRIPTION("DDC/CI bus driver");
MODULE_VERSION("0.3.3");
MODULE_LICENSE("GPL");
