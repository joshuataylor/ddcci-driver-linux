/*
 *  DDC/CI sub-bus definition and i2c polling infrastructure
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
#include <asm/uaccess.h>
#include <asm-generic/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/sem.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/ddcci.h>

#define DDCCI_RECV_BUFFER_SIZE 130
#define DEVICE_NAME "ddcci"

static unsigned int probe_interval = 17;
static unsigned int delay = 60;
static unsigned short autoprobe_addrs[127] = {0xF0, 0xF2, 0xF4, 0xF6, 0xF8};
static int autoprobe_addr_count = 5;

static struct workqueue_struct *poll_workqueue;
static DEFINE_SEMAPHORE(core_lock);
static DEFINE_HASHTABLE(adapter_table, 5);
static struct i2c_client poll_client;
static struct i2c_board_info poll_board_info;

extern struct i2c_driver ddcci_i2c_driver;

EXPORT_SYMBOL_GPL(ddcci_bus_type);

/* I2C adapter level data structure */
struct ddcci_bus_data {
	struct i2c_adapter *adapter;
	unsigned long quirks;
	unsigned char recv_buffer[DDCCI_RECV_BUFFER_SIZE];
	struct delayed_work work;
	struct semaphore sem;
	struct hlist_node hash_node;
};

/* Get the ddcci_bus_data structure associated with an i2c adapter */
struct ddcci_bus_data *ddcci_get_bus_data(struct i2c_adapter *adapter)
{
	struct hlist_node *tmp = NULL;
	struct ddcci_bus_data *cur;

	hash_for_each_possible_safe(adapter_table, cur, tmp, hash_node, (unsigned long)adapter) {
		if (cur->adapter == adapter)
			return cur;
	}
	return NULL;
}

extern int ddcci_detect(struct i2c_client *client, struct i2c_board_info *info);

/*
 * Callback for bus_find_device()
 *
 * Find i2c client on address 0x37 on given adapter.
 */
static int ddcci_client_find_helper(struct device *dev, void* p)
{
	unsigned char addr = 0x37;
	struct i2c_adapter *adapter = p;
	struct i2c_client *client;

	client = i2c_verify_client(dev);
	if (!client)
		return 0;
	if (client->adapter != adapter)
		return 0;
	if (client->addr != (DDCCI_DEFAULT_DEVICE_ADDR >> 1))
		return 0;
	return 1;
}

static void ddcci_poll_with_client(struct i2c_client *client)
{
	int ret;
	/* Prepare structures */
	memset(&poll_board_info, 0, sizeof(poll_board_info));
	poll_board_info.addr = client->addr;
	/* Redetect device */
	ret = ddcci_detect(client, &poll_board_info);
	if (ret >= 0)
		return;
	/* Try once again to be sure the device vanished. */
	ret = ddcci_detect(client, &poll_board_info);
	if (ret >= 0)
		return;
	/* Remove device */
	i2c_unregister_device(client);
}

static void ddcci_poll_without_client(struct i2c_adapter *adapter, unsigned char addr)
{
	int ret;
	/* Prepare structures */
	memset(&poll_client, 0, sizeof(poll_client));
	memset(&poll_board_info, 0, sizeof(poll_board_info));
	poll_client.adapter = adapter;
	poll_client.addr = addr;
	poll_board_info.addr = addr;
	/* Detect device. The routine will fill poll_board_info on success */
	ret = ddcci_detect(&poll_client, &poll_board_info);
	if (ret < 0) {
		if (ret != -ENODEV)
			pr_info("DDC/CI device detection poll failed on %s:%02x: %d",
				dev_name(&adapter->dev), addr, ret);
		return;
	}
	pr_debug("DDC/CI device detected on %s:%02x",
		 dev_name(&adapter->dev), addr);
	/* Create new device and let the driver handle the rest */
	if (!i2c_new_device(adapter, &poll_board_info)) {
		pr_info("creating an i2c device failed on %s:%02x",
			dev_name(&adapter->dev), addr);
		return;
	}
}

static void ddcci_poll_work(struct work_struct *work)
{
	int ret;
	struct device *dev;
	struct i2c_client *client = NULL;
	struct delayed_work *dwork = to_delayed_work(work);
	struct ddcci_bus_data *bus_data = container_of(dwork, struct ddcci_bus_data, work);

	if (down_interruptible(&core_lock))
		goto out;
	dev = bus_find_device(&i2c_bus_type, NULL,
			      bus_data->adapter,
			      ddcci_client_find_helper);
	if (dev) {
		if (device_trylock(dev)) {
			client = to_i2c_client(dev);
			if (dev->driver == &ddcci_i2c_driver.driver) {
				ddcci_poll_with_client(client);
			}
			device_unlock(dev);
		}
		put_device(dev);
	}
	else {
		ddcci_poll_without_client(bus_data->adapter,
					  DDCCI_DEFAULT_DEVICE_ADDR >> 1);
	}

	up(&core_lock);
out:
	/* Queue next poll */
	queue_delayed_work(poll_workqueue, dwork, msecs_to_jiffies(probe_interval*1000));
};

static int ddcci_attach_to_adapter(struct i2c_adapter *adapter)
{
	struct ddcci_bus_data *bus_data;

	/* Create workqueue on first attach */
	if (unlikely(poll_workqueue == NULL)) {
		poll_workqueue = create_singlethread_workqueue("ddcci-worker");
		if (!poll_workqueue) {
			pr_err("failed to create workqueue\n");
			return -ENOMEM;
		}
		pr_debug("created workqueue %s\n", "ddcci-worker");
	}

	pr_debug("attaching to i2c adapter %s\n", dev_name(&adapter->dev));

	/* Check if it already exists */
	if (ddcci_get_bus_data(adapter)) {
		return -EEXIST;
	}

	/* Initialize adapter level data structure */
	bus_data = kzalloc(sizeof(*bus_data), GFP_KERNEL);
	if (!bus_data) {
		return -ENOMEM;
	}
	sema_init(&bus_data->sem, 1);
	INIT_DELAYED_WORK(&bus_data->work, &ddcci_poll_work);
	bus_data->adapter = adapter;

	/* Add to hash table */
	hash_add(adapter_table, &bus_data->hash_node, (unsigned long)adapter);

	/* Queue initial device poll */
	queue_delayed_work(poll_workqueue, &bus_data->work, msecs_to_jiffies(delay));

	pr_debug("attached to i2c adapter %s\n", dev_name(&adapter->dev));

	return 0;
}

static void ddcci_detach_from_adapter(struct i2c_adapter *adapter)
{
	struct ddcci_bus_data *bus_data;

	/* Stop if we aren't attached to this adapter */
	if (!(bus_data = ddcci_get_bus_data(adapter))) {
		return;
	}

	pr_debug("detaching from i2c adapter %s\n", dev_name(&adapter->dev));
	/* Cancel poll worker */
	cancel_delayed_work_sync(&bus_data->work);

	/* Remove from hashtable and free adapter level data structure */
	hash_del(&bus_data->hash_node);
	kfree(bus_data);
	pr_debug("detached from i2c adapter %s\n", dev_name(&adapter->dev));
}



/* Callback for i2c bus notifier. */
static int ddcci_i2c_bus_notify(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct device *dev = data;
	struct i2c_adapter *adapter = to_i2c_adapter(dev);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		if (dev->type == &i2c_adapter_type) {
			if (adapter->class == I2C_CLASS_DDC) {
				ddcci_attach_to_adapter(to_i2c_adapter(dev));
			}
		}
		break;
	case BUS_NOTIFY_DEL_DEVICE:
		if (dev->type == &i2c_adapter_type) {
			if (adapter->class == I2C_CLASS_DDC) {
				ddcci_detach_from_adapter(to_i2c_adapter(dev));
			}
		}
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block ddcci_i2c_bus_nb = {
	.notifier_call = ddcci_i2c_bus_notify
};

/* Callback for bus_for_each_dev() -- attaches to I2C DDC class adapters */
static int __init ddcci_adapter_attach_helper(struct device *dev, void *p)
{
	int ret;
	struct i2c_adapter *adapter;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adapter = to_i2c_adapter(dev);
	if (adapter->class != I2C_CLASS_DDC)
		return 0;
	ret = ddcci_attach_to_adapter(adapter);

	if (ret && ret != -EEXIST) {
		pr_err("could not attach to i2c adapter %s: %i\n",
			dev_name(dev), ret
		);
	}

	return 0;
}

static int __init ddcci_module_init(void)
{
	int ret;
	struct device *dev = NULL;

	sema_init(&core_lock, 0);
	hash_init(adapter_table);

	/* Register bus */
	ret = bus_register(&ddcci_bus_type);
	if (ret) {
		pr_err("failed to register bus 'ddcci'\n");
		goto err_busreg;
	}

	/* Ensure work queue pointer is NULL */
	poll_workqueue = NULL;

	/* Register I2C bus notifier */
	ret = bus_register_notifier(&i2c_bus_type, &ddcci_i2c_bus_nb);
	if (ret) {
		pr_err("failed to register i2c bus notification hook\n");
		goto err_notifyreg;
	}

	/* Attach to already existing DDC I2C adapters */
	bus_for_each_dev(&i2c_bus_type, NULL, NULL,
			 ddcci_adapter_attach_helper);

	up(&core_lock);
	return 0;

err_notifyreg:
	bus_unregister(&ddcci_bus_type);
err_busreg:
	up(&core_lock);
	return ret;
}

static void __exit ddcci_module_exit(void)
{
	int bkt;
	struct hlist_node *tmp = NULL;
	struct ddcci_bus_data *cur = NULL;

	bus_unregister_notifier(&i2c_bus_type, &ddcci_i2c_bus_nb);

	down(&core_lock);
	hash_for_each_safe(adapter_table, bkt, tmp, cur, hash_node) {
		ddcci_detach_from_adapter(cur->adapter);
	}

	if (poll_workqueue) {
		flush_workqueue(poll_workqueue);
		destroy_workqueue(poll_workqueue);
	}

	bus_unregister(&ddcci_bus_type);
}

/* Let the kernel know the calls for module init and exit */
module_init(ddcci_module_init);
module_exit(ddcci_module_exit);

/* Module parameter description */
module_param(probe_interval, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(probe_interval, "interval to probe the I2C bus for a DDC/CI compatible device (in s, default 13)");
module_param(delay, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(delay, "default delay after bus writes (in ms, default 60)");
module_param_array(autoprobe_addrs, ushort, &autoprobe_addr_count, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(autoprobe_addrs, "internal dependent device addresses to autoprobe");

/* Module description */
MODULE_AUTHOR("Christoph Grenz");
MODULE_DESCRIPTION("DDC/CI bus driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
