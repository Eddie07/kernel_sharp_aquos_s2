/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>
#include <linux/sensors.h>

static struct class *sensors_class;

DECLARE_RWSEM(sensors_list_lock);
LIST_HEAD(sensors_list);

static ssize_t sensors_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->name);
}

static ssize_t sensors_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->vendor);
}

static ssize_t sensors_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->version);
}

static ssize_t sensors_handle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->handle);
}

static ssize_t sensors_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->type);
}

static ssize_t sensors_max_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->max_range);
}

static ssize_t sensors_resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->resolution);
}

static ssize_t sensors_power_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->sensor_power);
}

static ssize_t sensors_min_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->min_delay);
}

static ssize_t sensors_fifo_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			sensors_cdev->fifo_reserved_event_count);
}

static ssize_t sensors_fifo_max_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			sensors_cdev->fifo_max_event_count);
}

static ssize_t sensors_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long data = 0;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if (data > 1) {
		dev_err(dev, "Invalid value of input, input=%ld\n", data);
		return -EINVAL;
	}

	if (sensors_cdev->sensors_enable == NULL) {
		dev_err(dev, "Invalid sensor class enable handle\n");
		return -EINVAL;
	}
	ret = sensors_cdev->sensors_enable(sensors_cdev, data);
	if (ret)
		return ret;

	sensors_cdev->enabled = data;
	return size;
}


static ssize_t sensors_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n",
			sensors_cdev->enabled);
}

static ssize_t sensors_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long data = 0;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	/* The data unit is millisecond, the min_delay unit is microseconds. */
	if ((data * 1000) < sensors_cdev->min_delay) {
		dev_err(dev, "Invalid value of delay, delay=%ld\n", data);
		return -EINVAL;
	}
	if (sensors_cdev->sensors_poll_delay == NULL) {
		dev_err(dev, "Invalid sensor class delay handle\n");
		return -EINVAL;
	}
	ret = sensors_cdev->sensors_poll_delay(sensors_cdev, data);
	if (ret)
		return ret;

	sensors_cdev->delay_msec = data;
	return size;
}

static ssize_t sensors_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n",
			sensors_cdev->delay_msec);
}


static DEVICE_ATTR(name, 0444, sensors_name_show, NULL);
static DEVICE_ATTR(vendor, 0444, sensors_vendor_show, NULL);
static DEVICE_ATTR(version, 0444, sensors_version_show, NULL);
static DEVICE_ATTR(handle, 0444, sensors_handle_show, NULL);
static DEVICE_ATTR(type, 0444, sensors_type_show, NULL);
static DEVICE_ATTR(max_range, 0444, sensors_max_range_show, NULL);
static DEVICE_ATTR(resolution, 0444, sensors_resolution_show, NULL);
static DEVICE_ATTR(sensor_power, 0444, sensors_power_show, NULL);
static DEVICE_ATTR(min_delay, 0444, sensors_min_delay_show, NULL);
static DEVICE_ATTR(fifo_reserved_event_count, 0444, sensors_fifo_event_show, NULL);
static DEVICE_ATTR(fifo_max_event_count, 0444, sensors_fifo_max_show, NULL);
static DEVICE_ATTR(enable, 0664, sensors_enable_show, sensors_enable_store);
static DEVICE_ATTR(poll_delay, 0664, sensors_delay_show, sensors_delay_store);
static struct attribute *sensors_class_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_vendor.attr,
	&dev_attr_version.attr,
	&dev_attr_handle.attr,
	&dev_attr_type.attr,
	&dev_attr_max_range.attr,
	&dev_attr_resolution.attr,
	&dev_attr_sensor_power.attr,
	&dev_attr_min_delay.attr,
	&dev_attr_fifo_reserved_event_count.attr,
	&dev_attr_fifo_max_event_count.attr,
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sensors_class);

/**
 * sensors_classdev_register - register a new object of sensors_classdev class.
 * @parent: The device to register.
 * @sensors_cdev: the sensors_classdev structure for this device.
*/
int sensors_classdev_register(struct device *parent,
				struct sensors_classdev *sensors_cdev)
{
	sensors_cdev->dev = device_create(sensors_class, parent, 0,
				      sensors_cdev, "%s", sensors_cdev->name);
	if (IS_ERR(sensors_cdev->dev))
		return PTR_ERR(sensors_cdev->dev);

	down_write(&sensors_list_lock);
	list_add_tail(&sensors_cdev->node, &sensors_list);
	up_write(&sensors_list_lock);

	pr_debug("Registered sensors device: %s\n",
			sensors_cdev->name);
	return 0;
}
EXPORT_SYMBOL(sensors_classdev_register);

/**
 * sensors_classdev_unregister - unregister a object of sensors class.
 * @sensors_cdev: the sensor device to unregister
 * Unregister a previously registered via sensors_classdev_register object.
*/
void sensors_classdev_unregister(struct sensors_classdev *sensors_cdev)
{
	device_unregister(sensors_cdev->dev);
	down_write(&sensors_list_lock);
	list_del(&sensors_cdev->node);
	up_write(&sensors_list_lock);
}
EXPORT_SYMBOL(sensors_classdev_unregister);

static int __init sensors_init(void)
{
	sensors_class = class_create(THIS_MODULE, "sensors");
	if (IS_ERR(sensors_class))
		return PTR_ERR(sensors_class);
	sensors_class->dev_groups = sensors_class_groups;
	return 0;
}

static void __exit sensors_exit(void)
{
	class_destroy(sensors_class);
}

subsys_initcall(sensors_init);
module_exit(sensors_exit);
