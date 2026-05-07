#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h> 
#include "miscbeep.h"

struct miscbeep_dev dev;

static int beep_open(struct inode *inode, struct file *flip)
{
	flip->private_data = &dev;
	return 0;
}

static ssize_t beep_write(struct file *flip, const char __user *buf, size_t size, loff_t *offset)
{
	char onoff;
	int err;

	err = copy_from_user(&onoff, buf, sizeof(onoff));
	if (err < 0) {
		return -EFAULT;
	}

	if (onoff == 1)
		gpio_set_value(dev.gpio, 0);
	else if (onoff == 0)
		gpio_set_value(dev.gpio, 1);
	else
		pr_err("please input 1 to turn on beep, or 0 to turn off beep\n");

	return 0;
}

static struct file_operations beep_fops = {
	.owner = THIS_MODULE,
	.open = beep_open,
	.write = beep_write,
};

static struct miscdevice beep_miscdev = {
	.minor		= 144,
	.name		= "beep",
	.fops		= &beep_fops,
};

static int beep_probe(struct platform_device *pdev)
{
	int err;

	dev.np = of_find_node_by_path("/beep");
	if (!dev.np) {
		pr_err("can't find beep in dts\n");
		return -EINVAL;
	}

	dev.gpio = of_get_named_gpio(dev.np, "beep-gpio",0);
	if (!gpio_is_valid(dev.gpio)) {
		return -ENODEV;
	}

	// 设置为输出并且关闭蜂鸣器
	err = gpio_direction_output(dev.gpio, 1);
	if (err < 0) {
		pr_err("set gpio direction fali\n");
		return -EINVAL;
	}

	err = misc_register(&beep_miscdev);
	if (err < 0) {
		pr_err("BEEP error: cannot register device");
		return err;
	}

	return 0;
}

static int beep_remove(struct platform_device *pdev)
{

	// 关闭蜂鸣器
	gpio_set_value(dev.gpio, 1);

	// 注销misc设备驱动
	(void)misc_deregister(&beep_miscdev);
	return 0;
}

static const struct of_device_id beep_of_match_table[] = {
	{ .compatible = "beep" },
	{},
};

static struct platform_driver beep_drv = {
	.probe = beep_probe,
	.remove = beep_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "beep",
		.of_match_table = beep_of_match_table,
	}
};

static int __init beep_init(void)
{
	return platform_driver_register(&beep_drv);
}

static void __exit beep_exit(void)
{
	platform_driver_unregister(&beep_drv);
}

module_init(beep_init);
module_exit(beep_exit);
MODULE_LICENSE("GPL");