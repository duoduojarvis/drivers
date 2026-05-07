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
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include "key.h"

struct key_dev key_dev;

static int key_open(struct inode *nd, struct file *flip)
{
	flip->private_data = &key_dev;
	return 0;
}

static ssize_t key_read(struct file *flip, char __user *buf, size_t size, loff_t *offset)
{
	int isPress;
	int ret;

	isPress = gpio_get_value(key_dev.gpio);
	pr_info("[kernel] isPress=%d\n", isPress);
	ret = copy_to_user(buf, &isPress, sizeof(isPress));

	return ret;
}

//定时器服务函数
static void timer_function(unsigned long arg)
{
	int key_value;
	struct key_dev *dev = (struct key_dev *)arg;

	key_value = gpio_get_value(dev->gpio);
	pr_info("key_value=0x%x\r\n", key_value);
	if (key_value == 0)
		led_on();
	else
		led_off();
}

static void init_key_timer(void)
{
	spin_lock_init(&key_dev.lock);
	init_timer(&key_dev.timer);
	key_dev.timer.function = timer_function;
	key_dev.timer.data = (unsigned long)&key_dev;
	key_dev.timeperiod = 5; // 毫秒
}

static irqreturn_t key_irq_handler(int irq, void *data)
{
	struct key_dev *dev = (struct key_dev *)data;

	dev->timer.data = (volatile long)data;
	// 启动定时器
	(void)mod_timer(&dev->timer, jiffies + msecs_to_jiffies(dev->timeperiod));
	return IRQ_RETVAL(IRQ_HANDLED);
}

static int key_dev_init(struct key_dev *key_dev)
{
	key_dev->np = of_find_node_by_path("/key");

	if (!key_dev->np) {
		pr_err("can't find key in dts\n");
		return -EINVAL;
	}

	key_dev->gpio = of_get_named_gpio(key_dev->np, "key-gpio", 0);
	if (!gpio_is_valid(key_dev->gpio))
		return -ENODEV;

	key_dev->irq.handler = key_irq_handler;
	sprintf(key_dev->irq.name, "KEY%d", 0);
	return 0;
}

static int key_irq_init(struct key_dev *key_dev)
{
	int err;
	struct key_irq *irq = &key_dev->irq;

	irq->irqnum =  gpio_to_irq(key_dev->gpio);
	pr_info("irqnum=%u\n", irq->irqnum);

	err = request_irq(irq->irqnum, irq->handler, IRQF_TRIGGER_FALLING, irq->name, key_dev);
	if (err < 0) {
		pr_err("request irq fail\n");
		return err;
	}
	return 0;
}


static struct file_operations key_fops= {
	.owner = THIS_MODULE,
	.open = key_open,
	.read = key_read,
};

struct miscdevice key_miscdev = {
	.minor		= 143,
	.name		= "key",
	.fops		= &key_fops,
};

static int key_probe(struct platform_device * pdev)
{
	int err;

	err = key_dev_init(&key_dev);
	if (err != 0) {
		pr_err("key_dev_init fail\n");
	}

	err = misc_register(&key_miscdev);
	if (err < 0) {
		pr_err("[KEY] error: cannot register device\n");
		return err;
	}

	// 初始化定时器 用于按键消抖
	init_key_timer();

	err = key_irq_init(&key_dev);
	if (err) {
		pr_err("key_irq_init fail\n");
		return err;
	}

	return 0;
}

static int key_remove(struct platform_device *pdev)
{
	// 注销misc设备驱动
	(void)misc_deregister(&key_miscdev);
	free_irq(key_dev.irq.irqnum, &key_dev);
	return 0;
}

static const struct of_device_id key_of_match_table[] = {
	{ .compatible = "key" },
	{},
};

static struct platform_driver key_drv = {
	.probe = key_probe,
	.remove = key_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "key",
		.of_match_table = key_of_match_table,
	}
};

static int __init mykey_init(void)
{
	return platform_driver_register(&key_drv);
}

static void __exit mykey_exit(void)
{
	platform_driver_unregister(&key_drv);
}

module_init(mykey_init);
module_exit(mykey_exit);
MODULE_LICENSE("GPL");

