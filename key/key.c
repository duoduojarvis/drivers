/*
 * iMX6ULL 按键 platform 驱动（miscdevice + gpiod + 中断消抖）
 *
 * 硬件：GPIO1_IO18，低电平触发（GPIO_ACTIVE_LOW）
 * 设备节点：/dev/key
 * 读取键值：1 = 按下，0 = 松开
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include "key.h"

#define KEY_DEBOUNCE_MS  10   /* 消抖延迟，毫秒 */

/* ---------- file_operations ---------- */

static int key_open(struct inode *inode, struct file *filp)
{
	/*
	 * misc 框架 open 时将 filp->private_data 设为 miscdevice 指针，
	 * 用 container_of 反推外层 key_dev。
	 */
	struct miscdevice *miscdev = filp->private_data;
	struct key_dev *dev = container_of(miscdev, struct key_dev, miscdev);

	filp->private_data = dev;
	return 0;
}

static ssize_t key_read(struct file *filp, char __user *buf,
			size_t size, loff_t *offset)
{
	struct key_dev *dev = filp->private_data;
	unsigned char val;
	unsigned long flags;

	if (size < 1)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	val = dev->key_val;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (copy_to_user(buf, &val, 1))
		return -EFAULT;

	return 1;
}

static const struct file_operations key_fops = {
	.owner  = THIS_MODULE,
	.open   = key_open,
	.read   = key_read,
};

/* ---------- 中断消抖 ---------- */

/*
 * 消抖定时器服务函数：定时器到期后读取 GPIO 确认键值。
 * gpiod_get_value 返回逻辑值：GPIO_ACTIVE_LOW 下，
 * 物理低电平（按下）→ 逻辑 1；物理高电平（松开）→ 逻辑 0。
 */
static void key_timer_func(unsigned long arg)
{
	struct key_dev *dev = (struct key_dev *)arg;
	unsigned long flags;
	int val;

	val = gpiod_get_value(dev->gpiod);  /* 逻辑值：1=按下 0=松开 */

	spin_lock_irqsave(&dev->lock, flags);
	dev->key_val = (unsigned char)val;
	spin_unlock_irqrestore(&dev->lock, flags);

	pr_debug("[key] debounced: key_val=%d\n", val);
}

/* 中断处理：任意边沿触发，重启消抖定时器 */
static irqreturn_t key_irq_handler(int irq, void *data)
{
	struct key_dev *dev = data;

	mod_timer(&dev->timer, jiffies + msecs_to_jiffies(KEY_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

/* ---------- platform driver ---------- */

static int key_probe(struct platform_device *pdev)
{
	struct key_dev *dev;
	int irq, err;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* 申请 GPIO，方向输入，DTS 中 "key-gpio" 属性 */
	dev->gpiod = devm_gpiod_get(&pdev->dev, "key", GPIOD_IN);
	if (IS_ERR(dev->gpiod)) {
		dev_err(&pdev->dev, "failed to get key gpio: %ld\n",
			PTR_ERR(dev->gpiod));
		return PTR_ERR(dev->gpiod);
	}

	/* 获取中断号 */
	irq = gpiod_to_irq(dev->gpiod);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq from gpio\n");
		return irq;
	}
	dev->irqnum = irq;

	/* 初始化自旋锁和消抖定时器 */
	spin_lock_init(&dev->lock);
	setup_timer(&dev->timer, key_timer_func, (unsigned long)dev);

	/* 注册中断（双边沿触发，devm_ 自动释放） */
	err = devm_request_irq(&pdev->dev, irq, key_irq_handler,
			       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			       "key", dev);
	if (err) {
		dev_err(&pdev->dev, "failed to request irq %d: %d\n", irq, err);
		return err;
	}

	/* 注册 misc 设备 */
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name  = "key";
	dev->miscdev.fops  = &key_fops;

	err = misc_register(&dev->miscdev);
	if (err) {
		dev_err(&pdev->dev, "failed to register misc device: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, dev);
	dev_info(&pdev->dev, "key probe ok, irq=%d\n", irq);
	return 0;
}

static int key_remove(struct platform_device *pdev)
{
	struct key_dev *dev = platform_get_drvdata(pdev);

	misc_deregister(&dev->miscdev);
	del_timer_sync(&dev->timer);  /* 等待定时器完成后再释放 */
	return 0;
}

static const struct of_device_id key_of_match[] = {
	{ .compatible = "alientek,key" },
	{ }
};
MODULE_DEVICE_TABLE(of, key_of_match);

static struct platform_driver key_driver = {
	.probe  = key_probe,
	.remove = key_remove,
	.driver = {
		.name           = "key",
		.of_match_table = key_of_match,
	},
};

module_platform_driver(key_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zrc");
MODULE_DESCRIPTION("iMX6ULL key platform driver (miscdevice + gpiod + debounce)");
