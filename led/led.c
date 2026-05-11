#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include "led.h"

#define LED_CLOSE_TIMER  _IO(0XEF, 0x1) /* 关闭定时器 */
#define LED_OPEN_TIMER   _IO(0XEF, 0x2) /* 打开定时器 */
#define LED_SET_PERIOD   _IO(0XEF, 0x3) /* 设置定时器周期 */

#define LED_DEFAULT_PERIOD  1000 /* 默认闪烁周期 1000ms */

/* ------------------------------------------------------------------ */
/* 硬件操作                                                             */
/* ------------------------------------------------------------------ */

static void led_hw_on(struct led_dev *dev)
{
	u32 val = readl(dev->gpio_dr);
	val &= ~BIT(3);
	writel(val, dev->gpio_dr);
}

static void led_hw_off(struct led_dev *dev)
{
	u32 val = readl(dev->gpio_dr);
	val |= BIT(3);
	writel(val, dev->gpio_dr);
}

static void led_set_status(struct led_dev *dev, int on)
{
	if (on)
		led_hw_on(dev);
	else
		led_hw_off(dev);
}

static void led_hw_init(struct led_dev *dev)
{
	u32 val;

	/* 使能 GPIO1 时钟：CCM_CCGR1[27:26] = 11 */
	val = readl(dev->ccm_ccgr1);
	val |= (0x3 << 26);
	writel(val, dev->ccm_ccgr1);

	/* 设置 GPIO1_IO03 复用为 GPIO 功能 */
	writel(0x5, dev->sw_mux);

	/* 设置 PAD 属性（100MHz，Hysteresis Enabled） */
	writel(0x10B0, dev->sw_pad);

	/* 设置 GPIO1_IO03 为输出方向 */
	val = readl(dev->gpio_gdir);
	val |= BIT(3);
	writel(val, dev->gpio_gdir);

	/* 默认关闭 LED（高电平） */
	val = readl(dev->gpio_dr);
	val |= BIT(3);
	writel(val, dev->gpio_dr);
}

/* ------------------------------------------------------------------ */
/* 定时器                                                               */
/* ------------------------------------------------------------------ */

static void led_timer_restart(struct led_dev *dev)
{
	unsigned long flags;
	int period;

	spin_lock_irqsave(&dev->lock, flags);
	period = dev->timeperiod;
	spin_unlock_irqrestore(&dev->lock, flags);

	mod_timer(&dev->timer, jiffies + msecs_to_jiffies(period));
}

static void led_timer_cb(unsigned long arg)
{
	struct led_dev *dev = (struct led_dev *)arg;

	dev->status = !dev->status;
	led_set_status(dev, dev->status);
	led_timer_restart(dev);
}

/* ------------------------------------------------------------------ */
/* 字符设备文件操作                                                      */
/* ------------------------------------------------------------------ */

static int led_open(struct inode *inode, struct file *filp)
{
	struct led_dev *dev = container_of(inode->i_cdev, struct led_dev, cdev);

	filp->private_data = dev;
	return 0;
}

static ssize_t led_write(struct file *filp, const char __user *buf,
			 size_t cnt, loff_t *offt)
{
	struct led_dev *dev = filp->private_data;
	unsigned char val;

	if (copy_from_user(&val, buf, 1))
		return -EFAULT;

	led_set_status(dev, val);
	return cnt;
}

static long led_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct led_dev *dev = filp->private_data;
	unsigned long flags;

	switch (cmd) {
	case LED_CLOSE_TIMER:
		del_timer_sync(&dev->timer);
		break;
	case LED_OPEN_TIMER:
		led_timer_restart(dev);
		break;
	case LED_SET_PERIOD:
		spin_lock_irqsave(&dev->lock, flags);
		dev->timeperiod = (int)arg;
		spin_unlock_irqrestore(&dev->lock, flags);
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations led_fops = {
	.owner          = THIS_MODULE,
	.open           = led_open,
	.write          = led_write,
	.unlocked_ioctl = led_ioctl,
};

/* ------------------------------------------------------------------ */
/* platform driver probe / remove                                       */
/* ------------------------------------------------------------------ */

static int led_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct led_dev *dev;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* 映射寄存器（索引与 DTS reg 属性顺序一致） */
	dev->sw_mux    = of_iomap(np, 0);
	dev->gpio_dr   = of_iomap(np, 1);
	dev->gpio_gdir = of_iomap(np, 2);
	dev->sw_pad    = of_iomap(np, 3);
	dev->ccm_ccgr1 = of_iomap(np, 4);

	if (!dev->sw_mux || !dev->gpio_dr || !dev->gpio_gdir ||
	    !dev->sw_pad || !dev->ccm_ccgr1) {
		dev_err(&pdev->dev, "of_iomap failed\n");
		ret = -ENOMEM;
		goto err_iomap;
	}

	led_hw_init(dev);

	/* 申请设备号 */
	ret = alloc_chrdev_region(&dev->dev_id, 0, 1, "led");
	if (ret) {
		dev_err(&pdev->dev, "alloc_chrdev_region failed\n");
		goto err_iomap;
	}

	/* 初始化并注册 cdev */
	cdev_init(&dev->cdev, &led_fops);
	ret = cdev_add(&dev->cdev, dev->dev_id, 1);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed\n");
		goto err_chrdev;
	}

	/* 创建 class 和 device，自动生成 /dev/led */
	dev->class = class_create(THIS_MODULE, "led");
	if (IS_ERR(dev->class)) {
		ret = PTR_ERR(dev->class);
		dev_err(&pdev->dev, "class_create failed\n");
		goto err_cdev;
	}

	dev->device = device_create(dev->class, &pdev->dev,
				    dev->dev_id, NULL, "led");
	if (IS_ERR(dev->device)) {
		ret = PTR_ERR(dev->device);
		dev_err(&pdev->dev, "device_create failed\n");
		goto err_class;
	}

	/* 初始化定时器 */
	spin_lock_init(&dev->lock);
	init_timer(&dev->timer);
	dev->timer.function = led_timer_cb;
	dev->timer.data     = (unsigned long)dev;
	dev->timeperiod     = LED_DEFAULT_PERIOD;

	platform_set_drvdata(pdev, dev);
	dev_info(&pdev->dev, "LED driver probed\n");
	return 0;

err_class:
	class_destroy(dev->class);
err_cdev:
	cdev_del(&dev->cdev);
err_chrdev:
	unregister_chrdev_region(dev->dev_id, 1);
err_iomap:
	if (dev->sw_mux)    iounmap(dev->sw_mux);
	if (dev->gpio_dr)   iounmap(dev->gpio_dr);
	if (dev->gpio_gdir) iounmap(dev->gpio_gdir);
	if (dev->sw_pad)    iounmap(dev->sw_pad);
	if (dev->ccm_ccgr1) iounmap(dev->ccm_ccgr1);
	return ret;
}

static int led_remove(struct platform_device *pdev)
{
	struct led_dev *dev = platform_get_drvdata(pdev);

	del_timer_sync(&dev->timer);
	device_destroy(dev->class, dev->dev_id);
	class_destroy(dev->class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->dev_id, 1);
	iounmap(dev->sw_mux);
	iounmap(dev->gpio_dr);
	iounmap(dev->gpio_gdir);
	iounmap(dev->sw_pad);
	iounmap(dev->ccm_ccgr1);

	dev_info(&pdev->dev, "LED driver removed\n");
	return 0;
}

static const struct of_device_id led_of_match[] = {
	{ .compatible = "alientek,led" },
	{ }
};
MODULE_DEVICE_TABLE(of, led_of_match);

static struct platform_driver led_driver = {
	.probe  = led_probe,
	.remove = led_remove,
	.driver = {
		.name           = "alientek-led",
		.of_match_table = led_of_match,
	},
};
module_platform_driver(led_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zrc");
MODULE_DESCRIPTION("iMX6ULL LED platform driver");
