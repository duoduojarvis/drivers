#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include "miscbeep.h"

/* ------------------------------------------------------------------ */
/* 文件操作                                                             */
/* ------------------------------------------------------------------ */

static int beep_open(struct inode *inode, struct file *filp)
{
	/*
	 * misc 框架在 open 时将 filp->private_data 设置为 miscdevice 指针，
	 * 通过 container_of 反推出外层 beep_dev 结构体。
	 */
	struct miscdevice *miscdev = filp->private_data;
	struct beep_dev *dev = container_of(miscdev, struct beep_dev, miscdev);

	filp->private_data = dev;
	return 0;
}

static ssize_t beep_write(struct file *filp, const char __user *buf,
			  size_t size, loff_t *offset)
{
	struct beep_dev *dev = filp->private_data;
	unsigned char onoff;

	if (copy_from_user(&onoff, buf, 1))
		return -EFAULT;

	/*
	 * gpiod_set_value 使用逻辑电平：
	 *   1 = 逻辑高 = 有效（蜂鸣器响，DTS 中已声明 GPIO_ACTIVE_LOW）
	 *   0 = 逻辑低 = 无效（蜂鸣器停）
	 */
	if (onoff)
		gpiod_set_value(dev->gpiod, 1);
	else
		gpiod_set_value(dev->gpiod, 0);

	return size;
}

static const struct file_operations beep_fops = {
	.owner = THIS_MODULE,
	.open  = beep_open,
	.write = beep_write,
};

/* ------------------------------------------------------------------ */
/* platform driver probe / remove                                       */
/* ------------------------------------------------------------------ */

static int beep_probe(struct platform_device *pdev)
{
	struct beep_dev *dev;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/*
	 * 从 DTS 获取 GPIO 描述符。
	 * "beep" 对应 DTS 属性 "beep-gpio"，GPIOD_OUT_LOW 表示初始
	 * 逻辑低（蜂鸣器关闭）。devm_ 版本在设备卸载时自动释放。
	 */
	dev->gpiod = devm_gpiod_get(&pdev->dev, "beep", GPIOD_OUT_LOW);
	if (IS_ERR(dev->gpiod)) {
		dev_err(&pdev->dev, "Failed to get beep gpio\n");
		return PTR_ERR(dev->gpiod);
	}

	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name  = "beep";
	dev->miscdev.fops  = &beep_fops;

	ret = misc_register(&dev->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "misc_register failed\n");
		return ret;
	}

	platform_set_drvdata(pdev, dev);
	dev_info(&pdev->dev, "beep driver probed\n");
	return 0;
}

static int beep_remove(struct platform_device *pdev)
{
	struct beep_dev *dev = platform_get_drvdata(pdev);

	gpiod_set_value(dev->gpiod, 0); /* 卸载前确保蜂鸣器关闭 */
	misc_deregister(&dev->miscdev);
	dev_info(&pdev->dev, "beep driver removed\n");
	return 0;
}

static const struct of_device_id beep_of_match[] = {
	{ .compatible = "alientek,beep" },
	{ }
};
MODULE_DEVICE_TABLE(of, beep_of_match);

static struct platform_driver beep_driver = {
	.probe  = beep_probe,
	.remove = beep_remove,
	.driver = {
		.name           = "alientek-beep",
		.of_match_table = beep_of_match,
	},
};
module_platform_driver(beep_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zrc");
MODULE_DESCRIPTION("iMX6ULL beep driver (miscdevice)");
