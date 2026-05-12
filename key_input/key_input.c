/*
 * key_input.c - 基于 Linux Input 子系统的按键驱动
 *
 * 硬件：iMX6ULL ALPHA，GPIO1_IO18，低有效
 * 框架：Input 子系统（对比旧版：misc + 自定义 read）
 *
 * 与旧版 key.c 的区别：
 *   旧版：miscdevice + 自定义 read()，用户空间主动轮询 /dev/key
 *   新版：input_dev + input_report_key()，内核主动上报事件到 /dev/input/eventX
 *         用户空间用标准 read(struct input_event) 阻塞等待，无需轮询
 *
 * 用户空间读取：
 *   struct input_event ev;
 *   read(fd, &ev, sizeof(ev));
 *   if (ev.type == EV_KEY && ev.code == KEY_0)
 *       printf("%s\n", ev.value ? "pressed" : "released");
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>

#define KEY_DEBOUNCE_MS  10   /* 防抖延时，ms */

/* ------------------------------------------------------------------ */
/* 设备私有数据                                                          */
/* ------------------------------------------------------------------ */

struct key_input_dev {
	struct gpio_desc  *gpiod;    /* GPIO 描述符，devm 管理 */
	struct input_dev  *idev;     /* Input 设备，devm 管理 */
	struct timer_list  timer;    /* 防抖定时器 */
	unsigned int       irqnum;   /* IRQ 编号 */
};

/* ------------------------------------------------------------------ */
/* 防抖定时器回调                                                        */
/* ------------------------------------------------------------------ */

/*
 * key_input_timer - 防抖定时器到期后执行，运行在软中断上下文。
 *
 * gpiod_get_value 返回逻辑值：
 *   GPIO_ACTIVE_LOW 情况下：物理低电平 → 逻辑 1（按下）
 *                           物理高电平 → 逻辑 0（松开）
 *
 * input_report_key：向 Input 核心上报键值和状态
 * input_sync：      提交本轮所有事件，触发用户空间 read 返回
 */
static void key_input_timer(unsigned long arg)
{
	struct key_input_dev *dev = (struct key_input_dev *)arg;
	int val = gpiod_get_value(dev->gpiod);  /* 1=按下，0=松开 */

	input_report_key(dev->idev, KEY_0, val);
	input_sync(dev->idev);
}

/* ------------------------------------------------------------------ */
/* IRQ 上半部                                                            */
/* ------------------------------------------------------------------ */

/*
 * key_input_irq - 上升/下降沿中断处理程序。
 *
 * 只做一件事：重置防抖定时器。
 * 10ms 内没有新的边沿触发，定时器到期，才去读 GPIO 真实电平。
 */
static irqreturn_t key_input_irq(int irq, void *data)
{
	struct key_input_dev *dev = data;

	mod_timer(&dev->timer, jiffies + msecs_to_jiffies(KEY_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Platform probe / remove                                              */
/* ------------------------------------------------------------------ */

static int key_input_probe(struct platform_device *pdev)
{
	struct key_input_dev *dev;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* ① 获取 GPIO */
	dev->gpiod = devm_gpiod_get(&pdev->dev, "key", GPIOD_IN);
	if (IS_ERR(dev->gpiod)) {
		dev_err(&pdev->dev, "get gpio failed: %ld\n",
			PTR_ERR(dev->gpiod));
		return PTR_ERR(dev->gpiod);
	}

	/* ② 分配 Input 设备（devm 版，移除时自动注销+释放）*/
	dev->idev = devm_input_allocate_device(&pdev->dev);
	if (!dev->idev)
		return -ENOMEM;

	dev->idev->name       = "key-input";
	dev->idev->dev.parent = &pdev->dev;

	/*
	 * ③ 声明支持的事件类型和键码：
	 *    EV_KEY：按键事件类型
	 *    KEY_0 ：键码（可改成 KEY_ENTER、KEY_POWER 等任意键码）
	 *
	 * Input 核心根据此声明决定是否把事件传给上层（evdev、kbd 等）
	 */
	input_set_capability(dev->idev, EV_KEY, KEY_0);

	/* ④ 注册到 Input 核心，自动创建 /dev/input/eventX */
	ret = input_register_device(dev->idev);
	if (ret) {
		dev_err(&pdev->dev, "input_register_device failed: %d\n", ret);
		return ret;
	}

	/* ⑤ 初始化防抖定时器 */
	setup_timer(&dev->timer, key_input_timer, (unsigned long)dev);

	/* ⑥ 申请 IRQ（双沿触发，上升=松开，下降=按下）*/
	dev->irqnum = gpiod_to_irq(dev->gpiod);
	ret = devm_request_irq(&pdev->dev, dev->irqnum,
			       key_input_irq,
			       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			       "key-input", dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, dev);
	dev_info(&pdev->dev, "key-input probe ok (irq=%d)\n", dev->irqnum);
	return 0;
}

static int key_input_remove(struct platform_device *pdev)
{
	struct key_input_dev *dev = platform_get_drvdata(pdev);

	/* 等待定时器完成，防止 remove 后定时器回调访问已释放内存 */
	del_timer_sync(&dev->timer);

	/* devm 自动完成：input_unregister_device / free_irq / gpiod_put */
	return 0;
}

/* ------------------------------------------------------------------ */
/* 模块注册                                                               */
/* ------------------------------------------------------------------ */

static const struct of_device_id key_input_of_match[] = {
	{ .compatible = "alientek,key-input" },
	{ }
};
MODULE_DEVICE_TABLE(of, key_input_of_match);

static struct platform_driver key_input_driver = {
	.driver = {
		.name           = "key-input",
		.of_match_table = key_input_of_match,
	},
	.probe  = key_input_probe,
	.remove = key_input_remove,
};

module_platform_driver(key_input_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zrc");
MODULE_DESCRIPTION("Key driver using Linux Input subsystem");
