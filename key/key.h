#ifndef __KEY_H
#define __KEY_H

#include <linux/gpio/consumer.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/spinlock.h>

struct key_dev {
	struct gpio_desc  *gpiod;    /* GPIO 描述符，devm_ 自动释放 */
	struct miscdevice  miscdev;  /* misc 设备，自动创建 /dev/key */
	struct timer_list  timer;    /* 消抖定时器 */
	spinlock_t         lock;
	unsigned int       irqnum;
	unsigned char      key_val;  /* 消抖后的键值：1=按下 0=松开 */
};

#endif /* __KEY_H */
