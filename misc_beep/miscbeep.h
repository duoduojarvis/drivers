#ifndef __MISCBEEP_H
#define __MISCBEEP_H

#include <linux/miscdevice.h>
#include <linux/gpio/consumer.h>

struct beep_dev {
	struct gpio_desc  *gpiod;     /* GPIO 描述符，devm_ 自动释放 */
	struct miscdevice  miscdev;   /* misc 设备，自动创建 /dev/beep */
};

#endif /* __MISCBEEP_H */
