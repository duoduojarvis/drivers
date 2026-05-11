#ifndef __LED_H
#define __LED_H

#include <linux/cdev.h>
#include <linux/timer.h>
#include <linux/spinlock.h>

struct led_dev {
	void __iomem *ccm_ccgr1;
	void __iomem *sw_mux;
	void __iomem *sw_pad;
	void __iomem *gpio_dr;
	void __iomem *gpio_gdir;

	dev_t          dev_id;
	struct cdev    cdev;
	struct class  *class;
	struct device *device;

	struct timer_list timer;
	spinlock_t        lock;
	int               timeperiod; /* 定时器周期，单位 ms */
	unsigned char     status;     /* 0: LED 关闭，1: LED 打开 */
};

#endif /* __LED_H */
