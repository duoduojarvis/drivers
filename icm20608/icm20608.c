/*
 * icm20608.c - IIO driver for InvenSense ICM-20608
 *              6-axis IMU: 3-axis Accel + 3-axis Gyro, via SPI
 *
 * Hardware:  iMX6ULL ALPHA, ECSPI3
 *              SCLK → UART2_RX_DATA (GPIO1_IO21)
 *              MOSI → UART2_CTS_B   (GPIO1_IO22)
 *              MISO → UART2_RTS_B   (GPIO1_IO23)
 *              CS   → UART2_TX_DATA (GPIO1_IO20, GPIO 软件片选)
 *
 * SPI 协议：
 *   读：第1字节 = 0x80 | reg_addr；后续 N 字节为数据
 *   写：第1字节 = reg_addr；第2字节 = data
 *
 * 数据寄存器（14字节连续读，0x3B~0x48）：
 *   [AX_H AX_L][AY_H AY_L][AZ_H AZ_L][T_H T_L][GX_H GX_L][GY_H GY_L][GZ_H GZ_L]
 *
 * 工作模式：
 *   - 若 DTS 配置了 INT 引脚（spi->irq > 0）：
 *       IRQ 上半部 → schedule_work → work 下半部批量读取 → 缓存 raw[]
 *       read_raw 直接返回缓存值（无需等待 SPI）
 *   - 若无 INT 引脚（polling 模式）：
 *       read_raw 每次直接做 SPI burst read
 *
 * Sysfs：
 *   in_accel_x_raw / in_accel_y_raw / in_accel_z_raw
 *   in_anglvel_x_raw / in_anglvel_y_raw / in_anglvel_z_raw
 *   in_temp_raw
 *   in_accel_scale / in_anglvel_scale / in_temp_scale / in_temp_offset
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/bitops.h>

/* ------------------------------------------------------------------ */
/* 寄存器定义                                                            */
/* ------------------------------------------------------------------ */

#define ICM20608_SMPLRT_DIV     0x19  /* 采样率分频 */
#define ICM20608_CONFIG         0x1A  /* DLPF 配置  */
#define ICM20608_GYRO_CONFIG    0x1B  /* 陀螺仪量程 */
#define ICM20608_ACCEL_CONFIG   0x1C  /* 加速度量程 */
#define ICM20608_ACCEL_CONFIG2  0x1D  /* 加速度 DLPF */
#define ICM20608_INT_ENABLE     0x38  /* 中断使能   */
#define ICM20608_INT_STATUS     0x3A  /* 中断状态   */
#define ICM20608_ACCEL_XOUT_H   0x3B  /* 数据起始   */
#define ICM20608_PWR_MGMT_1     0x6B  /* 电源管理 1 */
#define ICM20608_PWR_MGMT_2     0x6C  /* 电源管理 2 */
#define ICM20608_WHO_AM_I       0x75  /* 设备 ID    */

#define ICM20608_ID_G           0xAF  /* ICM-20608G */
#define ICM20608_ID             0xAE  /* ICM-20608  (ALIENTEK 板) */

/* 初始化配置值 */
#define ICM20608_SMPLRT_DIV_VAL  0x04  /* 200 Hz = 1kHz / (1+4) */
#define ICM20608_CONFIG_VAL      0x04  /* 陀螺仪 DLPF ~20Hz */
#define ICM20608_GYRO_FS_2000    0x18  /* ±2000 dps */
#define ICM20608_ACCEL_FS_16G    0x18  /* ±16g */
#define ICM20608_ACCEL_CFG2_VAL  0x04  /* 加速度 DLPF ~20Hz */

#define ICM20608_RESET_DELAY_MS  100   /* 复位后等待时间 */

/* raw[] 数组下标（也作为 iio_chan_spec.address）*/
#define IDX_ACCEL_X  0
#define IDX_ACCEL_Y  1
#define IDX_ACCEL_Z  2
#define IDX_TEMP     3
#define IDX_GYRO_X   4
#define IDX_GYRO_Y   5
#define IDX_GYRO_Z   6

/* ------------------------------------------------------------------ */
/* 设备私有数据                                                          */
/* ------------------------------------------------------------------ */

struct icm20608_data {
	struct spi_device  *spi;
	struct mutex        lock;    /* 保护 raw[] 和 SPI 并发访问 */
	struct work_struct  work;    /* workqueue 下半部：批量读取传感器数据 */
	s16                 raw[7];  /* 缓存：ax ay az temp gx gy gz */
};

/* ------------------------------------------------------------------ */
/* SPI 寄存器读写                                                        */
/* ------------------------------------------------------------------ */

/* 读单个寄存器 */
static int icm20608_reg_read(struct spi_device *spi, u8 reg, u8 *val)
{
	u8 cmd = reg | BIT(7);  /* bit7=1 表示读操作 */

	return spi_write_then_read(spi, &cmd, 1, val, 1);
}

/* 写单个寄存器 */
static int icm20608_reg_write(struct spi_device *spi, u8 reg, u8 val)
{
	u8 buf[2] = { reg & ~BIT(7), val };  /* bit7=0 表示写操作 */

	return spi_write(spi, buf, 2);
}

/*
 * icm20608_bulk_read - 从 reg 开始连续读 len 字节。
 * spi_write_then_read：发送1字节地址后，在同一 SPI 事务内读取 len 字节，
 * CS 全程保持有效，保证数据原子性。
 */
static int icm20608_bulk_read(struct spi_device *spi, u8 reg,
			      u8 *buf, int len)
{
	u8 cmd = reg | BIT(7);

	return spi_write_then_read(spi, &cmd, 1, buf, len);
}

/* ------------------------------------------------------------------ */
/* 批量读取所有传感器数据                                                 */
/* ------------------------------------------------------------------ */

/*
 * icm20608_read_all - 一次 SPI 事务读取寄存器 0x3B~0x48（14字节）。
 *
 * 布局：[AX_H AX_L AY_H AY_L AZ_H AZ_L T_H T_L GX_H GX_L GY_H GY_L GZ_H GZ_L]
 *
 * 14字节在同一 CS 事务内读完，保证同一采样周期内数据一致性。
 * 必须在持有 data->lock 的情况下调用。
 */
static int icm20608_read_all(struct icm20608_data *data)
{
	u8 buf[14];
	int ret;

	ret = icm20608_bulk_read(data->spi, ICM20608_ACCEL_XOUT_H, buf, 14);
	if (ret)
		return ret;

	data->raw[IDX_ACCEL_X] = (s16)((buf[0]  << 8) | buf[1]);
	data->raw[IDX_ACCEL_Y] = (s16)((buf[2]  << 8) | buf[3]);
	data->raw[IDX_ACCEL_Z] = (s16)((buf[4]  << 8) | buf[5]);
	data->raw[IDX_TEMP]    = (s16)((buf[6]  << 8) | buf[7]);
	data->raw[IDX_GYRO_X]  = (s16)((buf[8]  << 8) | buf[9]);
	data->raw[IDX_GYRO_Y]  = (s16)((buf[10] << 8) | buf[11]);
	data->raw[IDX_GYRO_Z]  = (s16)((buf[12] << 8) | buf[13]);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Workqueue 下半部                                                      */
/* ------------------------------------------------------------------ */

/*
 * icm20608_work_handler - 由 IRQ 上半部触发，运行在进程上下文。
 *
 * 上半部（icm20608_irq_handler）只做 schedule_work，不做任何 SPI 操作；
 * 下半部在 workqueue 线程中执行实际的 SPI burst read，更新 raw[] 缓存。
 *
 * 这样做的原因：spi_sync 可能睡眠（等待总线空闲），不能在中断上下文调用；
 * workqueue 运行在内核线程，允许睡眠。
 */
static void icm20608_work_handler(struct work_struct *work)
{
	struct icm20608_data *data =
		container_of(work, struct icm20608_data, work);
	int ret;

	mutex_lock(&data->lock);
	ret = icm20608_read_all(data);
	mutex_unlock(&data->lock);

	if (ret)
		dev_err_ratelimited(&data->spi->dev,
				    "read_all error: %d\n", ret);
}

/* ------------------------------------------------------------------ */
/* IRQ 上半部                                                            */
/* ------------------------------------------------------------------ */

/*
 * icm20608_irq_handler - DATA_RDY 中断处理程序（上半部）。
 *
 * 运行在中断上下文，不能睡眠，不能调用 SPI。
 * 仅把实际工作投递到 workqueue 下半部处理。
 */
static irqreturn_t icm20608_irq_handler(int irq, void *dev_id)
{
	struct icm20608_data *data = dev_id;

	schedule_work(&data->work);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* IIO 通道描述                                                          */
/* ------------------------------------------------------------------ */

/*
 * 加速度计通道宏：type=IIO_ACCEL，修饰符区分 X/Y/Z
 * 生成 sysfs 属性：in_accel_x_raw / in_accel_y_raw / in_accel_z_raw
 *                  in_accel_scale（三轴共享）
 */
#define ICM20608_ACCEL_CHAN(_mod, _idx)					\
{									\
	.type                    = IIO_ACCEL,				\
	.modified                = 1,					\
	.channel2                = (_mod),				\
	.address                 = (_idx),				\
	.info_mask_separate      = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
}

/*
 * 陀螺仪通道宏：type=IIO_ANGL_VEL
 * 生成 sysfs 属性：in_anglvel_x_raw 等
 */
#define ICM20608_GYRO_CHAN(_mod, _idx)					\
{									\
	.type                    = IIO_ANGL_VEL,			\
	.modified                = 1,					\
	.channel2                = (_mod),				\
	.address                 = (_idx),				\
	.info_mask_separate      = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec icm20608_channels[] = {
	ICM20608_ACCEL_CHAN(IIO_MOD_X, IDX_ACCEL_X),  /* in_accel_x_raw  */
	ICM20608_ACCEL_CHAN(IIO_MOD_Y, IDX_ACCEL_Y),  /* in_accel_y_raw  */
	ICM20608_ACCEL_CHAN(IIO_MOD_Z, IDX_ACCEL_Z),  /* in_accel_z_raw  */
	{
		/* in_temp_raw，in_temp_scale，in_temp_offset */
		.type               = IIO_TEMP,
		.address            = IDX_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				    | BIT(IIO_CHAN_INFO_SCALE)
				    | BIT(IIO_CHAN_INFO_OFFSET),
	},
	ICM20608_GYRO_CHAN(IIO_MOD_X, IDX_GYRO_X),    /* in_anglvel_x_raw */
	ICM20608_GYRO_CHAN(IIO_MOD_Y, IDX_GYRO_Y),    /* in_anglvel_y_raw */
	ICM20608_GYRO_CHAN(IIO_MOD_Z, IDX_GYRO_Z),    /* in_anglvel_z_raw */
};

/* ------------------------------------------------------------------ */
/* IIO read_raw 回调                                                     */
/* ------------------------------------------------------------------ */

static int icm20608_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct icm20608_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (data->spi->irq > 0) {
			/*
			 * IRQ 模式：workqueue 持续更新 raw[]，
			 * 这里只需加锁读缓存，无需触发 SPI。
			 */
			mutex_lock(&data->lock);
			*val = data->raw[chan->address];
			mutex_unlock(&data->lock);
		} else {
			/*
			 * Polling 模式：按需直接做 SPI burst read。
			 */
			mutex_lock(&data->lock);
			ret = icm20608_read_all(data);
			if (!ret)
				*val = data->raw[chan->address];
			mutex_unlock(&data->lock);
			if (ret)
				return ret;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			/*
			 * 量程 ±16g：灵敏度 = 2048 LSB/g
			 * 1 LSB = 1/2048 g = 9.80665/2048 m/s² ≈ 0.004788 m/s²
			 */
			*val  = 0;
			*val2 = 4788;   /* μ单位：0.004788 m/s²/LSB */
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_ANGL_VEL:
			/*
			 * 量程 ±2000 dps：灵敏度 = 16.4 LSB/dps
			 * 1 LSB = 0.06098 dps = 0.06098 * π/180 rad/s ≈ 0.001065 rad/s
			 */
			*val  = 0;
			*val2 = 1065;   /* μ单位：0.001065 rad/s/LSB */
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			/*
			 * 温度灵敏度：326.8 LSB/°C → 1 LSB = 0.003059 °C
			 */
			*val  = 0;
			*val2 = 3059;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			/*
			 * T(°C) = raw / 326.8 + 25
			 * IIO 约定：实际值 = (raw + offset) * scale
			 * → offset = 25 / (1/326.8) = 25 * 326.8 = 8170
			 */
			*val = 8170;
			return IIO_VAL_INT;
		}
		return -EINVAL;

	default:
		return -EINVAL;
	}
}

static const struct iio_info icm20608_info = {
	.read_raw      = icm20608_read_raw,
	.driver_module = THIS_MODULE,
};

/* ------------------------------------------------------------------ */
/* 芯片初始化                                                             */
/* ------------------------------------------------------------------ */

static int icm20608_hw_init(struct icm20608_data *data)
{
	struct spi_device *spi = data->spi;
	u8 id;
	int ret;

	/* 读 WHO_AM_I 确认芯片 */
	ret = icm20608_reg_read(spi, ICM20608_WHO_AM_I, &id);
	if (ret) {
		dev_err(&spi->dev, "WHO_AM_I read failed: %d\n", ret);
		return ret;
	}
	if (id != ICM20608_ID && id != ICM20608_ID_G) {
		dev_err(&spi->dev, "unexpected WHO_AM_I=0x%02X\n", id);
		return -ENODEV;
	}
	dev_info(&spi->dev, "WHO_AM_I=0x%02X OK\n", id);

	/* SW Reset */
	ret = icm20608_reg_write(spi, ICM20608_PWR_MGMT_1, BIT(7));
	if (ret)
		return ret;
	msleep(ICM20608_RESET_DELAY_MS);

	/* 退出 sleep，选择 PLL 时钟源 */
	ret = icm20608_reg_write(spi, ICM20608_PWR_MGMT_1, 0x01);
	if (ret)
		return ret;

	/* 使能所有轴 */
	ret = icm20608_reg_write(spi, ICM20608_PWR_MGMT_2, 0x00);
	if (ret)
		return ret;

	/* 采样率 200Hz */
	ret = icm20608_reg_write(spi, ICM20608_SMPLRT_DIV, ICM20608_SMPLRT_DIV_VAL);
	if (ret)
		return ret;

	/* 陀螺仪 DLPF */
	ret = icm20608_reg_write(spi, ICM20608_CONFIG, ICM20608_CONFIG_VAL);
	if (ret)
		return ret;

	/* 陀螺仪量程 ±2000 dps */
	ret = icm20608_reg_write(spi, ICM20608_GYRO_CONFIG, ICM20608_GYRO_FS_2000);
	if (ret)
		return ret;

	/* 加速度量程 ±16g */
	ret = icm20608_reg_write(spi, ICM20608_ACCEL_CONFIG, ICM20608_ACCEL_FS_16G);
	if (ret)
		return ret;

	/* 加速度 DLPF */
	ret = icm20608_reg_write(spi, ICM20608_ACCEL_CONFIG2, ICM20608_ACCEL_CFG2_VAL);
	if (ret)
		return ret;

	return 0;
}

/* ------------------------------------------------------------------ */
/* SPI probe / remove                                                   */
/* ------------------------------------------------------------------ */

static int icm20608_probe(struct spi_device *spi)
{
	struct iio_dev       *indio_dev;
	struct icm20608_data *data;
	int ret;

	/* 确认 SPI 参数 */
	spi->bits_per_word = 8;
	spi->mode          = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		return ret;
	}

	/* 分配 iio_dev（私有数据追加在尾部）*/
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->spi = spi;
	mutex_init(&data->lock);
	INIT_WORK(&data->work, icm20608_work_handler);
	spi_set_drvdata(spi, indio_dev);

	/* 硬件初始化 */
	ret = icm20608_hw_init(data);
	if (ret)
		return ret;

	/* IRQ 配置（可选：需要 DTS 中配置 interrupts 属性）*/
	if (spi->irq > 0) {
		/* 使能芯片 DATA_RDY 中断 */
		ret = icm20608_reg_write(spi, ICM20608_INT_ENABLE, BIT(0));
		if (ret)
			return ret;

		ret = devm_request_irq(&spi->dev, spi->irq,
				       icm20608_irq_handler,
				       IRQF_TRIGGER_RISING,
				       "icm20608", data);
		if (ret) {
			dev_err(&spi->dev, "request_irq failed: %d\n", ret);
			return ret;
		}
		dev_info(&spi->dev, "IRQ mode: irq=%d\n", spi->irq);
	} else {
		dev_info(&spi->dev, "polling mode (no INT pin)\n");
	}

	/* 填充 IIO 描述符 */
	indio_dev->dev.parent   = &spi->dev;
	indio_dev->name         = "icm20608";
	indio_dev->info         = &icm20608_info;
	indio_dev->modes        = INDIO_DIRECT_MODE;
	indio_dev->channels     = icm20608_channels;
	indio_dev->num_channels = ARRAY_SIZE(icm20608_channels);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "IIO register failed: %d\n", ret);
		return ret;
	}

	dev_info(&spi->dev, "ICM-20608 probe ok (accel+gyro+temp)\n");
	return 0;
}

static int icm20608_remove(struct spi_device *spi)
{
	struct iio_dev       *indio_dev = spi_get_drvdata(spi);
	struct icm20608_data *data      = iio_priv(indio_dev);

	/* 等待 workqueue 中未完成的任务结束，再 devm 自动注销 IIO */
	cancel_work_sync(&data->work);

	/* 芯片进入睡眠 */
	icm20608_reg_write(spi, ICM20608_PWR_MGMT_1, BIT(6));
	return 0;
}

/* ------------------------------------------------------------------ */
/* 模块注册                                                               */
/* ------------------------------------------------------------------ */

static const struct spi_device_id icm20608_id[] = {
	{ "icm20608", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, icm20608_id);

static const struct of_device_id icm20608_of_match[] = {
	{ .compatible = "alientek,icm20608" },
	{ }
};
MODULE_DEVICE_TABLE(of, icm20608_of_match);

static struct spi_driver icm20608_driver = {
	.driver = {
		.name           = "icm20608",
		.of_match_table = icm20608_of_match,
	},
	.probe    = icm20608_probe,
	.remove   = icm20608_remove,
	.id_table = icm20608_id,
};

module_spi_driver(icm20608_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zrc");
MODULE_DESCRIPTION("InvenSense ICM-20608 6-axis IMU IIO driver");
