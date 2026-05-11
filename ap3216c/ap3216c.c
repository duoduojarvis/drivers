/*
 * ap3216c.c - IIO driver for Lite-On AP3216C
 *             ambient light sensor (ALS) + proximity sensor (PS) + IR
 *
 * Hardware:  iMX6ULL ALPHA, I2C1, address 0x1E
 * Interface: IIO subsystem
 *
 * Register map:
 *   0x00  System Configuration  (0=PDN, 1=ALS, 2=PS+IR, 3=ALS+PS+IR, 4=SW Reset)
 *   0x0A  IR  Data Low   bit7=IR_OF(invalid), bits[1:0]=IR[1:0]
 *   0x0B  IR  Data High  bits[7:0]=IR[9:2]   → 10-bit: (H << 2) | (L & 0x03)
 *   0x0C  ALS Data Low  [7:0]
 *   0x0D  ALS Data High [7:0]   → 16-bit: (H << 8) | L
 *   0x0E  PS  Data Low   bit7=object, bits[3:0]=PS[3:0]
 *   0x0F  PS  Data High  bits[5:0]=PS[9:4]   → 10-bit: (H & 0x3F) << 4 | (L & 0x0F)
 *
 * NOTE: 0x0A~0x0F 必须用 block read 一次读完，避免两次独立 I2C 读
 * 之间芯片更新寄存器导致高低字节不一致（竞争窗口问题）。
 *
 * Sysfs (after insmod + DTS match):
 *   /sys/bus/iio/devices/iio:deviceX/in_illuminance_raw   ← ALS
 *   /sys/bus/iio/devices/iio:deviceX/in_proximity_raw     ← PS
 *   /sys/bus/iio/devices/iio:deviceX/in_intensity_ir_raw  ← IR
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/bitops.h>

/* ------------------------------------------------------------------ */
/* Register definitions                                                 */
/* ------------------------------------------------------------------ */

#define AP3216C_REG_SYS     0x00  /* System Configuration */
#define AP3216C_REG_IR_L    0x0A  /* IR  Data Low         */
#define AP3216C_REG_IR_H    0x0B  /* IR  Data High        */
#define AP3216C_REG_ALS_L   0x0C  /* ALS Data Low         */
#define AP3216C_REG_ALS_H   0x0D  /* ALS Data High        */
#define AP3216C_REG_PS_L    0x0E  /* PS  Data Low         */
#define AP3216C_REG_PS_H    0x0F  /* PS  Data High        */

#define AP3216C_SYS_RESET   0x04  /* SW Reset             */
#define AP3216C_SYS_ALL     0x03  /* ALS + PS + IR active */

#define AP3216C_RESET_DELAY_MS   50   /* datasheet: >10 ms after reset   */
#define AP3216C_START_DELAY_MS  160   /* ALS needs ~100 ms for first meas */

/* ------------------------------------------------------------------ */
/* Device private data                                                  */
/* ------------------------------------------------------------------ */

struct ap3216c_data {
	struct i2c_client *client;
	struct mutex       lock;  /* serialize concurrent sysfs reads */
};

/* ------------------------------------------------------------------ */
/* Register read helper — burst read all 6 data bytes atomically        */
/* ------------------------------------------------------------------ */

/*
 * ap3216c_read_all: 一次 I2C block read 读取 0x0A~0x0F 共 6 字节。
 *
 * 原因：AP3216C 持续更新数据寄存器（ALS ~100ms/次，PS ~12.5ms/次）。
 * 若对每个通道做独立的 i2c_smbus_read_byte_data，高低字节之间存在
 * 竞争窗口——芯片可能在两次读之间更新寄存器，导致拼出错误值（如 256/512/768）。
 * block read 在同一 I2C 事务内完成，消除该竞争。
 *
 * buf 布局（对应寄存器 0x0A~0x0F）：
 *   buf[0] = IR_L  (0x0A)   buf[1] = IR_H  (0x0B)
 *   buf[2] = ALS_L (0x0C)   buf[3] = ALS_H (0x0D)
 *   buf[4] = PS_L  (0x0E)   buf[5] = PS_H  (0x0F)
 */
/*
 * ap3216c_read_reg16 - 2 字节原子读：写寄存器地址 + 读 2 字节，一个 I2C 事务。
 *
 * AP3216C 不支持 6 字节连续 block read（读 2 字节后地址指针会绕回）。
 * 因此对每个通道（IR/ALS/PS）各做一次独立的 2 字节读，
 * 保证每个通道的 L/H 字节来自同一事务，消除寄存器更新竞争。
 */
static int ap3216c_read_reg16(struct i2c_client *client, u8 reg, u8 buf[2])
{
	struct i2c_msg msgs[2] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 1,
			.buf   = &reg,
		},
		{
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = buf,
		},
	};
	int ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret == 2) ? 0 : (ret < 0 ? ret : -EIO);
}

static int ap3216c_read_all(struct ap3216c_data *data,
			    int *ir_val, int *als_val, int *ps_val)
{
	u8 ir_buf[2], als_buf[2], ps_buf[2];
	int ret;

	/* 3 次独立的 2 字节读，分别对应 IR/ALS/PS 寄存器对 */
	ret = ap3216c_read_reg16(data->client, AP3216C_REG_IR_L,  ir_buf);
	if (ret)
		return ret;

	ret = ap3216c_read_reg16(data->client, AP3216C_REG_ALS_L, als_buf);
	if (ret)
		return ret;

	ret = ap3216c_read_reg16(data->client, AP3216C_REG_PS_L,  ps_buf);
	if (ret)
		return ret;

	/* ALS 16-bit */
	*als_val = ((int)als_buf[1] << 8) | als_buf[0];

	/*
	 * IR_L bit7 = IR_OF（IR 数据无效标志）：置位时 IR 和 PS 读数均不可靠。
	 * 注意 IR 的字节序与 PS/ALS 相反：高字节是 IR[9:2]，低字节只用 [1:0]。
	 */
	if (ir_buf[0] & BIT(7)) {
		*ir_val = 0;
		*ps_val = 0;
	} else {
		*ir_val = ((int)ir_buf[1] << 2) | (ir_buf[0] & 0x03); /* 10-bit */
		*ps_val = ((ps_buf[1] & 0x3F) << 4) | (ps_buf[0] & 0x0F); /* 10-bit */
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* IIO channel spec                                                     */
/* ------------------------------------------------------------------ */

static const struct iio_chan_spec ap3216c_channels[] = {
	{
		/* Ambient Light Sensor → in_illuminance_raw */
		.type              = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		/* Proximity Sensor → in_proximity_raw */
		.type              = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		/* IR Intensity → in_intensity_ir_raw */
		.type              = IIO_INTENSITY,
		.modified          = 1,
		.channel2          = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

/* ------------------------------------------------------------------ */
/* IIO read_raw callback                                                */
/* ------------------------------------------------------------------ */

static int ap3216c_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	int ir, als, ps, ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&data->lock);
	ret = ap3216c_read_all(data, &ir, &als, &ps);
	mutex_unlock(&data->lock);

	if (ret < 0)
		return ret;

	switch (chan->type) {
	case IIO_LIGHT:
		*val = als;
		break;
	case IIO_PROXIMITY:
		*val = ps;
		break;
	case IIO_INTENSITY:
		*val = ir;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_info ap3216c_info = {
	.read_raw      = ap3216c_read_raw,
	.driver_module = THIS_MODULE,
};

/* ------------------------------------------------------------------ */
/* I2C probe / remove                                                   */
/* ------------------------------------------------------------------ */

static int ap3216c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ap3216c_data *data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C_FUNC_I2C not supported\n");
		return -EOPNOTSUPP;
	}

	/* Allocate iio_dev; private data appended after it (iio_priv) */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);
	i2c_set_clientdata(client, indio_dev);

	/* SW Reset */
	ret = i2c_smbus_write_byte_data(client, AP3216C_REG_SYS, AP3216C_SYS_RESET);
	if (ret < 0) {
		dev_err(&client->dev, "SW reset failed: %d\n", ret);
		return ret;
	}
	msleep(AP3216C_RESET_DELAY_MS);

	/* Enable ALS + PS + IR */
	ret = i2c_smbus_write_byte_data(client, AP3216C_REG_SYS, AP3216C_SYS_ALL);
	if (ret < 0) {
		dev_err(&client->dev, "enable failed: %d\n", ret);
		return ret;
	}
	msleep(AP3216C_START_DELAY_MS); /* wait for first valid measurement */

	indio_dev->dev.parent  = &client->dev;
	indio_dev->name        = "ap3216c";
	indio_dev->info        = &ap3216c_info;
	indio_dev->modes       = INDIO_DIRECT_MODE;
	indio_dev->channels    = ap3216c_channels;
	indio_dev->num_channels = ARRAY_SIZE(ap3216c_channels);

	/* devm_ variant: auto-unregisters on device removal */
	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		dev_err(&client->dev, "IIO register failed: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "AP3216C probe ok (ALS+PS+IR)\n");
	return 0;
}

static int ap3216c_remove(struct i2c_client *client)
{
	/* Power down the sensor on unload */
	i2c_smbus_write_byte_data(client, AP3216C_REG_SYS, 0x00);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Module boilerplate                                                   */
/* ------------------------------------------------------------------ */

static const struct i2c_device_id ap3216c_id[] = {
	{ "ap3216c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);

static const struct of_device_id ap3216c_of_match[] = {
	{ .compatible = "alientek,ap3216c" },
	{ }
};
MODULE_DEVICE_TABLE(of, ap3216c_of_match);

static struct i2c_driver ap3216c_driver = {
	.driver = {
		.name           = "ap3216c",
		.of_match_table = ap3216c_of_match,
	},
	.probe    = ap3216c_probe,
	.remove   = ap3216c_remove,
	.id_table = ap3216c_id,
};

module_i2c_driver(ap3216c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zrc");
MODULE_DESCRIPTION("Lite-On AP3216C ambient light + proximity IIO driver");
