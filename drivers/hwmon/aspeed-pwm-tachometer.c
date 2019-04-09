/*
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#define ASPEED_PWM_CTRL			0x00	//PWM0 General Register
#define ASPEED_PWM_CTRL_CH(x)		((x * 0x10) + 0x00)
#define  PWM_LOAD_AS_WDT			BIT(19)	//load selection as WDT
#define  PWM_DUTY_LOAD_AS_WDT_EN	BIT(18)	//enable PWM duty load as WDT
#define  PWM_DUTY_SYNC_DIS			BIT(17)	//disable PWM duty sync
#define	 PWM_CLK_ENABLE				BIT(16)	//enable PWM clock
#define  PWM_LEVEL_OUTPUT			BIT(15)	//output PWM level
#define  PWM_INVERSE				BIT(14) //inverse PWM pin
#define  PWM_OPEN_DRAIN_EN			BIT(13)	//enable open-drain
#define  PWM_PIN_EN					BIT(12) //enable PWM pin
#define  PWM_CLK_DIV_H_MASK			(0xf << 8)	//PWM clock division H bit [3:0]
#define  PWM_CLK_DIV_L_MASK			(0xff)	//PWM clock division H bit [3:0]

/*
\xregmid {11:8 }{RW}{PWM clock division H bit [3:0]}{
                     0: divide 1 \n
                     1: divide 2 \n
                     2: divide 4 \n
                     3: divide 8 \n
                     ... \n
                     F: divide 32768}
\xregmid {7 :0 }{RW}{PWM clock division L bit [7:0]}{
                     00: divide 1 \n
                     01: divide 2 \n
                     02: divide 3 \n
                     03: divide 4 \n
                     ... \n
                     FF: divide 256}
*/

#define ASPEED_PWM_DUTY_CYCLE		0x04	//PWM0 Duty Cycle Register
#define ASPEED_PWM_DUTY_CYCLE_CH(x)		((x * 0x10) + 0x04)
#define  PWM_LOOP_BIT_MASK				(0xf << 24)	//loop bit [7:0]
#define  PWM_PERIOD_BIT					(24)	//pwm period bit [7:0]
#define  PWM_PERIOD_BIT_MASK			(0xff << 24)	//pwm period bit [7:0]
#define  PWM_RISING_FALLING_AS_WDT_MASK (0xff << 16)	//pwm rising/falling point bit [7:0] as WDT
#define  PWM_RISING_FALLING_MASK		(0xffff)	
#define  PWM_RISING_FALLING_BIT			(8)	//pwm falling point bit [7:0]
#define  PWM_RISING_RISING_BIT			(0)	//pwm rising point bit [7:0]

#define ASPEED_TECHO_CTRL		0x08	//TACH0 General Register
#define ASPEED_TECHO_CTRL_CH(x)			((x * 0x10) + 0x08)
#define  TECHO_IER						BIT(31)	//enable tacho interrupt
#define  TECHO_INVERS_LIMIT				BIT(30) //inverse tacho limit comparison
#define  TECHO_LOOPBACK					BIT(29) //tacho loopback
#define  TECHO_ENABLE					BIT(28)	//{enable tacho}
#define  TECHO_DEBOUNCE_MASK			(0x3 << 26) //{tacho de-bounce}
#define  TECHIO_EDGE_MASK				(0x3 << 24) //tacho edge}
#define  TECHIO_EDGE_BIT				(24) //tacho edge}
#define  TECHO_CLK_DIV_T_MASK			(0xf << 20) 
#define  TECHO_CLK_DIV_BIT				(20)
#define  TECHO_THRESHOLD_MASK			(0xfffff)	//tacho threshold bit
/*
\xregmid {23:20}{RW}{tacho clock division T bit [3:0]}{
                     0: divide 1 \n
                     1: divide 4 \n
                     2: divide 16 \n
                     3: divide 64 \n
                     ... \n
                     B: divide 4194304 \n
					 others: reserved}
\xregmidb{19 :0 }{RW}{tacho threshold bit [19:0]}
*/

#define ASPEED_TECHO_STS		0x0C	//TACH0 Status Register
#define ASPEED_TECHO_STS_CH(x)			((x * 0x10) + 0x0C)
#define  TECHO_ISR			BIT(31)	//interrupt status and clear
#define  PWM_OUT			BIT(25)	//{pwm_out}
#define  PWM_OEN			BIT(24)	//{pwm_oeN}
#define  TACHO_DEB_INPUT	BIT(23)	//tacho deB input
#define  TACHO_RAW_INPUT	BIT(22) //tacho raw input}
#define  TACHO_VALUE_UPDATE	BIT(21)	//tacho value updated since the last read
#define  TACHO_FULL_MEASUREMENT	BIT(20) //{tacho full measurement}
#define  TACHO_VALUE_MASK	0xfffff	//tacho value bit [19:0]}

#define MAX_CDEV_NAME_LEN 16

struct aspeed_pwm_channel_params {
	int load_wdt_selection;		//0: rising , 1: falling
	int load_wdt_enable;
	int	duty_sync_enable;
	int invert_pin;
	u8	divide_h;
	u8	divide_l;
	u8	period;
	u8	rising;
	u8	falling;
};


static const struct aspeed_pwm_channel_params default_pwm_params[] = {
	[0] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,	
	},
	[1] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,
	},
	[2] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[3] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[4] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[5] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[6] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[7] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[8] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[9] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[10] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[11] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[12] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[13] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[14] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
	[15] = {
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.divide_h = 20,
		.divide_l = 16,
		.period = 0xff,
		.rising = 0x00,
		.falling = 0x80,		
	},
};

/*
 * 5:4 fan tach edge mode selection bit:
 * 00: falling
 * 01: rising
 * 10: both
 * 11: reserved.
 */

struct aspeed_techo_channel_params {
	int limited_inverse;
	u16 threshold;
	u8	tacho_edge;
	u8	divide;
};


static const struct aspeed_techo_channel_params default_techo_params[] = {
	[0] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[1] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[2] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[3] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[4] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[5] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[6] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[7] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[8] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[9] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[10] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[11] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[12] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[13] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[14] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
	[15] = {
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = 0,
		.divide = 3,
	},
};

struct aspeed_pwm_tachometer_data {
	struct regmap *regmap;
	unsigned long clk_freq;
	struct reset_control *reset;	
	bool pwm_present[16];
	bool fan_tach_present[16];
	struct aspeed_pwm_channel_params *pwm_channel;
	struct aspeed_techo_channel_params *techo_channel;
	struct aspeed_cooling_device *cdev[8];
	const struct attribute_group *groups[3];
};

struct aspeed_cooling_device {
	char name[16];
	struct aspeed_pwm_tachometer_data *priv;
	struct thermal_cooling_device *tcdev;
	int pwm_channel;
	u8 *cooling_levels;
	u8 max_state;
	u8 cur_state;
};

static int regmap_aspeed_pwm_tachometer_reg_write(void *context, unsigned int reg,
					     unsigned int val)
{
	void __iomem *regs = (void __iomem *)context;

	writel(val, regs + reg);
	return 0;
}

static int regmap_aspeed_pwm_tachometer_reg_read(void *context, unsigned int reg,
					    unsigned int *val)
{
	void __iomem *regs = (void __iomem *)context;

	*val = readl(regs + reg);
	return 0;
}

static const struct regmap_config aspeed_pwm_tachometer_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x100,
	.reg_write = regmap_aspeed_pwm_tachometer_reg_write,
	.reg_read = regmap_aspeed_pwm_tachometer_reg_read,
	.fast_io = true,
};

static void aspeed_set_pwm_channel_enable(struct regmap *regmap, u8 pwm_channel,
				       bool enable)
{
	printk("aspeed_set_pwm_channel_enable ch[%d], enable %d \n", pwm_channel, enable);

	regmap_update_bits(regmap, ASPEED_PWM_CTRL_CH(pwm_channel), (PWM_CLK_ENABLE | PWM_PIN_EN), enable ? (PWM_CLK_ENABLE | PWM_PIN_EN) : 0);
}

static void aspeed_set_fan_tach_ch_enable(struct aspeed_pwm_tachometer_data *priv, u8 fan_tach_ch,
					  bool enable)
{
	u32 reg_value = 0;

	printk("aspeed_set_fan_tach_ch_enable [%d]\n", enable);

	reg_value = TECHO_ENABLE | 
			(priv->techo_channel[fan_tach_ch].tacho_edge << TECHIO_EDGE_BIT) |
			(priv->techo_channel[fan_tach_ch].divide << TECHO_CLK_DIV_BIT);

	if(priv->techo_channel[fan_tach_ch].limited_inverse)
		reg_value |= TECHO_INVERS_LIMIT;
		
	if(priv->techo_channel[fan_tach_ch].threshold)
		reg_value |= (TECHO_IER | priv->techo_channel[fan_tach_ch].threshold); 

#if 1
	regmap_write(priv->regmap, ASPEED_TECHO_CTRL_CH(fan_tach_ch), reg_value);
#else
	regmap_update_bits(regmap, ASPEED_TECHO_CTRL_CH(fan_tach_ch),
			   TECHO_ENABLE,
			   enable ?
			   TECHO_ENABLE : 0);
#endif
}

static void aspeed_set_pwm_channel_fan_ctrl(struct aspeed_pwm_tachometer_data *priv,
					 u8 index, u8 fan_ctrl)
{
	u32 reg_value = 0;

	printk("aspeed_set_pwm_channel_fan_ctrl channel %d , fan_ctrl %d defaule period %d \n", index, fan_ctrl, priv->pwm_channel[index].period);

	if (fan_ctrl == 0) {
		aspeed_set_pwm_channel_enable(priv->regmap, index, false);
	} else {
		reg_value = (priv->pwm_channel[index].period << PWM_PERIOD_BIT) | 
					(0 << PWM_RISING_RISING_BIT) | (fan_ctrl << PWM_RISING_FALLING_BIT);

#if 1
		regmap_write(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index), reg_value);
#else
		regmap_update_bits(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index),
				   PWM_PERIOD_BIT_MASK | PWM_RISING_FALLING_MASK,
				   reg_value);
#endif
		aspeed_set_pwm_channel_enable(priv->regmap, index, true);
	}

}

#define BOTH_EDGES 0x02 /* 10b */

static int aspeed_get_fan_tach_ch_rpm(struct aspeed_pwm_tachometer_data *priv,
				      u8 fan_tach_ch)
{
	u32 raw_data, tach_div, clk_source, val;
	u8 mode, both;

	printk("aspeed_get_fan_tach_ch_rpm fan_tach_ch %d \n", fan_tach_ch);

	regmap_read(priv->regmap, ASPEED_TECHO_STS_CH(fan_tach_ch), &val);
	raw_data = val & TACHO_VALUE_MASK;

	tach_div = priv->techo_channel[fan_tach_ch].divide;
	/*
	 * We need the mode to determine if the raw_data is double (from
	 * counting both edges).
	 */
	mode = priv->techo_channel[fan_tach_ch].tacho_edge;
	both = (mode & BOTH_EDGES) ? 1 : 0;
	printk("clk %d, raw_data %x , tach_div %x  both %x \n", priv->clk_freq, raw_data, tach_div, both);

	tach_div = (0x1 << both) << (tach_div * 2);
	clk_source = priv->clk_freq;

	if (raw_data == 0)
		return 0;

	return (clk_source * 60) / (2 * raw_data * tach_div);

}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int ret;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);
	long fan_ctrl;

	ret = kstrtol(buf, 10, &fan_ctrl);
	if (ret != 0)
		return ret;

	if (fan_ctrl < 0 || fan_ctrl > 0x100)
		return -EINVAL;
	
	if (priv->pwm_channel[index].falling == fan_ctrl)
		return count;

	priv->pwm_channel[index].falling = fan_ctrl;
	aspeed_set_pwm_channel_fan_ctrl(priv, index, fan_ctrl);

	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", priv->pwm_channel[index].falling);
}

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int rpm;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	rpm = aspeed_get_fan_tach_ch_rpm(priv, index);
	if (rpm < 0)
		return rpm;

	return sprintf(buf, "%d\n", rpm);
}

static umode_t pwm_is_visible(struct kobject *kobj,
			      struct attribute *a, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);
printk("pwm_is_visible \n");
	if (!priv->pwm_present[index])
		return 0;
	return a->mode;
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
				  struct attribute *a, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);
	printk("fan_dev_is_visible \n");

	if (!priv->fan_tach_present[index])
		return 0;
	return a->mode;
}

static SENSOR_DEVICE_ATTR(pwm1, 0644,
			show_pwm, set_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm2, 0644,
			show_pwm, set_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm3, 0644,
			show_pwm, set_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm4, 0644,
			show_pwm, set_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm5, 0644,
			show_pwm, set_pwm, 4);
static SENSOR_DEVICE_ATTR(pwm6, 0644,
			show_pwm, set_pwm, 5);
static SENSOR_DEVICE_ATTR(pwm7, 0644,
			show_pwm, set_pwm, 6);
static SENSOR_DEVICE_ATTR(pwm8, 0644,
			show_pwm, set_pwm, 7);
static SENSOR_DEVICE_ATTR(pwm9, 0644,
			show_pwm, set_pwm, 8);
static SENSOR_DEVICE_ATTR(pwm10, 0644,
			show_pwm, set_pwm, 9);
static SENSOR_DEVICE_ATTR(pwm11, 0644,
			show_pwm, set_pwm, 10);
static SENSOR_DEVICE_ATTR(pwm12, 0644,
			show_pwm, set_pwm, 11);
static SENSOR_DEVICE_ATTR(pwm13, 0644,
			show_pwm, set_pwm, 12);
static SENSOR_DEVICE_ATTR(pwm14, 0644,
			show_pwm, set_pwm, 13);
static SENSOR_DEVICE_ATTR(pwm15, 0644,
			show_pwm, set_pwm, 14);
static SENSOR_DEVICE_ATTR(pwm16, 0644,
			show_pwm, set_pwm, 15);
static struct attribute *pwm_dev_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm4.dev_attr.attr,
	&sensor_dev_attr_pwm5.dev_attr.attr,
	&sensor_dev_attr_pwm6.dev_attr.attr,
	&sensor_dev_attr_pwm7.dev_attr.attr,
	&sensor_dev_attr_pwm8.dev_attr.attr,
	&sensor_dev_attr_pwm9.dev_attr.attr,
	&sensor_dev_attr_pwm10.dev_attr.attr,
	&sensor_dev_attr_pwm11.dev_attr.attr,
	&sensor_dev_attr_pwm12.dev_attr.attr,
	&sensor_dev_attr_pwm13.dev_attr.attr,
	&sensor_dev_attr_pwm14.dev_attr.attr,	
	&sensor_dev_attr_pwm15.dev_attr.attr,
	&sensor_dev_attr_pwm16.dev_attr.attr,	
	NULL,
};

static const struct attribute_group pwm_dev_group = {
	.attrs = pwm_dev_attrs,
	.is_visible = pwm_is_visible,
};

static SENSOR_DEVICE_ATTR(fan1_input, 0444,
		show_rpm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, 0444,
		show_rpm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_input, 0444,
		show_rpm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_input, 0444,
		show_rpm, NULL, 3);
static SENSOR_DEVICE_ATTR(fan5_input, 0444,
		show_rpm, NULL, 4);
static SENSOR_DEVICE_ATTR(fan6_input, 0444,
		show_rpm, NULL, 5);
static SENSOR_DEVICE_ATTR(fan7_input, 0444,
		show_rpm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan8_input, 0444,
		show_rpm, NULL, 7);
static SENSOR_DEVICE_ATTR(fan9_input, 0444,
		show_rpm, NULL, 8);
static SENSOR_DEVICE_ATTR(fan10_input, 0444,
		show_rpm, NULL, 9);
static SENSOR_DEVICE_ATTR(fan11_input, 0444,
		show_rpm, NULL, 10);
static SENSOR_DEVICE_ATTR(fan12_input, 0444,
		show_rpm, NULL, 11);
static SENSOR_DEVICE_ATTR(fan13_input, 0444,
		show_rpm, NULL, 12);
static SENSOR_DEVICE_ATTR(fan14_input, 0444,
		show_rpm, NULL, 13);
static SENSOR_DEVICE_ATTR(fan15_input, 0444,
		show_rpm, NULL, 14);
static SENSOR_DEVICE_ATTR(fan16_input, 0444,
		show_rpm, NULL, 15);
static struct attribute *fan_dev_attrs[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan5_input.dev_attr.attr,
	&sensor_dev_attr_fan6_input.dev_attr.attr,
	&sensor_dev_attr_fan7_input.dev_attr.attr,
	&sensor_dev_attr_fan8_input.dev_attr.attr,
	&sensor_dev_attr_fan9_input.dev_attr.attr,
	&sensor_dev_attr_fan10_input.dev_attr.attr,
	&sensor_dev_attr_fan11_input.dev_attr.attr,
	&sensor_dev_attr_fan12_input.dev_attr.attr,
	&sensor_dev_attr_fan13_input.dev_attr.attr,
	&sensor_dev_attr_fan14_input.dev_attr.attr,
	&sensor_dev_attr_fan15_input.dev_attr.attr,
	&sensor_dev_attr_fan16_input.dev_attr.attr,
	NULL
};

static const struct attribute_group fan_dev_group = {
	.attrs = fan_dev_attrs,
	.is_visible = fan_dev_is_visible,
};

static void aspeed_create_pwm_channel(struct aspeed_pwm_tachometer_data *priv,
				   u8 pwm_channel)
{
	priv->pwm_present[pwm_channel] = true;

	//use default 
	aspeed_set_pwm_channel_fan_ctrl(priv, pwm_channel, priv->pwm_channel[pwm_channel].falling );
}

static void aspeed_create_fan_tach_channel(struct aspeed_pwm_tachometer_data *priv,
					   u8 *fan_tach_ch,
					   int count)
{
	u8 val, index;
printk("aspeed_create_fan_tach_channel count %d \n", count);
	for (val = 0; val < count; val++) {
		index = fan_tach_ch[val];
		priv->fan_tach_present[index] = true;
		aspeed_set_fan_tach_ch_enable(priv, index, true);
		printk("aspeed_create_fan_tach_channel fan idx %d \n", index);
	}
}

static int
aspeed_pwm_cz_get_max_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;
	printk("aspeed_pwm_cz_get_max_state TODO　~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	*state = cdev->max_state;

	return 0;
}

static int
aspeed_pwm_cz_get_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;
	printk("aspeed_pwm_cz_get_cur_state TODO　~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	*state = cdev->cur_state;

	return 0;
}

static int
aspeed_pwm_cz_set_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	if (state > cdev->max_state)
		return -EINVAL;
printk("aspeed_pwm_cz_set_cur_state TODO　\n");
	cdev->cur_state = state;
	cdev->priv->pwm_channel[cdev->pwm_channel].falling =
					cdev->cooling_levels[cdev->cur_state];
	aspeed_set_pwm_channel_fan_ctrl(cdev->priv, cdev->pwm_channel,
				     cdev->cooling_levels[cdev->cur_state]);

	return 0;
}

static const struct thermal_cooling_device_ops aspeed_pwm_cool_ops = {
	.get_max_state = aspeed_pwm_cz_get_max_state,
	.get_cur_state = aspeed_pwm_cz_get_cur_state,
	.set_cur_state = aspeed_pwm_cz_set_cur_state,
};

static int aspeed_create_pwm_cooling(struct device *dev,
				     struct device_node *child,
				     struct aspeed_pwm_tachometer_data *priv,
				     u32 pwm_channel, u8 num_levels)
{
	int ret;
	struct aspeed_cooling_device *cdev;
printk("aspeed_create_pwm_cooling 0 \n");
	cdev = devm_kzalloc(dev, sizeof(*cdev), GFP_KERNEL);

	if (!cdev)
		return -ENOMEM;
	printk("aspeed_create_pwm_cooling 1 \n");

	cdev->cooling_levels = devm_kzalloc(dev, num_levels, GFP_KERNEL);
	if (!cdev->cooling_levels)
		return -ENOMEM;
	printk("aspeed_create_pwm_cooling 2 \n");

	cdev->max_state = num_levels - 1;
	ret = of_property_read_u8_array(child, "cooling-levels",
					cdev->cooling_levels,
					num_levels);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read.\n");
		return ret;
	}
	snprintf(cdev->name, MAX_CDEV_NAME_LEN, "%s%d", child->name, pwm_channel);
	printk("aspeed_create_pwm_cooling 3 \n");

	cdev->tcdev = thermal_of_cooling_device_register(child,
							 cdev->name,
							 cdev,
							 &aspeed_pwm_cool_ops);
	if (IS_ERR(cdev->tcdev))
		return PTR_ERR(cdev->tcdev);
	printk("aspeed_create_pwm_cooling 4 \n");

	cdev->priv = priv;
	cdev->pwm_channel = pwm_channel;

	priv->cdev[pwm_channel] = cdev;
	printk("aspeed_create_pwm_cooling 5 \n");

	return 0;
}

static int aspeed_create_fan(struct device *dev,
			     struct device_node *child,
			     struct aspeed_pwm_tachometer_data *priv)
{
	u8 *fan_tach_ch;
	u32 pwm_channel;
	int ret, count;

	ret = of_property_read_u32(child, "reg", &pwm_channel);
	if (ret)
		return ret;
	printk("reg is pwm_channel %d ************************************** \n", pwm_channel);
	
	aspeed_create_pwm_channel(priv, (u8)pwm_channel);

	ret = of_property_count_u8_elems(child, "cooling-levels");
	printk("aspeed_create_fan 2 \n");

	if (ret > 0) {
		ret = aspeed_create_pwm_cooling(dev, child, priv, pwm_channel,
						ret);
		if (ret)
			return ret;
	}
	printk("aspeed_create_fan 3 *******************************************\n");

	count = of_property_count_u8_elems(child, "aspeed,fan-tach-ch");
	if (count < 1)
		return -EINVAL;
	fan_tach_ch = devm_kzalloc(dev, sizeof(*fan_tach_ch) * count,
				   GFP_KERNEL);
	if (!fan_tach_ch)
		return -ENOMEM;
	ret = of_property_read_u8_array(child, "aspeed,fan-tach-ch",
					fan_tach_ch, count);
	if (ret)
		return ret;

	aspeed_create_fan_tach_channel(priv, fan_tach_ch, count);

	return 0;
}

static int aspeed_pwm_tachometer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np, *child;
	struct aspeed_pwm_tachometer_data *priv;
	void __iomem *regs;
	struct resource *res;
	struct device *hwmon;
	struct clk *clk;
	int ret;

	np = dev->of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
	priv->pwm_channel = default_pwm_params;	
	priv->techo_channel = default_techo_params;
	priv->regmap = devm_regmap_init(dev, NULL, (__force void *)regs,
			&aspeed_pwm_tachometer_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return -ENODEV;
	priv->clk_freq = clk_get_rate(clk);

	priv->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->reset)) {
		dev_err(&pdev->dev, "can't get aspeed_pwm_tacho reset\n");
		return PTR_ERR(priv->reset);
	}

	//scu init
	reset_control_assert(priv->reset);
	reset_control_deassert(priv->reset);

	for_each_child_of_node(np, child) {
		ret = aspeed_create_fan(dev, child, priv);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	priv->groups[0] = &pwm_dev_group;
	priv->groups[1] = &fan_dev_group;
	priv->groups[2] = NULL;
	hwmon = devm_hwmon_device_register_with_groups(dev,
						       "aspeed_pwm_tachometer",
						       priv, priv->groups);
printk("aspeed_pwm_tachometer_probe ====================================== end \n");	
	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id of_pwm_tachometer_match_table[] = {
	{ .compatible = "aspeed,ast2600-pwm-tachometer", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_tachometer_match_table);

static struct platform_driver aspeed_pwm_tachometer_driver = {
	.probe		= aspeed_pwm_tachometer_probe,
	.driver		= {
		.name	= "aspeed_pwm_tachometer",
		.of_match_table = of_pwm_tachometer_match_table,
	},
};

module_platform_driver(aspeed_pwm_tachometer_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM and Fan Tachometer device driver");
MODULE_LICENSE("GPL");
