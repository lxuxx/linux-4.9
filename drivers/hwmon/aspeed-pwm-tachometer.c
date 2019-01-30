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
#include <linux/regmap.h>
#include <linux/thermal.h>

#define ASPEED_PWM_CTRL			0x00	//PWM0 General Register
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
#define  PWM_LOOP_BIT_MASK				(0xf << 24)	//loop bit [7:0]
#define  PWM_PERIOD_BIT_MASK			(0xff << 24)	//pwm period bit [7:0]
#define  PWM_RISING_FALLING_AS_WDT_MASK (0xff << 16)	//pwm rising/falling point bit [7:0] as WDT
#define  PWM_RISING_FALLING_MASK		(0xf << 8)	//pwm falling point bit [7:0]
#define  PWM_RISING_RISING_MASK			(0xf)	//pwm rising point bit [7:0]

#define ASPEED_TECHO_CTRL		0x08	//TACH0 General Register
#define  TECHO_IER						BIT(31)	//enable tacho interrupt
#define  TECHO_INVERS_LIMIT				BIT(30) //inverse tacho limit comparison
#define  TECHO_LOOPBACK					BIT(29) //tacho loopback
#define  TECHO_ENABLE					BIT(28)	//{enable tacho}
#define  TECHO_DEBOUNCE_MASK			(0x3 << 26) //{tacho de-bounce}
#define  TECHIO_EDGE_MASK				(0x3 << 24) //tacho edge}
#define  TECHO_CLK_DIV_T_MASK			(0xf << 20) 
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
#define  TECHO_ISR			BIT(31)	//interrupt status and clear
#define  PWM_OUT			BIT(25)	//{pwm_out}
#define  PWM_OEN			BIT(24)	//{pwm_oeN}
#define  TACHO_DEB_INPUT	BIT(23)	//tacho deB input
#define  TACHO_RAW_INPUT	BIT(22) //tacho raw input}
#define  TACHO_VALUE_UPDATE	BIT(21)	//tacho value updated since the last read
#define  TACHO_FULL_MEASUREMENT	BIT(20) //{tacho full measurement}
#define  TACHO_VALUE_MASK	0xfffff	//tacho value bit [19:0]}

#define MAX_CDEV_NAME_LEN 16

struct aspeed_cooling_device {
	char name[16];
	struct aspeed_pwm_tachometer_data *priv;
	struct thermal_cooling_device *tcdev;
	int pwm_port;
	u8 *cooling_levels;
	u8 max_state;
	u8 cur_state;
};

struct aspeed_pwm_tachometer_data {
	struct regmap *regmap;
	unsigned long clk_freq;
	bool pwm_present[8];
	bool fan_tach_present[16];
	u8 type_pwm_clock_unit[3];
	u8 type_pwm_clock_division_h[3];
	u8 type_pwm_clock_division_l[3];
	u8 type_fan_tach_clock_division[3];
	u8 type_fan_tach_mode[3];
	u16 type_fan_tach_unit[3];
	u8 pwm_port_type[8];
	u8 pwm_port_fan_ctrl[8];
	u8 fan_tach_ch_source[16];
	struct aspeed_cooling_device *cdev[8];
	const struct attribute_group *groups[3];
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

static void aspeed_set_pwm_clock_values(struct regmap *regmap, u8 type,
					u8 div_high, u8 div_low, u8 unit)
{
#if 0
	u32 reg_value = ((div_high << type_params[type].h_value) |
			 (div_low << type_params[type].l_value) |
			 (unit << type_params[type].unit_value));

	regmap_update_bits(regmap, type_params[type].clk_ctrl_reg,
			   type_params[type].clk_ctrl_mask, reg_value);
#endif	
}

static void aspeed_set_pwm_port_enable(struct regmap *regmap, u8 pwm_port,
				       bool enable)
{
#if 0
	regmap_update_bits(regmap, pwm_port_params[pwm_port].ctrl_reg,
			   pwm_port_params[pwm_port].pwm_en,
			   enable ? pwm_port_params[pwm_port].pwm_en : 0);
#endif
}

static void aspeed_set_pwm_port_duty_rising_falling(struct regmap *regmap,
						    u8 pwm_port, u8 rising,
						    u8 falling)
{
#if 0
	u32 reg_value = (rising <<
			 pwm_port_params[pwm_port].duty_ctrl_rise_point);
	reg_value |= (falling <<
		      pwm_port_params[pwm_port].duty_ctrl_fall_point);

	regmap_update_bits(regmap, pwm_port_params[pwm_port].duty_ctrl_reg,
			   pwm_port_params[pwm_port].duty_ctrl_rise_fall_mask,
			   reg_value);
#endif	
}

static void aspeed_set_tacho_type_enable(struct regmap *regmap, u8 type,
					 bool enable)
{
#if 0
	regmap_update_bits(regmap, type_params[type].ctrl_reg,
			   TYPE_CTRL_FAN_TYPE_EN,
			   enable ? TYPE_CTRL_FAN_TYPE_EN : 0);
#endif
}

static void aspeed_set_tacho_type_values(struct regmap *regmap, u8 type,
					 u8 mode, u16 unit, u8 division)
{
#if 0
	u32 reg_value = ((mode << TYPE_CTRL_FAN_MODE) |
			 (unit << TYPE_CTRL_FAN_PERIOD) |
			 (division << TYPE_CTRL_FAN_DIVISION));

	regmap_update_bits(regmap, type_params[type].ctrl_reg,
			   TYPE_CTRL_FAN_MASK, reg_value);
	regmap_update_bits(regmap, type_params[type].ctrl_reg1,
			   TYPE_CTRL_FAN1_MASK, unit << 16);
#endif	
}

static void aspeed_set_fan_tach_ch_enable(struct regmap *regmap, u8 fan_tach_ch,
					  bool enable)
{
#if 0
	regmap_update_bits(regmap, ASPEED_PTCR_CTRL,
			   ASPEED_PTCR_CTRL_FAN_NUM_EN(fan_tach_ch),
			   enable ?
			   ASPEED_PTCR_CTRL_FAN_NUM_EN(fan_tach_ch) : 0);
#endif
}

static void aspeed_set_pwm_port_fan_ctrl(struct aspeed_pwm_tachometer_data *priv,
					 u8 index, u8 fan_ctrl)
{
#if 0
	u16 period, dc_time_on;

	period = priv->type_pwm_clock_unit[priv->pwm_port_type[index]];
	period += 1;
	dc_time_on = (fan_ctrl * period) / PWM_MAX;

	if (dc_time_on == 0) {
		aspeed_set_pwm_port_enable(priv->regmap, index, false);
	} else {
		if (dc_time_on == period)
			dc_time_on = 0;

		aspeed_set_pwm_port_duty_rising_falling(priv->regmap, index, 0,
							dc_time_on);
		aspeed_set_pwm_port_enable(priv->regmap, index, true);
	}
#endif	
}

static u32 aspeed_get_fan_tach_ch_measure_period(struct aspeed_pwm_tachometer_data
						 *priv, u8 type)
{
#if 0
	u32 clk;
	u16 tacho_unit;
	u8 clk_unit, div_h, div_l, tacho_div;

	clk = priv->clk_freq;
	clk_unit = priv->type_pwm_clock_unit[type];
	div_h = priv->type_pwm_clock_division_h[type];
	div_h = 0x1 << div_h;
	div_l = priv->type_pwm_clock_division_l[type];
	if (div_l == 0)
		div_l = 1;
	else
		div_l = div_l * 2;

	tacho_unit = priv->type_fan_tach_unit[type];
	tacho_div = priv->type_fan_tach_clock_division[type];

	tacho_div = 0x4 << (tacho_div * 2);
	return clk / (clk_unit * div_h * div_l * tacho_div * tacho_unit);
#else
	return 0;
#endif
}

static int aspeed_get_fan_tach_ch_rpm(struct aspeed_pwm_tachometer_data *priv,
				      u8 fan_tach_ch)
{
#if 0
	u32 raw_data, tach_div, clk_source, msec, usec, val;
	u8 fan_tach_ch_source, type, mode, both;
	int ret;

	regmap_write(priv->regmap, ASPEED_PTCR_TRIGGER, 0);
	regmap_write(priv->regmap, ASPEED_PTCR_TRIGGER, 0x1 << fan_tach_ch);

	fan_tach_ch_source = priv->fan_tach_ch_source[fan_tach_ch];
	type = priv->pwm_port_type[fan_tach_ch_source];

	msec = (1000 / aspeed_get_fan_tach_ch_measure_period(priv, type));
	usec = msec * 1000;

	ret = regmap_read_poll_timeout(
		priv->regmap,
		ASPEED_PTCR_RESULT,
		val,
		(val & RESULT_STATUS_MASK),
		ASPEED_RPM_STATUS_SLEEP_USEC,
		usec);

	/* return -ETIMEDOUT if we didn't get an answer. */
	if (ret)
		return ret;

	raw_data = val & RESULT_VALUE_MASK;
	tach_div = priv->type_fan_tach_clock_division[type];
	/*
	 * We need the mode to determine if the raw_data is double (from
	 * counting both edges).
	 */
	mode = priv->type_fan_tach_mode[type];
	both = (mode & BOTH_EDGES) ? 1 : 0;

	tach_div = (0x4 << both) << (tach_div * 2);
	clk_source = priv->clk_freq;

	if (raw_data == 0)
		return 0;

	return (clk_source * 60) / (2 * raw_data * tach_div);
#else
	return 0;
#endif
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
#if 0
	if (priv->pwm_port_fan_ctrl[index] == fan_ctrl)
		return count;

	priv->pwm_port_fan_ctrl[index] = fan_ctrl;
	aspeed_set_pwm_port_fan_ctrl(priv, index, fan_ctrl);
#endif
	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", priv->pwm_port_fan_ctrl[index]);
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

	if (!priv->pwm_present[index])
		return 0;
	return a->mode;
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
				  struct attribute *a, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

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

static void aspeed_create_pwm_port(struct aspeed_pwm_tachometer_data *priv,
				   u8 pwm_port)
{
	aspeed_set_pwm_port_enable(priv->regmap, pwm_port, true);
	priv->pwm_present[pwm_port] = true;
#if 0
	priv->pwm_port_type[pwm_port] = TYPEM;

	priv->pwm_port_fan_ctrl[pwm_port] = INIT_FAN_CTRL;
	aspeed_set_pwm_port_fan_ctrl(priv, pwm_port, INIT_FAN_CTRL);
#endif	
}

static void aspeed_create_fan_tach_channel(struct aspeed_pwm_tachometer_data *priv,
					   u8 *fan_tach_ch,
					   int count,
					   u8 pwm_source)
{
	u8 val, index;

	for (val = 0; val < count; val++) {
		index = fan_tach_ch[val];
		aspeed_set_fan_tach_ch_enable(priv->regmap, index, true);
		priv->fan_tach_present[index] = true;
		priv->fan_tach_ch_source[index] = pwm_source;
	}
}

static int
aspeed_pwm_cz_get_max_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	*state = cdev->max_state;

	return 0;
}

static int
aspeed_pwm_cz_get_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

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

	cdev->cur_state = state;
	cdev->priv->pwm_port_fan_ctrl[cdev->pwm_port] =
					cdev->cooling_levels[cdev->cur_state];
	aspeed_set_pwm_port_fan_ctrl(cdev->priv, cdev->pwm_port,
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
				     u32 pwm_port, u8 num_levels)
{
	int ret;
	struct aspeed_cooling_device *cdev;

	cdev = devm_kzalloc(dev, sizeof(*cdev), GFP_KERNEL);

	if (!cdev)
		return -ENOMEM;

	cdev->cooling_levels = devm_kzalloc(dev, num_levels, GFP_KERNEL);
	if (!cdev->cooling_levels)
		return -ENOMEM;

	cdev->max_state = num_levels - 1;
	ret = of_property_read_u8_array(child, "cooling-levels",
					cdev->cooling_levels,
					num_levels);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read.\n");
		return ret;
	}
	snprintf(cdev->name, MAX_CDEV_NAME_LEN, "%s%d", child->name, pwm_port);

	cdev->tcdev = thermal_of_cooling_device_register(child,
							 cdev->name,
							 cdev,
							 &aspeed_pwm_cool_ops);
	if (IS_ERR(cdev->tcdev))
		return PTR_ERR(cdev->tcdev);

	cdev->priv = priv;
	cdev->pwm_port = pwm_port;

	priv->cdev[pwm_port] = cdev;

	return 0;
}

static int aspeed_create_fan(struct device *dev,
			     struct device_node *child,
			     struct aspeed_pwm_tachometer_data *priv)
{
	u8 *fan_tach_ch;
	u32 pwm_port;
	int ret, count;

	ret = of_property_read_u32(child, "reg", &pwm_port);
	if (ret)
		return ret;
	aspeed_create_pwm_port(priv, (u8)pwm_port);

	ret = of_property_count_u8_elems(child, "cooling-levels");

	if (ret > 0) {
		ret = aspeed_create_pwm_cooling(dev, child, priv, pwm_port,
						ret);
		if (ret)
			return ret;
	}

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
	aspeed_create_fan_tach_channel(priv, fan_tach_ch, count, pwm_port);

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
	priv->regmap = devm_regmap_init(dev, NULL, (__force void *)regs,
			&aspeed_pwm_tachometer_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return -ENODEV;
	priv->clk_freq = clk_get_rate(clk);

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
						       "aspeed_pwm_tacho",
						       priv, priv->groups);
	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id of_pwm_tachometer_match_table[] = {
	{ .compatible = "aspeed,ast2600-pwm-tacho", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_tachometer_match_table);

static struct platform_driver aspeed_pwm_tachometer_driver = {
	.probe		= aspeed_pwm_tachometer_probe,
	.driver		= {
		.name	= "aspeed_pwm_tacho",
		.of_match_table = of_pwm_tachometer_match_table,
	},
};

module_platform_driver(aspeed_pwm_tachometer_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM and Fan Tachometer device driver");
MODULE_LICENSE("GPL");
