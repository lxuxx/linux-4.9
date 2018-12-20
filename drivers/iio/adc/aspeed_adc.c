/*
 * Aspeed AST2400/2500 ADC
 *
 * Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iopoll.h>

#define ASPEED_RESOLUTION_BITS		10
#define ASPEED_CLOCKS_PER_SAMPLE	12

#define ASPEED_REG_ENGINE_CONTROL		0x00
#define  ASPEED_ADC_CTRL_CH12_EN		BIT(28)

#define  ASPEED_ADC_CTRL_CH0_EN			BIT(16)

#define  ASPEED_ADC_CTRL_INIT_RDY		BIT(8)
//ASPEED G3
#define  ASPEED_G3_ADC_CTRL_COMPEN_CLR	BIT(6)
#define  ASPEED_G3_ADC_CTRL_COMPEN		BIT(5)

#define  ASPEED_ADC_CTRL_COMPENSATION	BIT(4)


#define  ASPEED_OPERATION_MODE_POWER_DOWN	(0x0 << 1)
#define  ASPEED_OPERATION_MODE_STANDBY		(0x1 << 1)
#define  ASPEED_OPERATION_MODE_NORMAL		(0x7 << 1)
#define  ASPEED_ENGINE_ENABLE			BIT(0)

#define ASPEED_REG_INTERRUPT_CONTROL	0x04
#define ASPEED_REG_VGA_DETECT_CONTROL	0x08
#define ASPEED_REG_CLOCK_CONTROL		0x0C
#define ASPEED_REG_MAX					0xC0

#define ASPEED_REG_COMPENSATION_TRIM	0xC4

#define ASPEED_ADC_INIT_POLLING_TIME	500
#define ASPEED_ADC_INIT_TIMEOUT		500000

struct aspeed_adc_model_data {
	const char *model_name;
	unsigned int min_sampling_rate;	// Hz
	unsigned int max_sampling_rate;	// Hz
	unsigned int vref_voltage;	// mV
	bool wait_init_sequence;
};

struct aspeed_adc_data {
	struct device		*dev;
	void __iomem		*base;
	spinlock_t		clk_lock;
	struct regmap			*scu;
	int	compen_value;		//Compensating value	
	struct clk_hw		*clk_prescaler;
	struct clk_hw		*clk_scaler;
	struct reset_control	*rst;
};

#define ASPEED_CHAN(_idx, _data_reg_addr) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_data_reg_addr),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec aspeed_adc_iio_channels[] = {
	ASPEED_CHAN(0, 0x10),
	ASPEED_CHAN(1, 0x12),
	ASPEED_CHAN(2, 0x14),
	ASPEED_CHAN(3, 0x16),
	ASPEED_CHAN(4, 0x18),
	ASPEED_CHAN(5, 0x1A),
	ASPEED_CHAN(6, 0x1C),
	ASPEED_CHAN(7, 0x1E),
	ASPEED_CHAN(8, 0x20),
	ASPEED_CHAN(9, 0x22),
	ASPEED_CHAN(10, 0x24),
	ASPEED_CHAN(11, 0x26),
	ASPEED_CHAN(12, 0x28),
	ASPEED_CHAN(13, 0x2A),
	ASPEED_CHAN(14, 0x2C),
	ASPEED_CHAN(15, 0x2E),
};

static int aspeed_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data =
			of_device_get_match_data(data->dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = readw(data->base + chan->address);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = model_data->vref_voltage;
		*val2 = ASPEED_RESOLUTION_BITS;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(data->clk_scaler->clk) /
				ASPEED_CLOCKS_PER_SAMPLE;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int aspeed_adc_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data =
			of_device_get_match_data(data->dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val < model_data->min_sampling_rate ||
			val > model_data->max_sampling_rate)
			return -EINVAL;

		clk_set_rate(data->clk_scaler->clk,
				val * ASPEED_CLOCKS_PER_SAMPLE);
		return 0;

	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_RAW:
		/*
		 * Technically, these could be written but the only reasons
		 * for doing so seem better handled in userspace.  EPERM is
		 * returned to signal this is a policy choice rather than a
		 * hardware limitation.
		 */
		return -EPERM;

	default:
		return -EINVAL;
	}
}

static int aspeed_adc_reg_access(struct iio_dev *indio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);

	if (!readval || reg % 4 || reg > ASPEED_REG_MAX)
		return -EINVAL;
printk("aspeed_adc_reg_access read offset %x \n", reg);
	*readval = readl(data->base + reg);

	return 0;
}

static const struct iio_info aspeed_adc_iio_info = {
	.read_raw = aspeed_adc_read_raw,
	.write_raw = aspeed_adc_write_raw,
	.debugfs_reg_access = aspeed_adc_reg_access,
};

static void aspeed_g5_adc_init(struct aspeed_adc_data *data)
{
	//Auto Compensating Sensing Mode : do not use in AST-G5
	u32 scu_otp1;
	u8 trim;

	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	

	//scu read trim : 0x154 : AST_SCU_OTP1	
	if(regmap_read(data->scu, 0x154, &scu_otp1)) {
		printk("read scu trim value fail \n");
		trim = 0x0;
	} else {
		trim = scu_otp1 >> 28;
	}

	if((trim == 0x0))
		trim = 0x8;

	//write trim 0xC4 [3:0]
	writel(trim, data->base + ASPEED_REG_COMPENSATION_TRIM);

	writel(0x40, data->base + ASPEED_REG_CLOCK_CONTROL);

	writel(ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE, data->base + ASPEED_REG_ENGINE_CONTROL);

	while(!(readl(data->base + ASPEED_REG_ENGINE_CONTROL) & ASPEED_ADC_CTRL_INIT_RDY));

#if 0
	writel(ASPEED_ADC_CTRL_AUTO_COMPEN  | ASPEED_OPERATION_MODE_NORMAL | 
							ASPEED_ENGINE_ENABLE, data->base + ASPEED_REG_ENGINE_CONTROL);

	while(readl(data->base + ASPEED_REG_ENGINE_CONTROL) & ASPEED_ADC_CTRL_AUTO_COMPEN);

	//compensating value = 0x200 - ADC10[9:0]
	data->compen_value = 0x200 - ((readl(data->base +  ASPEED_REG_COMPENSATION_TRIM) >> 16) & 0x3ff);
	dev_dbg(data->dev, "compensating value %d \n",data->compen_value);
#else
	writel(ASPEED_ADC_CTRL_CH0_EN | ASPEED_ADC_CTRL_COMPENSATION | 
							ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE, 
							data->base + ASPEED_REG_ENGINE_CONTROL);

	readl(data->base + ASPEED_REG_ENGINE_CONTROL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	data->compen_value = 0x200 - readw(data->base + 0x10);
	dev_dbg(data->dev, "compensating value %d \n",data->compen_value);

	writel(~(ASPEED_ADC_CTRL_COMPENSATION | ASPEED_ADC_CTRL_CH0_EN) & 
			readl(data->base +  ASPEED_REG_ENGINE_CONTROL), data->base + ASPEED_REG_ENGINE_CONTROL);	

#endif
		
}

static void aspeed_g4_adc_init(struct aspeed_adc_data *data)
{
	//Compensating Sensing Mode
	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	
	//For AST2400 A0 workaround  ... ADC0c = 1 ;
//	writel(1, data->base + ASPEED_REG_CLOCK_CONTROL);
//	writel((0x3e7<< 17) | 0x40, data->base + ASPEED_REG_CLOCK_CONTROL);
	writel(0x40, data->base + ASPEED_REG_CLOCK_CONTROL);

	writel(ASPEED_ADC_CTRL_CH0_EN | ASPEED_ADC_CTRL_COMPENSATION | 
							ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE, 
							data->base + ASPEED_REG_ENGINE_CONTROL);

	readl(data->base + ASPEED_REG_ENGINE_CONTROL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	data->compen_value = 0x200 - readw(data->base + 0x10);
	dev_dbg(data->dev, "compensating value %d \n",data->compen_value);

	writel(~ASPEED_ADC_CTRL_COMPENSATION & readl(data->base + ASPEED_REG_ENGINE_CONTROL), data->base + ASPEED_REG_ENGINE_CONTROL);	
}

static void aspeed_g3_adc_init(struct aspeed_adc_data *data)
{
	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	
	
	writel(0x3e7, data->base + ASPEED_REG_CLOCK_CONTROL); 

	writel(ASPEED_ADC_CTRL_CH12_EN | ASPEED_G3_ADC_CTRL_COMPEN_CLR | 
							ASPEED_G3_ADC_CTRL_COMPEN | ASPEED_OPERATION_MODE_NORMAL | 
							ASPEED_ENGINE_ENABLE, data->base + ASPEED_REG_ENGINE_CONTROL);

	mdelay(50);

	//compensating value = 0x200 - ADC10[9:0] : use channel12
	if(readl(data->base + 0x28) & (0x1 << 8))
		data->compen_value = 0x200 - readw(data->base + 0x28);
	else
		data->compen_value = 0 - readw(data->base + 0x28);

	dev_dbg(data->dev, "compensating value %d \n",data->compen_value);

	writel(~ASPEED_G3_ADC_CTRL_COMPEN & readl(data->base + ASPEED_REG_ENGINE_CONTROL), data->base + ASPEED_REG_ENGINE_CONTROL);	
}

static int aspeed_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct aspeed_adc_data *data;
	const struct aspeed_adc_model_data *model_data;
	struct resource *res;
	const char *clk_parent_name;
	int ret;
	u32 adc_engine_control_reg_val;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	/* Register ADC clock prescaler with source specified by device tree. */
	spin_lock_init(&data->clk_lock);
	clk_parent_name = of_clk_get_parent_name(pdev->dev.of_node, 0);

	data->clk_prescaler = clk_hw_register_divider(
				&pdev->dev, "prescaler", clk_parent_name, 0,
				data->base + ASPEED_REG_CLOCK_CONTROL,
				17, 15, 0, &data->clk_lock);
	if (IS_ERR(data->clk_prescaler))
		return PTR_ERR(data->clk_prescaler);

	/*
	 * Register ADC clock scaler downstream from the prescaler. Allow rate
	 * setting to adjust the prescaler as well.
	 */
	data->clk_scaler = clk_hw_register_divider(
				&pdev->dev, "scaler", "prescaler",
				CLK_SET_RATE_PARENT,
				data->base + ASPEED_REG_CLOCK_CONTROL,
				0, 10, 0, &data->clk_lock);
	if (IS_ERR(data->clk_scaler)) {
		ret = PTR_ERR(data->clk_scaler);
		goto scaler_error;
	}
	data->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(data->rst)) {
		dev_err(&pdev->dev,
			"invalid or missing reset controller device tree entry");
		ret = PTR_ERR(data->rst);
		goto reset_error;
	}
	reset_control_assert(data->rst);
	reset_control_deassert(data->rst);

	model_data = of_device_get_match_data(&pdev->dev);

	if (model_data->wait_init_sequence) {
		/* Enable engine in normal mode. */
		writel(ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE,
		       data->base + ASPEED_REG_ENGINE_CONTROL);

		/* Wait for initial sequence complete. */
		ret = readl_poll_timeout(data->base + ASPEED_REG_ENGINE_CONTROL,
					 adc_engine_control_reg_val,
					 adc_engine_control_reg_val &
					 ASPEED_ADC_CTRL_INIT_RDY,
					 ASPEED_ADC_INIT_POLLING_TIME,
					 ASPEED_ADC_INIT_TIMEOUT);
		if (ret)
			goto poll_timeout_error;
	}

	/* Start all channels in normal mode. */
	ret = clk_prepare_enable(data->clk_scaler->clk);
	if (ret)
		goto clk_enable_error;

	if(!strcmp(model_data->model_name, "ast2300-adc")) {
		aspeed_g3_adc_init(data);
	} else if(!strcmp(model_data->model_name, "ast2400-adc")) {
		aspeed_g4_adc_init(data);
	} else if(!strcmp(model_data->model_name, "ast2500-adc")) {
		data->scu = syscon_regmap_lookup_by_compatible("aspeed,ast2500-scu");
		if (IS_ERR(data->scu)) {
			dev_err(&pdev->dev, "failed to find SCU regmap\n");
			return PTR_ERR(data->scu);
		}
		aspeed_g5_adc_init(data);		
	} else 
		goto adc_init_error;

	adc_engine_control_reg_val = GENMASK(31, 16) |
		ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE;

	writel(adc_engine_control_reg_val,
		data->base + ASPEED_REG_ENGINE_CONTROL);

	model_data = of_device_get_match_data(&pdev->dev);
	indio_dev->name = model_data->model_name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &aspeed_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = aspeed_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(aspeed_adc_iio_channels);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto iio_register_error;

	return 0;

iio_register_error:
	writel(ASPEED_OPERATION_MODE_POWER_DOWN,
		data->base + ASPEED_REG_ENGINE_CONTROL);
	clk_disable_unprepare(data->clk_scaler->clk);
adc_init_error:	
clk_enable_error:
poll_timeout_error:
	reset_control_assert(data->rst);
reset_error:
	clk_hw_unregister_divider(data->clk_scaler);
scaler_error:
	clk_hw_unregister_divider(data->clk_prescaler);
	return ret;
}

static int aspeed_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct aspeed_adc_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	writel(ASPEED_OPERATION_MODE_POWER_DOWN,
		data->base + ASPEED_REG_ENGINE_CONTROL);
	clk_disable_unprepare(data->clk_scaler->clk);
	reset_control_assert(data->rst);
	clk_hw_unregister_divider(data->clk_scaler);
	clk_hw_unregister_divider(data->clk_prescaler);

	return 0;
}

///*******************************************************************
#if 0
static const struct aspeed_adc_config ast2300_config = { 
	.adc_version = 3, 
	.adc_ch_num = 12,
	.tmper_ch_num = 0, 
};

static const struct aspeed_adc_config ast2400_config = { 
	.adc_version = 4, 
	.adc_ch_num =16, 
	.tmper_ch_num = 0, 
};

static const struct aspeed_adc_config ast2500_config = { 
	.adc_version = 5,
	.adc_ch_num =16, 
	.tmper_ch_num = 2, 
};

static const struct of_device_id aspeed_adc_matches[] = {
	{ .compatible = "aspeed,ast2300-adc",	.data = &ast2300_config, },
	{ .compatible = "aspeed,ast2400-adc",	.data = &ast2400_config, },
	{ .compatible = "aspeed,ast2500-adc",	.data = &ast2500_config, },	
	{},
};
#endif
//*****************************************************************

static const struct aspeed_adc_model_data ast2400_model_data = {
	.model_name = "ast2400-adc",
	.vref_voltage = 2500, // mV
	.min_sampling_rate = 10000,
	.max_sampling_rate = 500000,
};

static const struct aspeed_adc_model_data ast2500_model_data = {
	.model_name = "ast2500-adc",
	.vref_voltage = 1800, // mV
	.min_sampling_rate = 1,
	.max_sampling_rate = 1000000,
	.wait_init_sequence = true,
};

static const struct of_device_id aspeed_adc_matches[] = {
	{ .compatible = "aspeed,ast2400-adc", .data = &ast2400_model_data },
	{ .compatible = "aspeed,ast2500-adc", .data = &ast2500_model_data },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_adc_matches);

static struct platform_driver aspeed_adc_driver = {
	.probe = aspeed_adc_probe,
	.remove = aspeed_adc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_adc_matches,
	}
};

module_platform_driver(aspeed_adc_driver);

MODULE_AUTHOR("Rick Altherr <raltherr@google.com>");
MODULE_DESCRIPTION("Aspeed AST2400/2500 ADC Driver");
MODULE_LICENSE("GPL");
