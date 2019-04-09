/*
 *  Aspeed I2C Interrupt Controller.
 *
 *  Copyright (C) 2012-2017 ASPEED Technology Inc.
 *  Copyright 2017 IBM Corporation
 *  Copyright 2017 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/reset.h>
#include <linux/delay.h>

#define ASPEED_I2CG_ISR				0x00
#define ASPEED_I2CG_SLAVE_ISR		0x04	/* ast2600 */
#define ASPEED_I2CG_OWNER			0x08
#define ASPEED_I2CG_CTRL			0x0C
#define ASPEED_I2CG_CLK_DIV_CTRL	0x10	/* ast2600 */

/* 0x0C : I2CG SRAM Buffer Enable  */
#define ASPEED_I2CG_SRAM_BUFFER_ENABLE		BIT(0)

/*ast2600 */
#define ASPEED_I2CG_SLAVE_PKT_NAK		BIT(4)
#define ASPEED_I2CG_M_S_SEPARATE_INTR	BIT(3)
#define ASPEED_I2CG_CTRL_NEW_REG		BIT(2)
#define ASPEED_I2CG_CTRL_NEW_CLK_DIV	BIT(1)

struct aspeed_i2c_ic {
	void __iomem		*base;
	int			parent_irq;
	u32			i2c_irq_mask;
	struct reset_control	*rst;
	struct irq_domain	*irq_domain;
	int			bus_num;
};

static const struct of_device_id aspeed_i2c_ic_of_match[] = {
	{ .compatible = "aspeed,ast2400-i2c-ic", .data = (void *) 14},
	{ .compatible = "aspeed,ast2500-i2c-ic", .data = (void *) 14},
	{ .compatible = "aspeed,ast2600-i2c-ic", .data = (void *) 16},
	{ .compatible = "aspeed,ast2600-i2c-global", .data = (void *) 0},	
	{},
};

/*
 * The aspeed chip provides a single hardware interrupt for all of the I2C
 * busses, so we use a dummy interrupt chip to translate this single interrupt
 * into multiple interrupts, each associated with a single I2C bus.
 */
static void aspeed_g6_i2c_ic_irq_handler(struct irq_desc *desc)
{
	struct aspeed_i2c_ic *i2c_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	if(readl(i2c_ic->base + ASPEED_I2CG_CTRL) & ASPEED_I2CG_M_S_SEPARATE_INTR) {
		status = readl(i2c_ic->base);
		status &= i2c_ic->i2c_irq_mask;
		status |= (readl(i2c_ic->base + ASPEED_I2CG_SLAVE_ISR) & i2c_ic->i2c_irq_mask) << 16;
	} else {
		status = readl(i2c_ic->base);
		status &= i2c_ic->i2c_irq_mask;
	}
	for_each_set_bit(bit, &status, i2c_ic->bus_num) {
		bus_irq = irq_find_mapping(i2c_ic->irq_domain, bit);
		generic_handle_irq(bus_irq);
	}
	chained_irq_exit(chip, desc);
}
 
static void aspeed_i2c_ic_irq_handler(struct irq_desc *desc)
{
	struct aspeed_i2c_ic *i2c_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	status = readl(i2c_ic->base);
	status &= i2c_ic->i2c_irq_mask;
	for_each_set_bit(bit, &status, i2c_ic->bus_num) {
		bus_irq = irq_find_mapping(i2c_ic->irq_domain, bit);
		generic_handle_irq(bus_irq);
	}
	chained_irq_exit(chip, desc);
}

static void noop(struct irq_data *data) { }

static unsigned int noop_ret(struct irq_data *data)
{
	return 0;
}

static struct irq_chip i2c_irq_chip = {
	.name			= "i2c-ic",
	.irq_startup	= noop_ret,
	.irq_shutdown	= noop,
	.irq_enable		= noop,
	.irq_disable	= noop,
	.irq_ack		= noop,
	.irq_mask		= noop,
	.irq_unmask		= noop,
	.flags			= IRQCHIP_SKIP_SET_WAKE,
};
/*
 * Set simple handler and mark IRQ as valid. Nothing interesting to do here
 * since we are using a dummy interrupt chip.
 */
static int aspeed_i2c_ic_map_irq_domain(struct irq_domain *domain,
					unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &i2c_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops aspeed_i2c_ic_irq_domain_ops = {
	.map = aspeed_i2c_ic_map_irq_domain,
};

static int aspeed_i2c_ic_probe(struct platform_device *pdev)
{
	struct aspeed_i2c_ic *i2c_ic;
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *match;
	u32 bus_owner;
	u32 new_mode;
	int ret = 0;

	match = of_match_node(aspeed_i2c_ic_of_match, node);
	if (!match)
		return -ENOMEM;

	i2c_ic = kzalloc(sizeof(*i2c_ic), GFP_KERNEL);
	if (!i2c_ic)
		return -ENOMEM;

	i2c_ic->base = of_iomap(node, 0);
	if (!i2c_ic->base) {
		ret = -ENOMEM;
		goto err_free_ic;
	}

	i2c_ic->bus_num = (int) match->data;

	if (i2c_ic->bus_num) {
		i2c_ic->parent_irq = irq_of_parse_and_map(node, 0);
		if (i2c_ic->parent_irq < 0) {
			ret = i2c_ic->parent_irq;
			goto err_iounmap;
		}
	} 

	i2c_ic->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);

	if (IS_ERR(i2c_ic->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller device tree entry");
		return PTR_ERR(i2c_ic->rst);
	}

	//SCU I2C Reset 
	reset_control_assert(i2c_ic->rst);
	udelay(3);
	reset_control_deassert(i2c_ic->rst);

	/* ast2600 init */
	if ((of_device_is_compatible(node, "aspeed,ast2600-i2c-global")) || 
		(of_device_is_compatible(node, "aspeed,ast2600-i2c-ic"))) {
		/* only support in ast-g6 platform */

		if(!of_property_read_u32(node, "new-mode", &new_mode)) {
			if(new_mode)
				writel(ASPEED_I2CG_SLAVE_PKT_NAK | ASPEED_I2CG_CTRL_NEW_REG, i2c_ic->base + ASPEED_I2CG_CTRL);
		}
		/* assign 4 base clock 
		 * base clk1 : 1M for 1KHz
		 * base clk2 : 4M for 400KHz	 
		 * base clk3 : 10M for 1MHz	 
		 * base clk4 : 35M for 3.4MHz	 
		*/
		//TODO ~~
		//get i2c clk source first 
		//writel(xx , i2c_ic->base + ASPEED_I2CG_CLK_DIV_CTRL);
	} else if(of_device_is_compatible(node, "aspeed,ast2500-i2c-ic")) {		
		writel(ASPEED_I2CG_SRAM_BUFFER_ENABLE, i2c_ic->base + ASPEED_I2CG_CTRL);
	} else {
		
	}
	
	if (!of_property_read_u32(node, "bus-owner", &bus_owner)) {
		writel(bus_owner, i2c_ic->base + ASPEED_I2CG_OWNER);
		i2c_ic->i2c_irq_mask = ~bus_owner;
	} else {
		writel(0, i2c_ic->base + ASPEED_I2CG_OWNER);
		i2c_ic->i2c_irq_mask = 0xffffffff;
	}

	if(i2c_ic->bus_num) {
		i2c_ic->irq_domain = irq_domain_add_linear(node,
				     i2c_ic->bus_num,
				     &aspeed_i2c_ic_irq_domain_ops,
				     i2c_ic);
		if (!i2c_ic->irq_domain) {
			ret = -ENOMEM;
			goto err_iounmap;
		}

		i2c_ic->irq_domain->name = "aspeed-i2c-domain";

		if(of_device_is_compatible(node, "aspeed,ast2600-i2c-ic")) {
			irq_set_chained_handler_and_data(i2c_ic->parent_irq,
							 aspeed_g6_i2c_ic_irq_handler, i2c_ic);
		} else {
			irq_set_chained_handler_and_data(i2c_ic->parent_irq,
							 aspeed_i2c_ic_irq_handler, i2c_ic);
		}
		pr_info("i2c controller registered, irq %d\n", i2c_ic->parent_irq);
	} else 
		pr_info("i2c global registered \n");

	return 0;

err_iounmap:
	iounmap(i2c_ic->base);
err_free_ic:
	kfree(i2c_ic);
	return ret;
}

static struct platform_driver aspeed_i2c_ic_driver = {
	.probe  = aspeed_i2c_ic_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_i2c_ic_of_match,
	},
};

static int __init aspeed_i2c_ic_init(void)
{
	return platform_driver_register(&aspeed_i2c_ic_driver);
}
postcore_initcall(aspeed_i2c_ic_init);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("ASPEED I2C INTC Driver");
MODULE_LICENSE("GPL v2");
