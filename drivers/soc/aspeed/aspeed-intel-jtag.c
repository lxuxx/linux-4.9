/*
 * ast-jtag.c - JTAG driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "aspeed-intel-jtag.h"
/*************************************************************************************/
//ast-g6 register add 
#define ASPEED_INTEL_JTAG_DATA0				0x20
#define ASPEED_INTEL_JTAG_DATA1				0x24
#define ASPEED_INTEL_JTAG_PADDING_CTRL0		0x28
#define ASPEED_INTEL_JTAG_PADDING_CTRL1		0x2C
#define ASPEED_INTEL_JTAG_SHIFT_CTRL		0x30
#define ASPEED_INTEL_JTAG_GBL_CTRL			0x34
#define ASPEED_INTEL_JTAG_ISR				0x38

/* 	ASPEED_INTEL_JTAG_PADDING_CTRL - 0x28 : Padding control */
#define JTAG_PADDING_DATA(x)		((x) << 24)
#define JTAG_POST_PADDING_NUM(x)	((x) << 12)
#define JTAG_PRE_PADDING_NUM(x)		(x)

/* 	ASPEED_INTEL_JTAG_SHIFT_CTRL - 0x30 : Shift control */
#define JTAG_TCK_FREE_RUN_EN		BIT(31)
#define JTAG_STATIC_SHIFT_EN		BIT(30)
#define JTAG_SHIFT_TMS(x)			((x) << 16)
#define JTAG_POST_TMS_SHIFT_NUM(x)	((x) << 13)
#define JTAG_PRE_TMS_SHIFT_NUM(x)	((x) << 10)
#define JTAG_PADDING_SELECT1		BIT(9)
#define JTAG_END_OF_SHIFT			BIT(8)
#define JTAG_START_OF_SHIFT			BIT(7)
#define JTAG_DATA_SHIFT_NUM(x)		(x)

/*	ASPEED_INTEL_JTAG_GBL_CTRL - 0x34 : Global control */
#define JTAG_ENG_MODE_EN			BIT(31)
#define JTAG_ENG_OUTPUT_EN			BIT(30)
#define JTAG_ENG_FORCE_RESET		BIT(29)

#define JTAG_FIFO_RESET				BIT(25)
#define JTAG_ENGINE_FIFO_MODE		BIT(24)


#define JTAG_STATIC_SHIFT_VAL		BIT(16)
#define JTAG_CLK_DIV(x)				(x)			/*TCK period = Period of HCLK * (JTAG14[10:0] + 1)*/
#define JTAG_CLK_DIVISOR_MASK		(0xfff)
#define JTAG_GET_CLK_DIVISOR(x)		(x & 0xfff)

/* ASPEED_INTEL_JTAG_ISR	- 0x38 : Interrupt Control */
#define JTAG_SHIFT_COMP_ISR_EN		BIT(16)
#define JTAG_SHIFT_COMP_ISR			BIT(0)
/* ASPEED_INTEL_JTAG_ISR	- 0x3C : Status */
#define JTAG_ENG_BUSY
/*************************************************************************************/
struct intel_jtag_xfer {
	u8 first_shift;
	u8 last_shift;
	u16 tms_value;
	u8 post_tms_cycle;
	u8 pre_tms_cycle;
	int padding_enable;
	u16 pre_padding;
	u16 post_padding;
	u8 padding_bit;
	u16 shiftbits;	
	u32 *shiftdata;
};

#define JTAGIOC_BASE       'J'

#define ASPEED_INTEL_JTAG_IOCRUNTEST	_IO(JTAGIOC_BASE, 0)
#define ASPEED_INTEL_JTAG_IOCXFER		_IOWR(JTAGIOC_BASE, 1, struct intel_jtag_xfer)
#define ASPEED_INTEL_JTAG_SIOCFREQ		_IOW(JTAGIOC_BASE, 2, unsigned int)
#define ASPEED_INTEL_JTAG_GIOCFREQ		_IOR(JTAGIOC_BASE, 3, unsigned int)
/******************************************************************************/
//#define ASPEED_INTEL_JTAG_DEBUG

#ifdef ASPEED_INTEL_JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

#define JTAG_MSG(fmt, args...) printk(fmt, ## args)

struct aspeed_intel_jtag_info {
	void __iomem	*reg_base;
	int 			irq;			//JTAG IRQ number
	struct reset_control *reset;
	struct clk 		*clk;
	u32				hclk;
	u32 			flag;
	struct completion	xfer_complete;
	bool 			is_open;
};
/*************************************************************************************/
static DEFINE_SPINLOCK(jtag_state_lock);

/******************************************************************************/
static inline u32
aspeed_intel_jtag_read(struct aspeed_intel_jtag_info *aspeed_intel_jtag, u32 reg)
{
#if 0
	u32 val;
	val = readl(aspeed_intel_jtag->reg_base + reg);
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(aspeed_intel_jtag->reg_base + reg);;
#endif
}

static inline void
aspeed_intel_jtag_write(struct aspeed_intel_jtag_info *aspeed_intel_jtag, u32 val, u32 reg)
{
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_intel_jtag->reg_base + reg);
}

/******************************************************************************/
static void aspeed_intel_jtag_set_freq(struct aspeed_intel_jtag_info *aspeed_intel_jtag, unsigned int freq)
{
	u16 i;
	for (i = 0; i < 0x7ff; i++) {
//		JTAG_DBUG("[%d] : freq : %d , target : %d \n", i, ast_get_pclk()/(i + 1), freq);
		if ((aspeed_intel_jtag->hclk / (i + 1)) <= freq)
			break;
	}
//	printk("div = %x \n", i);
	aspeed_intel_jtag_write(aspeed_intel_jtag, ((aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) & ~JTAG_CLK_DIVISOR_MASK) | i),  ASPEED_INTEL_JTAG_GBL_CTRL);

}

static unsigned int aspeed_intel_jtag_get_freq(struct aspeed_intel_jtag_info *aspeed_intel_jtag)
{
	return aspeed_intel_jtag->hclk / (JTAG_GET_CLK_DIVISOR(aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL)) + 1);
}
/******************************************************************************/
static int aspeed_intel_jtag_xfer(struct aspeed_intel_jtag_info *aspeed_intel_jtag, struct intel_jtag_xfer *xfer)
{
	int i, count = 0;
	u32 jtag_shift_config = 0;
	
	if(xfer->shiftbits > 512)
		return 1;

	init_completion(&aspeed_intel_jtag->xfer_complete);
	//reset fifo
	aspeed_intel_jtag_write(aspeed_intel_jtag, aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) | 
						JTAG_FIFO_RESET, ASPEED_INTEL_JTAG_GBL_CTRL);
	//wait for rest 
	while(aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) & JTAG_FIFO_RESET);

	//switch fifo to cpu mode
	aspeed_intel_jtag_write(aspeed_intel_jtag, aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) & 
						~JTAG_ENGINE_FIFO_MODE, ASPEED_INTEL_JTAG_GBL_CTRL);

	//shift data to fifo
	count = xfer->shiftbits / 32;
	if(xfer->shiftbits % 32)
		count++;
	
	for(i = 0; i < count; i++) {
		if(i % 2)
			aspeed_intel_jtag_write(aspeed_intel_jtag, xfer->shiftdata[i], ASPEED_INTEL_JTAG_DATA0);
		else
			aspeed_intel_jtag_write(aspeed_intel_jtag, xfer->shiftdata[i], ASPEED_INTEL_JTAG_DATA1);
	}

	//switch fifo to engine mode
	aspeed_intel_jtag_write(aspeed_intel_jtag, aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) |
						JTAG_ENGINE_FIFO_MODE, ASPEED_INTEL_JTAG_GBL_CTRL);

	//padding setting
	if(xfer->padding_enable) {
		aspeed_intel_jtag_write(aspeed_intel_jtag, 
						JTAG_PADDING_DATA(xfer->padding_bit) |
						JTAG_PRE_PADDING_NUM(xfer->pre_padding) |
						JTAG_POST_PADDING_NUM(xfer->post_padding), 
						ASPEED_INTEL_JTAG_PADDING_CTRL1);
	}

	//shift trigger
	jtag_shift_config = JTAG_TCK_FREE_RUN_EN | 
						JTAG_STATIC_SHIFT_EN |
						JTAG_SHIFT_TMS(xfer->tms_value) |
						JTAG_PRE_TMS_SHIFT_NUM(xfer->pre_tms_cycle) |
						JTAG_POST_TMS_SHIFT_NUM(xfer->post_tms_cycle) |
						JTAG_PADDING_SELECT1;

	if(xfer->first_shift)
		jtag_shift_config |=JTAG_START_OF_SHIFT;

	if(xfer->last_shift)
		jtag_shift_config |=JTAG_END_OF_SHIFT;

	if(xfer->shiftbits)
		jtag_shift_config |=JTAG_DATA_SHIFT_NUM(xfer->shiftbits);
						
	aspeed_intel_jtag_write(aspeed_intel_jtag, jtag_shift_config, ASPEED_INTEL_JTAG_SHIFT_CTRL);
	
	wait_for_completion(&aspeed_intel_jtag->xfer_complete);
	
	return 0;
}

static irqreturn_t aspeed_intel_jtag_interrupt(int this_irq, void *dev_id)
{
	u32 status;
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = dev_id;

	status = aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_ISR);
	JTAG_DBUG("sts %x \n", status);

	if (status & JTAG_SHIFT_COMP_ISR) {
		aspeed_intel_jtag_write(aspeed_intel_jtag, JTAG_SHIFT_COMP_ISR_EN | JTAG_SHIFT_COMP_ISR, ASPEED_INTEL_JTAG_ISR);
	}

	complete(&aspeed_intel_jtag->xfer_complete);
	return IRQ_HANDLED;
}

static long aspeed_intel_jtag_ioctl(struct file *file, unsigned int cmd,
					   unsigned long arg)
{
	int ret = 0;
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
		case ASPEED_INTEL_JTAG_IOCRUNTEST:
			aspeed_intel_jtag_write(aspeed_intel_jtag, 
						aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) |
						JTAG_ENG_FORCE_RESET, ASPEED_INTEL_JTAG_GBL_CTRL);
			while(aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL) & JTAG_ENG_FORCE_RESET);
			break;
		case ASPEED_INTEL_JTAG_GIOCFREQ:
			ret = __put_user(aspeed_intel_jtag_get_freq(aspeed_intel_jtag), (unsigned int __user *)arg);
			break;
		case ASPEED_INTEL_JTAG_SIOCFREQ:
			if ((unsigned int)arg > aspeed_intel_jtag->hclk)
				ret = -EFAULT;
			else
				aspeed_intel_jtag_set_freq(aspeed_intel_jtag, (unsigned int)arg);

			break;
		case ASPEED_INTEL_JTAG_IOCXFER:
			if (aspeed_intel_jtag_xfer(aspeed_intel_jtag, argp))
				ret = -EFAULT;
			break;
		default:
			return -ENOTTY;
			break;
	}

	return ret;
}

static int aspeed_intel_jtag_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = dev_get_drvdata(c->this_device);

	spin_lock(&jtag_state_lock);

//	drvdata = container_of(inode->i_cdev, struct aspeed_intel_jtag_info, cdev);

	if (aspeed_intel_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	aspeed_intel_jtag->is_open = true;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static int aspeed_intel_jtag_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = dev_get_drvdata(c->this_device);

	spin_lock(&jtag_state_lock);

	aspeed_intel_jtag->is_open = false;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t freq_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = dev_get_drvdata(dev);
//	printk("PCLK = %d \n", ast_get_pclk());
//	printk("DIV  = %d \n", JTAG_GET_CLK_DIVISOR(aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_TCK)) + 1);
	return sprintf(buf, "Frequency : %d\n", aspeed_intel_jtag->hclk / (JTAG_GET_CLK_DIVISOR(aspeed_intel_jtag_read(aspeed_intel_jtag, ASPEED_INTEL_JTAG_GBL_CTRL)) + 1));
}

static ssize_t freq_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 20);
	aspeed_intel_jtag_set_freq(aspeed_intel_jtag, val);

	return count;
}

static DEVICE_ATTR_RW(freq);

static struct attribute *jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	NULL
};

static struct attribute_group jtag_attribute_group = {
	.attrs = jtag_sysfs_entries,
};

static const struct file_operations aspeed_intel_jtag_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= aspeed_intel_jtag_ioctl,
	.open			= aspeed_intel_jtag_open,
	.release		= aspeed_intel_jtag_release,
};

static struct miscdevice aspeed_intel_jtag_misc = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "aspeed-intel-jtag",
	.fops 	= &aspeed_intel_jtag_fops,
};

/************************************************************************************************************/
static int aspeed_intel_jtag_probe(struct platform_device *pdev)
{
	struct aspeed_intel_jtag_info *aspeed_intel_jtag;
	struct resource *res;
	int ret = 0;

	JTAG_DBUG("aspeed_intel_jtag_probe\n");

	if (!(aspeed_intel_jtag = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_intel_jtag_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_intel_jtag->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_intel_jtag->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	aspeed_intel_jtag->irq = platform_get_irq(pdev, 0);
	if (aspeed_intel_jtag->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_intel_jtag->reset = devm_reset_control_get_exclusive(&pdev->dev, "jtag");
	if (IS_ERR(aspeed_intel_jtag->reset)) {
		dev_err(&pdev->dev, "can't get jtag reset\n");
		return PTR_ERR(aspeed_intel_jtag->reset);
	}

	aspeed_intel_jtag->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_intel_jtag->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	aspeed_intel_jtag->hclk = clk_get_rate(aspeed_intel_jtag->clk);

	//scu init
	reset_control_assert(aspeed_intel_jtag->reset);
	udelay(3);
	reset_control_deassert(aspeed_intel_jtag->reset);

	aspeed_intel_jtag_write(aspeed_intel_jtag, JTAG_ENG_MODE_EN | JTAG_ENG_OUTPUT_EN | JTAG_CLK_DIV(7), ASPEED_INTEL_JTAG_GBL_CTRL);

	ret = devm_request_irq(&pdev->dev, aspeed_intel_jtag->irq, aspeed_intel_jtag_interrupt,
						   0, dev_name(&pdev->dev), aspeed_intel_jtag);
	if (ret) {
		printk("JTAG Unable to get IRQ");
		goto out_region;
	}

	aspeed_intel_jtag_write(aspeed_intel_jtag, JTAG_SHIFT_COMP_ISR_EN, ASPEED_INTEL_JTAG_ISR);

	init_completion(&aspeed_intel_jtag->xfer_complete);

	ret = misc_register(&aspeed_intel_jtag_misc);
	if (ret) {
		printk(KERN_ERR "JTAG : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, aspeed_intel_jtag);
	dev_set_drvdata(aspeed_intel_jtag_misc.this_device, aspeed_intel_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
	if (ret) {
		printk(KERN_ERR "aspeed_intel_jtag: failed to create sysfs device attributes.\n");
		return -1;
	}

	printk(KERN_INFO "aspeed_intel_jtag: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(aspeed_intel_jtag->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "aspeed_intel_jtag: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_intel_jtag_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_intel_jtag_info *aspeed_intel_jtag = platform_get_drvdata(pdev);

	JTAG_DBUG("aspeed_intel_jtag_remove\n");

	sysfs_remove_group(&pdev->dev.kobj, &jtag_attribute_group);

	misc_deregister(&aspeed_intel_jtag_misc);

	free_irq(aspeed_intel_jtag->irq, aspeed_intel_jtag);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(aspeed_intel_jtag->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_intel_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_intel_jtag_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static const struct of_device_id aspeed_intel_jtag_of_matches[] = {
	{ .compatible = "aspeed,ast-jtag", },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_intel_jtag_of_matches);

static struct platform_driver aspeed_intel_jtag_driver = {
	.probe 		= aspeed_intel_jtag_probe,
	.remove 		= aspeed_intel_jtag_remove,
#ifdef CONFIG_PM
	.suspend        = aspeed_intel_jtag_suspend,
	.resume         = aspeed_intel_jtag_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_intel_jtag_of_matches,
	},
};

module_platform_driver(aspeed_intel_jtag_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
