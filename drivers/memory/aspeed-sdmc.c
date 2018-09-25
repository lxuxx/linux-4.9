/*
 * ASPEED SDRAM Controller driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/aspeed-sdmc.h>

#define ASPEED_SDMC_PROTECT				0x00		/*	protection key register	*/
#define ASPEED_SDMC_CONFIG				0x04		/*	Configuration register */
#define ASPEED_SDMC_MEM_REQ				0x08		/*	Graphics Memory Protection register */

#define ASPEED_SDMC_ISR					0x50		/*	Interrupt Control/Status Register */
#define ASPEED_SDMC_CACHE_ECC_RANGE                 0x54            /*      ECC/CACHE Address Range Control Register */

/*	ASPEED_SDMC_PROTECT: 0x00  - protection key register */
#define SDMC_PROTECT_UNLOCK			0xFC600309

/*	ASPEED_SDMC_CONFIG :0x04	 - Configuration register */
#define SDMC_CONFIG_VER_GET(x)		((x >> 28) & 0x3)
#define ASPEED_LEGACY_SDMC			0
#define ASPEED_G5_SDMC				1
#define ASPEED_G6_SDMC				3

#define SDMC_G5_CONFIG_CACHE_EN		BIT(10)	//only in ast2500 

#define SDMC_CONFIG_EEC_EN			BIT(7)
#define SDMC_G5_CONFIG_DDR4			BIT(4) //only in ast2500/ast2600

#define SDMC_CONFIG_VRAM_GET(x)		((x >> 2) & 0x3)
#define SDMC_CONFIG_MEM_GET(x)		(x & 0x3)


/*	#define ASPEED_SDMC_ISR	 : 0x50	- Interrupt Control/Status Register */
#define SDMC_ISR_CLR				BIT(31)
#define SDMC_ISR_RW_ACCESS			BIT(29)

#define SDMC_ISR_GET_ECC_RECOVER(x)	((x >> 16) & 0xff)
#define SDMC_ISR_GET_ECC_UNRECOVER(x)	((x >> 12) & 0xf)

//#define ASPEED_SDMC_LOCK
//#define ASPEED_SDMC_DEBUG

#ifdef ASPEED_SDMC_DEBUG
#define SDMCDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SDMCDBUG(fmt, args...)
#endif

#define SDMCMSG(fmt, args...) printk(fmt, ## args)

void __iomem	*ast_sdmc_base = 0;

struct aspeed_sdmc_device {
	void __iomem	*reg;
	struct kobject *ast_sdmc_kobj;
	int aspeed_version;
};

struct aspeed_sdmc_device *aspeed_sdmc;

extern u8
ast_sdmc_get_ecc(void)
{
	if(readl(aspeed_sdmc->reg + ASPEED_SDMC_CONFIG) & SDMC_CONFIG_EEC_EN)
		return 1;
	else
		return 0;
}

extern u8
ast_sdmc_get_cache(void)
{
	if(aspeed_sdmc->aspeed_version == 5) {
		if(readl(aspeed_sdmc->reg + ASPEED_SDMC_CONFIG) & SDMC_G5_CONFIG_CACHE_EN)
			return 1;
		else
			return 0;		
	} else
		return 0;	
}

extern u8
ast_sdmc_get_dram(void)
{
	if(aspeed_sdmc->aspeed_version == 4)
		return 0;

	if(readl(aspeed_sdmc->reg + ASPEED_SDMC_CONFIG) & SDMC_G5_CONFIG_DDR4)
		return 1;
	else
		return 0;
}

static const u32 aspeed_vram_table[] = {
	0x00800000,	//8MB
	0x01000000,	//16MB
	0x02000000,	//32MB
	0x04000000,	//64MB
};

extern u32
ast_sdmc_get_vram_size(void)
{
	u32 size_conf = SDMC_CONFIG_VRAM_GET(readl(aspeed_sdmc->reg + ASPEED_SDMC_CONFIG));
	return aspeed_vram_table[size_conf];
}

static const u32 ast2400_dram_table[] = {
	0x04000000,	//64MB
	0x08000000,	//128MB
	0x10000000, //256MB
	0x20000000,	//512MB
};

static const u32 ast2500_dram_table[] = {
	0x08000000,	//128MB
	0x10000000,	//256MB
	0x20000000,	//512MB
	0x40000000,	//1024MB
};

static const u32 ast2600_dram_table[] = {
	0x10000000,	//256MB
	0x20000000,	//512MB
	0x40000000,	//1024MB
	0x80000000,	//2048MB
};

extern u32
ast_sdmc_get_mem_size(void)
{
	u32 size = 0;
	u32 conf = readl(aspeed_sdmc->reg + ASPEED_SDMC_CONFIG);
	u32 size_conf = SDMC_CONFIG_MEM_GET(conf); 
	
	switch(SDMC_CONFIG_MEM_GET(conf)) {
		case ASPEED_LEGACY_SDMC:
			size = ast2400_dram_table[size_conf];
			break;
		case ASPEED_G5_SDMC:
			size = ast2500_dram_table[size_conf];
			break;
		case ASPEED_G6_SDMC:
			size = ast2600_dram_table[size_conf];
			break;
	}

	return size;
}

EXPORT_SYMBOL(ast_sdmc_get_mem_size);

extern u32
ast_sdmc_get_ecc_size(void)
{
	return readl(aspeed_sdmc->reg + ASPEED_SDMC_CACHE_ECC_RANGE);
}

extern void
ast_sdmc_disable_mem_protection(u8 req)
{
	writel(readl(aspeed_sdmc->reg + ASPEED_SDMC_MEM_REQ) & ~(1<< req), aspeed_sdmc->reg + ASPEED_SDMC_MEM_REQ);
}

static ssize_t show_ecc(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d: %s\n", ast_sdmc_get_ecc(), ast_sdmc_get_ecc()? "Enable":"Disable");
}

static DEVICE_ATTR(ecc, S_IRUGO | S_IWUSR, show_ecc, NULL); 

static ssize_t show_ecc_counter(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "recoverable count %d: un-recoverable count %d\n", 
			SDMC_ISR_GET_ECC_RECOVER(readl(aspeed_sdmc->reg + ASPEED_SDMC_ISR)), 
			SDMC_ISR_GET_ECC_UNRECOVER(readl(aspeed_sdmc->reg + ASPEED_SDMC_ISR)));
}

static ssize_t store_ecc_counter(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		writel(readl(aspeed_sdmc->reg + ASPEED_SDMC_ISR) | SDMC_ISR_CLR, aspeed_sdmc->reg + ASPEED_SDMC_ISR);
	
	return count;
}

static DEVICE_ATTR(ecc_counter, S_IRUGO | S_IWUSR, show_ecc_counter, store_ecc_counter); 

static ssize_t show_cache(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d: %s\n", ast_sdmc_get_cache(), ast_sdmc_get_cache()? "Enable":"Disable");
}

static DEVICE_ATTR(cache, S_IRUGO | S_IWUSR, show_cache, NULL); 


static ssize_t show_dram(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d: %s\n", ast_sdmc_get_dram(), ast_sdmc_get_dram()? "DDR4":"DDR3");
}

static DEVICE_ATTR(dram, S_IRUGO, show_dram, NULL); 

static struct attribute *ast_sdmc_attributes[] = {
	&dev_attr_ecc.attr,
	&dev_attr_ecc_counter.attr,	
	&dev_attr_cache.attr,
	&dev_attr_dram.attr,
	NULL
};

static const struct attribute_group sdmc_attribute_group = {
	.attrs = ast_sdmc_attributes
};

static const struct of_device_id aspeed_sdmc_of_match[] = {
	{ .compatible = "aspeed,ast2400-sdmc", .data = (void *)4, },
	{ .compatible = "aspeed,ast2500-sdmc", .data = (void *)5, },
	{ .compatible = "aspeed,ast2600-sdmc", .data = (void *)6, },
	{ }
};

static int aspeed_sdmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	const struct of_device_id *dev_id;
	int ret;

	SDMCDBUG("\n");

	aspeed_sdmc = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_sdmc_device), GFP_KERNEL);
	if (!aspeed_sdmc)
		return -ENOMEM;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	aspeed_sdmc->reg = devm_ioremap_resource(&pdev->dev, res);

	dev_id = of_match_node(aspeed_sdmc_of_match, pdev->dev.of_node);
	if (!dev_id)
		return -EINVAL;

	aspeed_sdmc->aspeed_version = (int)dev_id->data;

	//show in /sys/kernel/dram
	aspeed_sdmc->ast_sdmc_kobj = kobject_create_and_add("dram", kernel_kobj);
	if (!aspeed_sdmc->ast_sdmc_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(aspeed_sdmc->ast_sdmc_kobj, &sdmc_attribute_group);
	if (ret < 0)
		return ret;
	
	return ret;
}

static struct platform_driver aspeed_sdmc_driver = {
	.probe = aspeed_sdmc_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_sdmc_of_match,
	},
};

static int aspeed_sdmc_init(void)
{
	return platform_driver_register(&aspeed_sdmc_driver);
}

core_initcall(aspeed_sdmc_init);
