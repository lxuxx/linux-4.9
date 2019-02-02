/*
 * Copyright Aspeed Technology.
 * Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>


#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>


#include "core.h"
#define AST2600_BOOT_ADDR_REG_OFFSET 0x0
#define AST2600_BOOT_SIG_REG_OFFSET 0x4

unsigned char *secboot_base;
void __init ast2600_smp_prepare_cpus(unsigned int max_cpus)
{
	/*TODO- All activities required for SMP like enabling Snoop Control Unit
	 * any sysctl initializations etc to be done here*/

	//Here we may have to get the info of the boot_address_register and boot_signature_register
	//from the device tree and map the register to the virtual space
	//so that when ast2600_boot_secondary is called we can write the ast2600_secondary_startup
	//address and issue a SEV()
	struct device_node *secboot_node;
	int retval = 0;
	printk("%s \n", __func__);

	/* TODO- Any initializations for CPU1 that is required should go here*/
	secboot_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-smpmem");
	if (!secboot_node) {
		pr_err("secboot device node found!!\n");
		retval = -ENODEV;
		return;
	}

//	secboot_base = (unsigned char *) of_iomap(secboot_node, 0);
	secboot_base = (unsigned char *) ioremap(0x1e6e2180, 0x40);

	if (!secboot_base) {
		pr_err("could not map the secondary boot base!");
		retval = -ENOMEM;
		return;
	}
	__raw_writel(0xBADABABA, secboot_base + AST2600_BOOT_SIG_REG_OFFSET);
	printk("%s end \n", __func__);
}

static int ast2600_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int retval = 0;
	unsigned int timeout;
	printk("%s \n", __func__);

	/* TODO - Do the magic to get the secondary core up and running linuxi
	 * Wake up!! Lazy bones!!*/
	//Basically get the ast2600_secondary_startup addres
	//Write it in the boot_addr_register
	//Write the signature to the boot signature register
	//Wakeup the hovering processor

	printk("%s \n", __func__);
	__raw_writel(0, secboot_base + AST2600_BOOT_ADDR_REG_OFFSET);
	__raw_writel(__pa_symbol(secondary_startup), secboot_base + AST2600_BOOT_ADDR_REG_OFFSET);
	__raw_writel(0xABBAADDA, secboot_base + AST2600_BOOT_SIG_REG_OFFSET);
	printk("secondary_startup address %x\n", __pa_symbol(secondary_startup));
	printk("AST2600_BOOT_ADDR_REG_OFFSET %x\n", __raw_readl(secboot_base + AST2600_BOOT_ADDR_REG_OFFSET));
	printk("AST2600_BOOT_SIG_REG_OFFSET %x\n", __raw_readl(secboot_base + AST2600_BOOT_SIG_REG_OFFSET));
	wmb();

	/* barrier it to make sure everyone sees it */
	dsb_sev();
        /*
	 *          * Wait for the other CPU to boot, but timeout if it doesn't
	 *                   */

	iounmap(secboot_base);
	printk("%s end \n", __func__);	
out:
	return retval;

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
//For Now dont fore see anything here but you never know
#endif

static void ast2600_secondary_init(unsigned int cpu)
{
	printk("%s \n", __func__);
}


static const struct smp_operations ast2600_smp_ops __initconst = {
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= NULL, //For now
	.cpu_die		= NULL, //For now
#endif
	.smp_prepare_cpus	= ast2600_smp_prepare_cpus,
	.smp_secondary_init	= ast2600_secondary_init,
	.smp_boot_secondary	= ast2600_boot_secondary,
};

CPU_METHOD_OF_DECLARE(ast2600_smp, "aspeed,ast2600-smp", &ast2600_smp_ops);
