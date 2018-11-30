/*
 * Copyright 2018 Aspeed Technology.
 * Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/of_address.h>
#include <linux/of_platform.h>

static const char *const aspeed_g6_dt_compat[] __initconst = {
	"aspeed,ast2600",
	NULL,
};

DT_MACHINE_START(ASPEED, "Aspeed G6 ast2600")
	.dt_compat      = aspeed_g6_dt_compat,
	.l2c_aux_val    = 0,
	.l2c_aux_mask   = ~0,
	MACHINE_END

