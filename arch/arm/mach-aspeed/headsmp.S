/*
 * Copyright (C) ASPEED Technology Inc.
 * Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>

	.align 3
	.arm

ENTRY(ast2600_secondary_startup)
	bl	v7_invalidate_l1
/* TODO - do any other inits or cleanups that are required*/
	b	secondary_startup
ENDPROC(ast2600_secondary_startup)
