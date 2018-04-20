/*
 *  linux/arch/arm/mach-aspeed/ast.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <asm/mach/arch.h>
#include <asm/system_misc.h>

/* Reset the system. It is called by machine_restart(). */
static void ast_restart(enum reboot_mode mode, const char *cmd)
{
	/* We'll take a jump through zero as a poor second */
	soft_restart(0);
}


static const char *const ast_dt_match[] __initconst = {
	"aspeed",
	NULL,
};

DT_MACHINE_START(ast_dt, "ASpeed BMC SoC")
	.dt_compat		= ast_dt_match,
	.restart			= ast_restart,
MACHINE_END
