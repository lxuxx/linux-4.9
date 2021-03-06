/********************************************************************************
* File Name     : arch/arm/mach-aspeed/include/plat/ast1070-scu.h
* Author        : Ryan Chen
* Description   : AST1070 SCU Service Header
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*   History      :
*      1. 2013/05/03 Ryan Chen create this file
*
********************************************************************************/

#ifndef __AST1070_SCU_H_INCLUDED
#define __AST1070_SCU_H_INCLUDED

extern void ast1070_scu_init(u8 chip,u32 lpc_base);

extern u32 ast1070_revision_id_info(u8 chip);
extern void ast1070_scu_dma_init(u8 chip);

extern void ast1070_scu_init_i2c(u8 node);
extern void ast1070_scu_init_uart(u8 node);
extern void ast1070_multi_func_uart(u8 node, u8 uart);

#endif
