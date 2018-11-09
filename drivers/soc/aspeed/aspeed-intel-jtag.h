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
/* Function Prototypes */
enum aspeed_jtag_state {
	ILLEGAL_JTAG_STATE = -1,
	RESET = 0,
	IDLE = 1,
	DRSELECT = 2,
	DRCAPTURE = 3,
	DRSHIFT = 4,
	DREXIT1 = 5,
	DRPAUSE = 6,
	DREXIT2 = 7,
	DRUPDATE = 8,
	IRSELECT = 9,
	IRCAPTURE = 10,
	IRSHIFT = 11,
	IREXIT1 = 12,
	IRPAUSE = 13,
	IREXIT2 = 14,
	IRUPDATE = 15

};

struct aspeed_intel_jtag {
	/* Global variable to store the current JTAG state */
	enum aspeed_jtag_state jtag_state;

	/* Store current stop-state for DR and IR scan commands */
	enum aspeed_jtag_state drstop_state;
	enum aspeed_jtag_state irstop_state;

	/* Store current padding values */
	u32 dr_pre;
	u32 dr_post;
	u32 ir_pre;
	u32 ir_post;
	u32 dr_length;
	u32 ir_length;
	u8 *dr_pre_data;
	u8 *dr_post_data;
	u8 *ir_pre_data;
	u8 *ir_post_data;
	u8 *dr_buffer;
	u8 *ir_buffer;
};

struct aspeed_jtag_config {
	void *dev;
	u8 *action;
	int (*jtag_io) (void *dev, int tms, int tdi, int tdo);
};

struct aspeed_intel_jtag_state {
	struct aspeed_jtag_config	*config;
	struct aspeed_intel_jtag	js;
	char			msg_buff[1024 + 1];
	long			stack[1028];
};

