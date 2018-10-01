/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
#ifndef DT_BINDINGS_ASPEED_CLOCK_H
#define DT_BINDINGS_ASPEED_CLOCK_H

#define ASPEED_CLK_GATE_ECLK		0
#define ASPEED_CLK_GATE_GCLK		1
#define ASPEED_CLK_GATE_MCLK		2
#define ASPEED_CLK_GATE_VCLK		3
#define ASPEED_CLK_GATE_BCLK		4
#define ASPEED_CLK_GATE_DCLK		5
#define ASPEED_CLK_GATE_REFCLK		6
#define ASPEED_CLK_GATE_USBPORT2CLK	7
#define ASPEED_CLK_GATE_LCLK		8
#define ASPEED_CLK_GATE_USBUHCICLK	9
#define ASPEED_CLK_GATE_D1CLK		10
#define ASPEED_CLK_GATE_YCLK		11
#define ASPEED_CLK_GATE_USBPORT1CLK	12
#define ASPEED_CLK_GATE_UART1CLK	13
#define ASPEED_CLK_GATE_UART2CLK	14
#define ASPEED_CLK_GATE_UART5CLK	15
#define ASPEED_CLK_GATE_ESPICLK		16
#define ASPEED_CLK_GATE_MAC1CLK		17
#define ASPEED_CLK_GATE_MAC2CLK		18
#define ASPEED_CLK_GATE_RSACLK		19
#define ASPEED_CLK_GATE_UART3CLK	20
#define ASPEED_CLK_GATE_UART4CLK	21
#define ASPEED_CLK_GATE_SDCLK		22
#define ASPEED_CLK_GATE_LHCCLK		23
#define ASPEED_CLK_GATE_SDEXTCLK	24
#define ASPEED_CLK_HPLL			25
#define ASPEED_CLK_AHB			26
#define ASPEED_CLK_APB			27
#define ASPEED_CLK_UART			28
#define ASPEED_CLK_SDIO			29
#define ASPEED_CLK_ECLK			30
#define ASPEED_CLK_ECLK_MUX		31
#define ASPEED_CLK_LHCLK		32
#define ASPEED_CLK_MAC			33
#define ASPEED_CLK_BCLK			34
#define ASPEED_CLK_MPLL			35
#define ASPEED_CLK_24M			36


#define ASPEED_RESET_XDMA		0
#define ASPEED_RESET_MCTP		1
#define ASPEED_RESET_ADC		2
#define ASPEED_RESET_JTAG_MASTER	3
#define ASPEED_RESET_MIC		4
#define ASPEED_RESET_PWM		5
#define ASPEED_RESET_PCIVGA		6
#define ASPEED_RESET_I2C		7
#define ASPEED_RESET_AHB		8
#define ASPEED_RESET_CRT1		9
#define ASPEED_RESET_SDHCI		10
#define ASPEED_RESET_UHCI		11
#define ASPEED_RESET_EHCI_P1	12
#define ASPEED_RESET_CRT		13
#define ASPEED_RESET_MAC2		14
#define ASPEED_RESET_MAC1		15
#define ASPEED_RESET_PECI		16
#define ASPEED_RESET_P2X		17
#define ASPEED_RESET_PCIE_DIR	18
#define ASPEED_RESET_PCIE		19
#define ASPEED_RESET_2D		20
#define ASPEED_RESET_VIDEO		21
#define ASPEED_RESET_LPC		22
#define ASPEED_RESET_HACE		23
#define ASPEED_RESET_EHCI_P2	24
#define ASPEED_RESET_SRAM_CTRL		25
#define ASPEED_RESET_ESPI		26
#endif
