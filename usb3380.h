/*
 * Internal usb3380 register header file
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef USB3380_H
#define USB3380_H

#include <stdint.h>

enum usb3380_endpoints {
	EP_EP0   = 0x0,
	EP_GPEP0 = 0x2,
	EP_GPEP1 = 0x4,
	EP_GPEP2 = 0x6,
	EP_GPEP3 = 0x8,

	EP_CSROUT = 0xd,
	EP_CSRIN  = 0xd,
	EP_PCIOUT = 0xe,
	EP_PCIIN  = 0xe,
	EP_STATIN = 0xf,
	EP_RCIN   = 0xc,
};

/** structure for CSROUT data out */
typedef struct CSROUT_ep {
	uint32_t csrctl;
	uint32_t csrdata;
} CSROUT_ep_t;

enum usb3380_regs_csrctl_bits {
	CSR_BYTE_EN_0   = 1 << 0,
	CSR_BYTE_EN_1   = 1 << 1,
	CSR_BYTE_EN_2   = 1 << 2,
	CSR_BYTE_EN_3   = 1 << 3,

	CSR_BYTE_EN_ALL = CSR_BYTE_EN_0 | CSR_BYTE_EN_1 | CSR_BYTE_EN_2 | CSR_BYTE_EN_3,

	CSR_SS_PCIE_CFG = 0 << 4,
	CSR_SS_MM_CFG   = 1 << 4,
	CSR_SS_8051_PRG = 2 << 4,
	CSR_START       = 1 << 6,
	CSR_WRITE       = 0 << 7,
	CSR_READ        = 1 << 7,

	CSR_ADDR_OFF_BITS = 16,
};


enum usb3380_regs_pcimstctl_bits {
	PCIMSTCTL_CMD_MEMORY = 0 << 4,
	PCIMSTCTL_CMD_IO     = 1 << 4,
	PCIMSTCTL_CMD_CONFIG = 2 << 4,
	PCIMSTCTL_CMD_MSG    = 3 << 4,

	PCIMSTCTL_MASTER_START = 1 << 6,

	PCIMSTCTL_PCIE_WRITE = 0 << 7,
	PCIMSTCTL_PCIE_READ = 1 << 7,

	PCIMSTCTL_PCIE_MSG_CODE_BITS = 8,

	PCIMSTCTL_PCIE_DW_LEN_OFF_BITS = 24,
};

enum usb3380_regs {
	DEVINIT = 0x00,

	PCICTL = 0x0c,
	PCIIRQENB0 = 0x10,
	PCIIRQENB1 = 0x14,
	CPUIRQENB0 = 0x18,
	CPUIRQENB1 = 0x1c,
	USBIRQENB0 = 0x20,
	USBIRQENB1 = 0x24,

	IRQSTAT0 = 0x28, /**< Also in STATIN endpoint */
	IRQSTAT1 = 0x2C, /**< Also in STATIN endpoint */

	IDXADDR = 0x30, /**< see 15.13 Indexed Registers */
	IDXDATA = 0x34, /**< see 15.13 Indexed Registers */

	FIFOCTL = 0x38, /**< FIFO availability control bit 3 */
	BAR2CTL = 0x3c, /**< Register 15-14. */
	BAR3CTL = 0x40, /**< Register 15-15. */

	GPIOCTRL = 0x50,
	GPIOSTAT = 0x54,
	GPIOPWMV = 0x58,
	GPIOPWMRC = 0x5c,
	GPIOPWMFREQ = 0x60,

	ROOTMSGDISP = 0x78,

	// 15.6 USB Interface Control Registers (80 – FF)
	STDRSP = 0x80,
	PRODVENDID = 0x84,
	RELNUM = 0x88,
	USBCTL = 0x8c,
	USBSTAT = 0x90,
	XCVRDIAG = 0x94,
	SETUPDW0 = 0x98,
	SETUPDW1 = 0x9c,

	OURADDR = 0xa4,
	OURCONF = 0xa8,

	USB_CLASS = 0xb4,
	SS_SEL = 0xb8,
	SS_DEL = 0xbc,
	USB2LPM = 0xc0,
	USB3BELT = 0xc8,
	USBCTL2 = 0xc8,
	IN_TIMEOUT = 0xcc, /**< IN Endpoint credit timeout */
	ISODELAY = 0xd0,

	// PCI Express/Configuration Cursor Registers (100 – 17F)
	PCIMSTSTAT = 0x10c,

	SEMAPHORE = 0x118,

	// DMA Registers (180 – 1FC) & (680 – 6BC)

	// Dedicated Endpoint Registers (200 – 254)
	// see usb3380_dep_regs

	// EP 0 and GPEPx Registers (300h – 4FFh)

	// FIFO Registers (500h – 614h)
	EP_FIFO_SIZE_BASE = 0x500,

	// USB Power Management Registers (6C0h – 6C4h)
};

enum usb3380_devinit_regs {
	MCU_RESET_MASK = 1U,
};

enum usb3380_dep_regs {
	DEP_CSROUT_OFF = 0x000,
	DEP_CSRIN_OFF = 0x010,
	DEP_PCIOUT_OFF = 0x020,
	DEP_PCIIN_OFF = 0x030,
	DEP_STATIN_OFF = 0x040,
	DEP_RCIN_OFF = 0x050,

	DEP_CFG = 0x200,
	DEP_RSP = 0x204,
};

enum usb3380_ep_regs {
	EP_EP0_OFF = 0x000,
	EP_GPEP0_OFF = 0x020,
	EP_GPEP1_OFF = 0x040,
	EP_GPEP2_OFF = 0x060,
	EP_GPEP3_OFF = 0x080,
	EP_PCIINOUT_OFF = 0x0E0,
	EP_RCIN_OFF = 0x100,

	EP_CFG = 0x300,
	EP_RSP = 0x304,
	EP_IRQENB = 0x308,
	EP_STAT = 0x30c,
	EP_AVAIL = 0x310,
};

enum usb3380_ep_size {
	EP_FIFO_64 = 0,
	EP_FIFO_128 = 1,
	EP_FIFO_256 = 2,
	EP_FIFO_512 = 3,
	EP_FIFO_1024 = 4,
	EP_FIFO_2048 = 5,
	EP_FIFO_4096 = 6,
};

enum usb3380_pcictl_bits {
	PCIBAR0 = 0,
	PCIBAR1 = 1,
};

/** Bits for BAR2CTL & BAR3CTL */
enum usb3380_barxctl_bits {
	BARXCTL_GPEPX_DIR = 1<<0, /**< IN/OUT direction */
	BARXCTL_GPEP_NUM_OFF_BITS = 1,

	BARXCTL_GPEP_Q0_OFF_BITS = 0,
	BARXCTL_GPEP_Q1_OFF_BITS = 4,
	BARXCTL_GPEP_Q2_OFF_BITS = 8,
	BARXCTL_GPEP_Q3_OFF_BITS = 12,

	/** Range of BARx 0xffff - 64KB, 0xfffe - 128KB, ... ,0x1000 - 2GB */
	BARXCTL_GPEP_RANGE_OFF_BITS = 16,
};

/** USBCTL */
enum usb3380_usbctl_bits {
	USBSTAT_GEN_RES = 1<<5, /**< Generate Resume */
	USBSTAT_FS_MODE = 1<<6, /**< 12 Mbps */
	USBSTAT_HS_MODE = 1<<7, /**< 480 Mbps */
	USBSTAT_SS_MODE = 1<<8, /**< 5 Gbps */
	USBSTAT_SUSPEND = 1<<9,
};

/** Register 15-21. Root Message Dispatch Root Message Dispatch */
enum usb3380_rootmsgdisp_bits {
	ROOTMSG_DISP_CORRERR = 1<<0,
	ROOTMSG_DISP_NONFATAL = 1<<1,
	ROOTMSG_DISP_FATAL = 1<<2,
	ROOTMSG_DISP_MSI = 1<<3,
	ROOTMSG_DISP_INTA = 1<<4,
	ROOTMSG_DISP_INTB = 1<<5,
	ROOTMSG_DISP_INTC = 1<<6,
	ROOTMSG_DISP_INTD = 1<<7,
	ROOTMSG_DISP_PME = 1<<8,
};

/** special indexed registers accesed via IDXADDR / IDXDATA
 *
 * EP Max packet size, poll interval for Interrupt, etc. USB-specific
 * values can be set here
 */
enum usb3380_indexed_regs {
	CHIPREV         = 0x03,
	HS_INTPOLL_RATE = 0x08,
	FS_INTPOLL_RATE = 0x09,
};

enum usb3380_pcie_ports {
	PCIE_PORT_0 = 0x0000, /**< Virtual PCI-PCI Bridge */
	PCIE_PORT_2 = 0x2000, /**< Root Complex */
	PCIE_PORT_USBC = 0x3000, /**< USB Controller */
};

/* PCI configuration registers */
enum pcicfg_regs {
	PCICFG_VIDDID  = 0x00,
	PCICFG_CMDSTAT = 0x04,
	PCICFG_CLASS   = 0x08,
	PCICFG_MSCCTL  = 0x0c,
	PCICFG_BAR0    = 0x10,
	PCICFG_BAR1    = 0x14,
	PCICFG_TYPE1_BUSNUM = 0x18,
	PCICFG_TYPE1_SSTATIO = 0x1c,
	PCICFG_TYPE1_MEMBASELMT = 0x20,
	PCICFG_TYPE1_MEMPREF = 0x24,
	PCICFG_TYPE1_PREF_UPBASE = 0x28,
	PCICFG_TYPE1_PREF_UPLMT = 0x2c,
	PCICFG_TYPE1_IOBBASELIM = 0x30,
	PCICFG_TYPE1_CAPPTR = 0x34,
	PCICFG_TYPE1_EXPROM = 0x38,
	PCICFG_TYPE1_BRIDGECTRL = 0x3c,

	PCICFG_BAR2    = 0x18,
	PCICFG_BAR3    = 0x1C,
	PCICFG_BAR4    = 0x20,
	PCICFG_BAR5    = 0x24,

	PCICFG_PCIECAP_DEVSC = 0x70,
};

enum pcicfg_bar_bits {
	BAR_TYPE_IO = 0x1,
	BAR_TYPE_MEM_MSK = 0x5,


	BAR_TYPE_MEM_32BIT = 0,
	BAR_TYPE_MEM_64BIT = 4,
};

enum pcicfg_busno_bits {
	BUSNO_PRIMARY_OFF = 0,
	BUSNO_SECONDARY_OFF = 8,
	BUSNO_SUBORDINATE_OFF = 16,
};

enum pcicfg_mscctl_bits {
	HEADER_TYPE_OFF_BITS = 16,
	HEADER_TYPE_MASK = 0x7f,

	HEADER_TYPE_STANDARD = 0,
	HEADER_TYPE_PCITOPCI = 1,
};

enum pcicfg_cap_types {
	PCICAP_MSI = 0x05,
	PCICAP_PCIE = 0x10,
};

enum pcicfg_cap_msi_bits {
	PCICAP_MSI_ENABLE = 1<<0,
	PCICAP_MSI_MMCAP_OFF_BITS = 1,
	PCICAP_MSI_MMEN_OFF_BITS = 4,
	PCICAP_MSI_64BIT = 1<<7,
};

#endif
