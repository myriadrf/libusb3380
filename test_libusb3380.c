/*
 * Simple test to check libusb3380
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
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

#include "xtrx_port.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "libusb3380.h"

int main(int argc, char** argv)
{
	int bus = 0;
	int dev = 0;
	int func = 0;
	unsigned i;
	libusb3380_context_t* ctx;
	libusb3380_pcidev_t* pcidev;
	libusb3380_pcie_rc_cfg_t pcicfg;
	uint32_t cfg[64];

	if (usb3380_context_init(&ctx)) {
		return 1;
	}
	if (argc > 1) {
		bus = atoi(argv[1]);
	}
	if (argc > 2) {
		dev = atoi(argv[2]);
	}
	if (argc > 3) {
		func = atoi(argv[3]);
	}

	usb3380_init_rc_def(&pcicfg);
	if (usb3380_init_root_complex(ctx, &pcicfg)) {
		return 3;
	}

	for (i = 0; i < 64; i++) {
		int res = usb3380_pci_cfg_read(ctx,
									   MAKE_CFG_0(bus, dev, func, i<<2),
									   &cfg[i]);
		if (res) {
			return 3;
		}
	}
	for (i = 0; i < 64; i++) {
		printf("Cfg %02x: %08x\n", i << 2, cfg[i]);
	}

	int res = usb3380_init_first_dev(ctx, 0, &pcidev);
	if (res) {
		printf("No PCIe device were found!\n");
		return 7;
	}

	printf("Detected PCIe device: %04x:%04x\n",
		   usb3380_pci_dev_vid(pcidev),
		   usb3380_pci_dev_did(pcidev));

	for (i = 0; i < 6; i++) {
		printf("BAR%d: addr:%08x len:%08x\n", i,
			   usb3380_pci_dev_bar_addr(pcidev, i),
			   usb3380_pci_dev_bar_length(pcidev, i));
	}
	return 0;
}
