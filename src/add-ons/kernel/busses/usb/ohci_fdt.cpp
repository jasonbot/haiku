/*
 * Copyright 2026 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

#include <new>
#include <stdio.h>
#include <string.h>

#include <KernelExport.h>
#include <device_manager.h>
#include <drivers/bus/FDT.h>
#include <USB3.h>

#include "ohci.h"


#define USB_MODULE_NAME	"ohci_fdt"

#define CALLED(x...)	TRACE_MODULE("CALLED %s\n", __PRETTY_FUNCTION__)


device_manager_info* gDeviceManager;
static usb_for_controller_interface* gUSB;


#define OHCI_FDT_DEVICE_MODULE_NAME "busses/usb/ohci_fdt/driver_v1"
#define OHCI_FDT_USB_BUS_MODULE_NAME "busses/usb/ohci_fdt/device_v1"


struct ohci_fdt_sim_info {
	OHCI*			ohci;
	uint64			regs;
	uint64			regsLen;
	uint64			interrupt;
	device_node*	driver_node;
};


//	#pragma mark -


static status_t
init_bus(device_node* node, void** bus_cookie)
{
	CALLED();

	driver_module_info* driver;
	ohci_fdt_sim_info* bus;
	device_node* parent = gDeviceManager->get_parent_node(node);
	gDeviceManager->get_driver(parent, &driver, (void**)&bus);
	gDeviceManager->put_node(parent);

	Stack *stack;
	if (gUSB->get_stack((void**)&stack) != B_OK) {
		return B_ERROR;
	}

	uint8 offset = bus->regs & (B_PAGE_SIZE - 1);
	phys_addr_t physicalBase = bus->regs - offset;
	size_t mapSize = (bus->regsLen + offset + B_PAGE_SIZE - 1)
		& ~(B_PAGE_SIZE - 1);

	OHCI *ohci = new(std::nothrow) OHCI(physicalBase, mapSize,
		(int32)bus->interrupt, stack, node);
	if (ohci == NULL) {
		return B_NO_MEMORY;
	}

	if (ohci->InitCheck() < B_OK) {
		TRACE_MODULE_ERROR("bus failed init check\n");
		delete ohci;
		return B_ERROR;
	}

	if (ohci->Start() != B_OK) {
		delete ohci;
		return B_ERROR;
	}

	bus->ohci = ohci;
	*bus_cookie = ohci;

	return B_OK;
}


static void
uninit_bus(void* bus_cookie)
{
	CALLED();
	OHCI* ohci = (OHCI*)bus_cookie;
	delete ohci;
}


static status_t
register_child_devices(void* cookie)
{
	CALLED();
	ohci_fdt_sim_info* bus = (ohci_fdt_sim_info*)cookie;
	device_node* node = bus->driver_node;

	char prettyName[25];
	sprintf(prettyName, "OHCI Controller %" B_PRIu16, 0);

	device_attr attrs[] = {
		// properties of this controller for the usb bus manager
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = prettyName }},
		{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE,
			{ .string = USB_FOR_CONTROLLER_MODULE_NAME }},

		// private data to identify the device
		{ NULL }
	};

	return gDeviceManager->register_node(node, OHCI_FDT_USB_BUS_MODULE_NAME,
		attrs, NULL, NULL);
}


static void
_init_rk3399_usb2_phy(uint64 regs)
{
	// The RK3399 USB2 PHY control registers live in the GRF at 0xff770000,
	// which uses the write-mask scheme: bits [31:16] mask, bits [15:0] value.
	// The companion OHCI shares its port's PHY with its EHCI partner, so
	// these writes are idempotent with what ehci_fdt already performed.
	uint32 clkout, phySus;
	if (regs == 0xfe3a0000) {
		// usb_host0_ohci -> u2phy0 HOST port
		clkout = 0xe450;
		phySus = 0xe458;
	} else if (regs == 0xfe3e0000) {
		// usb_host1_ohci -> u2phy1 HOST port
		clkout = 0xe460;
		phySus = 0xe468;
	} else {
		return;
	}

	void* grfBase = NULL;
	area_id grfArea = map_physical_memory("RK3399 GRF",
		0xff770000, 0x10000, B_ANY_KERNEL_BLOCK_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &grfBase);
	if (grfArea < 0 || grfBase == NULL)
		return;

	volatile uint32* grf = (volatile uint32*)grfBase;

	// Enable the 24MHz reference clock output (clkout_ctl bit 4 =
	// 0 to enable). Matches phy-rockchip-inno-usb2.c.
	grf[clkout / 4] = 0x00100000;

	// Power the HOST port: phy_sus bits[1:0] = 0b10 power on /
	// normal operation. (Writing 0b01 would suspend the port and
	// stop the companion OHCI's reference clock, making its
	// registers unreadable.)
	grf[phySus / 4] = 0x00030002;

	delete_area(grfArea);
}


static status_t
init_device(device_node* node, void** device_cookie)
{
	CALLED();
	ohci_fdt_sim_info* bus = new(std::nothrow) ohci_fdt_sim_info;
	if (bus == NULL) {
		return B_NO_MEMORY;
	}

	device_node* parent = gDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	gDeviceManager->get_driver(parent, (driver_module_info**)&fdt,
		(void**)&fdtDevice);
	gDeviceManager->put_node(parent);

	uint64 regs, regsLen;
	if (!fdt->get_reg(fdtDevice, 0, &regs, &regsLen)) {
		TRACE_MODULE_ERROR("no registers for FDT OHCI node\n");
		delete bus;
		return B_ERROR;
	}

	uint64 interrupt;
	device_node* interruptController;
	if (!fdt->get_interrupt(fdtDevice, 0, &interruptController, &interrupt)) {
		TRACE_MODULE_ERROR("no interrupt for FDT OHCI node\n");
		delete bus;
		return B_ERROR;
	}

	// Enable USB host clocks via CRU (Clock Reset Unit) at 0xff760000.
	// RK3399 gates use hiword-mask semantics: bits[31:16] mask, bits[15:0]
	// value, with CLK_GATE_SET_TO_DISABLE (write 0 = clock enabled).
	// Controller clocks for the usb_host0/usb_host1 EHCI+OHCI pairs:
	//   HCLK_HOST0/ARB, HCLK_HOST1/ARB, HCLK_HSIC   -> CLKGATE_CON20 (0x0350) bits 5-9
	//   SCLK_USB2PHY0_REF, _1_REF, clk_hsicphy      -> CLKGATE_CON6  (0x0318) bits 4-6
	//   ACLK_USB3                                   -> CLKGATE_CON12 (0x0330) bit 0
	//   clk_usbphy*_480m_src                        -> CLKGATE_CON13 (0x0334) bit 12
	//   PCLK_USBPHY_MUX_G                           -> CLKGATE_CON21 (0x0354) bit 4
	//   ACLK_USB3_NOC/OTG0/OTG1/PERF/GRF            -> CLKGATE_CON30 (0x0378) bits 0-4
	// USB host soft resets live in SOFTRST_CON7 (0x041c):
	//   bit3=USBHOST0, bit4=HOSTC0_AUX, bit5=HOST0_ARB,
	//   bit6=USBHOST1, bit7=HOSTC1_AUX, bit8=HOST1_ARB, bit11=HSIC
	if (regs == 0xfe3a0000 || regs == 0xfe3e0000) {
		void* cru_base = NULL;
		area_id cru_area = map_physical_memory("RK3399 CRU",
			0xff760000, 0x1000, B_ANY_KERNEL_BLOCK_ADDRESS,
			B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
			&cru_base);
		if (cru_area >= 0 && cru_base != NULL) {
			volatile uint32* cru = (volatile uint32*)cru_base;
			dprintf("ohci_fdt: CRU gates: CON6 0x%08" B_PRIx32
				" CON20 0x%08" B_PRIx32 " CON21 0x%08" B_PRIx32
				" CON30 0x%08" B_PRIx32 "\n",
				cru[0x0318 / 4], cru[0x0350 / 4], cru[0x0354 / 4],
				cru[0x0378 / 4]);
			dprintf("ohci_fdt: CRU resets: CON7 0x%08" B_PRIx32
				" CON9 0x%08" B_PRIx32 " CON18 0x%08" B_PRIx32 "\n",
				cru[0x041c / 4], cru[0x0424 / 4], cru[0x0448 / 4]);
			cru[0x0350 / 4] = 0x03e00000; // CON20 bits 5-9
			cru[0x0318 / 4] = 0x00700000; // CON6  bits 4-6
			cru[0x0330 / 4] = 0x00010000; // CON12 bit 0
			cru[0x0334 / 4] = 0x00100000; // CON13 bit 12 (480m src)
			cru[0x0354 / 4] = 0x00100000; // CON21 bit 4
			cru[0x0378 / 4] = 0x001f0000; // CON30 bits 0-4
			// Deassert the USB host / HSIC soft resets.
			cru[0x041c / 4] = 0x01f80000; // CON7 bits 3-8 + 11
			delete_area(cru_area);
		}
	}

	// Enable the USB2 PHY host port for this OHCI companion.
	_init_rk3399_usb2_phy(regs);

	bus->ohci = NULL;
	bus->regs = regs;
	bus->regsLen = regsLen;
	bus->interrupt = interrupt;
	bus->driver_node = node;

	TRACE_MODULE("FDT OHCI node regs 0x%" B_PRIx64 " size 0x%" B_PRIx64
		" irq %" B_PRIu64 "\n", regs, regsLen, interrupt);

	*device_cookie = bus;
	return B_OK;
}


static void
uninit_device(void* device_cookie)
{
	CALLED();
	ohci_fdt_sim_info* bus = (ohci_fdt_sim_info*)device_cookie;
	delete bus;
}


static status_t
register_device(device_node* parent)
{
	CALLED();
	device_attr attrs[] = {
		{B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{.string = "FDT OHCI Host Controller"}},
		{}
	};

	return gDeviceManager->register_node(parent,
		OHCI_FDT_DEVICE_MODULE_NAME, attrs, NULL, NULL);
}


static bool
_is_ohci_compatible(const char* compatible)
{
	if (strcmp(compatible, "generic-ohci") == 0)
		return true;
	if (strncmp(compatible, "rockchip,", 9) == 0
		&& strstr(compatible, "-ohci") != NULL)
		return true;
	return false;
}


static float
supports_device(device_node* parent)
{
	CALLED();
	const char* bus;
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
		!= B_OK) {
		return -1;
	}

	if (strcmp(bus, "fdt") != 0)
		return 0.0f;

	device_attr* attr = NULL;
	while (gDeviceManager->get_next_attr(parent, &attr) == B_OK) {
		if (attr->type != B_STRING_TYPE)
			continue;
		if (strcmp(attr->name, "fdt/compatible") != 0)
			continue;
		if (_is_ohci_compatible(attr->value.string))
			return 0.8f;
	}

	return 0.0f;
}


module_dependency module_dependencies[] = {
	{ USB_FOR_CONTROLLER_MODULE_NAME, (module_info**)&gUSB },
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{}
};


static usb_bus_interface gOHCIFDTDeviceModule = {
	{
		{
			OHCI_FDT_USB_BUS_MODULE_NAME,
			0,
			NULL
		},
		NULL,  // supports device
		NULL,  // register device
		init_bus,
		uninit_bus,
		NULL,  // register child devices
		NULL,  // rescan
		NULL,  // device removed
	},
};


static driver_module_info sOHCIFDTDevice = {
	{
		OHCI_FDT_DEVICE_MODULE_NAME,
		0,
		NULL
	},
	supports_device,
	register_device,
	init_device,
	uninit_device,
	register_child_devices,
	NULL, // rescan
	NULL, // device removed
};


module_info* modules[] = {
	(module_info* )&sOHCIFDTDevice,
	(module_info* )&gOHCIFDTDeviceModule,
	NULL
};
