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

#include "ehci.h"


#define USB_MODULE_NAME	"ehci_fdt"

#define CALLED(x...)	TRACE_MODULE("CALLED %s\n", __PRETTY_FUNCTION__)


device_manager_info* gDeviceManager;
static usb_for_controller_interface* gUSB;


#define EHCI_FDT_DEVICE_MODULE_NAME "busses/usb/ehci_fdt/driver_v1"
#define EHCI_FDT_USB_BUS_MODULE_NAME "busses/usb/ehci_fdt/device_v1"


struct ehci_fdt_sim_info {
	EHCI*			ehci;
	uint64			regs;
	uint64			regsLen;
	uint64			interrupt;
	device_node*	driver_node;
};


//	#pragma mark -


static status_t
init_bus(device_node* node, void** bus_cookie)
{
	dprintf("ehci_fdt: init_bus called\n");
	CALLED();

	driver_module_info* driver;
	ehci_fdt_sim_info* bus;
	device_node* parent = gDeviceManager->get_parent_node(node);
	gDeviceManager->get_driver(parent, &driver, (void**)&bus);
	gDeviceManager->put_node(parent);

	dprintf("ehci_fdt: got parent bus info, regs=0x%" B_PRIx64 " len=0x%" B_PRIx64 " irq=%" B_PRIu64 "\n",
		bus->regs, bus->regsLen, bus->interrupt);

	Stack *stack;
	dprintf("ehci_fdt: getting USB stack\n");
	if (gUSB->get_stack((void**)&stack) != B_OK) {
		dprintf("ehci_fdt: failed to get USB stack\n");
		return B_ERROR;
	}
	dprintf("ehci_fdt: got USB stack\n");

	uint8 offset = bus->regs & (B_PAGE_SIZE - 1);
	phys_addr_t physicalBase = bus->regs - offset;
	size_t mapSize = (bus->regsLen + offset + B_PAGE_SIZE - 1)
		& ~(B_PAGE_SIZE - 1);

	dprintf("ehci_fdt: creating EHCI object, physBase=0x%lx mapSize=%zu offset=%u\n",
		physicalBase, mapSize, offset);

	EHCI *ehci = new(std::nothrow) EHCI(physicalBase, mapSize, offset,
		(int32)bus->interrupt, stack, node);
	if (ehci == NULL) {
		dprintf("ehci_fdt: failed to allocate EHCI object\n");
		return B_NO_MEMORY;
	}
	dprintf("ehci_fdt: EHCI object created, checking init\n");

	if (ehci->InitCheck() < B_OK) {
		dprintf("ehci_fdt: InitCheck failed\n");
		TRACE_MODULE_ERROR("bus failed init check\n");
		delete ehci;
		return B_ERROR;
	}
	dprintf("ehci_fdt: InitCheck passed, starting EHCI\n");
	dprintf("ehci_fdt: calling ehci->Start()\n");

	if (ehci->Start() != B_OK) {
		dprintf("ehci_fdt: Start failed\n");
		delete ehci;
		return B_ERROR;
	}
	dprintf("ehci_fdt: EHCI started successfully\n");

	bus->ehci = ehci;
	*bus_cookie = ehci;

	return B_OK;
}


static void
uninit_bus(void* bus_cookie)
{
	CALLED();
	EHCI* ehci = (EHCI*)bus_cookie;
	delete ehci;
}


static status_t
register_child_devices(void* cookie)
{
	CALLED();
	ehci_fdt_sim_info* bus = (ehci_fdt_sim_info*)cookie;
	device_node* node = bus->driver_node;

	char prettyName[25];
	sprintf(prettyName, "EHCI Controller %" B_PRIu16, 0);

	device_attr attrs[] = {
		// properties of this controller for the usb bus manager
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = prettyName }},
		{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE,
			{ .string = USB_FOR_CONTROLLER_MODULE_NAME }},

		// private data to identify the device
		{ NULL }
	};

	return gDeviceManager->register_node(node, EHCI_FDT_USB_BUS_MODULE_NAME,
		attrs, NULL, NULL);
}


static status_t
init_device(device_node* node, void** device_cookie)
{
	dprintf("ehci_fdt: init_device called\n");
	CALLED();
	ehci_fdt_sim_info* bus = new(std::nothrow) ehci_fdt_sim_info;
	if (bus == NULL) {
		dprintf("ehci_fdt: failed to allocate bus info\n");
		return B_NO_MEMORY;
	}

	device_node* parent = gDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	gDeviceManager->get_driver(parent, (driver_module_info**)&fdt,
		(void**)&fdtDevice);
	gDeviceManager->put_node(parent);

	uint64 regs, regsLen;
	dprintf("ehci_fdt: getting registers from FDT\n");
	if (!fdt->get_reg(fdtDevice, 0, &regs, &regsLen)) {
		dprintf("ehci_fdt: no registers for FDT EHCI node\n");
		TRACE_MODULE_ERROR("no registers for FDT EHCI node\n");
		delete bus;
		return B_ERROR;
	}
	dprintf("ehci_fdt: got registers: regs=0x%" B_PRIx64 " len=0x%" B_PRIx64 "\n", regs, regsLen);

	uint64 interrupt;
	device_node* interruptController;
	dprintf("ehci_fdt: getting interrupt from FDT\n");
	if (!fdt->get_interrupt(fdtDevice, 0, &interruptController, &interrupt)) {
		dprintf("ehci_fdt: no interrupt for FDT EHCI node\n");
		TRACE_MODULE_ERROR("no interrupt for FDT EHCI node\n");
		delete bus;
		return B_ERROR;
	}
	dprintf("ehci_fdt: got interrupt: %" B_PRIu64 "\n", interrupt);

	// Enable USB host clocks via CRU (Clock Reset Unit)
	// RK3399 CRU is at 0xff760000
	// CRU_CLKGATE_CON15 (0x037c) controls USB host clocks
	// Bit 3: USB HOST0 (EHCI at 0xfe380000)
	// Bit 4: USB HOST1 (EHCI at 0xfe3c0000)
	if (regs == 0xfe380000 || regs == 0xfe3c0000) {
		dprintf("ehci_fdt: enabling USB host clocks via CRU\n");
		void* cru_base = NULL;
		area_id cru_area = map_physical_memory("RK3399 CRU",
			0xff760000, 0x1000, B_ANY_KERNEL_BLOCK_ADDRESS,
			B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
			&cru_base);
		if (cru_area >= 0 && cru_base != NULL) {
			// CRU_CLKGATE_CON15 is at offset 0x037c
			// Bits are write-1-to-enable, write-0-to-no-change
			// We need to set bits 3 and 4 (mask 0x18) with write mask 0x180000
			volatile uint32* clkgate_con15 = (volatile uint32*)((uint8*)cru_base + 0x037c);
			dprintf("ehci_fdt: CRU mapped at %p, writing to CLKGATE_CON15\n", cru_base);
			*clkgate_con15 = 0x00180018; // Enable USB HOST0 and HOST1 clocks
			dprintf("ehci_fdt: USB host clocks enabled\n");
		} else {
			dprintf("ehci_fdt: failed to map CRU, area=%d\n", cru_area);
		}
	}

	bus->ehci = NULL;
	bus->regs = regs;
	bus->regsLen = regsLen;
	bus->interrupt = interrupt;
	bus->driver_node = node;

	TRACE_MODULE("FDT EHCI node regs 0x%" B_PRIx64 " size 0x%" B_PRIx64
		" irq %" B_PRIu64 "\n", regs, regsLen, interrupt);

	*device_cookie = bus;
	dprintf("ehci_fdt: init_device complete\n");
	return B_OK;
}


static void
uninit_device(void* device_cookie)
{
	CALLED();
	ehci_fdt_sim_info* bus = (ehci_fdt_sim_info*)device_cookie;
	delete bus;
}


static status_t
register_device(device_node* parent)
{
	CALLED();
	device_attr attrs[] = {
		{B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{.string = "FDT EHCI Host Controller"}},
		{}
	};

	return gDeviceManager->register_node(parent,
		EHCI_FDT_DEVICE_MODULE_NAME, attrs, NULL, NULL);
}


static bool
_is_ehci_compatible(const char* compatible)
{
	if (strcmp(compatible, "generic-ehci") == 0)
		return true;
	if (strncmp(compatible, "rockchip,", 9) == 0
		&& strstr(compatible, "-ehci") != NULL)
		return true;
	if (strcmp(compatible, "snps,ehci") == 0)
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
		if (_is_ehci_compatible(attr->value.string))
			return 0.8f;
	}

	return 0.0f;
}


module_dependency module_dependencies[] = {
	{ USB_FOR_CONTROLLER_MODULE_NAME, (module_info**)&gUSB },
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{}
};


static usb_bus_interface gEHCIFDTDeviceModule = {
	{
		{
			EHCI_FDT_USB_BUS_MODULE_NAME,
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


static driver_module_info sEHCIFDTDevice = {
	{
		EHCI_FDT_DEVICE_MODULE_NAME,
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
	(module_info* )&sEHCIFDTDevice,
	(module_info* )&gEHCIFDTDeviceModule,
	NULL
};
