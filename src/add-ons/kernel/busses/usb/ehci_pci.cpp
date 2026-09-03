/*
 * Copyright 2006-2011, Haiku Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		Michael Lotz <mmlr@mlotz.ch>
 *		Jérôme Duval <korli@users.berlios.de>
 */


#include <stdio.h>

#include <driver_settings.h>
#include <bus/PCI.h>
#include <USB3.h>
#include <KernelExport.h>

#include "ehci.h"


#define CALLED(x...)	TRACE_MODULE("CALLED %s\n", __PRETTY_FUNCTION__)

#define USB_MODULE_NAME	"ehci"


device_manager_info* gDeviceManager;
static usb_for_controller_interface* gUSB;


#define EHCI_PCI_DEVICE_MODULE_NAME "busses/usb/ehci/pci/driver_v1"
#define EHCI_PCI_USB_BUS_MODULE_NAME "busses/usb/ehci/device_v1"


typedef struct {
	EHCI*	ehci;
	pci_device_module_info* pci;
	pci_device* device;

	pci_info pciinfo;

	device_node* node;
	device_node* driver_node;
} ehci_pci_sim_info;


//	#pragma mark -


static status_t
init_bus(device_node* node, void** bus_cookie)
{
	CALLED();

	driver_module_info* driver;
	ehci_pci_sim_info* bus;
	device_node* parent = gDeviceManager->get_parent_node(node);
	gDeviceManager->get_driver(parent, &driver, (void**)&bus);
	gDeviceManager->put_node(parent);

	Stack *stack;
	if (gUSB->get_stack((void**)&stack) != B_OK)
		return B_ERROR;

	EHCI *ehci = new(std::nothrow) EHCI(&bus->pciinfo, bus->pci, bus->device, stack, node);
	if (ehci == NULL) {
		return B_NO_MEMORY;
	}

	if (ehci->InitCheck() < B_OK) {
		TRACE_MODULE_ERROR("bus failed init check\n");
		delete ehci;
		return B_ERROR;
	}

	if (ehci->Start() != B_OK) {
		delete ehci;
		return B_ERROR;
	}

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
	ehci_pci_sim_info* bus = (ehci_pci_sim_info*)cookie;
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

	return gDeviceManager->register_node(node, EHCI_PCI_USB_BUS_MODULE_NAME,
		attrs, NULL, NULL);
}


static status_t
init_device(device_node* node, void** device_cookie)
{
	CALLED();
	ehci_pci_sim_info* bus = (ehci_pci_sim_info*)calloc(1,
		sizeof(ehci_pci_sim_info));
	if (bus == NULL)
		return B_NO_MEMORY;

	pci_device_module_info* pci;
	pci_device* device;
	{
		device_node* pciParent = gDeviceManager->get_parent_node(node);
		gDeviceManager->get_driver(pciParent, (driver_module_info**)&pci,
			(void**)&device);
		gDeviceManager->put_node(pciParent);
	}

	bus->pci = pci;
	bus->device = device;
	bus->driver_node = node;

	pci_info *pciInfo = &bus->pciinfo;
	pci->get_pci_info(device, pciInfo);

	*device_cookie = bus;
	return B_OK;
}


static void
uninit_device(void* device_cookie)
{
	CALLED();
	ehci_pci_sim_info* bus = (ehci_pci_sim_info*)device_cookie;
	free(bus);

}


static status_t
register_device(device_node* parent)
{
	CALLED();
	device_attr attrs[] = {
		{B_DEVICE_PRETTY_NAME, B_STRING_TYPE, {.string = "EHCI PCI"}},
		{}
	};

	return gDeviceManager->register_node(parent,
		EHCI_PCI_DEVICE_MODULE_NAME, attrs, NULL, NULL);
}


static float
supports_device(device_node* parent)
{
	CALLED();
	const char* bus;
	uint16 type, subType, api;

	// make sure parent is a EHCI PCI device node
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
		< B_OK) {
		return -1;
	}

	if (strcmp(bus, "pci") != 0)
		return 0.0f;

	if (gDeviceManager->get_attr_uint16(parent, B_DEVICE_SUB_TYPE, &subType,
			false) < B_OK
		|| gDeviceManager->get_attr_uint16(parent, B_DEVICE_TYPE, &type,
			false) < B_OK
		|| gDeviceManager->get_attr_uint16(parent, B_DEVICE_INTERFACE, &api,
			false) < B_OK) {
		TRACE_MODULE("Could not find type/subtype/interface attributes\n");
		return -1;
	}

	if (type == PCI_serial_bus && subType == PCI_usb && api == PCI_usb_ehci) {
		pci_device_module_info* pci;
		pci_device* device;
		gDeviceManager->get_driver(parent, (driver_module_info**)&pci,
			(void**)&device);
		TRACE_MODULE("EHCI Device found!\n");

		return 0.8f;
	}

	return 0.0f;
}


module_dependency module_dependencies[] = {
	{ USB_FOR_CONTROLLER_MODULE_NAME, (module_info**)&gUSB },
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{}
};


static usb_bus_interface gEHCIPCIDeviceModule = {
	{
		{
			EHCI_PCI_USB_BUS_MODULE_NAME,
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

// Root device that binds to the PCI bus. It will register an usb_bus_interface
// node for each device.
static driver_module_info sEHCIDevice = {
	{
		EHCI_PCI_DEVICE_MODULE_NAME,
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
	(module_info* )&sEHCIDevice,
	(module_info* )&gEHCIPCIDeviceModule,
	NULL
};
