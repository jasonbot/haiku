/*
 * Copyright 2019-2021 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *   Alexander von Gluck IV <kallisti5@unixzen.com>
 */

#include <arch_cpu_defs.h>
#include <arch_dtb.h>
#include <arch_smp.h>
#include <boot/platform.h>
#include <boot/stage2.h>

extern "C" {
#include <libfdt.h>
}

#include "dtb.h"


void arm64_handle_fdt_psci_node(const void *fdt, int node);
void arm64_handle_fdt_cpu_node(const void *fdt, int node);


/* TODO: Code taken from ARM port just for building purposes */

/* The potential interrupt controoller would be present in the dts as:
 * compatible = "arm,gic-v3";
 */
const struct supported_interrupt_controllers {
	const char*	dtb_compat;
	const char*	kind;
} kSupportedInterruptControllers[] = {
	{ "arm,cortex-a9-gic", INTC_KIND_GICV1 },
	{ "arm,cortex-a15-gic", INTC_KIND_GICV2 },
	{ "arm,gic-400", INTC_KIND_GICV2 },
	{ "ti,omap3-intc", INTC_KIND_OMAP3 },
	{ "marvell,pxa-intc", INTC_KIND_PXA },
};


void
arch_handle_fdt(const void* fdt, int node)
{
	const char* deviceType = (const char*)fdt_getprop(fdt, node,
		"device_type", NULL);

	if (deviceType != NULL) {
		if (strcmp(deviceType, "cpu") == 0) {
			arm64_handle_fdt_cpu_node(fdt, node);
		}
	}

	int compatibleLen;
	const char* compatible = (const char*)fdt_getprop(fdt, node,
		"compatible", &compatibleLen);

	if (compatible == NULL)
		return;

	intc_info &interrupt_controller = gKernelArgs.arch_args.interrupt_controller;
	if (interrupt_controller.kind[0] == 0) {
		for (uint32 i = 0; i < B_COUNT_OF(kSupportedInterruptControllers); i++) {
			if (dtb_has_fdt_string(compatible, compatibleLen,
				kSupportedInterruptControllers[i].dtb_compat)) {

				memcpy(interrupt_controller.kind, kSupportedInterruptControllers[i].kind,
					sizeof(interrupt_controller.kind));

				dtb_get_reg(fdt, node, 0, interrupt_controller.regs1);
				dtb_get_reg(fdt, node, 1, interrupt_controller.regs2);
			}
		}
	}

	if (strcmp(compatible, "arm,psci-1.0") == 0)
		arm64_handle_fdt_psci_node(fdt, node);
}


void
arch_dtb_set_kernel_args(void)
{
	intc_info &interrupt_controller = gKernelArgs.arch_args.interrupt_controller;

	// Intentional bring-up fallback for the Pinebook Pro (RK3399): some
	// U-Boot control DTBs omit the "arm,gic-400" interrupt-controller node
	// entirely, so the FDT walk above finds no interrupt controller and the
	// kernel would boot without a GIC (no timer interrupt). If we found no
	// interrupt controller and this is an RK3399, synthesize the fixed GICv2
	// register locations.
	if (interrupt_controller.kind[0] == 0
		&& gKernelArgs.arch_args.fdt != NULL) {
		const void* fdt = gKernelArgs.arch_args.fdt;
		int root = fdt_path_offset(fdt, "/");
		if (root >= 0) {
			int compatLen;
			const char* compatible = (const char*)fdt_getprop(fdt, root,
				"compatible", &compatLen);
			if (compatible != NULL
				&& dtb_has_fdt_string(compatible, compatLen,
					"rockchip,rk3399")) {
				memcpy(interrupt_controller.kind, INTC_KIND_GICV2,
					sizeof(interrupt_controller.kind));
				interrupt_controller.regs1.start = 0xfee00000;
				interrupt_controller.regs1.size = 0x10000;
				interrupt_controller.regs2.start = 0xfef00000;
				interrupt_controller.regs2.size = 0xc0000;
				dprintf("Synthesized RK3399 GICv2 interrupt controller "
					"(kind: %s)\n", interrupt_controller.kind);
			}
		}
	}

	dprintf("Chosen interrupt controller:\n");
	if (interrupt_controller.kind[0] == 0) {
		dprintf("kind: None!\n");
	} else {
		dprintf("  kind: %s\n", interrupt_controller.kind);
		dprintf("  regs: %#" B_PRIx64 ", %#" B_PRIx64 "\n",
			interrupt_controller.regs1.start,
			interrupt_controller.regs1.size);
		dprintf("        %#" B_PRIx64 ", %#" B_PRIx64 "\n",
			interrupt_controller.regs2.start,
			interrupt_controller.regs2.size);
	}
}
