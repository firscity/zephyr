/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/xen/generic.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>

#define FDT_SIZE_BYTES	0x8000

/* x0 from bootloader stored on boot */
__attribute__((__section__(".data")))
uintptr_t xen_fdt_addr;

__attribute__((__section__(".data")))
uint32_t xen_fdt_size;

/* buffer for device tree blob */
__attribute__((__section__(".data")))
uint8_t xen_fdt[FDT_SIZE_BYTES] __aligned(XEN_PAGE_SIZE);

extern void z_early_memcpy(void *dst, const void *src, size_t n);
extern void z_early_memset(void *dst, int c, size_t n);

void xen_copy_fdt(void)
{
	z_early_memset(xen_fdt, 0, FDT_SIZE_BYTES);
	z_early_memcpy(xen_fdt, (const void *) xen_fdt_addr,
		       sys_be32_to_cpu(xen_fdt_size));
}

static int xen_print_fdt(void)
{
	printk("Saved Xen device tree address is 0x%lx, size = 0x%x\n",
	       xen_fdt_addr, sys_be32_to_cpu(xen_fdt_size));
	return 0;
}

SYS_INIT(xen_print_fdt, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
