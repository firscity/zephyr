/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm64/hypercall.h>
#include <xen/events.h>
#include <xen/public/xen.h>
#include <xen/public/memory.h>

#include <kernel.h>
#include <init.h>
#include <sys/printk.h>
#include <device.h>
#include <kernel/thread.h>

/*
 * During Xen Enlighten initialization we need to have allocated memory page,
 * where hypervisor shared_info will be mapped. k_aligned_alloc() is not
 * available on PRE_KERNEL_1 stage, so we will use statically allocated buffer,
 * which will be casted to 'struct shared_info'. It is needed to initialize Xen
 * event channels as soon as possible after start.
 */
static uint8_t shared_info_buf[CONFIG_MMU_PAGE_SIZE]
			__attribute__((aligned (CONFIG_MMU_PAGE_SIZE)));

/* Remains NULL until mapping will be finished by Xen */
shared_info_t *HYPERVISOR_shared_info = NULL;


int xen_consoleio_putc(int c)
{
	register unsigned int r0 __asm__("r0") = CONSOLEIO_write;
	register size_t r1 __asm__("r1") = sizeof(char);
	register char * r2 __asm__("r2") = (char *) &c;
	register unsigned int r16 __asm__("r16") = __HYPERVISOR_console_io;
	register int ret __asm__("r0");

	__asm__ volatile ("hvc #0xEA1" : "=r" (ret) : "0" (r0), "r" (r1),
			"r" (r2), "r" (r16) : "memory");

	if (ret == 0)
		return sizeof(char);
	else
		return 0;
}

static int xen_map_shared_info (const struct shared_info *shared_page) {
	int ret = 0;
	struct xen_add_to_physmap xatp;

	xatp.domid = DOMID_SELF;
	xatp.idx = 0;
	xatp.space = XENMAPSPACE_shared_info;
	xatp.gpfn = ( ((xen_pfn_t) shared_page) >> 12);

	ret = HYPERVISOR_memory_op(XENMEM_add_to_physmap, &xatp);

	printk("%s: enlighten mapping result = %d\n", __func__, ret);

	return ret;
}

static int xen_enlighten_init(const struct device *dev) {
	ARG_UNUSED(dev);
	int ret = 0;

	struct shared_info *info = (struct shared_info *) shared_info_buf;

	__stdout_hook_install(xen_consoleio_putc);
	__printk_hook_install(xen_consoleio_putc);
	ret = xen_map_shared_info(info);
	if (ret) {
		printk("%s: failed to map for Xen shared page, ret = %d\n",
			__func__, ret);
		return ret;
	}

	HYPERVISOR_shared_info = info;
	printk("Xen Enlighten mapped to %p\n", HYPERVISOR_shared_info);

	xen_events_init();
	return ret;
}

SYS_INIT(xen_enlighten_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
