/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm64/hypercall.h>
#include <xen/public/xen.h>
#include <xen/public/hvm/hvm_op.h>
#include <xen/public/hvm/params.h>
#include <xen/public/memory.h>
#include <xen/public/io/console.h>
#include <xen/public/io/xs_wire.h>
#include <xen/public/io/xenbus.h>

#include <string.h>
#include <stdio.h>
#include <kernel.h>
#include <init.h>
#include <errno.h>
#include <sys/printk.h>
#include <device.h>
#include <kernel_arch_interface.h>
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
struct shared_info *HYPERVISOR_shared_info = NULL;

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

//#ifdef CONFIG_...
	xen_console_init(NULL);
//#endif /* CONFIG_... */

	struct shared_info *info = (struct shared_info *) shared_info_buf;

	ret = xen_map_shared_info(info);
	if (ret) {
		printk("%s: failed to map for Xen shared page, ret = %d\n",
			__func__, ret);
		return ret;
	}

	HYPERVISOR_shared_info = info;
	printk("Xen Enlighten mapped to %p\n", HYPERVISOR_shared_info);
	return ret;
}

SYS_INIT(xen_enlighten_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
