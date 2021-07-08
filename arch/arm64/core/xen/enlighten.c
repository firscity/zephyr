/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm64/hypercall.h>
#include <xen/events.h>
#include <xen/generic.h>
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
static uint8_t shared_info_buf[XEN_PAGE_SIZE]
			__attribute__((aligned(XEN_PAGE_SIZE)));

/* Remains NULL until mapping will be finished by Xen */
shared_info_t *HYPERVISOR_shared_info = NULL;

static int xen_map_shared_info (const shared_info_t *shared_page) {
	struct xen_add_to_physmap xatp;

	xatp.domid = DOMID_SELF;
	xatp.idx = 0;
	xatp.space = XENMAPSPACE_shared_info;
	xatp.gpfn = (((xen_pfn_t) shared_page) >> XEN_PAGE_SHIFT);

	return HYPERVISOR_memory_op(XENMEM_add_to_physmap, &xatp);
}

static int xen_enlighten_init(const struct device *dev) {
	ARG_UNUSED(dev);
	int ret = 0;
	shared_info_t *info = (shared_info_t *) shared_info_buf;

	ret = xen_map_shared_info(info);
	if (ret) {
		printk("%s: failed to map for Xen shared page, ret = %d\n",
			__func__, ret);
		return ret;
	}

	/* Set value for globally visible pointer */
	HYPERVISOR_shared_info = info;

	ret = xen_events_init();
	if (ret) {
		printk("%s: failed init Xen event channels, ret = %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

SYS_INIT(xen_enlighten_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
