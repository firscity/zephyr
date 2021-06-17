/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm64/hypercall.h>
#include <xen/events.h>
#include <xen/generic.h>
#include <xen/hvm.h>
#include <xen/public/xen.h>
#include <xen/public/hvm/hvm_op.h>
#include <xen/public/hvm/params.h>
#include <xen/public/io/console.h>
#include <xen/public/io/xs_wire.h>
#include <xen/public/io/xenbus.h>
#include <xen/public/io/ring.h>

#include <string.h>
#include <kernel.h>
#include <init.h>
#include <sys/printk.h>
#include <device.h>
#include <kernel_arch_interface.h>

static struct xencons_interface *console_intf = NULL;
static uint64_t console_evtchn = 0;

extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));

static int __write_console(const char *data, int len)
{
	XENCONS_RING_IDX cons, prod;
	int sent = 0;

	cons = console_intf->out_cons;
	prod = console_intf->out_prod;
	compiler_barrier();			/* update queue values before going on */
//	BUG_ON((prod - cons) > sizeof(console_intf->out));

	while ((sent < len) && ((prod - cons) < sizeof(console_intf->out)))
		console_intf->out[MASK_XENCONS_IDX(prod++, console_intf->out)] = data[sent++];

	compiler_barrier();			/* write ring before updating pointer */
	console_intf->out_prod = prod;

	if (sent)
		notify_evtchn(console_evtchn);
	return sent;
}


static int console_write(int c) {
	char ch;
	ch = (char) c;

	__write_console(&ch, sizeof(ch));

	return 0;
}

int xen_console_init(const struct device *dev) {
	ARG_UNUSED(dev);

	int ret = 0;
	uint64_t console_pfn = 0;


	ret = hvm_get_parameter(HVM_PARAM_CONSOLE_EVTCHN, &console_evtchn);
	if (ret) {
		printk("%s: failed to get Xen console event channel, ret = %d\n", __func__, ret);
		return ret;
	}

	ret = hvm_get_parameter(HVM_PARAM_CONSOLE_PFN, &console_pfn);
	if (ret) {
		printk("%s: failed to get Xen console PFN, ret = %d\n", __func__, ret);
		return ret;
	}

	/*
	 * TODO: Unfortunately, virt_region_get() - is a static function,
	 * but Zephyr is identity mapped (phys:virt - 1:1), so it should work
	 */
	console_intf = (struct xencons_interface *)
					(console_pfn << XEN_PAGE_SHIFT);
	arch_mem_map(console_intf, (uintptr_t) console_intf,
			CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW | K_MEM_CACHE_WB);

	char print[] = "First print the Zephyr PV console!\n";
	(void)__write_console(print, sizeof(print));

	//__printk_hook_install(console_write);
	//__stdout_hook_install(console_write);

	printk("%s: console inited, evtchn = %llx\n", __func__, console_evtchn);
	return 0;
}

