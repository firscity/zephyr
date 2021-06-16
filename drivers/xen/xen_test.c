/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <xen/xen.h>

#include <string.h>
#include <stdio.h>
#include <kernel.h>
#include <init.h>
#include <errno.h>
#include <sys/printk.h>
#include <device.h>
#include <kernel_arch_interface.h>
#include <kernel/thread.h>

static struct xencons_interface *console_intf = NULL;
struct shared_info *shared_info_page = NULL;

static uint64_t console_evtchn = 0;

K_KERNEL_STACK_DEFINE(signal_thrd_stack, 1024);
struct k_thread signal_thrd;

K_KERNEL_STACK_DEFINE(read_thrd_stack, 1024 /*1472*/);
struct k_thread read_thrd;

K_KERNEL_STACK_DEFINE(sanity_thrd_stack, 1024);
struct k_thread sanity_thrd;

struct k_condvar xenstore_condvar;
struct k_mutex xenstore_mutex;
struct k_sem isr_sem;


static int hvm_get_parameter(int idx, uint64_t *value)
{
	struct xen_hvm_param xhv;

	xhv.domid = DOMID_SELF;
	xhv.index = idx;

	register unsigned int r0 __asm__("r0") = HVMOP_get_param;
	register size_t r1 __asm__("r1") = (size_t) &xhv;
	register unsigned int r16 __asm__("r16") = __HYPERVISOR_hvm_op;
	register int ret __asm__("r0");


	__asm__ volatile ("hvc #0xEA1" : "=r" (ret) : "0" (r0), "r" (r1), "r" (r16) : "memory");

	if (ret < 0)
		return ret;

	*value = xhv.value;
	return ret;
}

static int xen_map_shared_info (const struct shared_info *shared_page) {
	struct xen_add_to_physmap xatp;

	xatp.domid = DOMID_SELF;
	xatp.idx = 0;
	xatp.space = XENMAPSPACE_shared_info;
	xatp.gpfn = ( ((size_t) shared_page) >> 12);

	register unsigned int r0 __asm__("r0") = XENMEM_add_to_physmap;
	register size_t r1 __asm__("r1") = (size_t) &xatp;
	register unsigned int r16 __asm__("r16") = __HYPERVISOR_memory_op;
	register int ret __asm__("r0");

	__asm__ volatile ("hvc #0xEA1" : "=r" (ret) : "0" (r0), "r" (r1), "r" (r16) : "memory");


	printk("%s: mapping result = %d\n", __func__, ret);

	return ret;
}

static void notify_hv_evtchn(evtchn_port_t evtchn) {
	struct evtchn_send send = {.port = evtchn};

	register unsigned int r0 __asm__("r0") = EVTCHNOP_send;
	register size_t r1 __asm__("r1") = (size_t) &send;
	register unsigned int r16 __asm__("r16") = __HYPERVISOR_event_channel_op;
	register int ret __asm__("r0");

	__asm__ volatile ("hvc #0xEA1" : "=r" (ret) : "0" (r0), "r" (r1), "r" (r16) : "memory");
}

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
		notify_hv_evtchn(console_evtchn);
	return sent;
}


static void read_thread(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* __write console will trigger HV interrupt */
	char print[] = "First condvar irq print!\n";
	(void)__write_console(print, sizeof(print));

	printk("%s: taking condvar 1\n", __func__);
	compiler_barrier();

	k_mutex_lock(&xenstore_mutex, K_FOREVER);
	k_condvar_wait(&xenstore_condvar, &xenstore_mutex, K_FOREVER);
	k_mutex_unlock(&xenstore_mutex);
	printk("%s: received condvar 1\n", __func__);


	char print1[] = "Second condvar irq print!\n";
	(void)__write_console(print1, sizeof(print1));

	printk("%s: taking condvar 2\n", __func__);
	compiler_barrier();

	k_mutex_lock(&xenstore_mutex, K_FOREVER);
	k_condvar_wait(&xenstore_condvar, &xenstore_mutex, K_FOREVER);
	k_mutex_unlock(&xenstore_mutex);
	printk("%s: received condvar 2\n", __func__);

}

static void sanity_thread(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		printk("%s:>>>>>>>>>> running\n", __func__);
		k_sleep(K_SECONDS(3));
	}
}



static void signal_thread(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {

		/* receive semaphore from ISR, send signal to "read" thread */
		k_sem_take(&isr_sem, K_FOREVER);
		printk("%s: took a semaphore\n", __func__);

                k_mutex_lock(&xenstore_mutex, K_FOREVER);
                printk("%s: signaling condvar\n", __func__);
                k_condvar_signal(&xenstore_condvar);
                k_mutex_unlock(&xenstore_mutex);
	}
}

static void irq_routine(void *arg) {

//	if (shared_info_page->evtchn_pending[0] & (1 << xenstore_evtchn)) {
//		printk("%s: giving semaphore\n", __func__);
//		k_sem_give(&isr_sem);
////		printk("%s: gave semaphore\n", __func__);
//	}

//	printk("%s: giving semaphore\n", __func__);
	k_sem_give(&isr_sem);

	shared_info_page->vcpu_info[0].evtchn_upcall_pending = 0;
	shared_info_page->vcpu_info[0].evtchn_pending_sel = 0;
	shared_info_page->evtchn_pending[0] = 0;

//	printk("%s: ISR out\n", __func__);
}

static int xen_init(const struct device *dev) {
	ARG_UNUSED(dev);

	int ret = 0;
	uint64_t console_pfn = 0;

	shared_info_page = k_aligned_alloc(4096, 4096);
	if (!shared_info_page) {
		printk("%s: failed to allocate memory for Xen shared page!\n", __func__);
		return -ENOMEM;
	}

	ret = xen_map_shared_info(shared_info_page);
	if (ret) {
		printk("%s: failed to map for Xen shared page, ret = %d\n", __func__, ret);
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
	console_intf = (struct xencons_interface *) (console_pfn << 12);
	arch_mem_map(console_intf, (uintptr_t) console_intf, 4096, K_MEM_PERM_RW | K_MEM_CACHE_WB);
	printk("console virt after mapping = %p\n", console_intf);

	ret = hvm_get_parameter(HVM_PARAM_CONSOLE_EVTCHN, &console_evtchn);
	if (ret) {
		printk("%s: failed to get Xen console event channel, ret = %d\n", __func__, ret);
		return ret;
	}
	printk("%s: console eventchn = %llx\n", __func__, console_evtchn);

	char print[] = "First print the Zephyr PV console!\n";
	(void)__write_console(print, sizeof(print));





	/* ========================= Threads setup ======================================== */
	k_condvar_init(&xenstore_condvar);
	k_mutex_init(&xenstore_mutex);
	k_sem_init(&isr_sem, 0, 10);

	printk("%s: vars inited\n", __func__);

	k_thread_create(&signal_thrd, signal_thrd_stack,
			K_KERNEL_STACK_SIZEOF(signal_thrd_stack),
			signal_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);

	k_thread_name_set(&signal_thrd, "signal_thread");
	printk("%s: signal thread inited\n", __func__);


	k_thread_create(&read_thrd, read_thrd_stack,
				K_KERNEL_STACK_SIZEOF(read_thrd_stack),
				read_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);

	k_thread_name_set(&read_thrd, "read_thread");
	printk("%s: read thread inited, stack defined at %p\n", __func__, read_thrd_stack);


	k_thread_create(&sanity_thrd, sanity_thrd_stack,
			K_KERNEL_STACK_SIZEOF(sanity_thrd_stack),
			sanity_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_name_set(&sanity_thrd, "sanity_thread");

	IRQ_CONNECT(    DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq),
			DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, priority),
			irq_routine, NULL,
			DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, flags));

	irq_enable(DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq));

	printk("%s: irq inited\n", __func__);
	return 0;
}

SYS_INIT(xen_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


