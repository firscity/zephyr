#include <arch/arm64/hypercall.h>
#include <xen/public/xen.h>
#include <xen/public/event_channel.h>
#include <xen/events.h>
#include <xen/hvm.h>

#include <kernel.h>
#include <errno.h>
#include <init.h>
#include <device.h>

K_KERNEL_STACK_DEFINE(test_thrd_1_stack, 1024);
struct k_thread test_thrd_1;
k_tid_t test_thrd_1_tid;

K_KERNEL_STACK_DEFINE(test_thrd_2_stack, 1024);
struct k_thread test_thrd_2;
k_tid_t test_thrd_2_tid;

struct k_sem event_sem;
struct k_sem thrd_sem;

extern shared_info_t *HYPERVISOR_shared_info;
uintptr_t enter_sp_addr = 0;
uintptr_t before_cs_sp_addr = 0;
uintptr_t after_cs_sp_addr = 0;

void event_test_isr(void *p) {
	vcpu_info_t *vcpu = &HYPERVISOR_shared_info->vcpu_info[0];
	vcpu->evtchn_upcall_pending = 0;
	vcpu->evtchn_pending_sel = 0;
	HYPERVISOR_shared_info->evtchn_pending[0] = 0;
	k_sem_give(&event_sem);
}

static void test_thrd_func_1(void *p1, void *p2, void *p3) {
	while (1) {
		printk("waiting for event semaphore in %s\n", __func__);
		k_sem_take(&event_sem, K_FOREVER);
		printk("took event semaphore in %s\n", __func__);
		k_sem_give(&thrd_sem);
	}
}

static void test_thrd_func_2(void *p1, void *p2, void *p3) {
	while (1) {
		printk("waiting for semaphore in %s\n", __func__);
		k_sem_take(&thrd_sem, K_FOREVER);
		printk("took semaphore in %s\n", __func__);
	}
}

void save_test_sp(uintptr_t sp) {
//	saved_sp_addr = sp;
}

int test_code_init(const struct device *dev) {
	ARG_UNUSED(dev);
	k_sem_init(&event_sem, 0, 1);
	k_sem_init(&thrd_sem, 0, 1);

//	ret = hvm_get_parameter(HVM_PARAM_CONSOLE_EVTCHN, &xs_evtchn);
//	if (ret) {
//		printk("%s: failed to get Xenbus evtchn, ret = %d\n",
//				__func__, ret);
//		return ret;
//	}
//
//	printk("Console evtchn = %llu\n", xs_evtchn);
	test_thrd_1_tid = k_thread_create(&test_thrd_1, test_thrd_1_stack,
			K_KERNEL_STACK_SIZEOF(test_thrd_1_stack),
			test_thrd_func_1, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	if (!test_thrd_1_tid) {
		printk("%s: Failed to create thread 1\n", __func__);
		return -1;
	}
	k_thread_name_set(test_thrd_1_tid, "thread_1");
	printk("%s: thread 1 inited\n", __func__);

	test_thrd_2_tid = k_thread_create(&test_thrd_2, test_thrd_2_stack,
			K_KERNEL_STACK_SIZEOF(test_thrd_2_stack),
			test_thrd_func_2, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	if (!test_thrd_2_tid) {
		printk("%s: Failed to create thread 2\n", __func__);
		return -1;
	}
	k_thread_name_set(test_thrd_2_tid, "thread_2");
	printk("%s: thread 2 inited\n", __func__);

//	bind_event_channel(xs_evtchn, event_test_isr, NULL);
	IRQ_CONNECT(DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq),
		DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, priority), event_test_isr,
		NULL, DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, flags));

	irq_enable(DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq));

	return 0;
}

SYS_INIT(test_code_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
