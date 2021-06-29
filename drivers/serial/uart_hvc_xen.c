/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm64/hypercall.h>
#include <xen/console.h>
#include <xen/events.h>
#include <xen/generic.h>
#include <xen/hvm.h>
#include <xen/public/io/console.h>
#include <xen/public/io/ring.h>
#include <xen/public/io/xenbus.h>
#include <xen/public/io/xs_wire.h>
#include <xen/public/sched.h>
#include <xen/public/xen.h>

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <kernel_arch_interface.h>
#include <stdio.h>
#include <string.h>
#include <sys/printk.h>

#include <drivers/uart.h>



static struct xencons_interface *console_intf = NULL;
static uint64_t console_evtchn = 0;

static int __read_from_ring(char *data, int len)
{
	int recv = 0;
	XENCONS_RING_IDX cons = console_intf->in_cons;
	XENCONS_RING_IDX prod = console_intf->in_prod;
	XENCONS_RING_IDX in_idx = 0;

	compiler_barrier();
	__ASSERT((prod - cons) > sizeof(console_intf->in),
			"Invalid input buffer");

	while (cons != prod && recv < len) {
		in_idx = MASK_XENCONS_IDX(cons, console_intf->in);
		data[recv] = console_intf->in[in_idx];
		recv++;
		cons++;
	}

	compiler_barrier();
	console_intf->in_cons = cons;

	notify_evtchn(console_evtchn);
	return recv;
}

static int __write_to_ring(const char *data, int len)
{
	int sent = 0;
	XENCONS_RING_IDX cons = console_intf->out_cons;
	XENCONS_RING_IDX prod = console_intf->out_prod;
	XENCONS_RING_IDX out_idx = 0;

	compiler_barrier();
	__ASSERT((prod - cons) > sizeof(console_intf->out),
			"Invalid output buffer");

	while ((sent < len) && ((prod - cons) < sizeof(console_intf->out))) {
		out_idx = MASK_XENCONS_IDX(prod, console_intf->out);
		console_intf->out[out_idx] = data[sent];
		prod++;
		sent++;
	}

	compiler_barrier();			/* write ring before updating pointer */
	console_intf->out_prod = prod;

	if (sent)
		notify_evtchn(console_evtchn);

	return sent;
}

static int xen_hvc_poll_in(const struct device *dev,
			unsigned char *c) {
	/* TODO: definitely this is bad solution - notifying HV every time */
	(void) __read_from_ring(c, sizeof(*c));

	return 0;
}

static void xen_hvc_poll_out(const struct device *dev,
			unsigned char c) {
	/* TODO: definitely this is bad solution - notifying HV every time */
	(void) __write_to_ring(&c, sizeof(c));
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int xen_hvc_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len) {
	int ret = 0, sent = 0;

	while (len) {
		sent = __write_to_ring(tx_data, len);

		ret += sent;
		tx_data += sent;
		len -= sent;

		if (len) {
			/* Need to be able to read it from another domain */
			HYPERVISOR_sched_op(SCHEDOP_yield, NULL);
		}
	}

	return ret;
}

static int xen_hvc_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size) {
	return __read_from_ring(rx_data, size);
}

static void xen_hvc_irq_tx_enable(const struct device *dev) {

}

static void xen_hvc_irq_tx_disable(const struct device *dev) {

}

static int xen_hvc_irq_tx_ready(const struct device *dev) {
	return 0;
}

static void xen_hvc_irq_rx_enable(const struct device *dev) {

}

static void xen_hvc_irq_rx_disable(const struct device *dev) {

}

static int xen_hvc_irq_tx_complete(const struct device *dev) {
	return 0;
}

static int xen_hvc_irq_rx_ready(const struct device *dev){
	return 0;
}

static void xen_hvc_irq_err_enable(const struct device *dev) {

}

static void xen_hvc_irq_err_disable(const struct device *dev) {

}

static int xen_hvc_irq_is_pending(const struct device *dev) {
	return 0;
}

static int xen_hvc_irq_update(const struct device *dev) {
	return 0;
}

static void xen_hvc_irq_callback_set(const struct device *dev,
		 uart_irq_callback_user_data_t cb,
		 void *user_data) {

}
#endif

static const struct uart_driver_api xen_hvc_api = {
	.poll_in = xen_hvc_poll_in,
	.poll_out = xen_hvc_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = xen_hvc_fifo_fill,
	.fifo_read = xen_hvc_fifo_read,
	.irq_tx_enable = xen_hvc_irq_tx_enable,
	.irq_tx_disable = xen_hvc_irq_tx_disable,
	.irq_tx_ready = xen_hvc_irq_tx_ready,
	.irq_rx_enable = xen_hvc_irq_rx_enable,
	.irq_rx_disable = xen_hvc_irq_rx_disable,
	.irq_tx_complete = xen_hvc_irq_tx_complete,
	.irq_rx_ready = xen_hvc_irq_rx_ready,
	.irq_err_enable = xen_hvc_irq_err_enable,
	.irq_err_disable = xen_hvc_irq_err_disable,
	.irq_is_pending = xen_hvc_irq_is_pending,
	.irq_update = xen_hvc_irq_update,
	.irq_callback_set = xen_hvc_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};



int xen_console_init(const struct device *dev) {
	ARG_UNUSED(dev);
	int ret = 0;
	uint64_t console_pfn = 0;
	char buf[100];


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

	snprintf(buf, sizeof(buf), "First print the Zephyr PV console!\n");
	(void)__write_to_ring(buf, strlen(buf));

	snprintf(buf, sizeof(buf), "Console evtchn = %lld\n", console_evtchn);
	(void)__write_to_ring(buf, strlen(buf));

	return 0;
}


DEVICE_DEFINE(uart_hvc_xen, "XEN_HVC", xen_console_init, NULL, NULL, NULL,
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &xen_hvc_api);
