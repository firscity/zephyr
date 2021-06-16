/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm64/hypercall.h>
#include <xen/public/xen.h>
#include <xen/public/event_channel.h>
#include <xen/events.h>

#include <kernel.h>
#include <errno.h>

static evtchn_handle_t event_channels[EVTCHN_2L_NR_CHANNELS];

/* One bit per port, sets to 1 during binding */
static uint64_t evtchn_states[EVTCHN_2L_NR_CHANNELS / (8 * sizeof(uint64_t))];

static void empty_callback(void *data) {
	;
}

void notify_evtchn(evtchn_port_t port) {
	struct evtchn_send send;

	if (port >= EVTCHN_2L_NR_CHANNELS) {
		printk("%s: trying to send notify for invalid evtchn #%u",
				__func__, port);
		return;
	}

	send.port = port;

	HYPERVISOR_event_channel_op(EVTCHNOP_send, &send);
}

int bind_event_channel(evtchn_port_t port, evtchn_cb_t cb, void *data) {
	if (port >= EVTCHN_2L_NR_CHANNELS) {
		printk("%s: trying to bind invalid evtchn #%u", __func__, port);
		return -EINVAL;
	}

	if (!cb) {
		printk("%s: NULL callback for evtchn #%u", __func__, port);
		return -EINVAL;
	}

	if (event_channels[port].cb != empty_callback)
		printk("%s: re-bind callback for evtchn #%u", __func__, port);

	event_channels[port].priv = data;
	event_channels[port].cb = cb;

	return 0;
}

int unbind_event_channel(evtchn_port_t port) {
	if (port >= EVTCHN_2L_NR_CHANNELS) {
		printk("%s: trying to unbind invalid evtchn #%u",
				__func__, port);
		return -EINVAL;
	}

	event_channels[port].cb = empty_callback;
	event_channels[port].priv = NULL;

	return 0;
}

int mask_event_channel(evtchn_port_t port) {

}

int unmask_event_channel(evtchn_port_t port) {

}

static void events_isr(void *data) {

}



int xen_events_init(void) {
	int i, ret = 0;

	/* bind all ports with default callback */
	for (i = 0; i < EVTCHN_2L_NR_CHANNELS; i++) {
		event_channels[i].cb = empty_callback;
		event_channels[i].priv = NULL;
	}

	IRQ_CONNECT(DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq),
		DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, priority), events_isr,
		NULL, DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, flags));

	irq_enable(DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq));

	printk("%s: irq inited\n", __func__);
	return 0;
}
