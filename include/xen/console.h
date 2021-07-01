/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __XEN_CONSOLE_H__
#define __XEN_CONSOLE_H__
#include <init.h>
#include <device.h>
#include <drivers/uart.h>

#ifdef CONFIG_XEN_EARLY_CONSOLEIO
extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));
#endif /* CONFIG_XEN_EARLY_CONSOLEIO */

struct hvc_xen_data {
	const struct device *dev;
	struct xencons_interface *intf;
	uint64_t evtchn;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

int xen_console_init(const struct device *dev);

#endif /* __XEN_CONSOLE_H__ */
