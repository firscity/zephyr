/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __XEN_CONSOLE_H__
#define __XEN_CONSOLE_H__
#include <init.h>
#include <device.h>

extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));

int xen_console_init(const struct device *dev);

#endif /* __XEN_CONSOLE_H__ */
