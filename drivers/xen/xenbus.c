/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <xen/xen.h>
#include <xen/hvm/hvm_op.h>
#include <xen/hvm/params.h>
#include <xen/memory.h>
#include <xen/version.h>
#include <xen/io/console.h>
#include <xen/io/xs_wire.h>
#include <xen/io/xenbus.h>
#include <arch/arm64/hypercall.h>

#include <string.h>
#include <stdio.h>
#include <kernel.h>
#include <init.h>
#include <errno.h>
#include <sys/printk.h>
#include <device.h>
#include <kernel_arch_interface.h>
#include <kernel/thread.h>

static int xenbus_init(const struct device *dev) {
	ARG_UNUSED(dev);
	int ret = 0;



	return ret;
}
