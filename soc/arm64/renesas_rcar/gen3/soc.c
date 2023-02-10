/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

/**
 *
 * @brief Perform basic hardware initialization
 *
 * @return 0
 */

static int soc_init(void)
{
	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
