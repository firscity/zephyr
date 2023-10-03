/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <libfdt.h>

extern uint8_t xen_fdt[];

int main(void)
{
	int chosen_offset, lenp;
	const void *fdt = xen_fdt;
	const struct fdt_property *bootargs;

	printk("Saved Xen device tree magic is 0x%x\n",
	       sys_be32_to_cpu(*((uint32_t *) xen_fdt)));

	chosen_offset = fdt_path_offset(fdt, "/chosen");
	if (chosen_offset < 0) {
		printk("No chosen available\n");
		return chosen_offset;
	}

	bootargs = fdt_get_property(fdt, chosen_offset, "bootargs", &lenp);
	if (!bootargs) {
		printk("No bootargs available!\n");
		return -1;
	}

	printk("Zephyr bootargs, received from Xen - \"%s\" \n", bootargs->data);

	return 0;
}
