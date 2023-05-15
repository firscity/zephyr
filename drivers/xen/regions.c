/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel_arch_interface.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arch_interface.h>
#include <zephyr/sys/mem_blocks.h>
#include <zephyr/xen/generic.h>

LOG_MODULE_REGISTER(xen_regions);

#define EXTENDED_REGIONS_IDX 1
#define XEN_HYP_NODE DT_INST(0, xen_xen)

static K_MUTEX_DEFINE(ext_regions_lock);

struct extended_regions {
	sys_mem_blocks_t *block;
	uint64_t phys_addr;
	uint64_t size;
};

/*
 * At least one extended region should be provided.
 * If there is no extended regions - alternative allocation should be implemented
 * by the driver user and driver should be disabled.
 */
BUILD_ASSERT(DT_NUM_REGS(XEN_HYP_NODE) > EXTENDED_REGIONS_IDX,
	     "No Extended regions were provided in DT");

#define NOT_OVERLAPPING_BOUNDARIES(x_start, x_end, x_size, y_start, y_end, y_size) \
	MAX(x_end, y_end) - MIN(x_start, y_start) >= x_size + y_size

#define NOT_OVERLAPPING(x_start, x_size, y_start, y_size)		\
	NOT_OVERLAPPING_BOUNDARIES((uint64_t)x_start,			\
				  (uint64_t)x_start + (uint64_t)x_size, \
				  (uint64_t)x_size,			\
				  (uint64_t)y_start,			\
				  (uint64_t)y_start + (uint64_t)y_size, \
				  (uint64_t)y_size)

/*
 * Add BUILD_ASSERT that rejects extended region, starting from 0x0.
 * This is done because mem_blocks is not designed to handle buffer
 * starting from 0 and has assert with check. In this case starting address
 * should be shifted to page 1. That shouldn't be a problem because
 * minimal extended region size is set to 64MB in XEN.
 * For example for the extended region 0->0x30000000 the following
 * regs should be provided:
 * regs = <.... 0x0 0x1000 0x0 0x2FFFF000 ... >;
 *
 * Check reg addr and reg size to be aligned to
 * CONFIG_MMU_PAGE_SIZE. This is the requirement for arch_mem_map call.
 *
 * Extended regions should not overlap kernel Virtual Memory region.
 */
#define EXT_BLOCK(node_id, idx)						\
	BUILD_ASSERT(DT_REG_ADDR_BY_IDX(XEN_HYP_NODE, idx),		\
		     "Extended region starting from 0 is not supported, please skip page 0"); \
	BUILD_ASSERT(DT_REG_SIZE_BY_IDX(XEN_HYP_NODE, idx) ==		\
		     ROUND_DOWN(DT_REG_SIZE_BY_IDX(XEN_HYP_NODE, idx), CONFIG_MMU_PAGE_SIZE), \
		     "Extended region size is not aligned");		\
	BUILD_ASSERT(DT_REG_ADDR_BY_IDX(XEN_HYP_NODE, idx) ==		\
		     ROUND_DOWN(DT_REG_ADDR_BY_IDX(XEN_HYP_NODE, idx), CONFIG_MMU_PAGE_SIZE), \
		     "Extended region address is not aligned");		\
	BUILD_ASSERT(NOT_OVERLAPPING(DT_REG_ADDR_BY_IDX(XEN_HYP_NODE, idx), \
				     DT_REG_SIZE_BY_IDX(XEN_HYP_NODE, idx), \
				     CONFIG_KERNEL_VM_BASE,		\
				     CONFIG_KERNEL_VM_SIZE),		\
		     "Extended region should not overlap virtual ram");	\
	SYS_MEM_BLOCKS_DEFINE_STATIC_WITH_EXT_BUF(			\
		region_##idx, XEN_PAGE_SIZE,				\
		XEN_PFN_DOWN(DT_REG_SIZE_BY_IDX(XEN_HYP_NODE, idx)),	\
		(uint8_t *)DT_REG_ADDR_BY_IDX(XEN_HYP_NODE, idx));

DT_FOREACH_REG(XEN_HYP_NODE, EXT_BLOCK);

#define EXT_REG_POINTER(node_id, idx)					\
	{								\
		.block = &region_##idx,					\
		.phys_addr = DT_REG_ADDR_BY_IDX(XEN_HYP_NODE, idx),	\
		.size = DT_REG_SIZE_BY_IDX(XEN_HYP_NODE, idx),		\
	},

/*
 * TODO: For now we allocate buffer structures for DT_NUM_REGS, when we
 * need a range 1...DT_FOREACH_REG. Although region 0 is ignored, it still
 * takes space for sys_block structures allocation. This requires optimization
 */
static struct extended_regions ext_blocks[DT_NUM_REGS(XEN_HYP_NODE)] = {
	DT_FOREACH_REG(XEN_HYP_NODE, EXT_REG_POINTER)
};

void *xen_region_get_pages(size_t nr_pages)
{
	int ret, i;
	void *ptr;

	/*
	 * Locking ranges iteration, although each call sys_mem_blocks_alloc_contiguous
	 * has it's own syncronisation, to avoid the situation of 2 parralel calls of
	 * get_page and put_page on full buffers when put_page clears space in block 1
	 * when get_page has already checked block 1. This will lead us to -ENOMEM while
	 * block 1 space was freed already
	 */
	k_mutex_lock(&ext_regions_lock, K_FOREVER);

	for (i = EXTENDED_REGIONS_IDX; i < DT_NUM_REGS(XEN_HYP_NODE); i++) {
		ret = sys_mem_blocks_alloc_contiguous(ext_blocks[i].block, nr_pages, &ptr);
		if (ret == 0) {
			break;
		} else if (ret && ret != -ENOMEM) {
			LOG_ERR("Unable to allocate pages, ret = %d", ret);
			ptr = NULL;
		}
	}

	k_mutex_unlock(&ext_regions_lock);

	return ptr;
}

static int addr_is_in_buffer(uint8_t *ptr, sys_mem_blocks_t *block)
{
	/* Compare ptr to buffer address to determine if it was allocated in this block */
	return (ptr >= block->buffer) &&
		(ptr < block->buffer + (block->info.num_blocks << block->info.blk_sz_shift));
}

void xen_region_put_pages(void *ptr, size_t nr_pages)
{
	int ret, i;
	sys_mem_blocks_t *block = NULL;

	k_mutex_lock(&ext_regions_lock, K_FOREVER);

	for (i = EXTENDED_REGIONS_IDX; i < DT_NUM_REGS(XEN_HYP_NODE); i++) {
		if (addr_is_in_buffer(ptr, ext_blocks[i].block)) {
			block = ext_blocks[i].block;
			break;
		}
	}

	if (!block) {
		LOG_ERR("Unable to find buffer, matching to addr %p", ptr);
		goto unlock;
	}

	ret = sys_mem_blocks_free_contiguous(block, ptr, nr_pages);
	if (ret) {
		LOG_ERR("Unable to free pages, ret = %d", ret);
		goto unlock;
	}

unlock:
	k_mutex_unlock(&ext_regions_lock);
}

static int xen_regions_init(const struct device *d)
{
	int i;

	ARG_UNUSED(d);

	LOG_DBG("Initializing Xen Extended regions driver");

	for (i = EXTENDED_REGIONS_IDX; i < DT_NUM_REGS(XEN_HYP_NODE); i++) {
		/*
		 * Using arch_mem_map to map the extended regions as 1:1.
		 * The main idea of this functionality is to avoid using heap
		 * and use 'unused memory' provided by XEN as extended_regions.
		 * If using device_map call to map the space - then there is no
		 * difference comparing to heap - we will get vaddr.
		 * The idea is to allocate memory for the foreign mapping outside
		 * of the region Z_VIRT_RAM_START..Z_VIRT_RAM_END.
		 * phys_addr and size variables must be aligned to
		 * CONFIG_MMU_PAGE_SIZE.
		 */
		arch_mem_map((void *)ext_blocks[i].phys_addr, ext_blocks[i].phys_addr,
			     ext_blocks[i].size, K_MEM_CACHE_WB | K_MEM_PERM_RW);
	}

	return 0;
}

SYS_INIT(xen_regions_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
