/*
 * Copyright 2020 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <devicetree.h>
#include <sys/util.h>
#include <arch/arm64/arm_mmu.h>

static const struct arm_mmu_region mmu_regions[] = {

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 0),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 0),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_RW | MT_NS),

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 1),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 1),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_RW | MT_NS),

	MMU_REGION_FLAT_ENTRY("HYPERVISOR",
			      DT_REG_ADDR_BY_IDX(DT_INST(0, xen_xen), 0),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, xen_xen), 0),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_RW | MT_NS),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
