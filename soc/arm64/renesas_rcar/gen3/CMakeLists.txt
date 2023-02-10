# Copyright (c) 2023 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(
  soc.c
)

zephyr_library_sources_ifdef(CONFIG_SOC_R8A7795  pfc_r8a7795.c)
zephyr_library_sources_ifdef(CONFIG_ARM_MMU      mmu_regions.c)
