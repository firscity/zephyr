/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __XEN_REGIONS_H__
#define __XEN_REGIONS_H__

/**
 * @brief Allocate pages from the extended regions
 *
 * @param nr_pages - number of pages to be allocated
 * @return - pointer to the allocated pages
 */
void *xen_region_get_pages(size_t nr_pages);

/**
 * @brief Free pages on extended regions
 * @param ptr - pointer to the pages, allocated by xen_region_get_pages call
 * @param nr_pages - number of pages that were allocated
 */
void xen_region_put_pages(void *ptr, size_t nr_pages);

/**
 * @brief Map extended region memory
 * @param ptr - pointer to the pages, allocated by xen_region_get_pages call
 * @param nr_pages - number of pages that should be mapped
 */
void xen_region_map(void *ptr, size_t nr_pages);

/**
 * @brief Unmap extended region memory
 * @param ptr - pointer to the pages, allocated by xen_region_get_pages call
 * @param nr_pages - number of pages that should be unmapped
 */
void xen_region_unmap(void *ptr, size_t nr_pages);

#endif /* __XEN_REGIONS_H__ */
