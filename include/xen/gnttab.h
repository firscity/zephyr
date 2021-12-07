/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __XEN_GNTTAB_H__
#define __XEN_GNTTAB_H__

#include <xen/public/grant_table.h>

grant_ref_t gnttab_grant_access(domid_t domid, unsigned long mfn,
		int readonly);
grant_ref_t gnttab_grant_transfer(domid_t domid, unsigned long mfn);
int gnttab_update_grant(grant_ref_t gref, domid_t domid,
		unsigned long mfn, int readonly);
int gnttab_end_access(grant_ref_t gref);
unsigned long gnttab_end_transfer(grant_ref_t gref);

/*
 * Allocates 4K page for grant and share it via returned
 * gref. Need to k_free memory, which was allocated in
 * @map parameter after grant releasing.
 */
grant_ref_t gnttab_alloc_and_grant(void **map);
int gnttab_map_refs(struct gnttab_map_grant_ref *map_ops, unsigned int count);
int gnttab_unmap_refs(struct gnttab_map_grant_ref *unmap_ops, unsigned int count);
const char *gnttabop_error(int16_t status);

#endif /* __XEN_GNTTAB_H__ */
