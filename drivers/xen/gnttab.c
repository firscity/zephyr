/* SPDX-License-Identifier: MIT */
/*
 ****************************************************************************
 * (C) 2006 - Cambridge University
 * (C) 2021-2022 - EPAM Systems
 ****************************************************************************
 *
 *        File: gnttab.c
 *      Author: Steven Smith (sos22@cam.ac.uk)
 *     Changes: Grzegorz Milos (gm281@cam.ac.uk)
 *
 *        Date: July 2006
 *
 * Environment: Xen Minimal OS
 * Description: Simple grant tables implementation. About as stupid as it's
 *  possible to be and still work.
 *
 ****************************************************************************
 */
#include <zephyr/arch/arm64/hypercall.h>
#include <zephyr/xen/generic.h>
#include <zephyr/xen/gnttab.h>
#include <zephyr/xen/public/grant_table.h>
#include <zephyr/xen/public/memory.h>
#include <zephyr/xen/public/xen.h>
#include <zephyr/sys/barrier.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(xen_gnttab);

/* Timeout for grant table ops retrying */
#define GOP_RETRY_DELAY 200
#define DT_GNTTAB_SIZE		DT_REG_SIZE_BY_IDX(DT_INST(0, xen_xen), 0)
#define GNT_ENTRIES_PER_FRAME	(XEN_PAGE_SIZE / sizeof(grant_entry_v1_t))

/* Grant refs are unsigned integer and we need some value for invalid cases. Since
 * first GNTTAB_NR_RESERVED_ENTRIES are reserved we can use 0 for these purposes.
 */
#define GNTTAB_INVAL_GREF	0
#define GNTTAB_LAST_GREF	UINT32_MAX

BUILD_ASSERT(!(DT_GNTTAB_SIZE % XEN_PAGE_SIZE),
	     "Size of gnttab have to be aligned on XEN_PAGE_SIZE");
BUILD_ASSERT(DT_GNTTAB_SIZE <= CONFIG_KERNEL_VM_SIZE);

DEVICE_MMIO_TOPLEVEL_STATIC(grant_tables, DT_INST(0, xen_xen));

static struct gnttab {
	struct k_mutex lock;
	unsigned long nr_grant_frames;
	unsigned long max_grant_frames;
	grant_entry_v1_t *table;
	grant_ref_t *gref_list;
} gnttab;

static int extend_gnttab(void);

static grant_ref_t get_grant_entry(void)
{
	int rc;
	grant_ref_t gref = GNTTAB_INVAL_GREF;

	k_mutex_lock(&gnttab.lock, K_FOREVER);
	if (gnttab.gref_list[0] == GNTTAB_LAST_GREF) {
		/* Map one more frame if possible, need to hold mutex */
		rc = extend_gnttab();
		if (rc) {
			k_mutex_unlock(&gnttab.lock);
			LOG_WRN("Failed to extend gnttab rc = %d, waiting for free gref", rc);
			return gref;
		}
	}

	gref = gnttab.gref_list[0];
	gnttab.gref_list[0] = gnttab.gref_list[gref];
	k_mutex_unlock(&gnttab.lock);

	return gref;
}

static void put_grant_entry(grant_ref_t gref)
{
	k_mutex_lock(&gnttab.lock, K_FOREVER);
	gnttab.gref_list[gref] = gnttab.gref_list[0];
	gnttab.gref_list[0] = gref;
	k_mutex_unlock(&gnttab.lock);
}

static void gnttab_grant_permit_access(grant_ref_t gref, domid_t domid,
		unsigned long gfn, bool readonly)
{
	uint16_t flags = GTF_permit_access;

	if (readonly) {
		flags |= GTF_readonly;
	}

	gnttab.table[gref].frame = gfn;
	gnttab.table[gref].domid = domid;
	/* Need to be sure that gfn and domid will be set before flags */
	barrier_dmem_fence_full();

	gnttab.table[gref].flags = flags;
}

grant_ref_t gnttab_grant_access(domid_t domid, unsigned long gfn,
		bool readonly)
{
	grant_ref_t gref = get_grant_entry();

	gnttab_grant_permit_access(gref, domid, gfn, readonly);

	return gref;
}

/* Reset flags to zero in order to stop using the grant */
static int gnttab_reset_flags(grant_ref_t gref)
{
	uint16_t flags, nflags;
	uint16_t *pflags;

	pflags = &gnttab.table[gref].flags;
	nflags = *pflags;

	do {
		flags = nflags;
		if (flags & (GTF_reading | GTF_writing)) {
			LOG_WRN("gref = %u still in use! (0x%x)\n",
				gref, flags);
			return 1;
		}
		nflags = synch_cmpxchg(pflags, flags, 0);
	} while (nflags != flags);

	return 0;
}

int gnttab_end_access(grant_ref_t gref)
{
	int rc;

	__ASSERT((gref >= GNTTAB_NR_RESERVED_ENTRIES) &&
		 (gref < gnttab.nr_grant_frames * GNT_ENTRIES_PER_FRAME),
		 "Invalid gref = %d", gref);

	rc = gnttab_reset_flags(gref);
	if (!rc) {
		return rc;
	}

	put_grant_entry(gref);

	return 0;
}

int32_t gnttab_alloc_and_grant(void **map, bool readonly)
{
	void *page;
	unsigned long gfn;
	grant_ref_t gref;

	__ASSERT_NO_MSG(map != NULL);

	page = k_aligned_alloc(XEN_PAGE_SIZE, XEN_PAGE_SIZE);
	if (page == NULL) {
		return -ENOMEM;
	}

	gfn = xen_virt_to_gfn(page);
	gref = gnttab_grant_access(0, gfn, readonly);

	*map = page;

	return gref;
}

static void gop_eagain_retry(int cmd, struct gnttab_map_grant_ref *gref)
{
	unsigned int step = 10, delay = step;
	int16_t *status = &gref->status;

	do {
		HYPERVISOR_grant_table_op(cmd, gref, 1);
		if (*status == GNTST_eagain) {
			k_sleep(K_MSEC(delay));
		}

		delay += step;
	} while ((*status == GNTST_eagain) && (delay < GOP_RETRY_DELAY));

	if (delay >= GOP_RETRY_DELAY) {
		LOG_ERR("Failed to map grant, timeout reached\n");
		*status = GNTST_bad_page;
	}
}

void *gnttab_get_page(void)
{
	int ret;
	void *page_addr;
	struct xen_remove_from_physmap rfpm;

	page_addr = k_aligned_alloc(XEN_PAGE_SIZE, XEN_PAGE_SIZE);
	if (!page_addr) {
		LOG_WRN("Failed to allocate memory for gnttab page!\n");
		return NULL;
	}

	rfpm.domid = DOMID_SELF;
	rfpm.gpfn = xen_virt_to_gfn(page_addr);

	/*
	 * GNTTABOP_map_grant_ref will simply replace the entry in the P2M
	 * and not release any RAM that may have been associated with
	 * page_addr, so we release this memory before mapping.
	 */
	ret = HYPERVISOR_memory_op(XENMEM_remove_from_physmap, &rfpm);
	if (ret) {
		LOG_WRN("Failed to remove gnttab page from physmap, ret = %d\n", ret);
		return NULL;
	}

	return page_addr;
}

void gnttab_put_page(void *page_addr)
{
	int ret, nr_extents = 1;
	struct xen_memory_reservation reservation;
	xen_pfn_t page = xen_virt_to_gfn(page_addr);

	/*
	 * After unmapping there will be a 4Kb holes in address space
	 * at 'page_addr' positions. To keep it contiguous and be able
	 * to return such addresses to memory allocator we need to
	 * populate memory on unmapped positions here.
	 */
	memset(&reservation, 0, sizeof(reservation));
	reservation.domid = DOMID_SELF;
	reservation.extent_order = 0;
	reservation.nr_extents = nr_extents;
	set_xen_guest_handle(reservation.extent_start, &page);

	ret = HYPERVISOR_memory_op(XENMEM_populate_physmap, &reservation);
	if (ret != nr_extents) {
		LOG_WRN("failed to populate physmap on gfn = 0x%llx, ret = %d\n",
			page, ret);
		return;
	}

	k_free(page_addr);
}

int gnttab_map_refs(struct gnttab_map_grant_ref *map_ops, unsigned int count)
{
	int i, ret;

	ret = HYPERVISOR_grant_table_op(GNTTABOP_map_grant_ref, map_ops, count);
	if (ret) {
		return ret;
	}

	for (i = 0; i < count; i++) {
		switch (map_ops[i].status) {
		case GNTST_no_device_space:
			LOG_WRN("map_grant_ref failed, no device space for page #%d\n", i);
			break;

		case GNTST_eagain:
			/* Operation not done; need to try again */
			gop_eagain_retry(GNTTABOP_map_grant_ref, &map_ops[i]);
			/* Need to re-check status for current page */
			i--;

			break;

		default:
			break;
		}
	}

	return 0;
}

int gnttab_unmap_refs(struct gnttab_map_grant_ref *unmap_ops, unsigned int count)
{
	return HYPERVISOR_grant_table_op(GNTTABOP_unmap_grant_ref, unmap_ops, count);
}


static const char * const gnttab_error_msgs[] = GNTTABOP_error_msgs;

const char *gnttabop_error(int16_t status)
{
	status = -status;
	if (status < 0 || (uint16_t) status >= ARRAY_SIZE(gnttab_error_msgs)) {
		return "bad status";
	} else {
		return gnttab_error_msgs[status];
	}
}

static int setup_grant_table(unsigned long nr_frames)
{
	int rc;
	struct gnttab_setup_table setup;
	xen_pfn_t *frames;

	frames = k_calloc(gnttab.nr_grant_frames, sizeof(*frames));
	if (!frames) {
		LOG_ERR("Failed to allocate memory for frames");
		return -ENOMEM;
	}

	setup.dom = DOMID_SELF;
	setup.nr_frames = gnttab.nr_grant_frames;
	set_xen_guest_handle(setup.frame_list, frames);
	rc = HYPERVISOR_grant_table_op(GNTTABOP_setup_table, &setup, 1);
	if (rc || setup.status) {
		LOG_ERR("Table setup failed; status = %s", gnttabop_error(setup.status));
		if (!rc) {
			/* Xen may return 0 with negative setup status, set it as call result */
			rc = setup.status;
		}
	}
	k_free(frames);

	return rc;
}

static int map_grant_frame(unsigned int start_frame)
{
	int rc;
	struct xen_add_to_physmap xatp;

	if (gnttab.nr_grant_frames == gnttab.max_grant_frames) {
		LOG_ERR("Reached max number of Xen grant frames");
		return -ENOMEM;
	}

	/* Stage 2 frame mapping */
	xatp.domid = DOMID_SELF;
	xatp.size = 0;
	xatp.space = XENMAPSPACE_grant_table;
	xatp.idx = start_frame;
	xatp.gpfn = xen_virt_to_gfn(Z_TOPLEVEL_ROM_NAME(grant_tables).phys_addr) + start_frame;
	rc = HYPERVISOR_memory_op(XENMEM_add_to_physmap, &xatp);
	if (rc) {
		LOG_ERR("add_to_physmap failed; status = %d\n", rc);
		return rc;
	}

	gnttab.nr_grant_frames++;

	return setup_grant_table(gnttab.nr_grant_frames);
}

static int extend_gnttab(void)
{
	int rc;
	grant_ref_t iter, start_gref, end_gref;
	grant_ref_t *old_list = gnttab.gref_list;
	unsigned long start = gnttab.nr_grant_frames;
	bool is_first_map = !gnttab.nr_grant_frames;
	size_t new_size, old_size = gnttab.nr_grant_frames * GNT_ENTRIES_PER_FRAME;

	if (gnttab.nr_grant_frames == gnttab.max_grant_frames) {
		LOG_ERR("Reached limit of Xen grant frames!");
		return -ENOSPC;
	}

	rc = map_grant_frame(start);
	if (rc) {
		/* Nothing to do here, left previous part of gnttab as is */
		return rc;
	}

	/* gnttab.nr_grant_frames will be updated after success map */
	new_size = gnttab.nr_grant_frames * GNT_ENTRIES_PER_FRAME;

	/* Since Zephyr does not have realloc, need to do it manually */
	gnttab.gref_list = k_calloc(new_size, sizeof(grant_ref_t));
	if (!gnttab.gref_list) {
		gnttab.gref_list = old_list;
		return -ENOMEM;
	}

	if (!is_first_map) {
		memcpy(gnttab.gref_list, old_list, old_size * sizeof(grant_ref_t));
		k_free(old_list);

		start_gref = old_size - 1;
	} else {
		start_gref = GNTTAB_NR_RESERVED_ENTRIES;
	}
	end_gref = new_size - 1;

	/* Overwrite old GNTTAB_LAST_GREF with new frame start gref */
	gnttab.gref_list[start_gref] = old_size;
	for (iter = end_gref; iter > start_gref; iter--) {
		gnttab.gref_list[iter] = gnttab.gref_list[0];
		gnttab.gref_list[0] = iter;
	}
	gnttab.gref_list[end_gref] = GNTTAB_LAST_GREF;

	return 0;
}

/* Picked from Linux implementation */
#define LEGACY_MAX_GNT_FRAMES_SUPPORTED		4
static unsigned long gnttab_get_max_frames(void)
{
	int ret;
	struct gnttab_query_size q = {
		.dom = DOMID_SELF,
	};

	ret = HYPERVISOR_grant_table_op(GNTTABOP_query_size, &q, 1);
	if ((ret < 0) || (q.status != GNTST_okay)) {
		return LEGACY_MAX_GNT_FRAMES_SUPPORTED;
	}

	return q.max_nr_frames;
}

static int gnttab_init(void)
{
	int rc;

	k_mutex_init(&gnttab.lock);
	gnttab.nr_grant_frames = 0;
	/* We need to know Xen limitations for domain */
	gnttab.max_grant_frames = gnttab_get_max_frames();

	/* initial mapping of a single gnttab frame, other will be mapped on demand */
	rc = extend_gnttab();
	if (rc) {
		LOG_ERR("Failed to init grant table frames, err = %d", rc);
		return rc;
	}

	/*
	 * Here we are doing Stage 1 mapping of whole DT region for grant tables.
	 * It may be much bigger, than actually mapped number of frames and may cause
	 * exception when someone try to access Stage 2 unmapped area, but since access
	 * is managed via get/put_grant_entry that can expand Stage 2 mapping,
	 * we do not need to care about it.
	 */
	DEVICE_MMIO_TOPLEVEL_MAP(grant_tables, K_MEM_CACHE_WB | K_MEM_PERM_RW);
	gnttab.table = (grant_entry_v1_t *)DEVICE_MMIO_TOPLEVEL_GET(grant_tables);

	LOG_DBG("%s: grant table mapped\n", __func__);

	return 0;
}

SYS_INIT(gnttab_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
