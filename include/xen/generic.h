/*
 * Copyright (c) 2021 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __XEN_GENERIC_H__
#define __XEN_GENERIC_H__

//#ifndef __XEN_INTERFACE_VERSION__

//#endif
#include <xen/public/xen.h>
/* Watch event queue */
struct xenbus_event {
    /* Keep these two as this for xs.c */
    char *path;
    char *token;
    struct xenbus_event *next;
};
typedef struct xenbus_event *xenbus_event_queue;

//xenbus_event_queue xenbus_events;
/*static */struct watch {
    char *token;
    char *path;
    xenbus_event_queue *events;
    struct watch *next;
};
//struct watch *watches;
struct xenbus_req_info
{
    int in_use:1;
//    struct wait_queue_head waitq;
    void *reply;
};

struct write_req {
    const void *data;
    unsigned len;
};


#define NR_REQS 32

typedef unsigned long xenbus_transaction_t;
#define XBT_NIL ((xenbus_transaction_t)0)

#endif /* __XEN_GENERIC_H__ */
