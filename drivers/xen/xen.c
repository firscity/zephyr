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

#define min(x,y) (                       \
        (x) < (y) ? (x) : (y)            \
        )

static struct xencons_interface *console_intf = NULL;
static uint64_t console_evtchn = 0, xenstore_evtchn = 0;
static struct xenstore_domain_interface *xenstore_intf = NULL;
struct shared_info *shared_info_page = NULL;
struct k_sem isr_sem;

K_KERNEL_STACK_DEFINE(xenstore_thrd_stack, 1024);
struct k_thread xenstore_thrd;

K_KERNEL_STACK_DEFINE(read_thrd_stack, 1024);
struct k_thread read_thrd;

K_KERNEL_STACK_DEFINE(sanity_thrd_stack, 1024);
struct k_thread sanity_thrd;

struct k_condvar xenstore_condvar;
struct k_mutex xenstore_mutex;
struct k_sem thread1_sem;
struct k_sem thread2_sem;

#define DEBUG_XENBUS

#ifdef DEBUG_XENBUS
#define xenbus_printk printk
#else
static void debug(const char *fmt, ...) {};
#define xenbus_printk debug
#endif
//static DECLARE_WAIT_QUEUE_HEAD(xb_waitq);
//DECLARE_WAIT_QUEUE_HEAD(xenbus_watch_queue);
//
static struct xenbus_req_info req_info;

static int hvm_get_parameter(int idx, uint64_t *value)
{
	int ret = 0;
	struct xen_hvm_param xhv;

	xhv.domid = DOMID_SELF;
	xhv.index = idx;

	ret = HYPERVISOR_hvm_op(HVMOP_get_param, &xhv);
	if (ret < 0)
		return ret;

	*value = xhv.value;
	return ret;
}

static int xen_map_shared_info (const struct shared_info *shared_page) {
	int ret = 0;
	struct xen_add_to_physmap xatp;

	xatp.domid = DOMID_SELF;
	xatp.idx = 0;
	xatp.space = XENMAPSPACE_shared_info;
	xatp.gpfn = ( ((size_t) shared_page) >> 12);

	ret = HYPERVISOR_memory_op(XENMEM_add_to_physmap, &xatp);

	printk("%s: mapping result = %d\n", __func__, ret);

	return ret;
}

static void notify_hv_evtchn(evtchn_port_t evtchn) {
	struct evtchn_send send = {.port = evtchn};

	HYPERVISOR_event_channel_op(EVTCHNOP_send, &send);

//	printk("%s: result = %d\n", __func__, ret);
}

static int __write_console(const char *data, int len)
{
	XENCONS_RING_IDX cons, prod;
	int sent = 0;

	cons = console_intf->out_cons;
	prod = console_intf->out_prod;
	compiler_barrier();			/* update queue values before going on */
//	BUG_ON((prod - cons) > sizeof(console_intf->out));

	while ((sent < len) && ((prod - cons) < sizeof(console_intf->out)))
		console_intf->out[MASK_XENCONS_IDX(prod++, console_intf->out)] = data[sent++];

	compiler_barrier();			/* write ring before updating pointer */
	console_intf->out_prod = prod;

	if (sent)
		notify_hv_evtchn(console_evtchn);
	return sent;
}

static void memcpy_from_ring(const void *Ring,
        void *Dest,
        int off,
        int len)
{
    int c1, c2;
    const char *ring = Ring;
    char *dest = Dest;
    c1 = min(len, XENSTORE_RING_SIZE - off);
    c2 = len - c1;
    memcpy(dest, ring + off, c1);
    memcpy(dest + c1, ring, c2);
}

static void xenbus_thread_func(void *p1, void *p2, void *p3)
{
    struct xsd_sockmsg msg;
    unsigned prod = xenstore_intf->rsp_prod;

    printk("%s: xenbus thread in\n", __func__);
    for (;;) {
//        wait_event(xb_waitq, prod != xenstore_intf->rsp_prod);


	xenbus_printk("%s: xenbus thread before k_sem_take\n", __func__);
	compiler_barrier();
	k_sem_take(&isr_sem, K_FOREVER);
	printk("%s: woken up, received semaphore\n", __func__);
	compiler_barrier();
//	while (xenstore_intf->rsp_prod - xenstore_intf->rsp_cons < sizeof(msg)) {
//		printk("%s: xenbus thread after k_sem_take, try = %d\n", __func__, i);
//		k_usleep(500000);
//	}
        while (1) {
            xenbus_printk("%s: xenbus thread in while\n", __func__);
            prod = xenstore_intf->rsp_prod;
            xenbus_printk("%s: Rsp_cons %d, rsp_prod %d.\n", __func__, xenstore_intf->rsp_cons,
                  xenstore_intf->rsp_prod);
            if (xenstore_intf->rsp_prod - xenstore_intf->rsp_cons < sizeof(msg)) {
                printk("%s: breaking in while, msg to short\n", __func__);
                break;
            }
            compiler_barrier();
            memcpy_from_ring(xenstore_intf->rsp, &msg,
                             MASK_XENSTORE_IDX(xenstore_intf->rsp_cons),
                             sizeof(msg));
            xenbus_printk("%s: Msg len %lu, %d avail, id %d.\n", __func__, msg.len + sizeof(msg),
                  xenstore_intf->rsp_prod - xenstore_intf->rsp_cons, msg.req_id);

            if (xenstore_intf->rsp_prod - xenstore_intf->rsp_cons <
                sizeof(msg) + msg.len)
                break;

            xenbus_printk("%s: Message is good.\n", __func__);

            if (msg.type == XS_WATCH_EVENT) {
                struct xenbus_event *event = k_malloc(sizeof(*event) + msg.len);
                xenbus_event_queue *events = NULL;
                char *data = (char*)event + sizeof(*event);
                struct watch *watch;

                memcpy_from_ring(xenstore_intf->rsp, data,
                    MASK_XENSTORE_IDX(xenstore_intf->rsp_cons + sizeof(msg)),
                    msg.len);

                event->path = data;
                event->token = event->path + strlen(event->path)  + 1;

                compiler_barrier();
                xenstore_intf->rsp_cons += msg.len + sizeof(msg);

                for (watch = watches; watch; watch = watch->next)
                    if (!strcmp(watch->token, event->token)) {
                        events = watch->events;
                        break;
                    }

                if (events) {
                    event->next = *events;
                    *events = event;
//                    wake_up(&xenbus_watch_queue);
                } else {
                    printk("%s: unexpected watch token %s\n", __func__, event->token);
                    k_free(event);
                }
            } else {
                k_mutex_lock(&xenstore_mutex, K_FOREVER);
                req_info.reply = k_malloc(sizeof(msg) + msg.len);
                memcpy_from_ring(xenstore_intf->rsp, req_info.reply,
                                 MASK_XENSTORE_IDX(xenstore_intf->rsp_cons),
                                 msg.len + sizeof(msg));
                compiler_barrier();
                xenstore_intf->rsp_cons += msg.len + sizeof(msg);
                xenbus_printk("%s: signaling condvar\n", __func__);
                k_condvar_signal(&xenstore_condvar);
                k_mutex_unlock(&xenstore_mutex);
            }

            compiler_barrier();
            notify_hv_evtchn(xenstore_evtchn);
        }
    }
}

/* Send data to xenbus.  This can block.  All of the requests are seen
   by xenbus as if sent atomically.  The header is added
   automatically, using type %type, req_id %req_id, and trans_id
   %trans_id. */
static void xb_write(int type, int req_id, xenbus_transaction_t trans_id,
		     const struct write_req *req, int nr_reqs)
{
    XENSTORE_RING_IDX prod;
    int r;
    int len = 0;
    const struct write_req *cur_req;
    int req_off;
    int total_off;
    int this_chunk;
    struct xsd_sockmsg m = {.type = type, .req_id = req_id,
        .tx_id = trans_id };
    struct write_req header_req = { &m, sizeof(m) };

    xenbus_printk("%s: starting\n", __func__);
    for (r = 0; r < nr_reqs; r++)
        len += req[r].len;
    m.len = len;
    len += sizeof(m);

    cur_req = &header_req;

    //BUG_ON(len > XENSTORE_RING_SIZE);
    /* Wait for the ring to drain to the point where we can send the
       message. */
    prod = xenstore_intf->req_prod;

    if (prod + len - xenstore_intf->req_cons > XENSTORE_RING_SIZE)
    {
        /* Wait for there to be space on the ring */
        printk("prod %d, len %d, cons %d, size %d; waiting.\n",
                prod, len, xenstore_intf->req_cons, XENSTORE_RING_SIZE);
        while (xenstore_intf->req_prod + len - xenstore_intf->req_cons <= XENSTORE_RING_SIZE) {
        	printk("%s: waiting buffer\n", __func__);
        	k_usleep(500000);
        }
        xenbus_printk("Back from wait.\n");
        prod = xenstore_intf->req_prod;
    }

    /* We're now guaranteed to be able to send the message without
       overflowing the ring.  Do so. */
    total_off = 0;
    req_off = 0;
    xenbus_printk("%s: start writing\n", __func__);
    while (total_off < len)
    {
        this_chunk = min(cur_req->len - req_off,
                XENSTORE_RING_SIZE - MASK_XENSTORE_IDX(prod));
        memcpy((char *)xenstore_intf->req + MASK_XENSTORE_IDX(prod),
                (char *)cur_req->data + req_off, this_chunk);
        prod += this_chunk;
        req_off += this_chunk;
        total_off += this_chunk;
        if (req_off == cur_req->len)
        {
            req_off = 0;
            if (cur_req == &header_req)
                cur_req = req;
            else
                cur_req++;
        }
    }


    printk("Complete main loop of xb_write., total off = %d\n", total_off);
//    BUG_ON(req_off != 0);
//    BUG_ON(total_off != len);
//    BUG_ON(prod > xenstore_intf->req_cons + XENSTORE_RING_SIZE);

    /* Remote must see entire message before updating indexes */
    compiler_barrier();

    xenstore_intf->req_prod += len;

    compiler_barrier();
    /* Send evtchn to notify remote */
    notify_hv_evtchn(xenstore_evtchn);
}


/* Send a mesasge to xenbus, in the same fashion as xb_write, and
   block waiting for a reply.  The reply is malloced and should be
   freed by the caller. */
struct xsd_sockmsg *
xenbus_msg_reply(int type,
		 xenbus_transaction_t trans,
		 struct write_req *io,
		 int nr_reqs)
{
    int id = 0;
//    DEFINE_WAIT(w);
    struct xsd_sockmsg *rep;

    k_mutex_lock(&xenstore_mutex, K_FOREVER);
    id = 0; /*allocate_xenbus_id()*/
//    add_waiter(w, req_info[id].waitq);

    xenbus_printk("%s: calling xb_write\n", __func__);
    compiler_barrier();
    xb_write(type, id, trans, io, nr_reqs);

    xenbus_printk("%s: xb_write end, start waiting\n", __func__);
    compiler_barrier();
    k_condvar_wait(&xenstore_condvar, &xenstore_mutex, K_FOREVER);
    xenbus_printk("%s: waiting finished\n", __func__);
//    schedule();
//    remove_waiter(w, req_info[id].waitq);
//    wake(current);

    rep = req_info.reply;
//    BUG_ON(rep->req_id != id);
//    release_xenbus_id(id);
    k_mutex_unlock(&xenstore_mutex);
    return rep;
}

static char *errmsg(struct xsd_sockmsg *rep)
{
    char *res;
    if (!rep) {
	char msg[] = "No reply";
	size_t len = strlen(msg) + 1;
	return memcpy(k_malloc(len), msg, len);
    }
    if (rep->type != XS_ERROR)
	return NULL;
    res = k_malloc(rep->len + 1);
    memcpy(res, rep + 1, rep->len);
    res[rep->len] = 0;
    k_free(rep);
    return res;
}

char *xenbus_read(xenbus_transaction_t xbt, const char *path, char **value)
{
    struct write_req req[] = { {path, strlen(path) + 1} };
    struct xsd_sockmsg *rep;
    char *res, *msg;

    rep = xenbus_msg_reply(XS_READ, xbt, req, ARRAY_SIZE(req));
    msg = errmsg(rep);
    if (msg) {
	*value = NULL;
	return msg;
    }
    res = k_malloc(rep->len + 1);
    memcpy(res, rep + 1, rep->len);
    res[rep->len] = 0;
    k_free(rep);
    *value = res;
    return NULL;
}

int xenbus_read_integer(const char *path)
{
    char *res, *buf;
    int t;

    res = xenbus_read(XBT_NIL, path, &buf);
    if (res) {
	printk("Failed to read %s.\n", path);
	k_free(res);
	return -1;
    }
    //sscanf(buf, "%d", &t);
    k_free(buf);
    return t;
}

/* List the contents of a directory.  Returns a k_malloc()ed array of
   pointers to k_malloc()ed strings.  The array is NULL terminated.  May
   block. */
char *xenbus_ls(xenbus_transaction_t xbt, const char *pre, char ***contents)
{
    struct xsd_sockmsg *reply, *repmsg;
    struct write_req req[] = { { pre, strlen(pre)+1 } };
    int nr_elems, x, i;
    char **res, *msg;

    xenbus_printk("%s: calling xenbus_msg_reply\n", __func__);
    repmsg = xenbus_msg_reply(XS_DIRECTORY, xbt, req, ARRAY_SIZE(req));
    xenbus_printk("%s: return from xenbus_msg_reply\n", __func__);
    msg = errmsg(repmsg);
    if (msg) {
	*contents = NULL;
	return msg;
    }
    reply = repmsg + 1;
    for (x = nr_elems = 0; x < repmsg->len; x++)
        nr_elems += (((char *)reply)[x] == 0);
    res = k_malloc(sizeof(res[0]) * (nr_elems + 1));
    for (x = i = 0; i < nr_elems; i++) {
        int l = strlen((char *)reply + x);
        res[i] = k_malloc(l + 1);
        memcpy(res[i], (char *)reply + x, l + 1);
        x += l + 1;
    }
    res[i] = NULL;
    k_free(repmsg);
    *contents = res;
    return NULL;
}

static void read_thread(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	char **dirs;
	int x;
	char *domid;
	/* TODO: Read domid, then /local/domain/$domid */
	char *pre = "domid", buf[50];
	char *msg = xenbus_read(XBT_NIL, pre, &domid);

	if (msg) {
		printk("Error in xenbus read: %s\n", msg);
		k_free(msg);
		return;
	}
	if (!domid) {
		printk ("NULL domid\n");
		return;
	}

	printk("%s: domid returned = %s\n", __func__, domid);

	snprintf(buf, 50, "/local/domain/%s", domid);
	printk("%s: running xenbus ls for %s\n", __func__, buf);
	msg = xenbus_ls(XBT_NIL, buf, &dirs);
	if (msg) {
		printk("Error in xenbus ls: %s\n", msg);
		k_free(msg);
		return;
	}

	printk("xenbus_ls test results for pre = %s\n", buf);
	for (x = 0; dirs[x]; x++)
	{
	    printk("ls %s[%d] -> %s\n", buf, x, dirs[x]);
	    k_free(dirs[x]);
	}
	k_free(dirs);

//	char print[] = "First condvar irq print!\n";
//	(void)__write_console(print, sizeof(print));
//	printk("%s: taking condvar 1\n", __func__);
//
//	k_mutex_lock(&xenstore_mutex, K_FOREVER);
//	compiler_barrier();
//	k_condvar_wait(&xenstore_condvar, &xenstore_mutex, K_FOREVER);
//	k_mutex_unlock(&xenstore_mutex);
//	printk("%s: received condvar 1\n", __func__);
//
//
//	char print1[] = "Second condvar irq print!\n";
//	(void)__write_console(print1, sizeof(print1));
//	printk("%s: taking condvar 2\n", __func__);
//
//	k_mutex_lock(&xenstore_mutex, K_FOREVER);
//	compiler_barrier();
//	k_condvar_wait(&xenstore_condvar, &xenstore_mutex, K_FOREVER);
//	k_mutex_unlock(&xenstore_mutex);
//	printk("%s: received condvar 2\n", __func__);

}

static void sanity_thread(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		printk("%s:>>>>>>>>>> running\n", __func__);
		k_sleep(K_SECONDS(3));
	}
}

static void irq_routine(void *arg) {
	static int a = 0;

	if (shared_info_page->evtchn_pending[0] & (1 << xenstore_evtchn)) {
//		printk("%s: giving semaphore\n", __func__);
		k_sem_give(&isr_sem);
//		printk("%s: gave semaphore\n", __func__);
//		printk("isr_sem->count = %d\n", isr_sem.count);
//		k_sem_give(&thread_sem);
	}

//	printk("%s: giving semaphore\n", __func__);
//	k_sem_give(&isr_sem);
	shared_info_page->vcpu_info[0].evtchn_upcall_pending = 0;
	shared_info_page->vcpu_info[0].evtchn_pending_sel = 0;
	shared_info_page->evtchn_pending[0] = 0;
	a++;
//	printk("%s: ISR out\n", __func__);
}

static int xen_init(const struct device *dev) {
	ARG_UNUSED(dev);

	int ret = 0;
	uint64_t console_pfn = 0;
	uint64_t xenstore_pfn = 0;
	k_tid_t tid = 0;
	shared_info_page = k_aligned_alloc(4096, 4096);
	if (!shared_info_page) {
		printk("%s: failed to allocate memory for Xen shared page!\n", __func__);
		return -ENOMEM;
	}



	ret = xen_map_shared_info(shared_info_page);
	if (ret) {
		printk("%s: failed to map for Xen shared page, ret = %d\n", __func__, ret);
		return ret;
	}

	ret = hvm_get_parameter(HVM_PARAM_CONSOLE_PFN, &console_pfn);
	if (ret) {
		printk("%s: failed to get Xen console PFN, ret = %d\n", __func__, ret);
		return ret;
	}

	/*
	 * TODO: Unfortunately, virt_region_get() - is a static function,
	 * but Zephyr is identity mapped (phys:virt - 1:1), so it should work
	 */
	console_intf = (struct xencons_interface *) (console_pfn << 12);
	arch_mem_map(console_intf, (uintptr_t) console_intf, 4096, K_MEM_PERM_RW | K_MEM_CACHE_WB);
	printk("console virt after mapping = %p\n", console_intf);

	ret = hvm_get_parameter(HVM_PARAM_CONSOLE_EVTCHN, &console_evtchn);
	if (ret) {
		printk("%s: failed to get Xen console event channel, ret = %d\n", __func__, ret);
		return ret;
	}
	printk("%s: console eventchn = %llx\n", __func__, console_evtchn);

	char print[] = "First print the Zephyr PV console!\n";
	(void)__write_console(print, sizeof(print));



	/* ======================================================= */
	ret = hvm_get_parameter(HVM_PARAM_STORE_PFN, &xenstore_pfn);
	if (ret) {
		printk("%s: failed to get Xenstore PFN, ret = %d\n", __func__, ret);
		return ret;
	}

	xenstore_intf = (struct xenstore_domain_interface *) (xenstore_pfn << 12);
	arch_mem_map(xenstore_intf, (uintptr_t) xenstore_intf, 4096, K_MEM_PERM_RW | K_MEM_CACHE_WB);
	printk("Xenstore addr after mapping = %p\n", xenstore_intf);

	ret = hvm_get_parameter(HVM_PARAM_STORE_EVTCHN, &xenstore_evtchn);
	if (ret) {
		printk("%s: failed to get Xen console event channel, ret = %d\n", __func__, ret);
		return ret;
	}
	printk("%s: Xenstore eventchn = %llx\n", __func__, xenstore_evtchn);

	k_condvar_init(&xenstore_condvar);
	k_mutex_init(&xenstore_mutex);
	k_sem_init(&isr_sem, 0, 1);

	printk("%s: vars inited\n", __func__);

	int version = HYPERVISOR_xen_version(XENVER_version, NULL);
	printk("xen version = %x\n", version);
/*
	tid = k_thread_create(&xenstore_thrd, xenstore_thrd_stack,
			K_KERNEL_STACK_SIZEOF(xenstore_thrd_stack),
			xenbus_thread_func, NULL, NULL, NULL, 2, 0, K_NO_WAIT);
	if (!tid)
		printk("%s: Failed to create Xenstore thread\n", __func__);

	k_thread_name_set(&xenstore_thrd, "xenstore_thread2");
	printk("%s: xenstore thread inited\n", __func__);

	tid = k_thread_create(&read_thrd, read_thrd_stack,
				K_KERNEL_STACK_SIZEOF(read_thrd_stack),
				read_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	if (!tid)
		printk("%s: Failed to create read thread\n", __func__);
	k_thread_name_set(&read_thrd, "read_thread");
	printk("%s: read thread inited, stack defined at %p\n", __func__, read_thrd_stack);

	k_thread_create(&sanity_thrd, sanity_thrd_stack,
			K_KERNEL_STACK_SIZEOF(sanity_thrd_stack),
			sanity_thread, NULL, NULL, NULL, 1, 0, K_NO_WAIT);
	k_thread_name_set(&sanity_thrd, "sanity_thread");
*/

	IRQ_CONNECT(    DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq),
			DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, priority),
			irq_routine, NULL,
			DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, flags));

	irq_enable(DT_IRQ_BY_IDX(DT_INST(0,xen_xen), 0, irq));

	printk("%s: irq inited\n", __func__);
	return 0;
}

SYS_INIT(xen_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

