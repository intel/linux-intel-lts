#ifndef _SWHV_ACRN_H_
#define _SWHV_ACRN_H_ 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <linux/version.h> /* LINUX_VERSION_CODE */
#include <linux/list.h> /* for struct list_head */

#include "swhv_defines.h"
#include "pw_version.h"

#define SW_DEFINE_LIST_HEAD(name, dummy) struct list_head name
#define SW_DECLARE_LIST_HEAD(name, dummy) extern struct list_head name
#define SW_LIST_ENTRY(name, dummy) struct list_head name
#define SW_LIST_HEAD_VAR(dummy) struct list_head
#define SW_LIST_HEAD_INIT(head) INIT_LIST_HEAD(head)
#define SW_LIST_ENTRY_INIT(node, field) INIT_LIST_HEAD(&node->field)
#define SW_LIST_ADD(head, node, field) list_add_tail(&node->field, head)
#define SW_LIST_GET_HEAD_ENTRY(head, type, field)                              \
	list_first_entry(head, struct type, field)
#define SW_LIST_UNLINK(node, field) list_del(&node->field)
#define SW_LIST_FOR_EACH_ENTRY(node, head, field)                              \
	list_for_each_entry(node, head, field)
#define SW_LIST_EMPTY(head) list_empty(head)
#define SW_LIST_HEAD_INITIALIZER(head) LIST_HEAD_INIT(head)

int device_open_i(struct inode *inode, struct file *file);

ssize_t device_read_i(struct file *file, /* see include/linux/fs.h   */
		      char __user *buffer, /* buffer to be filled with data */
		      size_t length, /* length of the buffer */
		      loff_t *offset);

long swhv_configure(struct swhv_driver_interface_msg __user *remote_msg,
		    int local_len);
long swhv_start(void);
long swhv_stop(void);
long swhv_get_cpu_count(u32 __user *remote_args);
long swhv_get_clock(u32 __user *remote_in_args, u64 __user *remote_args);
long swhv_get_topology(u64 __user *remote_args);
long swhv_get_hypervisor_type(u32 __user *remote_args);
int swhv_load_driver_i(void);
void swhv_unload_driver_i(void);
void cleanup_error_i(void);
long swhv_msr_read(u32 __user *remote_in_args, u64 __user *remote_args);
long swhv_collection_poll(void);

enum MSR_CMD_TYPE {
	MSR_OP_NONE = 0,
	MSR_OP_READ,
	MSR_OP_WRITE,
	MSR_OP_READ_CLEAR
};

enum MSR_CMD_STATUS { MSR_OP_READY = 0, MSR_OP_REQUESTED, MSR_OP_HANDLED };

struct profiling_msr_op {
	/* value to write or location to write into */
	uint64_t value;
	/* MSR address to read/write; last entry will have value of -1 */
	uint32_t msr_id;
	/* parameter; usage depends on operation */
	uint16_t param;
	uint8_t msr_op_type;
	uint8_t reg_type;
};

#define MAX_MSR_LIST_NUM 15
struct profiling_msr_ops_list {
	int32_t collector_id;
	uint32_t num_entries;
	int32_t msr_op_state; /* enum value from 'MSR_CMD_STATUS' */
	struct profiling_msr_op entries[MAX_MSR_LIST_NUM];
};

#define COLLECTOR_SOCWATCH 1

struct profiling_control {
	int32_t collector_id;
	int32_t reserved;
	uint64_t switches;
};

/**
 * struct - sw_collector_data
 * Information about the collector to be invoked at collection time.
 *
 * The collector_lists array holds linked lists of collectors to
 * be exercised at specific points in time during the collection
 * (e.g. begin, poll, end, etc.).  At a trigger time, the driver walks
 * that time's list of nodes, and exercises the collectors on that list.
 *
 * @list:                   List/link implementation
 * @cpumask:                Collect if cpu matches mask
 * @info:                   Ptr to metric info
 * @ops:                    Ptr to collector's operations
 * @last_update_jiffies:    Indicates when this node was last exercised.
 * @per_msg_payload_size:   Data size
 * @msg:                    Ptr to collected data
 */
typedef struct swhv_acrn_msr_collector_data {
	SW_LIST_ENTRY(list, swhv_acrn_msr_collector_data);
	pw_s16_t cpu_mask;
	pw_s16_t sample_id;
	struct profiling_msr_ops_list *msr_ops_list;
	size_t per_msg_payload_size;
} swhv_acrn_msr_collector_data_t;
#endif /* _SWHV_ACRN_H_ */
