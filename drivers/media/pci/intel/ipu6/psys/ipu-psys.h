/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_PSYS_H
#define IPU_PSYS_H

#include <linux/cdev.h>
#include <linux/workqueue.h>

#include <linux/version.h>
#include "ipu6.h"
#include "ipu6-bus.h"
#include "ipu-fw-psys.h"
#include "ipu-platform-psys.h"

/* PSYS Info bits*/
#define IPU_REG_PSYS_INFO_SEG_CMEM_MASTER(a)	(0x2c + ((a) * 12))
#define IPU_REG_PSYS_INFO_SEG_XMEM_MASTER(a)	(0x5c + ((a) * 12))

#define IPU_PSYS_REG_SPC_STATUS_CTRL		0x0
#define IPU_PSYS_REG_SPC_START_PC		0x4
#define IPU_PSYS_REG_SPC_ICACHE_BASE		0x10
#define IPU_REG_PSYS_INFO_SEG_0_CONFIG_ICACHE_MASTER	0x14

#define IPU_PSYS_SPC_STATUS_START			BIT(1)
#define IPU_PSYS_SPC_STATUS_RUN				BIT(3)
#define IPU_PSYS_SPC_STATUS_READY			BIT(5)
#define IPU_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE	BIT(12)
#define IPU_PSYS_SPC_STATUS_ICACHE_PREFETCH		BIT(13)

#define IPU_PSYS_REG_SPP0_STATUS_CTRL			0x20000

#define IPU_INFO_ENABLE_SNOOP			BIT(0)
#define IPU_INFO_DEC_FORCE_FLUSH		BIT(1)
#define IPU_INFO_DEC_PASS_THRU			BIT(2)
#define IPU_INFO_ZLW                            BIT(3)
#define IPU_INFO_STREAM_ID_SET(id)		(((id) & 0x1f) << 4)
#define IPU_INFO_REQUEST_DESTINATION_IOSF	BIT(9)
#define IPU_INFO_IMR_BASE			BIT(10)
#define IPU_INFO_IMR_DESTINED			BIT(11)

#define IPU_INFO_REQUEST_DESTINATION_PRIMARY IPU_INFO_REQUEST_DESTINATION_IOSF

#define IPU_REG_DMA_TOP_AB_GROUP1_BASE_ADDR         0x1ae000
#define IPU_REG_DMA_TOP_AB_GROUP2_BASE_ADDR         0x1af000
#define IPU_REG_DMA_TOP_AB_RING_MIN_OFFSET(n)       (0x4 + (n) * 0xc)
#define IPU_REG_DMA_TOP_AB_RING_MAX_OFFSET(n)       (0x8 + (n) * 0xc)
#define IPU_REG_DMA_TOP_AB_RING_ACCESS_OFFSET(n)    (0xc + (n) * 0xc)

enum ipu_device_ab_group1_target_id {
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R0_SPC_DMEM,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R1_SPC_DMEM,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R2_SPC_DMEM,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R3_SPC_STATUS_REG,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R4_SPC_MASTER_BASE_ADDR,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R5_SPC_PC_STALL,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R6_SPC_EQ,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R7_SPC_RESERVED,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R8_SPC_RESERVED,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R9_SPP0,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R10_SPP1,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R11_CENTRAL_R1,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R12_IRQ,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R13_CENTRAL_R2,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R14_DMA,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R15_DMA,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R16_GP,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R17_ZLW_INSERTER,
	IPU_DEVICE_AB_GROUP1_TARGET_ID_R18_AB,
};

/* IRQ-related registers in PSYS */
#define IPU_REG_PSYS_GPDEV_IRQ_EDGE		0x1aa200
#define IPU_REG_PSYS_GPDEV_IRQ_MASK		0x1aa204
#define IPU_REG_PSYS_GPDEV_IRQ_STATUS		0x1aa208
#define IPU_REG_PSYS_GPDEV_IRQ_CLEAR		0x1aa20c
#define IPU_REG_PSYS_GPDEV_IRQ_ENABLE		0x1aa210
#define IPU_REG_PSYS_GPDEV_IRQ_LEVEL_NOT_PULSE	0x1aa214
/* There are 8 FW interrupts, n = 0..7 */
#define IPU_PSYS_GPDEV_FWIRQ0			5
#define IPU_PSYS_GPDEV_FWIRQ1			6
#define IPU_PSYS_GPDEV_FWIRQ2			7
#define IPU_PSYS_GPDEV_FWIRQ3			8
#define IPU_PSYS_GPDEV_FWIRQ4			9
#define IPU_PSYS_GPDEV_FWIRQ5			10
#define IPU_PSYS_GPDEV_FWIRQ6			11
#define IPU_PSYS_GPDEV_FWIRQ7			12
#define IPU_PSYS_GPDEV_IRQ_FWIRQ(n)		(1 << (n))
#define IPU_REG_PSYS_GPDEV_FWIRQ(n)		(4 * (n) + 0x1aa100)

/*
 * psys subdomains power request regs
 */
enum ipu_device_buttress_psys_domain_pos {
	IPU_PSYS_SUBDOMAIN_ISA		= 0,
	IPU_PSYS_SUBDOMAIN_PSA		= 1,
	IPU_PSYS_SUBDOMAIN_BB		= 2,
	IPU_PSYS_SUBDOMAIN_IDSP1	= 3, /* only in IPU6M */
	IPU_PSYS_SUBDOMAIN_IDSP2	= 4, /* only in IPU6M */
};

#define IPU_PSYS_SUBDOMAINS_POWER_MASK  (BIT(IPU_PSYS_SUBDOMAIN_ISA) | \
					 BIT(IPU_PSYS_SUBDOMAIN_PSA) | \
					 BIT(IPU_PSYS_SUBDOMAIN_BB))

#define IPU_PSYS_SUBDOMAINS_POWER_REQ                   0xa0
#define IPU_PSYS_SUBDOMAINS_POWER_STATUS                0xa4

#define IPU_PSYS_CMD_TIMEOUT_MS	2000
#define IPU_PSYS_OPEN_TIMEOUT_US	   50
#define IPU_PSYS_OPEN_RETRY (10000 / IPU_PSYS_OPEN_TIMEOUT_US)

#define IPU_PSYS_PG_POOL_SIZE 16
#define IPU_PSYS_PG_MAX_SIZE 8192
#define IPU_MAX_PSYS_CMD_BUFFERS 32
#define IPU_PSYS_EVENT_CMD_COMPLETE IPU_FW_PSYS_EVENT_TYPE_SUCCESS
#define IPU_PSYS_EVENT_FRAGMENT_COMPLETE IPU_FW_PSYS_EVENT_TYPE_SUCCESS
#define IPU_PSYS_CLOSE_TIMEOUT_US   50
#define IPU_PSYS_CLOSE_TIMEOUT (100000 / IPU_PSYS_CLOSE_TIMEOUT_US)
#define IPU_MAX_RESOURCES 128

extern enum ipu6_version ipu_ver;

/* Opaque structure. Do not access fields. */
struct ipu_resource {
	u32 id;
	int elements;	/* Number of elements available to allocation */
	unsigned long *bitmap;	/* Allocation bitmap, a bit for each element */
};

enum ipu_resource_type {
	IPU_RESOURCE_DEV_CHN = 0,
	IPU_RESOURCE_EXT_MEM,
	IPU_RESOURCE_DFM
};

/* Allocation of resource(s) */
/* Opaque structure. Do not access fields. */
struct ipu_resource_alloc {
	enum ipu_resource_type type;
	struct ipu_resource *resource;
	int elements;
	int pos;
};

/*
 * This struct represents all of the currently allocated
 * resources from IPU model. It is used also for allocating
 * resources for the next set of PGs to be run on IPU
 * (ie. those PGs which are not yet being run and which don't
 * yet reserve real IPU resources).
 * Use larger array to cover existing resource quantity
 */

/* resource size may need expand for new resource model */
struct ipu_psys_resource_pool {
	u32 cells;	/* Bitmask of cells allocated */
	struct ipu_resource dev_channels[16];
	struct ipu_resource ext_memory[32];
	struct ipu_resource dfms[16];
	DECLARE_BITMAP(cmd_queues, 32);
	/* Protects cmd_queues bitmap */
	spinlock_t queues_lock;
};

/*
 * This struct keeps book of the resources allocated for a specific PG.
 * It is used for freeing up resources from struct ipu_psys_resources
 * when the PG is released from IPU (or model of IPU).
 */
struct ipu_psys_resource_alloc {
	u32 cells;	/* Bitmask of cells needed */
	struct ipu_resource_alloc
	 resource_alloc[IPU_MAX_RESOURCES];
	int resources;
};

struct task_struct;
struct ipu_psys {
	struct ipu_psys_capability caps;
	struct cdev cdev;
	struct device dev;

	struct mutex mutex;	/* Psys various */
	int ready; /* psys fw status */
	bool icache_prefetch_sp;
	bool icache_prefetch_isp;
	spinlock_t ready_lock;	/* protect psys firmware state */
	spinlock_t pgs_lock;	/* Protect pgs list access */
	struct list_head fhs;
	struct list_head pgs;
	struct list_head started_kcmds_list;
	struct ipu6_psys_pdata *pdata;
	struct ipu6_bus_device *adev;
	struct ia_css_syscom_context *dev_ctx;
	struct ia_css_syscom_config *syscom_config;
	struct ia_css_psys_server_init *server_init;
	struct task_struct *sched_cmd_thread;
	wait_queue_head_t sched_cmd_wq;
	atomic_t wakeup_count;  /* Psys schedule thread wakeup count */

	/* Resources needed to be managed for process groups */
	struct ipu_psys_resource_pool resource_pool_running;

	const struct firmware *fw;
	struct sg_table fw_sgt;
	u64 *pkg_dir;
	dma_addr_t pkg_dir_dma_addr;
	unsigned int pkg_dir_size;
	unsigned long timeout;

	int active_kcmds, started_kcmds;
	void *fwcom;

	int power_gating;
};

struct ipu_psys_fh {
	struct ipu_psys *psys;
	struct mutex mutex;	/* Protects bufs_list & kcmds fields */
	struct list_head list;
	/* Holds all buffers that this fh owns */
	struct list_head bufs_list;
	/* Holds all descriptors (fd:kbuffer associations) */
	struct list_head descs_list;
	struct list_head bufs_lru;
	wait_queue_head_t wait;
	struct ipu_psys_scheduler sched;

	u32 num_bufs;
	u32 num_descs;
	u32 num_bufs_lru;
};

struct ipu_psys_pg {
	struct ipu_fw_psys_process_group *pg;
	size_t size;
	size_t pg_size;
	dma_addr_t pg_dma_addr;
	struct list_head list;
	struct ipu_psys_resource_alloc resource_alloc;
};

struct ipu6_psys_constraint {
	struct list_head list;
	unsigned int min_freq;
};

struct ipu_psys_kcmd {
	struct ipu_psys_fh *fh;
	struct list_head list;
	struct ipu_psys_buffer_set *kbuf_set;
	enum ipu_psys_cmd_state state;
	void *pg_manifest;
	size_t pg_manifest_size;
	struct ipu_psys_kbuffer **kbufs;
	struct ipu_psys_buffer *buffers;
	size_t nbuffers;
	struct ipu_fw_psys_process_group *pg_user;
	struct ipu_psys_pg *kpg;
	u64 user_token;
	u64 issue_id;
	u32 priority;
	u32 kernel_enable_bitmap[4];
	u32 terminal_enable_bitmap[4];
	u32 routing_enable_bitmap[4];
	u32 rbm[5];
	struct ipu6_psys_constraint constraint;
	struct ipu_psys_event ev;
	struct timer_list watchdog;
};

struct ipu_dma_buf_attach {
	struct device *dev;
	u64 len;
	void *userptr;
	struct sg_table *sgt;
	struct page **pages;
	size_t npages;
};

struct ipu_psys_kbuffer {
	u64 len;
	void *userptr;
	void *kaddr;
	struct list_head list;
	dma_addr_t dma_addr;
	struct sg_table *sgt;
	struct dma_buf_attachment *db_attach;
	struct dma_buf *dbuf;
	u32 flags;
	/* The number of times this buffer is mapped */
	atomic_t map_count;
	bool valid;	/* True when buffer is usable */
};

struct ipu_psys_desc {
	struct ipu_psys_kbuffer	*kbuf;
	struct list_head	list;
	u32			fd;
};

#define inode_to_ipu_psys(inode) \
	container_of((inode)->i_cdev, struct ipu_psys, cdev)

void ipu_psys_setup_hw(struct ipu_psys *psys);
void ipu_psys_subdomains_power(struct ipu_psys *psys, bool on);
void ipu_psys_handle_events(struct ipu_psys *psys);
int ipu_psys_kcmd_new(struct ipu_psys_command *cmd, struct ipu_psys_fh *fh);
void ipu_psys_run_next(struct ipu_psys *psys);
struct ipu_psys_pg *__get_pg_buf(struct ipu_psys *psys, size_t pg_size);
struct ipu_psys_kbuffer *
ipu_psys_lookup_kbuffer(struct ipu_psys_fh *fh, int fd);
struct ipu_psys_kbuffer *
ipu_psys_mapbuf_locked(int fd, struct ipu_psys_fh *fh);
struct ipu_psys_kbuffer *
ipu_psys_lookup_kbuffer_by_kaddr(struct ipu_psys_fh *fh, void *kaddr);
int ipu_psys_resource_pool_init(struct ipu_psys_resource_pool *pool);
void ipu_psys_resource_pool_cleanup(struct ipu_psys_resource_pool *pool);
struct ipu_psys_kcmd *ipu_get_completed_kcmd(struct ipu_psys_fh *fh);
long ipu_ioctl_dqevent(struct ipu_psys_event *event,
		       struct ipu_psys_fh *fh, unsigned int f_flags);

#endif /* IPU_PSYS_H */
