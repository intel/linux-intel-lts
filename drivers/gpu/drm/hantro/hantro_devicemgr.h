/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro device manager header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_DEVICE_H__
#define __HANTRO_DEVICE_H__

#include "hantro_drm.h"

#define MAX(a, b) ({ \
	typeof(a) a_ = (a); \
	typeof(b) b_ = (b); \
	(a_ > b_) ? a_ : b_; \
})
#define MIN(a, b) ({ \
	typeof(a) a_ = (a); \
	typeof(b) b_ = (b); \
	(a_ < b_) ? a_ : b_; \
})

enum hantro_device_type {
	DEVICE_UNKNOWN = 0,
	DEVICE_KEEMBAY,
	DEVICE_THUNDERBAY
};

/* supported core type */
enum device_coretype {
	CORE_UNKNOWN = -1,
	CORE_DEVICE = 0,
	CORE_DEC,
	CORE_ENC,
	CORE_CACHE,
	CORE_DEC400,
};

/* structure to hold data performance data per device */
struct performance_data {
	int count;
	u64 last_resv;
	u64 totaltime;
	u64 hwcycles;
};

struct cache_core_config {
	enum cache_client_type client;
	unsigned long long base_addr;
	u32 iosize;
	int irq;
	enum driver_cache_dir dir;
	unsigned long long parentaddr;
};

struct cache_dev_t {
	struct cache_core_config core_cfg;	/* each core config, such as base addr, irq, etc */
	unsigned long hw_id;			/* hw id to indicate project */
	u32 core_id;				/* core id for driver and sw internal use */
	u32 is_valid;				/* indicate this core is hantro's core or not */
	u32 is_reserved;			/* indicate this core is occupied by user or not */
	struct file *cacheowner;		/* indicate which process is occupying the core */
	u32 irq_received;			/* indicate this core receives irq */
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 __iomem *hwregs;
	char reg_name[32];
	unsigned long long com_base_addr;	/* common base addr of each L2 */
	int irqlist[4];
	char irq_name[4][32];
	enum device_coretype parenttype;
	u32 parentid;				/* parent codec core's core_id */
	void *parentcore;			/*
						 * either struct hantroenc_t or struct hantrodec_t,
						 * or device itself
						 */
	struct device_info *pdevinfo;
	struct cache_dev_t *next;
};

struct dec400_core_cfg {
	unsigned long long dec400corebase;

	unsigned int iosize;
	unsigned long long parentaddr;
};

struct dec400_t {
	struct dec400_core_cfg core_cfg;

	u32 core_id;
	u8 __iomem *hwregs;

	char reg_name[32];
	enum device_coretype parenttype;
	u32 parentid;				/* parent codec core's core_id */
	void *parentcore;			/*
						 * either struct hantroenc_t or struct hantrodec_t,
						 * or device itself
						 */
	struct device_info *pdevinfo;
	struct dec400_t *next;
};

struct core_config {
	unsigned long long base_addr;
	u32 iosize;
	int irq;
	/*
	 * resource_shared indicate core share resources with other cores.
	 * If 1, cores can not work at same time.
	 */
	u32 resource_shared;
};

struct hantroenc_t {
	struct core_config core_cfg;	/* each core config ,such as base addr, irq, etc */
	u32 hw_id;			/* hw id to indicate project */
	u32 core_id;			/* core id for driver and sw internal use */
	u32 is_reserved;		/* indicate this core is occupied by user or not */
	int pid;			/* indicate which process is occupying the core */
	u32 irq_received;		/* indicate this core receives irq */
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 __iomem *hwregs;
	char reg_name[32];
	struct clk *dev_clk;
	unsigned long clk_freq;
	struct fasync_struct *async_queue;
	int irqlist[4];
	char irq_name[4][32];
	struct device_info *pdevinfo;
	struct performance_data perf_data;
	struct hantroenc_t *next;
};

#define HANTRO_VC8000D_REGS	393 /* VC8000D total regs */
#define DEC_IO_SIZE_MAX		(HANTRO_VC8000D_REGS * 4)

struct hantrodec_t {
	u32 cfg;
	int core_id;
	unsigned int iosize;
	u32 cfg_backup;
	/* indicate if main core exist */
	struct hantrodec_t *its_main_core_id;
	/* indicate if aux core exist */
	struct hantrodec_t *its_aux_core_id;
	/*
	 * all access to hwregs are through readl/writel
	 * so volatile is removed according to doc "volatile is evil"
	 */
	u8 __iomem *hwregs;
	int hw_id;
	u8 __iomem *page_lut_regs_read;
	char reg_name[32];
	unsigned long long multicorebase;
	/*
	 * Because one core may contain multi-pipeline,
	 * so multicore base may be changed
	 */
	unsigned long long multicorebase_actual;

	u32 dec_regs[DEC_IO_SIZE_MAX / 4];
	int irqlist[4];
	char irq_name[4][32];
	struct clk *dev_clk;
	unsigned long clk_freq;
	struct file *dec_owner;
	struct file *pp_owner;
	struct device_info *pdevinfo;
	struct performance_data perf_data;
	struct hantrodec_t *next;
};

struct hantro_cooling_data {
	unsigned int media_clk_state;
	unsigned int media_clk_max_state;
	unsigned long clk_freq;
	struct thermal_cooling_device *cooling_dev;
};

#define NODE_TYPES 2  /* encoder & decoder */
#define MAX_CORES 4   /* max core per device */
#define NODE_TYPE_DEC	1
#define NODE_TYPE_ENC	2

struct device_info {
	struct device *dev; /* related dev, for drm usage */
	struct drm_device *drm_dev;
	struct device *codec_rsvmem;
	struct dentry *debugfs_root;
	struct resource mem_res[2];
	phys_addr_t rsvmem_addr;
	phys_addr_t memsize;
	int deviceid;
	u32 config;
	struct hantro_cooling_data thermal_data;
	int deccore_num;
	int enccore_num;
	int dec400core_num;
	int cachecore_num;

	struct hantrodec_t *dechdr;
	struct hantroenc_t *enchdr;
	struct cache_dev_t *cachehdr;
	struct dec400_t *dec400hdr;

	/* orig cache global vars */
	wait_queue_head_t cache_hw_queue;
	wait_queue_head_t cache_wait_queue;
	/* spinlock for cache owner */
	spinlock_t cache_owner_lock;

	/* orig enc global vars */
	struct semaphore enc_core_sem;
	wait_queue_head_t enc_hw_queue;
	/* spinlock for enc owner */
	spinlock_t enc_owner_lock;
	wait_queue_head_t enc_wait_queue;

	/* orig dec global vars */
	int dec_irq;
	int pp_irq;
	/* spinlock for dec owner */
	spinlock_t owner_lock;
	wait_queue_head_t dec_wait_queue;
	wait_queue_head_t pp_wait_queue;
	wait_queue_head_t hw_queue;
	struct semaphore dec_core_sem;
	struct semaphore pp_core_sem;
	void *priv_data;
	struct idr clients;
	struct idr allocations;
	/* alloc mutex struct */
	struct mutex alloc_mutex;
	struct device_info *next;
};

struct device_info *get_device_node(u32 deviceid);
int get_device_core_num(u32 deviceid, enum device_coretype type);
struct hantrodec_t *get_dec_node(struct device_info *pdevinfo, u32 nodeidx);
struct hantrodec_t *get_dec_node_by_device_id(u32 deviceid, u32 nodeidx);
struct hantrodec_t *get_first_dec_nodes(struct device_info *pdevinfo);
struct hantroenc_t *get_enc_node(struct device_info *pdevinfo, u32 nodeidx);
struct hantroenc_t *get_enc_node_by_device_id(u32 deviceid, u32 nodeidx);
struct cache_dev_t *get_cache_nodes(u32 deviceid, u32 nodeidx);
struct dec400_t *get_dec400_nodes(u32 deviceid, u32 nodeidx);
struct dec400_t *get_dec400_node_by_type(u32 deviceid, u32 parenttype, u32 parentnodeidx);
int add_dec_node(struct device_info *pdevinfo, struct hantrodec_t *deccore);
int add_enc_node(struct device_info *pdevinfo, struct hantroenc_t *enccore);
int add_dec400_node(struct device_info *pdevinfo, struct dec400_t *dec400core);
int add_cache_node(struct device_info *pdevinfo, struct cache_dev_t *cachecore);
int get_device_count(void);
int device_remove(void);
int device_init(void);
void device_init_finish(void);
long hantrodevice_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

void device_print_debug(void);

#endif /* __HANTRO_DEVICE_H__ */
