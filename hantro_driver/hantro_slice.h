/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro decoder hardware driver.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __HANTRO_SLICE_H__
#define __HANTRO_SLICE_H__

#include "hantro.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* supported core type */
typedef enum {
	CORE_UNKNOWN = -1,
	CORE_SLICE = 0,
	CORE_DEC,
	CORE_ENC,
	CORE_CACHE,
	CORE_DEC400,
} slice_coretype;

struct cache_core_config {
	cache_client_type client;
	unsigned long long base_addr;
	u32 iosize;
	int irq;
	driver_cache_dir dir;
	u32 sliceidx;
	unsigned long long parentaddr;
};

struct cache_dev_t {
	struct cache_core_config core_cfg; /* config of each core,such as base addr, irq,etc */
	unsigned long hw_id; /* hw id to indicate project */
	u32 core_id; /* core id for driver and sw internal use */
	u32 is_valid; /* indicate this core is hantro's core or not */
	u32 is_reserved; /* indicate this core is occupied by user or not */
	struct file *cacheowner; /* indicate which process is occupying the core */
	u32 irq_received; /* indicate this core receives irq */
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 *hwregs;
	char reg_name[32];
	unsigned long long com_base_addr; /* common base addr of each L2 */
	int irqlist[4];
	char irq_name[4][32];
	slice_coretype parenttype;
	u32 parentid; /* parent codec core's core_id */
	void *parentcore; /* either struct hantroenc_t or struct hantrodec_t, or slice itself */
	void *parentslice;
	int sliceidx;
	struct cache_dev_t *next;
};

struct dec400_core_cfg {
	unsigned long long dec400corebase;
	volatile unsigned int iosize;
	u32 sliceidx;
	unsigned long long parentaddr;
};

struct dec400_t {
	struct dec400_core_cfg core_cfg;

	u32 core_id;
	volatile u8 *hwregs;

	char reg_name[32];
	slice_coretype parenttype;
	u32 parentid; /* parent codec core's core_id */
	void *parentcore; /* either struct hantroenc_t or struct hantrodec_t, or slice itself */
	void *parentslice;
	struct dec400_t *next;
};

typedef struct {
	unsigned long long base_addr;
	u32 iosize;
	int irq;
	/*
	 * resouce_shared indicate core share resources with other cores.
	 * If 1, cores can not work at same time.
	 */
	u32 resouce_shared;
	u32 sliceidx;
} CORE_CONFIG;

struct hantroenc_t {
	CORE_CONFIG core_cfg; /* config of each core,such as base addr, irq,etc */
	u32 hw_id; /* hw id to indicate project */
	u32 core_id; /* core id for driver and sw internal use */
	u32 is_reserved; /* indicate this core is occupied by user or not */
	int pid; /* indicate which process is occupying the core */
	u32 irq_received; /* indicate this core receives irq */
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 *hwregs;
	char reg_name[32];
	struct fasync_struct *async_queue;
	int irqlist[4];
	char irq_name[4][32];
	void *parentslice;
	int sliceidx;
	struct hantroenc_t *next;
};

#define HANTRO_G1_DEC_REGS	155 /* G1 total regs */
#define HANTRO_G2_DEC_REGS	337 /* G2 total regs */
#define HANTRO_VC8000D_REGS	393 /* VC8000D total regs */
#define DEC_IO_SIZE_MAX \
	(MAX(MAX(HANTRO_G2_DEC_REGS, HANTRO_G1_DEC_REGS), \
	     HANTRO_VC8000D_REGS) * 4)

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
	u8 *hwregs;
	int hw_id;
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

	struct file *dec_owner;
	struct file *pp_owner;
	void *parentslice;
	u32 sliceidx;
	struct hantrodec_t *next;
};

/*
current internal slice data structure will look like this:
	slicehdr->	slice 0		-> slice 1 -> ...
				|
		________________________________________
		|				|			|			|
	dec core 0		enc core 0	dec400 0		cache core 0
		|				|			|			|
	dec core 1		enc core 1	dec400 1		cache core 1
		|				|			|			|
	.......				......		   .....		     .....

Each core node contains its own info: io region, irq, hwid, direction, etc., depending on its type.
Each dec400 core has pointer to a dec or enc core. Each dec/enc core has a pointer to dec400 core.
Each cache core has pointer to a dec or enc core. Each dec/enc core has pointers to dec400 core.
We do it this way since we don't know which one will be probed first, dec/enc or dec400/cache.
And only ID to connect dec/enc to dec400/cache core is their HW address.
*/

struct slice_info {
	struct device *dev; /* related dev, for drm usage */
	struct device *codec_rsvmem;
	phys_addr_t rsvmem_addr;
	phys_addr_t memsize;
	u32 config;
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
	spinlock_t cache_owner_lock;

	/* orig enc global vars */
	struct semaphore enc_core_sem;
	wait_queue_head_t enc_hw_queue;
	spinlock_t enc_owner_lock;
	wait_queue_head_t enc_wait_queue;

	/* orig dec global vars */
	int dec_irq;
	int pp_irq;
	spinlock_t owner_lock;
	wait_queue_head_t dec_wait_queue;
	wait_queue_head_t pp_wait_queue;
	wait_queue_head_t hw_queue;
	struct semaphore dec_core_sem;
	struct semaphore pp_core_sem;

	struct slice_info *next;
};

int findslice_bydev(struct device *dev);
int addslice(struct device *dev, phys_addr_t sliceaddr, phys_addr_t slicesize);
struct slice_info *getslicenode(u32 sliceindex);
struct slice_info *getslicenode_inInit(u32 sliceindex);
int get_slicecorenum(u32 sliceindex, slice_coretype type);
struct hantrodec_t *get_decnodes(u32 sliceindex, u32 nodeidx);
struct hantrodec_t *getfirst_decnodes(struct slice_info *pslice);
struct hantroenc_t *get_encnodes(u32 sliceindex, u32 nodeidx);
struct cache_dev_t *get_cachenodes(u32 sliceindex, u32 nodeidx);
struct cache_dev_t *get_cachenodebytype(u32 sliceindex, u32 parenttype,
					u32 parentnodeidx);
struct dec400_t *get_dec400nodes(u32 sliceindex, u32 nodeidx);
struct dec400_t *get_dec400nodebytype(u32 sliceindex, u32 parenttype,
				      u32 parentnodeidx);
int add_decnode(u32 sliceindex, struct hantrodec_t *deccore);
int add_encnode(u32 sliceindex, struct hantroenc_t *enccore);
int add_dec400node(u32 sliceindex, struct dec400_t *dec400core);
int add_cachenode(u32 sliceindex, struct cache_dev_t *cachecore);
int get_slicenumber(void);
struct slice_info *getparentslice(void *node, int type);
int slice_remove(void);
int slice_init(void);
void slice_init_finish(void);
long hantroslice_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

void slice_printdebug(void);

#endif /* __HANTRO_SLICE_H__ */
