/*
 * Copyright(C) 2018 Verisilicon
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef HANTRO_SLICE_H
#define HANTRO_SLICE_H

#include "hantro.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MULTI_SLICE_LIMIT	8		//MAX slice supported now
#define MAX_CACHE_PERCORE	2		//MAX cache number connected with a core

#define NODE(id) ((u32)(id)&0xff)
#define SLICE(id) ((u32)(id)>>16)

/*supported core type*/
typedef enum {
	CORE_DEC,
	CORE_ENC,
	CORE_CACHE,
	CORE_DEC400,
}slice_coretype;

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
	struct cache_core_config core_cfg; //config of each core,such as base addr, irq,etc
	unsigned long hw_id; //hw id to indicate project
	u32 core_id; //core id for driver and sw internal use
	u32 is_valid; //indicate this core is hantro's core or not
	u32 is_reserved; //indicate this core is occupied by user or not
	int pid; //indicate which process is occupying the core
	u32 irq_received; //indicate this core receives irq
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 *hwregs;
	unsigned long long com_base_addr;//common base addr of each L2
	int irqlist[4];

	void *parentcore;	//either struct hantroenc_t or struct hantrodec_t;

	struct cache_dev_t *next;
};

typedef struct {
	unsigned long long base_addr;
	u32 iosize;
	int irq;
	//resouce_shared indicate core share resources with other cores.
	//If 1, cores can not work at same time.
	u32 resouce_shared;
	u32 sliceidx;
} CORE_CONFIG;

struct hantroenc_t {
	CORE_CONFIG  core_cfg; //config of each core,such as base addr, irq,etc
	u32 hw_id; //hw id to indicate project
	u32 core_id; //core id for driver and sw internal use
	u32 is_reserved; //indicate this core is occupied by user or not
	int pid; //indicate which process is occupying the core
	u32 irq_received; //indicate this core receives irq
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 *hwregs;
	struct fasync_struct *async_queue;
	int irqlist[4];

	void *dec400;
	struct cache_dev_t *cachecore[2];

	struct hantroenc_t *next;
};

#define HANTRO_G1_DEC_REGS            155 /*G1 total regs*/
#define HANTRO_G2_DEC_REGS                 337 /*G2 total regs*/
#define HANTRO_VC8000D_REGS             393 /*VC8000D total regs*/
#define DEC_IO_SIZE_MAX                 \
	(MAX(MAX(HANTRO_G2_DEC_REGS, HANTRO_G1_DEC_REGS), \
		HANTRO_VC8000D_REGS)*4)

struct hantrodec_t {
	u32 cfg;
	int core_id;
	unsigned int iosize;
	u32 cfg_backup;
	/* indicate if main core exist */
	struct hantrodec_t *its_main_core_id;
	/* indicate if aux core exist */
	struct hantrodec_t *its_aux_core_id;
	/*all access to hwregs are through readl/writel
	 * so volatile is removed according to doc "volatile is evil"
	 */
	u8 *hwregs;
	int hw_id;

	unsigned long long multicorebase;
	/* Because one core may contain multi-pipeline,
	 * so multicore base may be changed
	 */
	unsigned long long multicorebase_actual;

	u32 dec_regs[DEC_IO_SIZE_MAX / 4];
	int irqlist[4];

	u32 sliceidx;
	void *dec400;
	struct cache_dev_t *cachecore[2];

	struct file *dec_owner;
	struct file *pp_owner;
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
	int deccore_num;
	int enccore_num;
	int dec400core_num;
	int cachecore_num;

	struct hantrodec_t *dechdr;
	struct hantroenc_t *enchdr;
	struct cache_dev_t *cachehdr;
	/*fixme: after merge dec400 code*/
	void *dec400hdr;

	/* orig cache global vars*/
	wait_queue_head_t cache_hw_queue;
	wait_queue_head_t cache_wait_queue;
	spinlock_t cache_owner_lock;

	/*orig enc global vars*/
	struct semaphore enc_core_sem;
	wait_queue_head_t enc_hw_queue;
	spinlock_t enc_owner_lock;
	wait_queue_head_t enc_wait_queue;

	/*orig dec global vars*/
	int dec_irq;
	int pp_irq;
	spinlock_t owner_lock;
	wait_queue_head_t dec_wait_queue;
	wait_queue_head_t pp_wait_queue;
	wait_queue_head_t hw_queue;
	struct semaphore dec_core_sem;
	struct semaphore pp_core_sem;
};

int get_slicecorenum(u32 sliceindex, slice_coretype type);
struct hantrodec_t *get_decnodes(u32 sliceindex, u32 nodeidx);
struct hantroenc_t *get_encnodes(u32 sliceindex, u32 nodeidx);
struct cache_dev_t *get_cachenodes(u32 sliceindex, u32 nodeidx);
void *get_dec400nodes(u32 sliceindex);
int add_decnode(u32 sliceindex, struct hantrodec_t *deccore);
int add_encnode(u32 sliceindex, struct hantroenc_t *enccore);
int add_dec400node(u32 sliceindex, void *dec400core);
int add_cachenode(u32 sliceindex, struct cache_dev_t *cachecore);
int get_slicenumber(void);
struct slice_info *getparentslice(void *node, int type);
int slice_remove(void);
int slice_init(void);
long hantroslice_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

void slice_printdebug(void);

#endif 	/*#define HANTRO_SLICE_H*/

