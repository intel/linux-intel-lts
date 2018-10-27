// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Intel client SoC with integrated memory controller using IBECC
 *
 * Copyright (C) 2019 Intel Corporation
 *
 * The In-Band ECC (IBECC) IP provides ECC protection to all or specific
 * regions of the physical memory space. It's used for memory controllers
 * that don't support the out-of-band ECC which often needs an additional
 * storage device to each channel for storing ECC data. The first supported
 * platform is Ice Lake Neural Network Processor for Inference (ICL-NNPI).
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/nmi.h>
#include <linux/irq_work.h>
#include <linux/llist.h>
#include <linux/genalloc.h>
#include <linux/edac.h>
#include <linux/processor.h>
#include <linux/spinlock.h>
#include <linux/sched/clock.h>
#include <asm/cpu_device_id.h>
#include <asm/intel-family.h>

#include "edac_mc.h"
#include "edac_module.h"

#define IGEN6_REVISION	"v1.1.5"

#define EDAC_MOD_STR	"igen6_edac"
#define IGEN6_NMI_NAME	"igen6_ibecc"

/* Debug macros */
#define igen6_printk(level, fmt, arg...)		\
	edac_printk(level, "igen6", fmt, ##arg)

#define igen6_mc_printk(mci, level, fmt, arg...)	\
	edac_mc_chipset_printk(mci, level, "igen6", fmt, ##arg)

#define GET_BITFIELD(v, lo, hi) (((v) & GENMASK_ULL(hi, lo)) >> (lo))

#define NUM_CHANNELS			2 /* Max channels */
#define NUM_DIMMS			2 /* Max DIMMs per channel */

#define IGEN6_TOM_OFF			0xa0
#define IGEN6_TOUUD_OFF			0xa8
#define IGEN6_TOLUD_OFF			0xbc
#define IGEN6_CAPID_C_OFF		0xec
#define IGEN6_CAPID_C_IBECC		BIT(15)
#define _4GB				BIT_ULL(32)

#define IGEN6_ERRSTS_OFF		0xc8
#define IGEN6_ERRSTS_CE			BIT_ULL(6)
#define IGEN6_ERRSTS_UE			BIT_ULL(7)

#define IGEN6_ECC_BASE			(ibecc_cfg->ibecc_offset)
#define IGEN6_ECCACTIVATE_OFF		IGEN6_ECC_BASE
#define IGEN6_ECCACTIVATE_EN		BIT(0)

#define IGEN6_ECCERRLOG_OFF		(IGEN6_ECC_BASE + 0x170)
#define IGEN6_ECCERRLOG_CE		BIT_ULL(62)
#define IGEN6_ECCERRLOG_UE		BIT_ULL(63)
#define IGEN6_ECCERRLOG_ADDR_SHIFT	5
#define IGEN6_ECCERRLOG_ADDR(v)		GET_BITFIELD(v, 5, 38)
#define IGEN6_ECCERRLOG_SYND(v)		GET_BITFIELD(v, 46, 61)

#define IGEN6_MCHBAR_HI_OFF		0x4c
#define IGEN6_MCHBAR_LO_OFF		0x48
#define IGEN6_MCHBAR_EN			BIT_ULL(0)
#define IGEN6_MCHBAR_BASE(v)		(GET_BITFIELD(v, 16, 38) << 16)
#define IGEN6_MCHBAR_SIZE		0x10000

#define IGEN6_MAD_INTER_OFF		0x5000
#define IGEN6_MAD_INTRA_OFF		0x5004
#define IGEN6_MAD_DIMM_OFF		0x500c
#define IGEN6_HASH_OFF			0X5024
#define IGEN6_EHASH_OFF			0X5028

#define IGEN6_MAD_INTER_DDR_TYPE(v)	GET_BITFIELD(v, 0, 2)
#define IGEN6_MAD_INTER_ECHM(v)		GET_BITFIELD(v, 3, 3)
#define IGEN6_MAD_INTER_CH_L_MAP(v)	GET_BITFIELD(v, 4, 4)
#define IGEN6_MAD_INTER_CH_S_SIZE(v)	((u64)GET_BITFIELD(v, 12, 19) << 29)
#define IGEN6_MAD_INTRA_DIMM_L_MAP(v)	GET_BITFIELD(v, 0, 0)
#define IGEN6_MAD_INTRA_RI(v)		GET_BITFIELD(v, 4, 4)
#define IGEN6_MAD_INTRA_EIM(v)		GET_BITFIELD(v, 8, 8)
#define IGEN6_MAD_INTRA_ECC(v)		(GET_BITFIELD(v, 12, 13) == 0x3)

#define IGEN6_DIMM_CH_DIMM_L_SIZE(v)	((u64)GET_BITFIELD(v, 0, 6) << 29)
#define IGEN6_DIMM_CH_DLW(v)		GET_BITFIELD(v, 7, 8)
#define IGEN6_DIMM_CH_DLNOR(v)		(GET_BITFIELD(v, 9, 10) + 1)
#define IGEN6_DIMM_CH_DIMM_S_SIZE(v)	((u64)GET_BITFIELD(v, 16, 22) << 29)
#define IGEN6_DIMM_CH_DSW(v)		GET_BITFIELD(v, 24, 25)
#define IGEN6_DIMM_CH_DSNOR(v)		(GET_BITFIELD(v, 26, 27) + 1)
#define IGEN6_DIMM_CH_DLS_BG0(v)	GET_BITFIELD(v, 29, 29)

#define IGEN6_HASH_MASK(v)		(GET_BITFIELD(v, 6, 19) << 6)
#define IGEN6_HASH_LSB_MASK_BIT(v)	GET_BITFIELD(v, 24, 26)
#define IGEN6_HASH_MODE(v)		GET_BITFIELD(v, 28, 28)

#define igen6_getreg(type, offset)	\
	(*(type *)(igen6_pvt->mchbar + (offset)))
#define igen6_setreg(type, offset, val)	\
	(*(type *)(igen6_pvt->mchbar + (offset)) = (val))

static struct igen6_pvt {
	struct mem_ctl_info *mci;
	struct pci_dev *pdev;
	void __iomem *mchbar;
	u64 ch_s_size;
	int ch_l_map;
	u64 dimm_s_size[NUM_CHANNELS];
	u64 dimm_l_size[NUM_CHANNELS];
	int dimm_l_map[NUM_CHANNELS];
} *igen6_pvt;

/* The top of upper usable DRAM */
static u64 igen6_touud;
/* The top of low usable DRAM */
static u32 igen6_tolud;
/* The size of physical memory */
static u64 igen6_tom;

struct decoded_addr {
	u64 mem_addr;
	u64 sys_addr;
	u64 chan_addr;
	int chan;
	u64 sub_chan_addr;
	int sub_chan;
};

struct ecclog_node {
	struct llist_node llnode;
	u64 ecclog;
};

static struct ibecc_config {
	u32 ibecc_offset;
	bool (*ibecc_available)(u32 capid);
} *ibecc_cfg;

/*
 * An NMI is broadcast to all CPU cores on a CE/UE error on the ICL-NNPI
 * platform. Make sure only one concurrent NMI handler for it.
 */
static DEFINE_RAW_SPINLOCK(ecclog_lock);
static u64 last_handle_jiffies;
#define MAX_NMI_GAP_JIFFIES msecs_to_jiffies(8)

/*
 * printk() is not safe in NMI context. So in NMI handler, the driver uses
 * the lock-less memory alocator to allocate memory for ECC error log and
 * saves it to a lock-less list. Delay the printk() and the work of error
 * reporting to EDAC core in a worker.
 */
#define ECCLOG_POOLSZ	PAGE_SIZE
LLIST_HEAD(ecclog_llist);
static struct gen_pool *ecclog_pool;
static char ecclog_buf[ECCLOG_POOLSZ];
static struct irq_work ecclog_irq_work;
static struct work_struct ecclog_work;

/* Compute die IDs for ICL-NNPI with IBECC */
#define DID_ICL_SKU8	0x4581
#define DID_ICL_SKU10	0x4585
#define DID_ICL_SKU11	0x4589
#define DID_ICL_SKU12	0x458d

static bool icl_ibecc_available(u32 capid)
{
	/* Capid IBECC bit for ICL: 0 - available, 1 - unavailable */
	return !(IGEN6_CAPID_C_IBECC & capid) &&
		(boot_cpu_data.x86_stepping >= 1);
}

static struct ibecc_config icl_cfg = {
	.ibecc_offset		= 0xd800,
	.ibecc_available	= icl_ibecc_available,
};

static const struct pci_device_id igen6_pci_tbl[] = {
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU8), (kernel_ulong_t)&icl_cfg },
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU10), (kernel_ulong_t)&icl_cfg },
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU11), (kernel_ulong_t)&icl_cfg },
	{ PCI_VDEVICE(INTEL, DID_ICL_SKU12), (kernel_ulong_t)&icl_cfg },
	{ },
};
MODULE_DEVICE_TABLE(pci, igen6_pci_tbl);

static enum dev_type get_width(int dimm_l, u32 mad_dimm)
{
	u32 w = dimm_l ? IGEN6_DIMM_CH_DLW(mad_dimm) :
			 IGEN6_DIMM_CH_DSW(mad_dimm);

	switch (w) {
	case 0:
		return DEV_X8;
	case 1:
		return DEV_X16;
	case 2:
		return DEV_X32;
	default:
		return DEV_UNKNOWN;
	}
}

static enum mem_type get_memory_type(u32 mad_inter)
{
	u32 t = IGEN6_MAD_INTER_DDR_TYPE(mad_inter);

	switch (t) {
	case 0:
		return MEM_DDR4;
	case 1:
		return MEM_DDR3;
	case 2:
		return MEM_LPDDR3;
	case 3:
		return MEM_LPDDR4;
	case 4:
		return MEM_WIO2;
	default:
		return MEM_UNKNOWN;
	}
}

static u64 convert_saddr_to_maddr(u64 addr)
{
	if (addr < igen6_tolud)
		return addr;

	if (igen6_tom <= _4GB)
		return addr + igen6_tolud - _4GB;

	if (addr < _4GB)
		return addr + igen6_tolud - igen6_tom;

	return addr;
}

static int decode_chan_idx(u64 addr, u64 mask, int intlv_bit)
{
	u64 hash_addr = addr & mask, hash = 0;
	u64 intlv = (addr >> intlv_bit) & 1;
	int i;

	for (i = 6; i < 20; i++)
		hash ^= (hash_addr >> i) & 1;

	return (int)hash ^ intlv;
}

static u64 decode_chan_addr(u64 addr, int intlv_bit)
{
	u64 chan_addr;

	/* Remove the interleave bit and shift upper part down to fill gap */
	chan_addr  = GET_BITFIELD(addr, intlv_bit + 1, 63) << intlv_bit;
	chan_addr |= GET_BITFIELD(addr, 0, intlv_bit - 1);

	return chan_addr;
}

static void decode_addr(u64 addr, u32 hash, u64 s_size, int l_map,
			int *sel, u64 *sub_addr)
{
	int intlv_bit = IGEN6_HASH_LSB_MASK_BIT(hash) + 6;

	if (addr > 2 * s_size) {
		*sub_addr = addr - s_size;
		*sel = l_map;
		return;
	}

	if (IGEN6_HASH_MODE(hash)) {
		*sub_addr = decode_chan_addr(addr, intlv_bit);
		*sel = decode_chan_idx(addr, IGEN6_HASH_MASK(hash), intlv_bit);
	} else {
		*sub_addr = decode_chan_addr(addr, 6);
		*sel = GET_BITFIELD(addr, 6, 6);
	}
}

static int igen6_decode(struct decoded_addr *res)
{
	u64 addr = res->mem_addr, sub_addr, s_size;
	struct igen6_pvt *pvt = igen6_pvt;
	int sel, l_map;
	u32 hash;

	if (addr >= igen6_tom) {
		edac_dbg(0, "Address 0x%llx out of range\n", addr);
		return -EINVAL;
	}

	/* Decode channel */
	hash	= igen6_getreg(u32, IGEN6_HASH_OFF);
	s_size	= pvt->ch_s_size;
	l_map	= pvt->ch_l_map;
	decode_addr(addr, hash, s_size, l_map, &sel, &sub_addr);
	res->chan	= sel;
	res->chan_addr	= sub_addr;

	/* Decode sub-channel/DIMM */
	hash	= igen6_getreg(u32, IGEN6_EHASH_OFF);
	s_size	= pvt->dimm_s_size[sel];
	l_map	= pvt->dimm_l_map[sel];
	decode_addr(res->chan_addr, hash, s_size, l_map, &sel, &sub_addr);
	res->sub_chan	   = sel;
	res->sub_chan_addr = sub_addr;

	return 0;
}

static void igen6_output_error(struct decoded_addr *res, u64 ecclog)
{
	enum hw_event_mc_err_type type = ecclog & IGEN6_ECCERRLOG_UE ?
					 HW_EVENT_ERR_UNCORRECTED :
					 HW_EVENT_ERR_CORRECTED;

	edac_mc_handle_error(type, igen6_pvt->mci, 1,
			     res->sys_addr >> PAGE_SHIFT,
			     res->sys_addr & ~PAGE_MASK,
			     IGEN6_ECCERRLOG_SYND(ecclog),
			     res->chan, res->sub_chan,
			     -1, "", "");
}

static struct gen_pool *ecclog_gen_pool_create(void)
{
	struct gen_pool *pool;

	pool = gen_pool_create(ilog2(sizeof(struct ecclog_node)), -1);
	if (!pool)
		return NULL;

	if (gen_pool_add(pool, (unsigned long)ecclog_buf, ECCLOG_POOLSZ, -1)) {
		gen_pool_destroy(pool);
		return NULL;
	}

	return pool;
}

static int ecclog_gen_pool_add(u64 ecclog)
{
	struct ecclog_node *node;

	node = (void *)gen_pool_alloc(ecclog_pool, sizeof(*node));
	if (!node)
		return -ENOMEM;

	node->ecclog = ecclog;
	llist_add(&node->llnode, &ecclog_llist);

	return 0;
}

static u64 ecclog_read(void)
{
	u64 ecclog = igen6_getreg(u64, IGEN6_ECCERRLOG_OFF);

	if (ecclog & (IGEN6_ECCERRLOG_CE | IGEN6_ECCERRLOG_UE))
		return ecclog;

	return 0;
}

static void ecclog_clear(u64 ecclog)
{
	/* Clear CE/UE bits in IBECC register by writing 1 to it */
	ecclog |= IGEN6_ECCERRLOG_CE | IGEN6_ECCERRLOG_UE;
	igen6_setreg(u64, IGEN6_ECCERRLOG_OFF, ecclog);
}

static void errsts_clear(void)
{
	u16 errsts;

	if (pci_read_config_word(igen6_pvt->pdev, IGEN6_ERRSTS_OFF, &errsts)) {
		igen6_printk(KERN_ERR, "Failed to read ERRSTS\n");
		return;
	}

	if (!(errsts & (IGEN6_ERRSTS_CE | IGEN6_ERRSTS_UE)))
		return;

	/* Clear CE/UE bits in PCI ERRSTS register by writing 1 to it */
	errsts |= IGEN6_ERRSTS_CE | IGEN6_ERRSTS_UE;
	pci_write_config_word(igen6_pvt->pdev, IGEN6_ERRSTS_OFF, errsts);
}

static u64 ecclog_check(void)
{
	u64 ecclog = ecclog_read();

	if (ecclog)
		ecclog_clear(ecclog);
		/* errsts_clear() is not NMI safe, delay it in irq_work */

	return ecclog;
}

static void ecclog_work_cb(struct work_struct *work)
{
	struct mem_ctl_info *mci = igen6_pvt->mci;
	struct ecclog_node *node, *tmp;
	struct llist_node *head;
	struct decoded_addr res;

	head = llist_del_all(&ecclog_llist);
	if (!head)
		return;

	llist_for_each_entry_safe(node, tmp, head, llnode) {
		memset(&res, 0, sizeof(res));
		res.sys_addr = IGEN6_ECCERRLOG_ADDR(node->ecclog) <<
			       IGEN6_ECCERRLOG_ADDR_SHIFT;
		res.mem_addr = convert_saddr_to_maddr(res.sys_addr);

		edac_dbg(2, "ecc_error_log = 0x%llx\n", node->ecclog);
		igen6_mc_printk(mci, KERN_DEBUG, "HANDLING IBECC MEMORY ERROR\n");
		igen6_mc_printk(mci, KERN_DEBUG, "ADDR 0x%llx ", res.sys_addr);

		if (!igen6_decode(&res))
			igen6_output_error(&res, node->ecclog);

		gen_pool_free(ecclog_pool, (unsigned long)node, sizeof(*node));
	}
}

static void ecclog_irq_work_cb(struct irq_work *irq_work)
{
	errsts_clear();

	if (!llist_empty(&ecclog_llist))
		schedule_work(&ecclog_work);
}

static int ecclog_nmi_handler(unsigned int cmd, struct pt_regs *regs)
{
	u64 delta, ecclog;

	raw_spin_lock(&ecclog_lock);

	ecclog = ecclog_check();
	if (!ecclog) {
		delta = jiffies - last_handle_jiffies;
		raw_spin_unlock(&ecclog_lock);
		/*
		 * When a CE/UE error occurs, an NMI is delivered to all CPU
		 * cores. Only one core handles the error, and the rest cores
		 * see no error so that they complain they receive NMIs for
		 * unknown reason. A workaround for the complaint is to get a
		 * core to see if another core had "recently" handled the error.
		 * If it did, then return value from the handler could be faked
		 * to say this core handled one too.
		 */
		return delta < MAX_NMI_GAP_JIFFIES ? NMI_HANDLED : NMI_DONE;
	}

	if (!ecclog_gen_pool_add(ecclog))
		irq_work_queue(&ecclog_irq_work);
	last_handle_jiffies = jiffies;

	raw_spin_unlock(&ecclog_lock);

	return NMI_HANDLED;
}

static bool igen6_check_ecc(void)
{
	u32 activate = igen6_getreg(u32, IGEN6_ECCACTIVATE_OFF);

	return !!(activate & IGEN6_ECCACTIVATE_EN);
}

static int igen6_get_dimm_config(struct mem_ctl_info *mci)
{
	struct igen6_pvt *pvt = mci->pvt_info;
	u32 mad_inter, mad_intra, mad_dimm;
	int i, j, ndimms, tot_dimms = 0;
	struct dimm_info *dimm;
	enum mem_type mtype;
	enum dev_type dtype;
	u64 dsize;
	bool ecc;

	mad_inter = igen6_getreg(u32, IGEN6_MAD_INTER_OFF);
	mtype = get_memory_type(mad_inter);
	ecc = igen6_check_ecc();
	pvt->ch_s_size = IGEN6_MAD_INTER_CH_S_SIZE(mad_inter);
	pvt->ch_l_map  = IGEN6_MAD_INTER_CH_L_MAP(mad_inter);

	for (i = 0; i < NUM_CHANNELS; i++) {
		mad_intra = igen6_getreg(u32, IGEN6_MAD_INTRA_OFF + i * 4);
		mad_dimm  = igen6_getreg(u32, IGEN6_MAD_DIMM_OFF + i * 4);

		pvt->dimm_l_size[i] = IGEN6_DIMM_CH_DIMM_L_SIZE(mad_dimm);
		pvt->dimm_s_size[i] = IGEN6_DIMM_CH_DIMM_S_SIZE(mad_dimm);
		pvt->dimm_l_map[i]  = IGEN6_MAD_INTRA_DIMM_L_MAP(mad_intra);
		ndimms = 0;

		for (j = 0; j < NUM_DIMMS; j++) {
			dimm = EDAC_DIMM_PTR(mci->layers, mci->dimms,
					     mci->n_layers, i, j, 0);

			if (j ^ pvt->dimm_l_map[i]) {
				dtype = get_width(0, mad_dimm);
				dsize = pvt->dimm_s_size[i];
			} else {
				dtype = get_width(1, mad_dimm);
				dsize = pvt->dimm_l_size[i];
			}

			if (!dsize)
				continue;

			dimm->grain = 32;
			dimm->mtype = mtype;
			dimm->dtype = dtype;
			dimm->nr_pages  = MiB_TO_PAGES(dsize >> 20);
			dimm->edac_mode = EDAC_SECDED;
			snprintf(dimm->label, sizeof(dimm->label),
				 "Chan#%d_DIMM#%d", i, j);
			edac_dbg(0, "Channel %d, DIMM %d, Size %llu MiB (%u pages)\n",
				 i, j, dsize >> 20, dimm->nr_pages);

			ndimms++;
		}

		if (ndimms && !ecc) {
			igen6_printk(KERN_ERR, "ECC is disabled\n");
			return -ENODEV;
		}

		tot_dimms += ndimms;
	}

	if (!tot_dimms) {
		igen6_printk(KERN_ERR, "No DIMMs found\n");
		return -ENODEV;
	}

	return 0;
}

static void __iomem *igen6_pci_setup(struct pci_dev *pdev)
{
	union  {
		u64 v;
		struct {
			u32 v_lo;
			u32 v_hi;
		};
	} u;
	void __iomem *mchbar;

	edac_dbg(2, "\n");

	if (pci_enable_device(pdev)) {
		igen6_printk(KERN_ERR, "Failed to enable device %04x:%04x\n",
			     pdev->vendor, pdev->device);
		return NULL;
	}

	if (pci_read_config_dword(pdev, IGEN6_CAPID_C_OFF, &u.v_lo)) {
		igen6_printk(KERN_ERR, "Failed to read CAPID_C\n");
		goto fail;
	}
	if (!ibecc_cfg->ibecc_available(u.v_lo)) {
		edac_dbg(2, "No In-Band ECC IP\n");
		goto fail;
	}

	if (pci_read_config_dword(pdev, IGEN6_TOUUD_OFF, &u.v_lo)) {
		igen6_printk(KERN_ERR, "Failed to read TOUUD low part\n");
		goto fail;
	}
	if (pci_read_config_dword(pdev, IGEN6_TOUUD_OFF + 4, &u.v_hi)) {
		igen6_printk(KERN_ERR, "Failed to read TOUUD high part\n");
		goto fail;
	}
	igen6_touud = u.v & GENMASK_ULL(38, 20);

	if (pci_read_config_dword(pdev, IGEN6_TOLUD_OFF, &igen6_tolud)) {
		igen6_printk(KERN_ERR, "Failed to read TOLUD\n");
		goto fail;
	}
	igen6_tolud &= GENMASK(31, 20);

	if (pci_read_config_dword(pdev, IGEN6_TOM_OFF, &u.v_lo)) {
		igen6_printk(KERN_ERR, "Failed to read TOM low part\n");
		goto fail;
	}
	if (pci_read_config_dword(pdev, IGEN6_TOM_OFF + 4, &u.v_hi)) {
		igen6_printk(KERN_ERR, "Failed to read TOM high part\n");
		goto fail;
	}
	igen6_tom = u.v & GENMASK_ULL(38, 20);

	if (pci_read_config_dword(pdev, IGEN6_MCHBAR_LO_OFF, &u.v_lo)) {
		igen6_printk(KERN_ERR, "Failed to read MCHBAR\n");
		goto fail;
	}
	if (pci_read_config_dword(pdev, IGEN6_MCHBAR_HI_OFF, &u.v_hi)) {
		igen6_printk(KERN_ERR, "Failed to read MCHBAR1\n");
		goto fail;
	}
	if (!(u.v & IGEN6_MCHBAR_EN)) {
		igen6_printk(KERN_ERR, "MCHBAR is disabled\n");
		goto fail;
	}
	mchbar = ioremap_nocache(IGEN6_MCHBAR_BASE(u.v), IGEN6_MCHBAR_SIZE);
	if (!mchbar) {
		igen6_printk(KERN_ERR, "Failed to ioremap mchbar 0x%llx\n",
			     IGEN6_MCHBAR_BASE(u.v));
		goto fail;
	}

	return mchbar;
fail:
	pci_disable_device(pdev);
	return NULL;
}

#ifdef CONFIG_EDAC_DEBUG
static void igen6_reg_dump(void)
{
	int i;

	edac_dbg(2, "Hash	: 0x%x\n",
		 igen6_getreg(u32, IGEN6_HASH_OFF));
	edac_dbg(2, "Ehash	: 0x%x\n",
		 igen6_getreg(u32, IGEN6_EHASH_OFF));
	edac_dbg(2, "Mad_inter	: 0x%x\n",
		 igen6_getreg(u32, IGEN6_MAD_INTER_OFF));
	edac_dbg(2, "Eccerrlog	: 0x%llx\n",
		 igen6_getreg(u64, IGEN6_ECCERRLOG_OFF));

	for (i = 0; i < NUM_CHANNELS; i++) {
		edac_dbg(2, "Mad_intra_%d : 0x%x\n", i,
			 igen6_getreg(u32, IGEN6_MAD_INTRA_OFF + i * 4));
		edac_dbg(2, "Mad_dimm_%d  : 0x%x\n", i,
			 igen6_getreg(u32, IGEN6_MAD_DIMM_OFF + i * 4));
	}
	edac_dbg(2, "Touud	: 0x%llx", igen6_touud);
	edac_dbg(2, "Tolud	: 0x%x", igen6_tolud);
	edac_dbg(2, "Tom	: 0x%llx", igen6_tom);
}
#else
static void igen6_reg_dump(void) {}
#endif

static int igen6_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct edac_mc_layer layers[2];
	struct mem_ctl_info *mci;
	void __iomem *mchbar;
	struct igen6_pvt *pvt;
	int rc = -ENODEV;
	u64 ecclog;

	edac_dbg(2, "\n");

	ibecc_cfg = (struct ibecc_config *)ent->driver_data;
	mchbar = igen6_pci_setup(pdev);
	if (!mchbar)
		return -ENODEV;

	layers[0].type = EDAC_MC_LAYER_CHANNEL;
	layers[0].size = NUM_CHANNELS;
	layers[0].is_virt_csrow = false;
	layers[1].type = EDAC_MC_LAYER_SLOT;
	layers[1].size = NUM_DIMMS;
	layers[1].is_virt_csrow = true;

	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers, sizeof(*pvt));
	if (!mci) {
		rc = -ENOMEM;
		goto fail;
	}

	mci->ctl_name = "Intel_client_SoC";
	mci->mtype_cap = MEM_FLAG_LPDDR4 | MEM_FLAG_DDR4;
	mci->edac_ctl_cap = EDAC_FLAG_SECDED;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->mod_name = EDAC_MOD_STR;
	mci->dev_name = pci_name(pdev);
	mci->pdev = &pdev->dev;
	pvt = mci->pvt_info;
	pvt->mci = mci;
	pvt->mchbar = mchbar;
	pvt->pdev = pdev;
	igen6_pvt = pvt;

	igen6_reg_dump();

	rc = igen6_get_dimm_config(mci);
	if (rc)
		goto fail0;

	rc = edac_mc_add_mc(mci);
	if (rc) {
		igen6_printk(KERN_ERR, "Failed to register mci\n");
		goto fail0;
	}

	ecclog_pool = ecclog_gen_pool_create();
	if (!ecclog_pool) {
		rc = -ENOMEM;
		goto fail1;
	}

	INIT_WORK(&ecclog_work, ecclog_work_cb);
	init_irq_work(&ecclog_irq_work, ecclog_irq_work_cb);

	/* Check if any pending error before registering the NMI handler */
	ecclog = ecclog_check();
	if (ecclog) {
		if (!ecclog_gen_pool_add(ecclog))
			irq_work_queue(&ecclog_irq_work);
		last_handle_jiffies = jiffies;
	}

	rc = register_nmi_handler(NMI_LOCAL, ecclog_nmi_handler,
				  0, IGEN6_NMI_NAME);
	if (rc) {
		igen6_printk(KERN_ERR, "Failed to register nmi handler\n");
		goto fail2;
	}

	return 0;

fail2:
	gen_pool_destroy(ecclog_pool);
fail1:
	edac_mc_del_mc(mci->pdev);
fail0:
	edac_mc_free(mci);
fail:
	iounmap(mchbar);
	return rc;
}

static void igen6_remove(struct pci_dev *pdev)
{
	struct mem_ctl_info *mci;
	struct igen6_pvt *pvt;

	edac_dbg(2, "\n");

	unregister_nmi_handler(NMI_LOCAL, IGEN6_NMI_NAME);
	irq_work_sync(&ecclog_irq_work);
	flush_work(&ecclog_work);
	gen_pool_destroy(ecclog_pool);
	mci = edac_mc_del_mc(&pdev->dev);
	if (!mci) {
		edac_dbg(0, "mci should not be null\n");
		return;
	}
	pvt = mci->pvt_info;
	edac_mc_free(mci);
	iounmap(pvt->mchbar);
	pci_disable_device(pdev);
}

static struct pci_driver igen6_driver = {
	.name     = EDAC_MOD_STR,
	.probe    = igen6_probe,
	.remove   = igen6_remove,
	.id_table = igen6_pci_tbl,
};

static int __init igen6_init(void)
{
	const char *owner;
	int rc;

	edac_dbg(2, "\n");

	owner = edac_get_owner();
	if (owner && strncmp(owner, EDAC_MOD_STR, sizeof(EDAC_MOD_STR)))
		return -ENODEV;

	edac_op_state = EDAC_OPSTATE_NMI;

	rc = pci_register_driver(&igen6_driver);
	if (rc)
		return rc;

	igen6_printk(KERN_INFO, "%s\n", IGEN6_REVISION);

	return 0;
}

static void __exit igen6_exit(void)
{
	edac_dbg(2, "\n");

	pci_unregister_driver(&igen6_driver);
}

module_init(igen6_init);
module_exit(igen6_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Qiuxu Zhuo");
MODULE_DESCRIPTION("MC Driver for Intel client SoC using In-Band ECC");
