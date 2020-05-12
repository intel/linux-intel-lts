/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2020 Intel Corporation */

#ifndef __GNA_HW_H__
#define __GNA_HW_H__

#include <linux/io.h>
#include <linux/module.h>

/* GNA  PCI registers */
#define GNA_PCI_DCTRL		0x04
#define GNA_PCI_CLS		0x0C
#define GNA_PCI_BA		0x10
#define GNA_PCI_SSVI		0x2C
#define GNA_PCI_SSI		0x2E
#define GNA_PCI_INTL		0x3C
#define GNA_PCI_OVRCFGCTL	0x40
#define GNA_PCI_MC		0x92
#define GNA_PCI_MA		0x94
#define GNA_PCI_MD		0x98
#define GNA_PCI_D0I3_PWRCTL	0xB2
#define GNA_PCI_PMCS		0xE0

/* GNA MMIO registers */
#define GNASTS		0x80
#define GNACTRL		0x84
#define GNAMCTL		0x88
#define GNAPTC		0x8C
#define GNAPSC		0x90
#define GNAISI		0x94
#define GNAPISV		0x98
#define GNABP		0xA0
#define GNAD0I3C	0xA8
#define GNADESBASE	0xB0
#define GNAPWRCTRL	0xB2
#define GNAIBUFFS	0xB4

/* Register bit Shift */
#define GNA_CTRL_ABORT_BIT	BIT(2)

#define GNA_D0I3_POWER_OFF	(1 << 2)
#define GNA_D0I3_POWER_ON	0

#define GNA_INTERRUPT (GNA_STS_SCORE_COMPLETED | \
		GNA_STS_PCI_DMA_ERR | \
		GNA_STS_PCI_MMU_ERR | \
		GNA_STS_PCI_UNEXCOMPL_ERR | \
		GNA_STS_PARAM_OOR | \
		GNA_STS_VA_OOR)

#define GNA_PT_ENTRY_SIZE		4

/* page entries alignment for correct HW prefetching */
#define GNA_PREFETCH_ALIGNMENT		64

/* additional page entries for correct HW prefetching */
#define GNA_PREFETCH_ENTRIES		32

/* there are up to 1024 32-bit pointers in one page in Page Table (L1) */
#define GNA_PAGE_TABLE_LENGTH           (PAGE_SIZE / GNA_PT_ENTRY_SIZE)

/* minimum size of XNN layer descriptors in bytes (at least 1 layer) */
#define XNN_LYR_DSC_SIZE		(128)

#define GMM_CFG_SIZE			(128)

#define GNA_CFG_OFFSET			0x100

#define GNA_VAMAXADDR_OFFSET		0x200

#define GNA_PGDIRN_OFFSET		0x210

#define GNA_PGDIRN_LEN			64

#define GNA_PGDIR_ENTRIES		1024 /* 32-bit page addresses */

#define GNA_PGDIR_INVALID		1

union gna_ctrl_reg {
	struct reg {
		u32 start_accel:1; /* start accelerator */
		u32 active_list_en:1; /* active list enable */
		u32 abort_clr_accel:1; /* abort/clear accelerator */
		u32 pause_accel:1; /* pause execution */
		u32 resume_accel:1; /* resume execution */
		u32 gna_mode:2; /* mode (0:GMM, 1:xNN) */
		u32 __res_7:1; /* reserved */
		u32 compl_int_en:1; /* completion interrupt EN */
		u32 bp_pause_int_en:1; /* BP pause interrupt enable */
		u32 err_int_en:1; /* error interrupt enable */
		u32 __res_11:1; /* reserved */
		u32 comp_stats_en:4; /* compute statistics enable */
		u32 pm_ovr_power_on:1; /* PM override power on */
		u32 pm_ovr_clock_on:1; /* PM override force clck on */
		u32 pm_quite_idle_dis:1; /* PM quite-idle disable */
		u32 __res_23_19:5; /* reserved */
		u32 __res_31_24:8; /* reserved */
	} ctrl;
	u32 val; /* value of whole register */
};

struct gna_mmu_info {
	u32 vamax_size;
	u32 rsvd_size;
	u32 pd_size;
};

struct gna_desc_info {
	u32 rsvd_size;
	u32 cfg_size;
	u32 desc_size;
	struct gna_mmu_info mmu_info;
};

struct gna_private;

struct gna_compute_cfg;

extern void gna_debug_isi(struct gna_private *gna_priv, void __iomem *addr);

extern void gna_abort_hw(struct gna_private *gna_priv, void __iomem *addr);

extern void gna_start_scoring(struct gna_private *gna_priv, void __iomem *addr,
			      struct gna_compute_cfg *compute_cfg);

#define gna_reg_read(addr, offset) \
readl((addr) + (offset))
#define gna_reg_write(addr, offset, value) \
writel((value), (addr) + (offset))

#endif // __GNA_HW_H__
