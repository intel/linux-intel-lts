// SPDX-License-Identifier: GPL-2.0-only
/*
 * hstd.c - Host STL Driver
 *
 * Copyright (c) 2020, Intel Corp.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/security.h>
#include <linux/highmem.h>
#include <linux/io.h>

#define DRV_NAME "hstd"
#define DRV_VERSION "1.1"
#define PFX DRV_NAME ": "
#define  DEVICE_NAME "hstd"
#define  CLASS_NAME  "hstd"

#define PCIEXBAR_LO_0_0_0	    0x60U
#define PCIEXBAR_HI_0_0_0	    0x64U
#define SA_DEV	0U
#define SA_FUNC 0U
#define B_PCIEXBAR		    0x3FF0000000ULL
#define B_PCIEXBAREN		    BIT(0)
#define V_PCIEXBAR_EN		    0x1ULL
#define PCI_DEVFN(slot, func)	 ((((slot) & 0x1f) << 3) | ((func) & 0x07))


#define SBREG_BAR (0xFD000000U)
#define B_RBA_MASK 0xFFFFFFFFFF000000ULL
#define SB_PF		((SBREG_BAR & B_RBA_MASK) >> PAGE_SHIFT)
#define SBBAR_SIZE	0x1000000U

#define PCIE_BUS0_PGOFF	0x0U
#define PCIE_BUS_SIZE		0x100000U
#define PCIE_FUNCTION_MMIO_SIZE 0x1000U

#define TSN_GBE_DEV  (30U)
#define TSN_GBE_FUNC (4U)
#define TSN_GBE_BAR_SIZE	    0x20000U

#define OSE_TSN_GBE_0_DEV  (29U)
#define OSE_TSN_GBE_0_FUNC (1U)
#define OSE_TSN_GBE_0_BAR_SIZE		  0x20000U

#define OSE_TSN_GBE_1_DEV  (29U)
#define OSE_TSN_GBE_1_FUNC (2U)
#define OSE_TSN_GBE_1_BAR_SIZE		  0x20000U

#define MCHBAR_OFFSET	(0x48U)
#define MCHBAR_SIZE  0x10000U
#define B_MCHBAR_MASK  0x7FFFFF8000ULL

#define IEH_DEV   (16U)
#define IEH_FUNC  (5U)

#define FUNCTION_BIT_OFFSET (12U)
#define DEVICE_BIT_OFFSET (15U)
#define PCIE_ADDR_BUS0(Device, Function, Offset) \
	((((uint32_t) Device)   << DEVICE_BIT_OFFSET) \
	| (((uint32_t) Function) << FUNCTION_BIT_OFFSET) \
	| ((uint32_t) Offset))

#define BDF_TO_PGOFF(Device, Function, Offset) \
	(PCIE_ADDR_BUS0(Device, Function, Offset) >> PAGE_SHIFT)

#define	N_MMIO_ADDRS	7U
#define	N_MSR_ADDRS	  23U

#define IA32_PERF_GLOBAL_CTRL		0x38F
#define IA32_PERF_GLOBAL_STATUS		0x38E
#define	IA32_CR_PERF_GLOBAL_OVF_CTRL	0x390
#define	IA32_FIXED_CTR_CTRL		0x38D
#define	IA32_FIXED_CTR1			0x30A
#define	IA32_PERFMON_GP_CTR0		0xC1
#define	IA32_PERFEVTSEL0		0x186
#define	IA32_MC1_STATUS			0x405
#define IA32_MC2_STATUS			0x409
#define IA32_MC3_STATUS			0x40D
#define	IA_MICROCODE			0x8B
#define	IA32_MC2_CTL2	    0x282
#define IA32_MC3_CTL	    0x40C
#define IA32_MC1_CTL	    0x404
#define IA32_MC1_CTL2	    0x281
#define IA32_MC0_CTL	    0x400
#define IA32_MCG_CAP	    0x179
#define UNCORE_MC06_CTL     0x418
#define UNCORE_MC07_CTL     0x41C
#define UNCORE_MC08_CTL     0x420
#define UNCORE_MC06_STATUS  0x419
#define UNCORE_MC07_STATUS  0x41D
#define UNCORE_MC08_STATUS  0x421

#define ALL_64BITS (~(uint64_t)0)
#define IA_MICROCODE_WR_V	0x0U
#define GP0FIXED1_COUNTER_CONFIG	(BIT(0) | BIT(33))
#define GP0FIXED1_COUNTER_STOP		~(GP0FIXED1_COUNTER_CONFIG)
#define FIXED1_COUNTER_CONFIG		(BIT(4) | BIT(5))
#define GP0_COUNTER_CONFIG		0x143013CU
#define RESET_COUNTER			0x0U
#define IA_MICROCODE_BITS		0x0

static int hstd_major_num;
static struct class *hstd_class;
static struct device *hstd_dev;
static uint64_t phy_pcie_base;
static DEFINE_SPINLOCK(phy_pcie_base_lock);

struct mmio_address_range {
	uint32_t start_vm_pgoff;
	uint32_t map_size;
	uint8_t is_bar;
};
static struct mmio_address_range mmioaddrs[N_MMIO_ADDRS];

enum read_write_code {
	RO = 0,
	RW = 1
};

struct msr_table {
	uint32_t addr;
	uint8_t rw;
	int (*check)(uint64_t v_read, uint64_t v_write);
};

static int     hstd_open(struct inode *, struct file *);
static int     hstd_release(struct inode *, struct file *);

static inline int global_ctrl_check(uint64_t p, uint64_t v)
{
	uint64_t m = ~(BIT(0) | BIT(33));
	uint64_t v_wr = v & m;
	uint64_t v_rd = p & m;

	if (!(v_rd ^ v_wr))
		return 0;
	else
		return 1;
}

static inline int readonly_check(uint64_t p, uint64_t v)
{
	return 0;
}

static inline int global_ovfctrl_check(uint64_t p, uint64_t v)
{
	uint64_t m = ~(BIT(0) | BIT(33));
	uint64_t v_wr = v & m;
	uint64_t v_rd = p & m;

	if ((v_rd ^ v_wr))
		return 1;

	if (!(v & BIT(0)) || !(v & BIT(33)))
		return 1;

	return 0;
}

static inline int fixed_ctrctrl_check(uint64_t p, uint64_t v)
{
	uint64_t m = ~(BIT(4) | BIT(5));
	uint64_t v_wr = v & m;
	uint64_t v_rd = p & m;

	if ((v_rd ^ v_wr))
		return 1;

	if (!(v & BIT(4)) || !(v & BIT(5)))
		return 1;

	return 0;
}


static inline int all_bits_off_check(uint64_t p, uint64_t v)
{
	if (v ^ (~ALL_64BITS))
		return 1;

	return 0;
}

static inline int perf_evtsel0_check(uint64_t p, uint64_t v)
{
	if (v ^ GP0_COUNTER_CONFIG)
		return 1;

	return 0;
}

static inline int mc_status_check(uint64_t p, uint64_t v)
{
	if (v == 0x0)
		return 0;
	else
		return 1;
}


static inline int microcode_wr_check(uint64_t p, uint64_t v)
{
	if (v ^ IA_MICROCODE_WR_V)
		return 1;

	return 0;
}

static struct msr_table mst[N_MSR_ADDRS] = {
	{ IA32_PERF_GLOBAL_CTRL,	RW, global_ctrl_check },
	{ IA32_PERF_GLOBAL_STATUS,	RO, readonly_check },
	{ IA32_CR_PERF_GLOBAL_OVF_CTRL, RW, global_ovfctrl_check },
	{ IA32_FIXED_CTR_CTRL,		RW, fixed_ctrctrl_check },
	{ IA32_FIXED_CTR1,		RW, all_bits_off_check },
	{ IA32_PERFMON_GP_CTR0,		RW, all_bits_off_check },
	{ IA32_PERFEVTSEL0,		RW, perf_evtsel0_check },
	{ IA32_MC1_STATUS,		RW, mc_status_check },
	{ IA32_MC2_STATUS,		RW, mc_status_check },
	{ IA32_MC3_STATUS,		RW, mc_status_check },
	{ IA_MICROCODE,			RW, microcode_wr_check },
	{ IA32_MC2_CTL2,		RO, readonly_check },
	{ IA32_MC3_CTL,			RO, readonly_check },
	{ IA32_MC1_CTL,			RO, readonly_check },
	{ IA32_MC1_CTL2,		RO, readonly_check },
	{ IA32_MC0_CTL,			RO, readonly_check },
	{ IA32_MCG_CAP,			RO, readonly_check },
	{ UNCORE_MC06_CTL,			RO, readonly_check },
	{ UNCORE_MC07_CTL,			RO, readonly_check },
	{ UNCORE_MC08_CTL,			RO, readonly_check },
	{ UNCORE_MC06_STATUS,			RW, mc_status_check },
	{ UNCORE_MC07_STATUS,			RW, mc_status_check },
	{ UNCORE_MC08_STATUS,			RW, mc_status_check },
};

static int check_msr(uint32_t addr, uint64_t v_read, uint64_t v_write)
{
	int i = 0;
	int idx = -1;

	while (i < N_MSR_ADDRS) {
		if ((mst[i].addr == addr) && (mst[i].rw != RO)) {
			idx = i;
			break;
		}
		i++;
	}
	if (idx != -1)
		return mst[idx].check(v_read, v_write);

	return -EINVAL;

}

static int valid_msr_address(uint32_t addr)
{
	int i = 0;

	while (i < N_MSR_ADDRS) {
		if (mst[i].addr == addr)
			return 0;
		i++;
	}
	return 1;
}

static int valid_mmio_address(uint32_t addr, uint32_t size, int *idx)
{
	int i = 0;

	while (i < N_MMIO_ADDRS) {
		if ((mmioaddrs[i].start_vm_pgoff == addr) &&
				(mmioaddrs[i].map_size == size)) {
			*idx = i;
			return 0;
		}
		i++;
	}
	return 1;
}

static uint32_t read_device_bar(uint32_t dev, uint32_t fun, uint32_t offset)
{
	uint64_t addr;
	void __iomem *regs = NULL;
	uint32_t bar_value = 0x0U;

	if (phy_pcie_base != 0ULL) {
		addr = phy_pcie_base + PCIE_ADDR_BUS0(dev, fun, offset);
		regs = ioremap(addr, 4);
		if (regs) {
			bar_value = readl(regs);
			iounmap(regs);
		}
	}
	return bar_value;
}

static int set_mmio_address_range(void)
{
	uint32_t bar_value = 0x0U;

	mmioaddrs[0].start_vm_pgoff = PCIE_BUS0_PGOFF;
	mmioaddrs[0].map_size = PCIE_BUS_SIZE;
	mmioaddrs[0].is_bar = 0;

	mmioaddrs[1].start_vm_pgoff = SB_PF;
	mmioaddrs[1].map_size = SBBAR_SIZE;
	mmioaddrs[1].is_bar = 0;

	bar_value = read_device_bar(TSN_GBE_DEV, TSN_GBE_FUNC, 0x10);
	if (!bar_value)
		goto err_out;
	mmioaddrs[2].start_vm_pgoff = (bar_value >> PAGE_SHIFT);
	mmioaddrs[2].map_size = TSN_GBE_BAR_SIZE;
	mmioaddrs[2].is_bar = 1;
	bar_value = 0x0U;

	bar_value = read_device_bar(OSE_TSN_GBE_0_DEV,
			OSE_TSN_GBE_0_FUNC, 0x10);
	if (!bar_value)
		goto err_out;
	mmioaddrs[3].start_vm_pgoff = (bar_value >> PAGE_SHIFT);
	mmioaddrs[3].map_size = OSE_TSN_GBE_0_BAR_SIZE;
	mmioaddrs[3].is_bar = 1;
	bar_value = 0x0U;

	bar_value = read_device_bar(OSE_TSN_GBE_1_DEV,
			OSE_TSN_GBE_1_FUNC, 0x10);
	if (!bar_value)
		goto err_out;
	mmioaddrs[4].start_vm_pgoff = (bar_value >> PAGE_SHIFT);
	mmioaddrs[4].map_size = OSE_TSN_GBE_1_BAR_SIZE;
	mmioaddrs[4].is_bar = 1;
	bar_value = 0x0U;

	bar_value = read_device_bar(SA_DEV, SA_FUNC, MCHBAR_OFFSET);
	if (!bar_value)
		goto err_out;
	bar_value &= B_MCHBAR_MASK;
	mmioaddrs[5].start_vm_pgoff = (bar_value >> PAGE_SHIFT);
	mmioaddrs[5].map_size = MCHBAR_SIZE;
	mmioaddrs[5].is_bar = 1;
	bar_value = 0x0U;

	mmioaddrs[6].start_vm_pgoff = BDF_TO_PGOFF(IEH_DEV, IEH_FUNC, 0x0);
	mmioaddrs[6].map_size = PCIE_FUNCTION_MMIO_SIZE;
	mmioaddrs[6].is_bar = 0;

	return 0;
err_out:
	return 1;
}


static int hstd_open(struct inode *inodep, struct file *filep)
{
	return 0;
}

static int hstd_release(struct inode *inodep, struct file *filep)
{
	return 0;
}

static ssize_t hstd_msr_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	u32 __user *tmp = (u32 __user *) buf;
	u32 data[2];
	u32 reg = *ppos;
	int err = 0;
	ssize_t bytes = 0;
	int cpu;

	cpu = get_cpu();
	put_cpu();

	if (count % 8)
		return -EINVAL;

	if (valid_msr_address(reg))
		return -EINVAL;

	for (; count; count -= 8) {
		err = rdmsr_safe_on_cpu(cpu, reg, &data[0], &data[1]);
		if (err)
			break;
		if (copy_to_user(tmp, &data, 8)) {
			err = -EFAULT;
			break;
		}
		tmp += 2;
		bytes += 8;
	}

	return bytes ? bytes : err;
}

static ssize_t hstd_msr_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	const u32 __user *tmp = (const u32 __user *)buf;
	u32 data_w[2];
	u32 data_r[2];
	u32 reg = *ppos;
	int err = 0;
	ssize_t bytes = 0;
	uint64_t v_write;
	uint64_t v_read;
	int cpu;

	cpu = get_cpu();
	put_cpu();

	if (count % 8)
		return -EINVAL;

	for (; count; count -= 8) {
		if (copy_from_user(&data_w, tmp, 8)) {
			err = -EFAULT;
			break;
		}
		err = rdmsr_safe_on_cpu(cpu, reg, &data_r[0], &data_r[1]);
		if (err)
			break;
		v_write = ((uint64_t)data_w[1] << 32) | data_w[0];
		v_read = ((uint64_t)data_r[1] << 32) | data_r[0];

		err = check_msr(reg, v_read, v_write);
		if (err)
			break;

		err = wrmsr_safe_on_cpu(cpu, reg, data_w[0], data_w[1]);
		if (err)
			break;
		tmp += 2;
		bytes += 8;
	}

	return bytes ? bytes : err;
}


static uint32_t hstd_pci_config_dword_read(uint8_t bus,
		uint8_t dev,
		uint8_t func,
		uint8_t offset,
		uint32_t *pPciData)
{
	if ((dev >= 32)	|| (func >= 8)
			|| ((offset & 0x3) != 0)
			|| (pPciData == NULL))
		return 1;

	outl(0x80000000 | ((bus & 0xff) << 16)
			| (PCI_DEVFN(dev, func) << 8)
			| (offset&~3), 0xcf8U);
	*pPciData = inl(0xcfcU);

	return 0;
}

static int do_init(void)
{
	uint32_t data32 = 0U;
	uint64_t phy_pcie_base_raw;

	if (hstd_pci_config_dword_read(0U, SA_DEV, SA_FUNC,
				PCIEXBAR_LO_0_0_0, &data32))
		goto err_out;

	phy_pcie_base_raw = (uint64_t)data32;
	if (hstd_pci_config_dword_read(0U, SA_DEV, SA_FUNC,
				PCIEXBAR_HI_0_0_0, &data32))
		goto err_out;

	phy_pcie_base_raw = phy_pcie_base_raw | ((uint64_t)data32 << 32);
	if (V_PCIEXBAR_EN != (phy_pcie_base_raw & B_PCIEXBAREN))
		goto err_out;

	phy_pcie_base = phy_pcie_base_raw & B_PCIEXBAR;
	if (set_mmio_address_range())
		goto err_out;

	return 0;
err_out:
	return 1;

}

#ifndef CONFIG_MMU
/* can't do an in-place private mapping if there's no MMU */
static inline int private_mapping_ok(struct vm_area_struct *vma)
{
	return vma->vm_flags & VM_MAYSHARE;
}
#else

static inline int private_mapping_ok(struct vm_area_struct *vma)
{
	return 1;
}
#endif

static int hstd_mmio(struct file *file, struct vm_area_struct *vma)
{
	uint32_t ret = 0;
	unsigned long pf = 0;
	int idx = -1;
	size_t size = vma->vm_end - vma->vm_start;

	phys_addr_t offset = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;

	if (offset >> PAGE_SHIFT != vma->vm_pgoff)
		return -EINVAL;

	if (offset + (phys_addr_t)size - 1 < offset)
		return -EINVAL;

	if (!private_mapping_ok(vma))
		return -EINVAL;

	if ((SBREG_BAR & B_RBA_MASK) != (vma->vm_pgoff << PAGE_SHIFT)) {
		spin_lock(&phy_pcie_base_lock);
		if (phy_pcie_base == 0)
			ret = do_init();
		spin_unlock(&phy_pcie_base_lock);
		if (ret)
			goto err_out;

		if (valid_mmio_address(vma->vm_pgoff, size, &idx))
			goto err_out;

		if (mmioaddrs[idx].is_bar)
			pf = vma->vm_pgoff;
		else
			pf = (phy_pcie_base >> PAGE_SHIFT) + vma->vm_pgoff;
	} else {
		pf = vma->vm_pgoff;
		ret = 0;
	}

	if (remap_pfn_range(vma, vma->vm_start,
				pf, size,
				vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
err_out:
	return -EINVAL;
}


static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = hstd_open,
	.read = hstd_msr_read,
	.write = hstd_msr_write,
	.release = hstd_release,
	.mmap	= hstd_mmio,
};

int hstd_init(void)
{
	int err = 0;

	hstd_major_num = register_chrdev(0, DEVICE_NAME, &fops);
	if (hstd_major_num < 0)
		return hstd_major_num;

	hstd_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(hstd_class)) {
		err = PTR_ERR(hstd_class);
		goto out_chrdev;
	}

	hstd_dev = device_create(hstd_class, NULL,
			MKDEV(hstd_major_num, 0), NULL, DEVICE_NAME);
	if (IS_ERR(hstd_dev)) {
		err = PTR_ERR(hstd_dev);
		goto out_class;
	}
	err = do_init();
	if (err)
		goto out_init;

	pr_info(PFX "Module version %s successfuly initilized\n", DRV_VERSION);
	return 0;

out_init:
	device_destroy(hstd_class, MKDEV(hstd_major_num, 0));
out_class:
	class_destroy(hstd_class);
out_chrdev:
	unregister_chrdev(hstd_major_num, DEVICE_NAME);
	return err;
}

static void __exit hstd_exit(void)
{
	device_destroy(hstd_class, MKDEV(hstd_major_num, 0));
	class_unregister(hstd_class);
	class_destroy(hstd_class);
	unregister_chrdev(hstd_major_num, DEVICE_NAME);
}

module_init(hstd_init);
module_exit(hstd_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Gupta, Gyan <gyan.gupta@intel.com>");
MODULE_DESCRIPTION("Driver for Host STL");
