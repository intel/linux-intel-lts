// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro dec400 controller hardware driver.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"
#include "hantro_dec400.h"

static void dec400_reset_asic(struct dec400_t *dev)
{
	int i;

	for (i = 0; i < dev->core_cfg.iosize; i += sizeof(u32))
		iowrite32(0, dev->hwregs + i);
}

static int hantro_dec400_write_regs(struct dec400_t *dev,
				    struct core_desc *core)
{
	u32 data;
	int ret = 0;

	ret = copy_from_user(&data, core->regs + core->reg_id, sizeof(u32));
	if (ret) {
		pr_err("copy_from_user failed, returned %d\n", ret);
		return -EFAULT;
	}

	iowrite32(data, (dev->hwregs + core->reg_id * sizeof(u32)));
	return 0;
}

static int hantro_dec400_read_regs(struct dec400_t *dev, struct core_desc *core)
{
	u32 data;
	int ret = 0;

	data = ioread32((dev->hwregs + core->reg_id * sizeof(u32)));
	ret = copy_to_user(core->regs + core->reg_id, &data, sizeof(u32));
	if (ret) {
		pr_err("copy_to_user failed, returned %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

long hantrodec400_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	u32 id, deviceid, node, type;
	struct dec400_t *pdec400;
	struct core_desc coredesc;

	if (enable_dec400 == 0)
		return -EFAULT;

	switch (cmd) {
	case DEC400_IOCGHWOFFSET:
		ret = copy_from_user(&id, (void const __user *)arg, sizeof(u32));
		if (ret) {
			pr_err("copy_from_user failed, returned %d\n", ret);
			return -EFAULT;
		}

		deviceid = DEVICE_ID(id);
		type = NODETYPE(id);
		node = KCORE(id);
		pdec400 = get_dec400_node_by_type(deviceid, type, node);
		if (!pdec400)
			return -EINVAL;

		ret = __put_user(pdec400->core_cfg.dec400corebase,
				 (unsigned long long __user *)arg);
		if (ret) {
			pr_err("copy_to_user failed, returned %d\n", ret);
			return -EFAULT;
		}

		return 0;
	case DEC400_IOCGHWIOSIZE:
		ret = copy_from_user(&id, (void const __user *)arg, sizeof(u32));
		if (ret) {
			pr_err("copy_from_user failed, returned %d\n", ret);
			return -EFAULT;
		}

		deviceid = DEVICE_ID(id);
		type = NODETYPE(id);
		node = KCORE(id);
		pdec400 = get_dec400_node_by_type(deviceid, type, node);
		if (!pdec400)
			return -EFAULT;

		ret = __put_user(pdec400->core_cfg.iosize, (unsigned int __user *)arg);
		if (ret) {
			pr_err("copy_to_user failed, returned %d\n", ret);
			return -EFAULT;
		}

		return 0;
	case DEC400_IOCS_DEC_WRITE_REG:
		ret = copy_from_user(&coredesc, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (ret) {
			pr_err("copy_from_user failed, returned %d\n", ret);
			return -EFAULT;
		}

		deviceid = DEVICE_ID(coredesc.id);
		type = NODETYPE(coredesc.id);
		node = KCORE(coredesc.id);
		pdec400 = get_dec400_node_by_type(deviceid, type, node);
		if (!pdec400)
			return -EFAULT;

		return hantro_dec400_write_regs(pdec400, &coredesc);
	case DEC400_IOCS_DEC_READ_REG:
		ret = copy_from_user(&coredesc, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (ret) {
			pr_err("copy_from_user failed, returned %d\n", ret);
			return -EFAULT;
		}

		deviceid = DEVICE_ID(coredesc.id);
		type = NODETYPE(coredesc.id);
		node = KCORE(coredesc.id);
		pdec400 = get_dec400_node_by_type(deviceid, type, node);
		if (!pdec400)
			return -EFAULT;

		return hantro_dec400_read_regs(pdec400, &coredesc);
	default:
		return -EINVAL;
	}
	return 0;
}

int hantrodec400_probe(struct dtbnode *pnode)
{
	struct dec400_t *pdec400;
	int ret;

	if (enable_dec400 == 0)
		return 0;

	pdec400 = vmalloc(sizeof(*pdec400));
	if (!pdec400)
		return -EINVAL;

	pdec400->core_cfg.dec400corebase = pnode->ioaddr;
	pdec400->core_cfg.iosize = pnode->iosize;
	pdec400->core_cfg.parentaddr = pnode->parentaddr;
	if (!request_mem_region(pdec400->core_cfg.dec400corebase,
				pdec400->core_cfg.iosize, "hantrodec400")) {
		pr_err("DEC400: HW regs busy\n");
		ret = -ENODEV;
		goto free_alloc;
	}

	pdec400->hwregs = ioremap(pdec400->core_cfg.dec400corebase,
				  pdec400->core_cfg.iosize);
	if (!pdec400->hwregs) {
		release_mem_region(pdec400->core_cfg.dec400corebase,
				   pdec400->core_cfg.iosize);
		pr_err("DEC400: failed to map HW regs\n");
		ret = -ENODEV;
		goto free_alloc;
	}

	dec400_reset_asic(pdec400);
	add_dec400_node(pnode->pdevinfo, pdec400);
	pr_info("hantrodec400: HW at base <0x%llx>\n",
		pdec400->core_cfg.dec400corebase);
	return 0;

free_alloc:
	vfree(pdec400);
	return ret;
}

void hantrodec400_remove(struct device_info *pdevinfo)
{
	struct dec400_t *dev, *pp;

	dev = get_dec400_nodes(pdevinfo->deviceid, 0);
	while (dev) {
		if (dev->hwregs) {
			iounmap(dev->hwregs);
			release_mem_region(dev->core_cfg.dec400corebase,
					   dev->core_cfg.iosize);
		}

		pp = dev->next;
		vfree(dev);
		dev = pp;
	}
}
