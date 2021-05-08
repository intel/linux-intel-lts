// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro driver main entrance.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

/* Our header */
#include "hantro_priv.h"
#include "hantro_enc.h"
#include "hantro_dec.h"
#include "hantro_cache.h"
#include "hantro_dec400.h"
#include "hantro_cooling.h"

struct hantro_drm_handle hantro_drm;

long kmb_freq_table[3] = { 700000000, 500000000, 250000000 };
long tbh_freq_table[3] = { 800000000, 400000000, 200000000 };

bool verbose;
module_param(verbose, bool, 0);
MODULE_PARM_DESC(verbose, "Verbose log operations (default 0)");

bool enable_lut;
module_param(enable_lut, bool, 0);
MODULE_PARM_DESC(enable_lut, "Enable Page Table LUT(default 0)");

bool enable_encode = 1;
module_param(enable_encode, bool, 0);
MODULE_PARM_DESC(enable_encode, "Enable Encode(default 1)");

bool enable_decode = 1;
module_param(enable_decode, bool, 0);
MODULE_PARM_DESC(enable_decode, "Enable Decode(default 1)");

bool enable_dec400;
module_param(enable_dec400, bool, 0);
MODULE_PARM_DESC(enable_dec400, "Enable DEC400/L2(default 0)");

bool enable_irqmode = 1;
module_param(enable_irqmode, bool, 0);
MODULE_PARM_DESC(enable_irqmode, "Enable IRQ Mode(default 1)");

static int link_device_drm(struct device_info *pdevinfo)
{
	struct device_info *tmpdev;

	if (!pdevinfo)
		return -1;

	if (mutex_lock_interruptible(&hantro_drm.hantro_mutex))
		return -EBUSY;

	if (!hantro_drm.pdevice_list) {
		hantro_drm.pdevice_list = pdevinfo;
	} else {
		tmpdev = hantro_drm.pdevice_list;
		while (tmpdev->next)
			tmpdev = tmpdev->next;

		tmpdev->next = pdevinfo;
	}

	mutex_unlock(&hantro_drm.hantro_mutex);
	return 0;
}

static int unlink_device_drm(struct device_info *pdevinfo)
{
	struct device_info *tmpdev;

	if (!pdevinfo)
		return -1;

	if (!hantro_drm.pdevice_list)
		return -1;

	if (mutex_lock_interruptible(&hantro_drm.hantro_mutex))
		return -EBUSY;

	if (hantro_drm.pdevice_list == pdevinfo) {
		hantro_drm.pdevice_list = pdevinfo->next;
	} else {
		tmpdev = hantro_drm.pdevice_list;

		while (tmpdev->next && tmpdev->next != pdevinfo)
			tmpdev = tmpdev->next;
	}

	mutex_unlock(&hantro_drm.hantro_mutex);
	return 0;
}

struct device_info *get_device_info(int deviceid)
{
	struct device_info *tmpdev;

	if (mutex_lock_interruptible(&hantro_drm.hantro_mutex))
		return NULL;

	tmpdev = hantro_drm.pdevice_list;
	while (tmpdev && tmpdev->deviceid != deviceid)
		tmpdev = tmpdev->next;

	mutex_unlock(&hantro_drm.hantro_mutex);
	return tmpdev;
}

static int get_clock_name(struct dtbnode *pnode)
{
	const char *nodename;

	if (!pnode || !pnode->ofnode)
		return -EINVAL;

	nodename = pnode->ofnode->name;

	if (hantro_drm.device_type == DEVICE_KEEMBAY) {
		if (strstr(nodename, "decoderA") == nodename)
			sprintf(pnode->clock_name, "clk_xin_vdec");

		if (strstr(nodename, "decoderB") == nodename)
			sprintf(pnode->clock_name, "clk_xin_vdec");

		if (strstr(nodename, "encoderA") == nodename)
			sprintf(pnode->clock_name, "clk_xin_venc");

		if (strstr(nodename, "encoderB") == nodename)
			sprintf(pnode->clock_name, "clk_xin_jpeg");
	} else if (hantro_drm.device_type == DEVICE_THUNDERBAY) {
		if (strstr(nodename, "decoderA") == nodename)
			sprintf(pnode->clock_name, "vc8000da_aclk_computess%d",
				pnode->pdevinfo->deviceid);

		if (strstr(nodename, "decoderB") == nodename)
			sprintf(pnode->clock_name, "vc8000db_aclk_computess%d",
				pnode->pdevinfo->deviceid);

		if (strstr(nodename, "encoderA") == nodename)
			sprintf(pnode->clock_name, "vc8000ej_aclk_computess%d",
				pnode->pdevinfo->deviceid);

		if (strstr(nodename, "encoderB") == nodename)
			sprintf(pnode->clock_name, "vc8000e_aclk_computess%d",
				pnode->pdevinfo->deviceid);
	}

	return -EINVAL;
}

static int get_node_type(const char *name)
{
	if (strstr(name, NODENAME_DECODER) == name)
		return CORE_DEC;

	if (strstr(name, NODENAME_ENCODER) == name)
		return CORE_ENC;

	if (strstr(name, NODENAME_CACHE) == name)
		return CORE_CACHE;

	if (strstr(name, NODENAME_DEC400) == name)
		return CORE_DEC400;

	return CORE_UNKNOWN;
}

static struct dtbnode *try_create_node(struct platform_device *pdev,
				       struct device_node *ofnode,
				       struct device_info *pdevinfo, int parenttype,
				       phys_addr_t parentaddr)
{
	struct fwnode_handle *fwnode;
	struct resource r;
	int i, na, ns, ret = 0;
	int endian = of_device_is_big_endian(ofnode);
	u32 reg_u32[4];
	const char *reg_name;
	u64 ioaddress, iosize;
	struct dtbnode *pnode = kzalloc(sizeof(*pnode), GFP_KERNEL);

	if (!pnode)
		return NULL;

	pnode->type = get_node_type(ofnode->name);
	pnode->parentaddr = parentaddr;
	pnode->parenttype = parenttype;
	pnode->pdevinfo = pdevinfo;
	pnode->ofnode = ofnode;
	fwnode = &ofnode->fwnode;
	get_clock_name(pnode);

	na = of_n_addr_cells(ofnode);
	ns = of_n_size_cells(ofnode);
	if (na > 2 || ns > 2) {
		pr_err("cell size too big");
		kfree(pnode);
		return NULL;
	}

	fwnode_property_read_u32_array(fwnode, "reg", reg_u32, na + ns);
	if (na == 2) {
		if (!endian) {
			ioaddress = reg_u32[0];
			ioaddress <<= 32;
			ioaddress |= reg_u32[1];
		} else {
			ioaddress = reg_u32[1];
			ioaddress <<= 32;
			ioaddress |= reg_u32[0];
		}
	} else {
		ioaddress = reg_u32[0];
	}

	if (ns == 2) {
		if (!endian) {
			iosize = reg_u32[na];
			iosize <<= 32;
			iosize |= reg_u32[na + 1];
		} else {
			iosize = reg_u32[na + 1];
			iosize <<= 32;
			iosize |= reg_u32[na];
		}
	} else {
		iosize = reg_u32[na];
	}

	pnode->ioaddr = ioaddress;
	pnode->iosize = iosize;
	fwnode_property_read_string(fwnode, "reg-names", &reg_name);

	if (strlen(reg_name))
		strcpy(pnode->reg_name, reg_name);
	else
		strcpy(pnode->reg_name, "hantro_reg");

	for (i = 0; i < 4; i++) {
		if (of_irq_to_resource(ofnode, i, &r) > 0) {
			pnode->irq[i] = r.start;
			if (strlen(r.name))
				strcpy(pnode->irq_name[i], r.name);
			else
				strcpy(pnode->irq_name[i], "hantro_irq");

			PDEBUG("irq %s: mapping = %lld\n", r.name, r.end);
		} else {
			pnode->irq[i] = -1;
		}
	}

	switch (pnode->type) {
	case CORE_DEC:
		ret = hantrodec_probe(pnode);
		break;
	case CORE_ENC:
		ret = hantroenc_probe(pnode);
		break;
	case CORE_CACHE:
		ret = hantrocache_probe(pnode);
		break;
	case CORE_DEC400:
		ret = hantrodec400_probe(pnode);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		kfree(pnode);
		pnode = NULL;
	}

	return pnode;
}

/* hantro_mmu_control is used to check whether media MMU is enabled or disabled */
static void hantro_mmu_control(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u64 mmu_tcu_smmu_cr0;
	u8 __iomem *mmu_tcu_smmu_cr0_register;
	int is_mmu_enabled, count = 0;

	count = device_property_read_u64(dev, "mmu-tcu-reg",
					 &mmu_tcu_smmu_cr0);

	if (count == 0) {
		if (!request_mem_region(mmu_tcu_smmu_cr0, 0x32,
					"mmu_tcu_smmu_cr0")) {
			pr_info("mmu_tcu_smmu_cr0: failed to request mem region\n");
		}

		mmu_tcu_smmu_cr0_register =
			ioremap(mmu_tcu_smmu_cr0, 0x32);
		if (!mmu_tcu_smmu_cr0_register) {
			pr_info("mmu_tcu_smmu_cr0_register: failed to ioremap mmu_tcu_smmu_cr0_register\n");
		} else {
			is_mmu_enabled = ioread32(mmu_tcu_smmu_cr0_register);
			if (is_mmu_enabled)
				pr_info("hantro_init: Media MMU600 is enabled, is_mmu_enabled = %d\n",
					is_mmu_enabled);
			else
				pr_info("hantro_init: Media MMU600 is disabled, is_mmu_enabled = %d\n",
					is_mmu_enabled);

			release_mem_region(mmu_tcu_smmu_cr0, 0x32);
		}
	}
}

/* hantro_enable_clock to enable/disable the media SS clocks */
static int hantro_clock_control(struct device *dev, bool enable)
{
	struct clk *dev_clk = NULL;
	const char **clock_names;
	int i = 0, ret = 0, count = 0;
	unsigned long rate;

	if ((hantro_drm.device_type != DEVICE_KEEMBAY) &&
			(hantro_drm.device_type != DEVICE_THUNDERBAY))
		return -EINVAL;

	/* Read clock names */
	count = device_property_read_string_array(dev, "clock-names", NULL, 0);
	if (count > 0) {
		clock_names = kcalloc(count, sizeof(*clock_names), GFP_KERNEL);
		if (!clock_names)
			return 0;

		ret = device_property_read_string_array(dev, "clock-names",
							clock_names, count);
		if (ret < 0) {
			pr_err("failed to read clock names\n");
			kfree(clock_names);
			return 0;
		}

		for (i = 0; i < count; i++) {
			PDEBUG("hantro: clock_name = %s\n", clock_names[i]);
			dev_clk = clk_get(dev, clock_names[i]);
			if (enable) {
				clk_prepare_enable(dev_clk);

				if (verbose)
					pr_info("hantro: default clock frequency of clock_name = %s is %ld\n"
						, clock_names[i],
						clk_get_rate(dev_clk));

				if (hantro_drm.device_type == DEVICE_KEEMBAY)
					rate = kmb_freq_table[0];
				else if (hantro_drm.device_type == DEVICE_THUNDERBAY)
					rate = tbh_freq_table[0];

				clk_set_rate(dev_clk, rate);
				if (verbose)
					pr_info("hantro: set %lu Mhz clock frequency of clock_name = %s is %ld\n"
					, rate, clock_names[i], clk_get_rate(dev_clk));

			} else {
				clk_disable_unprepare(dev_clk);
			}

			clk_put(dev_clk);
		}

		kfree(clock_names);
	}

	return 0;
}

/* hantro_reset_clock to de-assert/assert the media SS cores and MMU */
static int hantro_reset_control(struct platform_device *pdev, bool deassert)
{
	struct device *dev = &pdev->dev;
	const char **reset_names;
	struct reset_control *dev_reset = NULL;
	int i = 0, ret = 0, count = 0;

	/* Read reset names */
	count = device_property_read_string_array(dev, "reset-names", NULL, 0);
	if (count > 0) {
		reset_names = kcalloc(count, sizeof(*reset_names), GFP_KERNEL);
		if (!reset_names)
			return 0;

		ret = device_property_read_string_array(dev, "reset-names",
							reset_names, count);
		if (ret < 0) {
			pr_err("failed to read clock names\n");
			kfree(reset_names);
			return 0;
		}

		for (i = 0; i < count; i++) {
			if (verbose)
				pr_info("hantro: reset_name = %s\n",
					reset_names[i]);

			dev_reset = devm_reset_control_get(dev, reset_names[i]);
			if (deassert) {
				ret = reset_control_deassert(dev_reset);
				if (ret < 0) {
					pr_err("failed to deassert reset : %s, %d\n",
					       reset_names[i], ret);
				}

			} else {
				ret = reset_control_assert(dev_reset);
				if (ret < 0) {
					pr_err("failed to assert reset : %s, %d\n",
					       reset_names[i], ret);
				}
			}
		}

		kfree(reset_names);
	}

	return 0;
}

static int hantro_analyze_subnode(struct platform_device *pdev,
				  struct device_node *pofnode,
				  struct device_info *pdevinfo)
{
	struct dtbnode *head, *nhead, *newtail, *node;

	head = kzalloc(sizeof(*head), GFP_KERNEL);
	if (!head)
		return -ENOMEM;

	head->type = CORE_DEVICE;
	head->parenttype = CORE_DEVICE;
	head->ofnode = pofnode;
	head->pdevinfo = pdevinfo;
	head->ioaddr = -1;
	head->iosize = 0;
	head->next = NULL;
	/* this is a wide first tree structure iteration, result is stored in device info */
	while (head) {
		nhead = NULL;
		newtail = NULL;
		while (head) {
			struct device_node *child, *ofnode = head->ofnode;

			for_each_child_of_node(ofnode, child) {
				node = try_create_node(pdev, child, pdevinfo,
						       head->type, head->ioaddr);
				if (node) {
					if (!nhead) {
						nhead = node;
						newtail = node;
					} else {
						newtail->next = node;
					}

					node->next = NULL;
					newtail = node;
				}
			}

			node = head->next;
			kfree(head);
			head = node;
		}

		head = nhead;
	}

	return 0;
}

static int init_codec_rsvd_mem(struct device *dev, struct device_info *pdevice,
			       const char *mem_name, unsigned int mem_idx)
{
	struct device *mem_dev;
	int rc = -1;

	/* Create a child device (of dev) to own the reserved memory. */
	mem_dev =
		devm_kzalloc(dev, sizeof(struct device), GFP_KERNEL | GFP_DMA);
	if (!mem_dev)
		return -ENOMEM;

	device_initialize(mem_dev);
	dev_set_name(mem_dev, "%s:%s", dev_name(dev), mem_name);
	mem_dev->parent = dev;
	mem_dev->dma_mask = dev->dma_mask;
	mem_dev->coherent_dma_mask = dev->coherent_dma_mask;
	/* Set up DMA configuration using information from parent's DT node. */
	mem_dev->release = of_reserved_mem_device_release;
	rc = device_add(mem_dev);
	if (rc)
		goto err;

	/* Initialized the device reserved memory region. */
	rc = of_reserved_mem_device_init_by_idx(mem_dev, dev->of_node, mem_idx);
	if (rc) {
		device_del(mem_dev);
		goto err;
	} else {
		dev_info(dev,
			 "Success: Codec reserved memory found at idx = %d, ret=%d\n",
			 mem_idx, rc);
	}

	pdevice->codec_rsvmem = mem_dev;
	return 0;
err:
	put_device(mem_dev);
	return rc;
}

static void set_device_type(struct device_info *pdevinfo)
{
	struct device_node *ofnode = NULL;
	struct fwnode_handle *fwnode;
	const char *compat_name;

	if (!pdevinfo || !pdevinfo->dev)
		return;

	ofnode = pdevinfo->dev->of_node;
	if (!ofnode)
		return;

	fwnode = &ofnode->fwnode;
	fwnode_property_read_string(fwnode, "compatible", &compat_name);
	if (strstr(compat_name, "kmb"))
		hantro_drm.device_type = DEVICE_KEEMBAY;
	else if (strstr(compat_name, "thunderbay"))
		hantro_drm.device_type = DEVICE_THUNDERBAY;
}

static int init_device_info(struct device *dev, struct device_info *pdevinfo)
{
	if (!pdevinfo)
		return -EINVAL;

	pdevinfo->dev = dev;
	dev_set_drvdata(dev, pdevinfo);
	pdevinfo->drm_dev = hantro_drm.drm_dev;
	pdevinfo->config = 0;
	pdevinfo->next = NULL;
	pdevinfo->deccore_num = 0;
	pdevinfo->enccore_num = 0;
	pdevinfo->cachecore_num = 0;
	pdevinfo->dec400core_num = 0;
	pdevinfo->dechdr = NULL;
	pdevinfo->enchdr = NULL;
	pdevinfo->cachehdr = NULL;
	pdevinfo->dec400hdr = NULL;

	init_waitqueue_head(&pdevinfo->cache_hw_queue);
	init_waitqueue_head(&pdevinfo->cache_wait_queue);
	spin_lock_init(&pdevinfo->cache_owner_lock);

	sema_init(&pdevinfo->enc_core_sem, 1);
	init_waitqueue_head(&pdevinfo->enc_hw_queue);
	spin_lock_init(&pdevinfo->enc_owner_lock);
	init_waitqueue_head(&pdevinfo->enc_wait_queue);

	pdevinfo->dec_irq = 0;
	pdevinfo->pp_irq = 0;
	spin_lock_init(&pdevinfo->owner_lock);
	init_waitqueue_head(&pdevinfo->dec_wait_queue);
	init_waitqueue_head(&pdevinfo->pp_wait_queue);
	init_waitqueue_head(&pdevinfo->hw_queue);
	sema_init(&pdevinfo->pp_core_sem, 1);
	pdevinfo->deviceid = atomic_read(&hantro_drm.devicecount);
	atomic_inc(&hantro_drm.devicecount);
	set_device_type(pdevinfo);
	return 0;
}

static int get_reserved_mem_size(struct device_info *pdevice)
{
	struct device *dev = pdevice->dev;
	struct device_node *memnp;
	int rc;

	/* Get pointer to memory region device node from "memory-region" phandle. */
	memnp = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!memnp) {
		dev_err(dev, "no memory-region node at index 0\n");
		return 0;
	}

	rc = of_address_to_resource(memnp, 0, &pdevice->mem_res[0]);
	of_node_put(memnp);
	if (rc) {
		dev_err(dev,
			"failed to translate memory-region to a resource\n");
		return 0;
	}

	/* Get pointer to memory region device node from "memory-region" phandle. */
	memnp = of_parse_phandle(dev->of_node, "memory-region", 1);
	if (!memnp) {
		dev_err(dev, "no memory-region node at index 0\n");
		return 0;
	}

	rc = of_address_to_resource(memnp, 0, &pdevice->mem_res[1]);
	of_node_put(memnp);
	if (rc) {
		dev_err(dev,
			"failed to translate memory-region to a resource\n");
		return 0;
	}

	return 0;
}

static int hantro_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int result = 0;
	struct device_info *pdevinfo = NULL;

	pr_info("%s: dev %s probe\n", __func__, pdev->name);
	if (!dev->of_node)
		return 0;

	pdevinfo = devm_kzalloc(dev, sizeof(struct device_info), GFP_KERNEL);
	if (!pdevinfo)
		return -ENOMEM;

	init_device_info(dev, pdevinfo);
	setup_thermal_cooling(pdevinfo);
	hantro_clock_control(dev, true);
	if (hantro_drm.device_type != DEVICE_KEEMBAY) {
		hantro_reset_control(pdev, true);
		hantro_mmu_control(pdev);
	}

	result = of_reserved_mem_device_init(dev);
	if (result)
		return result;

	dma_set_mask(dev, DMA_BIT_MASK(48));
	dma_set_coherent_mask(dev, DMA_BIT_MASK(48));
	hantro_analyze_subnode(pdev, dev->of_node, pdevinfo);
	result = init_codec_rsvd_mem(pdevinfo->dev, pdevinfo, "codec_reserved", 1);
	if (result) {
		dev_err(pdevinfo->dev, "Failed to set up codec reserved memory.\n");
		return result;
	}

	get_reserved_mem_size(pdevinfo);
	create_debugfs(pdevinfo, result == 0);
	link_device_drm(pdevinfo);
	result = create_sysfs(pdevinfo);
	if (result)
		pr_info("Failed to create sysfs.\n");

	result = class_compat_create_link(hantro_drm.media_class, &pdev->dev,
					  pdev->dev.parent);
	if (result)
		pr_info("Failed to create compatibility class link.\n");

	idr_init(&pdevinfo->clients);
	idr_init(&pdevinfo->allocations);
	mutex_init(&pdevinfo->alloc_mutex);
	mutex_init(&hantro_drm.hantro_mutex);

	return 0;
}

static int hantro_drm_remove(struct platform_device *pdev)
{
	struct device_info *pdevinfo;

	if (!pdev->dev.of_node)
		return 0;

	pdevinfo = dev_get_drvdata(&pdev->dev);
	if (!pdevinfo)
		return 0;

	if (pdevinfo->codec_rsvmem) {
		of_reserved_mem_device_release(pdevinfo->codec_rsvmem);
		device_del(pdevinfo->codec_rsvmem);
		put_device(pdevinfo->codec_rsvmem);
		pdevinfo->codec_rsvmem = NULL;
	}

	idr_destroy(&pdevinfo->clients);
	idr_destroy(&pdevinfo->allocations);
	mutex_destroy(&pdevinfo->alloc_mutex);
	mutex_destroy(&hantro_drm.hantro_mutex);

	class_compat_remove_link(hantro_drm.media_class, &pdev->dev,
				 pdev->dev.parent);
	remove_sysfs(pdevinfo);
	hantrodec_remove(pdevinfo);
	hantroenc_remove(pdevinfo);
	hantrodec400_remove(pdevinfo);
	hantrocache_remove(pdevinfo);
	unlink_device_drm(pdevinfo);
	return 0;
}

static const struct platform_device_id hantro_drm_platform_ids[] = {
	{
		.name = DRIVER_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(platform, hantro_drm_platform_ids);

static const struct of_device_id hantro_of_match[] = {
	/*to match dtb, else reg io will fail*/
	{ .compatible = "kmb,hantro" },
	{ .compatible = "thunderbay,hantro" },
	{ /* sentinel */ }
};

static int hantro_pm_suspend(struct device *kdev)
{
	return 0;
}

static int hantro_pm_resume(struct device *kdev)
{
	return 0;
}

static const struct dev_pm_ops hantro_pm_ops = {
	.suspend = hantro_pm_suspend,
	.resume = hantro_pm_resume,
};

static struct platform_driver hantro_drm_platform_driver = {
	.probe = hantro_drm_probe,
	.remove = hantro_drm_remove,
	.driver = {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = hantro_of_match,
			.pm = &hantro_pm_ops,
		},
	.id_table = hantro_drm_platform_ids,
};

static const struct platform_device_info hantro_platform_info = {
	.name = DRIVER_NAME,
	.id = -1,
	.dma_mask = DMA_BIT_MASK(48),
};

static void __exit hantro_cleanup(void)
{
	debugfs_remove(hantro_drm.debugfs_root);
	release_fence_data();

	hantrodec_cleanup();
	hantroenc_cleanup();

	drm_dev_unregister(hantro_drm.drm_dev);
	platform_device_unregister(hantro_drm.platformdev);
	platform_driver_unregister(&hantro_drm_platform_driver);
	drm_dev_put(hantro_drm.drm_dev);
	class_compat_unregister(hantro_drm.media_class);
	hantro_drm.media_class = NULL;
	mutex_destroy(&hantro_drm.hantro_mutex);
	pr_info("hantro driver removed\n");
}

static int __init hantro_init(void)
{
	int result, i;

	hantro_drm.debugfs_root = debugfs_create_dir("hantro", NULL);
	if (!hantro_drm.media_class)
		hantro_drm.media_class = class_compat_register("media");

	if (!hantro_drm.media_class)
		return -ENOMEM;

	if (IS_ERR_OR_NULL(hantro_drm.media_class)) {
		result = PTR_ERR(hantro_drm.media_class);
		hantro_drm.media_class = NULL;
		pr_err("[%s]: couldn't create driver class, return=%d\n",
		       DRIVER_NAME, result);
		result = (result == 0) ? -ENOMEM : result;
		return result;
	}

	mutex_init(&hantro_drm.hantro_mutex);
	hantro_drm.device_type = DEVICE_UNKNOWN;
	result = platform_driver_register(&hantro_drm_platform_driver);
	if (result < 0) {
		pr_info("hantro create platform driver fail");
		return result;
	}

	hantro_drm.platformdev =
		platform_device_register_full(&hantro_platform_info);
	if (!hantro_drm.platformdev) {
		platform_driver_unregister(&hantro_drm_platform_driver);
		pr_info("hantro create platform device fail");
		return PTR_ERR(hantro_drm.platformdev);
	}

	hantro_drm.drm_dev = create_hantro_drm(&hantro_drm.platformdev->dev);
	if (!hantro_drm.drm_dev || IS_ERR(hantro_drm.drm_dev)) {
		platform_device_unregister(hantro_drm.platformdev);
		platform_driver_unregister(&hantro_drm_platform_driver);
		return PTR_ERR(hantro_drm.drm_dev);
	}

	init_fence_data();
	for (i = 0; i < get_device_count(); i++) {
		struct device_info *pdevinfo = get_device_info(i);

		hantro_drm.config |= pdevinfo->config;
	}

	hantrodec_init();
	hantroenc_init();

	if (verbose)
		device_print_debug();

	pr_info("hantro device created\n");
	return result;
}

module_init(hantro_init);
module_exit(hantro_cleanup);

/* module description */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("VeriSilicon");
MODULE_AUTHOR("Mehmood, Arshad <arshad.mehmood@intel.com>");
MODULE_AUTHOR("Murugasen Krishnan, Kuhanh <kuhanh.murugasen.krishnan@intel.com>");
MODULE_AUTHOR("Hoe, Sheng Yang <sheng.yang.hoe@intel.com>");
MODULE_DESCRIPTION("Hantro DRM manager");
