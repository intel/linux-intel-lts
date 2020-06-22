// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#define FORMAT(fmt) "%s: %d: " fmt, __func__, __LINE__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#include <linux/pci.h>

#include "gna.h"

#include "gna_drv.h"

#define INTEL_GNA_DEVICE(platform, info) \
	{ PCI_VDEVICE(INTEL, platform), (kernel_ulong_t)(info) }

#define PLATFORM(id) .hwid = (id)

#define GNA_GEN1_FEATURES \
	.max_hw_mem = 256 * 1024 * 1024, \
	.num_pagetables = 64, \
	.num_page_entries = PAGE_SIZE/sizeof(u32), \
	/* desc_info all in bytes */ \
	.desc_info = { \
		.rsvd_size = 256, \
		.cfg_size = 256, \
		.desc_size = 784, \
		.mmu_info = { \
			.vamax_size = 4, \
			.rsvd_size = 12, \
			.pd_size = 4 * 64, \
		}, \
	}, \
	.max_layer_count = 1023

#define GNA_GEN2_FEATURES \
	GNA_GEN1_FEATURES, \
	.max_layer_count = 4096

static const struct gna_drv_info cnl_drv_info = {
	PLATFORM(GNA_DEV_HWID_CNL),
	GNA_GEN1_FEATURES
};

static const struct gna_drv_info glk_drv_info = {
	PLATFORM(GNA_DEV_HWID_GLK),
	GNA_GEN1_FEATURES
};

static const struct gna_drv_info ehl_drv_info = {
	PLATFORM(GNA_DEV_HWID_EHL),
	GNA_GEN1_FEATURES
};

static const struct gna_drv_info icl_drv_info = {
	PLATFORM(GNA_DEV_HWID_ICL),
	GNA_GEN1_FEATURES
};

static const struct gna_drv_info jsl_drv_info = {
	PLATFORM(GNA_DEV_HWID_JSL),
	GNA_GEN2_FEATURES
};

static const struct gna_drv_info tgl_drv_info = {
	PLATFORM(GNA_DEV_HWID_TGL),
	GNA_GEN2_FEATURES
};

/* PCI Routines */
static const struct pci_device_id gna_pci_ids[] = {
	INTEL_GNA_DEVICE(GNA_DEV_HWID_CNL, &cnl_drv_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_GLK, &glk_drv_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_EHL, &ehl_drv_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_ICL, &icl_drv_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_JSL, &jsl_drv_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_TGL, &tgl_drv_info),
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, gna_pci_ids);

static struct pci_driver gna_driver = {
	.name = GNA_DRV_NAME,
	.id_table = gna_pci_ids,
	.probe = gna_probe,
	.remove = gna_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &gna_pm,
	},
#endif
};

static char *gna_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = 0666;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}

static int __init gna_init(void)
{
	int ret;

	pr_debug("%s: enter\n", __func__);

	mutex_init(&gna_drv_priv.lock);

	gna_class = class_create(THIS_MODULE, "gna");
	if (IS_ERR(gna_class)) {
		pr_err("class device create failed\n");
		return PTR_ERR(gna_class);
	}
	gna_class->devnode = gna_devnode;

	ret = alloc_chrdev_region(&gna_drv_priv.devt, 0, MAX_GNA_DEVICES, "gna");
	if (ret) {
		pr_err("could not get major number\n");
		goto err_destroy_class;
	}

	pr_debug("major %d\n", MAJOR(gna_drv_priv.devt));
	pr_debug("minor %d\n", MINOR(gna_drv_priv.devt));

	gna_drv_priv.minor = MINOR(gna_drv_priv.devt);

	ret = pci_register_driver(&gna_driver);
	if (ret) {
		pr_err("pci register driver failed\n");
		goto err_unreg_chdev;
	}

	pr_debug("%s: exit\n", __func__);
	return 0;

err_unreg_chdev:
	unregister_chrdev_region(gna_drv_priv.devt, MAX_GNA_DEVICES);

err_destroy_class:
	class_destroy(gna_class);

	return ret;
}

static void __exit gna_exit(void)
{
	pr_debug("%s: enter\n", __func__);

	pci_unregister_driver(&gna_driver);
	unregister_chrdev_region(gna_drv_priv.devt, MAX_GNA_DEVICES);
	class_destroy(gna_class);

	pr_debug("%s: end\n", __func__);
}

module_init(gna_init);
module_exit(gna_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel GMM & Neural Network Accelerator Driver");
MODULE_VERSION(GNA_DRV_VER);

MODULE_ALIAS("pci:v00008086d00003190sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00008086d00004511sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00008086d00005A11sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00008086d00008A11sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00008086d00009A11sv*sd*bc*sc*i*");

MODULE_LICENSE("GPL v2");
