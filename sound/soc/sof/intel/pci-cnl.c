// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

#include <linux/module.h>
#include <linux/pci.h>
#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include <sound/sof.h>
#include "../ops.h"
#include "../sof-pci-dev.h"

/* platform specific devices */
#include "hda.h"

static const struct sof_dev_desc cnl_desc = {
	.machines		= snd_soc_acpi_intel_cnl_machines,
	.alt_machines		= snd_soc_acpi_intel_cnl_sdw_machines,
	.use_acpi_target_states	= true,
	.resindex_lpe_base	= 0,
	.resindex_pcicfg_base	= -1,
	.resindex_imr_base	= -1,
	.irqindex_host_ipc	= -1,
	.resindex_dma_base	= -1,
	.chip_info = &cnl_chip_info,
	.default_fw_path = "intel/sof",
	.default_tplg_path = "intel/sof-tplg",
	.default_fw_filename = "sof-cnl.ri",
	.nocodec_tplg_filename = "sof-cnl-nocodec.tplg",
	.ops = &sof_cnl_ops,
};

static const struct sof_dev_desc cfl_desc = {
	.machines		= snd_soc_acpi_intel_cfl_machines,
	.alt_machines		= snd_soc_acpi_intel_cfl_sdw_machines,
	.use_acpi_target_states	= true,
	.resindex_lpe_base	= 0,
	.resindex_pcicfg_base	= -1,
	.resindex_imr_base	= -1,
	.irqindex_host_ipc	= -1,
	.resindex_dma_base	= -1,
	.chip_info = &cnl_chip_info,
	.default_fw_path = "intel/sof",
	.default_tplg_path = "intel/sof-tplg",
	.default_fw_filename = "sof-cfl.ri",
	.nocodec_tplg_filename = "sof-cnl-nocodec.tplg",
	.ops = &sof_cnl_ops,
};

static const struct sof_dev_desc cml_desc = {
	.machines		= snd_soc_acpi_intel_cml_machines,
	.alt_machines		= snd_soc_acpi_intel_cml_sdw_machines,
	.use_acpi_target_states	= true,
	.resindex_lpe_base	= 0,
	.resindex_pcicfg_base	= -1,
	.resindex_imr_base	= -1,
	.irqindex_host_ipc	= -1,
	.resindex_dma_base	= -1,
	.chip_info = &cnl_chip_info,
	.default_fw_path = "intel/sof",
	.default_tplg_path = "intel/sof-tplg",
	.default_fw_filename = "sof-cml.ri",
	.nocodec_tplg_filename = "sof-cnl-nocodec.tplg",
	.ops = &sof_cnl_ops,
};

/* PCI IDs */
static const struct pci_device_id sof_pci_ids[] = {
	{ PCI_DEVICE_DATA(INTEL, HDA_CNL_LP, &cnl_desc) },
	{ PCI_DEVICE_DATA(INTEL, HDA_CNL_H, &cfl_desc) },
	{ PCI_DEVICE_DATA(INTEL, HDA_CML_LP, &cml_desc) },
	{ PCI_DEVICE_DATA(INTEL, HDA_CML_H, &cml_desc) },
	{ PCI_DEVICE_DATA(INTEL, HDA_CML_S, &cml_desc) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, sof_pci_ids);

/* pci_driver definition */
static struct pci_driver snd_sof_pci_intel_cnl_driver = {
	.name = "sof-audio-pci-intel-cnl",
	.id_table = sof_pci_ids,
	.probe = hda_pci_intel_probe,
	.remove = sof_pci_remove,
	.shutdown = sof_pci_shutdown,
	.driver = {
		.pm = &sof_pci_pm,
	},
};
module_pci_driver(snd_sof_pci_intel_cnl_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_IMPORT_NS(SND_SOC_SOF_INTEL_HDA_COMMON);
MODULE_IMPORT_NS(SND_SOC_SOF_PCI_DEV);
