# #
# Support for Intel Camera Imaging ISP subsystem.
# Copyright (c) 2010 - 2017, Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details
#

LOGICAL_FW_INPUT_SYSTEM			= ipu_system
LOGICAL_FW_PROCESSING_SYSTEM		= ipu_system
LOGICAL_FW_IPU_SYSTEM			= ipu_system
LOGICAL_FW_ISP_SYSTEM			= idsp_system
SP_CONTROL_CELL				= sp_control
SP_PROXY_CELL				= sp_proxy
ISP_CELL				= idsp
# The non-capital define isp2601 is used in the sdk, in order to distinguish
# between different isp versions the ISP_CELL_IDENTIFIER define is added.
ISP_CELL_IDENTIFIER			= IDSP

# Enable LLVM/Volcano for SP server builds.
VOLCANO_SP_IPU5                         = 1

# Enable LLVM/Volcano for IDSP builds
idsp_VOLCANO                            = 1
idsp_SCHEDULER                          = llvm
ENABLE_NO_ALIAS_FOR_LLVM                = 1

# The ISL-IS has two data paths - one for handling the main camera which may go
# directly to PS (the nonsoc path).
# The other path can handle up to 8 streams for SoC sensors and additional raw
# sensors (the soc path),
# In IPU5 the nonsoc path has str2mmio devices instead of s2v devices
HAS_S2M_IN_ISYS_ISL_NONSOC_PATH		= 1
HAS_S2V_IN_ISYS_ISL_NONSOC_PATH		= 0
# ISL-IS non-SoC path doesn't have ISA in IPU5-A0
HAS_ISA_IN_ISYS_ISL			= 0
HAS_PAF_IN_ISYS_ISL			= 0
HAS_DPC_PEXT_IN_ISYS_ISL		= 0
HAS_PMA_IF                              = 1

HAS_ISL_PIFCONV				= 1
HAS_OFS_OUT_CONVERTER			= 1
HAS_DFM					= 1

HAS_MIPIBE_IN_PSYS_ISL			= 0

HAS_SIS					= 1

DLI_SYSTEM				?= ipu5_system
OFS_OUTPUT_TO_TRANSFER_VMEM		= 1
RESOURCE_MANAGER_VERSION		= v3
MEM_RESOURCE_VALIDATION_ERROR		= 1
PROGDESC_ACC_SYMBOLS_VERSION		= v2
DEVPROXY_INTERFACE_VERSION		= v2
FW_ABI_IPU_TYPES_VERSION         = v2

HAS_ONLINE_MODE_SUPPORT_IN_ISYS_PSYS	= 1

MMU_INTERFACE_VERSION                   = v3
DEVICE_ACCESS_VERSION                   = v3
PSYS_SERVER_VERSION                     = v4
PSYS_SERVER_LOADER_VERSION              = v2
PSYS_HW_VERSION                         = GLV_A0_HW

# Enable FW_DMA for loading firmware
PSYS_SERVER_ENABLE_FW_LOAD_DMA          = 1

# The SPA device itself did not change for IPU5, only the (number of) instances.
# This requires a (partially) different implementation of the NCI
NCI_SPA_VERSION                         = v2
MANIFEST_TOOL_VERSION                   = v3
PSYS_CON_MGR_TOOL_VERSION               = v2
# TODO: Should be removed after performance issues OTF are solved
PSYS_PROC_MGR_VERSION               	= v2
IPU_RESOURCES_VERSION                   = v3

HAS_ACC_CLUSTER_PAF_PAL                 = 1
HAS_ACC_CLUSTER_PEXT_PAL                = 1

# TODO  use version naming scheme "v#" to decouple
# IPU_SYSVER from version.
PARAMBINTOOL_ISA_INIT_VERSION           = glvA0

#Enable DEC400
HAS_DEC400                              = 1

# Select EQC2EQ version
# Version 1: uniform address space, equal EQ addresses regardless of EQC device
# Version 2: multiple addresses per EQ, depending on location of EQC device
EQC2EQ_VERSION                          = v2
