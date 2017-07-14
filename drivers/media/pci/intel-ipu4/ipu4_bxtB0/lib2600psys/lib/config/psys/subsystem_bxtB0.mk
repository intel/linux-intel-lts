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

############################################################################
# This file is used to specify versions and properties of PSYS firmware
# components. Please note that these are subsystem specific. System specific
# properties should go to system_$IPU_SYSVER.mk. Also the device versions
# should be defined under "devices" or should be taken from the SDK.
############################################################################

# Specify PSYS server context spaces for caching context from DDR
PSYS_SERVER_NOF_CACHES			= 4

# Activate loading params and storing stats DDR<->REGs with DMA
PSYS_USE_ISA_DMA				= 1

# Used in ISA module
PSYS_ISL_DPC_DPC_V2				= 0

# Assume OFS will be running concurrently with IPF, and prioritize according to rates of services on devproxy
CONCURRENT_OFS_IPF_PRIORITY_OPTIMIZATION_ENABLED	= 1

HAS_GMEM						= 1
# use DMA NCI for OFS Service to reduce load in tproxy
DMA_NCI_IN_OFS_SERVICE = 1

# Used by regmem
REGMEM_OFFSET                    = 0
REGMEM_SIZE                      = 16
REGMEM_WORD_BYTES                = 4

# TODO  use version naming scheme "v#" to decouple
# IPU_SYSVER from version.
PSYS_SERVER_MANIFEST_VERSION     = bxtB0
PSYS_RESOURCE_MODEL_VERSION      = bxtB0
PSYS_ACCESS_BLOCKER_VERSION      = v1

# Disable support for PPG protocol to save codesize
PSYS_HAS_PPG_SUPPORT			= 0

PSYS_SERVER_MAX_PROC_GRP_SIZE				= 3344
PSYS_SERVER_MAX_MANIFEST_SIZE				= 4808
PSYS_SERVER_MAX_CLIENT_PKG_SIZE				= 2240
PSYS_SERVER_MAX_NUMBER_OF_TERMINAL_SECTIONS	= 83
PSYS_SERVER_MAX_NUMBER_OF_ACB_CONNECTIONS	= 15
PSYS_SERVER_MAX_NUMBER_OF_ISA_CONNECTIONS	= 7
PSYS_SERVER_MAX_NUMBER_OF_PSA_CONNECTIONS	= 4

HAS_SPC				= 1
HAS_SPP0			= 1
HAS_SPP1			= 1
HAS_ISP0			= 1
HAS_ISP1			= 1
HAS_ISP2			= 1
HAS_ISP3			= 1
