/*
 * Copyright (c) 2015--2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "intel-ipu5-resources.h"
#include "intel-ipu4-resources.h"
#include "intel-ipu4-psys-abi.h"

#ifndef INTEL_IPU4_RESOURCES_H
#define INTEL_IPU4_RESOURCES_H

/********** Generic resource handling **********/

extern const u32 intel_ipu_fw_psys_cell_types[];
extern const u16 intel_ipu4_num_dev_channels[];
extern const uint16_t intel_ipu_fw_psys_mem_size[];

extern const enum intel_ipu_mem_id intel_ipu_fw_psys_cell_mem
			[IPU_FW_PSYS_N_CELL_ID][IPU_FW_PSYS_N_DATA_MEM_TYPE_ID];

/* Opaque structure. Do not access fields. */
struct intel_ipu4_resource {
	u32 id;
	int elements;		/* Number of elements available to allocation */
	unsigned long *bitmap;	/* Allocation bitmap, a bit for each element */
};

enum intel_ipu_resource_type {
	INTEL_IPU_RESOURCE_DEV_CHN = 0,
	INTEL_IPU_RESOURCE_EXT_MEM
};

/* Allocation of resource(s) */
/* Opaque structure. Do not access fields. */
struct intel_ipu4_resource_alloc {
	enum intel_ipu_resource_type type;
	struct intel_ipu4_resource *resource;
	int elements;
	int pos;
};

/********** IPU4 PSYS-specific resource handling **********/

#define INTEL_IPU4_MAX_RESOURCES 32

struct ipu_fw_psys_program_group_manifest {
	u64 kernel_bitmap;
	u32 ID;
	u32 program_manifest_offset;
	u32 terminal_manifest_offset;
	u32 private_data_offset;
	u16 size;
	u8 alignment;
	u8 kernel_count;
	u8 program_count;
	u8 terminal_count;
	u8 subgraph_count;
	u8 reserved[1];
};

struct ipu_fw_psys_program_manifest {
	u64 kernel_bitmap;
	u32 ID;
	u32 program_type;
	s32 parent_offset;
	u32 program_dependency_offset;
	u32 terminal_dependency_offset;
	u16 size;
	u16 int_mem_size[IPU_FW_PSYS_N_MEM_TYPE_ID];
	u16 ext_mem_size[IPU_FW_PSYS_N_DATA_MEM_TYPE_ID];
	u16 dev_chn_size[IPU_FW_PSYS_N_DEV_CHN_ID];
	u8 cell_id;
	u8 cell_type_id;
	u8 program_dependency_count;
	u8 terminal_dependency_count;
	u8 reserved[4];
};

struct ipu_fw_generic_program_manifest {
	u16 *dev_chn_size;
	u16 *ext_mem_size;
	u8 cell_id;
	u8 cell_type_id;
};

struct ipu_fw_generic_process {
	u16 ext_mem_id;
	u16 ext_mem_offset;
	u16 dev_chn_offset;
	u16 cell_id;
};

struct intel_ipu_resource_definitions {
	u32 num_cells;
	u32 num_cells_type;
	const u32 *cells;
	u32 num_dev_channels;
	const u16 *dev_channels;

	u32 num_ext_mem_types;
	u32 num_ext_mem_ids;
	const u16 *ext_mem_ids;

	u32 cell_mem_row;
	const enum intel_ipu_mem_id *cell_mem;
	struct ipu_fw_generic_process process;
};

/*
 * This struct represents all of the currently allocated
 * resources from IPU model. It is used also for allocating
 * resources for the next set of PGs to be run on IPU
 * (ie. those PGs which are not yet being run and which don't
 * yet reserve real IPU4 resources).
 */
#define IPU_PSYS_RESOURCE_OVERALLOC 2 /* Some room for ABI / ext lib delta */
struct intel_ipu4_psys_resource_pool {
	u32 cells;	/* Bitmask of cells allocated */
	struct intel_ipu4_resource dev_channels[IPU_FW_PSYS_N_DEV_CHN_ID +
						IPU_PSYS_RESOURCE_OVERALLOC];
	struct intel_ipu4_resource ext_memory[IPU_FW_PSYS_N_MEM_ID +
					      IPU_PSYS_RESOURCE_OVERALLOC];
};

/*
 * This struct keeps book of the resources allocated for a specific PG.
 * It is used for freeing up resources from struct intel_ipu4_psys_resources
 * when the PG is released from IPU4 (or model of IPU4).
 */
struct intel_ipu4_psys_resource_alloc {
	u32 cells;	/* Bitmask of cells needed */
	struct intel_ipu4_resource_alloc
		resource_alloc[INTEL_IPU4_MAX_RESOURCES];
	int resources;
};

struct intel_ipu_resource_api {
	const struct intel_ipu_resource_definitions *(*get_defs)(void);
	int (*get_program_manifest_by_process)(
		struct ipu_fw_generic_program_manifest *gen_pm,
		const struct ipu_fw_psys_program_group_manifest *pg_manifest,
		struct ipu_fw_psys_process *process);
};

struct ipu_fw_psys_process_group;

int intel_ipu4_psys_resource_pool_init(
				struct intel_ipu4_psys_resource_pool *pool);

void intel_ipu4_psys_resource_pool_cleanup(
				struct intel_ipu4_psys_resource_pool *pool);

void intel_ipu4_psys_resource_alloc_init(
				struct intel_ipu4_psys_resource_alloc *alloc);

int intel_ipu4_psys_allocate_resources(const struct device *dev,
			       struct ipu_fw_psys_process_group *pg,
			       void *pg_manifest,
			       struct intel_ipu4_psys_resource_alloc *alloc,
			       struct intel_ipu4_psys_resource_pool *pool);
int intel_ipu4_psys_move_resources(const struct device *dev,
			   struct intel_ipu4_psys_resource_alloc *alloc,
			   struct intel_ipu4_psys_resource_pool *source_pool,
			   struct intel_ipu4_psys_resource_pool *target_pool);

void intel_ipu4_psys_free_resources(
			struct intel_ipu4_psys_resource_alloc *alloc,
			struct intel_ipu4_psys_resource_pool *pool);

void intel_ipu_register_ext_resources(struct intel_ipu_resource_api *ptr);
void intel_ipu_unregister_ext_resources(void);

#endif
