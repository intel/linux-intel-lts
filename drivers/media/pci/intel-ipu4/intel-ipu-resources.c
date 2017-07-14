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

#include <linux/bitmap.h>
#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <uapi/linux/intel-ipu4-psys.h>

#include "intel-ipu4-psys-abi.h"
#include "intel-ipu4-psys.h"
#include "intel-ipu-resources.h"

#define SANITY_CHECK 1

static struct intel_ipu_resource_api *ext_resources;

static const struct intel_ipu_resource_definitions default_defs = {
	.cells = intel_ipu_fw_psys_cell_types,
	.num_cells = IPU_FW_PSYS_N_CELL_ID,
	.num_cells_type = IPU_FW_PSYS_N_CELL_TYPE_ID,

	.dev_channels = intel_ipu4_num_dev_channels,
	.num_dev_channels = IPU_FW_PSYS_N_DEV_CHN_ID,

	.num_ext_mem_types = IPU_FW_PSYS_N_DATA_MEM_TYPE_ID,
	.num_ext_mem_ids = IPU_FW_PSYS_N_MEM_ID,
	.ext_mem_ids = intel_ipu_fw_psys_mem_size,

	.cell_mem_row = IPU_FW_PSYS_N_DATA_MEM_TYPE_ID,
	.cell_mem = &intel_ipu_fw_psys_cell_mem[0][0],

	.process.ext_mem_id = offsetof(struct ipu_fw_psys_process,
				       ext_mem_id[0]),
	.process.ext_mem_offset = offsetof(struct ipu_fw_psys_process,
					   ext_mem_offset[0]),
	.process.dev_chn_offset = offsetof(struct ipu_fw_psys_process,
					   dev_chn_offset[0]),
	.process.cell_id = offsetof(struct ipu_fw_psys_process, cell_id)
};

static const struct intel_ipu_resource_definitions *res_defs = &default_defs;

/********** Generic resource handling **********/

void intel_ipu_register_ext_resources(struct intel_ipu_resource_api *ptr)
{
	ext_resources = ptr;
	if (ptr->get_defs)
		res_defs = ptr->get_defs();
}
EXPORT_SYMBOL_GPL(intel_ipu_register_ext_resources);

void intel_ipu_unregister_ext_resources(void)
{
	ext_resources = NULL;
	res_defs = &default_defs;
}
EXPORT_SYMBOL_GPL(intel_ipu_unregister_ext_resources);

/*
 * Extension library gives byte offsets to its internal structures.
 * use those offsets to update fields. Without extension lib access
 * structures directly.
 */
static void set_process_cell_id(struct ipu_fw_psys_process *ptr, u8 value)
{
	if (ext_resources) {
		/* Byte offset */
		*((u8 *)ptr + res_defs->process.cell_id) = value;
		return;
	}
	ptr->cell_id = value;
}

static u8 get_process_cell_id(struct ipu_fw_psys_process *ptr)
{
	if (ext_resources)
		/* Byte offset */
		return *((u8 *)ptr + res_defs->process.cell_id);

	return ptr->cell_id;
}

static void set_process_dev_chn_offset(struct ipu_fw_psys_process *ptr,
				       u16 offset, u16 value)
{
	if (ext_resources) {
		/* dev_chn_offset is a byte offset, offset is u16 index */
		*((u16 *)((u8 *)ptr + res_defs->process.dev_chn_offset) +
		  offset) = value;
		return;
	}
	ptr->dev_chn_offset[offset] = value;
}

static void set_process_ext_mem_offset(struct ipu_fw_psys_process *ptr,
				       u16 offset, u16 value)
{
	if (ext_resources) {
		/* ext_mem_offset is a byte offset, offset is u16 index */
		*((u16 *)((u8 *)ptr + res_defs->process.ext_mem_offset) +
		  offset) = value;
		return;
	}
	ptr->ext_mem_offset[offset] = value;
}

static void set_process_ext_mem_id(struct ipu_fw_psys_process *ptr,
				   u16 offset, u8 value)
{
	if (ext_resources) {
		/* ext_mem_id is a byte offset, offset parameter is u8 index */
		*((u8 *)ptr + res_defs->process.ext_mem_id + offset) = value;
		return;
	}
	ptr->ext_mem_id[offset] = value;
}

static int intel_ipu4_resource_init(struct intel_ipu4_resource *res,
			     u32 id, int elements)
{
	if (elements <= 0) {
		res->bitmap = NULL;
		return 0;
	}

	res->bitmap = kcalloc(BITS_TO_LONGS(elements), sizeof(long),
			      GFP_KERNEL);
	if (!res->bitmap)
		return -ENOMEM;
	res->elements = elements;
	res->id = id;
	return 0;
}

static unsigned long
intel_ipu4_resource_alloc(struct intel_ipu4_resource *res, int n,
			  struct intel_ipu4_resource_alloc *alloc,
			  enum intel_ipu_resource_type type)
{
	unsigned long p;

	if (n <= 0) {
		alloc->elements = 0;
		return 0;
	}

	if (!res->bitmap)
		return (unsigned long)(-ENOSPC);

	p = bitmap_find_next_zero_area(res->bitmap, res->elements, 0, n, 0);
	if (SANITY_CHECK)
		alloc->resource = NULL;

	if (p >= res->elements)
		return (unsigned long)(-ENOSPC);
	bitmap_set(res->bitmap, p, n);
	alloc->resource = res;
	alloc->elements = n;
	alloc->pos = p;
	alloc->type = type;

	return p;
}

static void intel_ipu4_resource_free(struct intel_ipu4_resource_alloc *alloc)
{
	if (alloc->elements <= 0)
		return;

	bitmap_clear(alloc->resource->bitmap, alloc->pos, alloc->elements);
	if (SANITY_CHECK)
		alloc->resource = NULL;
}

static void intel_ipu4_resource_cleanup(struct intel_ipu4_resource *res)
{
	kfree(res->bitmap);
	if (SANITY_CHECK)
		res->bitmap = NULL;
}

/********** IPU4 PSYS-specific resource handling **********/

static struct ipu_fw_psys_program_manifest *
intel_ipu4_resource_get_program_manifest(
	const struct ipu_fw_psys_program_group_manifest *manifest,
	const unsigned int program_index)
{
	struct ipu_fw_psys_program_manifest *prg_manifest_base;
	u8 *program_manifest = NULL;
	u8 program_count;
	unsigned int i;

	program_count = manifest->program_count;

	prg_manifest_base = (struct ipu_fw_psys_program_manifest *)
		((char *)manifest +
		manifest->program_manifest_offset);
	if (program_index < program_count) {
		program_manifest = (u8 *)prg_manifest_base;
		for (i = 0; i < program_index; i++)
			program_manifest +=
				((struct ipu_fw_psys_program_manifest *)
				 program_manifest)->size;
	}

	return (struct ipu_fw_psys_program_manifest *)program_manifest;
}

static int intel_ipu4_psys_get_program_manifest_by_process(
	struct ipu_fw_generic_program_manifest *gen_pm,
	const struct ipu_fw_psys_program_group_manifest *pg_manifest,
	struct ipu_fw_psys_process *process)
{
	u32 process_id = process->ID;
	int programs = pg_manifest->program_count;
	int i;

	if (ext_resources)
		return ext_resources->get_program_manifest_by_process(
			gen_pm,
			pg_manifest,
			process);

	for (i = 0; i < programs; i++) {
		u32 program_id;
		struct ipu_fw_psys_program_manifest *pm =
			intel_ipu4_resource_get_program_manifest(
				pg_manifest, i);
		if (!pm)
			continue;
		program_id = pm->ID;
		if (program_id == process_id) {
			gen_pm->dev_chn_size = pm->dev_chn_size;
			gen_pm->cell_id = pm->cell_id;
			gen_pm->cell_type_id = pm->cell_type_id;
			gen_pm->ext_mem_size = pm->ext_mem_size;

			return 0;
		}
	}
	return -ENOENT;
}

int intel_ipu4_psys_resource_pool_init(
				struct intel_ipu4_psys_resource_pool *pool)
{
	int i, j, ret;
	pool->cells = 0;

	for (i = 0; i < res_defs->num_dev_channels; i++) {
		ret = intel_ipu4_resource_init(&pool->dev_channels[i], i,
					       res_defs->dev_channels[i]);
		if (ret)
			goto error;
	}

	for (j = 0; j < res_defs->num_ext_mem_ids; j++) {
		ret = intel_ipu4_resource_init(&pool->ext_memory[j], j,
					       res_defs->ext_mem_ids[j]);
		if (ret)
			goto memory_error;
	}

	return 0;

memory_error:
	for (j--; j >= 0; j--)
		intel_ipu4_resource_cleanup(&pool->ext_memory[j]);

error:
	for (i--; i >= 0; i--)
		intel_ipu4_resource_cleanup(&pool->dev_channels[i]);
	return ret;
}

void intel_ipu4_psys_resource_pool_cleanup(
				struct intel_ipu4_psys_resource_pool *pool)
{
	u32 i;

	for (i = 0; i < res_defs->num_dev_channels; i++)
		intel_ipu4_resource_cleanup(&pool->dev_channels[i]);

	for (i = 0; i < res_defs->num_ext_mem_ids; i++)
		intel_ipu4_resource_cleanup(&pool->ext_memory[i]);
}

void intel_ipu4_psys_resource_alloc_init(
				struct intel_ipu4_psys_resource_alloc *alloc)
{
	alloc->cells = 0;
	alloc->resources = 0;
}

static int intel_ipu4_psys_allocate_one_resource(const struct device *dev,
			struct ipu_fw_psys_process *process,
			struct intel_ipu4_resource *resource,
			struct ipu_fw_generic_program_manifest *pm,
			u32 resource_id,
			struct intel_ipu4_psys_resource_alloc *alloc)
{
	const u16 resource_req = pm->dev_chn_size[resource_id];
	unsigned long retl;

	if (resource_req <= 0)
		return 0;

	if (alloc->resources >= INTEL_IPU4_MAX_RESOURCES) {
		dev_err(dev, "out of resource handles\n");
		return -ENOSPC;
	}

	retl = intel_ipu4_resource_alloc(resource, resource_req,
				 &alloc->resource_alloc[alloc->resources],
				 INTEL_IPU_RESOURCE_DEV_CHN);
	if (IS_ERR_VALUE(retl)) {
		dev_dbg(dev, "out of resources\n");
		return (int)retl;
	}
	alloc->resources++;
	set_process_dev_chn_offset(process, resource_id, retl);

	return 0;
}

/*
 * ext_mem_type_id is a generic type id for memory (like DMEM, VMEM)
 * ext_mem_bank_id is detailed type id for  memory (like DMEM0, DMEM1 etc.)
 */
static int intel_ipu4_psys_allocate_memory_resource(const struct device *dev,
			struct ipu_fw_psys_process *process,
			struct intel_ipu4_resource *resource,
			struct ipu_fw_generic_program_manifest *pm,
			u32 ext_mem_type_id,
			u32 ext_mem_bank_id,
			struct intel_ipu4_psys_resource_alloc *alloc)
{
	const u16 memory_resource_req = pm->ext_mem_size[ext_mem_type_id];
	unsigned long retl;

	if (memory_resource_req <= 0)
		return 0;

	if (alloc->resources >= INTEL_IPU4_MAX_RESOURCES) {
		dev_err(dev, "out of resource handles\n");
		return -ENOSPC;
	}

	retl = intel_ipu4_resource_alloc(resource, memory_resource_req,
				 &alloc->resource_alloc[alloc->resources],
				 INTEL_IPU_RESOURCE_EXT_MEM);
	if (IS_ERR_VALUE(retl)) {
		dev_dbg(dev, "out of memory resources\n");
		return (int)retl;
	}

	alloc->resources++;

	set_process_ext_mem_id(process, ext_mem_type_id, ext_mem_bank_id);
	set_process_ext_mem_offset(process, ext_mem_type_id, retl);

	return 0;
}

/*
 * Allocate resources for pg from `pool'. Mark the allocated
 * resources into `alloc'. Returns 0 on success, -ENOSPC
 * if there are no enough resources, in which cases resources
 * are not allocated at all, or some other error on other conditions.
 */
int intel_ipu4_psys_allocate_resources(const struct device *dev,
			struct ipu_fw_psys_process_group *pg,
			void *pg_manifest,
			struct intel_ipu4_psys_resource_alloc *alloc,
			struct intel_ipu4_psys_resource_pool *pool)
{
	u32 resid;
	u32 mem_type_id;
	int ret, i;
	u16 *process_offset_table = (u16 *)((u8 *)pg + pg->processes_offset);
	uint8_t processes = pg->process_count;
	u32 cells = 0;

	for (i = 0; i < processes; i++) {
		u32 cell;
		struct ipu_fw_psys_process *process =
			(struct ipu_fw_psys_process *)
			((char *)pg + process_offset_table[i]);
		struct ipu_fw_generic_program_manifest pm;

		if (process == NULL) {
			dev_err(dev, "can not get process\n");
			ret = -ENOENT;
			goto free_out;
		}

		ret = intel_ipu4_psys_get_program_manifest_by_process(
			&pm, pg_manifest, process);
		if (ret < 0) {
			dev_err(dev, "can not get manifest\n");
			goto free_out;
		}

		if ((pm.cell_id != res_defs->num_cells &&
		     pm.cell_type_id == res_defs->num_cells_type)) {
			cell = get_process_cell_id(process);
		} else {
			/* Find a free cell of desired type */
			u32 type = pm.cell_type_id;

			for (cell = 0; cell < res_defs->num_cells; cell++)
				if (res_defs->cells[cell] == type &&
				    ((pool->cells | cells) & (1 << cell)) == 0)
					break;
			if (cell >= res_defs->num_cells) {
				dev_dbg(dev, "no free cells of right type\n");
				ret = -ENOSPC;
				goto free_out;
			}
			pg->resource_bitmap |= 1 << cell;
			set_process_cell_id(process, cell);
		}
		cells |= 1 << cell;
		if (pool->cells & cells) {
			dev_dbg(dev, "out of cell resources\n");
			ret = -ENOSPC;
			goto free_out;
		}

		for (resid = 0; resid < res_defs->num_dev_channels; resid++) {
			ret = intel_ipu4_psys_allocate_one_resource(dev,
				process, &pool->dev_channels[resid],
				&pm, resid, alloc);
			if (ret)
				goto free_out;
		}

		for (mem_type_id = 0;
		     mem_type_id < res_defs->num_ext_mem_types;
		     mem_type_id++) {
			u32 mem_bank_id;

			mem_bank_id =
			    res_defs->cell_mem[res_defs->cell_mem_row *
					       cell + mem_type_id];
			if (mem_bank_id == res_defs->num_ext_mem_ids)
				continue;

			ret = intel_ipu4_psys_allocate_memory_resource(dev,
				process, &pool->ext_memory[mem_bank_id],
				&pm, mem_type_id,
				mem_bank_id,
				alloc);
			if (ret)
				goto free_out;
		}

	}
	alloc->cells |= cells;
	pool->cells |= cells;
	return 0;

free_out:
	for (; i >= 0; i--) {
		struct ipu_fw_psys_process *process =
			(struct ipu_fw_psys_process *)
			((char *)pg + process_offset_table[i]);
		struct ipu_fw_generic_program_manifest pm;
		int retval;

		if (process == NULL)
			break;
		retval = intel_ipu4_psys_get_program_manifest_by_process(
			&pm, pg_manifest, process);
		if (retval < 0)
			break;
		if ((pm.cell_id != res_defs->num_cells &&
		     pm.cell_type_id == res_defs->num_cells_type))
			continue;
		pg->resource_bitmap &= ~(1 << get_process_cell_id(process));
		set_process_cell_id(process, 0);
	}

	intel_ipu4_psys_free_resources(alloc, pool);
	return ret;
}

int intel_ipu4_psys_move_resources(const struct device *dev,
			   struct intel_ipu4_psys_resource_alloc *alloc,
			   struct intel_ipu4_psys_resource_pool *source_pool,
			   struct intel_ipu4_psys_resource_pool *target_pool)
{
	int i;

	if (target_pool->cells & alloc->cells) {
		dev_dbg(dev, "out of cell resources\n");
		return -ENOSPC;
	}

	for (i = 0; i < alloc->resources; i++) {
		unsigned long bitmap = 0;
		unsigned int id = alloc->resource_alloc[i].resource->id;
		unsigned long fbit, end;

		switch (alloc->resource_alloc[i].type) {
		case INTEL_IPU_RESOURCE_DEV_CHN:
			bitmap_set(&bitmap, alloc->resource_alloc[i].pos,
				   alloc->resource_alloc[i].elements);
			if (*target_pool->dev_channels[id].bitmap & bitmap)
				return -ENOSPC;
			break;
		case INTEL_IPU_RESOURCE_EXT_MEM:
			end = alloc->resource_alloc[i].elements +
				alloc->resource_alloc[i].pos;

			fbit = find_next_bit(target_pool->ext_memory[id].bitmap,
					     end,
					     alloc->resource_alloc[i].pos);
			/* if find_next_bit returns "end" it didn't find 1bit */
			if (end != fbit)
				return -ENOSPC;
			break;
		default:
			dev_err(dev, "Illegal resource type\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < alloc->resources; i++) {
		u32 id = alloc->resource_alloc[i].resource->id;

		switch (alloc->resource_alloc[i].type) {
		case INTEL_IPU_RESOURCE_DEV_CHN:
			bitmap_set(target_pool->dev_channels[id].bitmap,
				   alloc->resource_alloc[i].pos,
				   alloc->resource_alloc[i].elements);
			intel_ipu4_resource_free(&alloc->resource_alloc[i]);
			alloc->resource_alloc[i].resource =
				&target_pool->dev_channels[id];
			break;
		case INTEL_IPU_RESOURCE_EXT_MEM:
			bitmap_set(target_pool->ext_memory[id].bitmap,
				   alloc->resource_alloc[i].pos,
				   alloc->resource_alloc[i].elements);
			intel_ipu4_resource_free(&alloc->resource_alloc[i]);
			alloc->resource_alloc[i].resource =
				&target_pool->ext_memory[id];
			break;
		default:
			/*
			 * Just keep compiler happy. This case failed already
			 * in above loop.
			 */
			break;
		}
	}

	target_pool->cells |= alloc->cells;
	source_pool->cells &= ~alloc->cells;

	return 0;
}

/* Free resources marked in `alloc' from `resources' */
void intel_ipu4_psys_free_resources(
			struct intel_ipu4_psys_resource_alloc *alloc,
			struct intel_ipu4_psys_resource_pool *pool)
{
	unsigned int i;

	pool->cells &= ~alloc->cells;
	alloc->cells = 0;
	for (i = 0; i < alloc->resources; i++)
		intel_ipu4_resource_free(&alloc->resource_alloc[i]);
	alloc->resources = 0;
}
