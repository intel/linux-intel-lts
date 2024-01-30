/* Copyright (c) 2022 Intel Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <uapi/linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "trace_common.h"
#include <linux/version.h>

struct counter_rec {
	u64 count;
	u64 size;
};

struct combined_key {
	u32 key1;
	u32 key2;
};

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(key_size, sizeof(u64));
	__uint(value_size, sizeof(u64));
	__uint(max_entries, 1);
} i915_context_create_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, struct counter_rec);
	__uint(max_entries, 1);
} object_create_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(key_size, sizeof(u64));
	__uint(value_size, sizeof(u64));
	__uint(max_entries, 1);
} cpu_object_fault_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, struct combined_key);/* key1 = src, key2=dst*/
	__type(value, struct counter_rec);
	__uint(max_entries, 256);
} object_migrate_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, struct counter_rec);
	__uint(max_entries, 1);
} gem_shrink_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(key_size, sizeof(u64));/* 0x10 for read 0x11 for write */
	__uint(value_size, sizeof(u64));
	__uint(max_entries, 256);
} gpu_fault_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(key_size, sizeof(u64));/* 11 for smem 1111 for lmem */
	__uint(value_size, sizeof(u64));
	__uint(max_entries, 8);
} vm_prefetch_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, struct counter_rec);/* count how many bytes of memory bound*/
	__uint(max_entries, 1);
} vm_bind_count_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, struct combined_key); /* key1: engine class key2: engine instance*/
	__type(value, u64);
	__uint(max_entries, 256);
} access_counter_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(key_size, sizeof(u64));
	__uint(value_size, sizeof(u64));
	__uint(max_entries, 1);
} ctx_create_map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64); /* 11 for full; 1111 for range*/
	__type(value, struct counter_rec);
	__uint(max_entries, 8);
} tlb_invalidate_map SEC(".maps");

struct intel_access_counter_ctx {
	u64 pad;
	u32 dev;
	u32 id;
	u32 region;
	u32 sub_region;
	u32 asid;
	u32 engine_class;
	u32 engine_instance;
	u64 vaddr;
};

struct intel_tlb_invalidate_ctx {
	u64 pad;
	u64 dev;
	u32 id;
	u64 start;
	u64 len;
};

struct i915_gem_object_migrate_ctx {
	u64 pad;
	u64 dev;
	u64 obj;
	u64 size;
	u32 src;
	u32 dst;
};

struct i915_context_create_ctx {
	u64 pad;
	u32 dev;
	u64 ctx;
	u64 vm;
};

struct intel_context_create_ctx {
	u64 pad;
	u32 guc;
	u32 pin_count;
	u32 sched_state;
	u8  guc_prio;
};

struct i915_gem_object_create_ctx {
	u64 pad;
	u64 obj;
	u64 size;
};

struct i915_gem_object_fault_ctx {
	u64 pad;
	u64 obj;
	u64 addr;
	u64 index;
	u8  gtt;
	u8  write;
};

struct i915_gem_shrink_ctx {
	u64 pad;
	u32 dev;
	u64 target;
	u32 flags;
};

struct i915_mm_fault_ctx {
	u64 pad;
	u64 dev;
	u64 vm;
	u64 obj;
	u64 size;
	u64 addr;
	u32 asid;
	u8  access_type;
	u8  fault_type;
	u8  engine_class;
	u8  engine_instance;
};

struct i915_vm_prefetch_ctx {
	u64 pad;
	u64 dev;
	u64 start;
	u64 len;
	u32 region;
};

/* see format at
 * /sys/kernel/debug/tracing/events/i915/i915_vma_bind/format
 */
struct i915_vma_bind_ctx {
	u64 pad;
	u64 obj;
	u64 vm;
	u64 offset;
	u64 size;
	u32 flags;
};

/* see all the tracepoints under /sys/kernel/debug/tracing */
SEC("tracepoint/i915/i915_vma_bind")
int i915_vma_bind(struct i915_vma_bind_ctx *ctx)
{
	u64 size = ctx->size;
	struct counter_rec zero = {0, 0};
	struct counter_rec *val;
	u64 key = 1234;

	val = bpf_map_lookup_elem(&vm_bind_count_map, &key);
	if (!val) {
		bpf_map_update_elem(&vm_bind_count_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&vm_bind_count_map, &key);
		if (!val)
			return 0;
	}

	val->count += 1;
	val->size += size;

	return 0;
}

SEC("tracepoint/i915/i915_gem_object_migrate")
int i915_gem_object_migrate(struct i915_gem_object_migrate_ctx *ctx)
{
	u64 size = ctx->size;
	struct counter_rec zero = {0, 0};
	struct counter_rec *val;
	u32 src = (ctx->src == 0 || ctx->src == 5) ? 11 : 1111;
	u32 dst = (ctx->dst == 0 || ctx->dst == 5) ? 11 : 1111;
	struct combined_key key = {src, dst};

	val = bpf_map_lookup_elem(&object_migrate_map, &key);
	if (!val) {
		bpf_map_update_elem(&object_migrate_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&object_migrate_map, &key);
		if (!val)
			return 0;
	}

	val->count += 1;
	val->size += size;

	return 0;
}

SEC("tracepoint/i915/intel_context_create")
int intel_context_create(struct intel_context_create_ctx *ctx)
{
	u64 zero = 0, *val;
	u64 key = 1234;

	val = bpf_map_lookup_elem(&ctx_create_map, &key);
	if (!val) {
		bpf_map_update_elem(&ctx_create_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&ctx_create_map, &key);
		if (!val)
			return 0;
	}

	(*val) += 1;

	return 0;
}

SEC("tracepoint/i915/i915_mm_fault")
int gpu_fault(struct i915_mm_fault_ctx *ctx)
{
	u64 zero = 0, *val;
	u64 key = ctx->access_type | 0x10;
	//char fmt[] = "gpu mm fault, access type %llx\n";

	//bpf_trace_printk(fmt, sizeof(fmt), key);
	val = bpf_map_lookup_elem(&gpu_fault_map, &key);
	if (!val) {
		bpf_map_update_elem(&gpu_fault_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&gpu_fault_map, &key);
		if (!val)
			return 0;
	}

	(*val) += 1;

	return 0;
}

SEC("tracepoint/i915/i915_vm_prefetch")
int vm_prefetch(struct i915_vm_prefetch_ctx *ctx)
{
	u64 zero = 0, *val;
	u64 key = (ctx->region == 0 || ctx->region == 5) ? 11 : 1111;

	val = bpf_map_lookup_elem(&vm_prefetch_map, &key);
	if (!val) {
		bpf_map_update_elem(&vm_prefetch_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&vm_prefetch_map, &key);
		if (!val)
			return 0;
	}

	(*val) += ctx->len;

	return 0;
}

SEC("tracepoint/i915/intel_tlb_invalidate")
int tlb_invalidate(struct intel_tlb_invalidate_ctx *ctx)
{
	struct counter_rec zero = {0, 0}, *val;
	u64 key = (ctx->len == 0) ? 11 : 1111;

	val = bpf_map_lookup_elem(&tlb_invalidate_map, &key);
	if (!val) {
		bpf_map_update_elem(&tlb_invalidate_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&tlb_invalidate_map, &key);
		if (!val)
			return 0;
	}

	val->size  += ctx->len;
	val->count  += 1;

	return 0;
}

SEC("tracepoint/i915/i915_gem_object_create")
int i915_gem_object_create(struct i915_gem_object_create_ctx *ctx)
{
	struct counter_rec *val;
	struct counter_rec zero = {0, 0};
	u64 key = 0;
	u64 obj_size;

	obj_size = ctx->size;
	val = bpf_map_lookup_elem(&object_create_map, &key);
	if (!val) {
		bpf_map_update_elem(&object_create_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&object_create_map, &key);
		if (!val)
			return 0;
	}

	val->count += 1;
	val->size += obj_size;

	return 0;
}

SEC("tracepoint/i915/i915_gem_object_fault")
int i915_gem_object_fault(struct i915_gem_object_fault_ctx *ctx)
{
	u64 zero = 0, *val;
	u64 key = 0;

	val = bpf_map_lookup_elem(&cpu_object_fault_map, &key);
	if (!val) {
		bpf_map_update_elem(&cpu_object_fault_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&cpu_object_fault_map, &key);
		if (!val)
			return 0;
	}

	(*val) += 1;

	return 0;
}

SEC("tracepoint/i915/intel_access_counter")
int access_counter(struct intel_access_counter_ctx *ctx)
{
	u64 zero = 0, *val;
	u32 class = ctx->engine_class;
	u32 instance = ctx->engine_instance;
	struct combined_key key = {class, instance};

	val = bpf_map_lookup_elem(&access_counter_map, &key);
	if (!val) {
		bpf_map_update_elem(&access_counter_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&access_counter_map, &key);
		if (!val)
			return 0;
	}

	(*val) += 1;

	return 0;
}

SEC("tracepoint/i915/i915_gem_shrink")
int i915_gem_shrink(struct i915_gem_shrink_ctx *ctx)
{
	struct counter_rec *val;
	struct counter_rec zero = {0, 0};
	u64 key = 1234;
	u64 shrink_size;

	shrink_size = ctx->target;
	val = bpf_map_lookup_elem(&gem_shrink_map, &key);
	if (!val) {
		bpf_map_update_elem(&gem_shrink_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&gem_shrink_map, &key);
		if (!val)
			return 0;
	}

	val->count += 1;
	val->size += shrink_size;

	return 0;
}

char _license[] SEC("license") = "GPL";
u32 _version SEC("version") = LINUX_VERSION_CODE;
