/* Copyright (c) 2022 Intel Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 */
#include <linux/ptrace.h>
#include <linux/version.h>
#include <uapi/linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

struct pair {
	u64 val;
	u64 pid;
};

struct latency_record {
	u64 total;
	u64 count;
	u64 min;
	u64 max;
};

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, struct latency_record);
	__uint(max_entries, 256);
} ct_request_worker_time_map SEC(".maps");

u64 start_time_ct_request_worker = 0;
u64 min_time_ct_request_worker = 0;
u64 max_time_ct_request_worker = 0;
u64 average_time_ct_request_worker = 0;
SEC("kprobe/ct_incoming_request_worker_func")
int ct_request_worker_start(struct pt_regs *ctx)
{
	start_time_ct_request_worker = bpf_ktime_get_ns();
	return 0;
}

SEC("kretprobe/ct_incoming_request_worker_func")
int ct_request_worker_end(struct pt_regs *ctx)
{
	struct latency_record zero = {0, 0, 1ULL<<63, 0};
	struct latency_record *val;
	u64 key = 1234;
	u64 latency;

	val = bpf_map_lookup_elem(&ct_request_worker_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&ct_request_worker_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&ct_request_worker_time_map, &key);
		if (!val)
			return 0;
	}

	latency = bpf_ktime_get_ns() - start_time_ct_request_worker;
	val->total += latency;
	val->count += 1;
	val->min = val->min > latency ? latency : val->min;
	val->max = val->max < latency ? latency : val->max;

	return 0;
}

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, u64);
	__uint(max_entries, 256);
} ct_tasklet_time_map SEC(".maps");

u64 start_time_ct_tasklet = 0;
SEC("kprobe/ct_receive_tasklet_func")
int ct_tasklet_start(struct pt_regs *ctx)
{
	start_time_ct_tasklet = bpf_ktime_get_ns();
	return 0;
}

SEC("kretprobe/ct_receive_tasklet_func")
int ct_tasklet_end(struct pt_regs *ctx)
{
	u64 zero = 0, *val;
	u64 key = 1234;

	val = bpf_map_lookup_elem(&ct_tasklet_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&ct_tasklet_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&ct_tasklet_time_map, &key);
		if (!val)
			return 0;
	}

	(*val) += bpf_ktime_get_ns() - start_time_ct_tasklet;

	return 0;
}

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, u64);
	__uint(max_entries, 256);
} guc_irq_time_map SEC(".maps");

u64 start_time_guc_irq = 0;
SEC("kprobe/guc_irq_handler")
int guc_irq_start(struct pt_regs *ctx)
{
	char fmt[] = "guc irq start time %llu\n";
	start_time_guc_irq = bpf_ktime_get_ns();
	bpf_trace_printk(fmt, sizeof(fmt), bpf_ktime_get_ns());
	return 0;
}

SEC("kretprobe/guc_irq_handler")
int guc_irq_end(struct pt_regs *ctx)
{
	char fmt[] = "guc irq end time %llu\n";
	u64 zero = 0, *val;
	u64 key = 1234;

	val = bpf_map_lookup_elem(&guc_irq_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&guc_irq_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&guc_irq_time_map, &key);
		if (!val)
			return 0;
	}

	(*val) += bpf_ktime_get_ns() - start_time_guc_irq;
	bpf_trace_printk(fmt, sizeof(fmt), bpf_ktime_get_ns());

	return 0;
}
struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, struct latency_record);
	__uint(max_entries, 256);
} object_migration_time_map SEC(".maps");

u64 start_time_migrate = 0;
SEC("kprobe/i915_gem_object_migrate")
int migrate_start(struct pt_regs *ctx)
{

	char fmt[] = "migrate start time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	struct pair v = {
		.val = bpf_ktime_get_ns(),
		.pid = pid,
	};

	start_time_migrate = bpf_ktime_get_ns();
	bpf_trace_printk(fmt, sizeof(fmt), v.val, pid);
	return 0;
}

SEC("kretprobe/i915_gem_object_migrate")
int migrate_end(struct pt_regs *ctx)
{
	char fmt[] = "migrate end time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	struct latency_record zero = {0, 0, 1ULL<<63, 0};
	struct latency_record *val;
	u64 key = 1234;
	u64 latency;
	struct pair v = {
		.val = bpf_ktime_get_ns(),
		.pid = pid,
	};

	val = bpf_map_lookup_elem(&object_migration_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&object_migration_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&object_migration_time_map, &key);
		if (!val)
			return 0;
	}

	/* TODO: add start time/pid pair into a hash, indexed by pid
	 * find the hash table item by pid, use the item for time calc.
	 */

	latency = bpf_ktime_get_ns() - start_time_migrate;
	val->total += latency;
	val->count += 1;
	val->min = val->min > latency ? latency : val->min;
	val->max = val->max < latency ? latency : val->max;

	bpf_trace_printk(fmt, sizeof(fmt), v.val, pid);
	return 0;
}

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, u64);
	__uint(max_entries, 256);
} gem_create_time_map SEC(".maps");

u64 start_time_gem_create = 0;
SEC("kprobe/i915_gem_create_ioctl")
int gem_create_start(struct pt_regs *ctx)
{

	char fmt[] = "gem create start time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	struct pair v = {
		.val = bpf_ktime_get_ns(),
		.pid = pid,
	};

	start_time_gem_create = bpf_ktime_get_ns();
	bpf_trace_printk(fmt, sizeof(fmt), v.val, pid);
	return 0;
}

SEC("kretprobe/i915_gem_create_ioctl")
int gem_create_end(struct pt_regs *ctx)
{
	char fmt[] = "gem create end time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	u64 zero = 0, *val;
	u64 key = 1234;
	struct pair v = {
		.val = bpf_ktime_get_ns(),
		.pid = pid,
	};

	val = bpf_map_lookup_elem(&gem_create_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&gem_create_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&gem_create_time_map, &key);
		if (!val)
			return 0;
	}

	(*val) += bpf_ktime_get_ns() - start_time_gem_create;

	bpf_trace_printk(fmt, sizeof(fmt), v.val, pid);
	return 0;
}

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, u64);
	__uint(max_entries, 256);
} vma_bind_time_map SEC(".maps");

u64 start_time_vma_bind = 0;
SEC("kprobe/i915_vma_bind")
int vma_bind_start(struct pt_regs *ctx)
{
	char fmt[] = "vma bind start time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	start_time_vma_bind = bpf_ktime_get_ns();
	bpf_trace_printk(fmt, sizeof(fmt), start_time_vma_bind, pid);
	return 0;
}

SEC("kretprobe/i915_vma_bind")
int vma_bind_end(struct pt_regs *ctx)
{
	char fmt[] = "vma bind end time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	u64 zero = 0, *val;
	u64 key = 1234;

	val = bpf_map_lookup_elem(&vma_bind_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&vma_bind_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&vma_bind_time_map, &key);
		if (!val)
			return 0;
	}

	(*val) += bpf_ktime_get_ns() - start_time_vma_bind;
	bpf_trace_printk(fmt, sizeof(fmt), bpf_ktime_get_ns(), pid);

	return 0;
}

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, u64);
	__type(value, u64);
	__uint(max_entries, 256);
} user_fence_time_map SEC(".maps");

u64 start_time_user_fence = 0;
SEC("kprobe/i915_gem_wait_user_fence_ioctl")
int user_fence_start(struct pt_regs *ctx)
{
	char fmt[] = "user fence start time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	start_time_user_fence = bpf_ktime_get_ns();
	bpf_trace_printk(fmt, sizeof(fmt), start_time_user_fence, pid);
	return 0;
}

SEC("kretprobe/i915_gem_wait_user_fence_ioctl")
int user_fence_end(struct pt_regs *ctx)
{
	char fmt[] = "user fence end time %llu pid %llx\n";
	u64 pid = bpf_get_current_pid_tgid();
	u64 zero = 0, *val;
	u64 key = 1234;

	val = bpf_map_lookup_elem(&user_fence_time_map, &key);
	if (!val) {
		bpf_map_update_elem(&user_fence_time_map, &key, &zero, BPF_NOEXIST);
		val = bpf_map_lookup_elem(&user_fence_time_map, &key);
		if (!val)
			return 0;
	}

	(*val) += bpf_ktime_get_ns() - start_time_user_fence;
	bpf_trace_printk(fmt, sizeof(fmt), bpf_ktime_get_ns(), pid);

	return 0;
}


char _license[] SEC("license") = "GPL";
u32 _version SEC("version") = LINUX_VERSION_CODE;
