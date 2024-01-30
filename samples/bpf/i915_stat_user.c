/* Copyright (c) 2022 Intel Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 */
#include <stdio.h>
#include <unistd.h>
#include <sys/resource.h>

#include <linux/bpf.h>
#include <bpf/bpf.h>
#include <bpf/libbpf.h>

struct counter_rec {
	__u64 count;
	__u64 size;
};

struct combined_key {
	__u32 key1;
	__u32 key2;
};


static struct bpf_object *obj;

static void print_vma_bind_counter(void)
{
	__u64 key = 1234;
	struct counter_rec value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "vm_bind_count_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return;

	printf("%llu bytes (%llu vma) bound to gpu\n", value.size, value.count);
}

static void print_gem_shrink(void)
{
	__u64 key = 1234;
	struct counter_rec value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "gem_shrink_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return;

	printf("gem shrink count %llu, total size %llu\n", value.count, value.size);
}

static void print_object_migrate_counter(void)
{
	struct combined_key key = {0, 0}, next_key;
	__u32 src, dst;
	struct counter_rec value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "object_migrate_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			src = next_key.key1;
			dst = next_key.key2;
			printf("migrated %llu bytes(%llu objects) from %s to %s\n",
				value.size, value.count,
				(src == 11) ? "smem":"lmem",
				(dst == 11) ? "smem":"lmem");
		}
		key = next_key;
	}
}

static void print_tlb_invalidate_counter(void)
{
	__u64 key = 0, next_key;
	struct counter_rec value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "tlb_invalidate_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			if (next_key == 11)
				printf("Full TLB invalidate count: %llu\n", value.count);
			if (next_key == 1111)
				printf("Range based TLB invalidate %llu bytes (%llu times)\n",
					value.size, value.count);
		}
		key = next_key;
	}
}

/* HW Engine class + instance */
#define RENDER_CLASS		0
#define VIDEO_DECODE_CLASS	1
#define VIDEO_ENHANCEMENT_CLASS	2
#define COPY_ENGINE_CLASS	3
#define OTHER_CLASS		4
#define COMPUTE_CLASS		5
#define MAX_ENGINE_CLASS	5
#define MAX_ENGINE_INSTANCE	8

static const char *intel_engine_class_2name(__u32 class)
{
	static const char * const uabi_names[] = {
		[RENDER_CLASS] = "rcs",
		[COPY_ENGINE_CLASS] = "bcs",
		[VIDEO_DECODE_CLASS] = "vcs",
		[VIDEO_ENHANCEMENT_CLASS] = "vecs",
		[OTHER_CLASS] = "other",
		[COMPUTE_CLASS] = "ccs",
	};

#define ARRAY_SIZE(arr) sizeof(arr) /sizeof((arr[0]))
	if (class >= ARRAY_SIZE(uabi_names) || !uabi_names[class])
		return "xxx";

	return uabi_names[class];
}

static void print_access_counter(void)
{
	struct combined_key key = {0, 0}, next_key;
	__u32 class, instance;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "access_counter_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			class = next_key.key1;
			instance = next_key.key2;
			printf("%s[%d] triggered %llu access counter\n",
				intel_engine_class_2name(class), instance,
				value);
		}
		key = next_key;
	}
}
static void print_vm_prefetch_counter(void)
{
	__u64 key = 0, next_key;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "vm_prefetch_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			printf("prefetched %llu bytes to %s\n",
				value, (next_key == 11) ? "smem":"lmem");
		}
		key = next_key;
	}
}

static void print_gpu_fault(void)
{
	__u64 key = 0, next_key;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "gpu_fault_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			printf("GPU %s memory fault count %llu\n",
				(next_key == 0x10) ? "read":"write", value);
		}
		key = next_key;
	}
}

static void print_context_create_counter(void)
{
	__u64 key = 1234;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "ctx_create_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return;

	printf("%llu context created\n", value);
}

static void print_cpu_object_fault(void)
{
	__u64 key = 0;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "cpu_object_fault_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return;

	printf("cpu object fault count: %llu\n", value);
}

static void print_object_create_counter(void)
{
	__u64 key = 0;
	struct counter_rec value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "object_create_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return;

	printf("%llu object created, total size %llu\n", value.count, value.size);
}

static void print_counters(void)
{
	print_context_create_counter();
	print_object_create_counter();
	print_vma_bind_counter();
	print_vm_prefetch_counter();
	print_cpu_object_fault();
	print_gpu_fault();
	print_tlb_invalidate_counter();
	print_access_counter();
	print_object_migrate_counter();
	print_gem_shrink();
}

static void clear_vma_bind_counter(void)
{
	__u64 key = 1234;
	struct counter_rec zero = {0, 0};
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "vm_bind_count_map");
	bpf_map_update_elem(map_fd, &key, &zero, BPF_ANY);
}

static void clear_gem_shrink_counter(void)
{
	__u64 key = 1234;
	struct counter_rec zero = {0, 0};
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "gem_shrink_map");
	bpf_map_update_elem(map_fd, &key, &zero, BPF_ANY);
}

static void clear_object_migrate_counter(void)
{
	struct combined_key key = {0, 0}, next_key;
	struct counter_rec value, zero = {0, 0};
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "object_migrate_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			bpf_map_update_elem(map_fd, &next_key, &zero, BPF_ANY);
		}
		key = next_key;
	}
}

static void clear_tlb_invalidate_counter(void)
{
	__u64 key = 0, next_key;
	struct counter_rec value, zero = {0, 0};
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "tlb_invalidate_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			bpf_map_update_elem(map_fd, &next_key, &zero, BPF_ANY);
		}
		key = next_key;
	}
}

static void clear_vm_prefetch_counter(void)
{
	__u64 key = 0, next_key;
	__u64 value, zero = 0;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "vm_prefetch_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			bpf_map_update_elem(map_fd, &next_key, &zero, BPF_ANY);
		}
		key = next_key;
	}
}

static void clear_access_counter(void)
{
	struct combined_key key = {0, 0}, next_key;
	__u64 value, zero = 0;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "access_counter_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			bpf_map_update_elem(map_fd, &next_key, &zero, BPF_ANY);
		}
		key = next_key;
	}
}

static void clear_gpu_fault(void)
{
	__u64 key = 0, next_key;
	__u64 value, zero = 0;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "gpu_fault_map");
	while (bpf_map_get_next_key(map_fd, &key, &next_key) == 0) {
		if (bpf_map_lookup_elem(map_fd, &next_key, &value) == 0) {
			bpf_map_update_elem(map_fd, &next_key, &zero, BPF_ANY);
		}
		key = next_key;
	}
}

static void clear_context_create_counter(void)
{
	__u64 key = 1234;
	__u64 value = 0;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "ctx_create_map");
	bpf_map_update_elem(map_fd, &key, &value, BPF_ANY);
}

static void clear_cpu_object_fault(void)
{
	__u64 key = 0;
	__u64 value = 0;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "cpu_object_fault_map");
	bpf_map_update_elem(map_fd, &key, &value, BPF_ANY);
}

static void clear_object_create_counter(void)
{
	__u64 key = 0;
	struct counter_rec value = {0, 0};
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "object_create_map");
	bpf_map_update_elem(map_fd, &key, &value, BPF_ANY);
}

static void clear_counters(void)
{
	clear_vma_bind_counter();
	clear_object_migrate_counter();
	clear_context_create_counter();
	clear_object_create_counter();
	clear_cpu_object_fault();
	clear_gem_shrink_counter();
	clear_gpu_fault();
	clear_vm_prefetch_counter();
	clear_access_counter();
	clear_tlb_invalidate_counter();
}

static void print_help(void)
{
	printf("A very basic i915 driver performance counters: \n");
	printf("	- press h for help\n");
	printf("	- press c to clear counters\n");
	printf("	- press p to print counters\n");
	printf("	- press q to quit\n");
}

int main(int ac, char **argv)
{
	struct rlimit r = {RLIM_INFINITY, RLIM_INFINITY};
	struct bpf_link *links[256];
	struct bpf_program *prog;
	char file[256];
	char input;
	int i = 0;

	setrlimit(RLIMIT_MEMLOCK, &r);

	snprintf(file, sizeof(file), "%s_kern.o", argv[0]);
	obj = bpf_object__open_file(file, NULL);
	if (libbpf_get_error(obj)) {
		fprintf(stderr, "ERROR: opening BPF object file %s failed\n", file);
		return 0;
	}

	if (bpf_object__load(obj)) {
		fprintf(stderr, "ERROR: loading BPF object file failed\n");
		goto cleanup;
	}

	bpf_object__for_each_program(prog, obj) {
		links[i] = bpf_program__attach(prog);
		if (libbpf_get_error(links[i])) {
			fprintf(stderr, "ERROR: bpf_program__attach failed\n");
			links[i] = NULL;
			goto cleanup;
		}
		i++;
	}

	while (scanf("%c", &input)) {
		if (input == 'c')
			clear_counters();
		if (input == 'p')
			print_counters();
		if (input == 'h')
			print_help();
		if (input == 'q')
			break;
	}
cleanup:
	for (i--; i >= 0; i--)
		bpf_link__destroy(links[i]);
	bpf_object__close(obj);
	return 0;
}
