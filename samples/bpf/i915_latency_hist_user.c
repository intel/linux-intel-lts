/* Copyright (c) 2022 Intel Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

#include <bpf/bpf.h>
#include <bpf/libbpf.h>

static struct bpf_object *obj;

struct latency_record {
	__u64 total;
	__u64 count;
	__u64 min;
	__u64 max;
};

static __u64 get_ct_request_worker_time(void)
{
	__u64 key = 1234;
	struct latency_record value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "ct_request_worker_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	printf("Max page fault worker time %-8llu ns\n", value.max);
	printf("Min page fault worker time  %-8llu ns\n", value.min);
	printf("average page fault worker time  %-8llu ns\n", value.total/value.count);
	return value.total;
}

static __u64 get_ct_tasklet_time(void)
{
	__u64 key = 1234;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "ct_tasklet_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	return value;
}

static __u64 get_guc_irq_time(void)
{
	__u64 key = 1234;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "guc_irq_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	return value;
}

static __u64 get_object_migration_time(void)
{
	__u64 key = 1234;
	struct latency_record value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "object_migration_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	printf("Max migration time of single object %-8llu ns\n", value.max);
	printf("Min migration time of single object %-8llu ns\n", value.min);
	printf("average migration time of single object %-8llu ns\n", value.total/value.count);

	return value.total;
}

static __u64 get_object_create_time(void)
{
	__u64 key = 1234;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "gem_create_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	return value;
}

static __u64 get_vma_bind_time(void)
{
	__u64 key = 1234;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "vma_bind_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	return value;
}

static __u64 get_user_fence_time(void)
{
	__u64 key = 1234;
	__u64 value;
	int map_fd;

	map_fd = bpf_object__find_map_fd_by_name(obj, "user_fence_time_map");
	if (bpf_map_lookup_elem(map_fd, &key, &value) != 0)
		return 0;

	return value;
}


static void stars(char *str, __u64 val, __u64 max, int width)
{
	int i;

	for (i = 0; i < (width * val / max); i++)
		str[i] = '*';
	if (val > max)
		str[i - 1] = '+';
	str[i] = '\0';
}


static void print_hist(void)
{
#define MAX_STARS 100
	struct {
		__u64 value;
		char name[128];
	} data[16];

	__u64 max_value;
	char starstr[128];
	int i;

	data[0].value = get_object_migration_time();
	data[1].value = get_object_create_time();
	data[2].value = get_vma_bind_time();
	data[3].value = get_user_fence_time();
	data[4].value = get_guc_irq_time();
	data[5].value = get_ct_tasklet_time();
	data[6].value = get_ct_request_worker_time();
	data[7].value = data[4].value + data[5].value + data[6].value;
	strcpy (data[0].name, "object migration");
	strcpy (data[1].name, "object creation ");
	strcpy (data[2].name, "vma bind        ");
	strcpy (data[3].name, "wait user fence ");
	strcpy (data[4].name, "guc irq         ");
	strcpy (data[5].name, "ct tasklet      ");
	strcpy (data[6].name, "ct request work ");
	strcpy (data[7].name, "kmd int handler ");

	max_value = data[0].value;
	for (i = 1; i < 8; i++) {
		if (data[i].value > max_value)
			max_value = data[i].value;
	}

	printf("i915 latency\n");
	printf("     what 	 : latency		distribution\n");
	if (max_value == 0)
		return;

	for (i = 0; i < 8; i++) {
		stars(starstr, data[i].value, max_value, MAX_STARS);
		printf("%-s : %-8lluns		|%-s|\n",
		       data[i].name, data[i].value, starstr);
	}
}

static void print_help(void)
{
	printf("A very basic i915 latency hist: \n");
	printf("	- press h for help\n");
	printf("	- press p to print hist\n");
	printf("	- press q to quit\n");
}

int main(int ac, char **argv)
{
	struct rlimit r = {RLIM_INFINITY, RLIM_INFINITY};
	struct bpf_link *links[2];
	struct bpf_program *prog;
	char filename[256], input;
	int map_fd, j = 0;

	if (setrlimit(RLIMIT_MEMLOCK, &r)) {
		perror("setrlimit(RLIMIT_MEMLOCK, RLIM_INFINITY)");
		return 1;
	}

	snprintf(filename, sizeof(filename), "%s_kern.o", argv[0]);
	obj = bpf_object__open_file(filename, NULL);
	if (libbpf_get_error(obj)) {
		fprintf(stderr, "ERROR: opening BPF object file failed\n");
		return 0;
	}

	/* load BPF program */
	if (bpf_object__load(obj)) {
		fprintf(stderr, "ERROR: loading BPF object file failed\n");
		goto cleanup;
	}

	map_fd = bpf_object__find_map_fd_by_name(obj, "object_migration_time_map");
	if (map_fd < 0) {
		fprintf(stderr, "ERROR: finding a map in obj file failed\n");
		goto cleanup;
	}

	bpf_object__for_each_program(prog, obj) {
		links[j] = bpf_program__attach(prog);
		if (libbpf_get_error(links[j])) {
			fprintf(stderr, "ERROR: bpf_program__attach failed\n");
			links[j] = NULL;
			goto cleanup;
		}
		j++;
	}

	while (scanf("%c", &input)) {
		if (input == 'p')
			print_hist();
		if (input == 'h')
			print_help();
		if (input == 'q')
			break;
	}
cleanup:
	for (j--; j >= 0; j--)
		bpf_link__destroy(links[j]);

	bpf_object__close(obj);
	return 0;
}
