/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#ifndef SELFTESTS_SELFTEST_H_INCLUDED
#define SELFTESTS_SELFTEST_H_INCLUDED

/**
 * selftests_run - Execute selftests.
 *
 * Return: 0 if the driver should continue; non-zero otherwise.
 */
int selftests_run(void);

/*
 * 'make checkpatch' will complain about the return statment in the following
 * macro:
 */
#define FAIL(msg, ...) \
	do { \
		pr_err("TEST FAILED: %s: assert: " msg "\n", __func__, ##__VA_ARGS__); \
		return -EINVAL; \
	} while (0)

#define TEST(cond, msg, ...) \
	do { \
		if (!(cond)) { \
			FAIL(msg, ##__VA_ARGS__); \
		} \
	} while (0)

#endif /* SELFTESTS_SELFTEST_H_INCLUDED */
