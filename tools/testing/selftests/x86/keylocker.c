// SPDX-License-Identifier: GPL-2.0-only
/*
 * keylocker.c, validating CPU-internal key management
 */
#undef _GNU_SOURCE
#define _GNU_SOURCE 1

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <err.h>
#include <sched.h>
#include <setjmp.h>
#include <signal.h>
#include <unistd.h>

#define HANDLE_SIZE	48

static bool keylocker_disabled;

/* Encode a 128-bit key to a 384-bit handle */
static inline void __encode_key(char *handle)
{
	static const unsigned char aeskey[] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
						0x71, 0x77, 0x74, 0x69, 0x6f, 0x6b, 0x6c, 0x78 };

	asm volatile ("movdqu %0, %%xmm0" : : "m" (*aeskey) :);

	/* Set no restriction to the handle */
	asm volatile ("mov $0, %%eax" :);

	/* ENCODEKEY128 %EAX */
	asm volatile (".byte 0xf3, 0xf, 0x38, 0xfa, 0xc0");

	asm volatile ("movdqu %%xmm0, %0; movdqu %%xmm1, %1; movdqu %%xmm2, %2;"
		      : "=m" (handle[0]), "=m" (handle[0x10]), "=m" (handle[0x20]));
}

static jmp_buf jmpbuf;

static void handle_sigill(int sig, siginfo_t *si, void *ctx_void)
{
	keylocker_disabled = true;
	siglongjmp(jmpbuf, 1);
}

static bool encode_key(char *handle)
{
	struct sigaction sa;
	bool success = true;
	int ret;

	memset(&sa, 0, sizeof(sa));

	/* Set signal handler */
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = handle_sigill;
	sigemptyset(&sa.sa_mask);
	ret = sigaction(SIGILL, &sa, 0);
	if (ret)
		err(1, "sigaction");

	if (sigsetjmp(jmpbuf, 1))
		success = false;
	else
		__encode_key(handle);

	/* Clear signal handler */
	sa.sa_flags = 0;
	sa.sa_sigaction = NULL;
	sa.sa_handler = SIG_DFL;
	sigemptyset(&sa.sa_mask);
	ret = sigaction(SIGILL, &sa, 0);
	if (ret)
		err(1, "sigaction");

	return success;
}

/*
 * Test if the internal key is the same in all the CPUs:
 *
 * Since the value is not readable, compare the encoded output of a AES key
 * between CPUs.
 */

static int nerrs;

static unsigned char cpu0_handle[HANDLE_SIZE] = { 0 };

static void test_internal_key(bool slept, long cpus)
{
	int cpu, errs;

	printf("Test the internal key consistency between CPUs\n");

	for (cpu = 0, errs = 0; cpu < cpus; cpu++) {
		char handle[HANDLE_SIZE] = { 0 };
		cpu_set_t mask;
		bool success;

		CPU_ZERO(&mask);
		CPU_SET(cpu, &mask);
		sched_setaffinity(0, sizeof(cpu_set_t), &mask);

		success = encode_key(handle);
		if (!success) {
			/* Take S3 sleep only if Key Locker is enabled. */
			if (slept)
				errs++;
			printf("[%s]\tKey Locker disabled at CPU%d\n",
			       slept ? "FAIL" : "NOTE", cpu);
			continue;
		}

		if (cpu == 0 && !slept) {
			/* Record the first handle value as reference */
			memcpy(cpu0_handle, handle, HANDLE_SIZE);
		} else if (memcmp(cpu0_handle, handle, HANDLE_SIZE)) {
			/* Take any mismatch as an error */
			printf("[FAIL]\tMismatched internal key at CPU%d\n",
			       cpu);
			errs++;
		}
	}

	if (errs == 0 && !keylocker_disabled)
		printf("[OK]\tAll the internal keys are the same\n");
	else
		nerrs += errs;
}

static void switch_to_sleep(bool *slept)
{
	ssize_t bytes;
	int fd;

	printf("Transition to Suspend-To-RAM state\n");

	fd = open("/sys/power/mem_sleep", O_RDWR);
	if (fd < 0)
		err(1, "Open /sys/power/mem_sleep");

	bytes = write(fd, "deep", strlen("deep"));
	if (bytes != strlen("deep"))
		err(1, "Write /sys/power/mem_sleep");
	close(fd);

	fd = open("/sys/power/state", O_RDWR);
	if (fd < 0)
		err(1, "Open /sys/power/state");

	bytes = write(fd, "mem", strlen("mem"));
	if (bytes != strlen("mem"))
		err(1, "Write /sys/power/state");
	close(fd);

	printf("Wake up from Suspend-To-RAM state\n");
	*slept = true;
}

int main(void)
{
	bool slept = false;
	long cpus;

	cpus = sysconf(_SC_NPROCESSORS_ONLN);
	printf("%ld CPUs in the system\n", cpus);

	test_internal_key(slept, cpus);
	if (keylocker_disabled)
		return nerrs ? 1 : 0;

	switch_to_sleep(&slept);
	test_internal_key(slept, cpus);
	return nerrs ? 1 : 0;
}
