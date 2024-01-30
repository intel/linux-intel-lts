Using eBPF for gfx performance profiling
========================================

BPF has bee widely used in performance profiling and tracing in recent years.
See a eBPF introduction on this page: https://www.brendangregg.com/ebpf.html.
This tutorial explored how to use eBPF for gfx performance profiling, with a
few sample programs. This tutorial documents some technical details of those
samples and how to build, run them.

Due to security concern, people might don't want to run profiling tools
under root. We especially explored how to run eBPF C program under none root.

Note: There are a few different ways building a BPF program. One simpler way
is to use the BCC (bpf compiler  collection, see https://github.com/iovisor/bcc)
which has some C wrappers for the bpf kernel program, and a python front end
for bpf user space program. You can install latest BCC compiler from this link:
https://github.com/iovisor/bcc/blob/master/INSTALL.md#ubuntu---source.

But in this tutorial we don't use BCC due to 2 reasons:
1. BCC doesn't seem to have a C frontend for bpf user space program. We want
to integrate bpf into a performance tool which uses a C/C++ front end.

2. BCC seems require root privilege to run. As an attempt, we tried to give
the bpf python program (.py) CAP_BPF and CAP_PERFMON, or give /usr/bin/python3
the same capabilities, bpf program failed to run with error indicating failed
to attach eBPF byte code to kernel event.

We end up with using plain C language for both the BPF kernel program and user
space program. This is the same way used by samples/bpf.

Prerequisite
============

1. The Linux kernel version should >= 5.8.
The way we run eBPF program under none root is to grant user process CAP_BPF
and CAP_PERFMON (there might be other capabilities required depend on what
the program does). Both CAP_BPF and CAP_PERFMON were introduced in kernel
5.8
If you want to try the samples on an older kernel, you need to backport
at least CAP_BPF and CAP_PERFMON to old kernel.

2. A typicl Linux kernel configuration is as below. If you don't need those
_NET* options, you can remove them

CONFIG_CGROUP_BPF=y
CONFIG_BPF=y
CONFIG_BPF_SYSCALL=y
CONFIG_ARCH_WANT_DEFAULT_BPF_JIT=y
CONFIG_BPF_JIT_ALWAYS_ON=y
CONFIG_BPF_JIT_DEFAULT_ON=y
CONFIG_IPV6_SEG6_BPF=y
CONFIG_NETFILTER_XT_MATCH_BPF=m
CONFIG_BPFILTER=y
CONFIG_BPFILTER_UMH=m
CONFIG_NET_CLS_BPF=m
CONFIG_NET_ACT_BPF=m
CONFIG_BPF_JIT=y
CONFIG_BPF_STREAM_PARSER=y
CONFIG_LWTUNNEL_BPF=y
CONFIG_HAVE_EBPF_JIT=y
CONFIG_BPF_EVENTS=y
CONFIG_BPF_KPROBE_OVERRIDE=y
CONFIG_TEST_BPF=m

The sample programs
===================

Two simple samples were added to samples/bpf folder:

1. A bpf hello world. This is a very simple bpf program that attach to the
sys_clone kprobe. When it is triggered (by __x64_sys_clone in our set up),
call bpf_trace_printk to write a "Hello World" to /sys/kernel/debug/tracing/trace_pipe

Please note the user space bpf program is very simple only because it calls
an existing function load_bpf_file to load, attach bpf program, and enable kprobe
event. load_bpf_file function calls into a few bpf and perf_event syscall, and
ioctls to load bpf program, enable perf event (in this case kprobe) and attach
bpf program to kerf event.

2. A bpf program to instrument i915 driver vma bind tracepoint. When the vma
bind tracepoint is hit, the eBPF kernel program accumulate how many bytes has
been bound per VM base. Those statistics information is saved to a hash type
bpf map which user space program can read and print out to the standard io,
or draw a graph etc, whatever you prefer.

To write your own eBPF program, you can refer to those samples and a lot of
other samples at this same folder.

Two concepts are important when you write your eBPF program:
 - program type: Program type is a parameter user passed when he calls
   the BPF_PROG_LOAD BPF syscall, specifying what the BPF program is going to
   do. Different BPF program type allows you to do different things in kernel:
   instrument to a kprobe event, network stack, or many others. The first sample
   above uses the BPF_PROG_TYPE_KPROBE. The second sample uses _TRACEPOINT
   program type. Note program type is set in the load_bpf_file function. See
   more details at https://blogs.oracle.com/linux/post/bpf-a-tour-of-program-types

 - map type: eBPF map is currently the only way to exchange information b/t eBPF
   kernel program and user space program. There are about 10 map types defined. See
   some program guide here: https://prototype-kernel.readthedocs.io/en/latest/bpf/ebpf_maps.html

Build the samples
=================

The new BPF sample programs were added to this same samples/bpf folder. It is very
easy to build them by following instructions in samples/bpf/README.rst.

But if you want to build your own program out of the kernel/samples/bpf tree, for example,
to use BPF for an existing program, you need to look into the Makefile in the same folder
to figure out compiler, header file and library dependencies, and write
your own Makefile. Below text gives some information on how to write your own Makefile.

The user space program is compiled just like a normal Linux program using gcc. It depends
on linux kernel header files(below path is relative to linux kernel source top):
-I./usr/include -I./tools/testing/selftests/bpf/ -I./tools/lib/ -I./tools/include -I./tools/perf

And it depends on linux kernel tools/lib/bpf/libbpf.a library.

The bpf_load.c file in the samples/bpf folder works as a private library. It simplifies
bpf user space program. You can make it a library.

The bpf kernel space program is built a little more  complicated. It requires 4 steps to build:

1. Using clang to compile the kernel program source into IR with native target:

clang -nostdinc -isystem /usr/lib/gcc/x86_64-linux-gnu/9/include -I/home/szeng/dii-tools/linux/./arch/x86/include -I/home/szeng/dii-tools/linux/./arch/x86/include/generated -I/home/szeng/dii-tools/linux/./include -I/home/szeng/dii-tools/linux/./arch/x86/include/uapi -I/home/szeng/dii-tools/linux/./arch/x86/include/generated/uapi -I/home/szeng/dii-tools/linux/./include/uapi -I/home/szeng/dii-tools/linux/./include/generated/uapi -I/home/szeng/dii-tools/linux/./include/linux/kconfig.h -fno-stack-protector -g -I/home/szeng/dii-tools/linux/samples/bpf -I/home/szeng/dii-tools/linux/./tools/testing/selftests/bpf/ -I/home/szeng/dii-tools/linux/./tools/lib/ -D__KERNEL__ -D__BPF_TRACING__ -Wno-unused-value -Wno-pointer-sign -D__TARGET_ARCH_x86 -Wno-compare-distinct-pointer-types -Wno-gnu-variable-sized-type-not-at-end -Wno-address-of-packed-member -Wno-tautological-compare -Wno-unknown-warning-option -I/home/szeng/dii-tools/linux/./samples/bpf/ -include asm_goto_workaround.h -O2 -emit-llvm -Xclang -disable-llvm-passes -c /home/szeng/dii-tools/linux/samples/bpf/tracepoint_kern.c -o ./tracepoint_kern.ir

You can see it depends on a lot of Linux kernel tree header files. So maybe the easiest
way to do it is, make a copy of the Linux kernel source tree and use -I to define the
kernel source code header files search path.

2. Using opt command to perform bpf IR builtin processing and optimization:

opt -O2 -mtriple=bpf-pc-linux ./tracepoint_kern.ir > tracepoint_kern.opt

3. Use llvm-dis to disassemble the 'opt' output into human readable assemble codes.

llvm-dis ./tracepoint_kern.opt

The output is saved in ./tracepoint_kern.opt.ll

4. Finally use LLC to compile the assembly code into bpf byte code binary.

llc -march=bpf -filetype=obj ./tracepoint_kern.opt.ll -o /home/szeng/dii-tools/linux/samples/bpf/tracepoint_kern.o

The tracepoint_kern.o is the bpf byte code to be loaded into Linux kernel for
execution. But be aware in this byte codes there are ELF file metadata which
can't be loaded to kernel. We can use llvm-objdump to know the eBPF program
start (skipping the meta data header) and size, for example:

llvm-objdump -arch-name=bpf -S ./tracepoint_kern.o

From the output, you can see the offset and size of the text section, then use
dd to extract it:

dd if=./tracepoint_kern.o of=tracepoint_kern bs=1 count=408 skip=64

tracepoint_kern is the extracted byte codes that can be loaded to Linux kernel.
If you use load_bpf_file function to load bpf byte code, then you don't need
to extract it manually - all the logics are built in load_bpf_file function.

Run the samples
===============

Run the bpf samples under root can be as simple as typing the sample names:

./tracepoint

But if you want to run bpf samples under non-root, there is extra work to do.
More specifically, Since bpf program runs inside Linux kernel, it requires
certain capabilities to run. Root user has the whole set of all capabilities,
we don't need to do extra work to run under root. You can type "man capabilities"
on a Linux machine to get more details of Linux capability concept, or here is
a up to date version: https://man7.org/linux/man-pages/man7/capabilities.7.html

The main idea to run BPF program under non-privileged user is to give the BPF
user program specific capabilities and when program is loaded the process will
inherit certain capabilities depending on program capabilities and other settings.
Depending on the functionality of your bpf program, specific capabilities need
to be granted to your ebp program. For example, if your bpf program instruments
raw socket network stack, you need to grant "cap_net_raw" capability. If your
program instrument the kernel kprobes or tracepoints, it requires cap_perfmon
capability. cap_bpf is a general capability that is required for all eBPF program.
Below are steps to run bpf under none privileged user.

1. install libcap

We use libcap to grant bpf program capabilities. libcap in most Linux distribution
is out of date. Since we require cap_bpf and cap_perfmon which are introduced not
long time ago, we need install latest libcap for that support.

git clone git://git.kernel.org/pub/scm/libs/libcap/libcap.git

Then follow the README file to build, install.

2. grant bpf user program cap_bpf and cap_perfmon

sudo setcap cap_bpf,cap_perfmon=+iep ./tracepoint

When succeed, you program will have those capabilities:
getcap  ./tracepoint
./tracepoint cap_perfmon,cap_bpf=eip

3. Remount debugfs and tracefs

The sample program accesses tracefs to get the event id (in the tracepoint sample,
it is /sys/kernel/debug/tracing/events/i915/i915_vma_bind/id) so it can instrument
this specific event.

The sample program also access to /sys/kernel/debug/tracing/kprobe_events to
enable the events that user want to instrument.

By default, debugfs and tracefs are all mounted only for root to access. To run
bpf program under non-privileged user, debugfs and tracefs need to be remounted
so user can access it:

sudo mount -o remount,mode=755 /sys/kernel/tracing/
sudo mount -o remount,mode=755 /sys/kernel/debug

4. Finally run the samples w/o root privilege:

./tracepoint
