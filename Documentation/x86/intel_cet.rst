.. SPDX-License-Identifier: GPL-2.0

=========================================
Control-flow Enforcement Technology (CET)
=========================================

[1] Overview
============

Control-flow Enforcement Technology (CET) is an Intel processor feature
that provides protection against return/jump-oriented programming (ROP)
attacks.  It can be set up to protect both applications and the kernel.
Only user-mode protection is implemented in the 64-bit kernel, including
shadow stack support for running legacy 32-bit applications.  IBT is not
supported for 32-bit applications.

CET introduces Shadow Stack and Indirect Branch Tracking.  Shadow stack is
a secondary stack allocated from memory and cannot be directly modified by
applications.  When executing a CALL instruction, the processor pushes the
return address to both the normal stack and the shadow stack.  Upon
function return, the processor pops the shadow stack copy and compares it
to the normal stack copy.  If the two differ, the processor raises a
control-protection fault.  Indirect branch tracking verifies indirect
CALL/JMP targets are intended as marked by the compiler with 'ENDBR'
opcodes.

There are two Kconfig options:

    X86_SHADOW_STACK, and X86_IBT.

To build a CET-enabled kernel, Binutils v2.31 and GCC v8.1 or LLVM v10.0.1
or later are required.  To build a CET-enabled application, GLIBC v2.28 or
later is also required.

There are two command-line options for disabling CET features::

    no_user_shstk - disables user shadow stack, and
    no_user_ibt   - disables user indirect branch tracking.

At run time, /proc/cpuinfo shows CET features if the processor supports
CET.

[2] Application Enabling
========================

An application's CET capability is marked in its ELF header and can be
verified from readelf/llvm-readelf output:

    readelf -n <application> | grep -a SHSTK
        properties: x86 feature: IBT, SHSTK

If an application supports CET and is statically linked, it will run with
CET protection.  If the application needs any shared libraries, the loader
checks all dependencies and enables CET when all requirements are met.

[3] Backward Compatibility
==========================

GLIBC provides a few CET tunables via the GLIBC_TUNABLES environment
variable:

GLIBC_TUNABLES=glibc.tune.hwcaps=-SHSTK,-IBT
    Turn off SHSTK/IBT.

GLIBC_TUNABLES=glibc.tune.x86_shstk=<on, permissive>
    This controls how dlopen() handles SHSTK legacy libraries::

        on         - continue with SHSTK enabled;
        permissive - continue with SHSTK off.

Details can be found in the GLIBC manual pages.

[4] CET arch_prctl()'s
======================

Several arch_prctl()'s have been added for CET:

arch_prctl(ARCH_X86_CET_STATUS, u64 *addr)
    Return CET feature status.

    The parameter 'addr' is a pointer to a user buffer.
    On returning to the caller, the kernel fills the following
    information::

        *addr       = shadow stack/indirect branch tracking status
        *(addr + 1) = shadow stack base address
        *(addr + 2) = shadow stack size

arch_prctl(ARCH_X86_CET_DISABLE, unsigned int features)
    Disable shadow stack and/or indirect branch tracking as specified in
    'features'.  Return -EPERM if CET is locked.

arch_prctl(ARCH_X86_CET_LOCK)
    Lock in all CET features.  They cannot be turned off afterwards.

Note:
  There is no CET-enabling arch_prctl function.  By design, CET is enabled
  automatically if the binary and the system can support it.

[5] The implementation of the Shadow Stack
==========================================

Shadow Stack size
-----------------

A task's shadow stack is allocated from memory to a fixed size of
MIN(RLIMIT_STACK, 4 GB).  In other words, the shadow stack is allocated to
the maximum size of the normal stack, but capped to 4 GB.  However,
a compat-mode application's address space is smaller, each of its thread's
shadow stack size is MIN(1/4 RLIMIT_STACK, 4 GB).

Signal
------

The main program and its signal handlers use the same shadow stack.
Because the shadow stack stores only return addresses, a large shadow
stack covers the condition that both the program stack and the signal
alternate stack run out.

The kernel creates a restore token for the shadow stack restoring address
and verifies that token when restoring from the signal handler.

Fork
----

The shadow stack's vma has VM_SHADOW_STACK flag set; its PTEs are required
to be read-only and dirty.  When a shadow stack PTE is not RO and dirty, a
shadow access triggers a page fault with the shadow stack access bit set
in the page fault error code.

When a task forks a child, its shadow stack PTEs are copied and both the
parent's and the child's shadow stack PTEs are cleared of the dirty bit.
Upon the next shadow stack access, the resulting shadow stack page fault
is handled by page copy/re-use.

When a pthread child is created, the kernel allocates a new shadow stack
for the new thread.
