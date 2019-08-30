/*
 * virtio and hyperviosr service module (VHM): vm management
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2017 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Copyright (C) 2017 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Liang Ding <liang.ding@intel.com>
 * Jason Zeng <jason.zeng@intel.com>
 *
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <asm/processor.h>
#include <linux/vhm/acrn_hv_defs.h>
#include <linux/vhm/vhm_ioctl_defs.h>
#include <linux/vhm/acrn_vhm_ioreq.h>
#include <linux/vhm/acrn_vhm_mm.h>
#include <linux/vhm/vhm_hypercall.h>

LIST_HEAD(vhm_vm_list);
DEFINE_RWLOCK(vhm_vm_list_lock);

struct vhm_vm *find_get_vm(unsigned long vmid)
{
	struct vhm_vm *vm;

	read_lock_bh(&vhm_vm_list_lock);
	list_for_each_entry(vm, &vhm_vm_list, list) {
		if (vm->vmid == vmid) {
			refcount_inc(&vm->refcnt);
			read_unlock_bh(&vhm_vm_list_lock);
			return vm;
		}
	}
	read_unlock_bh(&vhm_vm_list_lock);
	return NULL;
}
EXPORT_SYMBOL_GPL(find_get_vm);

void put_vm(struct vhm_vm *vm)
{
	if (refcount_dec_and_test(&vm->refcnt)) {
		free_guest_mem(vm);

		if (vm->req_buf && vm->pg) {
			put_page(vm->pg);
			vm->pg = NULL;
			vm->req_buf = NULL;
		}

		kfree(vm);
		pr_info("vhm: freed vm\n");
	}
}
EXPORT_SYMBOL_GPL(put_vm);

void get_vm(struct vhm_vm *vm)
{
	refcount_inc(&vm->refcnt);
}
EXPORT_SYMBOL_GPL(get_vm);

int vhm_get_vm_info(unsigned long vmid, struct vm_info *info)
{
	struct vhm_vm *vm;

	vm = find_get_vm(vmid);
	if (unlikely(vm == NULL)) {
		pr_err("vhm: failed to find vm from vmid %ld\n",
			vmid);
		return -EINVAL;
	}
	/*TODO: hardcode max_vcpu here, should be fixed by getting at runtime */
	info->max_vcpu = 4;
	info->max_gfn = vm->max_gfn;
	put_vm(vm);
	return 0;
}
EXPORT_SYMBOL_GPL(vhm_get_vm_info);

int vhm_inject_msi(unsigned long vmid, unsigned long msi_addr,
		unsigned long msi_data)
{
	struct acrn_msi_entry *msi;
	int ret;

	/* vhm_inject_msi is called in vhm_irqfd_inject from eventfd_signal
	 * and the interrupt is disabled.
	 * So the GFP_ATOMIC should be used instead of GFP_KERNEL to
	 * avoid the sleeping with interrupt disabled.
	 */
	msi = acrn_mempool_alloc(GFP_ATOMIC);
	/* msi_addr: addr[19:12] with dest vcpu id */
	/* msi_data: data[7:0] with vector */
	msi->msi_addr = msi_addr;
	msi->msi_data = msi_data;
	ret = hcall_inject_msi(vmid, virt_to_phys(msi));
	acrn_mempool_free(msi);
	if (ret < 0) {
		pr_err("vhm: failed to inject!\n");
		return -EFAULT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(vhm_inject_msi);

unsigned long vhm_vm_gpa2hpa(unsigned long vmid, unsigned long gpa)
{
	struct vm_gpa2hpa *gpa2hpa;
	int ret;
	unsigned long hpa;

	gpa2hpa = acrn_mempool_alloc(GFP_KERNEL);
	gpa2hpa->gpa = gpa;
	gpa2hpa->hpa = -1UL; /* Init value as invalid gpa */
	ret = hcall_vm_gpa2hpa(vmid, virt_to_phys(gpa2hpa));
	if (ret < 0) {
		pr_err("vhm: failed to inject!\n");
		acrn_mempool_free(gpa2hpa);
		return -EFAULT;
	}
	hpa = gpa2hpa->hpa;
	mb();
	acrn_mempool_free(gpa2hpa);
	return hpa;
}
EXPORT_SYMBOL_GPL(vhm_vm_gpa2hpa);

void vm_list_add(struct list_head *list)
{
	list_add(list, &vhm_vm_list);
}
