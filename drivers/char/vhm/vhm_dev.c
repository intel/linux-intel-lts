/*
 * virtio and hyperviosr service module (VHM): main framework
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
 * Xiao Zheng <xiao.zheng@intel.com>
 * Jason Chen CJ <jason.cj.chen@intel.com>
 * Jack Ren <jack.ren@intel.com>
 * Mingqiang Chi <mingqiang.chi@intel.com>
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/pci.h>

#include <linux/vhm/acrn_hv_defs.h>
#include <linux/vhm/vhm_ioctl_defs.h>
#include <linux/vhm/acrn_vhm_ioreq.h>
#include <linux/vhm/acrn_vhm_mm.h>
#include <linux/vhm/vhm_vm_mngt.h>
#include <linux/vhm/vhm_hypercall.h>
#include <linux/vhm/vhm_eventfd.h>

#include <asm/hypervisor.h>
#include <asm/acrnhyper.h>

#define  DEVICE_NAME "acrn_vhm"
#define  CLASS_NAME  "vhm"

#define VHM_API_VERSION_MAJOR	1
#define VHM_API_VERSION_MINOR	0

static int    major;
static struct class *vhm_class;
static struct device *vhm_device;
static struct tasklet_struct vhm_io_req_tasklet;

static int vhm_dev_open(struct inode *inodep, struct file *filep)
{
	struct vhm_vm *vm;
	int i;

	vm = kzalloc(sizeof(struct vhm_vm), GFP_KERNEL);
	pr_info("vhm_dev_open: opening device node\n");

	if (!vm)
		return -ENOMEM;
	vm->vmid = ACRN_INVALID_VMID;
	vm->dev = vhm_device;

	for (i = 0; i < HUGEPAGE_HLIST_ARRAY_SIZE; i++)
		INIT_HLIST_HEAD(&vm->hugepage_hlist[i]);
	mutex_init(&vm->hugepage_lock);

	INIT_LIST_HEAD(&vm->ioreq_client_list);
	spin_lock_init(&vm->ioreq_client_lock);

	atomic_set(&vm->refcnt, 1);
	write_lock_bh(&vhm_vm_list_lock);
	vm_list_add(&vm->list);
	write_unlock_bh(&vhm_vm_list_lock);
	filep->private_data = vm;
	return 0;
}

static ssize_t vhm_dev_read(struct file *filep, char *buffer, size_t len,
		loff_t *offset)
{
	/* Does Nothing */
	pr_info("vhm_dev_read: reading device node\n");
	return 0;
}

static ssize_t vhm_dev_write(struct file *filep, const char *buffer,
		size_t len, loff_t *offset)
{
	/* Does Nothing */
	pr_info("vhm_dev_read: writing device node\n");
	return 0;
}

static long vhm_dev_ioctl(struct file *filep,
		unsigned int ioctl_num, unsigned long ioctl_param)
{
	long ret = 0;
	struct vhm_vm *vm;
	struct ic_ptdev_irq ic_pt_irq;

	pr_debug("[%s] ioctl_num=0x%x\n", __func__, ioctl_num);

	if (ioctl_num == IC_GET_API_VERSION) {
		struct api_version api_version;

		api_version.major_version = VHM_API_VERSION_MAJOR;
		api_version.minor_version = VHM_API_VERSION_MINOR;

		if (copy_to_user((void *)ioctl_param, &api_version,
			sizeof(struct api_version)))
			return -EFAULT;

		return 0;
	}

	memset(&ic_pt_irq, 0, sizeof(ic_pt_irq));
	vm = (struct vhm_vm *)filep->private_data;
	if (vm == NULL) {
		pr_err("vhm: invalid VM !\n");
		return -EFAULT;
	}
	if (((vm->vmid == ACRN_INVALID_VMID) && (ioctl_num != IC_CREATE_VM)) ||
			test_bit(VHM_VM_DESTROYED, &vm->flags)) {
		pr_err("vhm: invalid VM ID !\n");
		return -EFAULT;
	}

	switch (ioctl_num) {
	case IC_CREATE_VM: {
		struct acrn_create_vm *created_vm;

		created_vm = acrn_mempool_alloc(GFP_KERNEL);

		if (copy_from_user(created_vm, (void *)ioctl_param,
			sizeof(struct acrn_create_vm))) {
			acrn_mempool_free(created_vm);
			return -EFAULT;
		}

		ret = hcall_create_vm(virt_to_phys(created_vm));
		if ((ret < 0) ||
			(created_vm->vmid == ACRN_INVALID_VMID)) {
			pr_err("vhm: failed to create VM from Hypervisor !\n");
			acrn_mempool_free(created_vm);
			return -EFAULT;
		}

		if (copy_to_user((void *)ioctl_param, created_vm,
			sizeof(struct acrn_create_vm))) {
			ret = -EFAULT;
			goto create_vm_fail;
		}
		vm->vmid = created_vm->vmid;

		if (created_vm->req_buf) {
			ret = acrn_ioreq_init(vm, created_vm->req_buf);
			if (ret < 0)
				goto create_vm_fail;
		}

		acrn_ioeventfd_init(vm->vmid);
		acrn_irqfd_init(vm->vmid);
		acrn_mempool_free(created_vm);

		pr_info("vhm: VM %ld created\n", vm->vmid);
		break;

create_vm_fail:
		hcall_destroy_vm(created_vm->vmid);
		acrn_mempool_free(created_vm);
		vm->vmid = ACRN_INVALID_VMID;
		break;

	}

	case IC_START_VM: {
		ret = hcall_start_vm(vm->vmid);
		if (ret < 0) {
			pr_err("vhm: failed to start VM %ld!\n", vm->vmid);
			return -EFAULT;
		}
		break;
	}

	case IC_PAUSE_VM: {
		ret = hcall_pause_vm(vm->vmid);
		if (ret < 0) {
			pr_err("vhm: failed to pause VM %ld!\n", vm->vmid);
			return -EFAULT;
		}
		break;
	}

	case IC_RESET_VM: {
		ret = hcall_reset_vm(vm->vmid);
		if (ret < 0) {
			pr_err("vhm: failed to restart VM %ld!\n", vm->vmid);
			return -EFAULT;
		}
		break;
	}

	case IC_DESTROY_VM: {
		ret = vhm_vm_destroy(vm);
		break;
	}

	case IC_CREATE_VCPU: {
		struct acrn_create_vcpu *cv;

		cv = acrn_mempool_alloc(GFP_KERNEL);
		if (copy_from_user(cv, (void *)ioctl_param,
				sizeof(struct acrn_create_vcpu))) {
			acrn_mempool_free(cv);
			return -EFAULT;
		}

		ret = acrn_hypercall2(HC_CREATE_VCPU, vm->vmid,
				virt_to_phys(cv));
		if (ret < 0) {
			pr_err("vhm: failed to create vcpu %d!\n", cv->vcpu_id);
			acrn_mempool_free(cv);
			return -EFAULT;
		}
		atomic_inc(&vm->vcpu_num);
		acrn_mempool_free(cv);

		return ret;
	}

	case IC_SET_VCPU_REGS: {
		struct acrn_set_vcpu_regs *asvr;

		asvr = acrn_mempool_alloc(GFP_KERNEL);
		if (copy_from_user(asvr, (void *)ioctl_param, sizeof(*asvr))) {
			acrn_mempool_free(asvr);
			return -EFAULT;
		}

		ret = acrn_hypercall2(HC_SET_VCPU_REGS, vm->vmid,
				virt_to_phys(asvr));
		acrn_mempool_free(asvr);
		if (ret < 0) {
			pr_err("vhm: failed to set bsp state of vm %ld!\n",
					vm->vmid);
			return -EFAULT;
		}

		return ret;
	}

	case IC_SET_MEMSEG: {
		struct vm_memmap memmap;

		if (copy_from_user(&memmap, (void *)ioctl_param,
			sizeof(struct vm_memmap)))
			return -EFAULT;

		ret = map_guest_memseg(vm, &memmap);
		break;
	}

	case IC_UNSET_MEMSEG: {
		struct vm_memmap memmap;

		if (copy_from_user(&memmap, (void *)ioctl_param,
			sizeof(struct vm_memmap)))
			return -EFAULT;

		ret = unmap_guest_memseg(vm, &memmap);
		break;
	}

	case IC_SET_IOREQ_BUFFER: {
		/* init ioreq buffer */
		ret = acrn_ioreq_init(vm, (unsigned long)ioctl_param);
		if (ret < 0 && ret != -EEXIST)
			return ret;
		ret = 0;
		break;
	}

	case IC_CREATE_IOREQ_CLIENT: {
		int client_id;

		client_id = acrn_ioreq_create_fallback_client(vm->vmid, "acrndm");
		if (client_id < 0)
			return -EFAULT;
		return client_id;
	}

	case IC_DESTROY_IOREQ_CLIENT: {
		int client = ioctl_param;

		acrn_ioreq_destroy_client(client);
		break;
	}

	case IC_ATTACH_IOREQ_CLIENT: {
		int client = ioctl_param;

		return acrn_ioreq_attach_client(client, 0);
	}

	case IC_NOTIFY_REQUEST_FINISH: {
		struct ioreq_notify notify;

		if (copy_from_user(&notify, (void *)ioctl_param,
					sizeof(notify)))
			return -EFAULT;

		ret = acrn_ioreq_complete_request(notify.client_id,
				notify.vcpu, NULL);
		if (ret < 0)
			return -EFAULT;
		break;
	}

	case IC_ASSERT_IRQLINE: {
		struct acrn_irqline irq;

		if (copy_from_user(&irq, (void *)ioctl_param, sizeof(irq)))
			return -EFAULT;

		ret = hcall_assert_irqline(vm->vmid, virt_to_phys(&irq));
		if (ret < 0) {
			pr_err("vhm: failed to assert irq!\n");
			return -EFAULT;
		}
		break;
	}
	case IC_DEASSERT_IRQLINE: {
		struct acrn_irqline irq;

		if (copy_from_user(&irq, (void *)ioctl_param, sizeof(irq)))
			return -EFAULT;

		ret = hcall_deassert_irqline(vm->vmid, virt_to_phys(&irq));
		if (ret < 0) {
			pr_err("vhm: failed to deassert irq!\n");
			return -EFAULT;
		}
		break;
	}
	case IC_PULSE_IRQLINE: {
		struct acrn_irqline irq;

		if (copy_from_user(&irq, (void *)ioctl_param, sizeof(irq)))
			return -EFAULT;

		ret = hcall_pulse_irqline(vm->vmid,
					virt_to_phys(&irq));
		if (ret < 0) {
			pr_err("vhm: failed to assert irq!\n");
			return -EFAULT;
		}
		break;
	}

	case IC_CLEAR_VM_IOREQ: {
		/*
		 * TODO: Query VM status with additional hypercall.
		 * VM should be in paused status.
		 *
		 * In SMP SOS, we need flush the current pending ioreq dispatch
		 * tasklet and finish it before clearing all ioreq of this VM.
		 * With tasklet_kill, there still be a very rare race which
		 * might lost one ioreq tasklet for other VMs. So arm one after
		 * the clearing. It's harmless.
		 */
		tasklet_schedule(&vhm_io_req_tasklet);
		tasklet_kill(&vhm_io_req_tasklet);
		tasklet_schedule(&vhm_io_req_tasklet);
		acrn_ioreq_clear_request(vm);
		break;
	}

	case IC_SET_IRQLINE: {
		ret = hcall_set_irqline(vm->vmid, ioctl_param);
		if (ret < 0) {
			pr_err("vhm: failed to set irqline!\n");
			return -EFAULT;
		}
		break;
	}

	case IC_INJECT_MSI: {
		struct acrn_msi_entry *msi;

		msi = acrn_mempool_alloc(GFP_KERNEL);

		if (copy_from_user(msi, (void *)ioctl_param, sizeof(*msi))) {
			acrn_mempool_free(msi);
			return -EFAULT;
		}

		ret = hcall_inject_msi(vm->vmid, virt_to_phys(msi));
		acrn_mempool_free(msi);
		if (ret < 0) {
			pr_err("vhm: failed to inject!\n");
			return -EFAULT;
		}
		break;
	}

	case IC_ASSIGN_PTDEV: {
		uint16_t bdf;

		if (copy_from_user(&bdf,
				(void *)ioctl_param, sizeof(uint16_t)))
			return -EFAULT;

		ret = hcall_assign_ptdev(vm->vmid, bdf);
		if (ret < 0) {
			pr_err("vhm: failed to assign ptdev!\n");
			return -EFAULT;
		}
		break;
	}
	case IC_DEASSIGN_PTDEV: {
		uint16_t bdf;

		if (copy_from_user(&bdf,
				(void *)ioctl_param, sizeof(uint16_t)))
			return -EFAULT;

		ret = hcall_deassign_ptdev(vm->vmid, bdf);
		if (ret < 0) {
			pr_err("vhm: failed to deassign ptdev!\n");
			return -EFAULT;
		}
		break;
	}

	case IC_SET_PTDEV_INTR_INFO: {
		struct hc_ptdev_irq *hc_pt_irq;

		if (copy_from_user(&ic_pt_irq,
				(void *)ioctl_param, sizeof(ic_pt_irq)))
			return -EFAULT;

		hc_pt_irq = acrn_mempool_alloc(GFP_KERNEL);
		memcpy(hc_pt_irq, &ic_pt_irq, sizeof(*hc_pt_irq));

		ret = hcall_set_ptdev_intr_info(vm->vmid,
				virt_to_phys(hc_pt_irq));

		acrn_mempool_free(hc_pt_irq);
		if (ret < 0) {
			pr_err("vhm: failed to set intr info for ptdev!\n");
			return -EFAULT;
		}

		break;
	}
	case IC_RESET_PTDEV_INTR_INFO: {
		struct hc_ptdev_irq *hc_pt_irq;

		if (copy_from_user(&ic_pt_irq,
				(void *)ioctl_param, sizeof(ic_pt_irq)))
 			return -EFAULT;

		hc_pt_irq = acrn_mempool_alloc(GFP_KERNEL);
		memcpy(hc_pt_irq, &ic_pt_irq, sizeof(*hc_pt_irq));

		ret = hcall_reset_ptdev_intr_info(vm->vmid,
				virt_to_phys(hc_pt_irq));
		if (ret < 0) {
			pr_err("vhm: failed to reset intr info for ptdev!\n");
			acrn_mempool_free(hc_pt_irq);
			return -EFAULT;
		}
		acrn_mempool_free(hc_pt_irq);
		break;
	}

	case IC_VM_PCI_MSIX_REMAP: {
		/* This is not used any more */
		ret = -EFAULT;
		break;
	}

	case IC_PM_GET_CPU_STATE: {
		uint64_t cmd;

		if (copy_from_user(&cmd,
				(void *)ioctl_param, sizeof(cmd)))
			return -EFAULT;

		switch (cmd & PMCMD_TYPE_MASK) {
		case PMCMD_GET_PX_CNT:
		case PMCMD_GET_CX_CNT: {
			uint64_t *pm_info;

			pm_info = acrn_mempool_alloc(GFP_KERNEL);
			ret = hcall_get_cpu_state(cmd, virt_to_phys(pm_info));
			if (ret < 0) {
				acrn_mempool_free(pm_info);
				return -EFAULT;
			}

			if (copy_to_user((void *)ioctl_param,
					pm_info, sizeof(*pm_info)))
					ret = -EFAULT;
			acrn_mempool_free(pm_info);
			break;
		}
		case PMCMD_GET_PX_DATA: {
			struct cpu_px_data *px_data;

			px_data = acrn_mempool_alloc(GFP_KERNEL);
			ret = hcall_get_cpu_state(cmd, virt_to_phys(px_data));
			if (ret < 0) {
				acrn_mempool_free(px_data);
				return -EFAULT;
			}

			if (copy_to_user((void *)ioctl_param,
					px_data, sizeof(*px_data)))
					ret = -EFAULT;
			acrn_mempool_free(px_data);
			break;
		}
		case PMCMD_GET_CX_DATA: {
			struct cpu_cx_data *cx_data;

			cx_data = acrn_mempool_alloc(GFP_KERNEL);

			ret = hcall_get_cpu_state(cmd, virt_to_phys(cx_data));
			if (ret < 0) {
				acrn_mempool_free(cx_data);
				return -EFAULT;
			}

			if (copy_to_user((void *)ioctl_param,
					cx_data, sizeof(*cx_data)))
					ret = -EFAULT;
			acrn_mempool_free(cx_data);
			break;
		}
		default:
			ret = -EFAULT;
			break;
		}
		break;
	}

	case IC_VM_INTR_MONITOR: {
		struct page *page;

		ret = get_user_pages_fast(ioctl_param, 1, 1, &page);
		if (unlikely(ret != 1) || (page == NULL)) {
			pr_err("vhm-dev: failed to pin intr hdr buffer!\n");
			return -ENOMEM;
		}

		ret = hcall_vm_intr_monitor(vm->vmid, page_to_phys(page));
		if (ret < 0) {
			pr_err("vhm-dev: monitor intr data err=%ld\n", ret);
			return -EFAULT;
		}
		break;
	}

	case IC_EVENT_IOEVENTFD: {
		struct acrn_ioeventfd args;

		if (copy_from_user(&args, (void *)ioctl_param, sizeof(args)))
			return -EFAULT;
		ret = acrn_ioeventfd(vm->vmid, &args);
		break;
	}

	case IC_EVENT_IRQFD: {
		struct acrn_irqfd args;

		if (copy_from_user(&args, (void *)ioctl_param, sizeof(args)))
			return -EFAULT;
		ret = acrn_irqfd(vm->vmid, &args);
		break;
	}

	default:
		pr_warn("Unknown IOCTL 0x%x\n", ioctl_num);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void io_req_tasklet(unsigned long data)
{
	struct vhm_vm *vm;

	read_lock(&vhm_vm_list_lock);
	list_for_each_entry(vm, &vhm_vm_list, list) {
		if (!vm || !vm->req_buf)
			continue;

		acrn_ioreq_distribute_request(vm);
	}
	read_unlock(&vhm_vm_list_lock);
}

static void vhm_intr_handler(void)
{
	tasklet_schedule(&vhm_io_req_tasklet);
}

int vhm_vm_destroy(struct vhm_vm *vm)
{
	int ret;

	if (test_and_set_bit(VHM_VM_DESTROYED, &vm->flags))
		return -ENODEV;

	acrn_ioeventfd_deinit(vm->vmid);
	acrn_irqfd_deinit(vm->vmid);
	acrn_ioreq_free(vm);

	ret = hcall_destroy_vm(vm->vmid);
	if (ret < 0)
		pr_err("Failed to destroy VM %ld!\n", vm->vmid);
	write_lock_bh(&vhm_vm_list_lock);
	list_del_init(&vm->list);
	write_unlock_bh(&vhm_vm_list_lock);
	vm->vmid = ACRN_INVALID_VMID;

	return 0;
}

static int vhm_dev_release(struct inode *inodep, struct file *filep)
{
	struct vhm_vm *vm = filep->private_data;

	if (vm == NULL) {
		pr_err("vhm: invalid VM !\n");
		return -EFAULT;
	}
	vhm_vm_destroy(vm);
	put_vm(vm);
	filep->private_data = NULL;
	return 0;
}

static const struct file_operations fops = {
	.open = vhm_dev_open,
	.read = vhm_dev_read,
	.write = vhm_dev_write,
	.release = vhm_dev_release,
	.unlocked_ioctl = vhm_dev_ioctl,
	.poll = vhm_dev_poll,
};

static ssize_t
store_offline_cpu(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
#ifdef CONFIG_X86
	u64 cpu, lapicid;

	if (kstrtoull(buf, 0, &cpu) < 0)
		return -EINVAL;

	if (cpu_possible(cpu)) {
		lapicid = cpu_data(cpu).apicid;
		pr_info("vhm: try to offline cpu %lld with lapicid %lld\n",
				cpu, lapicid);
		if (hcall_sos_offline_cpu(lapicid) < 0) {
			pr_err("vhm: failed to offline cpu from Hypervisor!\n");
			return -EINVAL;
		}
	}
#endif
	return count;
}

static DEVICE_ATTR(offline_cpu, S_IWUSR, NULL, store_offline_cpu);

static struct attribute *vhm_attrs[] = {
	&dev_attr_offline_cpu.attr,
	NULL
};

static struct attribute_group vhm_attr_group = {
	.attrs = vhm_attrs,
};

#define SUPPORT_HV_API_VERSION_MAJOR	1
#define SUPPORT_HV_API_VERSION_MINOR	0
static int __init vhm_init(void)
{
	static struct hc_api_version api_version;

	if (x86_hyper_type != X86_HYPER_ACRN)
		return -ENODEV;

	pr_info("vhm: initializing\n");

	if (hcall_get_api_version(virt_to_phys(&api_version)) < 0) {
		pr_err("vhm: failed to get api version from Hypervisor !\n");
		return -EINVAL;
	}

	if (api_version.major_version == SUPPORT_HV_API_VERSION_MAJOR &&
		api_version.minor_version == SUPPORT_HV_API_VERSION_MINOR) {
		pr_info("vhm: hv api version %d.%d\n",
			api_version.major_version, api_version.minor_version);
	} else {
		pr_err("vhm: not support hv api version %d.%d!\n",
			api_version.major_version, api_version.minor_version);
		return -EINVAL;
	}

	/* Try to dynamically allocate a major number for the device */
	major = register_chrdev(0, DEVICE_NAME, &fops);
	if (major < 0) {
		pr_warn("vhm: failed to register a major number\n");
		return major;
	}
	pr_info("vhm: registered correctly with major number %d\n", major);

	/* Register the device class */
	vhm_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(vhm_class)) {
		unregister_chrdev(major, DEVICE_NAME);
		pr_warn("vhm: failed to register device class\n");
		return PTR_ERR(vhm_class);
	}
	pr_info("vhm: device class registered correctly\n");

	/* Register the device driver */
	vhm_device = device_create(vhm_class, NULL, MKDEV(major, 0),
		NULL, DEVICE_NAME);
	if (IS_ERR(vhm_device)) {
		class_destroy(vhm_class);
		unregister_chrdev(major, DEVICE_NAME);
		pr_warn("vhm: failed to create the device\n");
		return PTR_ERR(vhm_device);
	}
	pr_info("register IPI handler\n");
	tasklet_init(&vhm_io_req_tasklet, io_req_tasklet, 0);

	acrn_setup_intr_irq(vhm_intr_handler);
	if (sysfs_create_group(&vhm_device->kobj, &vhm_attr_group)) {
		pr_warn("vhm: sysfs create failed\n");
		return -EINVAL;
	}

	acrn_ioreq_driver_init();
	/* initialize memory pool with 16 elements and 512 bytes element size */
	acrn_mempool_init(16, 512);
	pr_info("vhm: Virtio & Hypervisor service module initialized\n");
	return 0;
}
static void __exit vhm_exit(void)
{
	tasklet_kill(&vhm_io_req_tasklet);
	acrn_remove_intr_irq();
	device_destroy(vhm_class, MKDEV(major, 0));
	class_unregister(vhm_class);
	class_destroy(vhm_class);
	unregister_chrdev(major, DEVICE_NAME);
	sysfs_remove_group(&vhm_device->kobj, &vhm_attr_group);

	acrn_mempool_deinit();
	pr_info("vhm: exit\n");
}

module_init(vhm_init);
module_exit(vhm_exit);

MODULE_AUTHOR("Intel");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("This is a char device driver, acts as a route "
		"responsible for transferring IO requsts from other modules "
		"either in user-space or in kernel to and from hypervisor");
MODULE_VERSION("0.1");
