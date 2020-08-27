// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Intel Corporation
 *   VPUSMM Kernel module
 */
#include <asm/page.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/printk.h>
#include <linux/wait.h>
#include <uapi/linux/stat.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/dma-direct.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>

#include <uapi/misc/vpusmm_uapi.h>

#define DRIVER_NAME		"vpusmm"

static dev_t vpusmm_device_number;
static struct class *vpusmm_sys_class;
static int vpusmm_platform_driver_registered;
static struct platform_device *vpusmm_platform_dev;

#define MAX_RSVMEM_CNT  32

/* There is only single instance of this type of device
 * so all information is global
 */
struct vpusmm_device {
	struct device *dev;
	struct device *mem_dev[MAX_RSVMEM_CNT];
	struct resource mem_res[MAX_RSVMEM_CNT];
	dev_t device_number;
	struct cdev cdev;
	int rmem_cnt;
};

struct vpusmm_session {
	struct vpusmm_device *dev;

	struct rb_root	 imp_rb;
	struct mutex	 imp_rb_lock;
};

/* DMABuf exported by VPU device is described by following data structure
 * the life of the buffer may be longer than session(it may be shared with
 * other driver),so it contains pointer to device rather than to session
 */
struct vpusmm_buffer {
	// DMA API device
	struct device *dev;

	// the kernel space virtual address of the DMA buffer
	void *cookie;

	// the dma address for current device
	dma_addr_t dma_addr;

	// length of the buffer
	size_t size;

	// DMA attr
	unsigned long dma_attrs;
};

/* DMABufs imported to VPU device are maintained in a rb tree with dmabuf's
 * pointer as key. so user space can unimport it by specifying fd and ptr2vpu
 * API can work by searching this rb tree.
 */
struct vpusmm_imported_dmabuf {
	struct rb_node node;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction direction;
	dma_addr_t vpu_addr;
	int refcount;
};

/* whether the sg_table is accessible by VPU side.
 * VPU can only access physical contiguous memory for now
 * this checks is for importting DMABuf from other driver
 */
static int is_sgt_vpu_accessible(struct device *dev, struct sg_table *sgt)
{
	if (sgt->nents >= 1) {
		/* check whether the entries in the sg_table are
		 * contiguous and accessible by VPU
		 */
		dma_addr_t next_addr = sg_dma_address(sgt->sgl);
		struct scatterlist *s;
		unsigned int i;

		for_each_sg(sgt->sgl, s, sgt->nents, i) {
			/*
			 *sg_dma_address(s) is only valid for entries
			 *that have sg_dma_len(s) != 0
			 */
			if (!sg_dma_len(s))
				continue;

			if (sg_dma_address(s) != next_addr)
				return 0;

			next_addr = sg_dma_address(s) + sg_dma_len(s);
		}
		return (next_addr - 1) <=
			min_not_zero(dev->coherent_dma_mask, dev->bus_dma_mask);
	}
	return 0;
}

/**********************************
 * VPU imported dmabuf management *
 **********************************/
static void
vpusmm_session_insert_imp_dmabuf(
	struct vpusmm_session *sess,
	struct vpusmm_imported_dmabuf *new_item)
{
	struct rb_root *root = &sess->imp_rb;
	struct rb_node **iter = &root->rb_node, *parent = NULL;
	struct dma_buf *value = new_item->dmabuf;
	struct vpusmm_imported_dmabuf *item;

	/*Go to the bottom of the tree */
	while (*iter) {
		parent = *iter;
		item = rb_entry(parent, struct vpusmm_imported_dmabuf, node);

		if (item->dmabuf == value)
			pr_err(" %s error: insert exist dmabuf!\n", __func__);

		if (item->dmabuf > value)
			iter = &(*iter)->rb_left;
		else
			iter = &(*iter)->rb_right;
	}

	/*Put the new node there */
	rb_link_node(&new_item->node, parent, iter);
	rb_insert_color(&new_item->node, root);
}

static struct vpusmm_imported_dmabuf *
vpusmm_session_find_imp_dmabuf(
	struct vpusmm_session *sess,
	struct dma_buf *dmabuf)
{
	struct rb_node *node = sess->imp_rb.rb_node;  /*top of the tree */
	struct vpusmm_imported_dmabuf *item = NULL;

	while (node) {
		item = rb_entry(node, struct vpusmm_imported_dmabuf, node);

		if (item->dmabuf > dmabuf)
			node = node->rb_left;
		else if (item->dmabuf < dmabuf)
			node = node->rb_right;
		else
			return item;  /*Found it */
	}
	return NULL;
}

static struct vpusmm_imported_dmabuf *
vpusmm_session_import_dmabuf(
	struct vpusmm_session *sess,
	struct dma_buf *dmabuf,
	enum dma_data_direction direction,
	int vpu_id)
{
	struct vpusmm_imported_dmabuf *item;
	struct dma_buf_attachment *attach;
	struct device *dma_dev = sess->dev->dev->parent;
	struct sg_table *sgt;

	//check whether its already imported
	mutex_lock(&sess->imp_rb_lock);
	item = vpusmm_session_find_imp_dmabuf(sess, dmabuf);
	if (item) {
		//found already imported item, increase refcount and return
		item->refcount++;
		goto exit;
	}

	attach = dma_buf_attach(dmabuf, dma_dev);
	if (IS_ERR(attach)) {
		item = ERR_CAST(attach);
		goto exit;
	}

	sgt = dma_buf_map_attachment(attach, direction);
	if (IS_ERR(sgt)) {
		dma_buf_detach(dmabuf, attach);
		item = ERR_CAST(sgt);
		goto exit;
	}

	if (!is_sgt_vpu_accessible(dma_dev, sgt)) {
		dma_buf_unmap_attachment(attach, sgt, direction);
		dma_buf_detach(dmabuf, attach);
		item = ERR_PTR(-EINVAL);
		goto exit;
	}

	item = kzalloc(sizeof(*item), GFP_KERNEL);
	if (item == NULL) {
		dma_buf_unmap_attachment(attach, sgt, direction);
		dma_buf_detach(dmabuf, attach);
		item = ERR_PTR(-ENOMEM);
		goto exit;
	}

	/*
	 *	the vpu_addr here needs special care, DMAbuf exporter's callback
	 *  map_dma_buf() will give dma_address for our device usually.
	 *	but in fake IOMMU cases, our device is just a fake platform device
	 *  and dma_address is simply the physical address, we need to translate it
	 *  into right dma_address given vpu_id.
	 *
	 *	1.in accordance with Linux DMA framework, we should add iommus into
	 *    vpusmm device tree node and map_dma_buf() will automatically
	 *    setup the mapping and return dma_address.
	 *
	 *	2.if use 1:1 mapping between vpusmm device and vpu slice, we should
	 *    extract vpu_id from MINOR(sess->dev->device_number) rather than
	 *    support any vpu_id.
	 */
	item->dmabuf = dmabuf;
	item->attach = attach;
	item->sgt = sgt;
	item->direction = direction;
	item->vpu_addr = sg_dma_address(sgt->sgl);
	item->refcount = 1;

	// insert into the rb tree
	vpusmm_session_insert_imp_dmabuf(sess, item);

exit:
	mutex_unlock(&sess->imp_rb_lock);
	return item;
}


static void vpusmm_session_unimport_dmabuf(
	struct vpusmm_session *sess,
	struct dma_buf *dmabuf)
{
	struct vpusmm_imported_dmabuf *item = NULL;

	mutex_lock(&sess->imp_rb_lock);
		item = vpusmm_session_find_imp_dmabuf(sess, dmabuf);
		if (item) {
			item->refcount--;
			if (item->refcount <= 0) {
				rb_erase(&item->node, &sess->imp_rb);
				dma_buf_unmap_attachment(item->attach, item->sgt, item->direction);
				dma_buf_detach(item->dmabuf, item->attach);
				dma_buf_put(item->dmabuf);
				kfree(item);
			}
		}
	mutex_unlock(&sess->imp_rb_lock);
}

/******************************
 * driver's open/release fops *
 ******************************/
static int vpusmm_open(struct inode *inode, struct file *filp)
{
	int retval = 0;
	struct vpusmm_session *sess;
	struct vpusmm_device *dev;

	dev = container_of(inode->i_cdev, struct vpusmm_device, cdev);

	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sess)) {
		retval = PTR_ERR(sess);
		sess = NULL;
		goto failed;
	}
	sess->dev = dev;
	sess->imp_rb = RB_ROOT;

	mutex_init(&sess->imp_rb_lock);

	// create session
	filp->private_data = sess;

	return retval;
failed:
	return retval;
}

static int vpusmm_release(struct inode *inode, struct file *filp)
{
	struct vpusmm_session *sess = filp->private_data;

	pr_info("[%s] PID:%d filp=%p iminor=%d, release f_flags=%#o\n",
		__func__, current->pid, filp, iminor(inode), filp->f_flags);

	// release all dmabuf maps that may be leaked
	mutex_lock(&sess->imp_rb_lock);
	{
		struct rb_node *cur = rb_first(&sess->imp_rb);

		while (cur) {
			struct vpusmm_imported_dmabuf *item = rb_entry(cur, struct vpusmm_imported_dmabuf, node);
			struct rb_node *next = rb_next(cur);

			if (item) {
				pr_info("[%s] PID:%d free leaked imported dmabuf %zu bytes, %d refs\n",
					__func__, current->pid, item->dmabuf->size, item->refcount);
				dma_buf_unmap_attachment(item->attach, item->sgt, item->direction);
				dma_buf_detach(item->dmabuf, item->attach);
				while (item->refcount > 0) {
					dma_buf_put(item->dmabuf);
					item->refcount--;
				}
				rb_erase(&item->node, &sess->imp_rb);
				kfree(item);
			}
			cur = next;
		}
	}
	mutex_unlock(&sess->imp_rb_lock);
	mutex_destroy(&sess->imp_rb_lock);

	kfree(sess);
	return 0;
}


/*********************
 *  DMABuf allocator *
 *********************/
static struct sg_table *vpusmm_map_dma_buf(
	struct dma_buf_attachment *attach,
	enum dma_data_direction dir)
{
	struct vpusmm_buffer *buff = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	if (WARN_ON(dir == DMA_NONE))
		return ERR_PTR(-EINVAL);

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	ret = dma_get_sgtable(buff->dev, sgt, buff->cookie, buff->dma_addr, buff->size);

	if (!ret) {
		if (!dma_map_sg_attrs(attach->dev, sgt->sgl, sgt->nents, dir,
					  DMA_ATTR_SKIP_CPU_SYNC)) {
			sg_free_table(sgt);
			kfree(sgt);
			sgt = ERR_PTR(-ENOMEM);
		} else {
			// success
		}
	}
	return sgt;
}

static void vpusmm_unmap_dma_buf(
	struct dma_buf_attachment *attach,
	struct sg_table *sgt,
	enum dma_data_direction dir)
{
	if (sgt) {
		if (dir != DMA_NONE)
			dma_unmap_sg_attrs(attach->dev, sgt->sgl,
					   sgt->nents,
					   dir,
					   DMA_ATTR_SKIP_CPU_SYNC);
		sg_free_table(sgt);
		kfree(sgt);
	}
}

void vpusmm_dmabuf_release(struct dma_buf *dmabuf)
{
	// this buffer is only referencec by dmabuf
	struct vpusmm_buffer *buff = dmabuf->priv;

	if (buff) {
		if (buff->cookie) {
			dma_free_attrs(buff->dev, buff->size, buff->cookie, buff->dma_addr, buff->dma_attrs);
			buff->cookie = NULL;
		}
		kfree(buff);
	}
}

// set USE_VMF to 1 to lazy mmap by page fault handler
static void vpusmm_device_vma_open(struct vm_area_struct *vma)
{
	(void)vma;
}
static void vpusmm_device_vma_close(struct vm_area_struct *vma)
{
	(void)vma;
}
static vm_fault_t vpusmm_device_vma_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma  = vmf->vma;
	struct vpusmm_buffer *buff = vma->vm_private_data;
	unsigned long offset	= vmf->pgoff << PAGE_SHIFT;
	unsigned long phys_base = dma_to_phys(buff->dev, buff->dma_addr);
	unsigned long phys_addr = phys_base + offset;
	unsigned long page_frame_num  = phys_addr >> PAGE_SHIFT;

	if (offset + (1<<PAGE_SHIFT) > PAGE_ALIGN(buff->size))
		return VM_FAULT_SIGBUS;

	if (!pfn_valid(page_frame_num))
		return VM_FAULT_SIGBUS;
	//pr_info("[vpusmm]: vpusmm_device_vma_fault at offset=0x%lx\n", offset);

	// Using page struct for mmap has following problem:
	//   when pages comes from Buddy instead of CMA, only the head page has been recounted,
	//   the rest pages is not refcounted, this is not a problem when the life cycle of
	//   the pages are not maintained with get_page()/put_page(), but when pass
	//   them into mmap, munmap will decrease refcount and free pages with zero refcount.
	// So instead we use PFN mapping to keep VM from manipulating page struct.
	//   we can see from zap_pte_range(), a special flag pte_special must be present
	//   together with VM_PFNMAP to prevent VM from accessing page struct. that's
	//   what remap_pte_range() & vmf_insert_pfn() does to make a special pte.
	//
	// See "Migrating nopfn to fault" from https://linux-mm.org/DeviceDriverMmap
	vmf->page = NULL;
	return vmf_insert_pfn(vma, vmf->address, page_frame_num);
}
static const struct vm_operations_struct vpusmm_device_vm_ops = {
	.open	= vpusmm_device_vma_open,
	.close   = vpusmm_device_vma_close,
	.fault   = vpusmm_device_vma_fault,
};

static int vpusmm_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct vpusmm_buffer *buff = dmabuf->priv;
	unsigned long vm_size;
	int ret;

	if (buff->dma_attrs & DMA_ATTR_NON_CONSISTENT) {
		vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP;
		vma->vm_ops = &vpusmm_device_vm_ops;
	} else {
		vma->vm_flags &= ~VM_PFNMAP;
		vma->vm_pgoff = 0;
		vm_size = vma->vm_end - vma->vm_start;

		/*check if user-requested size is valid. */
		if (vm_size > PAGE_ALIGN(buff->size)) {
			pr_info("[vpusmm]: vm_size too big\n");
			return -EINVAL;
		}
		ret = dma_mmap_attrs(buff->dev,
							  vma,
							  buff->cookie,
							  buff->dma_addr,
							  buff->size,
							  buff->dma_attrs);
		if (ret < 0) {
			pr_info("[vpusmm]: dma_mmap_attrs() failed with %d\n", ret);
			return ret;
		}
	}

	vma->vm_private_data = buff;
	return 0;
}
static int vpusmm_dmabuf_begin_cpu_access(struct dma_buf *dmabuf, enum dma_data_direction direction)
{
	struct vpusmm_buffer *buff = dmabuf->priv;

	if (buff->dma_attrs & DMA_ATTR_NON_CONSISTENT)
		dma_sync_single_for_cpu(buff->dev, buff->dma_addr, buff->size, direction);

	return 0;
}
static int vpusmm_dmabuf_end_cpu_access(struct dma_buf *dmabuf, enum dma_data_direction  direction)
{
	struct vpusmm_buffer *buff = dmabuf->priv;

	if (buff->dma_attrs & DMA_ATTR_NON_CONSISTENT)
		dma_sync_single_for_device(buff->dev, buff->dma_addr, buff->size, direction);

	return 0;
}
static void *vpusmm_dmabuf_vmap(struct dma_buf *dmabuf)
{
	struct vpusmm_buffer *buff = dmabuf->priv;

	return buff->cookie;
}

static const struct dma_buf_ops vpusmm_dmabuf_ops =  {
	.map_dma_buf = vpusmm_map_dma_buf,
	.unmap_dma_buf = vpusmm_unmap_dma_buf,
	.release = vpusmm_dmabuf_release,
	.mmap = vpusmm_dmabuf_mmap,

	.begin_cpu_access = vpusmm_dmabuf_begin_cpu_access,
	.end_cpu_access = vpusmm_dmabuf_end_cpu_access,
	.vmap = vpusmm_dmabuf_vmap,
};

/* allocate dma buffer suitable for VPU access.
 * export as DMABuf fd
 * sess will hold additional refcount to the dmabuf
 * on request of passing it to VPU side for processing
 */
static long vpusmm_session_alloc(struct vpusmm_session *sess, struct vpusmm_args_alloc *arg)
{
	int retval = 0;
	size_t buffer_size = arg->size;
	struct vpusmm_device *dev = sess->dev;
	struct dma_buf *dmabuf = NULL;
	const int flags = O_RDWR | O_CLOEXEC;
	phys_addr_t phys_addr;
	struct dma_buf_export_info exp_info = {
		.exp_name = KBUILD_MODNAME, /*white lie for debug */
		.owner = THIS_MODULE,
		.ops = &vpusmm_dmabuf_ops,
		.size = buffer_size,
		.flags = flags
	};
	struct vpusmm_buffer *buff = NULL;

	// only allocate from the first rmem, user-space API unchanged
	if (dev->rmem_cnt <= 0) {
		retval = -ENODEV;
		goto failed;
	}

	buff = kzalloc(sizeof(*buff), GFP_KERNEL);
	if (buff == NULL) {
		retval = -ENOMEM;
		goto failed;
	}

	buff->dev = dev->mem_dev[0];
	buff->size = buffer_size;

	buff->dma_attrs = 0;

	if (arg->force_physical_contiguous)
		buff->dma_attrs |= DMA_ATTR_FORCE_CONTIGUOUS;

	if (arg->cachable)
		buff->dma_attrs |= DMA_ATTR_NON_CONSISTENT;
	else
		buff->dma_attrs |= DMA_ATTR_WRITE_COMBINE;

	// we need Kernel mapping to support dmabuf op vmap
	buff->dma_attrs &= ~DMA_ATTR_NO_KERNEL_MAPPING;

	buff->cookie = dma_alloc_attrs(
						dev->mem_dev[0],
						buff->size,
						&buff->dma_addr,
						GFP_KERNEL | GFP_DMA,
						buff->dma_attrs);
	if (buff->cookie == NULL) {
		pr_info("[VPUSMM]:dma_alloc_attrs failed: size=%zu\n", buff->size);
		retval = -ENOMEM;
		goto failed;
	}

	phys_addr = dma_to_phys(dev->mem_dev[0], buff->dma_addr);

	//export the buffer as DMABuf
	exp_info.priv = buff;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		retval = PTR_ERR(dmabuf);
		dmabuf = NULL;
		goto failed;
	}

	retval = dma_buf_fd(dmabuf, flags);
	if (retval < 0) {
		goto failed;
	} else {
		arg->fd = retval;
		retval = 0;
	}

	pr_debug("%s: dma_addr=%llx, phys_addr=%llx allocated from %s\n",
			__func__, buff->dma_addr, phys_addr, dev_name(dev->dev));

	return 0;
failed:
	pr_err("%s failed with %d\n", __func__, retval);

	if (dmabuf) {
		// this will finally release underlying buff
		dma_buf_put(dmabuf);
	} else if (buff) {
		if (buff->cookie)
			dma_free_attrs(dev->mem_dev[0], buff->size, buff->cookie, buff->dma_addr, buff->dma_attrs);
		kfree(buff);
	}
	return retval;
}

static long vpusmm_session_import(
	struct vpusmm_session *sess,
	struct vpusmm_args_import *arg)
{
	struct dma_buf *dmabuf = NULL;
	struct vpusmm_imported_dmabuf *item;
	enum dma_data_direction direction;

	switch (arg->vpu_access) {
	case 1: //VPU_READ
		direction = DMA_TO_DEVICE;
		break;
	case 2: //VPU_WRITE
		direction = DMA_FROM_DEVICE;
		break;
	case 0: //VPU_DEFAULT: when user is not sure
	case 3: //VPU_RW == VPU_READ | VPU_WRITE
		direction = DMA_BIDIRECTIONAL;
		break;
	default:
		pr_info("[vpusmm]: Unknown vpu_access:%d\n", arg->vpu_access);
		return -EINVAL;
	}

	dmabuf = dma_buf_get(arg->fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	item = vpusmm_session_import_dmabuf(sess, dmabuf, direction, 0);
	if (IS_ERR(item)) {
		dma_buf_put(dmabuf);
		return PTR_ERR(item);
	}

	arg->vpu_addr = item->vpu_addr;
	arg->size = dmabuf->size;
	return 0;
}
static long vpusmm_session_unimport(struct vpusmm_session *sess, int *p_dmabuf_fd)
{
	struct dma_buf *dmabuf = NULL;

	dmabuf = dma_buf_get(*p_dmabuf_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	vpusmm_session_unimport_dmabuf(sess, dmabuf);
	dma_buf_put(dmabuf);
	return 0;
}

static long vpusmm_session_ptr2vpu(struct vpusmm_session *sess, unsigned long *arg)
{
	unsigned long vaddr = *arg;
	struct dma_buf *dmabuf = NULL;
	struct vm_area_struct *vma;
	struct vpusmm_imported_dmabuf *item;
	struct mm_struct *mm = NULL;
	struct task_struct *task = current;

	mm = get_task_mm(task);
	if (!mm) {
		pr_info("[vpusmm]: failed at line %d\n", __LINE__);
		goto failed;
	}
	down_read(&mm->mmap_sem);

	vma = find_vma(mm, vaddr);
	if (!vma) {
		pr_info("[vpusmm]: failed at line %d\n", __LINE__);
		goto failed;
	}

	if (vaddr < vma->vm_start) {
		pr_info("[vpusmm]: failed at line %d, vaddr=%lx, vma->vm_start=%lx\n", __LINE__, vaddr, vma->vm_start);
		goto failed;
	}

	// make sure the vma is backed by a dmabuf
	if (vma->vm_file == NULL) {
		pr_info("[vpusmm]: failed at line %d\n", __LINE__);
		goto failed;
	}

	dmabuf = vma->vm_file->private_data;
	if (dmabuf == NULL) {
		pr_info("[vpusmm]: failed at line %d\n", __LINE__);
		goto failed;
	}

	// how do we know it's a dmabuf backed file?
	// our imported dmabuf rbtree should have it inserted
	if (dmabuf->file != vma->vm_file) {
		pr_info("[vpusmm]: failed at line %d\n", __LINE__);
		goto failed;
	}
	up_read(&mm->mmap_sem);
	mmput(mm);

	mutex_lock(&sess->imp_rb_lock);
	item = vpusmm_session_find_imp_dmabuf(sess, dmabuf);
	mutex_unlock(&sess->imp_rb_lock);

	if (!item) {
		pr_info("[vpusmm]: failed to find the dmabuf in imported list for vaddr=0x%lx (%d)\n",
			vaddr, __LINE__);
		return -EFAULT;
	}

	*arg = (dma_addr_t)(vaddr - vma->vm_start) + item->vpu_addr;
	return 0;

failed:
	if (mm) {
		up_read(&mm->mmap_sem);
		mmput(mm);
	}
	return -EFAULT;
}

static long vpusmm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long retval = 0;
	struct vpusmm_session *sess = filp->private_data;
	const unsigned int io_dir = _IOC_DIR(cmd);
	const unsigned int io_size = _IOC_SIZE(cmd);
	char tmp[128] = {0};

	if (_IOC_TYPE(cmd) != VPUSMM_IOC_MAGIC ||
		_IOC_NR(cmd) >= _IOC_NR(VPUSMM_IOCTL_END))
		return -EINVAL;

	if (io_size > sizeof(tmp))
		return -EFAULT;

	if (io_dir & _IOC_READ) {
		if (copy_from_user(tmp, (void __user *)arg, io_size) != 0)
			return  -EFAULT;
	}

	switch (cmd) {
	case VPUSMM_IOCTL_ALLOC:
		retval = vpusmm_session_alloc(sess, (void *)tmp);
	break;
	case VPUSMM_IOCTL_IMPORT:
		retval = vpusmm_session_import(sess, (void *)tmp);
	break;
	case VPUSMM_IOCTL_UNIMPORT:
		retval = vpusmm_session_unimport(sess, (void *)tmp);
	break;
	case VPUSMM_IOCTL_PTR2VPU:
		retval = vpusmm_session_ptr2vpu(sess, (void *)tmp);
	break;
	case VPUSMM_IOCTL_PTR2PHYS:
		/*not supported any more */
		retval = -EINVAL;
	break;
	}

	if (retval == 0) {
		if (io_dir & _IOC_WRITE) {
			if (copy_to_user((void __user *)arg, tmp, io_size) != 0)
				return -EFAULT;
		}
	}
	return retval;
}

static const struct file_operations vpusmm_chrdev_fops = {
	.owner = THIS_MODULE,
	.open = vpusmm_open,
	.release = vpusmm_release,
	.unlocked_ioctl = vpusmm_ioctl,
};

/*For automatically allocated device IDs */
static DEFINE_IDA(vpusmm_devid_ida);

static int vpusmm_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct vpusmm_device *vpusmm_dev = NULL;
	struct device *mem_dev = NULL;
	struct device_node *np;
	size_t mem_size;
	struct device *dev = &pdev->dev;
	int id = pdev->id;
	int i;

	id = ida_simple_get(&vpusmm_devid_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		rc = -ENOMEM;
		goto err;
	}

	dev_info(dev, "%s (id=%d) is being probed\n", pdev->name, id);

	vpusmm_dev = devm_kzalloc(dev, sizeof(*vpusmm_dev), GFP_KERNEL);
	if (!vpusmm_dev) {
		rc = -ENOMEM;
		goto err;
	}

	vpusmm_dev->device_number = MKDEV(MAJOR(vpusmm_device_number), id);
	vpusmm_dev->dev = device_create(vpusmm_sys_class, dev,
										vpusmm_dev->device_number, NULL,
										DRIVER_NAME"%d", id);
	if (IS_ERR_OR_NULL(vpusmm_dev->dev)) {
		rc = PTR_ERR(vpusmm_dev->dev);
		goto err;
	}

	cdev_init(&vpusmm_dev->cdev, &vpusmm_chrdev_fops);
	vpusmm_dev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&vpusmm_dev->cdev, vpusmm_dev->device_number, 1);
	if (rc != 0)
		goto err;

	/*
	 * no of_node imply:
	 * 1. no IOMMU, VPU device is only 32-bit DMA capable
	 * 2. use default CMA because no device tree node specifying memory-region
	 */
	if (dev->of_node == NULL) {
		dma_set_mask(dev, DMA_BIT_MASK(32));
		dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	} else {
		/*
		 *  this ugly, VPU dma addrerss mask should be 32, here we faking them as
		 * as a general 64-bit device and ask another vpu driver translate
		 * physical address into dma address.
		 */
		//dma_set_mask(dev, DMA_BIT_MASK(64));
		//dma_set_coherent_mask(dev, DMA_BIT_MASK(64));
	}

	vpusmm_dev->rmem_cnt = of_count_phandle_with_args(
							dev->of_node, "memory-region", NULL);
	if (vpusmm_dev->rmem_cnt <= 0) {
		dev_info(dev, "%s with default cma is created as cdev\n",
					dev_name(vpusmm_dev->dev));
		/*use default cma when no memory-region is specified */
		vpusmm_dev->mem_dev[0] = dev;
		vpusmm_dev->rmem_cnt = 1;
	} else {
		dev_info(dev, "%s with %d rmems is created as cdev\n",
					dev_name(vpusmm_dev->dev), vpusmm_dev->rmem_cnt);

		if (vpusmm_dev->rmem_cnt > MAX_RSVMEM_CNT) {
			vpusmm_dev->rmem_cnt = MAX_RSVMEM_CNT;
			dev_info(dev, "rmems number is limited to %d\n",
						vpusmm_dev->rmem_cnt);
		}

		for (i = 0; i < vpusmm_dev->rmem_cnt; i++) {
			/* Create a child devices (of platform dev, not the cdev)
			 * to own the reserved memory.
			 */
			mem_dev = devm_kzalloc(dev, sizeof(struct device), GFP_KERNEL);
			if (!mem_dev) {
				rc = -ENOMEM;
				goto err;
			}

			vpusmm_dev->mem_dev[i] = mem_dev;

			device_initialize(mem_dev);
			dev_set_name(mem_dev, "%s:%s%d", dev_name(dev), "mem", i);
			mem_dev->parent = dev;
			mem_dev->dma_mask = dev->dma_mask;
			mem_dev->coherent_dma_mask = dev->coherent_dma_mask;
			mem_dev->bus_dma_mask = dev->bus_dma_mask;
			/*Set up DMA configuration using information from parent's DT node. */
			rc = of_dma_configure(mem_dev, dev->of_node, true);
			mem_dev->release = of_reserved_mem_device_release;

			rc = device_add(mem_dev);
			if (rc)
				goto err;

			/*Initialized the device reserved memory region. */
			rc = of_reserved_mem_device_init_by_idx(mem_dev, dev->of_node, i);
			if (rc) {
				dev_err(dev, "Couldn't get reserved memory with idx = %d, %d\n", i, rc);
				goto err;
			}

			/*Find out the size of the memory region. */
			np = of_parse_phandle(dev->of_node, "memory-region", i);
			if (!np) {
				dev_err(dev, "Couldn't find memory-region %d\n", i);
				rc = -EINVAL;
				goto err;
			}
			rc = of_address_to_resource(np, 0, &vpusmm_dev->mem_res[i]);
			if (rc) {
				dev_err(dev, "Couldn't map address to resource %d\n", i);
				goto err;
			}
			mem_size = resource_size(&vpusmm_dev->mem_res[i]);

			dev_info(dev, " %s %zuMB %llx~%llx\n", dev_name(mem_dev), mem_size/(1024*1024),
						  vpusmm_dev->mem_res[i].start, vpusmm_dev->mem_res[i].end);
		}
	}

	platform_set_drvdata(pdev, vpusmm_dev);
	return 0;

err:
	dev_err(dev, "%s failed for id %d, rc=%d\n", __func__, id, rc);

	if (vpusmm_dev) {
		for (i = 0; i < vpusmm_dev->rmem_cnt; i++) {
			mem_dev = vpusmm_dev->mem_dev[i];
			if ((mem_dev != NULL) && (mem_dev != dev))
				device_unregister(mem_dev);
		}
		cdev_del(&vpusmm_dev->cdev);
		device_destroy(vpusmm_sys_class, vpusmm_dev->device_number);
	}
	if (id >= 0)
		ida_simple_remove(&vpusmm_devid_ida, id);
	return rc;
}

static int vpusmm_remove(struct platform_device *pdev)
{
	struct vpusmm_device *vpusmm_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct device *mem_dev = NULL;
	int i;
	int id;

	if (vpusmm_dev) {
		id = MINOR(vpusmm_dev->device_number);
		for (i = 0; i < vpusmm_dev->rmem_cnt; i++) {
			mem_dev = vpusmm_dev->mem_dev[i];
			if ((mem_dev != NULL) && (mem_dev != dev))
				device_unregister(mem_dev);
		}
		cdev_del(&vpusmm_dev->cdev);
		device_destroy(vpusmm_sys_class, vpusmm_dev->device_number);
		if (id >= 0)
			ida_simple_remove(&vpusmm_devid_ida, id);
	}
	return 0;
}

/**
 *Open Firmware Device Identifier Matching Table
 */
static const struct of_device_id vpusmm_of_match[] = {
	{ .compatible = "intel,keembay-vpusmm", },
	{ } /*end of table */
};
MODULE_DEVICE_TABLE(of, vpusmm_of_match);
static struct platform_driver vpusmm_platform_driver = {
	.probe  = vpusmm_probe,
	.remove = vpusmm_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = vpusmm_of_match,
	},
};

static void vpusmm_cleanup(void)
{
	if (vpusmm_platform_dev) {
		platform_device_unregister(vpusmm_platform_dev);
		vpusmm_platform_dev = NULL;
	}

	// this will trigger device's platform_driver.remove() callback
	if (vpusmm_platform_driver_registered)
		platform_driver_unregister(&vpusmm_platform_driver);

	if (vpusmm_device_number) {
		unregister_chrdev_region(vpusmm_device_number, 8);
		vpusmm_device_number = 0;
	}

	if (vpusmm_sys_class) {
		class_destroy(vpusmm_sys_class);
		vpusmm_sys_class = NULL;
	}
}

static int __init vpusmm_init(void)
{
	int retval = 0;
	struct device_node *np = NULL;
	const struct platform_device_info pl_info_keembay = {
		.name		= DRIVER_NAME,
		.id		  = 0,
		.dma_mask	= DMA_BIT_MASK(32),
	};

	vpusmm_device_number = 0;
	vpusmm_sys_class = NULL;
	vpusmm_platform_driver_registered = 0;
	vpusmm_platform_dev = NULL;

	retval = alloc_chrdev_region(&vpusmm_device_number, 0, 8, DRIVER_NAME);
	if (retval < 0) {
		pr_err("[%s]: couldn't allocate device major number, return=%d\n", DRIVER_NAME, retval);
		vpusmm_device_number = 0;
		goto failed;
	}

	vpusmm_sys_class = class_create(THIS_MODULE, DRIVER_NAME"_class");
	if (IS_ERR_OR_NULL(vpusmm_sys_class)) {
		retval = PTR_ERR(vpusmm_sys_class);
		vpusmm_sys_class = NULL;
		pr_err("[%s]: couldn't create driver class, return=%d\n", DRIVER_NAME, retval);
		retval = (retval == 0) ? -ENOMEM : retval;
		goto failed;
	}

	np = of_find_compatible_node(NULL, NULL, vpusmm_of_match[0].compatible);
	if (np) {
		of_node_put(np);
	} else {
		// There is no vpusmm node in device tree, create platform device
		vpusmm_platform_dev = platform_device_register_full(&pl_info_keembay);
		if (IS_ERR_OR_NULL(vpusmm_platform_dev)) {
			retval = PTR_ERR(vpusmm_platform_dev);
			vpusmm_platform_dev   = NULL;
			pr_err("platform_device_register_full(%s) failed. return=%d\n", DRIVER_NAME, retval);
			goto failed;
		}
		pr_info("%s: device tree node not found, create a default device\n", __func__);
	}

	retval = platform_driver_register(&vpusmm_platform_driver);
	if (retval) {
		pr_err("%s: couldn't register platform driver. return=%d\n", DRIVER_NAME, retval);
		vpusmm_platform_driver_registered = 0;
		goto failed;
	} else
		vpusmm_platform_driver_registered = 1;

	return 0;
failed:
	vpusmm_cleanup();
	return retval;
}
static void vpusmm_exit(void)
{
	vpusmm_cleanup();
}

module_init(vpusmm_init)
module_exit(vpusmm_exit)

MODULE_DESCRIPTION("KeemBay VPU Shared memory management Driver");
MODULE_AUTHOR("Li, Tingqian <tingqian.li@intel.com>");
MODULE_LICENSE("GPL v2");
