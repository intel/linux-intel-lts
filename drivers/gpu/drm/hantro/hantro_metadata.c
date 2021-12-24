// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro Metadata file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_metadata.h"
#include "hantro_priv.h"

static int hantro_query_metadata(struct drm_device *dev, void *data,
				 struct drm_file *file_priv)
{
	struct hantro_metainfo_params *metadata_info_p =
		(struct hantro_metainfo_params *)data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj = NULL;

	obj = hantro_gem_object_lookup(dev, file_priv, metadata_info_p->handle);
	if (!obj)
		return -ENOENT;

	cma_obj = to_drm_gem_hantro_obj(obj);
	memcpy(&metadata_info_p->info, &cma_obj->dmapriv.meta_data_info,
	       sizeof(const struct viv_vidmem_metadata));
	hantro_unref_drmobj(obj);
	return 0;
}

static int hantro_update_metadata(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct hantro_metainfo_params *metadata_info_p =
		(struct hantro_metainfo_params *)data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj = NULL;

	obj = hantro_gem_object_lookup(dev, file_priv, metadata_info_p->handle);
	if (!obj)
		return -ENOENT;

	cma_obj = to_drm_gem_hantro_obj(obj);
	memcpy(&cma_obj->dmapriv.meta_data_info, &metadata_info_p->info,
	       sizeof(const struct viv_vidmem_metadata));
	hantro_unref_drmobj(obj);
	return 0;
}

long hantrometadata_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = file->private_data;
	struct drm_device *dev = hantro_drm.drm_dev;
	char stack_kdata[256];
	char *kdata = stack_kdata;
	int ret;

	switch (cmd) {
	case DRM_IOCTL_HANTRO_QUERY_METADATA:
		if (copy_from_user(kdata, (void __user *)arg,
				   sizeof(struct hantro_metainfo_params)) != 0)
			return -EFAULT;

		ret = hantro_query_metadata(dev, kdata, file_priv);

		if (ret < 0)
			return -EFAULT;

		if (copy_to_user((void __user *)arg, kdata,
				 sizeof(struct hantro_metainfo_params)) != 0)
			return -EFAULT;

		break;
	case DRM_IOCTL_HANTRO_UPDATE_METADATA:
		if (copy_from_user(kdata, (void __user *)arg,
				   sizeof(struct hantro_metainfo_params)) != 0)
			return -EFAULT;

		ret = hantro_update_metadata(dev, kdata, file_priv);

		if (ret < 0)
			return -EFAULT;

		if (copy_to_user((void __user *)arg, kdata,
				 sizeof(struct hantro_metainfo_params)) != 0)
			return -EFAULT;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}
