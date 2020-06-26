// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera ISP Configuration Video Node Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "kmb-isp-config.h"

#define KMB_ISP_CONFIG_NAME "kmb-camera-isp-config"

/**
 * kmb_isp_config_queue_setup - Setup queue
 * @q: pointer to video buffer queue
 * @num_buffers: pointer to number of buffers
 * @num_planes: pointer to number of planes
 * @sizes: array of image sizes
 * @alloc_devs: pointer to array of devices
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct kmb_isp_cfg_fh *cfg_fh = vb2_get_drv_priv(q);

	*num_planes = 1;
	sizes[0] = cfg_fh->data_size;
	return 0;
}

/**
 * kmb_isp_config_buffer_prepare - Buffer prepare
 * @vb: pointer to video buffer
 *
 * Called every time the buffer is queued perform to initialization required
 * before each hardware operation
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_buffer_prepare(struct vb2_buffer *vb)
{
	struct kmb_isp_cfg_fh *cfg_fh = vb2_get_drv_priv(vb->vb2_queue);

	vb2_set_plane_payload(vb, 0, cfg_fh->data_size);
	return 0;
}

/**
 * kmb_isp_config_buf_init - Buffer initialize
 * @vb: pointer to video buffer
 *
 * Called once after allocating a buffer for additional buffer-related
 * initialization
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_buf_init(struct vb2_buffer *vb)
{
	return 0;
}

/**
 * kmb_isp_config_buf_queue - Pass buffer vb to the driver
 * @vb: pointer to video buffer
 */
static void kmb_isp_config_buf_queue(struct vb2_buffer *vb)
{
	int ret;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct kmb_isp_cfg_data *buf = (struct kmb_isp_cfg_data *)vbuf;
	struct kmb_isp_cfg_fh *cfg_fh = vb2_get_drv_priv(vb->vb2_queue);
	struct kmb_isp_config *kmb_cfg = cfg_fh->kmb_cfg;

	buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	buf->addr += 0x60;

	ret = kmb_cfg->queue_ops.queue(kmb_cfg->priv, buf);
	if (ret) {
		dev_err(&kmb_cfg->video_out->dev,
				"Fail isp configuration queue ret %d\n", ret);
		kmb_cfg->queue_ops.flush(kmb_cfg->priv);
		return;
	}
}

/**
 * kmb_isp_config_start_streaming - Set streaming state to enable
 * @q: pointer to videobuf queue
 * @count: (unused)number of buffers owned by the driver
 *
 * Return: 0 if successful, -EPIPE - broken pipe
 */
static int kmb_isp_config_start_streaming(struct vb2_queue *q,
		unsigned int count)
{
	return 0;
}

/**
 * kmb_isp_config_stop_streaming - Set streaming state to disable
 * @q: pointer to videobuf queue
 */
static void kmb_isp_config_stop_streaming(struct vb2_queue *q)
{
	struct kmb_isp_cfg_fh *cfg_fh = vb2_get_drv_priv(q);
	struct kmb_isp_config *kmb_cfg = cfg_fh->kmb_cfg;

	kmb_cfg->queue_ops.flush(kmb_cfg->priv);
}

/* driver-specific operations */
static struct vb2_ops kmb_video_vb2_q_ops = {
	.queue_setup     = kmb_isp_config_queue_setup,
	.buf_prepare     = kmb_isp_config_buffer_prepare,
	.buf_init        = kmb_isp_config_buf_init,
	.buf_queue       = kmb_isp_config_buf_queue,
	.start_streaming = kmb_isp_config_start_streaming,
	.stop_streaming  = kmb_isp_config_stop_streaming,
};

/**
 * kmb_isp_config_querycap - Query device capabilities
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @cap: pointer to V4L2 device capabilities structure
 *
 * VIDIOC_QUERYCAP ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_querycap(struct file *file,
	void *fh, struct v4l2_capability *cap)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);
	cap->bus_info[0] = 0;
	strlcpy(cap->driver, KMB_ISP_CONFIG_NAME, sizeof(cap->driver));
	strlcpy(cap->card, KMB_ISP_CONFIG_NAME, sizeof(cap->card));
	cap->capabilities = cfg_fh->fh.vdev->device_caps |
			    V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = cfg_fh->fh.vdev->device_caps;

	return 0;
}

/**
 * kmb_isp_config_get_fmt - Get format for video capture in multiple plain mode
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @f: pointer to stream data format
 *
 * VIDIOC_G_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_get_fmt(struct file *file,
		void *fh, struct v4l2_format *f)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	f->fmt.pix.width = cfg_fh->data_size;
	f->fmt.pix.height = 1;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = f->fmt.pix.width;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

/**
 * kmb_isp_config_set_fmt - Set format for video capture in multiple plain mode
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @f: pointer to stream data format
 *
 * VIDIOC_S_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_set_fmt(struct file *file,
	void *fh, struct v4l2_format *f)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	f->fmt.pix.bytesperline = f->fmt.pix.width;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
	cfg_fh->data_size = f->fmt.pix.sizeimage;

	return 0;
}

/**
 * kmb_isp_config_reqbufs - Initiate memory mapped or user pointer I/O
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @req: pointer to request buffers structure
 *
 * VIDIOC_REQBUFS ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_reqbufs(struct file *file,
	void *fh, struct v4l2_requestbuffers *req)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);

	return vb2_reqbufs(&cfg_fh->vb2_q, req);
}

/**
 * kmb_isp_config_query_buf - Query the status of a buffer
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @pb: pointer to video buffer information
 *
 * VIDIOC_QUERYBUF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_query_buf(struct file *file, void *fh,
		struct v4l2_buffer *pb)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);

	return vb2_querybuf(&cfg_fh->vb2_q, pb);
}

/**
 * kmb_isp_config_qbuf - Queue buffer
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @pb: pointer to video buffer information
 *
 * VIDIOC_QBUF ioctl handler. Enqueue an empty (capturing) or filled (output)
 * buffer in the driver's incoming queue.
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_qbuf(struct file *file, void *fh,
	struct v4l2_buffer *pb)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);
	struct media_device *mdev = cfg_fh->fh.vdev->v4l2_dev->mdev;

	return vb2_qbuf(&cfg_fh->vb2_q, mdev, pb);
}

/**
 * kmb_video_dqbuf - Dequeue a filled buffer from the driver's outgoing queue
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @pb: pointer to video buffer information
 *
 * VIDIOC_DQBUF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_dqbuf(struct file *file, void *fh,
	struct v4l2_buffer *pb)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);

	return vb2_dqbuf(&cfg_fh->vb2_q, pb, file->f_flags & O_NONBLOCK);
}

/**
 * kmb_isp_config_streamon - Start streaming
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @buf_type: V4L2 buffer type
 *
 * VIDIOC_STREAMON ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_streamon(struct file *file,
	void *fh, enum v4l2_buf_type buf_type)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);
	int ret;

	ret = vb2_streamon(&cfg_fh->vb2_q, buf_type);
	if (ret < 0) {
		dev_err(&cfg_fh->fh.vdev->dev,
			"Stream ON failed! ret = %d\n", ret);
		return ret;
	}

	return 0;
}

/**
 * kmb_isp_config_streamoff - Stop streaming
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @buf_type: V4L2 buffer type
 *
 * VIDIOC_STREAMOFF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_streamoff(struct file *file,
	void *fh, enum v4l2_buf_type buf_type)
{
	struct kmb_isp_cfg_fh *cfg_fh =
			container_of(fh, struct kmb_isp_cfg_fh, fh);

	return vb2_streamoff(&cfg_fh->vb2_q, buf_type);
}

/* V4L2 ioctl operations */
static const struct v4l2_ioctl_ops kmb_vid_ioctl_ops = {
	.vidioc_querycap      = kmb_isp_config_querycap,
	.vidioc_g_fmt_vid_out = kmb_isp_config_get_fmt,
	.vidioc_s_fmt_vid_out = kmb_isp_config_set_fmt,
	.vidioc_reqbufs       = kmb_isp_config_reqbufs,
	.vidioc_querybuf      = kmb_isp_config_query_buf,
	.vidioc_qbuf          = kmb_isp_config_qbuf,
	.vidioc_dqbuf         = kmb_isp_config_dqbuf,
	.vidioc_streamon      = kmb_isp_config_streamon,
	.vidioc_streamoff     = kmb_isp_config_streamoff,
};

/**
 * kmb_isp_config_open - Device node open
 * @file: pointer to file containing video device data
 *
 * Return: 0 if successful, -ENOMEM if there is not enough memory
 */
static int kmb_isp_config_open(struct file *file)
{
	int ret;
	struct kmb_isp_cfg_fh *cfg_fh;
	struct kmb_isp_config *kmb_out = video_drvdata(file);
	struct video_device *output = video_devdata(file);

	cfg_fh = kzalloc(sizeof(*cfg_fh), GFP_KERNEL);
	if (!cfg_fh)
		return -ENOMEM;

	mutex_init(&cfg_fh->lock);
	v4l2_fh_init(&cfg_fh->fh, output);

	file->private_data = &cfg_fh->fh;
	v4l2_fh_add(&cfg_fh->fh);

	cfg_fh->kmb_cfg = kmb_out;
	cfg_fh->vb2_q.drv_priv = cfg_fh;
	cfg_fh->vb2_q.ops = &kmb_video_vb2_q_ops;
	cfg_fh->vb2_q.buf_struct_size = sizeof(struct kmb_isp_cfg_data);

	cfg_fh->vb2_q.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	cfg_fh->vb2_q.io_modes = VB2_DMABUF | VB2_MMAP;
	cfg_fh->vb2_q.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	cfg_fh->vb2_q.mem_ops = &vb2_dma_contig_memops;
	cfg_fh->vb2_q.dev = kmb_out->dma_dev;
	cfg_fh->vb2_q.lock = &cfg_fh->lock;

	ret = vb2_queue_init(&cfg_fh->vb2_q);
	if (ret < 0) {
		dev_err(&output->dev, "Error vb2 queue init\n");
		goto error_release_v4l2_fh;
	}

	/* default size */
	cfg_fh->data_size = 8192;

	return 0;

error_release_v4l2_fh:
	v4l2_fh_del(&cfg_fh->fh);
	v4l2_fh_exit(&cfg_fh->fh);
	kfree(cfg_fh);
	return ret;
}

/**
 * kmb_isp_config_release - Video device release
 * @file: pointer to file containing video device data
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_release(struct file *file)
{
	struct kmb_isp_cfg_fh *cfg_fh = container_of(
			file->private_data, struct kmb_isp_cfg_fh, fh);

	vb2_queue_release(&cfg_fh->vb2_q);

	v4l2_fh_del(&cfg_fh->fh);
	v4l2_fh_exit(&cfg_fh->fh);

	mutex_destroy(&cfg_fh->lock);

	kfree(cfg_fh);
	return 0;
}

/**
 * kmb_isp_config_poll - Poll
 * @file: pointer to file containing video device data
 * @wait: poll table wait
 *
 * Return: 0 if successful
 */
static unsigned int kmb_isp_config_poll(struct file *file,
	struct poll_table_struct *wait)
{
	struct kmb_isp_cfg_fh *cfg_fh = container_of(
			file->private_data, struct kmb_isp_cfg_fh, fh);

	return vb2_poll(&cfg_fh->vb2_q, file, wait);
}

/**
 * kmb_isp_config_mmap - Memory map
 * @file: pointer to file containing video device data
 * @vma: VMM memory area
 *
 * Return: 0 if successful
 */
static int kmb_isp_config_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct kmb_isp_cfg_fh *cfg_fh = container_of(
			file->private_data, struct kmb_isp_cfg_fh, fh);

	return vb2_mmap(&cfg_fh->vb2_q, vma);
}

/* FS operations for V4L2 device */
static const struct v4l2_file_operations kmb_vid_output_fops = {
	.owner          = THIS_MODULE,
	.open           = kmb_isp_config_open,
	.release        = kmb_isp_config_release,
	.poll           = kmb_isp_config_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = kmb_isp_config_mmap
};

/**
 * kmb_video_init - Initialize entity
 * @kmb_out: pointer to kmb isp config device
 *
 * Return: 0 if successful
 */
int kmb_isp_config_init(struct kmb_isp_config *kmb_out)
{
	int ret;

	kmb_out->video_out = video_device_alloc();
	if (!kmb_out->video_out) {
		dev_err(&kmb_out->video_out->dev,
			"Failed to allocate video device\n");
		return -ENOMEM;
	}

	mutex_init(&kmb_out->lock);

	kmb_out->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(
			&kmb_out->video_out->entity, 1, &kmb_out->pad);
	if (ret < 0)
		return ret;

	kmb_out->video_out->device_caps = V4L2_CAP_VIDEO_OUTPUT |
					  V4L2_CAP_STREAMING;
	kmb_out->video_out->fops  = &kmb_vid_output_fops;
	kmb_out->video_out->ioctl_ops = &kmb_vid_ioctl_ops;
	kmb_out->video_out->minor = -1;
	kmb_out->video_out->release  = video_device_release;
	kmb_out->video_out->vfl_type = VFL_TYPE_GRABBER;
	kmb_out->video_out->vfl_dir = VFL_DIR_TX;
	kmb_out->video_out->lock = &kmb_out->lock;
	snprintf(kmb_out->video_out->name, sizeof(kmb_out->video_out->name),
			KMB_ISP_CONFIG_NAME);

	video_set_drvdata(kmb_out->video_out, kmb_out);
	return 0;
}

/**
 * kmb_isp_config_deinit - Free resources associated with entity
 * @kmb_out: pointer to kmb isp config device
 */
void kmb_isp_config_deinit(struct kmb_isp_config *kmb_out)
{
	media_entity_cleanup(&kmb_out->video_out->entity);
	mutex_destroy(&kmb_out->lock);
}

/**
 * kmb_isp_config_register - Register V4L2 device
 * @kmb_out: pointer to kmb isp config device
 * @v4l2_dev: pointer to V4L2 device drivers
 *
 * Return: 0 if successful
 */
int kmb_isp_config_register(struct kmb_isp_config *kmb_out,
		       struct v4l2_device *v4l2_dev)
{
	int ret;

	kmb_out->video_out->v4l2_dev = v4l2_dev;
	ret = video_register_device(kmb_out->video_out, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register video device\n");
		return ret;
	}

	return 0;
}

/**
 * kmb_isp_config_unregister - Unregister V4L device
 * @kmb_out: pointer to kmb isp config device
 */
void kmb_isp_config_unregister(struct kmb_isp_config *kmb_out)
{
	kmb_isp_config_deinit(kmb_out);
	mutex_destroy(&kmb_out->lock);
	video_unregister_device(kmb_out->video_out);
}
