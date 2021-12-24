// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro driver fs operation.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"
#include "hantro_dec.h"
#include "hantro_enc.h"

static ssize_t bandwidth_dec_read_show(struct device *kdev,
				       struct device_attribute *attr, char *buf)
{
	/*
	 * sys/bus/platform/drivers/hantro/xxxxx.vpu/bandwidth_dec_read
	 * used to show bandwidth info to user space
	 * all core' bandwidth might be exported in same string.
	 * data is just an example. Real data should be read from HW registers
	 * this file is read only.
	 */
	u32 bandwidth;
	struct device_info *pdevinfo;

	pdevinfo = dev_get_drvdata(kdev);
	if (!pdevinfo)
		return 0;

	bandwidth = hantrodec_read_bandwidth(pdevinfo, 1);
	return snprintf(buf, PAGE_SIZE, "%u\n", bandwidth);
}

static ssize_t bandwidth_dec_write_show(struct device *kdev,
					struct device_attribute *attr,
					char *buf)
{
	u32 bandwidth;
	struct device_info *pdevinfo;

	pdevinfo = dev_get_drvdata(kdev);
	if (!pdevinfo)
		return 0;

	bandwidth = hantrodec_read_bandwidth(pdevinfo, 0);
	return snprintf(buf, PAGE_SIZE, "%u\n", bandwidth);
}

static ssize_t bandwidth_enc_read_show(struct device *kdev,
				       struct device_attribute *attr, char *buf)
{
	u32 bandwidth;
	struct device_info *pdevinfo;

	pdevinfo = dev_get_drvdata(kdev);
	if (!pdevinfo)
		return 0;

	bandwidth = hantroenc_read_bandwidth(pdevinfo, 1);
	return snprintf(buf, PAGE_SIZE, "%u\n", bandwidth);
}

static ssize_t bandwidth_enc_write_show(struct device *kdev,
					struct device_attribute *attr,
					char *buf)
{
	u32 bandwidth;
	struct device_info *pdevinfo;

	pdevinfo = dev_get_drvdata(kdev);
	if (!pdevinfo)
		return 0;

	bandwidth = hantroenc_read_bandwidth(pdevinfo, 0);
	return snprintf(buf, PAGE_SIZE, "%u\n", bandwidth);
}

static ssize_t clients_show(struct device *kdev, struct device_attribute *attr,
			    char *buf)
{
	struct device_info *pdevinfo = dev_get_drvdata(kdev);
	int handle, buf_used = 0, client_count = 0;
	bool noprint = false;
	struct hantro_client *client;
	int deviceidx = -1;
	static char optype[64];
	static char profile[64];
	static const char *const unknown = "Unknown";
	static const char *const optypes[] = { "Decode", "Encode" };
	static const char *const profiles[] = {
		/* brief Profile ID used for video processing. */
		"VAProfileMPEG2Simple",
		"VAProfileMPEG2Main",
		"VAProfileMPEG4Simple",
		"VAProfileMPEG4AdvancedSimple",
		"VAProfileMPEG4Main",
		"VAProfileH264Baseline",
		"VAProfileH264Main",
		"VAProfileH264High",
		"VAProfileVC1Simple",
		"VAProfileVC1Main",
		"VAProfileVC1Advanced",
		"VAProfileH263Baseline",
		"VAProfileJPEGBaseline",
		"VAProfileH264ConstrainedBaseline",
		"VAProfileVP8Version0_3",
		"VAProfileH264MultiviewHigh",
		"VAProfileH264StereoHigh",
		"VAProfileHEVCMain",
		"VAProfileHEVCMain10",
		"VAProfileVP9Profile0",
		"VAProfileVP9Profile1",
		"VAProfileVP9Profile2",
		"VAProfileVP9Profile3"
	};
	/* brief Profile ID used for video processing. */
	static const char * const hantro_profiles[] = {
		"HANTROProfileHEVCMSt",
		"HANTROProfileH264H10"
	};

	if (!pdevinfo)
		return 0;

	deviceidx = pdevinfo->deviceid;
	buf_used += snprintf(buf + buf_used, PAGE_SIZE,
			     "File Id : ContextId :  Operation   :                Codec                    :  Resolution\n");
	/* Go through all open drm files */
	mutex_lock(&pdevinfo->alloc_mutex);
	idr_for_each_entry(&pdevinfo->clients, client, handle) {
		if (client && client->deviceid == deviceidx) {
			if (buf_used < (PAGE_SIZE - 200)) {
				if (client->profile >= 0 &&
				    client->profile <= 22) {
					strncpy(profile,
						profiles[client->profile],
						strlen(profiles[client->profile]) +
							1);

				} else {
					if (client->profile >= 100 &&
					    client->profile <= 101)
						strncpy(profile,
							hantro_profiles
								[client->profile -
								 100],
							strlen(hantro_profiles
								       [client->profile -
									100]) +
								1);
					else
						strncpy(profile, unknown,
							strlen(unknown) + 1);
				}

				if (client->codec == 0 || client->codec == 1)
					strncpy(optype, optypes[client->codec],
						strlen(optypes[client->codec]) + 1);
				else
					strncpy(optype, unknown,
						strlen(unknown) + 1);

				buf_used += snprintf(buf + buf_used, PAGE_SIZE,
					"%8p %9x     %s (%d)     %-32s (%2d)     %lux%lu\n",
					client->file, client->clientid, optype,
					client->codec, profile, client->profile,
					client->width, client->height);
			} else {
				/*  optimization to save buf space due to a PAGE_SIZE mem only */
				if (!noprint) {
					buf_used +=
						snprintf(buf + buf_used,
							 PAGE_SIZE, " ....\n");
					noprint =
						true; /* print ... only one time */
				}
			}

			client_count++;
		}
	}
	mutex_unlock(&pdevinfo->alloc_mutex);
	buf_used += snprintf(buf + buf_used, PAGE_SIZE, "\n%d clients\n\n",
			     client_count);
	return buf_used;
}

int mem_usage_internal(unsigned int deviceidx, struct device *memdev,
		       u32 *pused_mem, u32 *pallocations, struct seq_file *s)
{
	struct drm_gem_hantro_object *cma_obj;
	struct device_info *pdevinfo;
	int alloc_count = 0, handle;
	ssize_t mem_used = 0;

	pdevinfo = get_device_info(deviceidx);
	if (!pdevinfo)
		return -EINVAL;

	if (s)
		seq_puts(s,
			 "Physical Addr    :  Virtual Addr       :   Size  : GEM handle : DMABuf fd\n");

	mutex_lock(&pdevinfo->alloc_mutex);
	/* Iterate through cma objects added to file's driver_priv */
	/* checkout hantro_record_mem */
	idr_for_each_entry(&pdevinfo->allocations, cma_obj, handle) {
		if (cma_obj && cma_obj->memdev == memdev) {
			if (s) {
				seq_printf(s,
					   " 0x%-13llx :  0x%-15p  : %6ldK : %4d  [%2d] :   ",
					   cma_obj->paddr, cma_obj->vaddr,
					   cma_obj->base.size / 1024,
					   cma_obj->handle,
					   kref_read(&cma_obj->base.refcount));
				if (cma_obj->fd >= 0)
					seq_printf(s, "%d\n", cma_obj->fd);
				else
					seq_puts(s, "\n");
			}

			mem_used += cma_obj->base.size;
			alloc_count++;
		}
	}

	mutex_unlock(&pdevinfo->alloc_mutex);
	if (s)
		seq_printf(s, "\n%ldK in %d allocations\n\n", mem_used / 1024,
			   alloc_count);

	if (pused_mem)
		*pused_mem = mem_used;

	if (pallocations)
		*pallocations = alloc_count;

	return 0;
}

/* print mem usage summary through sysfs */
static ssize_t mem_usage_show(struct device *kdev, struct device_attribute *attr,
			      char *buf)
{
	struct device_info *pdevinfo = dev_get_drvdata(kdev);
	int deviceidx, bufsize = 0;

	int used_mem0 = 0, alloc_count0 = 0;
	int used_mem1 = 0, alloc_count1 = 0;

	if (!pdevinfo)
		return 0;

	deviceidx = pdevinfo->deviceid;
	mem_usage_internal(deviceidx, pdevinfo->dev, &used_mem0, &alloc_count0,
			   NULL);
	if (pdevinfo->codec_rsvmem)
		mem_usage_internal(deviceidx, pdevinfo->codec_rsvmem, &used_mem1,
				   &alloc_count1, NULL);

	bufsize += snprintf(buf + bufsize, PAGE_SIZE, "Device %d mem usage:\n",
			    deviceidx);
	bufsize += snprintf(buf + bufsize, PAGE_SIZE,
		"\tCMA 0 : %lld MB - [0x%llx - 0x%llx]\n\t\t %dK in %d allocations\n\n",
		resource_size(&pdevinfo->mem_res[0]) / (1024 * 1024),
		pdevinfo->mem_res[0].start, pdevinfo->mem_res[0].end,
		used_mem0 / 1024, alloc_count0);
	bufsize += snprintf(buf + bufsize, PAGE_SIZE,
		"\tCMA 1 : %lld MB - [0x%llx - 0x%llx]\n\t\t %dK in %d allocations\n",
		resource_size(&pdevinfo->mem_res[1]) / (1024 * 1024),
		pdevinfo->mem_res[1].start, pdevinfo->mem_res[1].end,
		used_mem1 / 1024, alloc_count1);
	return bufsize;
}

static void reset_perf_data(void)
{
	struct device_info *pdevinfo;
	struct hantrodec_t *dechdr;
	struct hantroenc_t *enchdr;
	int i, devcnt;

	devcnt = get_device_count();
	for (i = 0; i < devcnt; i++) {
		pdevinfo = get_device_info(i);
		dechdr = pdevinfo->dechdr;
		while (dechdr) {
			dechdr->perf_data.count = 0;
			dechdr->perf_data.totaltime = 0;
			dechdr->perf_data.hwcycles = 0;
			dechdr = dechdr->next;
		}

		enchdr = pdevinfo->enchdr;
		while (enchdr) {
			enchdr->perf_data.count = 0;
			enchdr->perf_data.totaltime = 0;
			enchdr->perf_data.hwcycles = 0;
			enchdr = enchdr->next;
		}
	}
}

static ssize_t fps_show(struct device *kdev, struct device_attribute *attr,
			char *buf)
{
	struct device_info *pdevinfo;
	struct hantrodec_t *dechdr;
	struct hantroenc_t *enchdr;
	int buf_size = 0, i, diff, devcnt, core;
	u64 start, averagecycles, totaltime_hw, totaltime_sw;
	u32 fps, totalfps;
	long clk_freq;

	devcnt = get_device_count();
	reset_perf_data();
	start = sched_clock();
	/* wait for 1 sec */
	msleep(1000);
	diff = (sched_clock() - start) / 1000000; /* diff in ms */
	for (i = 0; i < devcnt; i++) {
		pdevinfo = get_device_info(i);
		buf_size +=
			snprintf(buf + buf_size, PAGE_SIZE, "Device %d:\n", i);
		dechdr = pdevinfo->dechdr;
		core = 0;
		totalfps = 0;
		buf_size += snprintf(buf + buf_size, PAGE_SIZE, "\tDecode\n");
		while (dechdr) {
			fps = 0;
			clk_freq = clk_get_rate(dechdr->dev_clk);
			if (dechdr->perf_data.count == 0) {
				buf_size += snprintf(buf + buf_size, PAGE_SIZE,
					"\t\tCore [%d]    0 fps, %ld Mhz\n",
					core, clk_freq / 1000000);
			} else {
				averagecycles = dechdr->perf_data.hwcycles /
						dechdr->perf_data.count;
				totaltime_hw = dechdr->perf_data.hwcycles /
					       (dechdr->clk_freq / 100000);
				totaltime_sw =
					dechdr->perf_data.totaltime / 10000;
				fps = (dechdr->perf_data.count * 1000) /
				      (diff == 0 ? 1 : diff);
				buf_size += snprintf(buf + buf_size, PAGE_SIZE,
					"\t\tCore [%d] %4d fps (%lld%%:%lld%%), %-8lld avg cycles, %ld Mhz\n",
					core, fps, totaltime_hw / diff,
					totaltime_sw / diff, averagecycles,
					clk_freq / 1000000);
				totalfps += fps;
			}

			dechdr = dechdr->next;
			core++;
		}
		buf_size += snprintf(buf + buf_size, PAGE_SIZE, "\t%d fps\n\n",
				     totalfps);

		core = 0;
		totalfps = 0;
		buf_size += snprintf(buf + buf_size, PAGE_SIZE, "\tEncode\n");
		enchdr = pdevinfo->enchdr;
		while (enchdr) {
			fps = 0;
			clk_freq = clk_get_rate(enchdr->dev_clk);
			if (clk_freq != 0) {
				if (enchdr->perf_data.count == 0) {
					buf_size += snprintf(buf + buf_size, PAGE_SIZE,
						"\t\tCore [%d]    0 fps, %ld Mhz\n",
						core, clk_freq / 1000000);
				} else {
					totaltime_sw =
						enchdr->perf_data.totaltime /
						10000;
					fps = (enchdr->perf_data.count * 1000) /
					      (diff == 0 ? 1 : diff);
					buf_size += snprintf(buf + buf_size, PAGE_SIZE,
						"\t\tCore [%d] %4d fps (%lld%%), %lu Mhz\n",
						core, fps, totaltime_sw / diff,
						enchdr->clk_freq / 1000000);
					totalfps += fps;
				}
			}

			enchdr = enchdr->next;
			core++;
		}

		buf_size += snprintf(buf + buf_size, PAGE_SIZE, "\t%d fps\n\n",
				     totalfps);
	}
	return buf_size;
}

static ssize_t version_show(struct device *kdev, struct device_attribute *attr,
			    char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"Hantro driver info:\n\tversion: %d.%d\n\t   Date: %s\n",
		DRIVER_MAJOR, DRIVER_MINOR, DRIVER_DATE);
}

static DEVICE_ATTR_RO(bandwidth_dec_read);
static DEVICE_ATTR_RO(bandwidth_dec_write);
static DEVICE_ATTR_RO(bandwidth_enc_read);
static DEVICE_ATTR_RO(bandwidth_enc_write);
static DEVICE_ATTR_RO(clients);
static DEVICE_ATTR_RO(mem_usage);
static DEVICE_ATTR_RO(fps);
static DEVICE_ATTR_RO(version);

static struct attribute *hantro_attrs[] = {
	&dev_attr_bandwidth_dec_read.attr,
	&dev_attr_bandwidth_dec_write.attr,
	&dev_attr_bandwidth_enc_read.attr,
	&dev_attr_bandwidth_enc_write.attr,
	&dev_attr_clients.attr,
	&dev_attr_mem_usage.attr,
	&dev_attr_fps.attr,
	&dev_attr_version.attr,
	NULL,
};

static const struct attribute_group hantro_attr_group = {
	.attrs = hantro_attrs,
};

/* print mem_usage details through debugfs */
static int mem_usage_debugfs_show(struct seq_file *s, void *v)
{
	struct device_info *pdevinfo = s->private;
	int deviceidx;

	if (!pdevinfo)
		return 0;

	deviceidx = pdevinfo->deviceid;
	seq_printf(s, "Memory usage for device %d:\n", deviceidx);
	seq_printf(s, "Pixel CMA: %lld MB - [0x%llx - 0x%llx]\n",
		   resource_size(&pdevinfo->mem_res[0]) / (1024 * 1024),
		   pdevinfo->mem_res[0].start, pdevinfo->mem_res[0].end);
	mem_usage_internal(deviceidx, pdevinfo->dev, NULL, NULL, s);
	if (pdevinfo->codec_rsvmem) {
		seq_printf(s, "Codec CMA: %lld MB - [0x%llx - 0x%llx]\n",
			   resource_size(&pdevinfo->mem_res[1]) / (1024 * 1024),
			   pdevinfo->mem_res[1].start, pdevinfo->mem_res[1].end);
		mem_usage_internal(deviceidx, pdevinfo->codec_rsvmem, NULL, NULL,
				   s);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mem_usage_debugfs);

int create_sysfs(struct device_info *pdevinfo)
{
	return devm_device_add_group(pdevinfo->dev, &hantro_attr_group);
}

void remove_sysfs(struct device_info *pdevinfo)
{
	devm_device_remove_group(pdevinfo->dev, &hantro_attr_group);
}

void create_debugfs(struct device_info *pdevinfo, bool has_codecmem)
{
	char filename[64];

	if (!hantro_drm.debugfs_root)
		return;

	sprintf(filename, "mem_usage%d", pdevinfo->deviceid);
	debugfs_create_file(filename, S_IFREG | 0444, hantro_drm.debugfs_root,
			    pdevinfo, &mem_usage_debugfs_fops);
}
