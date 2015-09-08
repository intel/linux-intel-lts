/*
 *  skl-debug.c - Debugfs for skl driver
 *
 *  Copyright (C) 2015 Intel Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/pci.h>
#include <linux/debugfs.h>
#include <sound/soc.h>
#include "skl.h"
#include "skl-nhlt.h"
#include "skl-tplg-interface.h"
#include "skl-topology.h"

#define MAX_SSP 4

struct nhlt_blob {
	size_t size;
	struct nhlt_specific_cfg *cfg;
};

struct skl_debug {
	struct skl *skl;
	struct device *dev;

	struct dentry *fs;
	struct dentry *nhlt;
	struct nhlt_blob ssp_blob[MAX_SSP];
	struct nhlt_blob dmic_blob;
	struct dentry *modules;
};

struct nhlt_specific_cfg
*skl_nhlt_get_debugfs_blob(struct skl_debug *d, u8 link_type, u32 instance)
{
	switch (link_type) {
	case NHLT_LINK_DMIC:
		return d->dmic_blob.cfg;

	case NHLT_LINK_SSP:
		if (instance >= MAX_SSP)
			return NULL;

		return d->ssp_blob[instance].cfg;

	default:
		break;
	}

	dev_err(d->dev, "NHLT debugfs query failed\n");
	return NULL;
}

static ssize_t nhlt_read(struct file *file, char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct nhlt_blob *blob = file->private_data;

	if (!blob->cfg)
		return -EIO;

	return simple_read_from_buffer(user_buf, count, ppos,
			blob->cfg, blob->size);
}

static ssize_t nhlt_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct nhlt_blob *blob = file->private_data;
	struct nhlt_specific_cfg *new_cfg;
	ssize_t written;
	size_t size = blob->size;

	if (count > 2 * HDA_SST_CFG_MAX)
		return -EIO;

	if (!blob->cfg) {
		/* allocate mem for blob */
		blob->cfg = kzalloc(count, GFP_KERNEL);
		if (!blob->cfg)
			return -ENOMEM;
		size = count;
	} else if (blob->size < count) {
		/* size if different, so relloc */
		new_cfg = krealloc(blob->cfg, count, GFP_KERNEL);
		if (!new_cfg)
			return -ENOMEM;
		size = count;
		blob->cfg = new_cfg;
	}

	written = simple_write_to_buffer(blob->cfg, size, ppos,
						user_buf, count);
	blob->size = written;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	print_hex_dump(KERN_DEBUG, "Debugfs Blob:", DUMP_PREFIX_OFFSET, 8, 4,
			blob->cfg, blob->size, false);

	return written;
}

static const struct file_operations nhlt_fops = {
	.open = simple_open,
	.read = nhlt_read,
	.write = nhlt_write,
	.llseek = default_llseek,
};

static ssize_t nhlt_control_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char *state;

	state = d->skl->nhlt_override ? "enable\n" : "disable\n";
	return simple_read_from_buffer(user_buf, count, ppos,
			state, strlen(state));
}

static ssize_t nhlt_control_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char buf[16];
	int len = min(count, (sizeof(buf) - 1));


	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = 0;

	if (!strncmp(buf, "enable\n", len))
		d->skl->nhlt_override = true;
	else if (!strncmp(buf, "disable\n", len))
		d->skl->nhlt_override = false;
	else
		return -EINVAL;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	return len;
}

static const struct file_operations ssp_cntrl_nhlt_fops = {
	.open = simple_open,
	.read = nhlt_control_read,
	.write = nhlt_control_write,
	.llseek = default_llseek,
};

static int skl_init_nhlt(struct skl_debug *d)
{
	int i;
	char name[12];

	if (!debugfs_create_file("control",
				0644, d->nhlt,
				d, &ssp_cntrl_nhlt_fops)) {
		dev_err(d->dev, "nhlt control debugfs init failed\n");
		return -EIO;
	}

	for (i = 0; i < MAX_SSP; i++) {
		snprintf(name, (sizeof(name)-1), "ssp%d", i);
		if (!debugfs_create_file(name,
					0644, d->nhlt,
					&d->ssp_blob[i], &nhlt_fops))
			dev_err(d->dev, "%s: debugfs init failed\n", name);
	}

	if (!debugfs_create_file("dmic", 0644,
				d->nhlt, &d->dmic_blob,
				&nhlt_fops))
		dev_err(d->dev, "%s: debugfs init failed\n", name);

	return 0;
}

#define MOD_BUF (2 * PAGE_SIZE)

static ssize_t skl_print_pins(struct skl_module_pin *m_pin, char *buf,
				int max_pin, ssize_t ret, bool direction)
{
	int i;

	for (i = 0; i < max_pin; i++)
		ret += snprintf(buf + ret, MOD_BUF - ret,
				"%s%d\n\tModule %d\n\tInstance %d\n\t%s\n\t%s\n\tIndex:%d\n",
				direction ? "Input Pin:" : "Output Pin:",
				i, m_pin[i].id.module_id,
				m_pin[i].id.instance_id,
				m_pin[i].in_use ? "Used" : "Unused",
				m_pin[i].is_dynamic ? "Dynamic" : "Static",
				m_pin[i].pin_index);
	return ret;
}

static ssize_t skl_print_fmt(struct skl_module_fmt *fmt, char *buf,
					ssize_t ret, bool direction)
{

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"%s\n\tCH %d\n\tFreq %d\n\tBit %d\tDepth %d\n\tCh config %x\n",
			direction ? "Input Format:" : "Output Format:",
			fmt->channels, fmt->s_freq, fmt->bit_depth,
			fmt->valid_bit_depth, fmt->ch_cfg);

	return ret;
}

static ssize_t module_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct skl_module_cfg *mconfig = file->private_data;
	struct skl_module *module = mconfig->module;
	struct skl_module_res *res = &module->resources[mconfig->res_idx];
	struct skl_module_intf *m_intf;
	char *buf;
	ssize_t ret;

	buf = kzalloc(MOD_BUF, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = snprintf(buf, MOD_BUF, "Module\n\tid: %d\n\tinstance id: %d\n",
			mconfig->id.module_id, mconfig->id.instance_id);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Resources\n\tMCPS %x\n\tIBS %x\n\tOBS %x\t\n",
			res->cps, res->ibs, res->obs);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Module data:\n\tCore %d\n\tIN queue %d\n\tOut queue %d\n\t%s\n",
			mconfig->core_id, mconfig->module->max_input_pins,
			mconfig->module->max_output_pins,
			mconfig->module->loadable ? "loadable" : "inbuilt");

	m_intf = &module->formats[mconfig->fmt_idx];
	ret += skl_print_fmt(&m_intf->input[0].pin_fmt, buf, ret, true);
	ret += skl_print_fmt(&m_intf->output[0].pin_fmt, buf, ret, false);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Module Gateway\n\tType %x\n\tInstance %d\n\tHW conn %x\n\tSlot %x\n",
			mconfig->dev_type, mconfig->vbus_id,
			mconfig->hw_conn_type, mconfig->time_slot);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Pipeline ID\n\t%d\n\tPriority %d\n\tConn Type %d\n\tPages %x\n",
			mconfig->pipe->ppl_id, mconfig->pipe->pipe_priority,
			mconfig->pipe->conn_type, mconfig->pipe->memory_pages);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tParams:\n\t\tHost DMA %d\n\t\tLink DMA %d\n",
			mconfig->pipe->p_params->host_dma_id,
			mconfig->pipe->p_params->link_dma_id);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tPCM params:\n\t\tCH %d\n\t\tFreq %d\n\t\tFormat %d\n",
			mconfig->pipe->p_params->ch,
			mconfig->pipe->p_params->s_freq,
			mconfig->pipe->p_params->s_fmt);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tLink %x\n\tStream %x\n",
			mconfig->pipe->p_params->linktype,
			mconfig->pipe->p_params->stream);

	ret += skl_print_pins(mconfig->m_in_pin, buf,
			mconfig->module->max_input_pins, ret, true);
	ret += skl_print_pins(mconfig->m_out_pin, buf,
			mconfig->module->max_output_pins, ret, false);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

static const struct file_operations mcfg_fops = {
	.open = simple_open,
	.read = module_read,
	.llseek = default_llseek,
};


void skl_debug_init_module(struct skl_debug *d,
			struct snd_soc_dapm_widget *w,
			struct skl_module_cfg *mconfig)
{
	if (!debugfs_create_file(w->name, 0444,
				d->modules, mconfig,
				&mcfg_fops))
		dev_err(d->dev, "%s: module debugfs init failed\n", w->name);
}

struct skl_debug *skl_debugfs_init(struct skl *skl)
{
	struct skl_debug *d;

	d = devm_kzalloc(&skl->pci->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;

	/* create the root dir first */
	d->fs = debugfs_create_dir("snd_soc_skl", NULL);
	if (IS_ERR(d->fs) || !d->fs) {
		dev_err(&skl->pci->dev, "debugfs root creation failed\n");
		return NULL;
	}

	d->skl = skl;
	d->dev = &skl->pci->dev;

	/* now create the NHLT dir */
	d->nhlt =  debugfs_create_dir("nhlt", d->fs);
	if (IS_ERR(d->nhlt) || !d->nhlt) {
		dev_err(&skl->pci->dev, "nhlt debugfs create failed\n");
		goto err;
	}

	/* now create the module dir */
	d->modules =  debugfs_create_dir("modules", d->fs);
	if (IS_ERR(d->modules) || !d->modules) {
		dev_err(&skl->pci->dev, "modules debugfs create failed\n");
		goto err;
	}

	skl_init_nhlt(d);

	return d;

err:
	debugfs_remove_recursive(d->fs);
	return NULL;
}

void skl_debugfs_exit(struct skl_debug *d)
{
	int i;

	debugfs_remove_recursive(d->fs);

	/* free blob memory, if allocated */
	for (i = 0; i < MAX_SSP; i++)
		kfree(d->ssp_blob[i].cfg);
	kfree(d->dmic_blob.cfg);

	kfree(d);

}
