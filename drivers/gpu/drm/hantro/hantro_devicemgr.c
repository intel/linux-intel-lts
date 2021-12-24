// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro device manager.
 *
 *    Copyright (c) 2018 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"

int get_device_core_num(u32 deviceid, enum device_coretype type)
{
	struct device_info *hdr = get_device_info(deviceid);

	if (!hdr)
		return 0;

	switch (type) {
	case CORE_ENC:
		return hdr->enccore_num;
	case CORE_DEC:
		return hdr->deccore_num;
	case CORE_CACHE:
		return hdr->cachecore_num;
	case CORE_DEC400:
		return hdr->dec400core_num;
	default:
		return 0;
	}
}

/* get dec nodes list hdr of a device */
struct hantrodec_t *get_dec_node(struct device_info *pdevinfo, u32 nodeidx)
{
	int i;
	struct hantrodec_t *p;

	if (!pdevinfo)
		return NULL;

	p = pdevinfo->dechdr;
	for (i = 0; i < nodeidx; i++) {
		if (!p)
			break;

		p = p->next;
	}

	return p;
}

/* get dec nodes list hdr of a device */
struct hantrodec_t *get_dec_node_by_device_id(u32 deviceid, u32 nodeidx)
{
	int i;
	struct hantrodec_t *p;
	struct device_info *hdr = get_device_info(deviceid);

	if (!hdr)
		return NULL;

	p = hdr->dechdr;
	for (i = 0; i < nodeidx; i++) {
		if (!p)
			break;

		p = p->next;
	}

	return p;
}

struct hantrodec_t *get_first_dec_nodes(struct device_info *pdevinfo)
{
	if (pdevinfo)
		return pdevinfo->dechdr;

	return NULL;
}

/* get enc nodes list hdr of a device */
struct hantroenc_t *get_enc_node(struct device_info *pdevinfo, u32 nodeidx)
{
	int i;
	struct hantroenc_t *p;

	if (!pdevinfo)
		return NULL;

	p = pdevinfo->enchdr;
	for (i = 0; i < nodeidx; i++) {
		if (!p)
			break;

		p = p->next;
	}

	return p;
}

/* get enc nodes list hdr of a device */
struct hantroenc_t *get_enc_node_by_device_id(u32 deviceid, u32 nodeidx)
{
	int i;
	struct hantroenc_t *p;
	struct device_info *hdr = get_device_info(deviceid);

	if (!hdr)
		return NULL;

	p = hdr->enchdr;
	for (i = 0; i < nodeidx; i++) {
		if (!p)
			break;

		p = p->next;
	}

	return p;
}

/* get cache nodes list hdr of a device */
struct cache_dev_t *get_cache_nodes(u32 deviceid, u32 nodeidx)
{
	int i;
	struct cache_dev_t *p;
	struct device_info *hdr = get_device_info(deviceid);

	if (!hdr)
		return NULL;

	p = hdr->cachehdr;
	for (i = 0; i < nodeidx; i++) {
		if (!p)
			break;

		p = p->next;
	}

	return p;
}

/* get dec400 nodes list hdr of a device */
struct dec400_t *get_dec400_nodes(u32 deviceid, u32 nodeidx)
{
	int i;
	struct dec400_t *p;
	struct device_info *hdr = get_device_info(deviceid);

	if (!hdr)
		return NULL;

	p = hdr->dec400hdr;
	for (i = 0; i < nodeidx; i++) {
		if (!p)
			break;

		p = p->next;
	}

	return p;
}

/* get dec400 nodes by parent type and parent core num */
struct dec400_t *get_dec400_node_by_type(u32 deviceid, u32 parenttype,
					 u32 parentnodeidx)
{
	struct dec400_t *p;
	struct device_info *hdr = get_device_info(deviceid);

	if (!hdr)
		return NULL;

	p = hdr->dec400hdr;
	while (p) {
		if (p->parentid == parentnodeidx &&
		    ((parenttype == NODE_TYPE_DEC &&
		      p->parenttype == CORE_DEC) ||
		     (parenttype == NODE_TYPE_ENC && p->parenttype == CORE_ENC)))
			break;

		p = p->next;
	}

	return p;
}

int add_dec_node(struct device_info *pdevinfo, struct hantrodec_t *deccore)
{
	struct hantrodec_t *pdec;

	if (!pdevinfo)
		return -EINVAL;

	pdec = pdevinfo->dechdr;
	if (!pdec) {
		pdevinfo->dechdr = deccore;
	} else {
		while (pdec->next)
			pdec = pdec->next;

		pdec->next = deccore;
	}

	deccore->next = NULL;
	pdevinfo->deccore_num++;
	deccore->core_id = pdevinfo->deccore_num - 1;
	deccore->pdevinfo = pdevinfo;
	pdevinfo->config |= CONFIG_HWDEC;

	sema_init(&pdevinfo->dec_core_sem, pdevinfo->deccore_num);
	return 0;
}

int add_enc_node(struct device_info *pdevinfo, struct hantroenc_t *enccore)
{
	struct hantroenc_t *penc;

	if (!pdevinfo)
		return -EINVAL;

	penc = pdevinfo->enchdr;
	if (!penc) {
		pdevinfo->enchdr = enccore;
	} else {
		while (penc->next)
			penc = penc->next;

		penc->next = enccore;
	}

	enccore->next = NULL;
	pdevinfo->enccore_num++;
	enccore->core_id = pdevinfo->enccore_num - 1;
	enccore->pdevinfo = pdevinfo;
	pdevinfo->config |= CONFIG_HWENC;
	return 0;
}

int add_dec400_node(struct device_info *pdevinfo, struct dec400_t *dec400core)
{
	struct dec400_t *pdec400;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;

	if (!pdevinfo)
		return -EINVAL;

	pdec400 = pdevinfo->dec400hdr;
	if (!pdec400) {
		pdevinfo->dec400hdr = dec400core;
	} else {
		while (pdec400->next)
			pdec400 = pdec400->next;

		pdec400->next = dec400core;
	}

	dec400core->next = NULL;
	pdevinfo->dec400core_num++;
	dec400core->core_id = pdevinfo->dec400core_num - 1;
	pdevinfo->config |= CONFIG_DEC400;
	/* set default */
	dec400core->parentcore = pdevinfo;
	dec400core->parentid = CORE_DEVICE;
	dec400core->pdevinfo = pdevinfo;

	if (dec400core->core_cfg.parentaddr == pdevinfo->rsvmem_addr) {
		dec400core->parentcore = pdevinfo;
		dec400core->parenttype = CORE_DEVICE;
		goto end;
	}

	penc = pdevinfo->enchdr;
	while (penc) {
		if ((unsigned long long)penc->core_cfg.base_addr ==
		    dec400core->core_cfg.parentaddr) {
			dec400core->parentcore = penc;
			dec400core->parentid = penc->core_id;
			dec400core->parenttype = CORE_ENC;
			goto end;
		}

		penc = penc->next;
	}

	pdec = pdevinfo->dechdr;
	while (pdec) {
		if ((unsigned long long)pdec->multicorebase ==
		    dec400core->core_cfg.parentaddr) {
			dec400core->parentcore = pdec;
			dec400core->parentid = pdec->core_id;
			dec400core->parenttype = CORE_DEC;
			goto end;
		}

		pdec = pdec->next;
	}

end:
	return 0;
}

int add_cache_node(struct device_info *pdevinfo, struct cache_dev_t *cachecore)
{
	struct cache_dev_t *pcache;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;

	if (!pdevinfo)
		return -ENODEV;

	pcache = pdevinfo->cachehdr;
	if (!pcache) {
		pdevinfo->cachehdr = cachecore;
	} else {
		while (pcache->next)
			pcache = pcache->next;

		pcache->next = cachecore;
	}

	cachecore->next = NULL;
	pdevinfo->cachecore_num++;
	cachecore->core_id = pdevinfo->cachecore_num - 1;
	pdevinfo->config |= CONFIG_L2CACHE;

	/* set default */
	cachecore->parentcore = pdevinfo;
	cachecore->parentid = pdevinfo->deviceid;
	cachecore->parenttype = CORE_DEVICE;
	cachecore->pdevinfo = pdevinfo;

	if (cachecore->core_cfg.client == VC8000E) {
		penc = pdevinfo->enchdr;
		while (penc) {
			if ((unsigned long long)penc->core_cfg.base_addr ==
			    cachecore->core_cfg.parentaddr) {
				cachecore->parentcore = penc;
				cachecore->parentid = penc->core_id;
				cachecore->parenttype = CORE_ENC;
				break;
			}

			penc = penc->next;
		}
	} else {
		pdec = pdevinfo->dechdr;
		while (pdec) {
			if ((unsigned long long)pdec->multicorebase ==
			    cachecore->core_cfg.parentaddr) {
				cachecore->parentcore = pdec;
				cachecore->parentid = pdec->core_id;
				cachecore->parenttype = CORE_DEC;
				break;
			}

			pdec = pdec->next;
		}
	}

	return 0;
}

int get_device_count(void)
{
	return atomic_read(&hantro_drm.devicecount);
}

long hantrodevice_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case DRM_IOCTL_HANTRO_GET_DEVICENUM:
		return get_device_count();
	default:
		return -EINVAL;
	}
}

void device_print_debug(void)
{
	struct hantrodec_t *pdec, *pdec2;
	struct hantroenc_t *penc, *penc2;
	struct device_info *pdevinfo;
	struct cache_dev_t *pcache, *pcache2;
	struct dec400_t *pdec400, *pdec400_2;
	int i, n = get_device_count(), k;
	int decn, encn, cachen, dec400n;
	struct device_info *pdevinfo0, *pdevinfo1;

	pr_info("device info start");
	pr_info("device num = %d", n);
	pdevinfo0 = hantro_drm.pdevice_list;
	for (i = 0; i < n; i++) {
		pr_info("device %d:%lx:%llx:%lld:%x", i,
			(unsigned long)pdevinfo0->dev, pdevinfo0->rsvmem_addr,
			pdevinfo0->memsize, pdevinfo0->config);

		decn = get_device_core_num(i, CORE_DEC);
		pr_info("dec num = %d", decn);
		encn = get_device_core_num(i, CORE_ENC);
		pr_info("enc num = %d", encn);
		cachen = get_device_core_num(i, CORE_CACHE);
		pr_info("cache  num = %d", cachen);
		dec400n = get_device_core_num(i, CORE_DEC400);
		pr_info("dec400n num = %d", dec400n);

		pdec = get_dec_node_by_device_id(i, 0);
		k = 0;
		while (pdec) {
			pr_info("dec core %d", k);
			pdec2 = get_dec_node_by_device_id(i, k);
			pdevinfo1 = pdec->pdevinfo;
			if (pdec != pdec2)
				pr_info("get_decnodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("get_parent_device fails @ dec %d:%d", i,
					k);

			pr_info("addr=%llx, size=%d", pdec->multicorebase,
				pdec->iosize);
			pr_info("irq0=%d, irq1=%d", pdec->irqlist[0],
				pdec->irqlist[1]);
			if (pdec->its_main_core_id) {
				pdec2 = pdec->its_main_core_id;
				pr_info("main core = %d:%d", pdevinfo1->deviceid,
					pdec2->core_id);
			}

			if (pdec->its_aux_core_id) {
				pdec2 = pdec->its_aux_core_id;
				pr_info("aux core = %d:%d", pdec2->pdevinfo->deviceid,
					pdec2->core_id);
			}

			pdec = pdec->next;
			k++;
		}

		penc = get_enc_node_by_device_id(i, 0);
		k = 0;
		while (penc) {
			pr_info("enc core %d:", k);
			penc2 = get_enc_node_by_device_id(i, k);
			pdevinfo1 = penc->pdevinfo;
			if (penc != penc2)
				pr_info("get_encnodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("get_parent_device fails @ enc %d:%d", i,
					k);

			pr_info("addr=%llx, size=%d", penc->core_cfg.base_addr,
				penc->core_cfg.iosize);
			pr_info("irq0=%d, irq1=%d", penc->irqlist[0],
				penc->irqlist[1]);
			penc = penc->next;
			k++;
		}

		pcache = get_cache_nodes(i, 0);
		k = 0;
		while (pcache) {
			pr_info("cache core %d:", k);
			pcache2 = get_cache_nodes(i, k);
			pdevinfo1 = pcache->pdevinfo;
			if (pcache != pcache2)
				pr_info("get_cache_nodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("get_parent_device fails @ cache %d:%d",
					i, k);

			pr_info("addr=%llx, size=%d, type=%d, dir=%d",
				pcache->core_cfg.base_addr,
				pcache->core_cfg.iosize,
				pcache->core_cfg.client, pcache->core_cfg.dir);
			pr_info("irq0=%d, irq1=%d", pcache->irqlist[0],
				pcache->irqlist[1]);
			pr_info("parent addr=%llx",
				pcache->core_cfg.parentaddr);
			if (pcache->parentcore) {
				if (pcache->core_cfg.client == VC8000E) {
					penc = (struct hantroenc_t *)
						       pcache->parentcore;
					pr_info("parent enc core = %d:%d,addr %llx",
						penc->pdevinfo->deviceid,
						penc->core_id,
						penc->core_cfg.base_addr);
				} else {
					pdec = (struct hantrodec_t *)
						       pcache->parentcore;
					pr_info("parent dec core = %d:%d,addr %llx",
						pdec->pdevinfo->deviceid, pdec->core_id,
						pdec->multicorebase);
				}

			} else {
				pr_info("parent core = NULL");
			}

			pcache = pcache->next;
			k++;
		}
		pdec400 = get_dec400_nodes(i, 0);
		k = 0;
		while (pdec400) {
			pr_info("dec400 core %d:", k);
			pdec400_2 = get_dec400_nodes(i, k);
			pdevinfo1 = pdec400->pdevinfo;
			if (pdec400 != pdec400_2)
				pr_info("get_dec400_nodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("get_parent_device fails @ dec400 %d:%d",
					i, k);

			pr_info("addr=%llx, size=%d",
				pdec400->core_cfg.dec400corebase,
				pdec400->core_cfg.iosize);
			pr_info("parent addr=%llx",
				pdec400->core_cfg.parentaddr);
			if (pdec400->parentcore) {
				switch (pdec400->parenttype) {
				case CORE_ENC:
					penc = (struct hantroenc_t *)
						       pdec400->parentcore;
					pr_info("parent enc core = %d:%d,addr %llx",
						penc->pdevinfo->deviceid,
						penc->core_id,
						penc->core_cfg.base_addr);
					break;
				case CORE_DEC:
					pdec = (struct hantrodec_t *)
						       pdec400->parentcore;
					pr_info("parent dec core = %d:%d,addr %llx",
						pdec->pdevinfo->deviceid, pdec->core_id,
						pdec->multicorebase);
					break;
				case CORE_DEVICE:
					pdevinfo = (struct device_info *)
							  pdec400->parentcore;
					pr_info("parent device addr %llx",
						pdevinfo->rsvmem_addr);
					break;
				default:
					pr_info("error: dec400 parent type unknown");
					break;
				}
			} else {
				pr_info("parent core = NULL");
			}

			pdec400 = pdec400->next;
			k++;
		}

		pdevinfo0 = pdevinfo0->next;
	}

	pr_info("device info finish");
}
