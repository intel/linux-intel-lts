// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro device manager.
 *
 *    Copyright (c) 2018 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"

int get_devicecorenum(u32 deviceid, device_coretype type)
{
	struct device_info *hdr = get_deviceinfo(deviceid);

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
struct hantrodec_t *get_decnode(struct device_info *hdr, u32 nodeidx)
{
	int i;
	struct hantrodec_t *p;

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

/* get dec nodes list hdr of a device */
struct hantrodec_t *get_decnode_bydeviceid(u32 deviceid, u32 nodeidx)
{
	int i;
	struct hantrodec_t *p;
	struct device_info *hdr = get_deviceinfo(deviceid);

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

struct hantrodec_t *getfirst_decnodes(struct device_info *pdevice)
{
	if (pdevice) //atomic_read(&devicenum))
		return pdevice->dechdr;

	return NULL;
}

/* get enc nodes list hdr of a device */
struct hantroenc_t *get_encnode(struct device_info *hdr, u32 nodeidx)
{
	int i;
	struct hantroenc_t *p;

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

/* get enc nodes list hdr of a device */
struct hantroenc_t *get_encnode_bydeviceid(u32 deviceid, u32 nodeidx)
{
	int i;
	struct hantroenc_t *p;
	struct device_info *hdr = get_deviceinfo(deviceid);

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
struct cache_dev_t *get_cachenodes(u32 deviceid, u32 nodeidx)
{
	int i;
	struct cache_dev_t *p;
	struct device_info *hdr = get_deviceinfo(deviceid);

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

struct cache_dev_t *get_cachenodebytype(u32 deviceid, u32 parenttype,
					u32 parentnodeidx)
{
	struct cache_dev_t *p;
	struct device_info *hdr = get_deviceinfo(deviceid);

	if (!hdr)
		return NULL;

	p = hdr->cachehdr;
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

/* get dec400 nodes list hdr of a device */
struct dec400_t *get_dec400nodes(u32 deviceid, u32 nodeidx)
{
	int i;
	struct dec400_t *p;
	struct device_info *hdr = get_deviceinfo(deviceid);

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
struct dec400_t *get_dec400nodebytype(u32 deviceid, u32 parenttype,
				      u32 parentnodeidx)
{
	struct dec400_t *p;
	struct device_info *hdr = get_deviceinfo(deviceid);

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

int add_decnode(struct device_info *pdevice, struct hantrodec_t *deccore)
{
	struct hantrodec_t *pdec;

	if (!pdevice)
		return -EINVAL;

	pdec = pdevice->dechdr;
	if (!pdec) {
		pdevice->dechdr = deccore;
	} else {
		while (pdec->next)
			pdec = pdec->next;

		pdec->next = deccore;
	}

	deccore->next = NULL;
	pdevice->deccore_num++;
	deccore->core_id = pdevice->deccore_num - 1;
	deccore->deviceidx = pdevice->deviceid;
	deccore->parentdevice = pdevice;
	pdevice->config |= CONFIG_HWDEC;

	sema_init(&pdevice->dec_core_sem, pdevice->deccore_num);
	return 0;
}

int add_encnode(struct device_info *pdevice, struct hantroenc_t *enccore)
{
	struct hantroenc_t *penc;

	if (!pdevice)
		return -EINVAL;

	penc = pdevice->enchdr;
	if (!penc) {
		pdevice->enchdr = enccore;
	} else {
		while (penc->next)
			penc = penc->next;

		penc->next = enccore;
	}

	enccore->next = NULL;
	pdevice->enccore_num++;
	enccore->core_id = pdevice->enccore_num - 1;
	enccore->core_cfg.deviceidx = pdevice->deviceid;
	enccore->parentdevice = pdevice;
	pdevice->config |= CONFIG_HWENC;
	return 0;
}

int add_dec400node(struct device_info *pdevice, struct dec400_t *dec400core)
{
	struct dec400_t *pdec400;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;

	if (!pdevice)
		return -EINVAL;

	pdec400 = pdevice->dec400hdr;
	if (!pdec400) {
		pdevice->dec400hdr = dec400core;
	} else {
		while (pdec400->next)
			pdec400 = pdec400->next;

		pdec400->next = dec400core;
	}

	dec400core->next = NULL;
	pdevice->dec400core_num++;
	dec400core->core_id = pdevice->dec400core_num - 1;
	pdevice->config |= CONFIG_DEC400;
	dec400core->core_cfg.deviceidx = pdevice->deviceid;
	/* set default */
	dec400core->parentcore = pdevice;
	dec400core->parentid = CORE_DEVICE;
	dec400core->parentdevice = pdevice;

	if (dec400core->core_cfg.parentaddr == pdevice->rsvmem_addr) {
		dec400core->parentcore = pdevice;
		dec400core->parenttype = CORE_DEVICE;
		goto end;
	}

	penc = pdevice->enchdr;
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

	pdec = pdevice->dechdr;
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

int add_cachenode(struct device_info *pdevice, struct cache_dev_t *cachecore)
{
	struct cache_dev_t *pcache;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;

	if (!pdevice)
		return -ENODEV;

	pcache = pdevice->cachehdr;
	if (!pcache) {
		pdevice->cachehdr = cachecore;
	} else {
		while (pcache->next)
			pcache = pcache->next;

		pcache->next = cachecore;
	}

	cachecore->next = NULL;
	pdevice->cachecore_num++;
	cachecore->core_id = pdevice->cachecore_num - 1;
	cachecore->core_cfg.deviceidx = pdevice->deviceid;
	pdevice->config |= CONFIG_L2CACHE;

	/* set default */
	cachecore->parentcore = pdevice;
	cachecore->parentid = pdevice->deviceid;
	cachecore->parenttype = CORE_DEVICE;
	cachecore->parentdevice = pdevice;

	if (cachecore->core_cfg.client == VC8000E) {
		penc = pdevice->enchdr;
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
		pdec = pdevice->dechdr;
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

int get_devicecount(void)
{
	return atomic_read(&hantro_drm.devicecount);
}

struct device_info *getparentdevice(void *node, int type)
{
	struct device_info *pdevice = NULL;

	switch (type) {
	case CORE_CACHE:
		pdevice = (struct device_info *)((struct cache_dev_t *)node)
				  ->parentdevice;
		break;
	case CORE_DEC:
		pdevice = (struct device_info *)((struct hantrodec_t *)node)
				  ->parentdevice;
		break;
	case CORE_ENC:
		pdevice = (struct device_info *)((struct hantroenc_t *)node)
				  ->parentdevice;
		break;
	case CORE_DEC400:
		pdevice = (struct device_info *)((struct dec400_t *)node)
				  ->parentdevice;
		break;
	default:
		break;
	}
	return pdevice;
}

long hantrodevice_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case DRM_IOCTL_HANTRO_GET_DEVICENUM:
		return get_devicecount();
	default:
		return -EINVAL;
	}
}

void device_printdebug(void)
{
	struct hantrodec_t *pdec, *pdec2;
	struct hantroenc_t *penc, *penc2;
	struct device_info *pdevice;
	struct cache_dev_t *pcache, *pcache2;
	struct dec400_t *pdec400, *pdec400_2;
	int i, n = get_devicecount(), k;
	int decn, encn, cachen, dec400n;
	struct device_info *pdevinfo0, *pdevinfo1;

	pr_info("########### device info start #############");
	pr_info("device num = %d", n);
	pdevinfo0 = hantro_drm.pdevice_list;
	for (i = 0; i < n; i++) {
		pr_info("device %d:%lx:%llx:%lld:%x", i,
			(unsigned long)pdevinfo0->dev, pdevinfo0->rsvmem_addr,
			pdevinfo0->memsize, pdevinfo0->config);

		decn = get_devicecorenum(i, CORE_DEC);
		pr_info("dec num = %d", decn);
		encn = get_devicecorenum(i, CORE_ENC);
		pr_info("enc num = %d", encn);
		cachen = get_devicecorenum(i, CORE_CACHE);
		pr_info("cache  num = %d", cachen);
		dec400n = get_devicecorenum(i, CORE_DEC400);
		pr_info("dec400n num = %d", dec400n);

		pdec = get_decnode_bydeviceid(i, 0);
		k = 0;
		while (pdec) {
			pr_info("dec core %d", k);
			pdec2 = get_decnode_bydeviceid(i, k);
			pdevinfo1 = getparentdevice(pdec, CORE_DEC);
			if (pdec != pdec2)
				pr_info("get_decnodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("getparentdevice fails @ dec %d:%d", i,
					k);

			pr_info("addr=%llx, size=%d", pdec->multicorebase,
				pdec->iosize);
			pr_info("irq0=%d, irq1=%d", pdec->irqlist[0],
				pdec->irqlist[1]);
			if (pdec->its_main_core_id) {
				pdec2 = pdec->its_main_core_id;
				pr_info("main core = %d:%d", pdec2->deviceidx,
					pdec2->core_id);
			}

			if (pdec->its_aux_core_id) {
				pdec2 = pdec->its_aux_core_id;
				pr_info("aux core = %d:%d", pdec2->deviceidx,
					pdec2->core_id);
			}

			pdec = pdec->next;
			k++;
		}

		penc = get_encnode_bydeviceid(i, 0);
		k = 0;
		while (penc) {
			pr_info("enc core %d:", k);
			penc2 = get_encnode_bydeviceid(i, k);
			pdevinfo1 = getparentdevice(penc, CORE_ENC);
			if (penc != penc2)
				pr_info("get_encnodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("getparentdevice fails @ enc %d:%d", i,
					k);

			pr_info("addr=%llx, size=%d", penc->core_cfg.base_addr,
				penc->core_cfg.iosize);
			pr_info("irq0=%d, irq1=%d", penc->irqlist[0],
				penc->irqlist[1]);
			penc = penc->next;
			k++;
		}

		pcache = get_cachenodes(i, 0);
		k = 0;
		while (pcache) {
			pr_info("cache core %d:", k);
			pcache2 = get_cachenodes(i, k);
			pdevinfo1 = getparentdevice(pcache, CORE_CACHE);
			if (pcache != pcache2)
				pr_info("get_cachenodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("getparentdevice fails @ cache %d:%d",
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
						penc->core_cfg.deviceidx,
						penc->core_id,
						penc->core_cfg.base_addr);
				} else {
					pdec = (struct hantrodec_t *)
						       pcache->parentcore;
					pr_info("parent dec core = %d:%d,addr %llx",
						pdec->deviceidx, pdec->core_id,
						pdec->multicorebase);
				}

			} else {
				pr_info("parent core = NULL");
			}

			pcache = pcache->next;
			k++;
		}
		pdec400 = get_dec400nodes(i, 0);
		k = 0;
		while (pdec400) {
			pr_info("dec400 core %d:", k);
			pdec400_2 = get_dec400nodes(i, k);
			pdevinfo1 = getparentdevice(pdec400, CORE_DEC400);
			if (pdec400 != pdec400_2)
				pr_info("get_dec400nodes fails @ %d", k);

			if (pdevinfo0 != pdevinfo1)
				pr_info("getparentdevice fails @ dec400 %d:%d",
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
						penc->core_cfg.deviceidx,
						penc->core_id,
						penc->core_cfg.base_addr);
					break;
				case CORE_DEC:
					pdec = (struct hantrodec_t *)
						       pdec400->parentcore;
					pr_info("parent dec core = %d:%d,addr %llx",
						pdec->deviceidx, pdec->core_id,
						pdec->multicorebase);
					break;
				case CORE_DEVICE:
					pdevice = (struct device_info *)
							  pdec400->parentcore;
					pr_info("parent device addr %llx",
						pdevice->rsvmem_addr);
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

	pr_info("########### device info finish #############");
}
