/*
 * Copyright(C) 2018 Verisilicon
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <linux/atomic.h>
#include "hantro_priv.h"


/*currently we simply not using dynamic chains*/
static struct slice_info *slicehdr;
static atomic_t sliceinitd;
static atomic_t slicenum;

int findslice_bydev(struct device *dev)
{
	int i = 0;
	struct slice_info *hdr = slicehdr;

	if (atomic_read(&sliceinitd) == 0)
		return -1;

	while (hdr != NULL) {
		if (hdr->dev == dev)
			return i;
		i++;
		hdr = hdr->next;
	}
	return -1;
}

struct slice_info *getslicenode_inInit(u32 sliceindex)
{
	int i = 0;
	struct slice_info *hdr = slicehdr;

	if (sliceindex >= atomic_read(&slicenum))
		return NULL;
	for (i = 0; i < sliceindex; i++)
		hdr = hdr->next;

	return hdr;
}

struct slice_info *getslicenode(u32 sliceindex)
{
	if (atomic_read(&sliceinitd) == 0)
		return NULL;
	return getslicenode_inInit(sliceindex);
}

int get_slicecorenum(u32 sliceindex, slice_coretype type)
{
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL)
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

/*get dec nodes list hdr of a slice*/
struct hantrodec_t *get_decnodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct hantrodec_t *p;
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL)
		return NULL;

	p = hdr->dechdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	return p;
}

struct hantrodec_t *getfirst_decnodes(struct slice_info *pslice)
{
	if (atomic_read(&slicenum))
		return pslice->dechdr;
	return NULL;
}


/*get enc nodes list hdr of a slice*/
struct hantroenc_t *get_encnodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct hantroenc_t *p;
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL)
		return NULL;
	p = hdr->enchdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	return p;
}

/*get cache nodes list hdr of a slice*/
struct cache_dev_t *get_cachenodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct cache_dev_t *p;
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL) {
		printk("%s:%d:%d", __func__, atomic_read(&sliceinitd), atomic_read(&slicenum));
		return NULL;
	}
	p = hdr->cachehdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	return p;
}

struct cache_dev_t *get_cachenodebytype(u32 sliceindex, u32 parenttype, u32 parentnodeidx)
{
	struct cache_dev_t *p;
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL)
		return NULL;
	p = hdr->cachehdr;
	while (p != NULL) {
		if (p->parentid == parentnodeidx &&
			((parenttype == NODE_TYPE_DEC && p->parenttype == CORE_DEC)
			|| (parenttype == NODE_TYPE_ENC && p->parenttype == CORE_ENC)))
			break;
		p = p->next;
	}
	return p;

}

/*get dec400 nodes list hdr of a slice*/
struct dec400_t *get_dec400nodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct dec400_t *p;
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL)
		return NULL;
	p = hdr->dec400hdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	return p;
}

/*get dec400 nodes by parent type and parent core num */
struct dec400_t *get_dec400nodebytype(u32 sliceindex, u32 parenttype, u32 parentnodeidx)
{
	struct dec400_t *p;
	struct slice_info *hdr = getslicenode(sliceindex);

	if (hdr == NULL)
		return NULL;
	p = hdr->dec400hdr;
	while (p != NULL) {
		if (p->parentid == parentnodeidx &&
			((parenttype == NODE_TYPE_DEC && p->parenttype == CORE_DEC)
			|| (parenttype == NODE_TYPE_ENC && p->parenttype == CORE_ENC)))
			break;
		p = p->next;
	}
	return p;
}

int add_decnode(u32 sliceindex, struct hantrodec_t *deccore)
{
	struct hantrodec_t *pdec;
	struct slice_info *splice = getslicenode_inInit(sliceindex);

#ifdef USE_DTB_PROBE
	if (splice == NULL)
		return -EINVAL;
#else
	if (splice == NULL && sliceindex == atomic_read(&slicenum)) {
		sliceindex = addslice(NULL, 0, 0);
		if (sliceindex < 0)
			return -EINVAL;
		splice = getslicenode_inInit(sliceindex);
	}
	if (splice == NULL)
		return -EINVAL;
#endif

	pdec = splice->dechdr;
	if (pdec == NULL)
		splice->dechdr = deccore;
	else {
		while (pdec->next != NULL)
			pdec = pdec->next;
		pdec->next = deccore;
	}
	deccore->next = NULL;
	splice->deccore_num++;
	deccore->core_id = splice->deccore_num - 1;
	deccore->sliceidx = sliceindex;
	deccore->parentslice = splice;
	splice->config |= CONFIG_HWDEC;

	sema_init(&splice->dec_core_sem, splice->deccore_num);
	return 0;
}

int add_encnode(u32 sliceindex, struct hantroenc_t *enccore)
{
	struct hantroenc_t *penc;
	struct slice_info *splice = getslicenode_inInit(sliceindex);

#ifdef USE_DTB_PROBE
	if (splice == NULL)
		return -EINVAL;
#else
	if (splice == NULL && sliceindex == atomic_read(&slicenum)) {
		sliceindex = addslice(NULL, 0, 0);
		if (sliceindex < 0)
			return -EINVAL;
		splice = getslicenode_inInit(sliceindex);
	}
#endif

	penc = splice->enchdr;
	if (penc == NULL)
		splice->enchdr = enccore;
	else {
		while (penc->next != NULL)
			penc = penc->next;
		penc->next = enccore;
	}
	enccore->next = NULL;
	splice->enccore_num++;
	enccore->core_id = splice->enccore_num - 1;
	enccore->core_cfg.sliceidx = sliceindex;
	enccore->parentslice = splice;
	splice->config |= CONFIG_HWENC;
	return 0;
}

int add_dec400node(u32 sliceindex, struct dec400_t *dec400core)
{
	struct dec400_t *pdec400;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;
	struct slice_info *splice = getslicenode_inInit(sliceindex);

	if (splice == NULL)
		return -EINVAL;

	pdec400 = splice->dec400hdr;
	if (pdec400 == NULL)
		splice->dec400hdr = dec400core;
	else {
		while (pdec400->next != NULL)
			pdec400 = pdec400->next;
		pdec400->next = dec400core;
	}
	dec400core->next = NULL;
	splice->dec400core_num++;
	dec400core->core_id = splice->dec400core_num - 1;
	splice->config |= CONFIG_DEC400;
	dec400core->core_cfg.sliceidx = sliceindex;

	//set default
	dec400core->parentcore = splice;
	dec400core->parentid = CORE_SLICE;
	dec400core->parentslice = splice;

	if (dec400core->core_cfg.parentaddr == splice->rsvmem_addr) {
		dec400core->parentcore = splice;
		dec400core->parenttype = CORE_SLICE;
		goto end;
	}
	penc = splice->enchdr;
	while (penc != NULL) {
		if ((unsigned long long)penc->core_cfg.base_addr == dec400core->core_cfg.parentaddr) {
			dec400core->parentcore = penc;
			dec400core->parentid = penc->core_id;
			dec400core->parenttype = CORE_ENC;
			goto end;
		}
		penc = penc->next;
	}
	pdec = splice->dechdr;
	while (pdec != NULL) {
		if ((unsigned long long)pdec->multicorebase == dec400core->core_cfg.parentaddr) {
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

int add_cachenode(u32 sliceindex, struct cache_dev_t *cachecore)
{
	struct cache_dev_t *pcache;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;
	struct slice_info *splice = getslicenode_inInit(sliceindex);

	if (splice == NULL)
		return -ENODEV;

	pcache = splice->cachehdr;
	if (pcache == NULL)
		splice->cachehdr = cachecore;
	else {
		while (pcache->next != NULL)
			pcache = pcache->next;
		pcache->next = cachecore;
	}
	cachecore->next = NULL;
	splice->cachecore_num++;
	cachecore->core_id = splice->cachecore_num - 1;
	cachecore->core_cfg.sliceidx = sliceindex;
	splice->config |= CONFIG_L2CACHE;

	//set default
	cachecore->parentcore = splice;
	cachecore->parentid = sliceindex;
	cachecore->parenttype = CORE_SLICE;
	cachecore->parentslice = splice;

	if (cachecore->core_cfg.client == VC8000E) {
		penc = splice->enchdr;
		while (penc != NULL) {
			if ((unsigned long long)penc->core_cfg.base_addr == cachecore->core_cfg.parentaddr) {
				cachecore->parentcore = penc;
				cachecore->parentid = penc->core_id;
				cachecore->parenttype = CORE_ENC;
				break;
			}
			penc = penc->next;
		}
	} else {
		pdec = splice->dechdr;
		while (pdec != NULL) {
			if ((unsigned long long)pdec->multicorebase == cachecore->core_cfg.parentaddr) {
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

int get_slicenumber(void)
{
	return atomic_read(&slicenum);
}

struct slice_info *getparentslice(void *node, int type)
{
	struct slice_info *pslice = NULL;

	switch (type) {
	case CORE_CACHE:
		pslice = (struct slice_info *)((struct cache_dev_t *)node)->parentslice;
;
		break;
	case CORE_DEC:
		pslice = (struct slice_info *)((struct hantrodec_t *)node)->parentslice;
		break;
	case CORE_ENC:
		pslice = (struct slice_info *)((struct hantroenc_t *)node)->parentslice;
		break;
	case CORE_DEC400:
		pslice = (struct slice_info *)((struct dec400_t *)node)->parentslice;
		break;
	default:
		break;
	}
	return pslice;
}

/*for driver unload*/
int slice_remove(void)
{
	struct slice_info *post, *prev;
	post = prev = slicehdr;
	while (prev != NULL) {
		post = prev->next;
		kfree(prev);
		prev = post;
	}
	atomic_set(&slicenum, 0);
	atomic_set(&sliceinitd, 0);
	slicehdr = NULL;
	return 0;
}

int addslice(struct device *dev, phys_addr_t sliceaddr, phys_addr_t slicesize)
{
	struct slice_info *pslice = kzalloc(sizeof(struct slice_info), GFP_KERNEL);

	if (pslice == NULL)
		return -ENOMEM;

	pslice->dev = dev;
	pslice->rsvmem_addr = sliceaddr;
	pslice->memsize = slicesize;
	pslice->config = 0;
	pslice->next = NULL;
	pslice->deccore_num = pslice->enccore_num = pslice->cachecore_num = pslice->dec400core_num = 0;
	pslice->dechdr = NULL;
	pslice->enchdr = NULL;
	pslice->cachehdr = NULL;
	pslice->dec400hdr = NULL;

	init_waitqueue_head(&pslice->cache_hw_queue);
	init_waitqueue_head(&pslice->cache_wait_queue);
	spin_lock_init(&pslice->cache_owner_lock);

	sema_init(&pslice->enc_core_sem, 1);
	init_waitqueue_head(&pslice->enc_hw_queue);
	spin_lock_init(&pslice->enc_owner_lock);
	init_waitqueue_head(&pslice->enc_wait_queue);

	pslice->dec_irq = 0;
	pslice->pp_irq = 0;
	spin_lock_init(&pslice->owner_lock);
	init_waitqueue_head(&pslice->dec_wait_queue);
	init_waitqueue_head(&pslice->pp_wait_queue);
	init_waitqueue_head(&pslice->hw_queue);
	sema_init(&pslice->pp_core_sem, 1);
	/*dec_core_sem could only be initialized after all dec core be inserted*/

	if (slicehdr == NULL) {
		slicehdr = pslice;
	} else {
		struct slice_info *head = slicehdr;

		while (head->next != NULL)
			head = head->next;
		head->next = pslice;
	}
	atomic_inc(&slicenum);
	return atomic_read(&slicenum) - 1;
}

/*for driver load*/
int __init slice_init(void)
{
	atomic_set(&slicenum, 0);
	atomic_set(&sliceinitd, 0);
	slicehdr = NULL;
	return 0;
}

void __init slice_init_finish(void)
{
	atomic_inc(&sliceinitd);
}

long hantroslice_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case DRM_IOCTL_HANTRO_GET_SLICENUM:
		return get_slicenumber();
	default:
		return -EINVAL;
	}
}

void slice_printdebug(void)
{
	struct hantrodec_t *pdec, *pdec2;
	struct hantroenc_t *penc, *penc2;
	struct slice_info *pslice;
	struct cache_dev_t *pcache, *pcache2;
	struct dec400_t *pdec400, *pdec400_2;
	int i, n = get_slicenumber(), k;
	int decn, encn, cachen, dec400n;
	struct slice_info *slice0, *slice1;

	pr_info("########### slice info start #############");
	pr_info("slice num = %d", n);
	slice0 = slicehdr;
	for (i = 0; i < n; i++) {
		pr_info("slice %d:%lx:%llx:%lld:%x", i, (unsigned long)slice0->dev, slice0->rsvmem_addr, slice0->memsize, slice0->config);

		decn = get_slicecorenum(i, CORE_DEC);
		pr_info("dec num = %d", decn);
		encn = get_slicecorenum(i, CORE_ENC);
		pr_info("enc num = %d", encn);
		cachen = get_slicecorenum(i, CORE_CACHE);
		pr_info("cache  num = %d", cachen);
		dec400n = get_slicecorenum(i, CORE_DEC400);
		pr_info("dec400n num = %d", dec400n);

		pdec = get_decnodes(i, 0);
		k = 0;
		while (pdec != NULL) {
			pr_info("dec core %d", k);
			pdec2 = get_decnodes(i, k);
			slice1 = getparentslice(pdec, CORE_DEC);
			if (pdec != pdec2)
				pr_info("get_decnodes fails @ %d", k);
			if (slice0 != slice1)
				pr_info("getparentslice fails @ dec %d:%d", i, k);
			pr_info("addr=%llx, size=%d", pdec->multicorebase, pdec->iosize);
			pr_info("irq0=%d, irq1=%d", pdec->irqlist[0], pdec->irqlist[1]);
			if (pdec->its_main_core_id != NULL) {
				pdec2 = pdec->its_main_core_id;
				pr_info("main core = %d:%d", pdec2->sliceidx, pdec2->core_id);
			}
			if (pdec->its_aux_core_id != NULL) {
				pdec2 = pdec->its_aux_core_id;
				pr_info("aux core = %d:%d", pdec2->sliceidx, pdec2->core_id);
			}
			pdec = pdec->next;
			k++;
		}

		penc = get_encnodes(i, 0);
		k = 0;
		while (penc != NULL) {
			pr_info("enc core %d:", k);
			penc2 = get_encnodes(i, k);
			slice1 = getparentslice(penc, CORE_ENC);
			if (penc != penc2)
				pr_info("get_encnodes fails @ %d", k);
			if (slice0 != slice1)
				pr_info("getparentslice fails @ enc %d:%d", i, k);
			pr_info("addr=%llx, size=%d", penc->core_cfg.base_addr, penc->core_cfg.iosize);
			pr_info("irq0=%d, irq1=%d", penc->irqlist[0], penc->irqlist[1]);
			penc = penc->next;
			k++;
		}

		pcache = get_cachenodes(i, 0);
		k = 0;
		while (pcache != NULL) {
			pr_info("cache core %d:", k);
			pcache2 = get_cachenodes(i, k);
			slice1 = getparentslice(pcache, CORE_CACHE);
			if (pcache != pcache2)
				pr_info("get_cachenodes fails @ %d", k);
			if (slice0 != slice1)
				pr_info("getparentslice fails @ cache %d:%d", i, k);
			pr_info("addr=%llx, size=%d, type=%d, dir=%d", pcache->core_cfg.base_addr,
				pcache->core_cfg.iosize, pcache->core_cfg.client, pcache->core_cfg.dir);
			pr_info("irq0=%d, irq1=%d", pcache->irqlist[0], pcache->irqlist[1]);
			pr_info("parent addr=%llx",  pcache->core_cfg.parentaddr);
			if (pcache->parentcore != NULL) {
				if (pcache->core_cfg.client == VC8000E) {
					penc = (struct hantroenc_t *)pcache->parentcore;
					pr_info("parent enc core = %d:%d,addr %llx", penc->core_cfg.sliceidx,
						penc->core_id, penc->core_cfg.base_addr);
				} else {
					pdec = (struct hantrodec_t *)pcache->parentcore;
					pr_info("parent dec core = %d:%d,addr %llx", pdec->sliceidx,
						pdec->core_id, pdec->multicorebase);
				}
			} else
				pr_info("parent core = NULL");
			pcache = pcache->next;
			k++;
		}
		pdec400 = get_dec400nodes(i, 0);
		k = 0;
		while (pdec400 != NULL) {
			pr_info("dec400 core %d:", k);
			pdec400_2 = get_dec400nodes(i, k);
			slice1 = getparentslice(pdec400, CORE_DEC400);
			if (pdec400 != pdec400_2)
				pr_info("get_dec400nodes fails @ %d", k);
			if (slice0 != slice1)
				pr_info("getparentslice fails @ dec400 %d:%d", i, k);
			pr_info("addr=%llx, size=%d", pdec400->core_cfg.dec400corebase,
				pdec400->core_cfg.iosize);
			pr_info("parent addr=%llx",  pdec400->core_cfg.parentaddr);
			if (pdec400->parentcore != NULL) {
				switch (pdec400->parenttype) {
				case CORE_ENC:
					penc = (struct hantroenc_t *)pdec400->parentcore;
					pr_info("parent enc core = %d:%d,addr %llx", penc->core_cfg.sliceidx,
						penc->core_id, penc->core_cfg.base_addr);
					break;
				case CORE_DEC:
					pdec = (struct hantrodec_t *)pdec400->parentcore;
					pr_info("parent dec core = %d:%d,addr %llx", pdec->sliceidx,
						pdec->core_id, pdec->multicorebase);
					break;
				case CORE_SLICE:
					pslice = (struct slice_info *)pdec400->parentcore;
					pr_info("parent slice addr %llx", pslice->rsvmem_addr);
					break;
				default:
					pr_info("error: dec400 parent type unknown");
					break;
				}
			} else
				pr_info("parent core = NULL");
			pdec400 = pdec400->next;
			k++;
		}
		slice0 = slice0->next;
	}
	pr_info("########### slice info finish #############");
}

