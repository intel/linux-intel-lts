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

#include "hantro_priv.h"


/*currently we simply not using dynamic chains*/
static struct slice_info slicehdr[MULTI_SLICE_LIMIT];
static struct mutex slice_mutex;

int get_slicecorenum(u32 sliceindex, slice_coretype type)
{
	if (sliceindex >= MULTI_SLICE_LIMIT)
		return 0;

	switch (type) {
	case CORE_ENC:
		return slicehdr[sliceindex].enccore_num;
	case CORE_DEC:
		return slicehdr[sliceindex].deccore_num;
	case CORE_CACHE:
		return slicehdr[sliceindex].cachecore_num;
	case CORE_DEC400:
		return slicehdr[sliceindex].dec400core_num;
	default:
		return 0;
	}
}

/*get dec nodes list hdr of a slice*/
struct hantrodec_t *get_decnodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct hantrodec_t *p;
	
	if(sliceindex >= MULTI_SLICE_LIMIT)
		return NULL;

	mutex_lock(&slice_mutex);
	p = slicehdr[sliceindex].dechdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	mutex_unlock(&slice_mutex);
	return p;
}

/*get enc nodes list hdr of a slice*/
struct hantroenc_t *get_encnodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct hantroenc_t *p;
	
	if(sliceindex >= MULTI_SLICE_LIMIT)
		return NULL;

	mutex_lock(&slice_mutex);
	p = slicehdr[sliceindex].enchdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	mutex_unlock(&slice_mutex);
	return p;
}

/*get cache nodes list hdr of a slice*/
struct cache_dev_t *get_cachenodes(u32 sliceindex, u32 nodeidx)
{
	int i;
	struct cache_dev_t *p;
	
	if (sliceindex >= MULTI_SLICE_LIMIT)
		return NULL;

	mutex_lock(&slice_mutex);
	p = slicehdr[sliceindex].cachehdr;
	for (i = 0; i < nodeidx; i++) {
		if (p == NULL)
			break;
		p = p->next;
	}
	mutex_unlock(&slice_mutex);
	return p;
}

/*get dec400 nodes list hdr of a slice*/
/*fixme: after merge dec400 code*/
void *get_dec400nodes(u32 sliceindex)
{
	if (sliceindex >= MULTI_SLICE_LIMIT)
		return NULL;

	return slicehdr[sliceindex].enchdr;
}

int add_decnode(u32 sliceindex, struct hantrodec_t *deccore)
{
	int c;
	struct slice_info *splice;
	struct hantrodec_t *pdec;
	struct cache_dev_t *pcache;
	
	if (sliceindex >= MULTI_SLICE_LIMIT)
		return -EINVAL;

	mutex_lock(&slice_mutex);
	splice = &slicehdr[sliceindex];
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

	deccore->cachecore[0] = deccore->cachecore[1] = NULL;
	deccore->dec400 = NULL;
	pcache = splice->cachehdr;
	c = 0;
	while (pcache != NULL) {
		if (pcache->core_cfg.parentaddr == (unsigned long long)deccore->multicorebase) {
			deccore->cachecore[c] = pcache;
			pcache->parentcore = deccore;
			c++;
		}
		pcache = pcache->next;
		if (c >= 2)
			break;
	}
	sema_init(&splice->dec_core_sem, splice->deccore_num);
	mutex_unlock(&slice_mutex);
	return 0;
}

int add_encnode(u32 sliceindex, struct hantroenc_t *enccore)
{
	int c;
	struct slice_info *splice;
	struct hantroenc_t *penc;
	struct cache_dev_t *pcache;

	if(sliceindex >= MULTI_SLICE_LIMIT)
		return -EINVAL;

	mutex_lock(&slice_mutex);
	splice = &slicehdr[sliceindex];
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

	pcache = splice->cachehdr;
	c = 0;
	while (pcache != NULL) {
		if (pcache->core_cfg.parentaddr == (unsigned long long)enccore->core_cfg.base_addr) {
			enccore->cachecore[c] = pcache;
			pcache->parentcore = enccore;
			c++;
		}
		pcache = pcache->next;
		if (c >= 2)
			break;
	}
	mutex_unlock(&slice_mutex);
	return 0;
}

int add_dec400node(u32 sliceindex, void *dec400core)
{
	if(sliceindex >= MULTI_SLICE_LIMIT)
		return -EINVAL;
	return 0;
}

int add_cachenode(u32 sliceindex, struct cache_dev_t *cachecore)
{
	struct slice_info *splice;
	struct cache_dev_t *pcache;
	struct hantrodec_t *pdec;
	struct hantroenc_t *penc;

	if(sliceindex >= MULTI_SLICE_LIMIT)
		return -EINVAL;

	mutex_lock(&slice_mutex);
	splice = &slicehdr[sliceindex];
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

	if(cachecore->core_cfg.client == VC8000E) {
		penc = splice->enchdr;
		while (penc != NULL) {
			if((unsigned long long)penc->core_cfg.base_addr == cachecore->core_cfg.parentaddr) {
				cachecore->parentcore = penc;
				if(penc->cachecore[0] == NULL)
					penc->cachecore[0] = cachecore;
				else
					penc->cachecore[1] = cachecore;
				break;
			}
			penc = penc->next;
		}
	} else {
		pdec = splice->dechdr;
		while (pdec != NULL) {
			if((unsigned long long)pdec->multicorebase == cachecore->core_cfg.parentaddr) {
				cachecore->parentcore = pdec;
				if(pdec->cachecore[0] == NULL)
					pdec->cachecore[0] = cachecore;
				else
					pdec->cachecore[1] = cachecore;
				break;
			}
			pdec = pdec->next;
		}
	}
	mutex_unlock(&slice_mutex);
	return 0;
}

int get_slicenumber(void)
{
	int i, c = 0;

	mutex_lock(&slice_mutex);
	for (i = 0; i < MULTI_SLICE_LIMIT; i++) {
		if (slicehdr[i].deccore_num != 0 || slicehdr[i].enccore_num != 0)
			c++;
	}
	mutex_unlock(&slice_mutex);
	return c;
}

struct slice_info *getparentslice(void *node, int type)
{
	switch (type) {
	case CORE_CACHE:
		return &slicehdr[((struct cache_dev_t *)node)->core_cfg.sliceidx];
	case CORE_DEC:
		return &slicehdr[((struct hantrodec_t *)node)->sliceidx];
	case CORE_ENC:
		return &slicehdr[((struct hantroenc_t *)node)->core_cfg.sliceidx];
	case CORE_DEC400:
	default:
		return NULL;
	}
}

/*for driver unload*/
int slice_remove(void)
{
	int i;

	for (i = 0; i < MULTI_SLICE_LIMIT; i++) {
		slicehdr[i].deccore_num = slicehdr[i].enccore_num = slicehdr[i].cachecore_num = slicehdr[i].dec400core_num = 0;
		slicehdr[i].dechdr = NULL;
		slicehdr[i].enchdr = NULL;
		slicehdr[i].cachehdr = NULL;
		slicehdr[i].dec400hdr = NULL;
	}
	return 0;
}

/*for driver load*/
int slice_init(void)
{
	int i;

	for (i = 0; i < MULTI_SLICE_LIMIT; i++) {
		slicehdr[i].deccore_num = slicehdr[i].enccore_num = slicehdr[i].cachecore_num = slicehdr[i].dec400core_num = 0;
		slicehdr[i].dechdr = NULL;
		slicehdr[i].enchdr = NULL;
		slicehdr[i].cachehdr = NULL;
		slicehdr[i].dec400hdr = NULL;

		init_waitqueue_head(&slicehdr[i].cache_hw_queue);
		init_waitqueue_head(&slicehdr[i].cache_wait_queue);
		spin_lock_init(&slicehdr[i].cache_owner_lock);

		sema_init(&slicehdr[i].enc_core_sem, 1);
		init_waitqueue_head(&slicehdr[i].enc_hw_queue);
		spin_lock_init(&slicehdr[i].enc_owner_lock);
		init_waitqueue_head(&slicehdr[i].enc_wait_queue);

		slicehdr[i].dec_irq = 0;
		slicehdr[i].pp_irq = 0;
		spin_lock_init(&slicehdr[i].owner_lock);
		init_waitqueue_head(&slicehdr[i].dec_wait_queue);
		init_waitqueue_head(&slicehdr[i].pp_wait_queue);
		init_waitqueue_head(&slicehdr[i].hw_queue);
		sema_init(&slicehdr[i].pp_core_sem, 1);
		/*dec_core_sem could only be initialized after all dec core be inserted*/
	}
	mutex_init(&slice_mutex);
	return 0;
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
#ifdef DEBUG
	struct hantrodec_t *pdec, *pdec2;
	struct hantroenc_t *penc, *penc2;
	struct cache_dev_t *pcache, *pcache2;
	int i, n = get_slicenumber(), k;
	int decn, encn, cachen;
	struct slice_info *slice0, *slice1;

	printk("########### slice info start #############");
	printk("slice num = %d", n);
	for (i = 0; i < n; i++) {
		printk("slice %d:", i);
		slice0 = &slicehdr[i];
		decn = get_slicecorenum(i, CORE_DEC);
		printk("dec num = %d", decn);
		encn = get_slicecorenum(i, CORE_ENC);
		printk("enc num = %d", encn);
		cachen = get_slicecorenum(i, CORE_CACHE);
		printk("cache  num = %d", cachen);

		pdec = get_decnodes(i, 0);
		k = 0;
		while (pdec != NULL) {
			printk("dec core %d", k);
			pdec2 = get_decnodes(i, k);
			slice1 = getparentslice(pdec, CORE_DEC);
			if (pdec != pdec2)
				printk("get_decnodes fails @ %d", k);
			if (slice0 != slice1)
				printk("getparentslice fails @ dec %d:%d", i, k);
			printk("addr=%lx, size=%d", pdec->multicorebase, pdec->iosize);
			printk("irq0=%d, irq1=%d", pdec->irqlist[0], pdec->irqlist[1]);
			if (pdec->its_main_core_id != NULL) {
				pdec2 = pdec->its_main_core_id;
				printk("main core = %d:%d", pdec2->sliceidx, pdec2->core_id);
			}
			if (pdec->its_aux_core_id != NULL) {
				pdec2 = pdec->its_aux_core_id;
				printk("aux core = %d:%d", pdec2->sliceidx, pdec2->core_id);
			}
			if (pdec->cachecore[0] != NULL) {
				pcache = pdec->cachecore[0];
				printk("cache 0 = %d:%lx", pcache->core_id, pcache->core_cfg.base_addr);
			}
			if (pdec->cachecore[1] != NULL) {
				pcache = pdec->cachecore[1];
				printk("cache 1 = %d:%lx", pcache->core_id, pcache->core_cfg.base_addr);
			}
			pdec = pdec->next;
			k++;
		}
		
		penc = get_encnodes(i, 0);
		k = 0;
		while(penc != NULL) {
			printk("enc core %d:", k);
			penc2 = get_encnodes(i, k);
			slice1 = getparentslice(penc, CORE_ENC);
			if (penc != penc2)
				printk("get_encnodes fails @ %d", k);
			if (slice0 != slice1)
				printk("getparentslice fails @ enc %d:%d", i, k);
			printk("addr=%lx, size=%d", penc->core_cfg.base_addr, penc->core_cfg.iosize);
			printk("irq0=%d, irq1=%d", penc->irqlist[0], penc->irqlist[1]);
			if (penc->cachecore[0] != NULL) {
				pcache = penc->cachecore[0];
				printk("cache 0 = %d:%lx", pcache->core_id, pcache->core_cfg.base_addr);
			}			
			if (penc->cachecore[1] != NULL) {
				pcache = penc->cachecore[1];
				printk("cache 1 = %d:%lx", pcache->core_id, pcache->core_cfg.base_addr);
			}			
			penc = penc->next;
			k++;
		}

		pcache = get_cachenodes(i, 0);
		k = 0;
		while(pcache != NULL) {
			printk("cache core %d:", k);
			pcache2 = get_cachenodes(i, k);
			slice1 = getparentslice(pcache, CORE_CACHE);
			if (pcache != pcache2)
				printk("get_cachenodes fails @ %d", k);
			if (slice0 != slice1)
				printk("getparentslice fails @ cache %d:%d", i, k);
			printk("addr=%lx, size=%d, type=%d, dir=%d", pcache->core_cfg.base_addr,
				pcache->core_cfg.iosize, pcache->core_cfg.client, pcache->core_cfg.dir);
			printk("irq0=%d, irq1=%d", pcache->irqlist[0], pcache->irqlist[1]);
			printk("parent addr=%llx",  pcache->core_cfg.parentaddr);
			if (pcache->parentcore != NULL) {
				if(pcache->core_cfg.client == VC8000E) {
					penc = (struct hantroenc_t *)pcache->parentcore;
					printk("parent enc core = %d:%d,addr %lx", penc->core_cfg.sliceidx,
						penc->core_id, penc->core_cfg.base_addr);
				} else {
					pdec = (struct hantrodec_t *)pcache->parentcore;
					printk("parent dec core = %d:%d,addr %lx", pdec->sliceidx,
						pdec->core_id, pdec->multicorebase);
				}
			} else
				printk("parent core = NULL");
			pcache = pcache->next;
			k++;
		}		
	}
	printk("########### slice info finish #############");
#endif	
}

