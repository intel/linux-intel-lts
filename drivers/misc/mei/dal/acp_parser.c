// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>

#include "acp_format.h"
#include "acp_parser.h"

/* CSS Header + CSS Crypto Block
 * Prefixes each signed ACP package
 */
#define AC_CSS_HEADER_LENGTH_V1 (128 + 520)
#define AC_CSS_HEADER_LENGTH_V2 (128 + 776)

/**
 * struct ac_pr_state - admin command pack reader state
 *
 * @cur   : current read position
 * @head  : acp file head
 * @total : size of acp file
 */
struct ac_pr_state {
	const char *cur;
	const char *head;
	unsigned int total;
};

/**
 * ac_pr_init - init pack reader
 *
 * @pr: pack reader
 * @data: acp file content (without CSS header)
 * @n: acp file size (without CSS header)
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int ac_pr_init(struct ac_pr_state *pr, const char *data,
		      unsigned int n)
{
	/* check integer overflow */
	if ((size_t)data > SIZE_MAX - n)
		return -EINVAL;

	pr->cur = data;
	pr->head = data;
	pr->total = n;
	return 0;
}

/**
 * ac_pr_8b_align_move - update pack reader cur pointer after reading n_move
 *                       bytes. Leave cur aligned to 8 bytes.
 *                       (e.g. when n_move is 3, increase cur by 8)
 *
 * @pr: pack reader
 * @n_move: number of bytes to move cur pointer ahead
 *          will be rounded up to keep cur 8 bytes aligned
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int ac_pr_8b_align_move(struct ac_pr_state *pr, size_t n_move)
{
	unsigned long offset;
	const char *new_cur = pr->cur + n_move;
	size_t len_from_head = new_cur - pr->head;

	if ((size_t)pr->cur > SIZE_MAX - n_move || new_cur < pr->head)
		return -EINVAL;

	offset = ((8 - (len_from_head & 7)) & 7);
	if ((size_t)new_cur > SIZE_MAX - offset)
		return -EINVAL;

	new_cur = new_cur + offset;
	if (new_cur > pr->head + pr->total)
		return -EINVAL;

	pr->cur = new_cur;
	return 0;
}

/**
 * ac_pr_align_move - update pack reader cur pointer after reading n_move bytes
 *                    Leave cur aligned to 4 bytes.
 *                    (e.g. when n_move is 1, increase cur by 4)
 *
 * @pr: pack reader
 * @n_move: number of bytes to move cur pointer ahead
 *          will be rounded up to keep cur 4 bytes aligned
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int ac_pr_align_move(struct ac_pr_state *pr, size_t n_move)
{
	const char *new_cur = pr->cur + n_move;
	size_t len_from_head = new_cur - pr->head;
	size_t offset;

	if ((size_t)pr->cur > SIZE_MAX - n_move || new_cur < pr->head)
		return -EINVAL;

	offset = ((4 - (len_from_head & 3)) & 3);
	if ((size_t)new_cur > SIZE_MAX - offset)
		return -EINVAL;

	new_cur = new_cur + offset;
	if (new_cur > pr->head + pr->total)
		return -EINVAL;

	pr->cur = new_cur;
	return 0;
}

/**
 * ac_pr_move - update pack reader cur pointer after reading n_move bytes
 *
 * @pr: pack reader
 * @n_move: number of bytes to move cur pointer ahead
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int ac_pr_move(struct ac_pr_state *pr, size_t n_move)
{
	const char *new_cur = pr->cur + n_move;

	/* integer overflow or out of acp pkg size */
	if ((size_t)pr->cur > SIZE_MAX - n_move ||
	    new_cur > pr->head + pr->total)
		return -EINVAL;

	pr->cur = new_cur;

	return 0;
}

/**
 * ac_pr_is_safe_to_read - check whether it is safe to read more n_move
 *                         bytes from the acp file
 *
 * @pr: pack reader
 * @n_move: number of bytes to check if it is safe to read
 *
 * Return: true when it is safe to read more n_move bytes
 *         false otherwise
 */
static bool ac_pr_is_safe_to_read(const struct ac_pr_state *pr, size_t n_move)
{
	/* pointer overflow */
	if ((size_t)pr->cur > SIZE_MAX - n_move)
		return false;

	if (pr->cur + n_move > pr->head + pr->total)
		return false;

	return true;
}

/**
 * ac_pr_is_end - check if cur is at the end of the acp file
 *
 * @pr: pack reader
 *
 * Return: true when cur is at the end of the acp
 *         false otherwise
 */
static bool ac_pr_is_end(const struct ac_pr_state *pr)
{
	return (pr->cur == pr->head + pr->total);
}

static unsigned int acp_get_sig_version(const char *data, size_t size)
{
	u32 sig_ver;

	if (size < AC_PACK_SIG_VER_OFFSET + sizeof(sig_ver))
		return AC_SIG_VERSION_NONE;

	sig_ver = *(u32 *)(data + AC_PACK_SIG_VER_OFFSET);

	if (sig_ver == AC_SIG_VERSION_1 || sig_ver == AC_SIG_VERSION_2)
		return sig_ver;

	return AC_SIG_VERSION_NONE;
}

static size_t acp_get_hash_pack_len(unsigned int sig_ver)
{
	switch (sig_ver) {
	case AC_SIG_VERSION_1:
		return AC_PACK_HASH_LEN_V1;
	case AC_SIG_VERSION_2:
		return AC_PACK_HASH_LEN_V2;
	}

	return 0;
}

/* Intel CSS Header + CSS Cypto Block which prefixes each signed ACP pkg */
static int acp_get_css_hdr_len(unsigned int sig_ver)
{
	switch (sig_ver) {
	case AC_SIG_VERSION_1:
		return AC_CSS_HEADER_LENGTH_V1;
	case AC_SIG_VERSION_2:
		return AC_CSS_HEADER_LENGTH_V2;
	}

	return 0;
}

/**
 * acp_load_reasons - load list of event codes that can be
 *                    received or posted by ta
 *
 * @pr: pack reader
 * @reasons: out param to hold the list of event codes
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_reasons(struct ac_pr_state *pr,
			    struct ac_ins_reasons **reasons)
{
	size_t len;
	struct ac_ins_reasons *r;

	if (!ac_pr_is_safe_to_read(pr, sizeof(*r)))
		return -EINVAL;

	r = (struct ac_ins_reasons *)pr->cur;

	if (r->len > AC_MAX_INS_REASONS_LENGTH)
		return -EINVAL;

	len = sizeof(*r) + r->len * sizeof(r->data[0]);
	if (!ac_pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*reasons = r;
	return ac_pr_align_move(pr, len);
}

/**
 * acp_load_taid_list - load list of ta ids which ta is allowed
 *                      to communicate with
 *
 * @pr: pack reader
 * @taid_list: out param to hold the loaded ta ids
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_taid_list(struct ac_pr_state *pr,
			      struct ac_ta_id_list **taid_list)
{
	size_t len;
	struct ac_ta_id_list *t;

	if (!ac_pr_is_safe_to_read(pr, sizeof(*t)))
		return -EINVAL;

	t = (struct ac_ta_id_list *)pr->cur;
	if (t->num > AC_MAX_USED_SERVICES)
		return -EINVAL;

	len = sizeof(*t) + t->num * sizeof(t->list[0]);

	if (!ac_pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*taid_list = t;
	return ac_pr_align_move(pr, len);
}

/**
 * acp_load_prop - load property from acp
 *
 * @pr: pack reader
 * @prop: out param to hold the loaded property
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_prop(struct ac_pr_state *pr, struct ac_prop_list **prop)
{
	size_t len;
	struct ac_prop_list *p;

	if (!ac_pr_is_safe_to_read(pr, sizeof(*p)))
		return -EINVAL;

	p = (struct ac_prop_list *)pr->cur;
	if (p->len > AC_MAX_PROPS_LENGTH)
		return -EINVAL;

	len = sizeof(*p) + p->len * sizeof(p->data[0]);

	if (!ac_pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*prop = p;
	return ac_pr_align_move(pr, len);
}

/**
 * acp_load_ta_pack - load ta pack from acp
 *
 * @pr: pack reader
 * @ta_pack: out param to hold the ta pack
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_ta_pack(struct ac_pr_state *pr, char **ta_pack)
{
	size_t len;
	char *t;

	/*8 byte align to obey jeff rule*/
	if (ac_pr_8b_align_move(pr, 0))
		return -EINVAL;

	t = (char *)pr->cur;

	/*
	 *assume ta pack is the last item of one package,
	 *move cursor to the end directly
	 */
	if (pr->cur > pr->head + pr->total)
		return -EINVAL;

	len = pr->head + pr->total - pr->cur;
	if (!ac_pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*ta_pack = t;
	return ac_pr_move(pr, len);
}

/**
 * acp_load_ins_jta_prop_head - load ta manifest header
 *
 * @pr: pack reader
 * @head: out param to hold manifest header
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_ins_jta_prop_head(struct ac_pr_state *pr,
				      struct ac_ins_jta_prop_header **head)
{
	if (!ac_pr_is_safe_to_read(pr, sizeof(**head)))
		return -EINVAL;

	*head = (struct ac_ins_jta_prop_header *)pr->cur;
	return ac_pr_align_move(pr, sizeof(**head));
}

/**
 * acp_load_ins_jta_prop - load ta properties information (ta manifest)
 *
 * @pr: pack reader
 * @pack: out param to hold ta manifest
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_ins_jta_prop(struct ac_pr_state *pr,
				 struct ac_ins_jta_prop *pack)
{
	int ret;

	ret = acp_load_ins_jta_prop_head(pr, &pack->head);
	if (ret)
		return ret;

	ret = acp_load_reasons(pr, &pack->post_reasons);
	if (ret)
		return ret;

	ret = acp_load_reasons(pr, &pack->reg_reasons);
	if (ret)
		return ret;

	ret = acp_load_prop(pr, &pack->prop);
	if (ret)
		return ret;

	ret = acp_load_taid_list(pr, &pack->used_service_list);

	return ret;
}

/**
 * acp_load_ins_jta_head - load ta installation header
 *
 * @pr: pack reader
 * @head: out param to hold the installation header
 * @hash_size: ta hash size
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_ins_jta_head(struct ac_pr_state *pr,
				 struct ac_ins_ta_header **head,
				 unsigned int hash_size)
{
	size_t hdr_size;

	if (!ac_pr_is_safe_to_read(pr, sizeof(**head)))
		return -EINVAL;

	*head = (struct ac_ins_ta_header *)pr->cur;
	hdr_size = sizeof(**head) + hash_size;
	return ac_pr_align_move(pr, hdr_size);
}

/**
 * acp_load_ins_jta - load ta installation information from acp
 *
 * @pr: pack reader
 * @pack: out param to hold install information
 * @hash_size: ta hash size
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_ins_jta(struct ac_pr_state *pr,
			    struct ac_ins_jta_pack *pack,
			    unsigned int hash_size)
{
	int ret;

	ret = acp_load_prop(pr, &pack->ins_cond);
	if (ret)
		return ret;

	ret = acp_load_ins_jta_head(pr, &pack->head, hash_size);

	return ret;
}

/**
 * acp_load_pack_head - load acp pack header
 *
 * @pr: pack reader
 * @head: out param to hold the acp header
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_pack_head(struct ac_pr_state *pr,
			      struct ac_pack_header **head)
{
	if (!ac_pr_is_safe_to_read(pr, sizeof(**head)))
		return -EINVAL;

	*head = (struct ac_pack_header *)pr->cur;
	return ac_pr_align_move(pr, sizeof(**head));
}

/**
 * acp_load_pack - load and parse pack from acp file
 *
 * @raw_pack: acp file content, without the acp CSS header
 * @size: acp file size (without CSS header)
 * @sig_ver: version of pack signature
 * @cmd_id: command id
 * @pack: out param to hold the loaded pack
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
static int acp_load_pack(const char *raw_pack, unsigned int size,
			 unsigned int sig_ver, unsigned int cmd_id,
			 struct ac_pack *pack)
{
	int ret;
	struct ac_pr_state pr;
	struct ac_ins_jta_pack_ext *pack_ext;
	struct ac_ins_jta_prop_ext *prop_ext;
	unsigned int hash_size;

	ret = ac_pr_init(&pr, raw_pack, size);
	if (ret)
		return ret;

	if (cmd_id != AC_INSTALL_JTA_PROP) {
		ret = acp_load_pack_head(&pr, &pack->head);
		if (ret)
			return ret;
	}

	if (cmd_id != AC_INSTALL_JTA_PROP && cmd_id != pack->head->cmd_id)
		return -EINVAL;

	hash_size = acp_get_hash_pack_len(sig_ver);

	switch (cmd_id) {
	case AC_INSTALL_JTA:
		pack_ext = (struct ac_ins_jta_pack_ext *)pack;
		ret = acp_load_ins_jta(&pr, &pack_ext->cmd_pack, hash_size);
		if (ret)
			break;
		ret = acp_load_ta_pack(&pr, &pack_ext->ta_pack);
		break;
	case AC_INSTALL_JTA_PROP:
		prop_ext = (struct ac_ins_jta_prop_ext *)pack;
		ret = acp_load_ins_jta_prop(&pr, &prop_ext->cmd_pack);
		if (ret)
			break;
		/* Note: the next section is JEFF file,
		 * and not ta_pack(JTA_properties+JEFF file),
		 * but we could reuse the ACP_load_ta_pack() here.
		 */
		ret = acp_load_ta_pack(&pr, &prop_ext->jeff_pack);
		break;
	default:
		return -EINVAL;
	}

	if (!ac_pr_is_end(&pr))
		return -EINVAL;

	return ret;
}

/**
 * acp_pload_ins_jta - load and parse ta pack from acp file
 *
 * Exported function in acp parser API
 *
 * @raw_data: acp file content
 * @size: acp file size
 * @pack: out param to hold the ta pack
 *
 * Return: 0 on success
 *         -EINVAL on invalid parameters
 */
int acp_pload_ins_jta(const void *raw_data, unsigned int size,
		      struct ac_ins_jta_pack_ext *pack)
{
	int ret;
	unsigned int sig_ver;
	unsigned int css_hdr_len;

	if (!raw_data || !pack)
		return -EINVAL;

	sig_ver = acp_get_sig_version(raw_data, size);
	if (sig_ver == AC_SIG_VERSION_NONE)
		return -EINVAL;

	css_hdr_len = acp_get_css_hdr_len(sig_ver);
	if (size < css_hdr_len)
		return -EINVAL;

	ret = acp_load_pack(raw_data + css_hdr_len, size - css_hdr_len,
			    sig_ver, AC_INSTALL_JTA, (struct ac_pack *)pack);

	return ret;
}
