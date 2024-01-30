// SPDX-License-Identifier: MIT
/*
 * Copyright 2021, Intel Corporation.
 */

#include <drm/drm_hdcp.h>
#include <drm/i915_cp_fw_hdcp_interface.h>
#include "i915_drv.h"
#include "gt/uc/intel_gsc_fw.h"
#include "gt/uc/intel_gsc_fwif.h"
#include "gem/i915_gem_region.h"
#include "i915_utils.h"
#include "display/intel_hdcp_gsc.h"

struct intel_hdcp_gsc_message {
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	void *hdcp_cmd;
};

static int
gsc_hdcp_initiate_session(struct device *dev, struct hdcp_port_data *data,
			  struct hdcp2_ake_init *ake_data)
{
	struct wired_cmd_initiate_hdcp2_session_in session_init_in = { { 0 } };
	struct wired_cmd_initiate_hdcp2_session_out
						session_init_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !ake_data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_session_in(&session_init_in, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&session_init_in,
				       sizeof(session_init_in),
				       (u8 *)&session_init_out,
				       sizeof(session_init_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (session_init_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X Failed. Status: 0x%X\n",
			    WIRED_INITIATE_HDCP2_SESSION,
			    session_init_out.header.status);
		return -EIO;
	}

	ake_data->msg_id = HDCP_2_2_AKE_INIT;
	ake_data->tx_caps = session_init_out.tx_caps;
	memcpy(ake_data->r_tx, session_init_out.r_tx, HDCP_2_2_RTX_LEN);

	return 0;
}

static int
gsc_hdcp_verify_receiver_cert_prepare_km(struct device *dev,
					 struct hdcp_port_data *data,
					 struct hdcp2_ake_send_cert *rx_cert,
					 bool *km_stored,
					 struct hdcp2_ake_no_stored_km
								*ek_pub_km,
					 size_t *msg_sz)
{
	struct wired_cmd_verify_receiver_cert_in verify_rxcert_in = { { 0 } };
	struct wired_cmd_verify_receiver_cert_out verify_rxcert_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !rx_cert || !km_stored || !ek_pub_km || !msg_sz)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_rxcert_in(&verify_rxcert_in, rx_cert, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&verify_rxcert_in,
				       sizeof(verify_rxcert_in),
				       (u8 *)&verify_rxcert_out,
				       sizeof(verify_rxcert_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed: %zd\n", byte);
		return byte;
	}

	if (verify_rxcert_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X Failed. Status: 0x%X\n",
			    WIRED_VERIFY_RECEIVER_CERT,
			    verify_rxcert_out.header.status);
		return -EIO;
	}

	*km_stored = !!verify_rxcert_out.km_stored;
	if (verify_rxcert_out.km_stored) {
		ek_pub_km->msg_id = HDCP_2_2_AKE_STORED_KM;
		*msg_sz = sizeof(struct hdcp2_ake_stored_km);
	} else {
		ek_pub_km->msg_id = HDCP_2_2_AKE_NO_STORED_KM;
		*msg_sz = sizeof(struct hdcp2_ake_no_stored_km);
	}

	memcpy(ek_pub_km->e_kpub_km, &verify_rxcert_out.ekm_buff,
	       sizeof(verify_rxcert_out.ekm_buff));

	return 0;
}

static int
gsc_hdcp_verify_hprime(struct device *dev, struct hdcp_port_data *data,
		       struct hdcp2_ake_send_hprime *rx_hprime)
{
	struct wired_cmd_ake_send_hprime_in send_hprime_in = { { 0 } };
	struct wired_cmd_ake_send_hprime_out send_hprime_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !rx_hprime)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_hprime_in(&send_hprime_in, rx_hprime, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&send_hprime_in,
				       sizeof(send_hprime_in),
				       (u8 *)&send_hprime_out,
				       sizeof(send_hprime_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (send_hprime_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X Failed. Status: 0x%X\n",
			    WIRED_AKE_SEND_HPRIME, send_hprime_out.header.status);
		return -EIO;
	}

	return 0;
}

static int
gsc_hdcp_store_pairing_info(struct device *dev, struct hdcp_port_data *data,
			    struct hdcp2_ake_send_pairing_info *pairing_info)
{
	struct wired_cmd_ake_send_pairing_info_in pairing_info_in = { { 0 } };
	struct wired_cmd_ake_send_pairing_info_out pairing_info_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !pairing_info)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_pairing_info_in(&pairing_info_in, pairing_info,
				       data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&pairing_info_in,
				       sizeof(pairing_info_in),
				       (u8 *)&pairing_info_out,
				       sizeof(pairing_info_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (pairing_info_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X failed. Status: 0x%X\n",
			    WIRED_AKE_SEND_PAIRING_INFO,
			    pairing_info_out.header.status);
		return -EIO;
	}

	return 0;
}

static int
gsc_hdcp_initiate_locality_check(struct device *dev,
				 struct hdcp_port_data *data,
				 struct hdcp2_lc_init *lc_init_data)
{
	struct wired_cmd_init_locality_check_in lc_init_in = { { 0 } };
	struct wired_cmd_init_locality_check_out lc_init_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !lc_init_data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_locality_check_in(&lc_init_in, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&lc_init_in, sizeof(lc_init_in),
				       (u8 *)&lc_init_out, sizeof(lc_init_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (lc_init_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X Failed. status: 0x%X\n",
			    WIRED_INIT_LOCALITY_CHECK, lc_init_out.header.status);
		return -EIO;
	}

	lc_init_data->msg_id = HDCP_2_2_LC_INIT;
	memcpy(lc_init_data->r_n, lc_init_out.r_n, HDCP_2_2_RN_LEN);

	return 0;
}

static int
gsc_hdcp_verify_lprime(struct device *dev, struct hdcp_port_data *data,
		       struct hdcp2_lc_send_lprime *rx_lprime)
{
	struct wired_cmd_validate_locality_in verify_lprime_in = { { 0 } };
	struct wired_cmd_validate_locality_out verify_lprime_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !rx_lprime)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_validate_locality_in(&verify_lprime_in, rx_lprime,
					    data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&verify_lprime_in,
				       sizeof(verify_lprime_in),
				       (u8 *)&verify_lprime_out,
				       sizeof(verify_lprime_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (verify_lprime_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X failed. status: 0x%X\n",
			    WIRED_VALIDATE_LOCALITY,
			    verify_lprime_out.header.status);
		return -EIO;
	}

	return 0;
}

static int gsc_hdcp_get_session_key(struct device *dev,
				    struct hdcp_port_data *data,
				    struct hdcp2_ske_send_eks *ske_data)
{
	struct wired_cmd_get_session_key_in get_skey_in = { { 0 } };
	struct wired_cmd_get_session_key_out get_skey_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data || !ske_data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_session_key_in(&get_skey_in, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&get_skey_in, sizeof(get_skey_in),
				       (u8 *)&get_skey_out, sizeof(get_skey_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (get_skey_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X failed. status: 0x%X\n",
			    WIRED_GET_SESSION_KEY, get_skey_out.header.status);
		return -EIO;
	}

	ske_data->msg_id = HDCP_2_2_SKE_SEND_EKS;
	memcpy(ske_data->e_dkey_ks, get_skey_out.e_dkey_ks,
	       HDCP_2_2_E_DKEY_KS_LEN);
	memcpy(ske_data->riv, get_skey_out.r_iv, HDCP_2_2_RIV_LEN);

	return 0;
}

static int
gsc_hdcp_repeater_check_flow_prepare_ack(struct device *dev,
					 struct hdcp_port_data *data,
					 struct hdcp2_rep_send_receiverid_list
							*rep_topology,
					 struct hdcp2_rep_send_ack
							*rep_send_ack)
{
	struct wired_cmd_verify_repeater_in verify_repeater_in = { { 0 } };
	struct wired_cmd_verify_repeater_out verify_repeater_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !rep_topology || !rep_send_ack || !data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_repeater_in(&verify_repeater_in, rep_topology, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&verify_repeater_in,
				       sizeof(verify_repeater_in),
				       (u8 *)&verify_repeater_out,
				       sizeof(verify_repeater_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (verify_repeater_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X failed. status: 0x%X\n",
			    WIRED_VERIFY_REPEATER,
			    verify_repeater_out.header.status);
		return -EIO;
	}

	memcpy(rep_send_ack->v, verify_repeater_out.v,
	       HDCP_2_2_V_PRIME_HALF_LEN);
	rep_send_ack->msg_id = HDCP_2_2_REP_SEND_ACK;

	return 0;
}

static int gsc_hdcp_verify_mprime(struct device *dev,
				  struct hdcp_port_data *data,
				  struct hdcp2_rep_stream_ready *stream_ready)
{
	struct wired_cmd_repeater_auth_stream_req_in *verify_mprime_in;
	struct wired_cmd_repeater_auth_stream_req_out
					verify_mprime_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;
	size_t cmd_size;

	if (!dev || !stream_ready || !data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	cmd_size = struct_size(verify_mprime_in, streams, data->k);
	if (cmd_size == SIZE_MAX)
		return -EINVAL;

	verify_mprime_in = kzalloc(cmd_size, GFP_KERNEL);
	if (!verify_mprime_in)
		return -ENOMEM;

	i915_cp_fw_fill_auth_stream_req_in(verify_mprime_in, stream_ready,
					  cmd_size, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)verify_mprime_in, cmd_size,
				       (u8 *)&verify_mprime_out,
				       sizeof(verify_mprime_out));
	kfree(verify_mprime_in);
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (verify_mprime_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X failed. status: 0x%X\n",
			    WIRED_REPEATER_AUTH_STREAM_REQ,
			    verify_mprime_out.header.status);
		return -EIO;
	}

	return 0;
}

static int gsc_hdcp_enable_authentication(struct device *dev,
					  struct hdcp_port_data *data)
{
	struct wired_cmd_enable_auth_in enable_auth_in = { { 0 } };
	struct wired_cmd_enable_auth_out enable_auth_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_enable_auth_in(&enable_auth_in, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&enable_auth_in,
				       sizeof(enable_auth_in),
				       (u8 *)&enable_auth_out,
				       sizeof(enable_auth_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (enable_auth_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "FW cmd 0x%08X failed. status: 0x%X\n",
			    WIRED_ENABLE_AUTH, enable_auth_out.header.status);
		return -EIO;
	}

	return 0;
}

static int
gsc_hdcp_close_session(struct device *dev, struct hdcp_port_data *data)
{
	struct wired_cmd_close_session_in session_close_in = { { 0 } };
	struct wired_cmd_close_session_out session_close_out = { { 0 } };
	struct drm_i915_private *i915;
	ssize_t byte;

	if (!dev || !data)
		return -EINVAL;

	i915 = kdev_to_i915(dev);
	if (!i915) {
		dev_err(dev, "DRM not initialized, aborting HDCP.\n");
		return -ENODEV;
	}

	i915_cp_fw_fill_close_session_in(&session_close_in, data);
	byte = intel_hdcp_gsc_msg_send(i915, (u8 *)&session_close_in,
				       sizeof(session_close_in),
				       (u8 *)&session_close_out,
				       sizeof(session_close_out));
	if (byte < 0) {
		drm_dbg_kms(&i915->drm, "intel_hdcp_gsc_msg_send failed. %zd\n", byte);
		return byte;
	}

	if (session_close_out.header.status != FW_HDCP_STATUS_SUCCESS) {
		drm_dbg_kms(&i915->drm, "Session Close Failed. status: 0x%X\n",
			    session_close_out.header.status);
		return -EIO;
	}

	return 0;
}

static const struct i915_hdcp_fw_ops gsc_hdcp_ops = {
	.initiate_hdcp2_session = gsc_hdcp_initiate_session,
	.verify_receiver_cert_prepare_km =
				gsc_hdcp_verify_receiver_cert_prepare_km,
	.verify_hprime = gsc_hdcp_verify_hprime,
	.store_pairing_info = gsc_hdcp_store_pairing_info,
	.initiate_locality_check = gsc_hdcp_initiate_locality_check,
	.verify_lprime = gsc_hdcp_verify_lprime,
	.get_session_key = gsc_hdcp_get_session_key,
	.repeater_check_flow_prepare_ack =
				gsc_hdcp_repeater_check_flow_prepare_ack,
	.verify_mprime = gsc_hdcp_verify_mprime,
	.enable_hdcp_authentication = gsc_hdcp_enable_authentication,
	.close_hdcp_session = gsc_hdcp_close_session,
};

int intel_gsc_hdcp_init(struct drm_i915_private *i915)
{
	struct i915_hdcp_fw_master *data;

	data = kzalloc(sizeof(struct i915_hdcp_fw_master), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_lock(&i915->hdcp_comp_mutex);
	i915->hdcp_master = data;
	i915->hdcp_master->fw_dev = i915->drm.dev;
	i915->hdcp_master->ops = &gsc_hdcp_ops;
	mutex_unlock(&i915->hdcp_comp_mutex);

	return 0;
}

int intel_gsc_hdcp_fini(struct drm_i915_private *i915)
{
	/* TODO: decrease GSC f/w reference count. */
	kfree(i915->hdcp_master);
	return 0;
}

/*This function helps allocate memory for the command that we will send to gsc cs */
static int intel_initialize_hdcp_gsc_message(struct drm_i915_private *i915,
		struct intel_hdcp_gsc_message *hdcp_message)
{
	struct intel_gt *gt = i915->media_gt;
	struct drm_i915_gem_object *obj = NULL;
	struct i915_vma *vma = NULL;
	void *cmd;
	int err;

	hdcp_message->obj = NULL;
	hdcp_message->hdcp_cmd = NULL;
	hdcp_message->vma = NULL;

	/* allocate object of one page for HDCP command memory and store it */
	obj = i915_gem_object_create_shmem(gt->i915, PAGE_SIZE);

	if (IS_ERR(obj)) {
		drm_err(&gt->i915->drm, "Failed to allocate HDCP streaming command!\n");
		return PTR_ERR(obj);
	}

	cmd = i915_gem_object_pin_map_unlocked(obj, i915_coherent_map_type(gt->i915, obj, true));
	if (IS_ERR(cmd)) {
		drm_err(&gt->i915->drm, "Failed to map gsc message page!\n");
		err = PTR_ERR(cmd);
		goto out_unpin;
	}

	vma = i915_vma_instance(obj, &gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_unmap;
	}

	err = i915_vma_pin(vma, 0, 0, PIN_GLOBAL);
	if (err)
		goto out_unmap;

	memset(cmd, 0, obj->base.size);

	hdcp_message->obj = obj;
	hdcp_message->hdcp_cmd = cmd;
	hdcp_message->vma = vma;

	return 0;

out_unmap:
	i915_gem_object_unpin_map(obj);
out_unpin:
	i915_gem_object_put(obj);
	return err;
}

static void intel_free_hdcp_gsc_message(struct intel_hdcp_gsc_message *hdcp_message)
{
	struct drm_i915_gem_object *obj = fetch_and_zero(&hdcp_message->obj);

	if (!obj)
		return;

	if (hdcp_message->vma)
		i915_vma_unpin(fetch_and_zero(&hdcp_message->vma));

	i915_gem_object_unpin_map(obj);
	i915_gem_object_put(obj);
	kfree(hdcp_message);
}

static int intel_gsc_send_sync(struct drm_i915_private *i915,
		struct intel_gsc_mtl_header *header, u64 addr,
		size_t msg_out_len)
{
	struct intel_gt *gt = i915->media_gt;
	int ret;

	header->flags = 0;
	ret = intel_gsc_fw_heci_send(&gt->uc.gsc, addr, header->message_size,
			     addr, msg_out_len + sizeof(*header));
	if (ret) {
		drm_err(&i915->drm, "failed to send gsc HDCP msg (%d)\n", ret);
		return ret;
	}
	/*
	 * Checking validity marker for memory sanity
	 */
	if (header->validity_marker != GSC_HECI_VALIDITY_MARKER) {
		drm_err(&i915->drm, "invalid validity marker\n");
		return -EINVAL;
	}

	if (header->status != 0) {
		drm_err(&i915->drm, "header status indicates error %d\n",
			header->status);
		return -EINVAL;
	}

	if (header->flags & INTEL_GSC_MSG_PENDING)
		return -EAGAIN;

	return 0;
}

/*
 * This function can now be used for sending requests and will also handle
 * receipt of reply messages hence no different function of message retrieval
 * is required. We will initialize intel_hdcp_gsc_message structure then add
 * gsc cs memory header as stated in specs after which the normal HDCP payload
 * will follow
 */
ssize_t intel_hdcp_gsc_msg_send(struct drm_i915_private *i915, u8 *msg_in,
				size_t msg_in_len, u8 *msg_out,
				size_t msg_out_len)
{
	struct intel_gt *gt = i915->media_gt;
	struct intel_gsc_mtl_header *header;
	const size_t max_msg_size = PAGE_SIZE - sizeof(*header);
	struct intel_hdcp_gsc_message *hdcp_message;
	u64 addr;
	u32 reply_size;
	int ret, tries = 0;

	if (!intel_uc_uses_gsc_uc(&gt->uc))
		return -ENODEV;

	if (msg_in_len > max_msg_size || msg_out_len > max_msg_size)
		return -ENOSPC;

	hdcp_message = kzalloc(sizeof(*hdcp_message), GFP_KERNEL);

	if (!hdcp_message)
		return -ENOMEM;

	ret = intel_initialize_hdcp_gsc_message(i915, hdcp_message);

	if (ret) {
		drm_err(&i915->drm,
			"Could not initialize hdcp_message\n");
		goto err;
	}

	header = hdcp_message->hdcp_cmd;
	addr = i915_ggtt_offset(hdcp_message->vma);

	memset(header, 0, sizeof(*header));
	header->validity_marker = GSC_HECI_VALIDITY_MARKER;
	header->gsc_address = HECI_MEADDRESS_HDCP;
	header->host_session_handle = 0;
	header->header_version = MTL_GSC_HEADER_VERSION;
	header->message_size = msg_in_len + sizeof(*header);

	memcpy(hdcp_message->hdcp_cmd + sizeof(*header), msg_in, msg_in_len);

	/*
	 * Keep sending request in case the pending bit is set no need to add
	 * message handle as we are using same address hence loc. of header is
	 * same and it will contain the message handle. we will send the message
	 * 20 times each message 50 ms apart
	 */
	do {
		ret = intel_gsc_send_sync(i915, header, addr, msg_out_len);

		/* Only try again if gsc says so */
		if (ret != -EAGAIN)
			break;

		msleep(50);

	} while (++tries < 20);

	if (ret)
		goto err;

	/* we use the same mem for the reply, so header is in the same loc */
	reply_size = header->message_size - sizeof(*header);
	if (reply_size > msg_out_len) {
		drm_warn(&i915->drm, "caller with insufficient HDCP reply size %u (%d)\n",
			 reply_size, (u32)msg_out_len);
		reply_size = msg_out_len;
	} else if (reply_size != msg_out_len) {
		drm_dbg_kms(&i915->drm, "caller unexpected HCDP reply size %u (%d)\n",
			reply_size, (u32)msg_out_len);
	}

	memcpy(msg_out, hdcp_message->hdcp_cmd + sizeof(*header), msg_out_len);

err:
	intel_free_hdcp_gsc_message(hdcp_message);
	return ret;
}
