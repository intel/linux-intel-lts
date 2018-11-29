/*
 * Copyright (C) 2018 Intel Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/tee_drv.h>

#include "tpm.h"
#include "tpm_ftpm_optee.h"

static struct platform_device *fwtpm_pdev;

static int fwtpm_optee_match(struct tee_ioctl_version_data *data,
			     const void *vers)
{
	return 1;
}

static int fwtpm_optee_init_session(struct fwtpm_device_priv *priv)
{
	struct tee_ioctl_open_session_arg sess_arg;
	struct tee_ioctl_version_data vers = {
		.impl_id = TEE_OPTEE_CAP_TZ,
		.impl_caps = TEE_IMPL_ID_OPTEE,
		.gen_caps = TEE_GEN_CAP_GP,
	};
	struct teec_uuid uuid = TA_FTPM_UUID;
	int rc = 0;

	if (priv->status & FTPM_SESSION_INITED)
		return rc;
	priv->ctx = tee_client_open_context(NULL, fwtpm_optee_match,
				      NULL, &vers);
	if (IS_ERR(priv->ctx))
		return -EINVAL;

	memset(&sess_arg, 0, sizeof(sess_arg));
	memcpy(&sess_arg.uuid, &uuid, sizeof(uuid));
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;

	rc = tee_client_open_session(priv->ctx, &sess_arg, NULL);
	if (!rc && sess_arg.ret)
		return -EIO;
	if (rc) {
		tee_client_close_context(priv->ctx);
		return rc;
	}
	priv->sess = sess_arg.session;

	priv->cmd_buf_shm = tee_shm_alloc(priv->ctx, TPM_BUFSIZE,
					  TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (!priv->cmd_buf_shm) {
		rc = -ENOMEM;
		goto err_close_sess;
	}

	priv->resp_buf_shm = tee_shm_alloc(priv->ctx, TPM_BUFSIZE,
					   TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (!priv->resp_buf_shm) {
		rc = -ENOMEM;
		goto err_shm_free;
	}

	priv->cmd_buf = tee_shm_get_va(priv->cmd_buf_shm, 0);
	if (IS_ERR(priv->cmd_buf)) {
		rc = -ERESTART;
		goto err_shm_free2;
	}

	priv->resp_buf = tee_shm_get_va(priv->resp_buf_shm, 0);
	if (IS_ERR(priv->resp_buf)) {
		rc = -ERESTART;
		goto err_shm_free2;
	}

	priv->status |= FTPM_SESSION_INITED;
	return rc;

err_shm_free2:
	tee_shm_free(priv->resp_buf_shm);

err_shm_free:
	tee_shm_free(priv->cmd_buf_shm);

err_close_sess:
	tee_client_close_session(priv->ctx, priv->sess);
	tee_client_close_context(priv->ctx);

	return rc;
}

static int fwtpm_optee_cmd(struct fwtpm_device_priv *priv,
			   enum fwtpm_optee_cmd cmd,
			   u8 *buf,
			   size_t count)
{
	struct tee_ioctl_invoke_arg arg;
	struct tee_param param[2];
	struct tpm_header *out_header;
	int rc = 0;
	__be32 len;

	if (!(priv->status & FTPM_SESSION_INITED))
		return -EIO;

	priv->cmd_len = count;
	memcpy(priv->cmd_buf, buf, count);

	memset(&arg, 0, sizeof(arg));
	arg.func = cmd;
	arg.session = priv->sess;
	arg.num_params = 2;

	/*
	 * change cancellation id for each new operation,
	 * it should be a unique value to identify this request
	 */
	arg.cancel_id = ++(priv->cancel_id);
	priv->is_canceled = false;

	memset(param, 0, sizeof(param));

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = priv->cmd_buf_shm;
	param[0].u.memref.size = TPM_BUFSIZE;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[1].u.memref.shm = priv->resp_buf_shm;
	param[1].u.memref.size = TPM_BUFSIZE;

	rc = tee_client_invoke_func(priv->ctx, &arg, param);

	if (!rc && arg.ret)
		return -EIO;

	out_header = (struct tpm_header *)priv->resp_buf;
	len = out_header->length;

	priv->resp_len = be32_to_cpu(len);

	return 0;
}

static int fwtpm_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct fwtpm_device_priv *priv = dev_get_drvdata(&chip->dev);
	int rc = 0;
	size_t len;

	/* sanity check */
	if (count < 6)
		return -EIO;

	len = priv->resp_len;
	if (count < len) {
		dev_err(&chip->dev,
			"Invalid size in recv: count=%zd, resp_len=%zd\n",
			count, len);
		rc = -EIO;
		goto out;
	}

	memcpy(buf, priv->resp_buf, len);
	priv->resp_len = 0;

out:
	return rc ? rc : len;
}

static int fwtpm_send(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct fwtpm_device_priv *priv = dev_get_drvdata(&chip->dev);
	int rc = 0;

	if (!(priv->status & FTPM_SESSION_INITED)) {
		rc = fwtpm_optee_init_session(priv);
		if (rc) {
			dev_err(&chip->dev,
				"Can't init session to swTPM TA\n");
			return -EIO;
		}
	}

	if (fwtpm_optee_cmd(priv, FTPM_HANDLE_CMD_SUBMIT, buf, count)) {
		dev_err(&chip->dev,
			"Sending command to swTPM TA failed\n");
		return -EIO;
	}

	return rc;
}

static void fwtpm_cancel(struct tpm_chip *chip)
{
	struct fwtpm_device_priv *priv = dev_get_drvdata(&chip->dev);
	struct tee_ioctl_cancel_arg arg;

	if (!(priv->status & FTPM_SESSION_INITED)) {
		dev_err(&chip->dev,
			"Session is not inited, nothing to cancel\n");
		return;
	}

	arg.cancel_id = priv->cancel_id;
	arg.session = priv->sess;

	if (tee_client_cancel_req(priv->ctx, &arg)) {
		dev_err(&chip->dev,
			"Cancel request failed\n");
		return;
	}

	priv->is_canceled = true;
}

static u8 fwtpm_status(struct tpm_chip *chip)
{
	return BIT(0);
}

static bool fwtpm_req_canceled(struct tpm_chip *chip, u8 status)
{
	struct fwtpm_device_priv *priv = dev_get_drvdata(&chip->dev);

	return priv->is_canceled;
}

static int fwtpm_suspend(struct device *dev)
{
	struct tpm_chip *chip = dev_get_drvdata(dev);
	struct fwtpm_device_priv *priv = dev_get_drvdata(&chip->dev);
	struct fwtpm_pm_data pm_data;
	int rc = 0;

	if (!(priv->status & FTPM_SESSION_INITED)) {
		rc = fwtpm_optee_init_session(priv);
		if (rc) {
			dev_err(&chip->dev,
				"Can't init session to swTPM TA\n");
			return -EIO;
		}
	}

	pm_data.pm_event = FTPM_PM_SUSPEND;

	if (fwtpm_optee_cmd(priv, FTPM_HANDLE_PM,
			    (u8 *)&pm_data, sizeof(pm_data))) {
		dev_err(&chip->dev,
			"Sending command to swTPM TA failed\n");
		return -EIO;
	}

	return rc;
}

static int fwtpm_resume(struct device *dev)
{
	struct tpm_chip *chip = dev_get_drvdata(dev);
	struct fwtpm_device_priv *priv = dev_get_drvdata(&chip->dev);
	struct fwtpm_pm_data pm_data;
	int rc = 0;

	if (!(priv->status & FTPM_SESSION_INITED)) {
		rc = fwtpm_optee_init_session(priv);
		if (rc) {
			dev_err(&chip->dev,
				"Can't init session to swTPM TA\n");
			return -EIO;
		}
	}

	pm_data.pm_event = FTPM_PM_RESUME;

	if (fwtpm_optee_cmd(priv, FTPM_HANDLE_PM,
			    (u8 *)&pm_data, sizeof(pm_data))) {
		dev_err(&chip->dev,
			"Sending command to swTPM TA failed\n");
		return -EIO;
	}

	return rc;
}

static const struct tpm_class_ops fwtpm_ops = {
	.status = fwtpm_status,
	.recv = fwtpm_recv,
	.send = fwtpm_send,
	.cancel = fwtpm_cancel,
	.req_canceled = fwtpm_req_canceled,
};

static const struct dev_pm_ops fwtpm_pm_ops = {
	.suspend = fwtpm_suspend,
	.resume = fwtpm_resume,
};

static SIMPLE_DEV_PM_OPS(i2c_atmel_pm_ops, tpm_pm_suspend, tpm_pm_resume);

static int fwtpm_probe(struct platform_device *pdev)
{
	struct tpm_chip *chip;
	struct fwtpm_device_priv *priv;
	int rc = 0;

	fwtpm_pdev = platform_device_register_simple("optee_fwtpm",
						     -1, NULL, 0);
	if (IS_ERR(fwtpm_pdev)) {
		rc = PTR_ERR(fwtpm_pdev);
		return rc;
	}

	priv = devm_kzalloc(&fwtpm_pdev->dev, sizeof(struct fwtpm_device_priv),
			    GFP_KERNEL);
	if (!priv) {
		rc = -ENOMEM;
		goto err_unreg_dev;
	}

	chip = tpmm_chip_alloc(&fwtpm_pdev->dev, &fwtpm_ops);
	if (IS_ERR(chip)) {
		rc = PTR_ERR(chip);
		goto err_unreg_dev;
	}

	dev_set_drvdata(&chip->dev, priv);


	rc = tpm_chip_register(chip);
	if (rc)
		goto err_unreg_dev;

	return rc;

err_unreg_dev:
	platform_device_unregister(pdev);

	return rc;
}

static const struct of_device_id fwtpm_of_match[] = {
	{ .compatible = "tcg,fwtpm", },
	{},
};
MODULE_DEVICE_TABLE(of, fwtpm_of_match);

static struct platform_driver fwtpm_pdrv = {
	.probe = fwtpm_probe,
	.driver = {
		.name = "optee_fwtpm;",
		.of_match_table = fwtpm_of_match,
		.pm = &fwtpm_pm_ops,
	},
};

static int __init fwtpm_driver_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&fwtpm_pdrv);
	if (rc)
		return rc;

	return rc;
}

static void __exit fwtpm_driver_exit(void)
{
	platform_driver_unregister(&fwtpm_pdrv);
}

module_init(fwtpm_driver_init);
module_exit(fwtpm_driver_exit);

MODULE_AUTHOR("Igor Opaniuk <igorx.opaniuk@intel.com>");
MODULE_AUTHOR("Vincent T Cao <vincent.t.cao@intel.com>");
MODULE_DESCRIPTION("fTPM 2.0 TEE Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
