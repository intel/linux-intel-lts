// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay SCMI mailbox driver.
 *
 * Copyright (c) 2019 Intel Corporation.
 */

#include <linux/arm-smccc.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

/* Function id of SiP service */
#define KMB_SIP_SVC_SCMI (0xFF19)

/*
 * Number of channels this mailbox supports: 1 channel,
 * between AP and SCP.
 */
#define NUM_CHANNELS (1)

/* How long to wait before triggering the mailbox receive event */
#define NOTIFY_WAIT_TIME_NS (50)

/**
 * struct keembay_scmi_mbox
 * @mbox:	Mailbox controller struct
 * @dev:	Platform device
 * @shmem_res:	Resource describing memory region shared between secure and
 *		non-secure world
 * @notify_hrt:	Timer to asynchronously trigger a mbox received data event
 */
struct keembay_scmi_mbox {
	struct mbox_controller mbox;
	struct device *dev;
	struct resource shmem_res;
	struct hrtimer notify_hrt;
};

static int keembay_scmi_request(u64 base_address)
{
	struct arm_smccc_res res;
	u64 function_id;
	u16 function_number = KMB_SIP_SVC_SCMI;

	function_id = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_32,
					 ARM_SMCCC_OWNER_SIP, function_number);

	arm_smccc_smc(function_id, base_address, 0, 0, 0, 0, 0, 0, &res);

	return (int)res.a0;
}

static enum hrtimer_restart keembay_scmi_async_notify(struct hrtimer *hrtimer)
{
	struct keembay_scmi_mbox *mbox =
		container_of(hrtimer, struct keembay_scmi_mbox, notify_hrt);
	struct mbox_chan *chan = &mbox->mbox.chans[0];

	mbox_chan_received_data(chan, NULL);

	return HRTIMER_NORESTART;
}

static int keembay_scmi_mailbox_send_data(struct mbox_chan *chan, void *data)
{
	struct keembay_scmi_mbox *mbox =
		(struct keembay_scmi_mbox *)chan->con_priv;
	struct device *dev = mbox->dev;
	int rc;

	/*
	 * Handle case where timer is still on and a new message arrives.
	 * We only have one timer, if it were to happen that a second
	 * request came in and we failed to respond as expected to the
	 * first, the caller's state machine may end up in an unexpected
	 * state.
	 */
	if (hrtimer_active(&mbox->notify_hrt)) {
		dev_warn(dev, "Mailbox was busy when request arrived.\n");
		return -EBUSY;
	}

	rc = keembay_scmi_request((u64)mbox->shmem_res.start);
	if (rc < 0) {
		dev_warn(dev, "Failed to send message to SCP: %d\n", rc);
		return rc;
	}

	/*
	 * If there is an asynchronous interrupt pending, trigger it
	 * via timer. We will know that, because secure world will
	 * respond with > 0 return value.
	 */
	if (rc) {
		rc = 0;

		hrtimer_start(&mbox->notify_hrt,
			      ns_to_ktime(NOTIFY_WAIT_TIME_NS),
			      HRTIMER_MODE_REL);
	}

	return rc;
}

static bool keembay_scmi_mailbox_last_tx_done(struct mbox_chan *chan)
{
	return true;
}

static int keembay_scmi_mailbox_startup(struct mbox_chan *chan)
{
	struct keembay_scmi_mbox *mbox =
		(struct keembay_scmi_mbox *)chan->con_priv;

	hrtimer_init(&mbox->notify_hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mbox->notify_hrt.function = keembay_scmi_async_notify;

	return 0;
}

static const struct mbox_chan_ops scmi_mbox_ops = {
	.startup = keembay_scmi_mailbox_startup,
	.send_data = keembay_scmi_mailbox_send_data,
	.last_tx_done = keembay_scmi_mailbox_last_tx_done,
};

static int keembay_scmi_get_shmem_res(struct device *dev, struct resource *res)
{
	struct device_node *node;
	int rc;

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(dev, "Couldn't find region.\n");
		return -EINVAL;
	}

	rc = of_address_to_resource(node, 0, res);
	of_node_put(node);
	if (rc) {
		dev_err(dev, "Couldn't resolve region.\n");
		return rc;
	}

	return 0;
}

static int keembay_scmi_mailbox_probe(struct platform_device *pdev)
{
	struct keembay_scmi_mbox *scmi_mbox;
	struct device *dev = &pdev->dev;
	int ret;

	dev_info(dev, "Keem Bay SCMI mailbox driver probed.\n");

	scmi_mbox = devm_kzalloc(dev, sizeof(*scmi_mbox), GFP_KERNEL);
	if (!scmi_mbox)
		return -ENOMEM;

	ret = keembay_scmi_get_shmem_res(dev, &scmi_mbox->shmem_res);
	if (ret) {
		dev_err(dev, "Failed to get SCMI shared region resource.\n");
		return -ENOMEM;
	}

	scmi_mbox->mbox.dev = dev;
	scmi_mbox->mbox.txdone_poll = true;
	scmi_mbox->mbox.txpoll_period = 5;
	scmi_mbox->mbox.ops = &scmi_mbox_ops;
	scmi_mbox->mbox.num_chans = NUM_CHANNELS;
	scmi_mbox->mbox.chans = devm_kcalloc(
		dev, NUM_CHANNELS, sizeof(*scmi_mbox->mbox.chans), GFP_KERNEL);
	if (!scmi_mbox->mbox.chans)
		return -ENOMEM;
	scmi_mbox->mbox.chans[0].con_priv = (void *)scmi_mbox;

	ret = devm_mbox_controller_register(dev, &scmi_mbox->mbox);
	if (ret)
		return ret;

	scmi_mbox->dev = dev;

	platform_set_drvdata(pdev, scmi_mbox);

	return 0;
}

static const struct of_device_id keembay_scmi_mailbox_of_match[] = {
	{
		.compatible = "intel,keembay-scmi-mailbox",
	},
	{}
};

static struct platform_driver keembay_scmi_mailbox_driver = {
	.driver = {
			.name = "keembay-scmi-mailbox",
			.of_match_table = keembay_scmi_mailbox_of_match,
		},
	.probe = keembay_scmi_mailbox_probe,
};
module_platform_driver(keembay_scmi_mailbox_driver);

MODULE_DESCRIPTION("Keem Bay SCMI mailbox driver");
MODULE_AUTHOR("Paul Murphy <paul.j.murphy@intel.com>");
MODULE_LICENSE("GPL v2");
