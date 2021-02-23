// SPDX-License-Identifier: GPL-2.0-only
/*  Copyright (C) 2020 Intel Corporation
 *
 *  Sec Class: Intel Confidential (IC)
 *
 *  All rights reserved.
 *
 *  This document contains proprietary information belonging to Intel.
 *  Passing on and copying of this document, use and
 *  communication of its contents is not permitted without prior written/
 *  authorisation.
 *
 *  Purpose: KMB NOC probe bandwidth measurement interface
 *
 *
 */

#include <linux/types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/compiler_types.h>
#include <linux/arm-smccc.h>
#include "noc_driver.h"

/* NOC Control Registers */
#define ID_COREID          0x0000 /* MAIN_PROBE_ID_COREID */
#define ID_REVISIONID      0x0004 /* MAIN_PROBE_ID_REVISIONID */
#define MAINCTL            0x0008 /* Probe global control bits */
#define CFGCTL             0x000C /* MAIN_PROBE_CFGCTL */
#define TRACEPORTSEL       0x0010 /* MAIN_PROBE_TRACEPORTSEL */
#define FILTERLUT          0x0014 /* MAIN_PROBE_FILTERLUT */
#define TRACEALARMEN       0x0018 /* MAIN_PROBE_TRACEALARMEN */
#define TRACEALARMSTATUS   0x001C /* MAIN_PROBE_TRACEALARMSTATUS */
#define TRACEALARMCLR      0x0020 /* MAIN_PROBE_TRACEALARMCLR */
#define STATPERIOD         0x0024 /* MAIN_PROBE_STATPERIOD */
#define STATGO             0x0028 /* MAIN_PROBE_STATGO */
#define STATALARMMIN       0x002C /* MAIN_PROBE_STATALARMMIN */
#define STATALARMMAX       0x0030 /* MAIN_PROBE_STATALARMMAX */
#define STATALARMSTATUS    0x0034 /* MAIN_PROBE_STATALARMSTATUS */
#define STATALARMCLR       0x0038 /* MAIN_PROBE_STATALARMCLR */
#define STATALARMEN        0x003C /* MAIN_PROBE_STATALARMEN */

/* NOC Filter Registers */
#define F_ROUTEIDBASE      0x0000 /* Offset for FILTERS_n_ROUTEIDBASE */
#define F_ROUTEIDMASK      0x0004 /* Offset for FILTERS_n_ROUTEIDMASK */
#define F_ADDRBASE_LOW     0x0008 /* Offset for FILTERS_n_ADDRBASE_LOW */
#define F_ADDRBASE_HIGH    0x000C /* Offset for FILTERS_n_ADDRBASE_HIGH */
#define F_WINDOWSIZE       0x0010 /* Offset for FILTERS_n_WINDOWSIZE */
#define F_OPCODE           0x001C /* Offset for FILTERS_n_OPCODE */
#define F_STATUS           0x0020 /* Offset for FILTERS_n_STATUS */
#define F_LENGTH           0x0024 /* Offset for FILTERS_n_LENGTH */
#define F_URGENCY          0x0028 /* Offset for FILTERS_n_URGENCY */
#define F_USERBASE         0x002C /* Offset for FILTERS_n_USERBASE */
#define F_USERMASK         0x0030 /* Offset for FILTERS_n_USERMASK */

/* NOC Counter Registers */
#define C_PORTSEL         0x0000 /* Offset for COUNTERS_n_PORTSEL */
#define C_SRC             0x0004 /* Offset for COUNTERS_n_SRC */
#define C_ALARMMODE       0x0008 /* Offset for COUNTERS_n_ALARMMODE */
#define C_VAL             0x000C /* Offset for COUNTERS_n_VAL */

#define PLATFORM_SIP_SVC_DSS_NOC_PROBE_READ         (0x8200ff28)
#define PLATFORM_SIP_SVC_DSS_NOC_PROBE_WRITE        (0x8200ff29)

/* Following information should come from DTSI */
int f_offset[] = {0x44, 0x80, 0xbc, 0xf8}; //Filter_offsets
int c_offset[] = {0x134, 0x148, 0x15c, 0x170}; //Counter offsets

static inline u32 noc_readl(u32 offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(PLATFORM_SIP_SVC_DSS_NOC_PROBE_READ, offset,
			0, 0, 0, 0, 0, 0, &res);
	return res.a1;
}

static inline void noc_writel(u32 offset, u32 value)
{
	struct arm_smccc_res res;

	arm_smccc_smc(PLATFORM_SIP_SVC_DSS_NOC_PROBE_WRITE, offset, value,
			0, 0, 0, 0, 0, &res);
}


int flex_noc_setup(enum noc_type noc, enum noc_counter counter, int trace_port)
{
	/* Validate params */
	if ((noc >= NOC_TYPE_MAX) || (counter >= NOC_COUNTER_MAX))
		return -EINVAL;

	//Stop ongoing stats
	noc_writel(MAINCTL, 0); //MAINCTL.StatEn = 0
	noc_writel(CFGCTL, 0); //CFGCTL.GlobalEn = 0

	//Setup trace port
	noc_writel(TRACEPORTSEL, trace_port);
	//COUNTERS_0_PORTSEL
	noc_writel((c_offset[counter] + C_PORTSEL), trace_port);
	//COUNTERS_1_PORTSEL
	noc_writel((c_offset[counter+1] + C_PORTSEL), trace_port);

	//Setup counter sources & triggers
	//COUNTERS_0_SRC (8 - Count BYTES 14 - FiltBytes)
	noc_writel((c_offset[counter] + C_SRC), 0x00000014);
	//COUNTERS_0_ALARMMODE - OFF
	noc_writel((c_offset[counter] + C_ALARMMODE), 0x00000002);
	//COUNTERS_1_SRC  (Carry from C0)
	noc_writel((c_offset[counter+1] + C_SRC), 0x00000010);
	//COUNTERS_1_ALARMMODE - MAX
	noc_writel((c_offset[counter+1] + C_ALARMMODE), 0x00000002);

	//Setup filters
	// Always Filter 0 is used for all counters ?
	//Filter 0 Route_id_base
	noc_writel((f_offset[counter/2] + F_ROUTEIDBASE), 0);
	//Filter 0 Route_id_mask
	noc_writel((f_offset[counter/2] + F_ROUTEIDMASK), 0);
	//Filter 0 Address Base Low
	noc_writel((f_offset[counter/2] + F_ADDRBASE_LOW), 0);
	//Filter 0 Address Base High
	noc_writel((f_offset[counter/2] + F_ADDRBASE_HIGH), 0);
	//Filter 0 window size
	noc_writel((f_offset[counter/2] + F_WINDOWSIZE), 0xffffffff);
	//Filter 0 opcode=0xF
	noc_writel((f_offset[counter/2] + F_OPCODE), 0xF);
	//Filter 0 status
	noc_writel((f_offset[counter/2] + F_STATUS), 0x3);
	//Filter 0 length
	noc_writel((f_offset[counter/2] + F_LENGTH), 0xF);

	return 0;
}
EXPORT_SYMBOL_GPL(flex_noc_setup);

enum noc_status flexnoc_probe_start(enum noc_type noc, int capture_time)
{
	int statperiod;

	if (noc >= NOC_TYPE_MAX)
		return NOC_PROBE_ERR_INVALID_ARGS;

	//TBD : Convert capture_time in msec to statperiod;
	statperiod = 0x1B;

	noc_writel(MAINCTL, 0x00000008); //MAINCTL.StatEn = 1
	noc_writel(FILTERLUT, 0x00000001); //FilterLUT = 1

	noc_writel(STATALARMMIN, 0x00000000); //STATALARMMIN
	//STATALARMMAX - Saturation value
	noc_writel(STATALARMMAX, 0x00000001);
	noc_writel(STATALARMEN, 0x00000001); //STATALARMEN

	//MAINCTL .StatEn = 1; .AlarmEn = 1 FiltByteAlwaysChainableEn = 1
	noc_writel(MAINCTL, 0x00000098);
	// STATPERIOD - 2**duration cycles
	noc_writel(STATPERIOD, statperiod);
	noc_writel(FILTERLUT, 0x00000001); //FilterLUT = 1
	noc_writel(CFGCTL, 0x00000001); //CFGCTL.GlobalEn = 1

	return NOC_PROBE_CAPTURE_STARTED;
}
EXPORT_SYMBOL_GPL(flexnoc_probe_start);

enum noc_status flexnoc_counter_capture(enum noc_type noc,
			enum noc_counter counter, u32  *value)
{
	u32 c0_0, c0_1;
	unsigned long j0,j1,delay;

	if ((noc >= NOC_TYPE_MAX) || (counter >= NOC_COUNTER_MAX)
		|| (NULL == value))
		return NOC_PROBE_ERR_INVALID_ARGS;

	delay = msecs_to_jiffies(2000);
	j0 = jiffies;
	j1 = j0 + delay;

	while (time_before(jiffies, j1)) {
		c0_0 = noc_readl((c_offset[counter] + C_VAL));
		usleep_range(10000, 11000);
		c0_1 = noc_readl((c_offset[counter] + C_VAL));


		/* If mainctrl is zero , return error */
		if (0 == noc_readl(MAINCTL))
			return NOC_PROBE_ERR_IN_PROGRESS;

		/* If counters are zero, keep reading */
		if (0 == c0_0 && 0 == c0_1) {
			break;
		}
		else if(c0_0 != c0_1) {
			continue;
		}
		else {
			/* Couters look good break the while */
			break;
		}
	}

	if (c0_0 != c0_1)
		return NOC_PROBE_ERR_IN_PROGRESS;

	c0_0 = noc_readl((c_offset[counter] + C_VAL));
	c0_1 = noc_readl((c_offset[counter+1] + C_VAL));

	*value = (c0_0 | (c0_1 << 16));

	return NOC_PROBE_COMPLETED;
}
EXPORT_SYMBOL_GPL(flexnoc_counter_capture);

static int noc_probe(struct platform_device *pdev)
{
	struct noc_device *cdev;
	int ret;

	/*  Char dev init */
	cdev = devm_kzalloc(&pdev->dev,
			sizeof(struct noc_device), GFP_KERNEL);
	ret = intel_noc_cdev_init(cdev);
	if (ret) {
		dev_err(&pdev->dev, "NOC char device init failed\n");
		devm_kfree(&pdev->dev, cdev);
		return -EINVAL;
	}

	return 0;
}

static int noc_remove(struct platform_device *pdev)
{
	//TBD : Call char device remove
	return 0;
}


static struct platform_driver noc_driver = {
	.probe = noc_probe,
	.remove = noc_remove,
	.driver = {
		.name = "noc",
	},
};


//Module init changes starts form here
static struct platform_device *intel_noc_pdev;

static int __init noc_driver_module_init(void)
{
	int ret;
	struct platform_device_info pdevinfo;
	struct platform_device *pd;

	ret = platform_driver_register(&noc_driver);
	if (ret) {
		pr_err("NOC: platform driver register failed\n");
		return -EINVAL;
	}

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.name = "noc";

	pd = platform_device_register_full(&pdevinfo);
	if (IS_ERR(pd)) {
		pr_err("NOC: platform device register failed\n");
		return -EINVAL;
	}
	intel_noc_pdev = pd;
	return 0;
}

static void noc_driver_module_exit(void)
{
	platform_driver_unregister(&noc_driver);
	if (NULL != intel_noc_pdev)
		platform_device_unregister(intel_noc_pdev);
}

module_init(noc_driver_module_init);
module_exit(noc_driver_module_exit);

MODULE_DESCRIPTION("KeemBay NOC Device driver");
MODULE_AUTHOR("Pandith N <pandith.n@intel.com>");
MODULE_LICENSE("GPL v2");
