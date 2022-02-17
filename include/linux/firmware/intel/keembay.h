/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Intel Keembay SOC Firmware API Layer
 *
 *  Copyright (C) 2020, Intel Corporation
 *
 *  Author: Muhammad Husaini Zulkifli <Muhammad.Husaini.Zulkifli@intel.com>
 */

#ifndef __FIRMWARE_KEEMBAY_SMC_H__
#define __FIRMWARE_KEEMBAY_SMC_H__

#include <linux/arm-smccc.h>

/*
 * This file defines an API function that can be called by a device driver in order to
 * communicate with Trusted Firmware - A profile(TF-A) or Trusted Firmware - M profile (TF-M).
 */

#define KEEMBAY_SET_1V8_IO_RAIL	1
#define KEEMBAY_SET_3V3_IO_RAIL	0

#define KEEMBAY_IOV_1_8V_uV	1800000
#define KEEMBAY_IOV_3_3V_uV	3300000

#define KEEMBAY_SET_SD_VOLTAGE_ID 0xFF26
#define KEEMBAY_GET_SD_VOLTAGE_ID 0xFF2A

#define ARM_SMCCC_SIP_KEEMBAY_SET_SD_VOLTAGE		\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,		\
			   ARM_SMCCC_SMC_32,		\
			   ARM_SMCCC_OWNER_SIP,		\
			   KEEMBAY_SET_SD_VOLTAGE_ID)

#define ARM_SMCCC_SIP_KEEMBAY_GET_SD_VOLTAGE		\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,		\
			   ARM_SMCCC_SMC_32,		\
			   ARM_SMCCC_OWNER_SIP,		\
			   KEEMBAY_GET_SD_VOLTAGE_ID)

#define KEEMBAY_REG_NUM_CONSUMERS 2

struct keembay_reg_supply {
	struct regulator *consumer;
};

#if IS_ENABLED(CONFIG_HAVE_ARM_SMCCC_DISCOVERY)
/*
 * Voltage applied on the IO Rail is controlled from the Always On Register using specific
 * bits in AON_CGF1 register. This is a secure register. Keem Bay SOC cannot exposed this
 * register address to the outside world.
 */
static inline int keembay_set_io_rail_supplied_voltage(int volt)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(ARM_SMCCC_SIP_KEEMBAY_SET_SD_VOLTAGE, volt, &res);

	return res.a0;
}

static inline int keembay_get_io_rail_supplied_voltage(void)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(ARM_SMCCC_SIP_KEEMBAY_GET_SD_VOLTAGE, &res);

	return res.a1;
}
#else
static inline int keembay_set_io_rail_supplied_voltage(int volt)
{
	return -ENODEV;
}

static inline int keembay_get_io_rail_supplied_voltage(void)
{
	return -ENODEV;
}
#endif

#endif /* __FIRMWARE_KEEMBAY_SMC_H__ */
