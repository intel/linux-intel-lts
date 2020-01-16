/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_DWXPCS_H
#define __LINUX_DWXPCS_H

#include <linux/types.h>

enum dwxpcs_pcs_mode {
	DWXPCS_MODE_SGMII_AN,
	DWXPCS_MODE_1000BASEX_AN,
};

struct dwxpcs_platform_data {
	int irq;
	enum dwxpcs_pcs_mode mode;
	int ext_phy_addr;
	bool speed_2500_en;
};

#endif
