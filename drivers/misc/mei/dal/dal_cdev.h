/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#ifndef __MEI_DAL_DEV_H__
#define __MEI_DAL_DEV_H__
void dal_dev_del(struct dal_device *ddev);
void dal_dev_setup(struct dal_device *ddev);
int dal_dev_add(struct dal_device *ddev);
int __init dal_dev_init(void);
void dal_dev_exit(void);
#endif /* __MEI_DAL_DEV_H__ */
