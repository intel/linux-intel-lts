/* ****************************************************************************
 *  Copyright(C) 2009-2018 Intel Corporation.  All Rights Reserved.
 *
 *  This file is part of SEP Development Kit
 *
 *  SEP Development Kit is free software; you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *
 *  SEP Development Kit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  As a special exception, you may use this file as part of a free software
 *  library without restriction.  Specifically, if other files instantiate
 *  templates or use macros or inline functions from this file, or you
 *  compile this file and link it with other files to produce an executable
 *  this file does not by itself cause the resulting executable to be
 *  covered by the GNU General Public License.  This exception does not
 *  however invalidate any other reasons why the executable file might be
 *  covered by the GNU General Public License.
 * ****************************************************************************
 */

#ifndef _SYS_INFO_H_
#define _SYS_INFO_H_

#include "lwpmudrv_defines.h"

#define KNIGHTS_FAMILY 0x06
#define KNL_MODEL 0x57
#define KNM_MODEL 0x85

#define is_Knights_family(family, model)                                 \
	((family == KNIGHTS_FAMILY) &&                                   \
	((model == KNL_MODEL) || (model == KNM_MODEL)))

typedef struct __generic_ioctl {
	U32 size;
	S32 ret;
	U64 rsv[3];
} GENERIC_IOCTL;

#define GENERIC_IOCTL_size(gio) ((gio)->size)
#define GENERIC_IOCTL_ret(gio) ((gio)->ret)

//
// This one is unusual in that it's really a variable
// size. The system_info field is just a easy way
// to access the base information, but the actual size
// when used tends to be much larger that what is
// shown here.
//
typedef struct __system_info {
	GENERIC_IOCTL gen;
	VTSA_SYS_INFO sys_info;
} IOCTL_SYS_INFO;

extern U32 *cpu_built_sysinfo;

#define IOCTL_SYS_INFO_gen(isi) ((isi)->gen)
#define IOCTL_SYS_INFO_sys_info(isi) ((isi)->sys_info)

extern U32 SYS_INFO_Build(void);
extern void SYS_INFO_Transfer(PVOID buf_usr_to_drv,
			 unsigned long len_usr_to_drv);
extern void SYS_INFO_Destroy(void);
extern void SYS_INFO_Build_Cpu(PVOID param);

#endif
