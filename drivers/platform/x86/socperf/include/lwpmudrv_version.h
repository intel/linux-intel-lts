/* ***********************************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(C) 2010-2019 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2010-2019 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ***********************************************************************************************
 */


/*
 *  File  : lwpmudrv_version.h
 */

#ifndef _LWPMUDRV_VERSION_H_
#define _LWPMUDRV_VERSION_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * @macro SOCPERF_VERSION_NODE_S
 * @brief
 * This structure supports versioning in Sep. The field major indicates the major version,
 * minor indicates the minor version and api indicates the api version for the current
 * sep build. This structure is initialized at the time when the driver is loaded.
 */

typedef struct SOCPERF_VERSION_NODE_S SOCPERF_VERSION_NODE;
typedef SOCPERF_VERSION_NODE * SOCPERF_VERSION;

struct SOCPERF_VERSION_NODE_S {
	union {
		U32 socperf_version;
		struct {
			S32 major : 8;
			S32 minor : 8;
			S32 api : 8;
			S32 update : 8;
		} s1;
	} u1;
};

#define SOCPERF_VERSION_NODE_socperf_version(version)                          \
	((version)->u1.socperf_version)
#define SOCPERF_VERSION_NODE_major(version) ((version)->u1.s1.major)
#define SOCPERF_VERSION_NODE_minor(version) ((version)->u1.s1.minor)
#define SOCPERF_VERSION_NODE_api(version) ((version)->u1.s1.api)
#define SEP_VERSION_NODE_update(version) ((version)->u1.s1.update)

#if defined(__cplusplus)
}
#endif

// SOCPERF VERSIONING

#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)
#define _STRINGIFY_W(x) L#x
#define STRINGIFY_W(x) _STRINGIFY_W(x)

#define SOCPERF_MAJOR_VERSION 3
#define SOCPERF_MINOR_VERSION 0
#define SOCPERF_API_VERSION 0
#define SOCPERF_UPDATE_VERSION 0
#if SOCPERF_UPDATE_VERSION > 0
#define SOCPERF_UPDATE_STRING " Update " STRINGIFY(SOCPERF_UPDATE_VERSION)
#else
#define SOCPERF_UPDATE_STRING ""
#endif

#define SOCPERF_PRODUCT_NAME "Sampling Enabling Product"
#define PRODUCT_VERSION_DATE __DATE__ " at " __TIME__
#define PRODUCT_COPYRIGHT                                                      \
	"Copyright (C) 2011-2018 Intel Corporation. All rights reserved."
#define PRODUCT_DISCLAIMER                                                                  \
	"Warning: This computer program is protected under U.S. and international\n"        \
	"copyright laws, and may only be used or copied in accordance with the terms\n"     \
	"of the license agreement. Except as permitted by such license, no part\n"          \
	"of this computer program may be reproduced, stored in a retrieval system,\n"       \
	"or transmitted in any form or by any means without the express written consent\n"  \
	"of Intel Corporation."

#define PRODUCT_VERSION "5.0"

#define SOCPERF_NAME "socperf"
#define SOCPERF_NAME_W L"socperf"

#define SOCPERF_MSG_PREFIX                                                     \
	SOCPERF_NAME "" STRINGIFY(SOCPERF_MAJOR_VERSION) "_" STRINGIFY(        \
		SOCPERF_MINOR_VERSION) ":"
#define SOCPERF_VERSION_STR                                                    \
	STRINGIFY(SOCPERF_MAJOR_VERSION)                                       \
	"." STRINGIFY(SOCPERF_MINOR_VERSION) "." STRINGIFY(                    \
		SOCPERF_API_VERSION)

// #if defined(DRV_OS_WINDOWS)
// #define SOCPERF_DRIVER_NAME SOCPERF_NAME STRINGIFY(SOCPERF_MAJOR_VERSION)
// #define SOCPERF_DRIVER_NAME_W SOCPERF_NAME_W STRINGIFY_W(SOCPERF_MAJOR_VERSION)
// #define SOCPERF_DEVICE_NAME SOCPERF_DRIVER_NAME
// #endif

#if defined(DRV_OS_LINUX) || defined(DRV_OS_SOLARIS) ||                        \
	defined(DRV_OS_ANDROID) || defined(DRV_OS_FREEBSD)
#define SOCPERF_DRIVER_NAME SOCPERF_NAME "" STRINGIFY(SOCPERF_MAJOR_VERSION)
#define SOCPERF_SAMPLES_NAME SOCPERF_DRIVER_NAME "_s"
#define SOCPERF_DEVICE_NAME "/dev/" SOCPERF_DRIVER_NAME
#endif

// #if defined(DRV_OS_MAC)
// #define SOCPERF_DRIVER_NAME SOCPERF_NAME "" STRINGIFY(SOCPERF_MAJOR_VERSION)
// #define SOCPERF_SAMPLES_NAME SOCPERF_DRIVER_NAME "_s"
// #define SOCPERF_DEVICE_NAME SOCPERF_DRIVER_NAME
// #endif

#endif
