/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1300 S Mopac Expwy,
  Austin, TX 78746


  BSD LICENSE

  Copyright(c) 2014 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _SWHV_DEFINES_H_
#define _SWHV_DEFINES_H_

/* ***************************************************
 * Common to kernel and userspace.
 * ***************************************************
 */
#define PW_SUCCESS 0
#define PW_ERROR 1
#define PW_SUCCESS_NO_COLLECT 2

//
// Start off with none of the OS'es are defined
//
#undef SWDRV_OS_LINUX
#undef SWDRV_OS_ANDROID
#undef SWDRV_OS_UNIX

//
// Make sure none of the architectures is defined here
//
#undef SWDRV_IA32
#undef SWDRV_EM64T

//
// Make sure one (and only one) of the OS'es gets defined here
//
// Unfortunately entirex defines _WIN32 so we need to check for linux
// first.  The definition of these flags is one and only one
// _OS_xxx is allowed to be defined.
//
#if defined(__ANDROID__)
#define SWDRV_OS_ANDROID
#define SWDRV_OS_UNIX
#elif defined(__linux__)
#define SWDRV_OS_LINUX
#define SWDRV_OS_UNIX
#else
#error "Compiling for an unknown OS"
#endif

//
// Make sure one (and only one) architecture is defined here
// as well as one (and only one) pointer__ size
//
#if defined(_M_IX86) || defined(__i386__)
#define SWDRV_IA32
#elif defined(_M_AMD64) || defined(__x86_64__)
#define SWDRV_EM64T
#else
#error "Unknown architecture for compilation"
#endif

#endif // _SWHV_DEFINES_H_
