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

/*
 * Description: file containing locking routines
 * used by the power driver.
 */

#ifndef __SW_LOCK_DEFS_H__
#define __SW_LOCK_DEFS_H__

#define SW_DEFINE_SPINLOCK(s) DEFINE_SPINLOCK(s)
#define SW_DECLARE_SPINLOCK(s) static spinlock_t s

#define SW_INIT_SPINLOCK(s) spin_lock_init(&s)
#define SW_DESTROY_SPINLOCK(s) /* NOP */

#define LOCK(l)                                                                \
	{                                                                      \
		unsigned long _tmp_l_flags;                                    \
		spin_lock_irqsave(&(l), _tmp_l_flags);

#define UNLOCK(l)                                                              \
	spin_unlock_irqrestore(&(l), _tmp_l_flags);                            \
	}

#define READ_LOCK(l)                                                           \
	{                                                                      \
		unsigned long _tmp_l_flags;                                    \
		read_lock_irqsave(&(l), _tmp_l_flags);

#define READ_UNLOCK(l)                                                         \
	read_unlock_irqrestore(&(l), _tmp_l_flags);                            \
	}

#define WRITE_LOCK(l)                                                          \
	{                                                                      \
		unsigned long _tmp_l_flags;                                    \
		write_lock_irqsave(&(l), _tmp_l_flags);

#define WRITE_UNLOCK(l)                                                        \
	write_unlock_irqrestore(&(l), _tmp_l_flags);                           \
	}

#endif /* __SW_LOCK_DEFS_H__ */
