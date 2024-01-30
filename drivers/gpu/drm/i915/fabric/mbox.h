/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2022 Intel Corporation.
 *
 */

#ifndef MBOX_H_INCLUDED
#define MBOX_H_INCLUDED

#if IS_ENABLED(CONFIG_IAF_DEBUG_MBOX_ACCESS)
void mbox_init_module(void);
void mbox_term_module(void);
#else
static inline void mbox_init_module(void) {}
static inline void mbox_term_module(void) {}
#endif

#endif
