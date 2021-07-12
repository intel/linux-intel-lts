/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019 Intel Corporation.
 */

#ifndef __BH_ERRCODE_H
#define __BH_ERRCODE_H

/*
 * BH Error codes numbers across Beihai Host and Firmware.
 */

#define BH_SUCCESS                              0x000

/* BHP specific error code section */

#define BPE_NOT_INIT                            0x001
#define BPE_SERVICE_UNAVAILABLE                 0x002
#define BPE_INTERNAL_ERROR                      0x003
#define BPE_COMMS_ERROR                         0x004
#define BPE_OUT_OF_MEMORY                       0x005
#define BPE_INVALID_PARAMS                      0x006
#define BPE_MESSAGE_TOO_SHORT                   0x007
#define BPE_MESSAGE_ILLEGAL                     0x008
#define BPE_NO_CONNECTION_TO_FIRMWARE           0x009
#define BPE_NOT_IMPLEMENT                       0x00A
#define BPE_OUT_OF_RESOURCE                     0x00B
#define BPE_INITIALIZED_ALREADY                 0x00C
#define BPE_CONNECT_FAILED                      0x00D

/* General error code section for Beihai on FW: 0x100 */

#define BHE_OUT_OF_MEMORY                       0x101
#define BHE_BAD_PARAMETER                       0x102
#define BHE_INSUFFICIENT_BUFFER                 0x103
#define BHE_MUTEX_INIT_FAIL                     0x104
#define BHE_COND_INIT_FAIL                      0x105
#define BHE_WD_TIMEOUT                          0x106
#define BHE_FAILED                              0x107
#define BHE_INVALID_HANDLE                      0x108
#define BHE_IPC_ERR_DEFAULT                     0x109
#define BHE_IPC_ERR_PLATFORM                    0x10A
#define BHE_IPC_SRV_INIT_FAIL                   0x10B

/* VM communication error code section:         0x200 */

#define BHE_MAILBOX_NOT_FOUND                   0x201
#define BHE_APPLET_CRASHED                      BHE_MAILBOX_NOT_FOUND
#define BHE_MSG_QUEUE_IS_FULL                   0x202
#define BHE_MAILBOX_DENIED                      0x203

/* VM InternalAppletCommunication error         0x240 */

#define BHE_IAC_INTERNAL_SESSION_NUM_EXCEED     0x241
#define BHE_IAC_CLIENT_SLOT_FULL                0x242
#define BHE_IAC_SERVICETA_EXITED                0x243
#define BHE_IAC_EXIST_INTERNAL_SESSION          0x244
#define BHE_IAC_SERVICETA_UNCAUGHT_EXCEPTION    0x245
#define BHE_IAC_SERVICE_SESSION_NOT_FOUND       0x246
#define BHE_IAC_SERVICE_HOST_SESSION_NUM_EXCEED 0x247

/* Firmware thread/mutex error code section:     0x280 */
#define BHE_THREAD_ERROR                        0x281
#define BHE_THREAD_TIMED_OUT                    0x282

/* Applet manager error code section:            0x300 */

#define BHE_LOAD_JEFF_FAIL                      0x303
#define BHE_PACKAGE_NOT_FOUND                   0x304
#define BHE_EXIST_LIVE_SESSION                  0x305
#define BHE_VM_INSTANCE_INIT_FAIL               0x306
#define BHE_QUERY_PROP_NOT_SUPPORT              0x307
#define BHE_INVALID_BPK_FILE                    0x308
#define BHE_PACKAGE_EXIST                       0x309
#define BHE_VM_INSTNACE_NOT_FOUND               0x312
#define BHE_STARTING_JDWP_FAIL                  0x313
#define BHE_GROUP_CHECK_FAIL                    0x314
#define BHE_SDID_UNMATCH                        0x315
#define BHE_APPPACK_UNINITED                    0x316
#define BHE_SESSION_NUM_EXCEED                  0x317
#define BHE_TA_PACKAGE_HASH_VERIFY_FAIL         0x318
#define BHE_SWITCH_ISD                          0x319
#define BHE_OPERATION_NOT_PERMITTED             0x31A

/* VM Applet instance error code section:        0x400 */
#define BHE_APPLET_GENERIC                      0x400
#define BHE_UNCAUGHT_EXCEPTION                  0x401
/* Bad parameters to applet */
#define BHE_APPLET_BAD_PARAMETER                0x402
/* Small response buffer */
#define BHE_APPLET_SMALL_BUFFER                 0x403
/* Bad state */
#define BHE_BAD_STATE                           0x404

/*TODO: Should be removed these UI error code when integrate with ME 9 */
#define BHE_UI_EXCEPTION                        0x501
#define BHE_UI_ILLEGAL_USE                      0x502
#define BHE_UI_ILLEGAL_PARAMETER                0x503
#define BHE_UI_NOT_INITIALIZED                  0x504
#define BHE_UI_NOT_SUPPORTED                    0x505
#define BHE_UI_OUT_OF_RESOURCES                 0x506

/* BeiHai VMInternalError code section:          0x600 */
#define BHE_UNKNOWN                             0x602
#define BHE_MAGIC_UNMATCH                       0x603
#define BHE_UNIMPLEMENTED                       0x604
#define BHE_INTR                                0x605
#define BHE_CLOSED                              0x606
/* TODO: no used error, should remove*/
#define BHE_BUFFER_OVERFLOW                     0x607
#define BHE_NOT_SUPPORTED                       0x608
#define BHE_WEAR_OUT_VIOLATION                  0x609
#define BHE_NOT_FOUND                           0x610
#define BHE_INVALID_PARAMS                      0x611
#define BHE_ACCESS_DENIED                       0x612
#define BHE_INVALID                             0x614
#define BHE_TIMEOUT                             0x615

/* SDM specific error code section:              0x800 */
#define BHE_SDM_FAILED                          0x800
#define BHE_SDM_NOT_FOUND                       0x801
#define BHE_SDM_ALREADY_EXIST                   0x803
#define BHE_SDM_TATYPE_MISMATCH                 0x804
#define BHE_SDM_TA_NUMBER_LIMIT                 0x805
#define BHE_SDM_SIGNAGURE_VERIFY_FAIL           0x806
#define BHE_SDM_PERMGROUP_CHECK_FAIL            0x807
#define BHE_SDM_INSTALL_CONDITION_FAIL          0x808
#define BHE_SDM_SVN_CHECK_FAIL                  0x809
#define BHE_SDM_TA_DB_NO_FREE_SLOT              0x80A
#define BHE_SDM_SD_DB_NO_FREE_SLOT              0x80B
#define BHE_SDM_SVL_DB_NO_FREE_SLOT             0x80C
#define BHE_SDM_SVL_CHECK_FAIL                  0x80D
#define BHE_SDM_DB_READ_FAIL                    0x80E
#define BHE_SDM_DB_WRITE_FAIL                   0x80F

/* Launcher specific error code section:         0x900 */
#define BHE_LAUNCHER_INIT_FAILED                0x901
#define BHE_SD_NOT_INSTALLED                    0x902
#define BHE_NTA_NOT_INSTALLED                   0x903
#define BHE_PROCESS_SPAWN_FAILED                0x904
#define BHE_PROCESS_KILL_FAILED                 0x905
#define BHE_PROCESS_ALREADY_RUNNING             0x906
#define BHE_PROCESS_IN_TERMINATING              0x907
#define BHE_PROCESS_NOT_EXIST                   0x908
#define BHE_PLATFORM_API_ERR                    0x909
#define BHE_PROCESS_NUM_EXCEED                  0x09A

/*
 * BeihaiHAL Layer error code section:
 * 0x1000,0x2000 reserved here, defined in CSG BeihaiStatusHAL.h
 */

#endif /* __BH_ERRCODE_H */
