/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2019, Intel Corporation.
 * Time-Sensitive Networking (TSN) Header
 */

#ifndef __STMMAC_TSN_H__
#define __STMMAC_TSN_H__

#define MIN_TSN_CORE_VER	0x50
#define EST_GCL_BANK_MAX		(2)
#define EST_TIWID_TO_EXTMAX(ti_wid)	((1 << ((ti_wid) + 7)) - 1)
#define STMMAC_TSN_STAT_SIZE		(16)

/* Hardware Tunable Enum */
enum tsn_hwtunable_id {
	TSN_HWTUNA_TX_EST_TILS = 0,
	TSN_HWTUNA_TX_EST_PTOV,
	TSN_HWTUNA_TX_EST_CTOV,
	TSN_HWTUNA_TX_FPE_AFSZ,
	TSN_HWTUNA_TX_FPE_HADV,
	TSN_HWTUNA_TX_FPE_RADV,
	TSN_HWTUNA_TX_TBS_ESTM,	 /* TBS Absolute or EST mode */
	TSN_HWTUNA_TX_TBS_LEOS,
	TSN_HWTUNA_TX_TBS_LEGOS,
	TSN_HWTUNA_TX_TBS_FTOS,	/* Fetch time offset (nsec) */
	TSN_HWTUNA_TX_TBS_FGOS,	/* Fetch GSN offset (slot number) */
	TSN_HWTUNA_MAX,
};

/* TSN Feature Enabled List */
enum tsn_feat_id {
	TSN_FEAT_ID_EST = 0,
	TSN_FEAT_ID_FPE = 1,
	TSN_FEAT_ID_TBS = 2,
	TSN_FEAT_ID_MAX,
};

/* TSN GCL Parameter Index */
#define I_GCE		0	/* Is GCE */
#define I_PRM		1	/* Is GCL Parameters */
/* Currently, the order of Param Index matches the GCL addr
 * order defined in IPv5.xx MTL EST GCL control register
 */
enum tsn_gcl_param_idx {
	GCL_PARAM_IDX_BTR_LO = 0,
	GCL_PARAM_IDX_BTR_HI = 1,
	GCL_PARAM_IDX_CTR_LO = 2,
	GCL_PARAM_IDX_CTR_HI = 3,
	GCL_PARAM_IDX_TER = 4,
	GCL_PARAM_IDX_LLR = 5,
};

/* TSN HW Capabilities */
struct tsn_hw_cap {
	bool est_support;	/* 1: supported */
	bool fpe_support;	/* 1: supported */
	bool tbs_support;	/* 1: supported */
	u32 txqcnt;		/* Number of TxQ (control gate) */
	u32 rxqcnt;		/* Number of RxQ (for FPRQ) */
	u32 gcl_depth;		/* GCL depth. */
	u32 ti_wid;		/* time interval width */
	u32 ext_max;		/* Max time extension */
	u32 cycle_max;		/* Max Cycle Time */
	u32 tils_max;		/* Max time interval left shift */
	u32 ptov_max;		/* Max PTP Offset */
	u32 ctov_max;		/* Max Current Time Offset */
	u32 idleslope_max;	/* Max idle slope */
	/* FPE */
	u32 pmac_bit;		/* Preemptible MAC bit */
	u32 afsz_max;		/* Adj Frag Size */
	u32 hadv_max;		/* Max Hold Advance */
	u32 radv_max;		/* Max Release Advance */
	u32 leos_max;		/* Launch Expiry Offset */
	u32 legos_max;		/* Launch Expiry GSN Offset */
	u32 ftos_max;		/* Max Fetch Time Offset */
	u32 fgos_max;		/* Max Fetch GSN Offset */
};

/* EST Gate Control Entry */
struct est_gc_entry {
	u8 command;		/* Gate command */
	u32 gates;		/* gate control: 0: closed,
				 *               1: open.
				 */
	u32 ti_nsec;		/* time interval in nsec */
};

/* EST GCL Related Registers */
struct est_gcrr {
	u32 base_nsec;		/* base time denominator (nsec) */
	u32 base_sec;		/* base time numerator (sec) */
	u32 cycle_nsec;		/* cycle time denominator (nsec) */
	u32 cycle_sec;		/* cycle time numerator sec)*/
	u32 ter_nsec;		/* time extension (nsec) */
	u32 llr;		/* GC list length */
};

/* EST Gate Control bank */
struct est_gc_bank {
	struct est_gc_entry *gcl;	/* Gate Control List */
	struct est_gcrr gcrr;		/* GCL Related Registers */
};

/* EST Gate Control Configuration */
struct est_gc_config {
	struct est_gc_bank gcb[EST_GCL_BANK_MAX];
	bool enable;			/* 1: enabled */
};

/* FPE hand-shaking event */
enum fpe_event {
	FPE_EVENT_UNKNOWN = 0,
	FPE_EVENT_TRSP = 1,	/* Tx'ed Response mPacket */
	FPE_EVENT_TVER = 2,	/* Tx'ed Verify mPacket */
	FPE_EVENT_RRSP = 4,	/* Rcvd Response mPacket */
	FPE_EVENT_RVER = 8,	/* Rcvd Verify mPacket */
};

/* FPE link state */
enum fpe_state {
	FPE_STATE_OFF = 0,
	FPE_STATE_CAPABLE = 1,
	FPE_STATE_ENTERING_ON = 2,
	FPE_STATE_ON = 3,
};

/* FPE link-partner hand-shaking mPacket type */
enum mpacket_type {
	MPACKET_VERIFY = 0,
	MPACKET_RESPONSE = 1,
};

/* FPE Configuration */
struct fpe_config {
	u32 txqpec;		/* TxQ Preemption Classification */
	bool enable;		/* 1: enabled */
	enum fpe_state lp_fpe_state;	/* Link Partner FPE state */
	enum fpe_state lo_fpe_state;	/* Local station FPE state */
	enum fpe_event fpe_event;	/* FPE Hand-shake state */
};

/* TSN MMC Statistics */
struct tsn_mmc_desc {
	bool valid;
	const char *desc;
};

struct tsn_mmc_stat {
	unsigned long count[STMMAC_TSN_STAT_SIZE];
};

enum fpe_task_state_t {
	__FPE_REMOVING,
	__FPE_TASK_SCHED,
};

struct tsnif_info {
	struct tsn_hw_cap cap;
	bool feat_en[TSN_FEAT_ID_MAX];
	u32 hwtunable[TSN_HWTUNA_MAX];
	struct est_gc_config est_gcc;
	struct fpe_config fpe_cfg;
	struct tsn_mmc_stat mmc_stat;
	const struct tsn_mmc_desc *mmc_desc;
	/* Workqueue for handling FPE hand-shaking */
	unsigned long task_state;
	struct workqueue_struct *fpe_wq;
	struct work_struct fpe_task;
	char wq_name[IFNAMSIZ + 4];
};

struct mac_device_info;

/* TSN functions */
int tsn_init(struct mac_device_info *hw, struct net_device *dev);
int tsn_feat_set(struct mac_device_info *hw, struct net_device *dev,
		 enum tsn_feat_id featid, bool enable);
bool tsn_has_feat(struct mac_device_info *hw, struct net_device *dev,
		  enum tsn_feat_id featid);
void tsn_hw_setup(struct mac_device_info *hw, struct net_device *dev,
		  u32 fprq);
void tsn_hw_unsetup(struct mac_device_info *hw, struct net_device *dev);
int tsn_hwtunable_set(struct mac_device_info *hw, struct net_device *dev,
		      enum tsn_hwtunable_id id, const u32 data);
int tsn_hwtunable_get(struct mac_device_info *hw, struct net_device *dev,
		      enum tsn_hwtunable_id id, u32 *data);
int tsn_est_enable_set(struct mac_device_info *hw, struct net_device *dev,
		       bool enable);
int tsn_est_bank_get(struct mac_device_info *hw, struct net_device *dev,
		     bool is_own, u32 *bank);
int tsn_est_gce_set(struct mac_device_info *hw, struct net_device *dev,
		    struct est_gc_entry *gce, u32 row,
		    u32 dbgb, bool is_dbgm);
int tsn_est_gcl_len_set(struct mac_device_info *hw, struct net_device *dev,
			u32 gcl_len, u32 dbgb, bool is_dbgm);
int tsn_est_gcl_len_get(struct mac_device_info *hw, struct net_device *dev,
			u32 *gcl_len, u32 dbgb, bool is_dbgm);
int tsn_est_gcrr_times_set(struct mac_device_info *hw,
			   struct net_device *dev,
			   struct est_gcrr *gcrr,
			   u32 dbgb, bool is_dbgm);
int tsn_est_gcc_get(struct mac_device_info *hw, struct net_device *dev,
		    struct est_gc_config **gcc);
void tsn_est_irq_status(struct mac_device_info *hw, struct net_device *dev);
void tsn_mmc_stat_update(struct mac_device_info *hw, struct net_device *dev);
int tsn_mmc_dump(struct mac_device_info *hw,
		 int index, unsigned long *count, const char **desc);
int tsn_cbs_recal_idleslope(struct mac_device_info *hw, struct net_device *dev,
			    u32 queue, u32 *idle_slope);
int tsn_fpe_set_txqpec(struct mac_device_info *hw, struct net_device *dev,
		       u32 txqpec);
int tsn_fpe_set_enable(struct mac_device_info *hw, struct net_device *dev,
		       bool enable);
int tsn_fpe_get_config(struct mac_device_info *hw, struct net_device *dev,
		       u32 *txqpec, bool *enable);
int tsn_fpe_show_pmac_sts(struct mac_device_info *hw, struct net_device *dev);
int tsn_fpe_send_mpacket(struct mac_device_info *hw, struct net_device *dev,
			 enum mpacket_type type);
void tsn_fpe_link_state_handle(struct mac_device_info *hw,
			       struct net_device *dev, bool is_up);
void tsn_fpe_irq_status(struct mac_device_info *hw, struct net_device *dev);

#endif /* __STMMAC_TSN_H__ */
