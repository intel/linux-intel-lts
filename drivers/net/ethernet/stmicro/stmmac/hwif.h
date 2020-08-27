/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
// Copyright (c) 2018 Synopsys, Inc. and/or its affiliates.
// stmmac HW Interface Callbacks

#ifndef __STMMAC_HWIF_H__
#define __STMMAC_HWIF_H__

#include <linux/netdevice.h>
#include <linux/stmmac.h>

#define stmmac_do_void_callback(__priv, __module, __cname,  __arg0, __args...) \
({ \
	int __result = -EINVAL; \
	if ((__priv)->hw->__module && (__priv)->hw->__module->__cname) { \
		(__priv)->hw->__module->__cname((__arg0), ##__args); \
		__result = 0; \
	} \
	__result; \
})
#define stmmac_do_callback(__priv, __module, __cname,  __arg0, __args...) \
({ \
	int __result = -EINVAL; \
	if ((__priv)->hw->__module && (__priv)->hw->__module->__cname) \
		__result = (__priv)->hw->__module->__cname((__arg0), ##__args); \
	__result; \
})

struct stmmac_extra_stats;
struct stmmac_safety_stats;
struct dma_desc;
struct dma_extended_desc;

/* Descriptors helpers */
struct stmmac_desc_ops {
	/* DMA RX descriptor ring initialization */
	void (*init_rx_desc)(struct dma_desc *p, int disable_rx_ic, int mode,
			int end, int bfsize);
	/* DMA TX descriptor ring initialization */
	void (*init_tx_desc)(struct dma_desc *p, int mode, int end);
	/* Invoked by the xmit function to prepare the tx descriptor */
	void (*prepare_tx_desc)(struct dma_desc *p, int is_fs, int len,
			bool csum_flag, int mode, bool tx_own, bool ls,
			unsigned int tot_pkt_len);
	void (*prepare_tso_tx_desc)(struct dma_desc *p, int is_fs, int len1,
			int len2, bool tx_own, bool ls, unsigned int tcphdrlen,
			unsigned int tcppayloadlen);
	/* Set/get the owner of the descriptor */
	void (*set_tx_owner)(struct dma_desc *p);
	int (*get_tx_owner)(struct dma_desc *p);
	/* Clean the tx descriptor as soon as the tx irq is received */
	void (*release_tx_desc)(struct dma_desc *p, int mode);
	/* Clear interrupt on tx frame completion. When this bit is
	 * set an interrupt happens as soon as the frame is transmitted */
	void (*set_tx_ic)(struct dma_desc *p);
	/* Last tx segment reports the transmit status */
	int (*get_tx_ls)(struct dma_desc *p);
	/* RX VLAN TCI */
	int (*get_rx_vlan_tci)(struct dma_desc *p);
	/* RX VLAN valid */
	bool (*get_rx_vlan_valid)(struct dma_desc *p);
	/* Return the transmit status looking at the TDES1 */
	int (*tx_status)(void *data, struct stmmac_extra_stats *x,
			struct dma_desc *p, void __iomem *ioaddr);
	/* Get the buffer size from the descriptor */
	int (*get_tx_len)(struct dma_desc *p);
	/* Handle extra events on specific interrupts hw dependent */
	void (*set_rx_owner)(struct dma_desc *p, int disable_rx_ic);
	/* Get the receive frame size */
	int (*get_rx_frame_len)(struct dma_desc *p, int rx_coe_type);
	/* Return the reception status looking at the RDES1 */
	int (*rx_status)(void *data, struct stmmac_extra_stats *x,
			struct dma_desc *p);
	void (*rx_extended_status)(void *data, struct stmmac_extra_stats *x,
			struct dma_extended_desc *p);
	/* Set tx timestamp enable bit */
	void (*enable_tx_timestamp) (struct dma_desc *p);
	/* get tx timestamp status */
	int (*get_tx_timestamp_status) (struct dma_desc *p);
	/* get timestamp value */
	void (*get_timestamp)(void *desc, u32 ats, u64 *ts);
	/* get rx timestamp status */
	int (*get_rx_timestamp_status)(void *desc, void *next_desc, u32 ats);
	/* Display ring */
	void (*display_ring)(void *head, unsigned int size, bool rx);
	/* set MSS via context descriptor */
	void (*set_mss)(struct dma_desc *p, unsigned int mss);
	/* get descriptor skbuff address */
	void (*get_addr)(struct dma_desc *p, dma_addr_t *addr);
	/* set descriptor skbuff address */
	void (*set_addr)(struct dma_desc *p, dma_addr_t addr);
	/* clear descriptor */
	void (*clear)(struct dma_desc *p);
	/* RSS */
	int (*get_rx_hash)(struct dma_desc *p, u32 *hash,
			   enum pkt_hash_types *type);
	int (*get_rx_header_len)(struct dma_desc *p, unsigned int *len);
	void (*set_sec_addr)(struct dma_desc *p, dma_addr_t addr);
	void (*set_sarc)(struct dma_desc *p, u32 sarc_type);
	void (*set_vlan_tag)(struct dma_desc *p, u16 tag, u16 inner_tag,
			     u32 inner_type);
	void (*set_vlan)(struct dma_desc *p, u32 type);
};

#define stmmac_init_rx_desc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, init_rx_desc, __args)
#define stmmac_init_tx_desc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, init_tx_desc, __args)
#define stmmac_prepare_tx_desc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, prepare_tx_desc, __args)
#define stmmac_prepare_tso_tx_desc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, prepare_tso_tx_desc, __args)
#define stmmac_set_tx_owner(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_tx_owner, __args)
#define stmmac_get_tx_owner(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_tx_owner, __args)
#define stmmac_release_tx_desc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, release_tx_desc, __args)
#define stmmac_set_tx_ic(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_tx_ic, __args)
#define stmmac_get_tx_ls(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_tx_ls, __args)
#define stmmac_get_rx_vlan_tci(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_rx_vlan_tci, __args)
#define stmmac_get_rx_vlan_valid(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_rx_vlan_valid, __args)
#define stmmac_tx_status(__priv, __args...) \
	stmmac_do_callback(__priv, desc, tx_status, __args)
#define stmmac_get_tx_len(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_tx_len, __args)
#define stmmac_set_rx_owner(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_rx_owner, __args)
#define stmmac_get_rx_frame_len(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_rx_frame_len, __args)
#define stmmac_rx_status(__priv, __args...) \
	stmmac_do_callback(__priv, desc, rx_status, __args)
#define stmmac_rx_extended_status(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, rx_extended_status, __args)
#define stmmac_enable_tx_timestamp(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, enable_tx_timestamp, __args)
#define stmmac_get_tx_timestamp_status(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_tx_timestamp_status, __args)
#define stmmac_get_timestamp(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, get_timestamp, __args)
#define stmmac_get_rx_timestamp_status(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_rx_timestamp_status, __args)
#define stmmac_display_ring(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, display_ring, __args)
#define stmmac_set_mss(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_mss, __args)
#define stmmac_get_desc_addr(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, get_addr, __args)
#define stmmac_set_desc_addr(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_addr, __args)
#define stmmac_clear_desc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, clear, __args)
#define stmmac_get_rx_hash(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_rx_hash, __args)
#define stmmac_get_rx_header_len(__priv, __args...) \
	stmmac_do_callback(__priv, desc, get_rx_header_len, __args)
#define stmmac_set_desc_sec_addr(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_sec_addr, __args)
#define stmmac_set_desc_sarc(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_sarc, __args)
#define stmmac_set_desc_vlan_tag(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_vlan_tag, __args)
#define stmmac_set_desc_vlan(__priv, __args...) \
	stmmac_do_void_callback(__priv, desc, set_vlan, __args)

struct stmmac_dma_cfg;
struct dma_features;

/* Specific DMA helpers */
struct stmmac_dma_ops {
	/* DMA core initialization */
	int (*reset)(void __iomem *ioaddr);
	void (*init)(void __iomem *ioaddr, struct stmmac_dma_cfg *dma_cfg,
		     int atds);
	void (*init_chan)(void __iomem *ioaddr,
			  struct stmmac_dma_cfg *dma_cfg, u32 chan);
	void (*init_rx_chan)(void __iomem *ioaddr,
			     struct stmmac_dma_cfg *dma_cfg,
			     dma_addr_t phy, u32 chan);
	void (*init_tx_chan)(void __iomem *ioaddr,
			     struct stmmac_dma_cfg *dma_cfg,
			     dma_addr_t phy, u32 chan);
	/* Configure the AXI Bus Mode Register */
	void (*axi)(void __iomem *ioaddr, struct stmmac_axi *axi);
	/* Dump DMA registers */
	void (*dump_regs)(void __iomem *ioaddr, u32 *reg_space);
	void (*dma_rx_mode)(void __iomem *ioaddr, int mode, u32 channel,
			    int fifosz, u8 qmode);
	void (*dma_tx_mode)(void __iomem *ioaddr, int mode, u32 channel,
			    int fifosz, u8 qmode);
	/* To track extra statistic (if supported) */
	void (*dma_diagnostic_fr) (void *data, struct stmmac_extra_stats *x,
				   void __iomem *ioaddr);
	void (*enable_dma_transmission) (void __iomem *ioaddr);
	void (*enable_dma_irq)(void __iomem *ioaddr, u32 chan);
	void (*disable_dma_irq)(void __iomem *ioaddr, u32 chan);
	void (*start_tx)(void __iomem *ioaddr, u32 chan);
	void (*stop_tx)(void __iomem *ioaddr, u32 chan);
	void (*start_rx)(void __iomem *ioaddr, u32 chan);
	void (*stop_rx)(void __iomem *ioaddr, u32 chan);
	int (*dma_interrupt) (void __iomem *ioaddr,
			      struct stmmac_extra_stats *x, u32 chan, u32 dir);
	/* If supported then get the optional core features */
	void (*get_hw_feature)(void __iomem *ioaddr,
			       struct dma_features *dma_cap);
	/* Program the HW RX Watchdog */
	void (*rx_watchdog)(void __iomem *ioaddr, u32 riwt, u32 number_chan);
	void (*set_tx_ring_len)(void __iomem *ioaddr, u32 len, u32 chan);
	void (*set_rx_ring_len)(void __iomem *ioaddr, u32 len, u32 chan);
	void (*set_rx_tail_ptr)(void __iomem *ioaddr, u32 tail_ptr, u32 chan);
	void (*set_tx_tail_ptr)(void __iomem *ioaddr, u32 tail_ptr, u32 chan);
	void (*enable_tso)(void __iomem *ioaddr, bool en, u32 chan);
	void (*qmode)(void __iomem *ioaddr, u32 channel, u8 qmode);
	void (*set_bfsize)(void __iomem *ioaddr, int bfsize, u32 chan);
	void (*enable_sph)(void __iomem *ioaddr, bool en, u32 chan);
	int (*enable_tbs)(void __iomem *ioaddr, bool en, u32 chan);
};

#define stmmac_reset(__priv, __args...) \
	stmmac_do_callback(__priv, dma, reset, __args)
#define stmmac_dma_init(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, init, __args)
#define stmmac_init_chan(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, init_chan, __args)
#define stmmac_init_rx_chan(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, init_rx_chan, __args)
#define stmmac_init_tx_chan(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, init_tx_chan, __args)
#define stmmac_axi(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, axi, __args)
#define stmmac_dump_dma_regs(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, dump_regs, __args)
#define stmmac_dma_rx_mode(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, dma_rx_mode, __args)
#define stmmac_dma_tx_mode(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, dma_tx_mode, __args)
#define stmmac_dma_diagnostic_fr(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, dma_diagnostic_fr, __args)
#define stmmac_enable_dma_transmission(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, enable_dma_transmission, __args)
#define stmmac_enable_dma_irq(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, enable_dma_irq, __args)
#define stmmac_disable_dma_irq(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, disable_dma_irq, __args)
#define stmmac_start_tx(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, start_tx, __args)
#define stmmac_stop_tx(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, stop_tx, __args)
#define stmmac_start_rx(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, start_rx, __args)
#define stmmac_stop_rx(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, stop_rx, __args)
#define stmmac_dma_interrupt_status(__priv, __args...) \
	stmmac_do_callback(__priv, dma, dma_interrupt, __args)
#define stmmac_get_hw_feature(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, get_hw_feature, __args)
#define stmmac_rx_watchdog(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, rx_watchdog, __args)
#define stmmac_set_tx_ring_len(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, set_tx_ring_len, __args)
#define stmmac_set_rx_ring_len(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, set_rx_ring_len, __args)
#define stmmac_set_rx_tail_ptr(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, set_rx_tail_ptr, __args)
#define stmmac_set_tx_tail_ptr(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, set_tx_tail_ptr, __args)
#define stmmac_enable_tso(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, enable_tso, __args)
#define stmmac_dma_qmode(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, qmode, __args)
#define stmmac_set_dma_bfsize(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, set_bfsize, __args)
#define stmmac_enable_sph(__priv, __args...) \
	stmmac_do_void_callback(__priv, dma, enable_sph, __args)
#define stmmac_enable_tbs(__priv, __args...) \
	stmmac_do_callback(__priv, dma, enable_tbs, __args)

struct mac_device_info;
struct net_device;
struct rgmii_adv;
struct stmmac_safety_stats;
struct stmmac_tc_entry;
struct stmmac_pps_cfg;
struct stmmac_rss;
enum tsn_feat_id;
enum tsn_hwtunable_id;
struct est_gc_entry;
struct est_gcrr;
struct est_gc_config;
enum mpacket_type;

/* Helpers to program the MAC core */
struct stmmac_ops {
	/* MAC core initialization */
	void (*core_init)(struct mac_device_info *hw, struct net_device *dev);
	/* Enable the MAC RX/TX */
	void (*set_mac)(void __iomem *ioaddr, bool enable);
	/* Start/Stop TX/RX state machine of MAC */
	void (*start_tx)(void __iomem *ioaddr);
	void (*stop_tx)(void __iomem *ioaddr);
	void (*start_rx)(void __iomem *ioaddr);
	void (*stop_rx)(void __iomem *ioaddr);
	/* Enable and verify that the IPC module is supported */
	int (*rx_ipc)(struct mac_device_info *hw);
	/* Enable RX Queues */
	void (*rx_queue_enable)(struct mac_device_info *hw, u8 mode, u32 queue);
	/* RX Queues Priority */
	void (*rx_queue_prio)(struct mac_device_info *hw, u32 prio, u32 queue);
	/* TX Queues Priority */
	void (*tx_queue_prio)(struct mac_device_info *hw, u32 prio, u32 queue);
	/* RX Queues Routing */
	void (*rx_queue_routing)(struct mac_device_info *hw, u8 packet,
				 u32 queue);
	/* Program RX Algorithms */
	void (*prog_mtl_rx_algorithms)(struct mac_device_info *hw, u32 rx_alg);
	/* Program TX Algorithms */
	void (*prog_mtl_tx_algorithms)(struct mac_device_info *hw, u32 tx_alg);
	/* Set MTL TX queues weight */
	void (*set_mtl_tx_queue_weight)(struct mac_device_info *hw,
					u32 weight, u32 queue);
	/* RX MTL queue to RX dma mapping */
	void (*map_mtl_to_dma)(struct mac_device_info *hw, u32 queue, u32 chan);
	/* Configure AV Algorithm */
	void (*config_cbs)(struct mac_device_info *hw, u32 send_slope,
			   u32 idle_slope, u32 high_credit, u32 low_credit,
			   u32 queue);
	/* Dump MAC registers */
	void (*dump_regs)(struct mac_device_info *hw, u32 *reg_space);
	/* Handle extra events on specific interrupts hw dependent */
	int (*host_irq_status)(struct mac_device_info *hw,
			       struct stmmac_extra_stats *x);
	/* Handle MTL interrupts */
	int (*host_mtl_irq_status)(struct mac_device_info *hw, u32 chan);
	/* Multicast filter setting */
	void (*set_filter)(struct mac_device_info *hw, struct net_device *dev);
	/* Flow control setting */
	void (*flow_ctrl)(struct mac_device_info *hw, unsigned int duplex,
			  unsigned int fc, unsigned int pause_time, u32 tx_cnt);
	/* Set power management mode (e.g. magic frame) */
	void (*pmt)(struct mac_device_info *hw, unsigned long mode);
	/* Set/Get Unicast MAC addresses */
	void (*set_umac_addr)(struct mac_device_info *hw, unsigned char *addr,
			      unsigned int reg_n);
	void (*get_umac_addr)(struct mac_device_info *hw, unsigned char *addr,
			      unsigned int reg_n);
	void (*set_eee_mode)(struct mac_device_info *hw,
			     bool en_tx_lpi_clockgating);
	void (*reset_eee_mode)(struct mac_device_info *hw);
	void (*set_eee_lpi_entry_timer)(struct mac_device_info *hw, int et);
	void (*set_eee_timer)(struct mac_device_info *hw, int ls, int tw);
	void (*set_eee_pls)(struct mac_device_info *hw, int link);
	void (*debug)(void __iomem *ioaddr, struct stmmac_extra_stats *x,
		      u32 rx_queues, u32 tx_queues);
	/* PCS calls */
	void (*pcs_ctrl_ane)(void __iomem *ioaddr, bool ane, bool srgmi_ral,
			     bool loopback);
	void (*pcs_rane)(void __iomem *ioaddr, bool restart);
	void (*pcs_get_adv_lp)(void __iomem *ioaddr, struct rgmii_adv *adv);
	/* Safety Features */
	int (*safety_feat_config)(void __iomem *ioaddr, unsigned int asp);
	int (*safety_feat_irq_status)(struct net_device *ndev,
			void __iomem *ioaddr, unsigned int asp,
			struct stmmac_safety_stats *stats);
	int (*safety_feat_dump)(struct stmmac_safety_stats *stats,
			int index, unsigned long *count, const char **desc);
	/* Flexible RX Parser */
	int (*rxp_config)(void __iomem *ioaddr, struct stmmac_tc_entry *entries,
			  unsigned int count);
	/* Flexible PPS */
	int (*flex_pps_config)(void __iomem *ioaddr, int index,
			       struct stmmac_pps_cfg *cfg, bool enable,
			       u32 sub_second_inc, u32 systime_flags);
	/* Loopback for selftests */
	void (*set_mac_loopback)(void __iomem *ioaddr, bool enable);
	/* RSS */
	int (*rss_configure)(struct mac_device_info *hw,
			     struct stmmac_rss *cfg, u32 num_rxq);
	/* VLAN */
	void (*update_vlan_hash)(struct mac_device_info *hw, u32 hash,
				 bool is_double);
	void (*enable_vlan)(struct mac_device_info *hw, u32 type);
	void (*rx_hw_vlan)(struct net_device *dev, struct mac_device_info *hw,
			   struct dma_desc *rx_desc, struct sk_buff *skb);
	void (*set_hw_vlan_mode)(void __iomem *ioaddr,
				 netdev_features_t features);
	int (*add_hw_vlan_rx_fltr)(struct net_device *dev,
				   struct mac_device_info *hw,
				   __be16 proto, u16 vid);
	int (*del_hw_vlan_rx_fltr)(struct net_device *dev,
				   struct mac_device_info *hw,
				   __be16 proto, u16 vid);
	void (*restore_hw_vlan_rx_fltr)(struct net_device *dev,
					struct mac_device_info *hw);
	/* TX Timestamp */
	int (*get_mac_tx_timestamp)(struct mac_device_info *hw, u64 *ts);
	/* Source Address Insertion / Replacement */
	void (*sarc_configure)(void __iomem *ioaddr, int val);
	/* Filtering */
	int (*config_l3_filter)(struct mac_device_info *hw, u32 filter_no,
				bool en, bool ipv6, bool sa, bool inv,
				u32 match);
	int (*config_l4_filter)(struct mac_device_info *hw, u32 filter_no,
				bool en, bool udp, bool sa, bool inv,
				u32 match);
	void (*set_arp_offload)(struct mac_device_info *hw, bool en, u32 addr);
	/* Check frame transmission is completed */
	int (*mtl_tx_completed)(void __iomem *ioaddr, u32 tx_queues);
	/* TSN APIs */
	void (*tsnif_setup)(struct mac_device_info *mac);
	int (*init_tsn)(struct mac_device_info *hw, struct net_device *dev);
	int (*set_tsn_feat)(struct mac_device_info *hw,
			    struct net_device *dev,
			    enum tsn_feat_id featid, bool enable);
	bool (*has_tsn_feat)(struct mac_device_info *hw, struct net_device *dev,
			     enum tsn_feat_id featid);
	void (*setup_tsn_hw)(struct mac_device_info *hw,
			     struct net_device *dev, u32 fprq);
	void (*unsetup_tsn_hw)(struct mac_device_info *hw,
			       struct net_device *dev);
	int (*set_tsn_hwtunable)(struct mac_device_info *hw,
				 struct net_device *dev,
				 enum tsn_hwtunable_id id,
				 const u32 data);
	int (*get_tsn_hwtunable)(struct mac_device_info *hw,
				 struct net_device *dev,
				 enum tsn_hwtunable_id id, u32 *data);
	int (*set_est_enable)(struct mac_device_info *hw,
			      struct net_device *dev, bool enable);
	int (*get_est_bank)(struct mac_device_info *hw, struct net_device *dev,
			    bool is_own, u32 *bank);
	int (*set_est_gce)(struct mac_device_info *hw, struct net_device *dev,
			   struct est_gc_entry *gce, u32 row,
			   u32 dbgb, bool is_dbgm);
	int (*get_est_gcl_len)(struct mac_device_info *hw,
			       struct net_device *dev, u32 *gcl_len,
			       u32 dbgb, bool is_dbgm);
	int (*set_est_gcl_len)(struct mac_device_info *hw,
			       struct net_device *dev, u32 gcl_len,
			       u32 dbgb, bool is_dbgm);
	int (*set_est_gcrr_times)(struct mac_device_info *hw,
				  struct net_device *dev,
				  struct est_gcrr *gcrr,
				  u32 dbgb, bool is_dbgm);
	int (*get_est_gcc)(struct mac_device_info *hw, struct net_device *dev,
			   struct est_gc_config **gcc);
	void (*est_irq_status)(struct mac_device_info *hw,
			       struct net_device *dev);
	void (*update_tsn_mmc_stat)(struct mac_device_info *hw,
				    struct net_device *dev);
	int (*dump_tsn_mmc)(struct mac_device_info *hw, int index,
			    unsigned long *count, const char **desc);
	int (*cbs_recal_idleslope)(struct mac_device_info *hw,
				   struct net_device *dev,
				   u32 queue,
				   u32 *idle_slope);
	int (*fpe_set_txqpec)(struct mac_device_info *hw,
			      struct net_device *dev, u32 txqpec);
	int (*fpe_set_enable)(struct mac_device_info *hw,
			      struct net_device *dev, bool enable);
	int (*fpe_get_config)(struct mac_device_info *hw,
			      struct net_device *dev, u32 *txqpec,
			      bool *enable);
	int (*fpe_show_pmac_sts)(struct mac_device_info *hw,
				 struct net_device *dev);
	int (*fpe_send_mpacket)(struct mac_device_info *hw,
				struct net_device *dev, enum mpacket_type type);
	void (*fpe_link_state_handle)(struct mac_device_info *hw,
				      struct net_device *dev, bool is_up);
	void (*fpe_irq_status)(struct mac_device_info *hw,
			       struct net_device *dev);
};

#define stmmac_core_init(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, core_init, __args)
#define stmmac_mac_set(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_mac, __args)
#define stmmac_start_mac_tx(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, start_tx, __args)
#define stmmac_stop_mac_tx(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, stop_tx, __args)
#define stmmac_start_mac_rx(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, start_rx, __args)
#define stmmac_stop_mac_rx(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, stop_rx, __args)
#define stmmac_rx_ipc(__priv, __args...) \
	stmmac_do_callback(__priv, mac, rx_ipc, __args)
#define stmmac_rx_queue_enable(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, rx_queue_enable, __args)
#define stmmac_rx_queue_prio(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, rx_queue_prio, __args)
#define stmmac_tx_queue_prio(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, tx_queue_prio, __args)
#define stmmac_rx_queue_routing(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, rx_queue_routing, __args)
#define stmmac_prog_mtl_rx_algorithms(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, prog_mtl_rx_algorithms, __args)
#define stmmac_prog_mtl_tx_algorithms(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, prog_mtl_tx_algorithms, __args)
#define stmmac_set_mtl_tx_queue_weight(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_mtl_tx_queue_weight, __args)
#define stmmac_map_mtl_to_dma(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, map_mtl_to_dma, __args)
#define stmmac_config_cbs(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, config_cbs, __args)
#define stmmac_dump_mac_regs(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, dump_regs, __args)
#define stmmac_host_irq_status(__priv, __args...) \
	stmmac_do_callback(__priv, mac, host_irq_status, __args)
#define stmmac_host_mtl_irq_status(__priv, __args...) \
	stmmac_do_callback(__priv, mac, host_mtl_irq_status, __args)
#define stmmac_set_filter(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_filter, __args)
#define stmmac_flow_ctrl(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, flow_ctrl, __args)
#define stmmac_pmt(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, pmt, __args)
#define stmmac_set_umac_addr(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_umac_addr, __args)
#define stmmac_get_umac_addr(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, get_umac_addr, __args)
#define stmmac_set_eee_mode(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_eee_mode, __args)
#define stmmac_reset_eee_mode(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, reset_eee_mode, __args)
#define stmmac_set_eee_lpi_timer(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_eee_lpi_entry_timer, __args)
#define stmmac_set_eee_timer(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_eee_timer, __args)
#define stmmac_set_eee_pls(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_eee_pls, __args)
#define stmmac_mac_debug(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, debug, __args)
#define stmmac_pcs_ctrl_ane(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, pcs_ctrl_ane, __args)
#define stmmac_pcs_rane(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, pcs_rane, __args)
#define stmmac_pcs_get_adv_lp(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, pcs_get_adv_lp, __args)
#define stmmac_safety_feat_config(__priv, __args...) \
	stmmac_do_callback(__priv, mac, safety_feat_config, __args)
#define stmmac_safety_feat_irq_status(__priv, __args...) \
	stmmac_do_callback(__priv, mac, safety_feat_irq_status, __args)
#define stmmac_safety_feat_dump(__priv, __args...) \
	stmmac_do_callback(__priv, mac, safety_feat_dump, __args)
#define stmmac_rxp_config(__priv, __args...) \
	stmmac_do_callback(__priv, mac, rxp_config, __args)
#define stmmac_flex_pps_config(__priv, __args...) \
	stmmac_do_callback(__priv, mac, flex_pps_config, __args)
#define stmmac_set_mac_loopback(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_mac_loopback, __args)
#define stmmac_rss_configure(__priv, __args...) \
	stmmac_do_callback(__priv, mac, rss_configure, __args)
#define stmmac_update_vlan_hash(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, update_vlan_hash, __args)
#define stmmac_enable_vlan(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, enable_vlan, __args)
#define stmmac_rx_hw_vlan(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, rx_hw_vlan, __args)
#define stmmac_set_hw_vlan_mode(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_hw_vlan_mode, __args)
#define stmmac_add_hw_vlan_rx_fltr(__priv, __args...) \
	stmmac_do_callback(__priv, mac, add_hw_vlan_rx_fltr, __args)
#define stmmac_del_hw_vlan_rx_fltr(__priv, __args...) \
	stmmac_do_callback(__priv, mac, del_hw_vlan_rx_fltr, __args)
#define stmmac_restore_hw_vlan_rx_fltr(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, restore_hw_vlan_rx_fltr, __args)
#define stmmac_get_mac_tx_timestamp(__priv, __args...) \
	stmmac_do_callback(__priv, mac, get_mac_tx_timestamp, __args)
#define stmmac_sarc_configure(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, sarc_configure, __args)
#define stmmac_config_l3_filter(__priv, __args...) \
	stmmac_do_callback(__priv, mac, config_l3_filter, __args)
#define stmmac_config_l4_filter(__priv, __args...) \
	stmmac_do_callback(__priv, mac, config_l4_filter, __args)
#define stmmac_set_arp_offload(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_arp_offload, __args)
#define stmmac_mtl_tx_completed(__priv, __args...) \
	stmmac_do_callback(__priv, mac, mtl_tx_completed, __args)
#define stmmac_tsnif_setup(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, tsnif_setup, __args)
#define stmmac_tsn_init(__priv, __args...) \
	stmmac_do_callback(__priv, mac, init_tsn, __args)
#define stmmac_set_tsn_feat(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, set_tsn_feat, __args)
#define stmmac_has_tsn_feat(__priv, __args...) \
	stmmac_do_callback(__priv, mac, has_tsn_feat, __args)
#define stmmac_tsn_hw_setup(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, setup_tsn_hw, __args)
#define stmmac_tsn_hw_unsetup(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, unsetup_tsn_hw, __args)
#define stmmac_set_tsn_hwtunable(__priv, __args...) \
	stmmac_do_callback(__priv, mac, set_tsn_hwtunable, __args)
#define stmmac_get_tsn_hwtunable(__priv, __args...) \
	stmmac_do_callback(__priv, mac, get_tsn_hwtunable, __args)
#define stmmac_set_est_enable(__priv, __args...) \
	stmmac_do_callback(__priv, mac, set_est_enable, __args)
#define stmmac_get_est_bank(__priv, __args...) \
	stmmac_do_callback(__priv, mac, get_est_bank, __args)
#define stmmac_set_est_gce(__priv, __args...) \
	stmmac_do_callback(__priv, mac, set_est_gce, __args)
#define stmmac_set_est_gcl_len(__priv, __args...) \
	stmmac_do_callback(__priv, mac, set_est_gcl_len, __args)
#define stmmac_get_est_gcl_len(__priv, __args...) \
	stmmac_do_callback(__priv, mac, get_est_gcl_len, __args)
#define stmmac_set_est_gcrr_times(__priv, __args...) \
	stmmac_do_callback(__priv, mac, set_est_gcrr_times, __args)
#define stmmac_get_est_gcc(__priv, __args...) \
	stmmac_do_callback(__priv, mac, get_est_gcc, __args)
#define stmmac_est_irq_status(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, est_irq_status, __args)
#define stmmac_update_tsn_mmc_stat(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, update_tsn_mmc_stat, __args)
#define stmmac_dump_tsn_mmc(__priv, __args...) \
	stmmac_do_callback(__priv, mac, dump_tsn_mmc, __args)
#define stmmac_cbs_recal_idleslope(__priv, __args...) \
	stmmac_do_callback(__priv, mac, cbs_recal_idleslope, __args)
#define stmmac_fpe_set_txqpec(__priv, __args...) \
	stmmac_do_callback(__priv, mac, fpe_set_txqpec, __args)
#define stmmac_fpe_set_enable(__priv, __args...) \
	stmmac_do_callback(__priv, mac, fpe_set_enable, __args)
#define stmmac_fpe_get_config(__priv, __args...) \
	stmmac_do_callback(__priv, mac, fpe_get_config, __args)
#define stmmac_fpe_show_pmac_sts(__priv, __args...) \
	stmmac_do_callback(__priv, mac, fpe_show_pmac_sts, __args)
#define stmmac_fpe_send_mpacket(__priv, __args...) \
	stmmac_do_callback(__priv, mac, fpe_send_mpacket, __args)
#define stmmac_fpe_link_state_handle(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, fpe_link_state_handle, __args)
#define stmmac_fpe_irq_status(__priv, __args...) \
	stmmac_do_void_callback(__priv, mac, fpe_irq_status, __args)

/* Helpers for serdes */
struct stmmac_serdes_ops {
	int (*serdes_powerup)(struct net_device *ndev);
	int (*serdes_powerdown)(struct net_device *ndev);
	int (*speed_mode_2500)(struct net_device *ndev);
};

#define stmmac_serdes_powerup(__priv, __args...) \
	stmmac_do_callback(__priv, serdes, serdes_powerup, __args)
#define stmmac_serdes_powerdown(__priv, __args...) \
	stmmac_do_callback(__priv, serdes, serdes_powerdown, __args)
#define stmmac_speed_mode_2500(__priv, __args...) \
	stmmac_do_callback(__priv, serdes, speed_mode_2500, __args)

struct mii_bus;
struct stmmac_priv;

/* PTP and HW Timer helpers */
struct stmmac_hwtimestamp {
	void (*config_hw_tstamping) (void __iomem *ioaddr, u32 data);
	void (*config_sub_second_increment)(void __iomem *ioaddr, u32 ptp_clock,
					   int gmac4, u32 *ssinc,
					   bool is_hfpga);
	int (*init_systime) (void __iomem *ioaddr, u32 sec, u32 nsec);
	int (*config_addend) (void __iomem *ioaddr, u32 addend);
	int (*adjust_systime) (void __iomem *ioaddr, u32 sec, u32 nsec,
			       int add_sub, int gmac4);
	void (*get_systime) (void __iomem *ioaddr, u64 *systime);
	void (*get_arttime)(struct mii_bus *mii, int intel_adhoc_addr,
			    u64 *art_time);
	void (*get_ptptime)(void __iomem *ioaddr, u64 *ptp_time);
	void (*tstamp_interrupt)(struct stmmac_priv *priv);
};

#define stmmac_config_hw_tstamping(__priv, __args...) \
	stmmac_do_void_callback(__priv, ptp, config_hw_tstamping, __args)
#define stmmac_config_sub_second_increment(__priv, __args...) \
	stmmac_do_void_callback(__priv, ptp, config_sub_second_increment, __args)
#define stmmac_init_systime(__priv, __args...) \
	stmmac_do_callback(__priv, ptp, init_systime, __args)
#define stmmac_config_addend(__priv, __args...) \
	stmmac_do_callback(__priv, ptp, config_addend, __args)
#define stmmac_adjust_systime(__priv, __args...) \
	stmmac_do_callback(__priv, ptp, adjust_systime, __args)
#define stmmac_get_systime(__priv, __args...) \
	stmmac_do_void_callback(__priv, ptp, get_systime, __args)
#define stmmac_get_arttime(__priv, __args...) \
	stmmac_do_void_callback(__priv, ptp, get_arttime, __args)
#define stmmac_get_ptptime(__priv, __args...) \
	stmmac_do_void_callback(__priv, ptp, get_ptptime, __args)
#define stmmac_tstamp_interrupt(__priv, __args...) \
	stmmac_do_void_callback(__priv, ptp, tstamp_interrupt, __args)

/* Helpers to manage the descriptors for chain and ring modes */
struct stmmac_mode_ops {
	void (*init) (void *des, dma_addr_t phy_addr, unsigned int size,
		      unsigned int extend_desc);
	unsigned int (*is_jumbo_frm) (int len, int ehn_desc);
	int (*jumbo_frm)(void *priv, struct sk_buff *skb, int csum);
	int (*set_16kib_bfsize)(int mtu);
	void (*init_desc3)(struct dma_desc *p);
	void (*refill_desc3) (void *priv, struct dma_desc *p);
	void (*clean_desc3) (void *priv, struct dma_desc *p);
};

#define stmmac_mode_init(__priv, __args...) \
	stmmac_do_void_callback(__priv, mode, init, __args)
#define stmmac_is_jumbo_frm(__priv, __args...) \
	stmmac_do_callback(__priv, mode, is_jumbo_frm, __args)
#define stmmac_jumbo_frm(__priv, __args...) \
	stmmac_do_callback(__priv, mode, jumbo_frm, __args)
#define stmmac_set_16kib_bfsize(__priv, __args...) \
	stmmac_do_callback(__priv, mode, set_16kib_bfsize, __args)
#define stmmac_init_desc3(__priv, __args...) \
	stmmac_do_void_callback(__priv, mode, init_desc3, __args)
#define stmmac_refill_desc3(__priv, __args...) \
	stmmac_do_void_callback(__priv, mode, refill_desc3, __args)
#define stmmac_clean_desc3(__priv, __args...) \
	stmmac_do_void_callback(__priv, mode, clean_desc3, __args)

struct tc_cls_u32_offload;
struct tc_cbs_qopt_offload;
struct flow_cls_offload;
struct tc_taprio_qopt_offload;
struct tc_etf_qopt_offload;

struct stmmac_tc_ops {
	int (*init)(struct stmmac_priv *priv);
	int (*setup_cls_u32)(struct stmmac_priv *priv,
			     struct tc_cls_u32_offload *cls);
	int (*setup_cbs)(struct stmmac_priv *priv,
			 struct tc_cbs_qopt_offload *qopt);
	int (*setup_cls)(struct stmmac_priv *priv,
			 struct flow_cls_offload *cls);
	int (*setup_taprio)(struct stmmac_priv *priv,
			    struct tc_taprio_qopt_offload *qopt);
	int (*setup_etf)(struct stmmac_priv *priv,
			 struct tc_etf_qopt_offload *qopt);
};

#define stmmac_tc_init(__priv, __args...) \
	stmmac_do_callback(__priv, tc, init, __args)
#define stmmac_tc_setup_cls_u32(__priv, __args...) \
	stmmac_do_callback(__priv, tc, setup_cls_u32, __args)
#define stmmac_tc_setup_cbs(__priv, __args...) \
	stmmac_do_callback(__priv, tc, setup_cbs, __args)
#define stmmac_tc_setup_cls(__priv, __args...) \
	stmmac_do_callback(__priv, tc, setup_cls, __args)
#define stmmac_tc_setup_taprio(__priv, __args...) \
	stmmac_do_callback(__priv, tc, setup_taprio, __args)
#define stmmac_tc_setup_etf(__priv, __args...) \
	stmmac_do_callback(__priv, tc, setup_etf, __args)

struct stmmac_counters;

struct stmmac_mmc_ops {
	void (*ctrl)(void __iomem *ioaddr, unsigned int mode);
	void (*intr_all_mask)(void __iomem *ioaddr);
	void (*read)(void __iomem *ioaddr, struct stmmac_counters *mmc);
};

#define stmmac_mmc_ctrl(__priv, __args...) \
	stmmac_do_void_callback(__priv, mmc, ctrl, __args)
#define stmmac_mmc_intr_all_mask(__priv, __args...) \
	stmmac_do_void_callback(__priv, mmc, intr_all_mask, __args)
#define stmmac_mmc_read(__priv, __args...) \
	stmmac_do_void_callback(__priv, mmc, read, __args)

struct stmmac_regs_off {
	u32 ptp_off;
	u32 mmc_off;
};

#ifdef CONFIG_STMMAC_NETWORK_PROXY
struct stmmac_pm_ops {
	int (*suspend)(struct stmmac_priv *priv, struct net_device *ndev);
	int (*resume)(struct stmmac_priv *priv, struct net_device *ndev);
};

#define stmmac_pm_suspend(__priv, __args...) \
	stmmac_do_callback(__priv, pm, suspend, __args)
#define stmmac_pm_resume(__priv, __args...) \
	stmmac_do_callback(__priv, pm, resume, __args)
#endif /* CONFIG_STMMAC_NETWORK_PROXY */

extern const struct stmmac_ops dwmac100_ops;
extern const struct stmmac_serdes_ops intel_serdes_ops;
extern const struct stmmac_dma_ops dwmac100_dma_ops;
extern const struct stmmac_ops dwmac1000_ops;
extern const struct stmmac_dma_ops dwmac1000_dma_ops;
extern const struct stmmac_ops dwmac4_ops;
extern const struct stmmac_dma_ops dwmac4_dma_ops;
extern const struct stmmac_ops dwmac410_ops;
extern const struct stmmac_dma_ops dwmac410_dma_ops;
extern const struct stmmac_ops dwmac510_ops;
extern const struct stmmac_dma_ops dwmac5_dma_ops;
extern const struct stmmac_tc_ops dwmac510_tc_ops;
extern const struct stmmac_ops dwxgmac210_ops;
extern const struct stmmac_dma_ops dwxgmac210_dma_ops;
extern const struct stmmac_desc_ops dwxgmac210_desc_ops;
extern const struct stmmac_mmc_ops dwmac_mmc_ops;
extern const struct stmmac_mmc_ops dwxgmac_mmc_ops;
#ifdef CONFIG_STMMAC_NETWORK_PROXY
extern const struct stmmac_pm_ops dwmac_pm_ops;
extern const struct stmmac_pm_ops dwmac_netprox_pm_ops;
#endif

#define GMAC_VERSION		0x00000020	/* GMAC CORE Version */
#define GMAC4_VERSION		0x00000110	/* GMAC4+ CORE Version */

int stmmac_hwif_init(struct stmmac_priv *priv);

/* TSN Interface HW IP Specific Functions
 * Note:
 *  These functions implement IP specifics logics and are callable by TSN APIs
 *  defined in struct stmmac_ops. To differentiate them from high level TSN
 *  APIs, we use tsnif_xxx here.
 */
#define tsnif_do_void_callback(__hw, __cname,  __arg0, __args...) \
({ \
	int __result = -EINVAL; \
	if ((__hw)->tsnif && (__hw)->tsnif->__cname) { \
		(__hw)->tsnif->__cname((__arg0), ##__args); \
		__result = 0; \
	} \
	__result; \
})
#define tsnif_do_callback(__hw, __cname,  __arg0, __args...) \
({ \
	int __result = -EINVAL; \
	if ((__hw)->tsnif && (__hw)->tsnif->__cname) \
		__result = (__hw)->tsnif->__cname((__arg0), ##__args); \
	__result; \
})

struct tsn_mmc_stat;
enum fpe_event;

struct tsnif_ops {
	u32 (*read_hwid)(void __iomem *ioaddr);
	bool (*has_tsn_cap)(void __iomem *ioaddr, enum tsn_feat_id featid);
	void (*hw_setup)(void __iomem *ioaddr, enum tsn_feat_id featid,
			 u32 fprq);
	/* IEEE 802.1Qbv Enhanced Scheduled Traffics (EST) */
	u32 (*est_get_gcl_depth)(void __iomem *ioaddr);
	u32 (*est_get_ti_width)(void __iomem *ioaddr);
	u32 (*est_get_txqcnt)(void __iomem *ioaddr);
	u32 (*est_get_rxqcnt)(void __iomem *ioaddr);
	void (*est_get_max)(u32 *ptov_max, u32 *ctov_max, u32 *ct_max,
			    u32 *idleslope_max);
	int (*est_write_gcl_config)(void __iomem *ioaddr, u32 data, u32 addr,
				    bool is_gcrr,
				    u32 dbgb, bool is_dbgm);
	int (*est_read_gcl_config)(void __iomem *ioaddr, u32 *data, u32 addr,
				   bool is_gcrr,
				   u32 dbgb, bool is_dbgm);
	int (*est_read_gce)(void __iomem *ioaddr, u32 row,
			    u32 *gates, u32 *ti_nsec,
			    u32 ti_wid, u32 txqcnt,
			    u32 dbgb, bool is_dbgm);
	void (*est_set_tils)(void __iomem *ioaddr, const u32 tils);
	void (*est_set_ptov)(void __iomem *ioaddr, const u32 ptov);
	void (*est_set_ctov)(void __iomem *ioaddr, const u32 ctov);
	int (*est_set_enable)(void __iomem *ioaddr, bool enable);
	bool (*est_get_enable)(void __iomem *ioaddr);
	u32 (*est_get_bank)(void __iomem *ioaddr, bool is_own);
	void (*est_switch_swol)(void __iomem *ioaddr);
	int (*est_irq_status)(void *ioaddr, struct net_device *dev,
			      struct tsn_mmc_stat *mmc_stat,
			      unsigned int txqcnt);
	/* Frame Preemption (FPE) */
	void (*fpe_get_info)(u32 *pmac_bit, u32 *afsz_max,
			     u32 *hadv_max, u32 *radv_max);
	void (*fpe_set_txqpec)(void *ioaddr, u32 txqpec, u32 txqmask);
	void (*fpe_set_enable)(void *ioaddr, bool enable);
	void (*fpe_get_config)(void *ioaddr, u32 *txqpec, bool *enable);
	void (*fpe_get_pmac_sts)(void *ioaddr, u32 *hrs);
	void (*fpe_set_afsz)(void *ioaddr, const u32 afsz);
	void (*fpe_set_hadv)(void *ioaddr, const u32 hadv);
	void (*fpe_set_radv)(void *ioaddr, const u32 radv);
	void (*fpe_send_mpacket)(void *ioaddr, enum mpacket_type type);
	void (*fpe_irq_status)(void *ioaddr, struct net_device *dev,
			       enum fpe_event *fpe_event);
	void (*fpe_mmc_irq_status)(void __iomem *ioaddr,
				   struct net_device *dev);
	void (*fpe_update_mmc_stat)(void __iomem *ioaddr,
				    struct tsn_mmc_stat *mmc_stat);
	/* Time-Based Scheduling (TBS) */
	void (*tbs_get_max)(u32 *leos_max, u32 *legos_max,
			    u32 *ftos_max, u32 *fgos_max);
	void (*tbs_set_estm)(void __iomem *ioaddr, const u32 estm);
	void (*tbs_set_leos)(void __iomem *ioaddr, const u32 leos,
			     const u32 estm);
	void (*tbs_set_legos)(void __iomem *ioaddr, const u32 legos,
			      const u32 leos);
	void (*tbs_set_ftos)(void __iomem *ioaddr, const u32 ftos,
			     const u32 estm, const u32 fgos);
	void (*tbs_set_fgos)(void __iomem *ioaddr, const u32 fgos,
			     const u32 ftos);
};

#define tsnif_read_hwid(__hw, __args...) \
	tsnif_do_callback(__hw, read_hwid, __args)
#define tsnif_has_tsn_cap(__hw, __args...) \
	tsnif_do_callback(__hw, has_tsn_cap, __args)
#define tsnif_hw_setup(__hw, __args...) \
	tsnif_do_void_callback(__hw, hw_setup, __args)
#define tsnif_est_get_gcl_depth(__hw, __args...) \
	tsnif_do_callback(__hw, est_get_gcl_depth, __args)
#define tsnif_est_get_ti_width(__hw, __args...) \
	tsnif_do_callback(__hw, est_get_ti_width, __args)
#define tsnif_est_get_txqcnt(__hw, __args...) \
	tsnif_do_callback(__hw, est_get_txqcnt, __args)
#define tsnif_est_get_rxqcnt(__hw, __args...) \
	tsnif_do_callback(__hw, est_get_rxqcnt, __args)
#define tsnif_est_get_max(__hw, __args...) \
	tsnif_do_void_callback(__hw, est_get_max, __args)
#define tsnif_est_write_gcl_config(__hw, __args...) \
	tsnif_do_callback(__hw, est_write_gcl_config, __args)
#define tsnif_est_read_gcl_config(__hw, __args...) \
	tsnif_do_callback(__hw, est_read_gcl_config, __args)
#define tsnif_est_read_gce(__hw, __args...) \
	tsnif_do_callback(__hw, est_read_gce, __args)
#define tsnif_est_set_tils(__hw, __args...) \
	tsnif_do_void_callback(__hw, est_set_tils, __args)
#define tsnif_est_set_ptov(__hw, __args...) \
	tsnif_do_void_callback(__hw, est_set_ptov, __args)
#define tsnif_est_set_ctov(__hw, __args...) \
	tsnif_do_void_callback(__hw, est_set_ctov, __args)
#define tsnif_est_set_enable(__hw, __args...) \
	tsnif_do_callback(__hw, est_set_enable, __args)
#define tsnif_est_get_enable(__hw, __args...) \
	tsnif_do_callback(__hw, est_get_enable, __args)
#define tsnif_est_get_bank(__hw, __args...) \
	tsnif_do_callback(__hw, est_get_bank, __args)
#define tsnif_est_switch_swol(__hw, __args...) \
	tsnif_do_void_callback(__hw, est_switch_swol, __args)
#define tsnif_est_irq_status(__hw, __args...) \
	tsnif_do_callback(__hw, est_irq_status, __args)
#define tsnif_fpe_get_info(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_get_info, __args)
#define tsnif_fpe_set_txqpec(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_set_txqpec, __args)
#define tsnif_fpe_set_enable(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_set_enable, __args)
#define tsnif_fpe_get_config(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_get_config, __args)
#define tsnif_fpe_get_pmac_sts(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_get_pmac_sts, __args)
#define tsnif_fpe_set_afsz(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_set_afsz, __args)
#define tsnif_fpe_set_hadv(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_set_hadv, __args)
#define tsnif_fpe_set_radv(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_set_radv, __args)
#define tsnif_fpe_set_radv(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_set_radv, __args)
#define tsnif_fpe_send_mpacket(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_send_mpacket, __args)
#define tsnif_fpe_irq_status(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_irq_status, __args)
#define tsnif_fpe_mmc_irq_status(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_mmc_irq_status, __args)
#define tsnif_fpe_update_mmc_stat(__hw, __args...) \
	tsnif_do_void_callback(__hw, fpe_update_mmc_stat, __args)
#define tsnif_tbs_get_max(__hw, __args...) \
	tsnif_do_void_callback(__hw, tbs_get_max, __args)
#define tsnif_tbs_set_estm(__hw, __args...) \
	tsnif_do_void_callback(__hw, tbs_set_estm, __args)
#define tsnif_tbs_set_leos(__hw, __args...) \
	tsnif_do_void_callback(__hw, tbs_set_leos, __args)
#define tsnif_tbs_set_legos(__hw, __args...) \
	tsnif_do_void_callback(__hw, tbs_set_legos, __args)
#define tsnif_tbs_set_ftos(__hw, __args...) \
	tsnif_do_void_callback(__hw, tbs_set_ftos, __args)
#define tsnif_tbs_set_fgos(__hw, __args...) \
	tsnif_do_void_callback(__hw, tbs_set_fgos, __args)

#endif /* __STMMAC_HWIF_H__ */
