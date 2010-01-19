/*******************************************************************************

  Intel(R) Gigabit Ethernet Linux driver
  Copyright(c) 2007-2009 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* Linux PRO/1000 Ethernet Driver main header file */

#ifndef _IGB_H_
#define _IGB_H_

#ifdef IGB_LRO
#include <net/tcp.h>
#endif

#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>

#ifdef SIOCETHTOOL
#include <linux/ethtool.h>
#endif

#ifdef SIOCSHWTSTAMP
#include <linux/clocksource.h>
#include <linux/timecompare.h>
#include <linux/net_tstamp.h>

#endif
struct igb_adapter;

#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
#define IGB_DCA
#endif
#ifdef IGB_DCA
#include <linux/dca.h>
#endif

#ifndef SIOCSHWTSTAMP
#undef IGB_PER_PKT_TIMESTAMP
#endif


#include "kcompat.h"

#include "e1000_api.h"
#include "e1000_82575.h"

#define IGB_ERR(args...) printk(KERN_ERR "igb: " args)

#define PFX "igb: "
#define DPRINTK(nlevel, klevel, fmt, args...) \
	(void)((NETIF_MSG_##nlevel & adapter->msg_enable) && \
	printk(KERN_##klevel PFX "%s: %s: " fmt, adapter->netdev->name, \
		__FUNCTION__ , ## args))

/* Interrupt defines */
#define IGB_START_ITR                    648 /* ~6000 ints/sec */

/* Interrupt modes, as used by the IntMode paramter */
#define IGB_INT_MODE_LEGACY                0
#define IGB_INT_MODE_MSI                   1
#define IGB_INT_MODE_MSIX                  2

/* TX/RX descriptor defines */
#define IGB_DEFAULT_TXD                  256
#define IGB_MIN_TXD                       80
#define IGB_MAX_TXD                     4096

#define IGB_DEFAULT_RXD                  256
#define IGB_MIN_RXD                       80
#define IGB_MAX_RXD                     4096

#define IGB_MIN_ITR_USECS                 10 /* 100k irq/sec */
#define IGB_MAX_ITR_USECS               8191 /* 120  irq/sec */

#define NON_Q_VECTORS                      1
#define MAX_Q_VECTORS                      8

/* Transmit and receive queues */
#define IGB_MAX_RX_QUEUES                 16
#define IGB_MAX_TX_QUEUES                 16

#define IGB_MAX_VF_MC_ENTRIES             30
#define IGB_MAX_VF_FUNCTIONS               8
#define IGB_MAX_VFTA_ENTRIES             128
#define IGB_MAX_UTA_ENTRIES              128
#define MAX_EMULATION_MAC_ADDRS           16
#define OUI_LEN                            3

struct vf_data_storage {
	unsigned char vf_mac_addresses[ETH_ALEN];
	u16 vf_mc_hashes[IGB_MAX_VF_MC_ENTRIES];
	u16 num_vf_mc_hashes;
	u16 default_vf_vlan_id;
	u16 vlans_enabled;
	unsigned char em_mac_addresses[MAX_EMULATION_MAC_ADDRS * ETH_ALEN];
	u32 uta_table_copy[IGB_MAX_UTA_ENTRIES];
	u32 flags;
	unsigned long last_nack;
};

#define IGB_VF_FLAG_CTS            0x00000001 /* VF is clear to send data */
#define IGB_VF_FLAG_UNI_PROMISC    0x00000002 /* VF has unicast promisc */
#define IGB_VF_FLAG_MULTI_PROMISC  0x00000004 /* VF has multicast promisc */

/* RX descriptor control thresholds.
 * PTHRESH - MAC will consider prefetch if it has fewer than this number of
 *           descriptors available in its onboard memory.
 *           Setting this to 0 disables RX descriptor prefetch.
 * HTHRESH - MAC will only prefetch if there are at least this many descriptors
 *           available in host memory.
 *           If PTHRESH is 0, this should also be 0.
 * WTHRESH - RX descriptor writeback threshold - MAC will delay writing back
 *           descriptors until either it has this many to write back, or the
 *           ITR timer expires.
 */
#define IGB_RX_PTHRESH                    (hw->mac.type <= e1000_82576 ? 16 : 8)
#define IGB_RX_HTHRESH                     8
#define IGB_RX_WTHRESH                     1
#define IGB_TX_PTHRESH                     8
#define IGB_TX_HTHRESH                     1
#define IGB_TX_WTHRESH                     ((hw->mac.type == e1000_82576 && \
                                             adapter->msix_entries) ? 1 : 16)

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* Supported Rx Buffer Sizes */
#define IGB_RXBUFFER_64    64     /* Used for packet split */
#define IGB_RXBUFFER_128   128    /* Used for packet split */
#define IGB_RXBUFFER_1024  1024
#define IGB_RXBUFFER_2048  2048
#define IGB_RXBUFFER_4096  4096
#define IGB_RXBUFFER_8192  8192
#define IGB_RXBUFFER_16384 16384

/* Packet Buffer allocations */
#define IGB_PBA_BYTES_SHIFT 0xA
#define IGB_TX_HEAD_ADDR_SHIFT 7
#define IGB_PBA_TX_MASK 0xFFFF0000

#define IGB_FC_PAUSE_TIME 0x0680 /* 858 usec */

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define IGB_TX_QUEUE_WAKE	32
/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define IGB_RX_BUFFER_WRITE	16	/* Must be power of 2 */

#define AUTO_ALL_MODES            0
#define IGB_EEPROM_APME         0x0400

#ifndef IGB_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define IGB_MASTER_SLAVE	e1000_ms_hw_default
#endif

#define IGB_MNG_VLAN_NONE -1

#ifdef IGB_LRO
#define IGB_LRO_MAX 32	/*Maximum number of LRO descriptors*/
#define IGB_LRO_GLOBAL 10

struct igb_lro_stats {
	u32 flushed;
	u32 coal;
	u32 recycled;
};

struct igb_lro_desc {
	struct  hlist_node lro_node;
	struct  sk_buff *skb;
	u32   source_ip;
	u32   dest_ip;
	u16   source_port;
	u16   dest_port;
	u16   vlan_tag;
	u16   len;
	u32   next_seq;
	u32   ack_seq;
	u16   window;
	u16   mss;
	u16   opt_bytes;
	u16   psh:1;
	u32   tsval;
	u32   tsecr;
	u32   append_cnt;
};

struct igb_lro_list {
	struct hlist_head active;
	struct hlist_head free;
	int active_cnt;
	struct igb_lro_stats stats;
};

#endif /* IGB_LRO */
/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct igb_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	union {
		/* TX */
		struct {
			unsigned long time_stamp;
			u16 length;
			u16 next_to_watch;
			u16 mapped_as_page;
			u16 gso_segs;
		};

#ifndef CONFIG_IGB_DISABLE_PACKET_SPLIT
		/* RX */
		struct {
			unsigned long page_offset;
			struct page *page;
			dma_addr_t page_dma;
		};
#endif
	};
};

struct igb_tx_queue_stats {
	u64 packets;
	u64 bytes;
	u64 restart_queue;
};

struct igb_rx_queue_stats {
	u64 packets;
	u64 bytes;
	u64 drops;
	u64 csum_err;
	u64 alloc_failed;
};

struct igb_q_vector {
	struct igb_adapter *adapter; /* backlink */
	struct igb_ring *rx_ring;
	struct igb_ring *tx_ring;
	struct napi_struct napi;

	u32 eims_value;
	u16 cpu;

	u16 itr_val;
	u8 set_itr;
	void __iomem *itr_register;

#ifdef IGB_LRO
	struct igb_lro_list *lrolist;   /* LRO list for queue vector*/
#endif
	char name[IFNAMSIZ + 9];
#ifndef HAVE_NETDEV_NAPI_LIST
	struct net_device poll_dev;
#endif
};

struct igb_ring {
	struct igb_q_vector *q_vector; /* backlink to q_vector */
	struct net_device *netdev;     /* back pointer to net_device */
	struct pci_dev *pdev;          /* pci device for dma mapping */
	dma_addr_t dma;                /* phys address of the ring */
	void *desc;                    /* descriptor ring memory */
	unsigned int size;             /* length of desc. ring in bytes */
	u16 count;                     /* number of desc. in the ring */
	u16 next_to_use;
	u16 next_to_clean;
	u8 queue_index;
	u8 reg_idx;
	void __iomem *head;
	void __iomem *tail;
	struct igb_buffer *buffer_info; /* array of buffer info structs */

	unsigned int total_bytes;
	unsigned int total_packets;

	u32 flags;

	union {
		/* TX */
		struct {
			struct igb_tx_queue_stats tx_stats;
			bool detect_tx_hung;
		};
		/* RX */
		struct {
			struct igb_rx_queue_stats rx_stats;
			u32 rx_buffer_len;
		};
	};
};

#define IGB_RING_FLAG_RX_CSUM        0x00000001 /* RX CSUM enabled */
#define IGB_RING_FLAG_RX_SCTP_CSUM   0x00000002 /* SCTP CSUM offload enabled */
#ifdef IGB_LRO
#define IGB_RING_FLAG_RX_LRO         0x00000004 /* LRO enabled */
#endif /* IGB_LRO */

#define IGB_RING_FLAG_TX_CTX_IDX     0x00000001 /* HW requires context index */

#define IGB_ADVTXD_DCMD (E1000_ADVTXD_DCMD_EOP | E1000_ADVTXD_DCMD_RS)

#define E1000_RX_DESC_ADV(R, i)	    \
	(&(((union e1000_adv_rx_desc *)((R).desc))[i]))
#define E1000_TX_DESC_ADV(R, i)	    \
	(&(((union e1000_adv_tx_desc *)((R).desc))[i]))
#define E1000_TX_CTXTDESC_ADV(R, i)	    \
	(&(((struct e1000_adv_tx_context_desc *)((R).desc))[i]))

/* igb_desc_unused - calculate if we have unused descriptors */
static inline int igb_desc_unused(struct igb_ring *ring)
{
	if (ring->next_to_clean > ring->next_to_use)
		return ring->next_to_clean - ring->next_to_use - 1;

	return ring->count + ring->next_to_clean - ring->next_to_use - 1;
}

/* board specific private data structure */
struct igb_adapter {
	struct timer_list watchdog_timer;
	struct timer_list phy_info_timer;
	struct vlan_group *vlgrp;
	u16 mng_vlan_id;
	u32 bd_number;
	u32 wol;
	u32 en_mng_pt;
	u16 link_speed;
	u16 link_duplex;

	/* Interrupt Throttle Rate */
	u32 rx_itr_setting;
	u32 tx_itr_setting;
	u16 tx_itr;
	u16 rx_itr;

	struct work_struct reset_task;
	struct work_struct watchdog_task;
	bool fc_autoneg;
	u8  tx_timeout_factor;
#ifdef ETHTOOL_PHYS_ID
	struct timer_list blink_timer;
	unsigned long led_status;
#endif

	/* TX */
	struct igb_ring *tx_ring[IGB_MAX_TX_QUEUES];
	unsigned long tx_queue_len;
	u32 tx_timeout_count;

	/* RX */
	struct igb_ring *rx_ring[IGB_MAX_RX_QUEUES];
	int num_tx_queues;
	int num_rx_queues;

	u32 max_frame_size;
	u32 min_frame_size;

	/* OS defined structs */
	struct net_device *netdev;
	struct pci_dev *pdev;
#ifndef HAVE_NETDEV_STATS_IN_NETDEV
	struct net_device_stats net_stats;
#endif
#ifdef IGB_LRO
	struct igb_lro_stats lro_stats;
#endif
#ifdef SIOCSHWTSTAMP
	struct cyclecounter cycles;
	struct timecounter clock;
	struct timecompare compare;
	struct hwtstamp_config hwtstamp_config;
#endif

	/* structs defined in e1000_hw.h */
	struct e1000_hw hw;
	struct e1000_hw_stats stats;
	struct e1000_phy_info phy_info;
	struct e1000_phy_stats phy_stats;

#ifdef ETHTOOL_TEST
	u32 test_icr;
	struct igb_ring test_tx_ring;
	struct igb_ring test_rx_ring;
#endif

	int msg_enable;

	unsigned int num_q_vectors;
	struct igb_q_vector *q_vector[MAX_Q_VECTORS];
	struct msix_entry *msix_entries;
	u32 eims_enable_mask;
	u32 eims_other;

	/* to not mess up cache alignment, always add to the bottom */
	unsigned long state;
	unsigned int flags;
	u32 eeprom_wol;

	u32 *config_space;
#ifdef HAVE_TX_MQ
	struct igb_ring *multi_tx_table[IGB_MAX_TX_QUEUES];
#endif /* HAVE_TX_MQ */
	u16 tx_ring_count;
	u16 rx_ring_count;
	struct vf_data_storage *vf_data;
	u32 lli_port;
	u32 lli_size;
	unsigned int vfs_allocated_count;
	int int_mode;
	u32 rss_queues;
	u32 vmdq_pools;
};


#define IGB_FLAG_HAS_MSI           (1 << 0)
#define IGB_FLAG_MSI_ENABLE        (1 << 1)
#define IGB_FLAG_DCA_ENABLED       (1 << 3)
#define IGB_FLAG_LLI_PUSH          (1 << 4)
#define IGB_FLAG_QUAD_PORT_A       (1 << 5)
#define IGB_FLAG_QUEUE_PAIRS       (1 << 6)

#define IGB_82576_TSYNC_SHIFT 19
#define IGB_82580_TSYNC_SHIFT 24
#define IGB_TS_HDR_LEN        16
enum e1000_state_t {
	__IGB_TESTING,
	__IGB_RESETTING,
	__IGB_DOWN
};

extern char igb_driver_name[];
extern char igb_driver_version[];

extern int igb_up(struct igb_adapter *);
extern void igb_down(struct igb_adapter *);
extern void igb_reinit_locked(struct igb_adapter *);
extern void igb_reset(struct igb_adapter *);
extern int igb_set_spd_dplx(struct igb_adapter *, u16);
extern int igb_setup_tx_resources(struct igb_ring *);
extern int igb_setup_rx_resources(struct igb_ring *);
extern void igb_free_tx_resources(struct igb_ring *);
extern void igb_free_rx_resources(struct igb_ring *);
extern void igb_configure_tx_ring(struct igb_adapter *, struct igb_ring *);
extern void igb_configure_rx_ring(struct igb_adapter *, struct igb_ring *);
extern void igb_setup_tctl(struct igb_adapter *);
extern void igb_setup_rctl(struct igb_adapter *);
extern netdev_tx_t igb_xmit_frame_ring_adv(struct sk_buff *, struct igb_ring *);
extern void igb_unmap_and_free_tx_resource(struct igb_ring *,
                                           struct igb_buffer *);
extern void igb_alloc_rx_buffers_adv(struct igb_ring *, int);
extern void igb_update_stats(struct igb_adapter *);
extern void igb_set_ethtool_ops(struct net_device *);
extern void igb_check_options(struct igb_adapter *);
#ifdef ETHTOOL_OPS_COMPAT
extern int ethtool_ioctl(struct ifreq *);
#endif
extern s32 igb_vlvf_set(struct igb_adapter *, u32, bool, u32);
extern void igb_configure_vt_default_pool(struct igb_adapter *adapter);

#endif /* _IGB_H_ */
