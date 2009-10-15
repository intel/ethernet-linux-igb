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

#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>

#ifdef SIOCETHTOOL
#include <linux/ethtool.h>
#endif

struct igb_adapter;

#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
#define IGB_DCA
#endif
#ifdef IGB_DCA
#include <linux/dca.h>
#endif

#ifdef IGB_LRO
#undef IGB_LRO
#ifdef NETIF_F_LRO
#if defined(CONFIG_INET_LRO) || defined(CONFIG_INET_LRO_MODULE)
#include <linux/inet_lro.h>
#define MAX_LRO_DESCRIPTORS		   8
#define IGB_LRO
#endif
#endif
#endif /* IGB_LRO */

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
#define IGB_INT_MODE_MSIX_1Q               2
#define IGB_INT_MODE_MSIX_MQ               3

#define HW_PERF
/* TX/RX descriptor defines */
#define IGB_DEFAULT_TXD                  256
#define IGB_MIN_TXD                       80
#define IGB_MAX_TXD                     4096

#define IGB_DEFAULT_RXD                  256
#define IGB_MIN_RXD                       80
#define IGB_MAX_RXD                     4096

#define IGB_MIN_ITR_USECS                 10 /* 100k irq/sec */
#define IGB_MAX_ITR_USECS              10000 /* 100  irq/sec */

/* Transmit and receive queues */
#ifndef CONFIG_IGB_SEPARATE_TX_HANDLER
#define IGB_MAX_RX_QUEUES                  (hw->mac.type > e1000_82575 ? 8 : 4)
#define IGB_ABS_MAX_TX_QUEUES              8
#else /* CONFIG_IGB_SEPARATE_TX_HANDLER */
#define IGB_MAX_RX_QUEUES                  4
#define IGB_ABS_MAX_TX_QUEUES              4
#endif  /* CONFIG_IGB_SEPARATE_TX_HANDLER */
#define IGB_MAX_TX_QUEUES                  IGB_MAX_RX_QUEUES

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
#define IGB_RX_PTHRESH                    16
#define IGB_RX_HTHRESH                     8
#define IGB_RX_WTHRESH                     1

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* Supported Rx Buffer Sizes */
#define IGB_RXBUFFER_128   128    /* Used for packet split */
#define IGB_RXBUFFER_256   256    /* Used for packet split */
#define IGB_RXBUFFER_512   512
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

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct igb_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	dma_addr_t page_dma;
	union {
		/* TX */
		struct {
			unsigned long time_stamp;
			u16 length;
			u16 next_to_watch;
		};

#ifndef CONFIG_IGB_DISABLE_PACKET_SPLIT
		/* RX */
		struct {
			unsigned long page_offset;
			struct page *page;
		};
#endif
	};
};

struct igb_queue_stats {
	u64 packets;
	u64 bytes;
};

struct igb_ring {
	struct igb_adapter *adapter; /* backlink */
	void *desc;                  /* descriptor ring memory */
	dma_addr_t dma;              /* phys address of the ring */
	unsigned int size;           /* length of desc. ring in bytes */
	unsigned int count;          /* number of desc. in the ring */
	u16 next_to_use;
	u16 next_to_clean;
	u16 head;
	u16 tail;
	struct igb_buffer *buffer_info; /* array of buffer info structs */

	u32 eims_value;
	u32 itr_val;
	u16 itr_register;
	u16 cpu;

	u16 queue_index;
	u16 reg_idx;

	unsigned int total_bytes;
	unsigned int total_packets;

	char name[IFNAMSIZ + 9];
	union {
		/* TX */
		struct {
			struct igb_queue_stats tx_stats;
			bool detect_tx_hung;
		};
		/* RX */
		struct {
			struct igb_queue_stats rx_stats;
			struct napi_struct napi;
			int set_itr;
			struct igb_ring *buddy;
#ifdef IGB_LRO
			struct net_lro_mgr lro_mgr;
			bool lro_used;
#endif
		};
	};
#ifndef HAVE_NETDEV_NAPI_LIST
	struct net_device poll_dev;
#endif
};


#define IGB_DESC_UNUSED(R) \
	((((R)->next_to_clean > (R)->next_to_use) ? 0 : (R)->count) + \
	(R)->next_to_clean - (R)->next_to_use - 1)

#define E1000_RX_DESC_ADV(R, i)	    \
	(&(((union e1000_adv_rx_desc *)((R).desc))[i]))
#define E1000_TX_DESC_ADV(R, i)	    \
	(&(((union e1000_adv_tx_desc *)((R).desc))[i]))
#define E1000_TX_CTXTDESC_ADV(R, i)	    \
	(&(((struct e1000_adv_tx_context_desc *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)

#define MAX_MSIX_COUNT 10
/* board specific private data structure */

struct igb_adapter {
	struct timer_list watchdog_timer;
	struct timer_list phy_info_timer;
	struct vlan_group *vlgrp;
	u16 mng_vlan_id;
	u32 bd_number;
	u32 rx_buffer_len;
	u32 wol;
	u32 en_mng_pt;
	u16 link_speed;
	u16 link_duplex;

	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;
	/* Interrupt Throttle Rate */
	u32 itr;
	u32 itr_setting;
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
	struct igb_ring *tx_ring;      /* One per active queue */
	unsigned int restart_queue;
	unsigned long tx_queue_len;
	u32 txd_cmd;
	u32 tx_timeout_count;

	/* RX */
	struct igb_ring *rx_ring;      /* One per active queue */
	int num_tx_queues;
	int num_rx_queues;

	u64 hw_csum_err;
	u64 hw_csum_good;
	u32 alloc_rx_buff_failed;
	u16 rx_ps_hdr_size;
	u32 max_frame_size;
	u32 min_frame_size;

	/* OS defined structs */
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct net_device_stats net_stats;

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
	struct msix_entry *msix_entries;
	int int_mode;
	u32 eims_enable_mask;
	u32 eims_other;
	u32 lli_port;
	u32 lli_size;
	u64 lli_int;
	unsigned long state;
	unsigned int flags;
	u32 eeprom_wol;
	u32 *config_space;
#ifdef HAVE_TX_MQ
	struct igb_ring *multi_tx_table[IGB_ABS_MAX_TX_QUEUES];
#endif /* HAVE_TX_MQ */
#ifdef IGB_LRO
	unsigned int lro_max_aggr;
	unsigned int lro_aggregated;
	unsigned int lro_flushed;
	unsigned int lro_no_desc;
#endif
	unsigned int tx_ring_count;
	unsigned int rx_ring_count;
};


#define IGB_FLAG_HAS_MSI           (1 << 0)
#define IGB_FLAG_MSI_ENABLE        (1 << 1)
#define IGB_FLAG_HAS_DCA           (1 << 2)
#define IGB_FLAG_DCA_ENABLED       (1 << 3)
#define IGB_FLAG_LLI_PUSH          (1 << 4)
#define IGB_FLAG_IN_NETPOLL        (1 << 5)
#define IGB_FLAG_QUAD_PORT_A       (1 << 6)
#define IGB_FLAG_NEED_CTX_IDX      (1 << 7)
#define IGB_FLAG_RX_CSUM_DISABLED  (1 << 8)

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
extern int igb_setup_tx_resources(struct igb_adapter *, struct igb_ring *);
extern int igb_setup_rx_resources(struct igb_adapter *, struct igb_ring *);
extern void igb_free_tx_resources(struct igb_ring *);
extern void igb_free_rx_resources(struct igb_ring *);
extern void igb_update_stats(struct igb_adapter *);
extern void igb_set_ethtool_ops(struct net_device *);
extern void igb_check_options(struct igb_adapter *);
#ifdef ETHTOOL_OPS_COMPAT
extern int ethtool_ioctl(struct ifreq *);
#endif

#endif /* _IGB_H_ */
