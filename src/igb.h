/*******************************************************************************

  Intel(R) Gigabit Ethernet Linux driver
  Copyright(c) 2007 Intel Corporation.

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

#include "kcompat.h"

#include "e1000_api.h"
#include "e1000_82575.h"

#define BAR_0		0
#define BAR_1		1
#define BAR_5		5

#define INTEL_IGB_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}

struct igb_adapter;
#ifndef CONFIG_PCI_MSI
#undef CONFIG_IGB_MQ_RX
#endif

#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
#define IGB_DCA
#endif
#ifdef IGB_DCA
#include <linux/dca.h>
#endif

#define IGB_ERR(args...) printk(KERN_ERR "igb: " args)

#define PFX "igb: "
#define DPRINTK(nlevel, klevel, fmt, args...) \
	(void)((NETIF_MSG_##nlevel & adapter->msg_enable) && \
	printk(KERN_##klevel PFX "%s: %s: " fmt, adapter->netdev->name, \
		__FUNCTION__ , ## args))

/* Interrupt defines */
#define IGB_MAX_TX_CLEAN 72

#define IGB_MIN_DYN_ITR 3000
#define IGB_MAX_DYN_ITR 96000
#define IGB_START_ITR 6000

#define IGB_DYN_ITR_PACKET_THRESHOLD 2
#define IGB_DYN_ITR_LENGTH_LOW 200
#define IGB_DYN_ITR_LENGTH_HIGH 1000

/* TX/RX descriptor defines */
#define IGB_DEFAULT_TXD                  256
#define IGB_MIN_TXD                       80
#define IGB_MAX_TXD                     4096

#define IGB_DEFAULT_RXD                  256
#define IGB_MIN_RXD                       80
#define IGB_MAX_RXD                     4096

/* Transmit and receive queues */
#define IGB_MAX_RX_QUEUES                  4
#define IGB_MAX_TX_QUEUES                  4

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
#define IGB_TX_QUEUE_WAKE	16
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
	union {
		/* TX */
		struct {
			unsigned long time_stamp;
			u32 length;
		};
		/* RX */
		struct {
			struct page *page;
			u64 page_dma;
		};
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

	unsigned int total_bytes;
	unsigned int total_packets;

	union {
		/* TX */
		struct {
			spinlock_t tx_clean_lock;
#ifdef NETIF_F_LLTX
			spinlock_t tx_lock;
#endif
			bool detect_tx_hung;
			char name[IFNAMSIZ + 5];
		};
		/* RX */
		struct {
			/* arrays of page information for packet split */
			struct sk_buff *pending_skb;
			int pending_skb_page;
#ifdef CONFIG_IGB_MQ_RX
			int no_itr_adjust;
			struct igb_queue_stats rx_stats;
#endif

#ifndef CONFIG_IGB_SEPARATE_TX_HANDLER
			struct igb_ring *buddy;
#endif
			struct net_device *netdev;
		};
	};
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
	int set_itr;

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
	u32 gotc;
	u64 gotc_old;
	u64 tpt_old;
	u64 colc_old;
	u32 tx_timeout_count;

	/* RX */
	struct igb_ring *rx_ring;      /* One per active queue */
	int num_tx_queues;
	int num_rx_queues;

	u64 hw_csum_err;
	u64 hw_csum_good;
	u64 rx_hdr_split;
	u32 alloc_rx_buff_failed;
	boolean_t rx_csum;
	u32 gorc;
	u64 gorc_old;
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
	u32 eims_enable_mask;
	u32 lli_port;
	u32 lli_size;

	/* to not mess up cache alignment, always add to the bottom */
	unsigned long state;
	struct {
		unsigned int has_msi:1;
		unsigned int msi_enabled:1;
		unsigned int has_dca:1;
		unsigned int dca_enabled:1;
		unsigned int lli_push:1;
		unsigned int in_netpoll:1;
		unsigned int quad_port_a:1;
	} flags;
	u32 eeprom_wol;
	u32 *config_space;
};

enum e1000_state_t {
	__IGB_TESTING,
	__IGB_RESETTING,
	__IGB_DOWN
};
#endif /* _IGB_H_ */
