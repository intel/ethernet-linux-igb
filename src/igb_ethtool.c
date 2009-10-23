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

/* ethtool support for igb */

#include <linux/netdevice.h>
#include <linux/vmalloc.h>

#ifdef SIOCETHTOOL
#include <linux/ethtool.h>

#include "igb.h"
#include "igb_regtest.h"
#include <linux/if_vlan.h>

#ifdef ETHTOOL_OPS_COMPAT
#include "kcompat_ethtool.c"
#endif

#ifdef ETHTOOL_GSTATS
struct igb_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define IGB_STAT(m) sizeof(((struct igb_adapter *)0)->m), \
		      offsetof(struct igb_adapter, m)
static const struct igb_stats igb_gstrings_stats[] = {
	{ "rx_packets", IGB_STAT(stats.gprc) },
	{ "tx_packets", IGB_STAT(stats.gptc) },
	{ "rx_bytes", IGB_STAT(stats.gorc) },
	{ "tx_bytes", IGB_STAT(stats.gotc) },
	{ "rx_broadcast", IGB_STAT(stats.bprc) },
	{ "tx_broadcast", IGB_STAT(stats.bptc) },
	{ "rx_multicast", IGB_STAT(stats.mprc) },
	{ "tx_multicast", IGB_STAT(stats.mptc) },
	{ "rx_errors", IGB_STAT(net_stats.rx_errors) },
	{ "tx_errors", IGB_STAT(net_stats.tx_errors) },
	{ "tx_dropped", IGB_STAT(net_stats.tx_dropped) },
	{ "multicast", IGB_STAT(stats.mprc) },
	{ "collisions", IGB_STAT(stats.colc) },
	{ "rx_length_errors", IGB_STAT(net_stats.rx_length_errors) },
	{ "rx_over_errors", IGB_STAT(net_stats.rx_over_errors) },
	{ "rx_crc_errors", IGB_STAT(stats.crcerrs) },
	{ "rx_frame_errors", IGB_STAT(net_stats.rx_frame_errors) },
	{ "rx_no_buffer_count", IGB_STAT(stats.rnbc) },
	{ "rx_missed_errors", IGB_STAT(stats.mpc) },
	{ "tx_aborted_errors", IGB_STAT(stats.ecol) },
	{ "tx_carrier_errors", IGB_STAT(stats.tncrs) },
	{ "tx_fifo_errors", IGB_STAT(net_stats.tx_fifo_errors) },
	{ "tx_heartbeat_errors", IGB_STAT(net_stats.tx_heartbeat_errors) },
	{ "tx_window_errors", IGB_STAT(stats.latecol) },
	{ "tx_abort_late_coll", IGB_STAT(stats.latecol) },
	{ "tx_deferred_ok", IGB_STAT(stats.dc) },
	{ "tx_single_coll_ok", IGB_STAT(stats.scc) },
	{ "tx_multi_coll_ok", IGB_STAT(stats.mcc) },
	{ "tx_timeout_count", IGB_STAT(tx_timeout_count) },
	{ "tx_restart_queue", IGB_STAT(restart_queue) },
	{ "rx_long_length_errors", IGB_STAT(stats.roc) },
	{ "rx_short_length_errors", IGB_STAT(stats.ruc) },
	{ "rx_align_errors", IGB_STAT(stats.algnerrc) },
	{ "tx_tcp_seg_good", IGB_STAT(stats.tsctc) },
	{ "tx_tcp_seg_failed", IGB_STAT(stats.tsctfc) },
	{ "rx_flow_control_xon", IGB_STAT(stats.xonrxc) },
	{ "rx_flow_control_xoff", IGB_STAT(stats.xoffrxc) },
	{ "tx_flow_control_xon", IGB_STAT(stats.xontxc) },
	{ "tx_flow_control_xoff", IGB_STAT(stats.xofftxc) },
	{ "rx_long_byte_count", IGB_STAT(stats.gorc) },
	{ "rx_csum_offload_good", IGB_STAT(hw_csum_good) },
	{ "rx_csum_offload_errors", IGB_STAT(hw_csum_err) },
	{ "tx_dma_out_of_sync", IGB_STAT(stats.doosync) },
	{ "alloc_rx_buff_failed", IGB_STAT(alloc_rx_buff_failed) },
	{ "tx_smbus", IGB_STAT(stats.mgptc) },
	{ "rx_smbus", IGB_STAT(stats.mgprc) },
	{ "dropped_smbus", IGB_STAT(stats.mgpdc) },
#ifdef IGB_LRO
	{ "lro_aggregated", IGB_STAT(lro_aggregated) },
	{ "lro_flushed", IGB_STAT(lro_flushed) },
	{ "lro_no_desc", IGB_STAT(lro_no_desc) },
#endif
};

#define IGB_QUEUE_STATS_LEN \
	 ((((struct igb_adapter *)netdev_priv(netdev))->num_rx_queues + \
	  ((struct igb_adapter *)netdev_priv(netdev))->num_tx_queues) * \
	(sizeof(struct igb_queue_stats) / sizeof(u64)))
#define IGB_GLOBAL_STATS_LEN	\
	(sizeof(igb_gstrings_stats) / sizeof(struct igb_stats))
#define IGB_STATS_LEN (IGB_GLOBAL_STATS_LEN + IGB_QUEUE_STATS_LEN)
#endif /* ETHTOOL_GSTATS */
#ifdef ETHTOOL_TEST
static const char igb_gstrings_test[][ETH_GSTRING_LEN] = {
	"Register test  (offline)", "Eeprom test    (offline)",
	"Interrupt test (offline)", "Loopback test  (offline)",
	"Link test   (on/offline)"
};
#define IGB_TEST_LEN (sizeof(igb_gstrings_test) / ETH_GSTRING_LEN)
#endif /* ETHTOOL_TEST */

static int igb_get_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 status;

	if (hw->phy.media_type == e1000_media_type_copper) {

		ecmd->supported = (SUPPORTED_10baseT_Half |
		                   SUPPORTED_10baseT_Full |
		                   SUPPORTED_100baseT_Half |
		                   SUPPORTED_100baseT_Full |
		                   SUPPORTED_1000baseT_Full|
		                   SUPPORTED_Autoneg |
		                   SUPPORTED_TP);
		ecmd->advertising = ADVERTISED_TP;

		if (hw->mac.autoneg == 1) {
			ecmd->advertising |= ADVERTISED_Autoneg;
			/* the e1000 autoneg seems to match ethtool nicely */
			ecmd->advertising |= hw->phy.autoneg_advertised;
		}

		ecmd->port = PORT_TP;
		ecmd->phy_address = hw->phy.addr;
	} else {
		ecmd->supported   = (SUPPORTED_1000baseT_Full |
				     SUPPORTED_FIBRE |
				     SUPPORTED_Autoneg);

		ecmd->advertising = (ADVERTISED_1000baseT_Full |
				     ADVERTISED_FIBRE |
				     ADVERTISED_Autoneg);

		ecmd->port = PORT_FIBRE;
	}

	ecmd->transceiver = XCVR_INTERNAL;

	status = E1000_READ_REG(hw, E1000_STATUS);

	if (status & E1000_STATUS_LU) {

		if ((status & E1000_STATUS_SPEED_1000) ||
		    hw->phy.media_type != e1000_media_type_copper)
			ecmd->speed = SPEED_1000;
		else if (status & E1000_STATUS_SPEED_100)
			ecmd->speed = SPEED_100;
		else
			ecmd->speed = SPEED_10;

		if ((status & E1000_STATUS_FD) ||
		    hw->phy.media_type != e1000_media_type_copper)
			ecmd->duplex = DUPLEX_FULL;
		else
			ecmd->duplex = DUPLEX_HALF;
	} else {
		ecmd->speed = -1;
		ecmd->duplex = -1;
	}

	ecmd->autoneg = hw->mac.autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE;
	return 0;
}

static int igb_set_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	/* When SoL/IDER sessions are active, autoneg/speed/duplex
	 * cannot be changed */
	if (e1000_check_reset_block(hw)) {
		DPRINTK(DRV, ERR, "Cannot change link characteristics "
		        "when SoL/IDER is active.\n");
		return -EINVAL;
	}

	while (test_and_set_bit(__IGB_RESETTING, &adapter->state))
		msleep(1);

	if (ecmd->autoneg == AUTONEG_ENABLE) {
		hw->mac.autoneg = 1;
		hw->phy.autoneg_advertised = ecmd->advertising |
		                             ADVERTISED_TP |
		                             ADVERTISED_Autoneg;
		ecmd->advertising = hw->phy.autoneg_advertised;
		if (adapter->fc_autoneg)
			hw->fc.requested_mode = e1000_fc_default;
	} else {
		if (igb_set_spd_dplx(adapter, ecmd->speed + ecmd->duplex)) {
			clear_bit(__IGB_RESETTING, &adapter->state);
			return -EINVAL;
		}
	}

	/* reset the link */
	if (netif_running(adapter->netdev)) {
		igb_down(adapter);
		igb_up(adapter);
	} else
		igb_reset(adapter);

	clear_bit(__IGB_RESETTING, &adapter->state);
	return 0;
}

static void igb_get_pauseparam(struct net_device *netdev,
                               struct ethtool_pauseparam *pause)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	pause->autoneg =
		(adapter->fc_autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE);

	if (hw->fc.current_mode == e1000_fc_rx_pause)
		pause->rx_pause = 1;
	else if (hw->fc.current_mode == e1000_fc_tx_pause)
		pause->tx_pause = 1;
	else if (hw->fc.current_mode == e1000_fc_full) {
		pause->rx_pause = 1;
		pause->tx_pause = 1;
	}
}

static int igb_set_pauseparam(struct net_device *netdev,
                              struct ethtool_pauseparam *pause)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	int retval = 0;

	adapter->fc_autoneg = pause->autoneg;

	while (test_and_set_bit(__IGB_RESETTING, &adapter->state))
		msleep(1);

	if (adapter->fc_autoneg == AUTONEG_ENABLE) {
		hw->fc.requested_mode = e1000_fc_default;
		if (netif_running(adapter->netdev)) {
			igb_down(adapter);
			igb_up(adapter);
		} else {
			igb_reset(adapter);
		}
	} else {
		if (pause->rx_pause && pause->tx_pause)
			hw->fc.requested_mode = e1000_fc_full;
		else if (pause->rx_pause && !pause->tx_pause)
			hw->fc.requested_mode = e1000_fc_rx_pause;
		else if (!pause->rx_pause && pause->tx_pause)
			hw->fc.requested_mode = e1000_fc_tx_pause;
		else if (!pause->rx_pause && !pause->tx_pause)
			hw->fc.requested_mode = e1000_fc_none;

		hw->fc.current_mode = hw->fc.requested_mode;

		retval = ((hw->phy.media_type == e1000_media_type_copper) ?
			  e1000_force_mac_fc(hw) : hw->mac.ops.setup_link(hw));
	}

	clear_bit(__IGB_RESETTING, &adapter->state);
	return retval;
}

static u32 igb_get_rx_csum(struct net_device *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	return adapter->rx_ring[0].rx_csum;
}

static int igb_set_rx_csum(struct net_device *netdev, u32 data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		adapter->rx_ring[i].rx_csum = !!data;

	return 0;
}

static u32 igb_get_tx_csum(struct net_device *netdev)
{
	return (netdev->features & NETIF_F_IP_CSUM) != 0;
}

static int igb_set_tx_csum(struct net_device *netdev, u32 data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	if (data) {
#ifdef NETIF_F_IPV6_CSUM
		netdev->features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
		if (adapter->hw.mac.type >= e1000_82576)
			netdev->features |= NETIF_F_SCTP_CSUM;
	} else {
		netdev->features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
		                      NETIF_F_SCTP_CSUM);
#else
		netdev->features |= NETIF_F_IP_CSUM;
		if (adapter->hw.mac.type == e1000_82576)
			netdev->features |= NETIF_F_SCTP_CSUM;
	} else {
		netdev->features &= ~(NETIF_F_IP_CSUM | NETIF_F_SCTP_CSUM);
#endif
	}

	return 0;
}

#ifdef NETIF_F_TSO
static int igb_set_tso(struct net_device *netdev, u32 data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	int i;
	struct net_device *v_netdev;

	if (data) {
		netdev->features |= NETIF_F_TSO;
#ifdef NETIF_F_TSO6
		netdev->features |= NETIF_F_TSO6;
#endif
	} else {
		netdev->features &= ~NETIF_F_TSO;
#ifdef NETIF_F_TSO6
		netdev->features &= ~NETIF_F_TSO6;
#endif
		/* disable TSO on all VLANs if they're present */
		if (!adapter->vlgrp)
			goto tso_out;
		for (i = 0; i < VLAN_GROUP_ARRAY_LEN; i++) {
			v_netdev = vlan_group_get_device(adapter->vlgrp, i);
			if (!v_netdev)
				continue;

			v_netdev->features &= ~NETIF_F_TSO;
#ifdef NETIF_F_TSO6
			v_netdev->features &= ~NETIF_F_TSO6;
#endif
			vlan_group_set_device(adapter->vlgrp, i, v_netdev);
		}
	}

tso_out:
	DPRINTK(PROBE, INFO, "TSO is %s\n", data ? "Enabled" : "Disabled");
	return 0;
}
#endif /* NETIF_F_TSO */

static u32 igb_get_msglevel(struct net_device *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	return adapter->msg_enable;
}

static void igb_set_msglevel(struct net_device *netdev, u32 data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	adapter->msg_enable = data;
}

static int igb_get_regs_len(struct net_device *netdev)
{
#define IGB_REGS_LEN 551
	return IGB_REGS_LEN * sizeof(u32);
}

static void igb_get_regs(struct net_device *netdev,
	                 struct ethtool_regs *regs, void *p)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 *regs_buff = p;
	u8 i;

	memset(p, 0, IGB_REGS_LEN * sizeof(u32));

	regs->version = (1 << 24) | (hw->revision_id << 16) | hw->device_id;

	/* General Registers */
	regs_buff[0] = E1000_READ_REG(hw, E1000_CTRL);
	regs_buff[1] = E1000_READ_REG(hw, E1000_STATUS);
	regs_buff[2] = E1000_READ_REG(hw, E1000_CTRL_EXT);
	regs_buff[3] = E1000_READ_REG(hw, E1000_MDIC);
	regs_buff[4] = E1000_READ_REG(hw, E1000_SCTL);
	regs_buff[5] = E1000_READ_REG(hw, E1000_CONNSW);
	regs_buff[6] = E1000_READ_REG(hw, E1000_VET);
	regs_buff[7] = E1000_READ_REG(hw, E1000_LEDCTL);
	regs_buff[8] = E1000_READ_REG(hw, E1000_PBA);
	regs_buff[9] = E1000_READ_REG(hw, E1000_PBS);
	regs_buff[10] = E1000_READ_REG(hw, E1000_FRTIMER);
	regs_buff[11] = E1000_READ_REG(hw, E1000_TCPTIMER);

	/* NVM Register */
	regs_buff[12] = E1000_READ_REG(hw, E1000_EECD);

	/* Interrupt */
	/* Reading EICS for EICR because they read the
	 * same but EICS does not clear on read */
	regs_buff[13] = E1000_READ_REG(hw, E1000_EICS);
	regs_buff[14] = E1000_READ_REG(hw, E1000_EICS);
	regs_buff[15] = E1000_READ_REG(hw, E1000_EIMS);
	regs_buff[16] = E1000_READ_REG(hw, E1000_EIMC);
	regs_buff[17] = E1000_READ_REG(hw, E1000_EIAC);
	regs_buff[18] = E1000_READ_REG(hw, E1000_EIAM);
	/* Reading ICS for ICR because they read the
	 * same but ICS does not clear on read */
	regs_buff[19] = E1000_READ_REG(hw, E1000_ICS);
	regs_buff[20] = E1000_READ_REG(hw, E1000_ICS);
	regs_buff[21] = E1000_READ_REG(hw, E1000_IMS);
	regs_buff[22] = E1000_READ_REG(hw, E1000_IMC);
	regs_buff[23] = E1000_READ_REG(hw, E1000_IAC);
	regs_buff[24] = E1000_READ_REG(hw, E1000_IAM);
	regs_buff[25] = E1000_READ_REG(hw, E1000_IMIRVP);

	/* Flow Control */
	regs_buff[26] = E1000_READ_REG(hw, E1000_FCAL);
	regs_buff[27] = E1000_READ_REG(hw, E1000_FCAH);
	regs_buff[28] = E1000_READ_REG(hw, E1000_FCTTV);
	regs_buff[29] = E1000_READ_REG(hw, E1000_FCRTL);
	regs_buff[30] = E1000_READ_REG(hw, E1000_FCRTH);
	regs_buff[31] = E1000_READ_REG(hw, E1000_FCRTV);

	/* Receive */
	regs_buff[32] = E1000_READ_REG(hw, E1000_RCTL);
	regs_buff[33] = E1000_READ_REG(hw, E1000_RXCSUM);
	regs_buff[34] = E1000_READ_REG(hw, E1000_RLPML);
	regs_buff[35] = E1000_READ_REG(hw, E1000_RFCTL);
	regs_buff[36] = E1000_READ_REG(hw, E1000_MRQC);
	regs_buff[37] = E1000_READ_REG(hw, E1000_VT_CTL);

	/* Transmit */
	regs_buff[38] = E1000_READ_REG(hw, E1000_TCTL);
	regs_buff[39] = E1000_READ_REG(hw, E1000_TCTL_EXT);
	regs_buff[40] = E1000_READ_REG(hw, E1000_TIPG);
	regs_buff[41] = E1000_READ_REG(hw, E1000_DTXCTL);

	/* Wake Up */
	regs_buff[42] = E1000_READ_REG(hw, E1000_WUC);
	regs_buff[43] = E1000_READ_REG(hw, E1000_WUFC);
	regs_buff[44] = E1000_READ_REG(hw, E1000_WUS);
	regs_buff[45] = E1000_READ_REG(hw, E1000_IPAV);
	regs_buff[46] = E1000_READ_REG(hw, E1000_WUPL);

	/* MAC */
	regs_buff[47] = E1000_READ_REG(hw, E1000_PCS_CFG0);
	regs_buff[48] = E1000_READ_REG(hw, E1000_PCS_LCTL);
	regs_buff[49] = E1000_READ_REG(hw, E1000_PCS_LSTAT);
	regs_buff[50] = E1000_READ_REG(hw, E1000_PCS_ANADV);
	regs_buff[51] = E1000_READ_REG(hw, E1000_PCS_LPAB);
	regs_buff[52] = E1000_READ_REG(hw, E1000_PCS_NPTX);
	regs_buff[53] = E1000_READ_REG(hw, E1000_PCS_LPABNP);

	/* Statistics */
	regs_buff[54] = adapter->stats.crcerrs;
	regs_buff[55] = adapter->stats.algnerrc;
	regs_buff[56] = adapter->stats.symerrs;
	regs_buff[57] = adapter->stats.rxerrc;
	regs_buff[58] = adapter->stats.mpc;
	regs_buff[59] = adapter->stats.scc;
	regs_buff[60] = adapter->stats.ecol;
	regs_buff[61] = adapter->stats.mcc;
	regs_buff[62] = adapter->stats.latecol;
	regs_buff[63] = adapter->stats.colc;
	regs_buff[64] = adapter->stats.dc;
	regs_buff[65] = adapter->stats.tncrs;
	regs_buff[66] = adapter->stats.sec;
	regs_buff[67] = adapter->stats.htdpmc;
	regs_buff[68] = adapter->stats.rlec;
	regs_buff[69] = adapter->stats.xonrxc;
	regs_buff[70] = adapter->stats.xontxc;
	regs_buff[71] = adapter->stats.xoffrxc;
	regs_buff[72] = adapter->stats.xofftxc;
	regs_buff[73] = adapter->stats.fcruc;
	regs_buff[74] = adapter->stats.prc64;
	regs_buff[75] = adapter->stats.prc127;
	regs_buff[76] = adapter->stats.prc255;
	regs_buff[77] = adapter->stats.prc511;
	regs_buff[78] = adapter->stats.prc1023;
	regs_buff[79] = adapter->stats.prc1522;
	regs_buff[80] = adapter->stats.gprc;
	regs_buff[81] = adapter->stats.bprc;
	regs_buff[82] = adapter->stats.mprc;
	regs_buff[83] = adapter->stats.gptc;
	regs_buff[84] = adapter->stats.gorc;
	regs_buff[86] = adapter->stats.gotc;
	regs_buff[88] = adapter->stats.rnbc;
	regs_buff[89] = adapter->stats.ruc;
	regs_buff[90] = adapter->stats.rfc;
	regs_buff[91] = adapter->stats.roc;
	regs_buff[92] = adapter->stats.rjc;
	regs_buff[93] = adapter->stats.mgprc;
	regs_buff[94] = adapter->stats.mgpdc;
	regs_buff[95] = adapter->stats.mgptc;
	regs_buff[96] = adapter->stats.tor;
	regs_buff[98] = adapter->stats.tot;
	regs_buff[100] = adapter->stats.tpr;
	regs_buff[101] = adapter->stats.tpt;
	regs_buff[102] = adapter->stats.ptc64;
	regs_buff[103] = adapter->stats.ptc127;
	regs_buff[104] = adapter->stats.ptc255;
	regs_buff[105] = adapter->stats.ptc511;
	regs_buff[106] = adapter->stats.ptc1023;
	regs_buff[107] = adapter->stats.ptc1522;
	regs_buff[108] = adapter->stats.mptc;
	regs_buff[109] = adapter->stats.bptc;
	regs_buff[110] = adapter->stats.tsctc;
	regs_buff[111] = adapter->stats.iac;
	regs_buff[112] = adapter->stats.rpthc;
	regs_buff[113] = adapter->stats.hgptc;
	regs_buff[114] = adapter->stats.hgorc;
	regs_buff[116] = adapter->stats.hgotc;
	regs_buff[118] = adapter->stats.lenerrs;
	regs_buff[119] = adapter->stats.scvpc;
	regs_buff[120] = adapter->stats.hrmpc;

	for (i = 0; i < 4; i++)
		regs_buff[121 + i] = E1000_READ_REG(hw, E1000_SRRCTL(i));
	for (i = 0; i < 4; i++)
		regs_buff[125 + i] = E1000_READ_REG(hw, E1000_PSRTYPE(i));
	for (i = 0; i < 4; i++)
		regs_buff[129 + i] = E1000_READ_REG(hw, E1000_RDBAL(i));
	for (i = 0; i < 4; i++)
		regs_buff[133 + i] = E1000_READ_REG(hw, E1000_RDBAH(i));
	for (i = 0; i < 4; i++)
		regs_buff[137 + i] = E1000_READ_REG(hw, E1000_RDLEN(i));
	for (i = 0; i < 4; i++)
		regs_buff[141 + i] = E1000_READ_REG(hw, E1000_RDH(i));
	for (i = 0; i < 4; i++)
		regs_buff[145 + i] = E1000_READ_REG(hw, E1000_RDT(i));
	for (i = 0; i < 4; i++)
		regs_buff[149 + i] = E1000_READ_REG(hw, E1000_RXDCTL(i));

	for (i = 0; i < 10; i++)
		regs_buff[153 + i] = E1000_READ_REG(hw, E1000_EITR(i));
	for (i = 0; i < 8; i++)
		regs_buff[163 + i] = E1000_READ_REG(hw, E1000_IMIR(i));
	for (i = 0; i < 8; i++)
		regs_buff[171 + i] = E1000_READ_REG(hw, E1000_IMIREXT(i));
	for (i = 0; i < 16; i++)
		regs_buff[179 + i] = E1000_READ_REG(hw, E1000_RAL(i));
	for (i = 0; i < 16; i++)
		regs_buff[195 + i] = E1000_READ_REG(hw, E1000_RAH(i));

	for (i = 0; i < 4; i++)
		regs_buff[211 + i] = E1000_READ_REG(hw, E1000_TDBAL(i));
	for (i = 0; i < 4; i++)
		regs_buff[215 + i] = E1000_READ_REG(hw, E1000_TDBAH(i));
	for (i = 0; i < 4; i++)
		regs_buff[219 + i] = E1000_READ_REG(hw, E1000_TDLEN(i));
	for (i = 0; i < 4; i++)
		regs_buff[223 + i] = E1000_READ_REG(hw, E1000_TDH(i));
	for (i = 0; i < 4; i++)
		regs_buff[227 + i] = E1000_READ_REG(hw, E1000_TDT(i));
	for (i = 0; i < 4; i++)
		regs_buff[231 + i] = E1000_READ_REG(hw, E1000_TXDCTL(i));
	for (i = 0; i < 4; i++)
		regs_buff[235 + i] = E1000_READ_REG(hw, E1000_TDWBAL(i));
	for (i = 0; i < 4; i++)
		regs_buff[239 + i] = E1000_READ_REG(hw, E1000_TDWBAH(i));
	for (i = 0; i < 4; i++)
		regs_buff[243 + i] = E1000_READ_REG(hw, E1000_DCA_TXCTRL(i));

	for (i = 0; i < 4; i++)
		regs_buff[247 + i] = E1000_READ_REG(hw, E1000_IP4AT_REG(i));
	for (i = 0; i < 4; i++)
		regs_buff[251 + i] = E1000_READ_REG(hw, E1000_IP6AT_REG(i));
	for (i = 0; i < 32; i++)
		regs_buff[255 + i] = E1000_READ_REG(hw, E1000_WUPM_REG(i));
	for (i = 0; i < 128; i++)
		regs_buff[287 + i] = E1000_READ_REG(hw, E1000_FFMT_REG(i));
	for (i = 0; i < 128; i++)
		regs_buff[415 + i] = E1000_READ_REG(hw, E1000_FFVT_REG(i));
	for (i = 0; i < 4; i++)
		regs_buff[543 + i] = E1000_READ_REG(hw, E1000_FFLT_REG(i));

	regs_buff[547] = E1000_READ_REG(hw, E1000_TDFH);
	regs_buff[548] = E1000_READ_REG(hw, E1000_TDFT);
	regs_buff[549] = E1000_READ_REG(hw, E1000_TDFHS);
	regs_buff[550] = E1000_READ_REG(hw, E1000_TDFPC);

}

static int igb_get_eeprom_len(struct net_device *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	return adapter->hw.nvm.word_size * 2;
}

static int igb_get_eeprom(struct net_device *netdev,
                          struct ethtool_eeprom *eeprom, u8 *bytes)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u16 *eeprom_buff;
	int first_word, last_word;
	int ret_val = 0;
	u16 i;

	if (eeprom->len == 0)
		return -EINVAL;

	eeprom->magic = hw->vendor_id | (hw->device_id << 16);

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;

	eeprom_buff = kmalloc(sizeof(u16) *
			(last_word - first_word + 1), GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	if (hw->nvm.type == e1000_nvm_eeprom_spi)
		ret_val = e1000_read_nvm(hw, first_word,
		                         last_word - first_word + 1,
		                         eeprom_buff);
	else {
		for (i = 0; i < last_word - first_word + 1; i++) {
			ret_val = e1000_read_nvm(hw, first_word + i, 1,
			                              &eeprom_buff[i]);
			if (ret_val)
				break;
		}
	}

	/* Device's eeprom is always little-endian, word addressable */
	for (i = 0; i < last_word - first_word + 1; i++)
		le16_to_cpus(&eeprom_buff[i]);

	memcpy(bytes, (u8 *)eeprom_buff + (eeprom->offset & 1),
			eeprom->len);
	kfree(eeprom_buff);

	return ret_val;
}

static int igb_set_eeprom(struct net_device *netdev,
                          struct ethtool_eeprom *eeprom, u8 *bytes)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u16 *eeprom_buff;
	void *ptr;
	int max_len, first_word, last_word, ret_val = 0;
	u16 i;

	if (eeprom->len == 0)
		return -EOPNOTSUPP;

	if (eeprom->magic != (hw->vendor_id | (hw->device_id << 16)))
		return -EFAULT;

	max_len = hw->nvm.word_size * 2;

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;
	eeprom_buff = kmalloc(max_len, GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	ptr = (void *)eeprom_buff;

	if (eeprom->offset & 1) {
		/* need read/modify/write of first changed EEPROM word */
		/* only the second byte of the word is being modified */
		ret_val = e1000_read_nvm(hw, first_word, 1,
					    &eeprom_buff[0]);
		ptr++;
	}
	if (((eeprom->offset + eeprom->len) & 1) && (ret_val == 0)) {
		/* need read/modify/write of last changed EEPROM word */
		/* only the first byte of the word is being modified */
		ret_val = e1000_read_nvm(hw, last_word, 1,
		                  &eeprom_buff[last_word - first_word]);
	}

	/* Device's eeprom is always little-endian, word addressable */
	for (i = 0; i < last_word - first_word + 1; i++)
		le16_to_cpus(&eeprom_buff[i]);

	memcpy(ptr, bytes, eeprom->len);

	for (i = 0; i < last_word - first_word + 1; i++)
		cpu_to_le16s(&eeprom_buff[i]);

	ret_val = e1000_write_nvm(hw, first_word,
	                          last_word - first_word + 1, eeprom_buff);

	/* Update the checksum over the first part of the EEPROM if needed
	 * and flush shadow RAM for 82573 controllers */
	if ((ret_val == 0) && ((first_word <= NVM_CHECKSUM_REG)))
		e1000_update_nvm_checksum(hw);

	kfree(eeprom_buff);
	return ret_val;
}

static void igb_get_drvinfo(struct net_device *netdev,
                            struct ethtool_drvinfo *drvinfo)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	u16 eeprom_data;

	strncpy(drvinfo->driver,  igb_driver_name, 32);
	strncpy(drvinfo->version, igb_driver_version, 32);

	/* EEPROM image version # is reported as firmware version # for
	 * 82575 controllers */
	e1000_read_nvm(&adapter->hw, 5, 1, &eeprom_data);
	snprintf(drvinfo->fw_version, 32, "%d.%d-%d",
		 (eeprom_data & 0xF000) >> 12,
		 (eeprom_data & 0x0FF0) >> 4,
		 eeprom_data & 0x000F);

	strncpy(drvinfo->bus_info, pci_name(adapter->pdev), 32);
	drvinfo->n_stats = IGB_STATS_LEN;
	drvinfo->testinfo_len = IGB_TEST_LEN;
	drvinfo->regdump_len = igb_get_regs_len(netdev);
	drvinfo->eedump_len = igb_get_eeprom_len(netdev);
}

static void igb_get_ringparam(struct net_device *netdev,
                              struct ethtool_ringparam *ring)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	ring->rx_max_pending = IGB_MAX_RXD;
	ring->tx_max_pending = IGB_MAX_TXD;
	ring->rx_mini_max_pending = 0;
	ring->rx_jumbo_max_pending = 0;
	ring->rx_pending = adapter->rx_ring_count;
	ring->tx_pending = adapter->tx_ring_count;
	ring->rx_mini_pending = 0;
	ring->rx_jumbo_pending = 0;
}

static int igb_set_ringparam(struct net_device *netdev,
                             struct ethtool_ringparam *ring)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct igb_ring *temp_ring;
	int i, err;
	u16 new_rx_count, new_tx_count;

	if ((ring->rx_mini_pending) || (ring->rx_jumbo_pending))
		return -EINVAL;

	new_rx_count = min(ring->rx_pending, (u32)IGB_MAX_RXD);
	new_rx_count = max(new_rx_count, (u16)IGB_MIN_RXD);
	new_rx_count = ALIGN(new_rx_count, REQ_RX_DESCRIPTOR_MULTIPLE);

	new_tx_count = min(ring->tx_pending, (u32)IGB_MAX_TXD);
	new_tx_count = max(new_tx_count, (u16)IGB_MIN_TXD);
	new_tx_count = ALIGN(new_tx_count, REQ_TX_DESCRIPTOR_MULTIPLE);

	if ((new_tx_count == adapter->tx_ring_count) &&
	    (new_rx_count == adapter->rx_ring_count)) {
		/* nothing to do */
		return 0;
	}

	if (adapter->num_tx_queues > adapter->num_rx_queues)
		temp_ring = vmalloc(adapter->num_tx_queues * sizeof(struct igb_ring));
	else
		temp_ring = vmalloc(adapter->num_rx_queues * sizeof(struct igb_ring));
	if (!temp_ring)
		return -ENOMEM;

	while (test_and_set_bit(__IGB_RESETTING, &adapter->state))
		msleep(1);

	if (netif_running(adapter->netdev))
		igb_down(adapter);

	/*
	 * We can't just free everything and then setup again,
	 * because the ISRs in MSI-X mode get passed pointers
	 * to the tx and rx ring structs.
	 */
	if (new_tx_count != adapter->tx_ring_count) {
		memcpy(temp_ring, adapter->tx_ring,
		       adapter->num_tx_queues * sizeof(struct igb_ring));

		for (i = 0; i < adapter->num_tx_queues; i++) {
			temp_ring[i].count = new_tx_count;
			err = igb_setup_tx_resources(&temp_ring[i]);
			if (err) {
				while (i) {
					i--;
					igb_free_tx_resources(&temp_ring[i]);
				}
				goto err_setup;
			}
		}

		for (i = 0; i < adapter->num_tx_queues; i++)
			igb_free_tx_resources(&adapter->tx_ring[i]);

		memcpy(adapter->tx_ring, temp_ring,
		       adapter->num_tx_queues * sizeof(struct igb_ring));

		adapter->tx_ring_count = new_tx_count;
	}

	if (new_rx_count != adapter->rx_ring->count) {
		memcpy(temp_ring, adapter->rx_ring,
		       adapter->num_rx_queues * sizeof(struct igb_ring));

		for (i = 0; i < adapter->num_rx_queues; i++) {
			temp_ring[i].count = new_rx_count;
			err = igb_setup_rx_resources(&temp_ring[i]);
			if (err) {
				while (i) {
					i--;
					igb_free_rx_resources(&temp_ring[i]);
				}
				goto err_setup;
			}

		}

		for (i = 0; i < adapter->num_rx_queues; i++)
			igb_free_rx_resources(&adapter->rx_ring[i]);

		memcpy(adapter->rx_ring, temp_ring,
		       adapter->num_rx_queues * sizeof(struct igb_ring));

		adapter->rx_ring_count = new_rx_count;
	}

	err = 0;
err_setup:
	if (netif_running(adapter->netdev))
		igb_up(adapter);

	clear_bit(__IGB_RESETTING, &adapter->state);
	vfree(temp_ring);
	return err;
}

static bool reg_pattern_test(struct igb_adapter *adapter, u64 *data,
			     int reg, u32 mask, u32 write)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 pat, val;
	static const u32 _test[] =
		{0x5A5A5A5A, 0xA5A5A5A5, 0x00000000, 0xFFFFFFFF};
	for (pat = 0; pat < ARRAY_SIZE(_test); pat++) {
		E1000_WRITE_REG(hw, reg, (_test[pat] & write));
		val = E1000_READ_REG(hw, reg);
		if (val != (_test[pat] & write & mask)) {
			DPRINTK(DRV, ERR, "pattern test reg %04X failed: got "
			        "0x%08X expected 0x%08X\n",
			        E1000_REGISTER(hw, reg), val,
				(_test[pat] & write & mask));
			*data = E1000_REGISTER(hw, reg);
			return 1;
		}
	}

	return 0;
}

static bool reg_set_and_check(struct igb_adapter *adapter, u64 *data,
			      int reg, u32 mask, u32 write)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 val;
	E1000_WRITE_REG(hw, reg, write & mask);
	val = E1000_READ_REG(hw, reg);
	if ((write & mask) != (val & mask)) {
		DPRINTK(DRV, ERR, "set/check reg %04X test failed: got 0x%08X "
		        "expected 0x%08X\n", reg, (val & mask), (write & mask));
		*data = E1000_REGISTER(hw, reg);
		return 1;
	}

	return 0;
}

#define REG_PATTERN_TEST(reg, mask, write)                                     \
	do {                                                                   \
		if (reg_pattern_test(adapter, data, reg, mask, write))         \
			return 1;                                              \
	} while (0)

#define REG_SET_AND_CHECK(reg, mask, write)                                    \
	do {                                                                   \
		if (reg_set_and_check(adapter, data, reg, mask, write))              \
			return 1;                                              \
	} while (0)

static int igb_reg_test(struct igb_adapter *adapter, u64 *data)
{
	struct e1000_hw *hw = &adapter->hw;
	struct igb_reg_test *test;
	u32 value, before, after;
	u32 i, toggle;

	switch (adapter->hw.mac.type) {
	case e1000_82576:
		test = reg_test_82576;
		toggle = 0x7FFFF3FF;
		break;
	default:
		test = reg_test_82575;
		toggle = 0x7FFFF3FF;
		break;
	}

	/* Because the status register is such a special case,
	 * we handle it separately from the rest of the register
	 * tests.  Some bits are read-only, some toggle, and some
	 * are writable on newer MACs.
	 */
	before = E1000_READ_REG(hw, E1000_STATUS);
	value = (E1000_READ_REG(hw, E1000_STATUS) & toggle);
	E1000_WRITE_REG(hw, E1000_STATUS, toggle);
	after = E1000_READ_REG(hw, E1000_STATUS) & toggle;
	if (value != after) {
		DPRINTK(DRV, ERR, "failed STATUS register test got: "
		        "0x%08X expected: 0x%08X\n", after, value);
		*data = 1;
		return 1;
	}
	/* restore previous status */
	E1000_WRITE_REG(hw, E1000_STATUS, before);

	/* Perform the remainder of the register test, looping through
	 * the test table until we either fail or reach the null entry.
	 */
	while (test->reg) {
		for (i = 0; i < test->array_len; i++) {
			switch (test->test_type) {
			case PATTERN_TEST:
				REG_PATTERN_TEST(test->reg +
						(i * test->reg_offset),
						test->mask,
						test->write);
				break;
			case SET_READ_TEST:
				REG_SET_AND_CHECK(test->reg +
						(i * test->reg_offset),
						test->mask,
						test->write);
				break;
			case WRITE_NO_TEST:
				writel(test->write,
				       (adapter->hw.hw_addr + test->reg)
				        + (i * test->reg_offset));
				break;
			case TABLE32_TEST:
				REG_PATTERN_TEST(test->reg + (i * 4),
						test->mask,
						test->write);
				break;
			case TABLE64_TEST_LO:
				REG_PATTERN_TEST(test->reg + (i * 8),
						test->mask,
						test->write);
				break;
			case TABLE64_TEST_HI:
				REG_PATTERN_TEST((test->reg + 4) + (i * 8),
						test->mask,
						test->write);
				break;
			}
		}
		test++;
	}

	*data = 0;
	return 0;
}

static int igb_eeprom_test(struct igb_adapter *adapter, u64 *data)
{
	u16 temp;
	u16 checksum = 0;
	u16 i;

	*data = 0;
	/* Read and add up the contents of the EEPROM */
	for (i = 0; i < (NVM_CHECKSUM_REG + 1); i++) {
		if ((e1000_read_nvm(&adapter->hw, i, 1, &temp)) < 0) {
			*data = 1;
			break;
		}
		checksum += temp;
	}

	/* If Checksum is not Correct return error else test passed */
	if ((checksum != (u16) NVM_SUM) && !(*data))
		*data = 2;

	return *data;
}

static irqreturn_t igb_test_intr(int irq, void *data)
{
	struct igb_adapter *adapter = (struct igb_adapter *) data;
	struct e1000_hw *hw = &adapter->hw;

	adapter->test_icr |= E1000_READ_REG(hw, E1000_ICR);

	return IRQ_HANDLED;
}

static int igb_intr_test(struct igb_adapter *adapter, u64 *data)
{
	struct e1000_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	u32 mask, ics_mask, i = 0, shared_int = TRUE;
	u32 irq = adapter->pdev->irq;

	*data = 0;

	/* Hook up test interrupt handler just for this test */
	if (adapter->msix_entries) {
		if (request_irq(adapter->msix_entries[0].vector,
		                &igb_test_intr, 0, netdev->name, adapter)) {
			*data = 1;
			return -1;
		}
	} else if (adapter->flags & IGB_FLAG_HAS_MSI) {
		shared_int = FALSE;
		if (request_irq(irq, &igb_test_intr, 0, netdev->name, adapter)) {
			*data = 1;
			return -1;
		}
	} else if (!request_irq(irq, &igb_test_intr, IRQF_PROBE_SHARED,
	                        netdev->name, adapter)) {
		shared_int = FALSE;
	} else if (request_irq(irq, &igb_test_intr, IRQF_SHARED,
	         netdev->name, adapter)) {
		*data = 1;
		return -1;
	}
	DPRINTK(HW, INFO, "testing %s interrupt\n",
	        (shared_int ? "shared" : "unshared"));

	/* Disable all the interrupts */
	E1000_WRITE_REG(hw, E1000_IMC, ~0);
	msleep(10);

	/* Define all writable bits for ICS */
	switch (hw->mac.type) {
	case e1000_82575:
		ics_mask = 0x37F47EDD;
		break;
	case e1000_82576:
		ics_mask = 0x77D4FBFD;
		break;
	default:
		ics_mask = 0x7FFFFFFF;
		break;
	}

	/* Test each interrupt */
	for (; i < 31; i++) {
		/* Interrupt to test */
		mask = 1 << i;

		if (!(mask & ics_mask))
			continue;

		if (!shared_int) {
			/* Disable the interrupt to be reported in
			 * the cause register and then force the same
			 * interrupt and see if one gets posted.  If
			 * an interrupt was posted to the bus, the
			 * test failed.
			 */
			adapter->test_icr = 0;

			/* Flush any pending interrupts */
			E1000_WRITE_REG(hw, E1000_ICR, ~0);

			E1000_WRITE_REG(hw, E1000_IMC, mask);
			E1000_WRITE_REG(hw, E1000_ICS, mask);
			msleep(10);

			if (adapter->test_icr & mask) {
				*data = 3;
				break;
			}
		}

		/* Enable the interrupt to be reported in
		 * the cause register and then force the same
		 * interrupt and see if one gets posted.  If
		 * an interrupt was not posted to the bus, the
		 * test failed.
		 */
		adapter->test_icr = 0;

		/* Flush any pending interrupts */
		E1000_WRITE_REG(hw, E1000_ICR, ~0);

		E1000_WRITE_REG(hw, E1000_IMS, mask);
		E1000_WRITE_REG(hw, E1000_ICS, mask);
		msleep(10);

		if (!(adapter->test_icr & mask)) {
			*data = 4;
			break;
		}

		if (!shared_int) {
			/* Disable the other interrupts to be reported in
			 * the cause register and then force the other
			 * interrupts and see if any get posted.  If
			 * an interrupt was posted to the bus, the
			 * test failed.
			 */
			adapter->test_icr = 0;

			/* Flush any pending interrupts */
			E1000_WRITE_REG(hw, E1000_ICR, ~0);

			E1000_WRITE_REG(hw, E1000_IMC, ~mask);
			E1000_WRITE_REG(hw, E1000_ICS, ~mask);
			msleep(10);

			if (adapter->test_icr & mask) {
				*data = 5;
				break;
			}
		}
	}

	/* Disable all the interrupts */
	E1000_WRITE_REG(hw, E1000_IMC, ~0);
	msleep(10);

	/* Unhook test interrupt handler */
	if (adapter->msix_entries)
		free_irq(adapter->msix_entries[0].vector, adapter);
	else
		free_irq(irq, adapter);

	return *data;
}

static void igb_free_desc_rings(struct igb_adapter *adapter)
{
	igb_free_tx_resources(&adapter->test_tx_ring);
	igb_free_rx_resources(&adapter->test_rx_ring);
}

static int igb_setup_desc_rings(struct igb_adapter *adapter)
{
	struct igb_ring *tx_ring = &adapter->test_tx_ring;
	struct igb_ring *rx_ring = &adapter->test_rx_ring;
	int i, ret_val;

	/* Setup Tx descriptor ring and Tx buffers */
	tx_ring->count = IGB_DEFAULT_TXD;
	tx_ring->pdev = adapter->pdev;
	tx_ring->reg_idx = adapter->vfs_allocated_count;

	if (igb_setup_tx_resources(tx_ring)) {
		ret_val = 1;
		goto err_nomem;
	}

	igb_setup_tctl(adapter);
	igb_configure_tx_ring(adapter, tx_ring);

	for (i = 0; i < tx_ring->count; i++) {
		union e1000_adv_tx_desc *tx_desc;
		unsigned int size = 1024;
		struct sk_buff *skb = alloc_skb(size, GFP_KERNEL);

		if (!skb) {
			ret_val = 2;
			goto err_nomem;
		}
		skb_put(skb, size);
		tx_ring->buffer_info[i].skb = skb;
		tx_ring->buffer_info[i].length = skb->len;
		tx_ring->buffer_info[i].dma =
			pci_map_single(tx_ring->pdev, skb->data, skb->len,
				       PCI_DMA_TODEVICE);
		tx_desc = E1000_TX_DESC_ADV(*tx_ring, i);
		tx_desc->read.buffer_addr =
			cpu_to_le64(tx_ring->buffer_info[i].dma);
		tx_desc->read.olinfo_status =
			cpu_to_le32(skb->len << E1000_ADVTXD_PAYLEN_SHIFT);
		tx_desc->read.cmd_type_len = cpu_to_le32(skb->len);
		tx_desc->read.cmd_type_len |=
			cpu_to_le32(E1000_ADVTXD_DTYP_DATA |
			            E1000_ADVTXD_DCMD_DEXT);
		tx_desc->read.cmd_type_len |=
			cpu_to_le32(IGB_ADVTXD_DCMD |
			            E1000_ADVTXD_DTYP_DATA |
			            E1000_ADVTXD_DCMD_IFCS |
			            E1000_ADVTXD_DCMD_DEXT);
	}

	/* Setup Rx descriptor ring and Rx buffers */
	rx_ring->count = IGB_DEFAULT_RXD;
	rx_ring->pdev = adapter->pdev;
	rx_ring->rx_buffer_len = IGB_RXBUFFER_2048;
	rx_ring->reg_idx = adapter->vfs_allocated_count;

	if (igb_setup_rx_resources(rx_ring)) {
		ret_val = 3;
		goto err_nomem;
	}

	/* set the default queue to queue 0 of PF */
	E1000_WRITE_REG(&adapter->hw, E1000_MRQC,
	                adapter->vfs_allocated_count << 3); 

	/* enable receive ring */
	igb_setup_rctl(adapter);
	igb_configure_rx_ring(adapter, rx_ring);

	if (igb_alloc_rx_buffers_adv(rx_ring, rx_ring->count)) {
		ret_val = 4;
		goto err_nomem;
	}


	return 0;

err_nomem:
	igb_free_desc_rings(adapter);
	return ret_val;
}

static void igb_phy_disable_receiver(struct igb_adapter *adapter)
{
	/* Write out to PHY registers 29 and 30 to disable the Receiver. */
	e1000_write_phy_reg(&adapter->hw, 29, 0x001F);
	e1000_write_phy_reg(&adapter->hw, 30, 0x8FFC);
	e1000_write_phy_reg(&adapter->hw, 29, 0x001A);
	e1000_write_phy_reg(&adapter->hw, 30, 0x8FF0);
}

static int igb_integrated_phy_loopback(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_reg = 0;

	hw->mac.autoneg = FALSE;

	if (hw->phy.type == e1000_phy_m88) {
		/* Auto-MDI/MDIX Off */
		e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, 0x0808);
		/* reset to update Auto-MDI/MDIX */
		e1000_write_phy_reg(hw, PHY_CONTROL, 0x9140);
		/* autoneg off */
		e1000_write_phy_reg(hw, PHY_CONTROL, 0x8140);
	}

	ctrl_reg = E1000_READ_REG(hw, E1000_CTRL);

	/* force 1000, set loopback */
	e1000_write_phy_reg(hw, PHY_CONTROL, 0x4140);

	/* Now set up the MAC to the same speed/duplex as the PHY. */
	ctrl_reg = E1000_READ_REG(hw, E1000_CTRL);
	ctrl_reg &= ~E1000_CTRL_SPD_SEL; /* Clear the speed sel bits */
	ctrl_reg |= (E1000_CTRL_FRCSPD | /* Set the Force Speed Bit */
		     E1000_CTRL_FRCDPX | /* Set the Force Duplex Bit */
		     E1000_CTRL_SPD_1000 |/* Force Speed to 1000 */
		     E1000_CTRL_FD |	 /* Force Duplex to FULL */
		     E1000_CTRL_SLU);	 /* Set link up enable bit */

	if (hw->phy.type == e1000_phy_m88)
		ctrl_reg |= E1000_CTRL_ILOS; /* Invert Loss of Signal */

	E1000_WRITE_REG(hw, E1000_CTRL, ctrl_reg);

	/* Disable the receiver on the PHY so when a cable is plugged in, the
	 * PHY does not begin to autoneg when a cable is reconnected to the NIC.
	 */
	if (hw->phy.type == e1000_phy_m88)
		igb_phy_disable_receiver(adapter);

	udelay(500);

	return 0;
}

static int igb_set_phy_loopback(struct igb_adapter *adapter)
{
	return igb_integrated_phy_loopback(adapter);
}

static int igb_setup_loopback_test(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 reg;

	reg = E1000_READ_REG(hw, E1000_CTRL_EXT);

	/* use CTRL_EXT to identify link type as SGMII can appear as copper */
	if (reg & E1000_CTRL_EXT_LINK_MODE_MASK) {
		reg = E1000_READ_REG(hw, E1000_RCTL);
		reg |= E1000_RCTL_LBM_TCVR;
		E1000_WRITE_REG(hw, E1000_RCTL, reg);

		E1000_WRITE_REG(hw, E1000_SCTL, E1000_ENABLE_SERDES_LOOPBACK);

		reg = E1000_READ_REG(hw, E1000_CTRL);
		reg &= ~(E1000_CTRL_RFCE |
			 E1000_CTRL_TFCE |
			 E1000_CTRL_LRST);
		reg |= E1000_CTRL_SLU |
		       E1000_CTRL_FD;
		E1000_WRITE_REG(hw, E1000_CTRL, reg);

		/* Unset switch control to serdes energy detect */
		reg = E1000_READ_REG(hw, E1000_CONNSW);
		reg &= ~E1000_CONNSW_ENRGSRC;
		E1000_WRITE_REG(hw, E1000_CONNSW, reg);

		/* Set PCS register for forced speed */
		reg = E1000_READ_REG(hw, E1000_PCS_LCTL);
		reg &= ~E1000_PCS_LCTL_AN_ENABLE;     /* Disable Autoneg*/
		reg |= E1000_PCS_LCTL_FLV_LINK_UP |   /* Force link up */
		       E1000_PCS_LCTL_FSV_1000 |      /* Force 1000    */
		       E1000_PCS_LCTL_FDV_FULL |      /* SerDes Full duplex */
		       E1000_PCS_LCTL_FSD |           /* Force Speed */
		       E1000_PCS_LCTL_FORCE_LINK;     /* Force Link */
		E1000_WRITE_REG(hw, E1000_PCS_LCTL, reg);

		return 0;
	}

	return igb_set_phy_loopback(adapter);
}

static void igb_loopback_cleanup(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;
	u16 phy_reg;

	rctl = E1000_READ_REG(hw, E1000_RCTL);
	rctl &= ~(E1000_RCTL_LBM_TCVR | E1000_RCTL_LBM_MAC);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	hw->mac.autoneg = TRUE;
	e1000_read_phy_reg(hw, PHY_CONTROL, &phy_reg);
	if (phy_reg & MII_CR_LOOPBACK) {
		phy_reg &= ~MII_CR_LOOPBACK;
		e1000_write_phy_reg(hw, PHY_CONTROL, phy_reg);
		e1000_phy_commit(hw);
	}
}

static void igb_create_lbtest_frame(struct sk_buff *skb,
                                    unsigned int frame_size)
{
	memset(skb->data, 0xFF, frame_size);
	frame_size &= ~1;
	memset(&skb->data[frame_size / 2], 0xAA, frame_size / 2 - 1);
	memset(&skb->data[frame_size / 2 + 10], 0xBE, 1);
	memset(&skb->data[frame_size / 2 + 12], 0xAF, 1);
}

static int igb_check_lbtest_frame(struct sk_buff *skb, unsigned int frame_size)
{
	frame_size &= ~1;
	if (*(skb->data + 3) == 0xFF) {
		if ((*(skb->data + frame_size / 2 + 10) == 0xBE) &&
		   (*(skb->data + frame_size / 2 + 12) == 0xAF)) {
			return 0;
		}
	}
	return 13;
}

static int igb_run_loopback_test(struct igb_adapter *adapter)
{
	struct igb_ring *tx_ring = &adapter->test_tx_ring;
	struct igb_ring *rx_ring = &adapter->test_rx_ring;
	int i, j, k, l, lc, good_cnt, ret_val = 0;
	unsigned long time;

	writel(rx_ring->count - 1, rx_ring->tail);

	/* Calculate the loop count based on the largest descriptor ring
	 * The idea is to wrap the largest ring a number of times using 64
	 * send/receive pairs during each loop
	 */

	if (rx_ring->count <= tx_ring->count)
		lc = ((tx_ring->count / 64) * 2) + 1;
	else
		lc = ((rx_ring->count / 64) * 2) + 1;

	k = l = 0;
	for (j = 0; j <= lc; j++) { /* loop count loop */
		for (i = 0; i < 64; i++) { /* send the packets */
			igb_create_lbtest_frame(tx_ring->buffer_info[k].skb,
			                        1024);
			pci_dma_sync_single_for_device(tx_ring->pdev,
				tx_ring->buffer_info[k].dma,
				tx_ring->buffer_info[k].length,
				PCI_DMA_TODEVICE);
			if (unlikely(++k == tx_ring->count))
				k = 0;
		}
		writel(k, tx_ring->tail);
		msleep(200);

		time = jiffies; /* set the start time for the receive */
		good_cnt = 0;
		do { /* receive the sent packets */
			pci_dma_sync_single_for_cpu(rx_ring->pdev,
			                rx_ring->buffer_info[l].dma,
			                rx_ring->rx_buffer_len,
			                PCI_DMA_FROMDEVICE);

			ret_val = igb_check_lbtest_frame(
			                     rx_ring->buffer_info[l].skb, 1024);
			if (!ret_val)
				good_cnt++;
			if (unlikely(++l == rx_ring->count))
				l = 0;
			/* time + 20 msecs (200 msecs on 2.4) is more than
			 * enough time to complete the receives, if it's
			 * exceeded, break and error off
			 */
		} while (good_cnt < 64 && jiffies < (time + 20));
		if (good_cnt != 64) {
			ret_val = 13; /* ret_val is the same as mis-compare */
			break;
		}
		if (jiffies >= (time + 20)) {
			ret_val = 14; /* error code for time out error */
			break;
		}
	} /* end loop count loop */
	return ret_val;
}

static int igb_loopback_test(struct igb_adapter *adapter, u64 *data)
{
	/* PHY loopback cannot be performed if SoL/IDER
	 * sessions are active */
	if (e1000_check_reset_block(&adapter->hw)) {
		DPRINTK(DRV, ERR, "Cannot do PHY loopback test "
		        "when SoL/IDER is active.\n");
		*data = 0;
		goto out;
	}
	*data = igb_setup_desc_rings(adapter);
	if (*data)
		goto out;
	*data = igb_setup_loopback_test(adapter);
	if (*data)
		goto err_loopback;
	*data = igb_run_loopback_test(adapter);
	igb_loopback_cleanup(adapter);

err_loopback:
	igb_free_desc_rings(adapter);
out:
	return *data;
}

static int igb_link_test(struct igb_adapter *adapter, u64 *data)
{
	struct e1000_hw *hw = &adapter->hw;
	*data = 0;
	if (adapter->hw.phy.media_type == e1000_media_type_internal_serdes) {
		int i = 0;
		adapter->hw.mac.serdes_has_link = FALSE;

		/* On some blade server designs, link establishment
		 * could take as long as 2-3 minutes */
		do {
			e1000_check_for_link(&adapter->hw);
			if (adapter->hw.mac.serdes_has_link)
				return *data;
			msleep(20);
		} while (i++ < 3750);

		*data = 1;
	} else {
		e1000_check_for_link(&adapter->hw);
		if (adapter->hw.mac.autoneg)
			msleep(4000);

		if (!(E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU))
			*data = 1;
	}
	return *data;
}

static void igb_diag_test(struct net_device *netdev,
                          struct ethtool_test *eth_test, u64 *data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	u16 autoneg_advertised;
	u8 forced_speed_duplex, autoneg;
	bool if_running = netif_running(netdev);

	set_bit(__IGB_TESTING, &adapter->state);
	if (eth_test->flags == ETH_TEST_FL_OFFLINE) {
		/* Offline tests */

		/* save speed, duplex, autoneg settings */
		autoneg_advertised = adapter->hw.phy.autoneg_advertised;
		forced_speed_duplex = adapter->hw.mac.forced_speed_duplex;
		autoneg = adapter->hw.mac.autoneg;

		DPRINTK(HW, INFO, "offline testing starting\n");

		/* Link test performed before hardware reset so autoneg doesn't
		 * interfere with test result */
		if (igb_link_test(adapter, &data[4]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		if (if_running)
			/* indicate we're in test mode */
			dev_close(netdev);
		else
			igb_reset(adapter);

		if (igb_reg_test(adapter, &data[0]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		igb_reset(adapter);
		if (igb_eeprom_test(adapter, &data[1]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		igb_reset(adapter);
		if (igb_intr_test(adapter, &data[2]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		igb_reset(adapter);
		if (igb_loopback_test(adapter, &data[3]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		/* restore speed, duplex, autoneg settings */
		adapter->hw.phy.autoneg_advertised = autoneg_advertised;
		adapter->hw.mac.forced_speed_duplex = forced_speed_duplex;
		adapter->hw.mac.autoneg = autoneg;

		/* force this routine to wait until autoneg complete/timeout */
		adapter->hw.phy.autoneg_wait_to_complete = TRUE;
		igb_reset(adapter);
		adapter->hw.phy.autoneg_wait_to_complete = FALSE;

		clear_bit(__IGB_TESTING, &adapter->state);
		if (if_running)
			dev_open(netdev);
	} else {
		DPRINTK(HW, INFO, "online testing starting\n");
		/* Online tests */
		if (igb_link_test(adapter, &data[4]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		/* Online tests aren't run; pass by default */
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;

		clear_bit(__IGB_TESTING, &adapter->state);
	}
	msleep_interruptible(4 * 1000);
}

static int igb_wol_exclusion(struct igb_adapter *adapter,
                             struct ethtool_wolinfo *wol)
{
	struct e1000_hw *hw = &adapter->hw;
	int retval = 1; /* fail by default */

	switch (hw->device_id) {
	case E1000_DEV_ID_82575GB_QUAD_COPPER:
		/* WoL not supported */
		wol->supported = 0;
		break;
	case E1000_DEV_ID_82575EB_FIBER_SERDES:
	case E1000_DEV_ID_82576_FIBER:
	case E1000_DEV_ID_82576_SERDES:
		/* Wake events not supported on port B */
		if (E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_FUNC_1) {
			wol->supported = 0;
			break;
		}
		/* return success for non excluded adapter ports */
		retval = 0;
		break;
	case E1000_DEV_ID_82576_QUAD_COPPER:
		/* quad port adapters only support WoL on port A */
		if (!(adapter->flags & IGB_FLAG_QUAD_PORT_A)) {
			wol->supported = 0;
			break;
		}
		/* return success for non excluded adapter ports */
		retval = 0;
		break;
	default:
		/* dual port cards only support WoL on port A from now on
		 * unless it was enabled in the eeprom for port B
		 * so exclude FUNC_1 ports from having WoL enabled */
		if (E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_FUNC_1 &&
		    !adapter->eeprom_wol) {
			wol->supported = 0;
			break;
		}

		retval = 0;
	}

	return retval;
}

static void igb_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	wol->supported = WAKE_UCAST | WAKE_MCAST |
	                 WAKE_BCAST | WAKE_MAGIC;
	wol->wolopts = 0;

	/* this function will set ->supported = 0 and return 1 if wol is not
	 * supported by this hardware */
	if (igb_wol_exclusion(adapter, wol) ||
	    !device_can_wakeup(&adapter->pdev->dev))
		return;

	/* apply any specific unsupported masks here */
	switch (adapter->hw.device_id) {
	default:
		break;
	}

	if (adapter->wol & E1000_WUFC_EX)
		wol->wolopts |= WAKE_UCAST;
	if (adapter->wol & E1000_WUFC_MC)
		wol->wolopts |= WAKE_MCAST;
	if (adapter->wol & E1000_WUFC_BC)
		wol->wolopts |= WAKE_BCAST;
	if (adapter->wol & E1000_WUFC_MAG)
		wol->wolopts |= WAKE_MAGIC;

	return;
}

static int igb_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	if (wol->wolopts & (WAKE_PHY | WAKE_ARP | WAKE_MAGICSECURE))
		return -EOPNOTSUPP;

	if (igb_wol_exclusion(adapter, wol))
		return wol->wolopts ? -EOPNOTSUPP : 0;

	/* these settings will always override what we currently have */
	adapter->wol = 0;

	if (wol->wolopts & WAKE_UCAST)
		adapter->wol |= E1000_WUFC_EX;
	if (wol->wolopts & WAKE_MCAST)
		adapter->wol |= E1000_WUFC_MC;
	if (wol->wolopts & WAKE_BCAST)
		adapter->wol |= E1000_WUFC_BC;
	if (wol->wolopts & WAKE_MAGIC)
		adapter->wol |= E1000_WUFC_MAG;
	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	return 0;
}

/* bit defines for adapter->led_status */
#define IGB_LED_ON		0

static int igb_phys_id(struct net_device *netdev, u32 data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	unsigned long timeout;

	timeout = data * 1000;

	/*
	 *  msleep_interruptable only accepts unsigned int so we are limited
	 * in how long a duration we can wait
	 */
	if (!timeout || timeout > UINT_MAX)
		timeout = UINT_MAX;

	e1000_blink_led(hw);
	msleep_interruptible(timeout);

	e1000_led_off(hw);
	clear_bit(IGB_LED_ON, &adapter->led_status);
	e1000_cleanup_led(hw);

	return 0;
}

static int igb_set_coalesce(struct net_device *netdev,
			    struct ethtool_coalesce *ec)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	int i;

	if ((ec->rx_coalesce_usecs > IGB_MAX_ITR_USECS) ||
	    ((ec->rx_coalesce_usecs > 3) &&
	     (ec->rx_coalesce_usecs < IGB_MIN_ITR_USECS)) ||
	    (ec->rx_coalesce_usecs == 2))
		return -EINVAL;

	/* convert to rate of irq's per second */
	if (ec->rx_coalesce_usecs && ec->rx_coalesce_usecs <= 3) {
		adapter->itr = IGB_START_ITR;
		adapter->itr_setting = ec->rx_coalesce_usecs;
	} else {
		adapter->itr = ec->rx_coalesce_usecs << 2;
		adapter->itr_setting = adapter->itr;
	}

	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct igb_q_vector *q_vector = adapter->q_vector[i];
		q_vector->itr_val = adapter->itr;
		q_vector->set_itr = 1;
	}

	return 0;
}

static int igb_get_coalesce(struct net_device *netdev,
			    struct ethtool_coalesce *ec)
{
	struct igb_adapter *adapter = netdev_priv(netdev);

	if (adapter->itr_setting <= 3)
		ec->rx_coalesce_usecs = adapter->itr_setting;
	else
		ec->rx_coalesce_usecs = adapter->itr_setting >> 2;

	return 0;
}

static int igb_nway_reset(struct net_device *netdev)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	if (netif_running(netdev))
		igb_reinit_locked(adapter);
	return 0;
}

#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
static int igb_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return IGB_STATS_LEN;
	case ETH_SS_TEST:
		return IGB_TEST_LEN;
	default:
		return -ENOTSUPP;
	}
}
#else
static int igb_get_stats_count(struct net_device *netdev)
{
	return IGB_STATS_LEN;
}

static int igb_diag_test_count(struct net_device *netdev)
{
	return IGB_TEST_LEN;
}
#endif

static void igb_get_ethtool_stats(struct net_device *netdev,
                                  struct ethtool_stats *stats, u64 *data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	u64 *queue_stat;
	int stat_count = sizeof(struct igb_queue_stats) / sizeof(u64);
	int j;
	int i;
	u64 restart_queue = 0, hw_csum_err = 0, hw_csum_good = 0;
#ifdef IGB_LRO
	int aggregated = 0, flushed = 0, no_desc = 0;
#endif

	/* collect tx ring stats */
	for (i = 0; i < adapter->num_tx_queues; i++)
		restart_queue += adapter->tx_ring[i].restart_queue;
	adapter->restart_queue = restart_queue;


	for (i = 0; i < adapter->num_rx_queues; i++) {
		hw_csum_err += adapter->rx_ring[i].hw_csum_err;
		hw_csum_good += adapter->rx_ring[i].hw_csum_good;
#ifdef IGB_LRO
		aggregated += adapter->rx_ring[i].lro_mgr.stats.aggregated;
		flushed += adapter->rx_ring[i].lro_mgr.stats.flushed;
		no_desc += adapter->rx_ring[i].lro_mgr.stats.no_desc;
	}
	adapter->lro_aggregated = aggregated;
	adapter->lro_flushed = flushed;
	adapter->lro_no_desc = no_desc;
#else
	}
#endif
	adapter->hw_csum_err = hw_csum_err;
	adapter->hw_csum_good = hw_csum_good;

	igb_update_stats(adapter);

	for (i = 0; i < IGB_GLOBAL_STATS_LEN; i++) {
		char *p = (char *)adapter+igb_gstrings_stats[i].stat_offset;
		data[i] = (igb_gstrings_stats[i].sizeof_stat ==
			sizeof(u64)) ? *(u64 *)p : *(u32 *)p;
	}
	for (j = 0; j < adapter->num_tx_queues; j++) {
		int k;
		queue_stat = (u64 *)&adapter->tx_ring[j].stats;
		for (k = 0; k < stat_count; k++)
			data[i + k] = queue_stat[k];
		i += k;
	}
	for (j = 0; j < adapter->num_rx_queues; j++) {
		int k;
		queue_stat = (u64 *)&adapter->rx_ring[j].stats;
		for (k = 0; k < stat_count; k++)
			data[i + k] = queue_stat[k];
		i += k;
	}
}

static void igb_get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_TEST:
		memcpy(data, *igb_gstrings_test,
			IGB_TEST_LEN*ETH_GSTRING_LEN);
		break;
	case ETH_SS_STATS:
		for (i = 0; i < IGB_GLOBAL_STATS_LEN; i++) {
			memcpy(p, igb_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		for (i = 0; i < adapter->num_tx_queues; i++) {
			sprintf(p, "tx_queue_%u_packets", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "tx_queue_%u_bytes", i);
			p += ETH_GSTRING_LEN;
		}
		for (i = 0; i < adapter->num_rx_queues; i++) {
			sprintf(p, "rx_queue_%u_packets", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "rx_queue_%u_bytes", i);
			p += ETH_GSTRING_LEN;
		}
/*		BUG_ON(p - data != IGB_STATS_LEN * ETH_GSTRING_LEN); */
		break;
	}
}

static struct ethtool_ops igb_ethtool_ops = {
	.get_settings           = igb_get_settings,
	.set_settings           = igb_set_settings,
	.get_drvinfo            = igb_get_drvinfo,
	.get_regs_len           = igb_get_regs_len,
	.get_regs               = igb_get_regs,
	.get_wol                = igb_get_wol,
	.set_wol                = igb_set_wol,
	.get_msglevel           = igb_get_msglevel,
	.set_msglevel           = igb_set_msglevel,
	.nway_reset             = igb_nway_reset,
	.get_link               = ethtool_op_get_link,
	.get_eeprom_len         = igb_get_eeprom_len,
	.get_eeprom             = igb_get_eeprom,
	.set_eeprom             = igb_set_eeprom,
	.get_ringparam          = igb_get_ringparam,
	.set_ringparam          = igb_set_ringparam,
	.get_pauseparam         = igb_get_pauseparam,
	.set_pauseparam         = igb_set_pauseparam,
	.get_rx_csum            = igb_get_rx_csum,
	.set_rx_csum            = igb_set_rx_csum,
	.get_tx_csum            = igb_get_tx_csum,
	.set_tx_csum            = igb_set_tx_csum,
	.get_sg                 = ethtool_op_get_sg,
	.set_sg                 = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
	.get_tso                = ethtool_op_get_tso,
	.set_tso                = igb_set_tso,
#endif
#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
	.get_sset_count         = igb_get_sset_count,
#else
	.get_stats_count        = igb_get_stats_count,
	.self_test_count        = igb_diag_test_count,
#endif
	.self_test              = igb_diag_test,
	.get_strings            = igb_get_strings,
	.phys_id                = igb_phys_id,
	.get_ethtool_stats      = igb_get_ethtool_stats,
#ifdef ETHTOOL_GPERMADDR
	.get_perm_addr          = ethtool_op_get_perm_addr,
#endif
	.get_coalesce           = igb_get_coalesce,
	.set_coalesce           = igb_set_coalesce,
#ifdef NETIF_F_LRO
	.get_flags              = ethtool_op_get_flags,
	.set_flags              = ethtool_op_set_flags,
#endif
};

void igb_set_ethtool_ops(struct net_device *netdev)
{
	SET_ETHTOOL_OPS(netdev, &igb_ethtool_ops);
}
#endif	/* SIOCETHTOOL */
