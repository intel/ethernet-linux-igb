/*******************************************************************************

  Intel(R) Gigabit Ethernet Linux driver
  Copyright(c) 2007-2010 Intel Corporation.

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


#include <linux/netdevice.h>

#include "igb.h"

/* This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */

#define IGB_MAX_NIC 32

#define OPTION_UNSET   -1
#define OPTION_DISABLED 0
#define OPTION_ENABLED  1

/* All parameters are treated the same, as an integer array of values.
 * This macro just reduces the need to repeat the same declaration code
 * over and over (plus this helps to avoid typo bugs).
 */

#define IGB_PARAM_INIT { [0 ... IGB_MAX_NIC] = OPTION_UNSET }
#ifndef module_param_array
/* Module Parameters are always initialized to -1, so that the driver
 * can tell the difference between no user specified value or the
 * user asking for the default value.
 * The true default values are loaded in when igb_check_options is called.
 *
 * This is a GCC extension to ANSI C.
 * See the item "Labeled Elements in Initializers" in the section
 * "Extensions to the C Language Family" of the GCC documentation.
 */

#define IGB_PARAM(X, desc) \
	static const int __devinitdata X[IGB_MAX_NIC+1] = IGB_PARAM_INIT; \
	MODULE_PARM(X, "1-" __MODULE_STRING(IGB_MAX_NIC) "i"); \
	MODULE_PARM_DESC(X, desc);
#else
#define IGB_PARAM(X, desc) \
	static int __devinitdata X[IGB_MAX_NIC+1] = IGB_PARAM_INIT; \
	static unsigned int num_##X; \
	module_param_array_named(X, X, int, &num_##X, 0); \
	MODULE_PARM_DESC(X, desc);
#endif

/* Interrupt Throttle Rate (interrupts/sec)
 *
 * Valid Range: 100-100000 (0=off, 1=dynamic, 3=dynamic conservative)
 */
IGB_PARAM(InterruptThrottleRate,
	  "Maximum interrupts per second, per vector, (max 100000), default 3=adaptive");
#define DEFAULT_ITR                    3
#define MAX_ITR                   100000
#define MIN_ITR                      120
/* IntMode (Interrupt Mode)
 *
 * Valid Range: 0 - 2
 *
 * Default Value: 2 (MSI-X)
 */
IGB_PARAM(IntMode, "Change Interrupt Mode (0=Legacy, 1=MSI, 2=MSI-X), default 2");
#define MAX_INTMODE                    IGB_INT_MODE_MSIX
#define MIN_INTMODE                    IGB_INT_MODE_LEGACY

IGB_PARAM(Node, "set the starting node to allocate memory on, default -1");

/* LLIPort (Low Latency Interrupt TCP Port)
 *
 * Valid Range: 0 - 65535
 *
 * Default Value: 0 (disabled)
 */
IGB_PARAM(LLIPort, "Low Latency Interrupt TCP Port (0-65535), default 0=off");

#define DEFAULT_LLIPORT                0
#define MAX_LLIPORT               0xFFFF
#define MIN_LLIPORT                    0

/* LLIPush (Low Latency Interrupt on TCP Push flag)
 *
 * Valid Range: 0, 1
 *
 * Default Value: 0 (disabled)
 */
IGB_PARAM(LLIPush, "Low Latency Interrupt on TCP Push flag (0,1), default 0=off");

#define DEFAULT_LLIPUSH                0
#define MAX_LLIPUSH                    1
#define MIN_LLIPUSH                    0

/* LLISize (Low Latency Interrupt on Packet Size)
 *
 * Valid Range: 0 - 1500
 *
 * Default Value: 0 (disabled)
 */
IGB_PARAM(LLISize, "Low Latency Interrupt on Packet Size (0-1500), default 0=off");

#define DEFAULT_LLISIZE                0
#define MAX_LLISIZE                 1500
#define MIN_LLISIZE                    0

/* RSS (Enable RSS multiqueue receive)
 *
 * Valid Range: 0 - 8
 *
 * Default Value:  1
 */
IGB_PARAM(RSS, "Number of Receive-Side Scaling Descriptor Queues (0-8), default 1=number of cpus");

#define DEFAULT_RSS       1
#define MAX_RSS          ((adapter->hw.mac.type == e1000_82575) ? 4 : 8)
#define MIN_RSS           0

/* VMDQ (Enable VMDq multiqueue receive)
 *
 * Valid Range: 0 - 8
 *
 * Default Value:  0
 */
IGB_PARAM(VMDQ, "Number of Virtual Machine Device Queues: 0-1 = disable, 2-8 enable, default 0");

#define DEFAULT_VMDQ      0
#define MAX_VMDQ          MAX_RSS
#define MIN_VMDQ          0

/* max_vfs (Enable SR-IOV VF devices)
 *
 * Valid Range: 0 - 7
 *
 * Default Value:  0
 */
IGB_PARAM(max_vfs, "Number of Virtual Functions: 0 = disable, 1-7 enable, default 0");

#define DEFAULT_SRIOV     0
#define MAX_SRIOV         7
#define MIN_SRIOV         0


/* QueuePairs (Enable TX/RX queue pairs for interrupt handling)
 *
 * Valid Range: 0 - 1
 *
 * Default Value:  1
 */
IGB_PARAM(QueuePairs, "Enable TX/RX queue pairs for interrupt handling (0,1), default 1=on");

#define DEFAULT_QUEUE_PAIRS           1
#define MAX_QUEUE_PAIRS               1
#define MIN_QUEUE_PAIRS               0

struct igb_option {
	enum { enable_option, range_option, list_option } type;
	const char *name;
	const char *err;
	int def;
	union {
		struct { /* range_option info */
			int min;
			int max;
		} r;
		struct { /* list_option info */
			int nr;
			struct igb_opt_list { int i; char *str; } *p;
		} l;
	} arg;
};

static int __devinit igb_validate_option(unsigned int *value,
                                         struct igb_option *opt,
                                         struct igb_adapter *adapter)
{
	if (*value == OPTION_UNSET) {
		*value = opt->def;
		return 0;
	}

	switch (opt->type) {
	case enable_option:
		switch (*value) {
		case OPTION_ENABLED:
			DPRINTK(PROBE, INFO, "%s Enabled\n", opt->name);
			return 0;
		case OPTION_DISABLED:
			DPRINTK(PROBE, INFO, "%s Disabled\n", opt->name);
			return 0;
		}
		break;
	case range_option:
		if (*value >= opt->arg.r.min && *value <= opt->arg.r.max) {
			DPRINTK(PROBE, INFO,
					"%s set to %d\n", opt->name, *value);
			return 0;
		}
		break;
	case list_option: {
		int i;
		struct igb_opt_list *ent;

		for (i = 0; i < opt->arg.l.nr; i++) {
			ent = &opt->arg.l.p[i];
			if (*value == ent->i) {
				if (ent->str[0] != '\0')
					DPRINTK(PROBE, INFO, "%s\n", ent->str);
				return 0;
			}
		}
	}
		break;
	default:
		BUG();
	}

	DPRINTK(PROBE, INFO, "Invalid %s value specified (%d) %s\n",
	       opt->name, *value, opt->err);
	*value = opt->def;
	return -1;
}

/**
 * igb_check_options - Range Checking for Command Line Parameters
 * @adapter: board private structure
 *
 * This routine checks all command line parameters for valid user
 * input.  If an invalid value is given, or if no user specified
 * value exists, a default value is used.  The final value is stored
 * in a variable in the adapter structure.
 **/

void __devinit igb_check_options(struct igb_adapter *adapter)
{
	int bd = adapter->bd_number;
	struct e1000_hw *hw = &adapter->hw;

	if (bd >= IGB_MAX_NIC) {
		DPRINTK(PROBE, NOTICE,
		       "Warning: no configuration for board #%d\n", bd);
		DPRINTK(PROBE, NOTICE, "Using defaults for all values\n");
#ifndef module_param_array
		bd = IGB_MAX_NIC;
#endif
	}

	{ /* Interrupt Throttling Rate */
		struct igb_option opt = {
			.type = range_option,
			.name = "Interrupt Throttling Rate (ints/sec)",
			.err  = "using default of " __MODULE_STRING(DEFAULT_ITR),
			.def  = DEFAULT_ITR,
			.arg  = { .r = { .min = MIN_ITR,
					 .max = MAX_ITR } }
		};

#ifdef module_param_array
		if (num_InterruptThrottleRate > bd) {
#endif
			unsigned int itr = InterruptThrottleRate[bd];

			switch (itr) {
			case 0:
				DPRINTK(PROBE, INFO, "%s turned off\n",
				        opt.name);
				break;
			case 1:
				DPRINTK(PROBE, INFO, "%s set to dynamic mode\n",
					opt.name);
				adapter->rx_itr_setting = itr;
				break;
			case 3:
				DPRINTK(PROBE, INFO,
				        "%s set to dynamic conservative mode\n",
					opt.name);
				adapter->rx_itr_setting = itr;
				break;
			default:
				igb_validate_option(&itr, &opt, adapter);
				/* Save the setting, because the dynamic bits
				 * change itr.  In case of invalid user value,
				 * default to conservative mode, else need to
				 * clear the lower two bits because they are
				 * used as control */
				if (itr == 3) {
					adapter->rx_itr_setting = itr;
				} else {
					adapter->rx_itr_setting = 1000000000 /
					                          (itr * 256);
					adapter->rx_itr_setting &= ~3;
				}
				break;
			}
#ifdef module_param_array
		} else {
			adapter->rx_itr_setting = opt.def;
		}
#endif
		adapter->tx_itr_setting = adapter->rx_itr_setting;
	}
	{ /* Interrupt Mode */
		struct igb_option opt = {
			.type = range_option,
			.name = "Interrupt Mode",
			.err  = "defaulting to 2 (MSI-X)",
			.def  = IGB_INT_MODE_MSIX,
			.arg  = { .r = { .min = MIN_INTMODE,
					 .max = MAX_INTMODE } }
		};

#ifdef module_param_array
		if (num_IntMode > bd) {
#endif
			unsigned int int_mode = IntMode[bd];
			igb_validate_option(&int_mode, &opt, adapter);
			adapter->int_mode = int_mode;
#ifdef module_param_array
		} else {
			adapter->int_mode = opt.def;
		}
#endif
	}
	{ /* Low Latency Interrupt TCP Port */
		struct igb_option opt = {
			.type = range_option,
			.name = "Low Latency Interrupt TCP Port",
			.err  = "using default of " __MODULE_STRING(DEFAULT_LLIPORT),
			.def  = DEFAULT_LLIPORT,
			.arg  = { .r = { .min = MIN_LLIPORT,
					 .max = MAX_LLIPORT } }
		};

#ifdef module_param_array
		if (num_LLIPort > bd) {
#endif
			adapter->lli_port = LLIPort[bd];
			if (adapter->lli_port) {
				igb_validate_option(&adapter->lli_port, &opt,
				        adapter);
			} else {
				DPRINTK(PROBE, INFO, "%s turned off\n",
					opt.name);
			}
#ifdef module_param_array
		} else {
			adapter->lli_port = opt.def;
		}
#endif
	}
	{ /* Low Latency Interrupt on Packet Size */
		struct igb_option opt = {
			.type = range_option,
			.name = "Low Latency Interrupt on Packet Size",
			.err  = "using default of " __MODULE_STRING(DEFAULT_LLISIZE),
			.def  = DEFAULT_LLISIZE,
			.arg  = { .r = { .min = MIN_LLISIZE,
					 .max = MAX_LLISIZE } }
		};

#ifdef module_param_array
		if (num_LLISize > bd) {
#endif
			adapter->lli_size = LLISize[bd];
			if (adapter->lli_size) {
				igb_validate_option(&adapter->lli_size, &opt,
				        adapter);
			} else {
				DPRINTK(PROBE, INFO, "%s turned off\n",
					opt.name);
			}
#ifdef module_param_array
		} else {
			adapter->lli_size = opt.def;
		}
#endif
	}
	{ /* Low Latency Interrupt on TCP Push flag */
		struct igb_option opt = {
			.type = enable_option,
			.name = "Low Latency Interrupt on TCP Push flag",
			.err  = "defaulting to Disabled",
			.def  = OPTION_DISABLED
		};

#ifdef module_param_array
		if (num_LLIPush > bd) {
#endif
			unsigned int lli_push = LLIPush[bd];
			igb_validate_option(&lli_push, &opt, adapter);
			adapter->flags |= lli_push ? IGB_FLAG_LLI_PUSH : 0;
#ifdef module_param_array
		} else {
			adapter->flags |= opt.def ? IGB_FLAG_LLI_PUSH : 0;
		}
#endif
	}
	{ /* SRIOV - Enable SR-IOV VF devices */
		struct igb_option opt = {
			.type = range_option,
			.name = "max_vfs - SR-IOV VF devices",
			.err  = "using default of " __MODULE_STRING(DEFAULT_SRIOV),
			.def  = DEFAULT_SRIOV,
			.arg  = { .r = { .min = MIN_SRIOV,
					 .max = MAX_SRIOV } }
		};

#ifdef module_param_array
		if (num_max_vfs > bd) {
#endif
			adapter->vfs_allocated_count = max_vfs[bd];
			igb_validate_option(&adapter->vfs_allocated_count, &opt, adapter);

#ifdef module_param_array
		} else {
			adapter->vfs_allocated_count = opt.def;
		}
#endif
		if (adapter->vfs_allocated_count) {
			switch (hw->mac.type) {
			case e1000_82575:
			case e1000_82580:
				adapter->vfs_allocated_count = 0;
				DPRINTK(PROBE, INFO, "SR-IOV option max_vfs not supported.\n");
			default:
				break;
			}
		}
	}
	{ /* VMDQ - Enable VMDq multiqueue receive */
		struct igb_option opt = {
			.type = range_option,
			.name = "VMDQ - VMDq multiqueue queue count",
			.err  = "using default of " __MODULE_STRING(DEFAULT_VMDQ),
			.def  = DEFAULT_VMDQ,
			.arg  = { .r = { .min = MIN_VMDQ,
					 .max = (MAX_VMDQ - adapter->vfs_allocated_count) } }
		};
#ifdef module_param_array
		if (num_VMDQ > bd) {
#endif
			adapter->vmdq_pools = (VMDQ[bd] == 1 ? 0 : VMDQ[bd]);
			if (adapter->vfs_allocated_count && !adapter->vmdq_pools) {
				DPRINTK(PROBE, INFO, "Enabling SR-IOV requires VMDq be set to at least 1\n");
				adapter->vmdq_pools = 1;
			}
			igb_validate_option(&adapter->vmdq_pools, &opt, adapter);

#ifdef module_param_array
		} else {
			if (!adapter->vfs_allocated_count)
				adapter->vmdq_pools = (opt.def == 1 ? 0 : opt.def);
			else
				adapter->vmdq_pools = 1;
		}
#endif
	}
	{ /* RSS - Enable RSS multiqueue receives */
		struct igb_option opt = {
			.type = range_option,
			.name = "RSS - RSS multiqueue receive count",
			.err  = "using default of " __MODULE_STRING(DEFAULT_RSS),
			.def  = DEFAULT_RSS,
			.arg  = { .r = { .min = MIN_RSS,
					 .max = MAX_RSS } }
		};

		if (adapter->vmdq_pools) {
			switch (hw->mac.type) {
			case e1000_82576:
				opt.arg.r.max = 2;
				break;
			case e1000_82575:
				if (adapter->vmdq_pools == 2)
					opt.arg.r.max = 3;
				if (adapter->vmdq_pools <= 2)
					break;
			default:
				opt.arg.r.max = 1;
				break;
			}
		}

#ifdef module_param_array
		if (num_RSS > bd) {
#endif
			adapter->rss_queues = RSS[bd];
			switch (adapter->rss_queues) {
			case 1:
				break;
			default:
				igb_validate_option(&adapter->rss_queues, &opt, adapter);
				if (adapter->rss_queues)
					break;
			case 0:
				adapter->rss_queues = min_t(u32, opt.arg.r.max, num_online_cpus());
				break;
			}
#ifdef module_param_array
		} else {
			adapter->rss_queues = opt.def;
		}
#endif
	}
	{ /* QueuePairs - Enable TX/RX queue pairs for interrupt handling */
		struct igb_option opt = {
			.type = enable_option,
			.name = "QueuePairs - TX/RX queue pairs for interrupt handling",
			.err  = "defaulting to Enabled",
			.def  = OPTION_ENABLED
		};
#ifdef module_param_array
		if (num_QueuePairs > bd) {
#endif
			unsigned int qp = QueuePairs[bd];
			/*
			 * we must enable queue pairs if the number of queues
			 * exceeds the number of avaialble interrupts.  We are
			 * limited to 10, or 3 per unallocated vf.
			 */
			if ((adapter->rss_queues > 4) ||
			    (adapter->vmdq_pools > 4) ||
			    ((adapter->rss_queues > 1) &&
			     ((adapter->vmdq_pools > 3) ||
			      (adapter->vfs_allocated_count > 6)))) {
				if (qp == OPTION_DISABLED) {
					qp = OPTION_ENABLED;
					DPRINTK(PROBE, INFO,
					        "Number of queues exceeds available interrupts, %s\n",opt.err);
				}
			}
			igb_validate_option(&qp, &opt, adapter);
			adapter->flags |= qp ? IGB_FLAG_QUEUE_PAIRS : 0;
#ifdef module_param_array
		} else {
			adapter->flags |= opt.def ? IGB_FLAG_QUEUE_PAIRS : 0;
		}
#endif
	}
	{ /* Node assignment */
		static struct igb_option opt = {
			.type = range_option,
			.name = "Node to start on",
			.err  = "defaulting to -1",
#ifdef HAVE_EARLY_VMALLOC_NODE
			.def  = 0,
#else
			.def  = -1,
#endif
			.arg  = { .r = { .min = 0,
					 .max = (MAX_NUMNODES - 1)}}
		};
		int node_param = opt.def;

		/* if the default was zero then we need to set the
		 * default value to an online node, which is not
		 * necessarily zero, and the constant initializer
		 * above can't take first_online_node */
		if (node_param == 0)
			/* must set opt.def for validate */
			opt.def = node_param = first_online_node;

#ifdef module_param_array
		if (num_Node > bd) {
#endif
			node_param = Node[bd];
			igb_validate_option((uint *)&node_param, &opt, adapter);

			if (node_param != OPTION_UNSET) {
				DPRINTK(PROBE, INFO, "node set to %d\n", node_param);
			}
#ifdef module_param_array
		}

		/* check sanity of the value */
		if (node_param != -1 && !node_online(node_param)) {
			DPRINTK(PROBE, INFO,
			        "ignoring node set to invalid value %d\n",
			        node_param);
			node_param = opt.def;
		}

#endif
		adapter->node = node_param;
	}
}

