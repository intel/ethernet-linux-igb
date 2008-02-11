/*******************************************************************************

  Intel(R) Gigabit Ethernet Linux driver
  Copyright(c) 2007-2008 Intel Corporation.

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
	static unsigned int num_##X = 0; \
	module_param_array_named(X, X, int, &num_##X, 0); \
	MODULE_PARM_DESC(X, desc);
#endif

/* Interrupt Throttle Rate (interrupts/sec)
 *
 * Valid Range: 100-100000 (0=off, 1=dynamic, 3=dynamic conservative)
 */
IGB_PARAM(InterruptThrottleRate, "Interrupt Throttling Rate");
#define DEFAULT_ITR                    3
#define MAX_ITR                   100000
#define MIN_ITR                      100
/* IntMode (Interrupt Mode)
 *
 * Valid Range: 0 - 3
 *
 * Default Value: 2 (MSI-X single queue)
 */
IGB_PARAM(IntMode, "Interrupt Mode");

#define DEFAULT_INTMODE                2
#define MAX_INTMODE                    3
#define MIN_INTMODE                    0

/* LLIPort (Low Latency Interrupt TCP Port)
 *
 * Valid Range: 0 - 65535
 *
 * Default Value: 0 (disabled)
 */
IGB_PARAM(LLIPort, "Low Latency Interrupt TCP Port");

#define DEFAULT_LLIPORT                0
#define MAX_LLIPORT               0xFFFF
#define MIN_LLIPORT                    0

/* LLIPush (Low Latency Interrupt on TCP Push flag)
 *
 * Valid Range: 0, 1
 *
 * Default Value: 0 (disabled)
 */
IGB_PARAM(LLIPush, "Low Latency Interrupt on TCP Push flag");

#define DEFAULT_LLIPUSH                0
#define MAX_LLIPUSH                    1
#define MIN_LLIPUSH                    0

/* LLISize (Low Latency Interrupt on Packet Size)
 *
 * Valid Range: 0 - 1500
 *
 * Default Value: 0 (disabled)
 */
IGB_PARAM(LLISize, "Low Latency Interrupt on Packet Size");

#define DEFAULT_LLISIZE                0
#define MAX_LLISIZE                 1500
#define MIN_LLISIZE                    0

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
					 .max = MAX_ITR }}
		};

#ifdef module_param_array
		if (num_InterruptThrottleRate > bd) {
#endif
			adapter->itr = InterruptThrottleRate[bd];
			switch (adapter->itr) {
			case 0:
				DPRINTK(PROBE, INFO, "%s turned off\n",
				        opt.name);
				break;
			case 1:
				DPRINTK(PROBE, INFO, "%s set to dynamic mode\n",
					opt.name);
				adapter->itr_setting = adapter->itr;
				adapter->itr = IGB_START_ITR;
				break;
			case 3:
				DPRINTK(PROBE, INFO,
				        "%s set to dynamic conservative mode\n",
					opt.name);
				adapter->itr_setting = adapter->itr;
				adapter->itr = IGB_START_ITR;
				break;
			default:
				igb_validate_option(&adapter->itr, &opt,
				        adapter);
				/* save the setting, because the dynamic bits change itr */
				/* in case of invalid user value, default to conservative mode,
 				 * else need to clear the lower two bits because they are
				 * used as control */
				if (adapter->itr == 3) {
					adapter->itr_setting = adapter->itr;
					adapter->itr = IGB_START_ITR;
				} 
				else {
					adapter->itr_setting = adapter->itr & ~3;
				}
				break;
			}
#ifdef module_param_array
		} else {
			adapter->itr_setting = opt.def;
			adapter->itr = 8000;
		}
#endif
	}
	{ /* Interrupt Mode */
		struct igb_option opt = {
			.type = range_option,
			.name = "Interrupt Mode",
			.err  = "defaulting to 2 (MSI-X single queue)",
			.def  = IGB_INT_MODE_MSIX_1Q,
			.arg  = { .r = { .min = MIN_INTMODE,
					 .max = MAX_INTMODE }}
		};

#ifdef module_param_array
		if (num_IntMode > bd) {
#endif
			unsigned int int_mode = IntMode[bd];
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) )
			/* < 2.6.24 can NOT support multiple rx queues */
			if (int_mode == IGB_INT_MODE_MSIX_MQ) {
				DPRINTK(PROBE, INFO,
				 "%s limited to (0-2) for < 2.6.24 kernels\n",
				 opt.name);
				opt.arg.r.max = IGB_INT_MODE_MSIX_1Q;
			}
#endif
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
					 .max = MAX_LLIPORT }}
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
					 .max = MAX_LLISIZE }}
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
}

