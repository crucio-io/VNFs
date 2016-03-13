/*
  Copyright(c) 2010-2014 Intel Corporation.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#include <rte_launch.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>

#include "dppd_cfg.h"
#include "dppd_port_cfg.h"
#include "cqm.h"
#include "commands.h"
#include "dppd_args.h"
#include "main.h"
#include "display.h"
#include "handle_acl.h"

#include "handle_routing.h"
#include "handle_qinq_decap4.h"

static unsigned update_interval = 1000;
volatile int stop_dppd = 0; /* set to 1 to stop dppd */

#ifdef ENABLE_VERBOSITY
uint8_t verbose = 0;
#endif

static void print_rx_tx_info(void)
{
	uint32_t lcore_id = -1;
	while(dppd_core_next(&lcore_id) == 0) {
		for (uint8_t task_id = 0; task_id < lcore_cfg[lcore_id].nb_tasks; ++task_id) {
			struct task_args *targ = &lcore_cfg[lcore_id].targs[task_id];

			mprintf("Core %u:", lcore_id);
			if (targ->rx_port != NO_PORT_AVAIL) {
				mprintf(" RX port %u (queue %u)", targ->rx_port, targ->rx_queue);
			}
			else {
				for (uint8_t j = 0; j < targ->nb_rxrings; ++j) {
					mprintf(" RX ring[%u,%u] %p", task_id, j, targ->rx_rings[j]);
				}
			}
			mprintf(" ==>");
			for (uint8_t j = 0; j < targ->nb_txports; ++j) {
				mprintf(" TX port %u (queue %u)", targ->tx_port_queue[j].port,
					targ->tx_port_queue[j].queue);
			}

			for (uint8_t j = 0; j < targ->nb_txrings; ++j) {
				mprintf(" TX ring %p", targ->tx_rings[j]);
			}

			mprintf("\n");
		}
	}
}

static void process_input(struct screen_state *screen_state, int* resize)
{
	static unsigned lcore_id, task_id, nb_packets, val, id,
		port, queue, rate, ip[4], prefix, next_hop_idx,
		interface, gre_id, svlan, cvlan, mac[6], user;
	static char mode[20];
	const char *str = get_key(screen_state, resize);
	struct rte_ring *ring;

	if (str == NULL) {
		// No command entered
	}
	else if (sscanf(str, "dump %u %u %u", &lcore_id, &task_id, &nb_packets) == 3) {
		if (lcore_id >= RTE_MAX_LCORE) {
			mprintf("Invalid core id %u (lcore ID above %d)\n", lcore_id, RTE_MAX_LCORE);
		}
		else if (!dppd_core_active(lcore_id)) {
			mprintf("Invalid core id %u (lcore is not active)\n", lcore_id);
		}
		else {
			cmd_dump(lcore_id, task_id, nb_packets);
		}
	}
	else if (sscanf(str, "rate %u %u %u", &queue, &port, &rate) == 3) {
		if (port > DPPD_MAX_PORTS) {
			mprintf("Max port id allowed is %u (specified %u)\n", DPPD_MAX_PORTS, port);
		}
		else if (!dppd_port_cfg[port].active) {
			mprintf("Port %u not active\n", port);
		}
		else if (queue >= dppd_port_cfg[port].n_txq) {
			mprintf("Number of active queues is %u\n",
				dppd_port_cfg[port].n_txq);
		}
		else if (rate > dppd_port_cfg[port].link_speed) {
			mprintf("Max rate allowed on port %u queue %u is %u Mbps\n",
				port, queue, dppd_port_cfg[port].link_speed);
		}
		else {
			if (rate == 0) {
				mprintf("Disabling rate limiting on port %u queue %u\n",
					port, queue);
			}
			else {
				mprintf("Setting rate limiting to %u Mbps on port %u queue %u\n",
					rate, port, queue);
			}
			rte_eth_set_queue_rate_limit(port, queue, rate);
		}
	}
#ifdef ENABLE_VERBOSITY
	else if (sscanf(str, "verbose %u", &id) == 1) {
		verbose = id;
	}
#endif
	else if (sscanf(str, "stat %10s %u", mode, &id) == 2) {
		cmd_stat(mode, id);
	}
	else if (sscanf(str, "stop %u", &lcore_id) == 1) {
		uint64_t mask = __UINT64_C(1) << lcore_id;
		if ((lcore_id >= RTE_MAX_LCORE) || !(dppd_cfg.core_mask & mask)) {
			mprintf("Invalid core id %u\n", lcore_id);
		}
		else {
			mprintf("Sending Stop to core #%u\n", lcore_id);
			stop_core(mask);
		}
	}
	else if (sscanf(str, "start %u", &lcore_id) == 1) {
		uint64_t mask = __UINT64_C(1) << lcore_id;
		if ((lcore_id >= RTE_MAX_LCORE) || !(dppd_cfg.core_mask & mask)) {
			mprintf("Invalid core id %u\n", lcore_id);
		}
		else {
			mprintf("Sending Start to core #%u\n", lcore_id);
			start_core(mask);
		}
	}
	else if (strcmp(str, "stop all") == 0) {
		mprintf("Sending Stop to set of cores 0x%lx\n", dppd_cfg.core_mask);
		stop_core(dppd_cfg.core_mask);
	}
	else if (strcmp(str, "start all") == 0) {
		mprintf("Sending Start to set of cores 0x%lx\n", dppd_cfg.core_mask);
		start_core(dppd_cfg.core_mask);
	}
	else if (strcmp(str, "reset stats") == 0) {
		reset_stats();
	}
	else if (sscanf(str, "update interval %u", &val) == 1) {
		if (val < 10) {
			mprintf("Minimum update interval is 10 ms\n");
		}
		else {
			mprintf("Setting update interval to %d ms\n", val);
			update_interval = val;
		}
	}
	else if (sscanf(str, "port info %u", &val) == 1) {
		cmd_portinfo(val);
	}
	else if (sscanf(str, "port up %u", &val) == 1) {
		cmd_port_up(val);
	}
	else if (sscanf(str, "port down %u", &val) == 1) {
		cmd_port_down(val);
	}
	else if (sscanf(str, "ring info %u %u", &lcore_id, &task_id) == 2) {
		cmd_ringinfo(lcore_id, task_id);
	}
	else if (strcmp(str, "ring info all") == 0) {
		cmd_ringinfo_all();
	}
	else if (strncmp(str, "rule add ", 9) == 0) {
		str += strlen("rule add ");

		char *space = strstr(str, " ");
		if (space) {
			space = strstr(space + 1, " ");
		}
		else {
			mprintf("Error in arp add command (see help)\n");
		}

		*space = 0;
		if (sscanf(str, "%u %u", &lcore_id, &task_id) != 2) {
			mprintf("%s\n", str);
			mprintf("Error parsing core and task id in arp add command (see help)\n");
		}
		else {
			str = space + 1;
			char *fields[9];
			char str_cpy[255];
			strncpy(str_cpy, str, 255);
			// example add rule command: rule add 15 0 1&0x0fff 1&0x0fff 0&0 128.0.0.0/1 128.0.0.0/1 5000-5000 5000-5000 allow
			int ret = rte_strsplit(str_cpy, 255, fields, 9, ' ');
			if (ret != 8) {
				mprintf("Error parsing add rule command\n");
				return ;
			}

			struct acl4_rule rule;
			struct acl4_rule* prule = &rule;
			if (str_to_rule(&rule, fields, -1, 1) == 0) {
				ring = ctrl_rings[lcore_id*MAX_TASKS_PER_CORE + task_id];
				if (lcore_id >= RTE_MAX_LCORE || !lcore_cfg[lcore_id].nb_tasks) {
					mprintf("Error adding route: Core %u not active\n", lcore_id);
				}
				else if (task_id >= lcore_cfg[lcore_id].nb_tasks) {
					mprintf("Error adding rule: Task %u is not active on core %u\n", task_id, lcore_id);
				}
				else if (!ring) {
					mprintf("No ring for control messages to core %u task %u\n", lcore_id, task_id);
				}
				else {
					while (rte_ring_sp_enqueue_bulk(ring, (void *const *)&prule, 1));
					while (!rte_ring_empty(ring));
				}
			}
			else {
				mprintf("Error in rule add command (see help)\n");
			}
		}

	}
	else if (sscanf(str, "route add %u %u %u.%u.%u.%u/%u %u", &lcore_id, &task_id,
			ip, ip + 1, ip + 2, ip + 3, &prefix, &next_hop_idx) == 8) {
		ring = ctrl_rings[lcore_id*MAX_TASKS_PER_CORE + task_id];
		if (lcore_id >= RTE_MAX_LCORE || !lcore_cfg[lcore_id].nb_tasks) {
			mprintf("Error adding route: Core %u not active\n", lcore_id);
		}
		else if (task_id >= lcore_cfg[lcore_id].nb_tasks) {
			mprintf("Error adding route: Task %u is not active on core %u\n", task_id, lcore_id);
		}
		else if (!ring) {
			mprintf("No ring for control messages to core %u task %u\n", lcore_id, task_id);
		}
		else {
			struct route_msg rmsg;
			struct route_msg *pmsg = &rmsg;

			rmsg.ip_bytes[0] = ip[0];
			rmsg.ip_bytes[1] = ip[1];
			rmsg.ip_bytes[2] = ip[2];
			rmsg.ip_bytes[3] = ip[3];
			rmsg.prefix = prefix;
			rmsg.nh = next_hop_idx;
			while (rte_ring_sp_enqueue_bulk(ring, (void *const *)&pmsg, 1));
			while (!rte_ring_empty(ring));
		}
	}
	else if (!strncmp(str, "arp add ", 8)) {
		struct arp_msg amsg;
		struct arp_msg *pmsg = &amsg;
		str = str + strlen("arp add ");

		char *space = strstr(str, " ");
		if (space) {
			space = strstr(space + 1, " ");
		}
		else {
			mprintf("Error in arp add command (see help)\n");
		}

		*space = 0;
		if (sscanf(str, "%u %u", &lcore_id, &task_id) != 2) {
			mprintf("%s\n", str);
			mprintf("Error parsing core and task id in arp add command (see help)\n");
		}
		else {

			str = space + 1;

			if (str_to_arp_msg(&amsg, str) == 0) {
				ring = ctrl_rings[lcore_id*MAX_TASKS_PER_CORE + task_id];
				if (lcore_id >= RTE_MAX_LCORE || !lcore_cfg[lcore_id].nb_tasks) {
					mprintf("Error adding route: Core %u not active\n", lcore_id);
				}
				else if (task_id >= lcore_cfg[lcore_id].nb_tasks) {
					mprintf("Error adding route: Task %u is not active on core %u\n", task_id, lcore_id);
				}
				else if (!ring) {
					mprintf("No ring for control messages to core %u task %u\n", lcore_id, task_id);
				}
				else {

					while (rte_ring_sp_enqueue_bulk(ring, (void *const *)&pmsg, 1));
					while (!rte_ring_empty(ring));
				}
			}
			else {
				mprintf("Error in arp add command (see help)\n");
			}
		}
	}
	else if (strcmp(str, "help") == 0) {
		mprintf("Available commands:\n");
		mprintf("\tstart all\n");
		mprintf("\tstop all\n");
		mprintf("\tstart <core id>\n");
		mprintf("\tstop <core id>\n");
		mprintf("\tverbose <level>\n");
		mprintf("\tstat <kind> <value>\n");
		mprintf("\treset stats\n");
		mprintf("\tupdate interval <value>\n");
		mprintf("\tport info <port id>\n");
		mprintf("\tport down <port id>\n");
		mprintf("\tport up <port id>\n");
		mprintf("\tring info <core id> <task id>\n");
		mprintf("\tring info all\n");
		mprintf("\tdump <core id> <task id> <nb packets>\n");
		mprintf("\trate <port id> <queue id> <rate> (rate does not include preamble, SFD and IFG)\n");
		mprintf("\trule add <core id> <task id> svlan_id&mask cvlan_id&mask ip_proto&mask source_ip/prefix destination_ip/prefix sport_range dport_range action\n");
		mprintf("\troute add <core id> <task id> <ip/prefix> <next hop id> (for example, route add 10.0.16.0/24 9)\n");
		mprintf("\tarp add <core id> <task id> <port id> <gre id> <svlan> <cvlan> <ip addr> <mac addr> <user>\n");
	}
	else {
		mprintf("Unknown command: %s\n", str);
	}
}

static void update_link_states(void)
{
	struct dppd_port_cfg *port_cfg;
	struct rte_eth_link link;

	for (uint8_t portid = 0; portid < DPPD_MAX_PORTS; ++portid) {
		if (!dppd_port_cfg[portid].active) {
			continue;
		}

		port_cfg  = &dppd_port_cfg[portid];
		rte_eth_link_get_nowait(portid, &link);
		port_cfg->link_up = link.link_status;
		port_cfg->link_speed = link.link_speed;
	}
}

static int tsc_diff_to_tv(uint64_t beg, uint64_t end, struct timeval *tv)
{
	if (end < beg) {
		return -1;
	}

	uint64_t diff = end - beg;
	uint64_t sec_tsc = rte_get_tsc_hz();
	uint64_t sec = diff/sec_tsc;

	tv->tv_sec = sec;
	diff -= sec*sec_tsc;
	tv->tv_usec = diff*1000000/sec_tsc;

	return 0;
}

/* start main loop */
void __attribute__((noreturn)) run(uint32_t flags)
{
	init_display(dppd_cfg.start_time);
	struct screen_state screen_state, old;
	memset(&screen_state, 0, sizeof(screen_state));
	memset(&old, 0, sizeof(old));
	stats_display_layout(screen_state, 0);

	print_rx_tx_info();

	/* start all tasks on worker cores */
	if (flags & TGSF_AUTOSTART) {
		start_core(dppd_cfg.core_mask);
	}

#ifndef BRAS_STATS
	while(1) {sleep(1000000);}
#endif

	uint64_t cur_tsc = rte_rdtsc();
	uint64_t next_update = cur_tsc + rte_get_tsc_hz();
	uint64_t stop_tsc = 0;
	int resize = 0;

	int32_t lsc_local;
	if (dppd_cfg.duration_time != 0) {
		stop_tsc = cur_tsc + dppd_cfg.start_time*rte_get_tsc_hz() + dppd_cfg.duration_time*rte_get_tsc_hz();
	}

	fd_set in_fd, out_fd, err_fd;
	struct timeval tv;
	int stdin_fd = fileno(stdin);
	int max_fd = stdin_fd + 1;

	FD_ZERO(&in_fd);
	FD_ZERO(&out_fd);
	FD_ZERO(&err_fd);

	while (stop_dppd == 0) {
		cur_tsc = rte_rdtsc();

		FD_SET(stdin_fd, &in_fd);

		/* Multiplex keyboard input handling with display. */
		if (tsc_diff_to_tv(cur_tsc, next_update, &tv) == 0) {
			select(max_fd, &in_fd, &out_fd, &err_fd, &tv);
		}

		if (FD_ISSET(stdin_fd, &in_fd)) {
			process_input(&screen_state, &resize);
			if (resize) {
				stats_display_layout(old, 1);
			} else if (old.chosen_screen != screen_state.chosen_screen ||
				   old.chosen_page != screen_state.chosen_page) {
				old = screen_state;
				/* change display state. next call to
				   show stats will display the active
				   screen */
				stats_display_layout(screen_state, 0);
			}
		}
		else {
			next_update += rte_get_tsc_hz()*update_interval/1000;

			update_stats();

			lsc_local = rte_atomic32_read(&lsc);

			if (lsc_local) {
				rte_atomic32_dec(&lsc);
				update_link_states();
				stats_display_layout(old, 1);
			}

			/* 0 is default screen to display, 1 shows
			   port related stats */
			if (screen_state.chosen_screen == 0) {
				display_stats_core_ports(screen_state.chosen_page);
			}
			else if (screen_state.chosen_screen == 1) {
				display_stats_eth_ports();
			}
		}

		if (stop_tsc && cur_tsc >= stop_tsc) {
			stop_dppd = 1;
		}
	}

	mprintf("total RX: %"PRIu64", total TX: %"PRIu64", average RX: %"PRIu64" pps, average TX: %"PRIu64" pps\n",
		global_total_rx(),
		global_total_tx(),
		global_avg_rx(),
		global_avg_tx());

	if (dppd_cfg.flags & TGSF_WAIT_ON_QUIT) {
		stop_core(dppd_cfg.core_mask);
	}

	end_display();
	exit(EXIT_SUCCESS);
}
