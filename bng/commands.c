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

#include <rte_table_hash.h>

#include "commands.h"
#include "stats.h"
#include "display.h"
#include "tx_worker.h"
#include "dppd_args.h"
#include "hash_utils.h"
#include "dppd_cfg.h"
#include "dppd_port_cfg.h"
#include "defines.h"

void start_core(uint64_t core_mask)
{
	mprintf("Starting worker cores:");

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if ((core_mask & (__UINT64_C(1) << lcore_id)) && (rte_eal_get_lcore_state(lcore_id) != RUNNING)) {
			mprintf(" %u", lcore_id);
		}
	}
	mprintf("\n");
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if ((core_mask & (__UINT64_C(1) << lcore_id)) && (rte_eal_get_lcore_state(lcore_id) != RUNNING)) {
			lcore_cfg[lcore_id].flags &= ~PCFG_TERMINATE;
			rte_eal_remote_launch(dppd_work_thread, &rte_cfg.nb_ports, lcore_id);
		}
	}
}

void stop_core(uint64_t core_mask)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (core_mask & (__UINT64_C(1) << lcore_id)) {
			lcore_cfg[lcore_id].flags |= PCFG_TERMINATE;
		}
	}

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (core_mask & (__UINT64_C(1) << lcore_id)) {
			if ((rte_eal_get_lcore_state(lcore_id) == RUNNING) || (rte_eal_get_lcore_state(lcore_id) == FINISHED)) {
				mprintf("stopping core %u...", lcore_id);
				rte_eal_wait_lcore(lcore_id);
				mprintf(" OK\n");
			}
			else {
				mprintf("core %u in state %d\n", lcore_id, rte_eal_get_lcore_state(lcore_id));
			}
		}
	}
}

void pool_count(void)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		struct lcore_cfg *lconf = &lcore_cfg[lcore_id];

		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_args *targ = &lconf->targs[task_id];
			if (targ->rx_port != NO_PORT_AVAIL && targ->pool) {
				const uint32_t count = rte_mempool_count(targ->pool);
				const uint32_t free_count = rte_mempool_free_count(targ->pool);
				mprintf("\t\tCore %u task %u count=%u, free count=%u\n", lcore_id, task_id, count, free_count);
			}
		}
	}
}

#ifdef DPPD_CMD_STAT_RX
static void stat_rx_get(uint64_t core_mask, uint32_t mode)
{
	mprintf("core_mask = %lx\n", core_mask);
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (core_mask & (__UINT64_C(1) << lcore_id)) {
			struct lcore_cfg *lconf = &lcore_cfg[lcore_id];
			mprintf("core %u(%32s):", lcore_id, lconf->name);
			for (uint16_t k = 0; k < MAX_RING_BURST + 1; ++k) {
				mprintf(" %lu", rte_atomic64_read(&lconf->nb_rx_buckets[k]));
			}
			mprintf("\n");
			if (mode) {
				for (uint16_t k = 0; k < MAX_RING_BURST + 1; ++k) {
					rte_atomic64_set(&lconf->nb_rx_buckets[k], 0);
				}
			}
		}
	}
}
#endif

static void stat_port_get(int port)
{
	struct rte_eth_stats stats;
	if ((dppd_cfg.flags & TGSF_USE_VF) == 0) {
		for (uint8_t i = 0; i < 16; ++i) {
			int ret;
			ret = rte_eth_dev_set_rx_queue_stats_mapping(port, i, i);
			if (ret) {
				mprintf("rte_eth_dev_set_rx_queue_stats_mapping() failed: error %d\n", ret);
			}
			ret = rte_eth_dev_set_tx_queue_stats_mapping(port, i, i);
			if (ret) {
				mprintf("rte_eth_dev_set_tx_queue_stats_mapping() failed: error %d\n", ret);
			}
		}
	}
	rte_eth_stats_get(port, &stats);
	mprintf("port %u: i=%lu, o=%lu L2i=%lu L2o=%lu no_buf=%lu\n",
		port, stats.ipackets, stats.opackets, stats.ilbpackets, stats.olbpackets, stats.rx_nombuf);
	if ((dppd_cfg.flags & TGSF_USE_VF) == 0) {
		for (uint8_t i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS; ++i) {
			mprintf("q %u: i=%lu, o=%lu e=%lu\n", i, stats.q_ipackets[i], stats.q_opackets[i], stats.q_errors[i]);
		}
	}
}

void cmd_dump(uint8_t lcore_id, uint8_t task_id, uint32_t nb_packets)
{
	mprintf("dump %u %u %u\n", lcore_id, task_id, nb_packets);
#ifdef DPPD_CMD_DUMP
	if (lcore_id > RTE_MAX_LCORE) {
		mprintf("core_id to high, maximum allowed is: %u\n", RTE_MAX_LCORE);
	}
	else if (task_id >= lcore_cfg[lcore_id].nb_tasks) {
		mprintf("task_id to high, should be in [0, %u]\n", lcore_cfg[lcore_id].nb_tasks - 1);
	}
	else {
		rte_atomic32_set(&lcore_cfg[lcore_id].task[task_id]->task_debug.nb_print, nb_packets);
	}
#else
	mprintf("dump disabled at compile time\n");
#endif
}

void cmd_stat(const char *mode, uint32_t id)
{
	if (strcmp(mode, "port") == 0) {
		if (!dppd_port_cfg[id].active) {
			mprintf("Invalid port id %u\n", id);
		}
		else {
			stat_port_get(id);
		}
	}
#ifdef DPPD_CMD_STAT_RX
	else if (strcmp(mode, "rx") == 0) {
		stat_rx_get(dppd_cfg.core_mask, id);
	}
#endif
	else if (strcmp(mode, "all_ports") == 0) {
		for (uint8_t port = 0; port < DPPD_MAX_PORTS; ++port) {
			if (dppd_port_cfg[id].active) {
				stat_port_get(port);
			}
		}
	}
	else if (strcmp(mode, "size") == 0) {
		mprintf("size = %lu\n", sizeof(struct lcore_cfg));
	}
}

void cmd_ringinfo_all(void)
{
	struct lcore_cfg *lconf;
	uint32_t lcore_id = -1;

	while(dppd_core_next(&lcore_id) == 0) {
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			cmd_ringinfo(lcore_id, task_id);
		}
	}
}

void cmd_ringinfo(uint8_t lcore_id, uint8_t task_id)
{
	struct lcore_cfg *lconf;
	struct rte_ring *ring;
	struct task_args* targ;
	uint32_t count;

	if (!dppd_core_active(lcore_id)) {
		mprintf("lcore %u is not active\n", lcore_id);
		return;
	}
	lconf = &lcore_cfg[lcore_id];
	if (task_id >= lconf->nb_tasks) {
		mprintf("Invalid task index %u: lcore %u has %u tasks\n", task_id, lcore_id, lconf->nb_tasks);
		return;
	}

	targ = &lconf->targs[task_id];
	mprintf("Core %u task %u: %u rings\n", lcore_id, task_id, targ->nb_rxrings);
	for (uint8_t i = 0; i < targ->nb_rxrings; ++i) {
		ring = targ->rx_rings[i];
		count = ring->prod.mask + 1;
		mprintf("\tRing %u:\n", i);
		mprintf("\t\tFlags: %s,%s\n", ring->flags & RING_F_SP_ENQ? "sp":"mp", ring->flags & RING_F_SC_DEQ? "sc":"mc");
		mprintf("\t\tMemory size: %zu bytes\n", rte_ring_get_memsize(count));
		mprintf("\t\tOccupied: %u/%u\n", rte_ring_count(ring), count);
	}
}

static int port_is_valid(uint8_t port_id)
{
	if (port_id > DPPD_MAX_PORTS) {
		mprintf("requested port is higher than highest supported port ID (%u)\n", DPPD_MAX_PORTS);
		return 0;
	}

	struct dppd_port_cfg* port_cfg = &dppd_port_cfg[port_id];
	if (!port_cfg->active) {
		mprintf("Port %u is not active\n", port_id);
		return 0;
	}
	return 1;
}

void cmd_port_up(uint8_t port_id)
{
	int err;

	if (!port_is_valid(port_id)) {
		return ;
	}

	if ((err = rte_eth_dev_set_link_up(port_id)) == 0) {
		mprintf("Bringing port %d up\n", port_id);
	}
	else {
		mprintf("Failed to bring port %d up with error %d\n", port_id, err);
	}
}

void cmd_port_down(uint8_t port_id)
{
	int err;

	if (!port_is_valid(port_id)) {
		return ;
	}

	if ((err = rte_eth_dev_set_link_up(port_id)) == 0) {
		mprintf("Bringing port %d down\n", port_id);
	}
	else {
		mprintf("Failed to bring port %d down with error %d\n", port_id, err);
	}
}

void cmd_portinfo(uint8_t port_id)
{
	if (!port_is_valid(port_id)) {
		return ;
	}
	struct dppd_port_cfg* port_cfg = &dppd_port_cfg[port_id];

	mprintf("Port info for port %u\n", port_id);
	mprintf("\tName: %s\n", port_cfg->name);
	mprintf("\tDriver: %s\n", port_cfg->driver_name);
	mprintf("\tMac address: "MAC_BYTES_FMT"\n", MAC_BYTES(port_cfg->eth_addr.addr_bytes));
	mprintf("\tLink speed: %u Mbps\n", port_cfg->link_speed);
	mprintf("\tLink status: %s\n", port_cfg->link_up? "up" : "down");
	mprintf("\tSocket: %u\n", port_cfg->socket);
	mprintf("\tPromiscuous: %s\n", port_cfg->promiscuous? "yes" : "no");
	mprintf("\tNumber of RX/TX descriptors: %u/%u\n", port_cfg->n_rxd, port_cfg->n_txd);
	mprintf("\tNumber of RX/TX queues: %u/%u (max: %u/%u)\n", port_cfg->n_rxq, port_cfg->n_txq, port_cfg->max_rxq, port_cfg->max_txq);
	mprintf("\tMemory pools:\n");
	for (uint8_t i = 0; i < 32; ++i) {
		if (port_cfg->pool[i]) {
			mprintf("\t\tname: %s (%p)\n", port_cfg->pool[i]->name, port_cfg->pool[i]);
		}
	}
}
