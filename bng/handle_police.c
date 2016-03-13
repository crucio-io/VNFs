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

#include <string.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_cycles.h>
#include <rte_version.h>

#include "handle_police.h"
#include "task_base.h"
#include "task_init.h"
#include "thread_basic.h"
#include "display.h"
#include "lconf.h"
#include "stats.h"
#include "prefetch.h"
#include "quit.h"
#include "defines.h"
#include "qinq.h"

struct task_police {
	struct task_base base;
	union {
		struct rte_meter_srtcm *sr_flows;
		struct rte_meter_trtcm *tr_flows;
	};

	uint16_t           *user_table;
	enum police_action police_act[3][3];
	uint16_t overhead;
	uint8_t runtime_flags;
};

typedef uint8_t (*hp) (struct task_police *task, struct rte_mbuf *mbuf, uint64_t tsc, uint32_t user);

static uint8_t handle_police(struct task_police *task, struct rte_mbuf *mbuf, uint64_t tsc, uint32_t user)
{
	enum rte_meter_color in_color = e_RTE_METER_GREEN;
	enum rte_meter_color out_color;
	uint32_t pkt_len = rte_pktmbuf_pkt_len(mbuf) + task->overhead;
	out_color = rte_meter_srtcm_color_aware_check(&task->sr_flows[user], tsc, pkt_len, in_color);

	if (task->police_act[in_color][out_color] == ACT_DROP) {
		rte_pktmbuf_free(mbuf);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		return 0;
	}
	return 1;
}

static uint8_t handle_police_tr(struct task_police *task, struct rte_mbuf *mbuf, uint64_t tsc, uint32_t user)
{
	enum rte_meter_color in_color = e_RTE_METER_GREEN;
	enum rte_meter_color out_color;
	uint32_t pkt_len = rte_pktmbuf_pkt_len(mbuf) + task->overhead;
	out_color = rte_meter_trtcm_color_aware_check(&task->tr_flows[user], tsc, pkt_len, in_color);

	if (task->police_act[in_color][out_color] == ACT_DROP) {
		rte_pktmbuf_free(mbuf);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		return 0;
	}

	if (task->runtime_flags  & TASK_MARK) {
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
		struct rte_sched_port_hierarchy *sched =
			(struct rte_sched_port_hierarchy *) &mbuf->hash.sched;
#else
		struct rte_sched_port_hierarchy *sched =
			(struct rte_sched_port_hierarchy *) &mbuf->pkt.hash.sched;
#endif
		sched->color = task->police_act[in_color][out_color];
	}
	return 1;
}

static inline int get_user(struct task_police *task, struct rte_mbuf *mbuf)
{
	if (task->runtime_flags & TASK_CLASSIFY) {
		struct qinq_hdr *pqinq = rte_pktmbuf_mtod(mbuf, struct qinq_hdr *);
		return PKT_TO_LUTQINQ(pqinq->svlan.vlan_tci, pqinq->cvlan.vlan_tci);
	}

#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	struct rte_sched_port_hierarchy *sched =
		(struct rte_sched_port_hierarchy *) &mbuf->hash.sched;
#else
	struct rte_sched_port_hierarchy *sched =
		(struct rte_sched_port_hierarchy *) &mbuf->pkt.hash.sched;
#endif

	return sched->pipe;
}

#define PHASE1_DELAY PREFETCH_OFFSET
#define PHASE2_DELAY PREFETCH_OFFSET
#define PHASE3_DELAY PREFETCH_OFFSET
#define PHASE4_DELAY PREFETCH_OFFSET

static inline void handle_pb(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts, hp handle_police_func)
{
	struct task_police *task = (struct task_police *)tbase;
	uint16_t n_kept = 0;
	uint16_t j;
	uint64_t cur_tsc = rte_rdtsc();
	uint32_t user[64];
	uint32_t cur_user;
	for (j = 0; j < PHASE1_DELAY && j < n_pkts; ++j) {
		PREFETCH0(mbufs[j]);
	}

	for (j = 0; j < PHASE2_DELAY && j + PHASE1_DELAY < n_pkts; ++j) {
		PREFETCH0(mbufs[j + PHASE1_DELAY]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j], void*));
	}

	for (j = 0; j < PHASE3_DELAY && j + PHASE2_DELAY + PHASE1_DELAY < n_pkts; ++j) {
		PREFETCH0(mbufs[j + PHASE2_DELAY + PHASE1_DELAY]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PHASE2_DELAY], void*));
                cur_user = get_user(task, mbufs[j]);
		user[j] = cur_user;
		PREFETCH0(&task->user_table[cur_user]);
	}

	/* At this point, the whole pipeline is running */
	for (j = 0; j + PHASE3_DELAY + PHASE2_DELAY + PHASE1_DELAY < n_pkts; ++j) {
		PREFETCH0(mbufs[j + PHASE3_DELAY + PHASE2_DELAY + PHASE1_DELAY]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PHASE3_DELAY + PHASE2_DELAY], void*));
		cur_user = get_user(task, mbufs[j + PHASE3_DELAY]);
		user[j + PHASE3_DELAY] = cur_user;
		PREFETCH0(&task->user_table[cur_user]);

		mbufs[n_kept] = mbufs[j];
		n_kept += handle_police_func(task, mbufs[j], cur_tsc, task->user_table[user[j]]);
	}

	/* Last part of pipeline */
	for (; j + PHASE3_DELAY + PHASE2_DELAY < n_pkts; ++j) {
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PHASE3_DELAY + PHASE2_DELAY], void*));
		PREFETCH0(&task->user_table[j + PHASE3_DELAY]);
		cur_user = get_user(task, mbufs[j + PHASE3_DELAY]);
		user[j + PHASE3_DELAY] = cur_user;
		PREFETCH0(&task->user_table[cur_user]);

		mbufs[n_kept] = mbufs[j];
		n_kept += handle_police_func(task, mbufs[j], cur_tsc, task->user_table[user[j]]);
	}

	for (; j + PHASE3_DELAY < n_pkts; ++j) {
		cur_user = get_user(task, mbufs[j + PHASE3_DELAY]);
		user[j + PHASE3_DELAY] = cur_user;
		PREFETCH0(&task->user_table[cur_user]);

		mbufs[n_kept] = mbufs[j];
		n_kept += handle_police_func(task, mbufs[j], cur_tsc, task->user_table[user[j]]);
	}

	for (; j < n_pkts; ++j) {
		mbufs[n_kept] = mbufs[j];
		n_kept += handle_police_func(task, mbufs[j], cur_tsc, task->user_table[user[j]]);
	}

	if (n_kept) {
		task->base.tx_pkt_no_buf(&task->base, mbufs, n_kept);
	}
}

static void handle_police_bulk(struct task_base *tbase, struct rte_mbuf **mbuf, uint16_t n_pkts)
{
        handle_pb(tbase, mbuf, n_pkts, handle_police);
}

static void handle_police_tr_bulk(struct task_base *tbase, struct rte_mbuf **mbuf, uint16_t n_pkts)
{
        handle_pb(tbase, mbuf, n_pkts, handle_police_tr);
}

static void init_task_police(struct task_base *tbase, struct task_args *targ)
{
	struct task_police *task = (struct task_police *)tbase;
	task->overhead = targ->overhead;
	task->user_table = targ->dppd_shared->user_table;
	task->runtime_flags = targ->runtime_flags;

	if (strcmp(targ->task_init->sub_mode_str, "trtcm")) {
		task->sr_flows = rte_zmalloc_socket(NULL, targ->n_users * sizeof(struct rte_meter_srtcm),
						 CACHE_LINE_SIZE, rte_lcore_to_socket_id(targ->lconf->id));
		DPPD_PANIC(task->sr_flows == NULL, "Failed to allocate flow contexts\n");
		DPPD_PANIC(!targ->cir, "Commited information rate is set to 0\n");
		DPPD_PANIC(!targ->cbs, "Commited information bucket size is set to 0\n");
		DPPD_PANIC(!targ->ebs, "Execess information bucket size is set to 0\n");

		struct rte_meter_srtcm_params params = {
			.cir = targ->cir,
			.cbs = targ->cbs,
			.ebs = targ->ebs,
		};

		for (uint32_t i = 0; i < targ->n_users; ++i) {
			rte_meter_srtcm_config(&task->sr_flows[i], &params);
		}
	}
	else {
		task->tr_flows = rte_zmalloc_socket(NULL, targ->n_users * sizeof(struct rte_meter_trtcm),
						    CACHE_LINE_SIZE, rte_lcore_to_socket_id(targ->lconf->id));
		DPPD_PANIC(task->tr_flows == NULL, "Failed to allocate flow contexts\n");
		DPPD_PANIC(!targ->pir, "Peak information rate is set to 0\n");
		DPPD_PANIC(!targ->cir, "Commited information rate is set to 0\n");
		DPPD_PANIC(!targ->pbs, "Peak information bucket size is set to 0\n");
		DPPD_PANIC(!targ->cbs, "Commited information bucket size is set to 0\n");

		struct rte_meter_trtcm_params params = {
			.pir = targ->pir,
			.pbs = targ->pbs,
			.cir = targ->cir,
			.cbs = targ->cbs,
		};

		for (uint32_t i = 0; i < targ->n_users; ++i) {
			rte_meter_trtcm_config(&task->tr_flows[i], &params);
		}
	}

	for (uint32_t i = 0; i < 3; ++i) {
		for (uint32_t j = 0; j < 3; ++j) {
			task->police_act[i][j] = targ->police_act[i][j];
		}
	}


	tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
	tbase->flags |= FLAG_NEVER_FLUSH;
}

struct task_init task_init_police = {
	.mode_str = "police",
	.init = init_task_police,
	.handle = handle_police_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_USER_TABLE,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_police)
};

struct task_init task_init_police2 = {
	.mode_str = "police",
	.sub_mode_str = "trtcm",
	.init = init_task_police,
	.handle = handle_police_tr_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_USER_TABLE,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_police)
};

void reg_task_police(void)
{
	reg_task(&task_init_police);
	reg_task(&task_init_police2);
}
