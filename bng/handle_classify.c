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
#include <rte_ip.h>

#include "handle_classify.h"
#include "task_base.h"
#include "task_init.h"
#include "defines.h"
#include "prefetch.h"
#include "qinq.h"
#include "classify.h"
#include "thread_basic.h"
#include "display.h"

struct task_classify {
	struct task_base    base;
	uint16_t           *user_table;
	uint8_t             *dscp;
};

static inline void handle_classify(struct task_classify *task, struct rte_mbuf *mbuf)
{
	const struct qinq_hdr *pqinq = rte_pktmbuf_mtod(mbuf, const struct qinq_hdr *);

	uint32_t qinq = PKT_TO_LUTQINQ(pqinq->svlan.vlan_tci, pqinq->cvlan.vlan_tci);

	/* Traffic class can be set by ACL task. If this is the case,
	   don't overwrite it using dscp. Instead, use the
	   traffic class that had been set. */

#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	struct rte_sched_port_hierarchy * sched32 = (struct rte_sched_port_hierarchy *)&mbuf->hash.sched;
	uint8_t prev_tc = sched32->traffic_class;
#else
	struct rte_sched_port_hierarchy *sched = (struct rte_sched_port_hierarchy *) &mbuf->pkt.hash.sched;
	uint8_t prev_tc = sched->traffic_class;
#endif

	const struct ipv4_hdr *ipv4_hdr = (const struct ipv4_hdr *)(pqinq + 1);
	uint8_t dscp = task->dscp[ipv4_hdr->type_of_service >> 2];

	uint8_t queue = dscp & 0x3;
	uint8_t tc = prev_tc? prev_tc : dscp >> 2;

	rte_sched_port_pkt_write(mbuf, 0, task->user_table[qinq], tc, queue, 0);
}

static void handle_classify_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_classify *task = (struct task_classify *)tbase;

	uint16_t j;
#ifdef BRAS_PREFETCH_OFFSET
	for (j = 0; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
		prefetch_nta(mbufs[j]);
	}
	for (j = 1; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
		prefetch_nta(rte_pktmbuf_mtod(mbufs[j - 1], void *));
	}
#endif
	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		prefetch_nta(mbufs[j + PREFETCH_OFFSET]);
		prefetch_nta(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_classify(task, mbufs[j]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	prefetch_nta(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_classify(task, mbufs[j]);
	}
#endif

	task->base.tx_pkt_no_buf(&task->base, mbufs, n_pkts);
}

static void init_task_classify(struct task_base *tbase, struct task_args *targ)
{
	tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
	tbase->flags |= FLAG_NEVER_FLUSH;
	struct task_classify *task_classify = (struct task_classify *)tbase;
	task_classify->user_table = targ->dppd_shared->user_table;
	task_classify->dscp = targ->dppd_shared->dscp;
}


struct task_init task_init_classify = {
	.mode_str = "classify",
	.init = init_task_classify,
	.handle = handle_classify_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_USER_TABLE | REQ_DSCP,
	.size = sizeof(struct task_classify)
};

void reg_task_classify(void)
{
	reg_task(&task_init_classify);
}
