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

#ifndef _HANDLE_QOS_H_
#define _HANDLE_QOS_H_

#include <rte_mbuf.h>
#include <rte_ip.h>

#include "stats.h"
#include "qinq.h"
#include "classify.h"
#include "prefetch.h"
#include "defines.h"

struct lcore_cfg;

struct task_qos {
	struct task_base base;
	struct rte_sched_port *sched_port;
	uint16_t *user_table;
	uint8_t  *dscp;
	uint32_t nb_buffered_pkts;
	uint8_t runtime_flags;
};

static inline void handle_qos_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qos *task = (struct task_qos *)tbase;
	if (task->runtime_flags & TASK_CLASSIFY) {
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
			prefetch_nta(mbufs[j + PREFETCH_OFFSET]);
			prefetch_nta(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
			const struct qinq_hdr *pqinq = rte_pktmbuf_mtod(mbufs[j], const struct qinq_hdr *);
			uint32_t qinq = PKT_TO_LUTQINQ(pqinq->svlan.vlan_tci, pqinq->cvlan.vlan_tci);
			const struct ipv4_hdr *ipv4_hdr = (const struct ipv4_hdr *)(pqinq + 1);
			uint8_t queue = task->dscp[ipv4_hdr->type_of_service >> 2] & 0x3;
			uint8_t tc = task->dscp[ipv4_hdr->type_of_service >> 2] >> 2;

			rte_sched_port_pkt_write(mbufs[j], 0, task->user_table[qinq], tc, queue, 0);
		}
#ifdef BRAS_PREFETCH_OFFSET
		prefetch_nta(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
		for (; j < n_pkts; ++j) {
			const struct qinq_hdr *pqinq = rte_pktmbuf_mtod(mbufs[j], const struct qinq_hdr *);
			uint32_t qinq = PKT_TO_LUTQINQ(pqinq->svlan.vlan_tci, pqinq->cvlan.vlan_tci);
			const struct ipv4_hdr *ipv4_hdr = (const struct ipv4_hdr *)(pqinq + 1);
			uint8_t queue = task->dscp[ipv4_hdr->type_of_service >> 2] & 0x3;
			uint8_t tc = task->dscp[ipv4_hdr->type_of_service >> 2] >> 2;

			rte_sched_port_pkt_write(mbufs[j], 0, task->user_table[qinq], tc, queue, 0);
		}
#endif
	}
	int16_t ret = rte_sched_port_enqueue(task->sched_port, mbufs, n_pkts);
	task->nb_buffered_pkts += ret;
	INCR_TX_DROP_COUNT(task->base.stats, n_pkts - ret);
}

void reg_task_qos(void);

#endif /* _HANDLE_QOS_H_ */
