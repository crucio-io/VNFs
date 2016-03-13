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

#include <rte_cycles.h>
#include <rte_table_hash.h>

#include "handle_qinq_decap6.h"
#include "handle_qinq_encap6.h"
#include "display.h"
#include "classify.h"
#include "stats.h"
#include "tx_pkt.h"
#include "defines.h"
#include "pkt_prototypes.h"
#include "dppd_assert.h"
#include "hash_utils.h"
#include "task_base.h"
#include "thread_generic.h"
#include "prefetch.h"
#include "hash_entry_types.h"

/* Packets must all be IPv6, always store QinQ tags for lookup (not configurable) */
struct task_qinq_decap6 {
	struct task_base                base;
	void (*tx_pkt)(struct task_base *tbase);
	struct rte_table_hash           *cpe_table;
	uint16_t                        *user_table;
	uint32_t                        bucket_index;
	struct ether_addr 		edaddr;
	void*                           period_data; /* used if using dual stack*/
	void (*period_func)(void* data);
	uint64_t                        cpe_timeout;
};

void handle_qinq_decap6_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

void update_arp_entries6(void* data);

static void init_task_qinq_decap6(struct task_base *tbase, struct task_args *targ)
{
	struct task_qinq_decap6 *task = (struct task_qinq_decap6 *)tbase;
	task->edaddr = targ->edaddr;
	task->cpe_table = targ->cpe_table;
	task->user_table = targ->dppd_shared->user_table;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);
	task->cpe_timeout = rte_get_tsc_hz()/1000*targ->cpe_table_timeout_ms;

	if (targ->cpe_table_timeout_ms) {
		if (targ->lconf->period_func) {
			task->period_func = targ->lconf->period_func;
			task->period_data = targ->lconf->period_data;
		}
		targ->lconf->period_func = update_arp_entries6;
		targ->lconf->period_data = tbase;
		targ->lconf->period_timeout = (rte_get_tsc_hz() >> 1) / NUM_VCPES;
	}
}

static void early_init(struct task_args *targ)
{
	if (!targ->cpe_table) {
		init_cpe6_table(targ);
	}
}

struct task_init task_init_qinq_decap6 = {
	.mode = QINQ_DECAP6,
	.mode_str = "qinqdecapv6",
	.early_init = early_init,
	.init = init_task_qinq_decap6,
	.handle = handle_qinq_decap6_bulk,
	.thread_x = thread_generic,
	.flag_req_data = REQ_USER_TABLE | REQ_LPM6,
	.size = sizeof(struct task_qinq_decap6)
};

void reg_task_qinq_decap6(void)
{
	reg_task(&task_init_qinq_decap6);
}

static inline void handle_qinq_decap6(struct task_qinq_decap6 *task, struct rte_mbuf *mbuf);

void handle_qinq_decap6_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_decap6 *task = (struct task_qinq_decap6 *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_qinq_decap6(task, mbufs[j]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_qinq_decap6(task, mbufs[j]);
	}
#endif
	task->tx_pkt(&task->base);
}

static inline void handle_qinq_decap6(struct task_qinq_decap6 *task, struct rte_mbuf *mbuf)
{
	struct qinq_hdr *pqinq = rte_pktmbuf_mtod(mbuf, struct qinq_hdr *);
	struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(pqinq + 1);

	uint16_t svlan = pqinq->svlan.vlan_tci & 0xFF0F;
	uint16_t cvlan = pqinq->cvlan.vlan_tci & 0xFF0F;

	struct cpe_data entry;
	entry.mac_port_8bytes = *((uint64_t *)(((uint8_t *)pqinq) + 5)) << 16;
	entry.qinq_svlan = svlan;
	entry.qinq_cvlan = cvlan;
	entry.user = task->user_table[PKT_TO_LUTQINQ(svlan, cvlan)];
	entry.tsc = rte_rdtsc() + task->cpe_timeout;

	int key_found = 0;
	void* entry_in_hash = NULL;
	int ret = rte_table_hash_ext_dosig_ops.
		f_add(task->cpe_table, pip6->src_addr, &entry, &key_found, &entry_in_hash);

	if (unlikely(ret)) {
		mprintf_once(1, "Failed to add key " IPv6_BYTES_FMT "\n", IPv6_BYTES(pip6->src_addr));
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return;
	}

	pqinq = (struct qinq_hdr *)rte_pktmbuf_adj(mbuf, 2 * sizeof(struct vlan_hdr));
	DPPD_ASSERT(pqinq);
	pqinq->ether_type = ETYPE_IPv6;
	// Dest MAC addresses
	ether_addr_copy(&task->edaddr, &pqinq->d_addr);
	tx_buf_pkt_single(&task->base, mbuf, 0);
}

void update_arp_entries6(void* data)
{
	uint64_t cur_tsc = rte_rdtsc();
	struct task_qinq_decap6 *task = (struct task_qinq_decap6 *)data;

	struct cpe_data *entries[4] = {0};
	void *key[4] = {0};
	uint64_t n_buckets = get_bucket(task->cpe_table, task->bucket_index, key, (void**)entries);

	for (uint8_t i = 0; i < 4 && entries[i]; ++i) {
		if (entries[i]->tsc < cur_tsc) {
			int key_found = 0;
			void* entry = 0;
			rte_table_hash_ext_dosig_ops.f_delete(task->cpe_table, key[i], &key_found, entry);
		}
	}

	task->bucket_index++;
	task->bucket_index &= (n_buckets - 1);

	if (task->period_func) {
		task->period_func(task->period_data);
	}
}
