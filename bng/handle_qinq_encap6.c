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

#include "handle_qinq_encap6.h"
#include "task_base.h"
#include "display.h"
#include "qinq.h"
#include "defines.h"
#include "classify.h"
#include "tx_pkt.h"
#include "hash_entry_types.h"
#include "prefetch.h"
#include "stats.h"
#include "mpls.h"
#include "thread_generic.h"
#include "hash_utils.h"
#include "quit.h"
#include "dppd_cfg.h"

struct task_qinq_encap6 {
	struct task_base                    base;
	void (*tx_pkt)(struct task_base *tbase);
	uint8_t				    tx_portid;
	uint8_t                             runtime_flags;
	struct rte_table_hash               *cpe_table;
};

/* Encapsulate IPv6 packet in QinQ where the QinQ is derived from the IPv6 address */
void handle_qinq_encap6_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
void handle_qinq_encap6_untag_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

static void init_task_qinq_encap6(struct task_base *tbase, struct task_args *targ)
{
	struct task_qinq_encap6 *task = (struct task_qinq_encap6 *)tbase;
	task->cpe_table = targ->cpe_table;
	task->runtime_flags = targ->runtime_flags;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);
}

void init_cpe6_table(struct task_args *targ)
{
	char name[64];
	sprintf(name, "core_%u_CPEv6Table", targ->lconf->id);

	uint8_t table_part = targ->nb_slave_threads;
	if (!rte_is_power_of_2(table_part)) {
		table_part = rte_align32pow2(table_part) >> 1;
	}

	uint32_t n_entries = MAX_GRE / table_part;
	struct rte_table_hash_ext_params table_hash_params = {
		.key_size = sizeof(struct ipv6_addr),
		.n_keys = n_entries,
		.n_buckets = n_entries >> 2,
		.n_buckets_ext = n_entries >> 3,
		.f_hash = hash_crc32,
		.seed = 0,
		.signature_offset = 0,
		.key_offset = 0,
	};

	size_t entry_size = sizeof(struct cpe_data);
	if (!rte_is_power_of_2(entry_size)) {
		entry_size = rte_align32pow2(entry_size);
	}

	struct rte_table_hash* phash = rte_table_hash_ext_dosig_ops.
		f_create(&table_hash_params, rte_lcore_to_socket_id(targ->lconf->id), entry_size);
	DPPD_PANIC(phash == NULL, "Unable to allocate memory for IPv6 hash table on core %u\n", targ->lconf->id);

	for (uint8_t task_id = 0; task_id < targ->lconf->nb_tasks; ++task_id) {
		enum task_mode smode = targ->lconf->targs[task_id].mode;
		if (smode == QINQ_DECAP6 || smode == QINQ_ENCAP6) {
			targ->lconf->targs[task_id].cpe_table = phash;
		}
	}
}

static void early_init(struct task_args *targ)
{
	if (!targ->cpe_table) {
		init_cpe6_table(targ);
	}
}

struct task_init task_init_qinq_encap6 = {
	.mode = QINQ_ENCAP6,
	.mode_str = "qinqencapv6",
	.init = init_task_qinq_encap6,
	.early_init = early_init,
	.handle = handle_qinq_encap6_bulk,
	.thread_x = thread_generic,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_qinq_encap6)
};

struct task_init task_init_qinq_encap6_untag = {
	.mode = QINQ_ENCAP6,
	.mode_str = "qinqencapv6",
	.sub_mode_str = "unmpls",
	.early_init = early_init,
	.init = init_task_qinq_encap6,
	.handle = handle_qinq_encap6_untag_bulk,
	.thread_x = thread_generic,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_qinq_encap6)
};

void reg_task_qinq_encap6(void)
{
	reg_task(&task_init_qinq_encap6);
	reg_task(&task_init_qinq_encap6_untag);
}

static inline void handle_qinq_encap6(struct rte_mbuf *mbuf, struct task_qinq_encap6 *task);

void handle_qinq_encap6_untag_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_encap6 *task = (struct task_qinq_encap6 *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		if (likely(mpls_untag(mbufs[j]))) {
			handle_qinq_encap6(mbufs[j], task);
		}
		else {
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbufs[j]);
		}
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		if (likely(mpls_untag(mbufs[j]))) {
			handle_qinq_encap6(mbufs[j], task);
		}
		else {
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbufs[j]);
		}
	}
#endif
	task->tx_pkt(&task->base);
}

void handle_qinq_encap6_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_encap6 *task = (struct task_qinq_encap6 *)tbase;
	uint16_t j;

        prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_qinq_encap6(mbufs[j], task);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_qinq_encap6(mbufs[j], task);
	}
#endif
	task->tx_pkt(&task->base);
}

static inline void handle_qinq_encap6(struct rte_mbuf *mbuf, struct task_qinq_encap6 *task)
{
	struct qinq_hdr *pqinq = (struct qinq_hdr *)rte_pktmbuf_prepend(mbuf, 2 * sizeof(struct vlan_hdr));

	DPPD_ASSERT(pqinq);
	struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(pqinq + 1);

	if (pip6->hop_limits) {
		pip6->hop_limits--;
	}
	else {
		mprintf("TTL = 0 => Dropping\n");
		rte_pktmbuf_free(mbuf);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		return;
	}

	// TODO: optimize to use bulk as intended with the rte_table_library
	uint64_t pkts_mask = RTE_LEN2MASK(1, uint64_t);
	uint64_t lookup_hit_mask;
	struct cpe_data* entries[64]; // TODO: use bulk size
	rte_table_hash_ext_dosig_ops.f_lookup(task->cpe_table, &mbuf, pkts_mask, &lookup_hit_mask, (void**)entries);

	if (lookup_hit_mask == 0x1) {
		/* will also overwrite part of the destination addr */
		(*(uint64_t *)pqinq) = entries[0]->mac_port_8bytes;
		pqinq->svlan.eth_proto = dppd_cfg.QinQ_tag;
		pqinq->cvlan.eth_proto = ETYPE_VLAN;
		pqinq->svlan.vlan_tci = entries[0]->qinq_svlan;
		pqinq->cvlan.vlan_tci = entries[0]->qinq_cvlan;
		pqinq->ether_type = ETYPE_IPv6;

		/* classification can only be done from this point */
		if (task->runtime_flags & TASK_CLASSIFY) {
			classify_packet(mbuf, entries[0]->user);
		}

		tx_buf_pkt_single(&task->base, mbuf, 0);
	}
	else {
		mprintf_verbose("core Unknown IP " IPv6_BYTES_FMT "\n", IPv6_BYTES(pip6->dst_addr));
		rte_pktmbuf_free(mbuf);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
	}
}
