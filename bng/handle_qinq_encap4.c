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
#include <rte_malloc.h>
#include <rte_cycles.h>

#include "handle_qinq_encap4.h"
#include "handle_qinq_decap4.h"
#include "defines.h"
#include "classify.h"
#include "stats.h"
#include "tx_pkt.h"
#include "prefetch.h"
#include "pkt_prototypes.h"
#include "hash_entry_types.h"
#include "task_init.h"
#include "bng_pkts.h"
#include "thread_generic.h"
#include "dppd_cksum.h"
#include "hash_utils.h"
#include "quit.h"
#include "dppd_cfg.h"

struct task_qinq_encap4 {
	struct task_base base;
	void (*tx_pkt)(struct task_base *tbase);
	struct rte_table_hash  *cpe_table;
	uint8_t          runtime_flags;
	uint8_t          *dscp;
	uint64_t         keys[64];
	struct rte_mbuf* fake_packets[64];
	uint64_t         cpe_timeout;
};

/* Encapsulate IPv4 packets in QinQ. QinQ tags are derived from gre_id. */
void handle_qinq_encap4_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
void handle_qinq_encap4_bulk_pe(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
static void arp_msg(struct task_base *tbase, void **data, uint16_t n_msgs);

/* Same functionality as handle_qinq_encap_v4_bulk but untag MPLS as well. */
void handle_qinq_encap4_untag_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

static void init_task_qinq_encap4(struct task_base *tbase, struct task_args *targ)
{
	struct task_qinq_encap4 *task = (struct task_qinq_encap4 *)(tbase);
	task->cpe_table = targ->cpe_table;
	task->cpe_timeout = rte_get_tsc_hz()/1000*targ->cpe_table_timeout_ms;

	if (!strcmp(targ->task_init->sub_mode_str, "pe")) {
		for (uint32_t i = 0; i < targ->dppd_shared->cpe_table_entries_count; ++i) {
			if (rte_bswap32(targ->dppd_shared->cpe_table_entries[i].ip) % targ->nb_slave_threads == targ->worker_thread_id) {
				struct cpe_table_entry *entry = &targ->dppd_shared->cpe_table_entries[i];

				struct cpe_key key = {
					.ip = entry->ip,
					.gre_id = entry->gre_id,
				};

				DPPD_PANIC(targ->mapping[entry->port_idx] == 255, "Error reading cpe table: Mapping for port %d is missing", entry->port_idx);
				struct cpe_data data = {
					.qinq_svlan = entry->svlan,
					.qinq_cvlan = entry->cvlan,
					.user = entry->user,
					.mac_port = {
						.mac = entry->eth_addr,
						.out_idx = targ->mapping[entry->port_idx],
					},
					.tsc = UINT64_MAX,
				};

				int key_found;
				void* entry_in_hash;
				rte_table_hash_key8_ext_dosig_ops.f_add(task->cpe_table, &key, &data, &key_found, &entry_in_hash);
			}
		}
	}

	task->dscp = targ->dppd_shared->dscp;
	task->runtime_flags = targ->runtime_flags;
	task->tx_pkt = tx_function(targ);

	tbase->flush_queues = flush_function(targ);

	for (uint32_t i = 0; i < 64; ++i) {
			task->fake_packets[i] = (struct rte_mbuf*)((uint8_t*)&task->keys[i] - sizeof (struct rte_mbuf));
	}

	targ->lconf->ctrl_timeout = rte_get_tsc_hz()/1000;
	targ->lconf->ctrl_func_m[targ->task] = arp_msg;
}

static void arp_msg(struct task_base *tbase, void **data, uint16_t n_msgs)
{
	struct task_qinq_encap4 *task = (struct task_qinq_encap4 *)tbase;
	struct arp_msg **msgs = (struct arp_msg **)data;

	arp_update_from_msg(task->cpe_table, msgs, n_msgs, task->cpe_timeout);
}

void init_qinq_gre_table(struct task_args *targ, struct dppd_shared* dppd_shared)
{
	struct rte_table_hash* qinq_gre_table;
	uint8_t table_part = targ->nb_slave_threads;
	if (!rte_is_power_of_2(table_part)) {
		table_part = rte_align32pow2(table_part) >> 1;
	}

	uint32_t n_entries = MAX_GRE / table_part;

	struct rte_table_hash_key8_ext_params table_hash_params = {
		.n_entries = n_entries,
		.n_entries_ext = n_entries >> 1,
		.f_hash = hash_crc32,
		.seed = 0,
		.signature_offset = 8,
		.key_offset = 0,
	};


	qinq_gre_table = rte_table_hash_key8_ext_dosig_ops.
		f_create(&table_hash_params, rte_lcore_to_socket_id(targ->lconf->id), sizeof(struct qinq_gre_data));
	for (uint32_t i = 0; i < dppd_shared->qinq_to_gre_lookup_count; ++i) {
		/* Only store QinQ <-> GRE mapping for packets that are handled by this worker thread */
		if (dppd_shared->qinq_to_gre_lookup[i].gre_id % targ->nb_slave_threads == targ->worker_thread_id) {
			struct vlans qinq2 = {
				.svlan = {.eth_proto = dppd_cfg.QinQ_tag, .vlan_tci = dppd_shared->qinq_to_gre_lookup[i].svlan},
				.cvlan = {.eth_proto = ETYPE_VLAN,   .vlan_tci = dppd_shared->qinq_to_gre_lookup[i].cvlan}
			};
			struct qinq_gre_data entry = {
				.gre_id = dppd_shared->qinq_to_gre_lookup[i].gre_id,
				.user = dppd_shared->qinq_to_gre_lookup[i].user,
			};

			int key_found = 0;
			void* entry_in_hash = NULL;
			rte_table_hash_key8_ext_dosig_ops.f_add(qinq_gre_table, &qinq2, &entry, &key_found, &entry_in_hash);
		}
	}

	for (uint8_t task_id = 0; task_id < targ->lconf->nb_tasks; ++task_id) {
		enum task_mode smode = targ->lconf->targs[task_id].mode;
		if (QINQ_DECAP4 == smode) {
			targ->lconf->targs[task_id].qinq_gre_table = qinq_gre_table;
		}
	}
}

void init_cpe4_table(struct task_args *targ)
{
	char name[64];
	sprintf(name, "core_%u_CPEv4Table", targ->lconf->id);

	uint8_t table_part = targ->nb_slave_threads;
	if (!rte_is_power_of_2(table_part)) {
		table_part = rte_align32pow2(table_part) >> 1;
	}

	uint32_t n_entries = MAX_GRE / table_part;
	struct rte_table_hash_key8_ext_params table_hash_params = {
		.n_entries = n_entries,
		.n_entries_ext = n_entries >> 1,
		.f_hash = hash_crc32,
		.seed = 0,
		.signature_offset = 8,
		.key_offset = 0,
	};
	size_t entry_size = sizeof(struct cpe_data);
	if (!rte_is_power_of_2(entry_size)) {
		entry_size = rte_align32pow2(entry_size);
	}

	struct rte_table_hash* phash = rte_table_hash_key8_ext_dosig_ops.
		f_create(&table_hash_params, rte_lcore_to_socket_id(targ->lconf->id), entry_size);
	DPPD_PANIC(NULL == phash, "Unable to allocate memory for IPv4 hash table on core %u\n", targ->lconf->id);

	/* for locality, copy the pointer to the port structure where it is needed at packet handling time */
	for (uint8_t task_id = 0; task_id < targ->lconf->nb_tasks; ++task_id) {
		enum task_mode smode = targ->lconf->targs[task_id].mode;
		if (QINQ_ENCAP4 == smode || QINQ_DECAP4 == smode) {
			targ->lconf->targs[task_id].cpe_table = phash;
		}
	}
}

static void early_init_table(struct task_args* targ)
{
	if (!targ->cpe_table) {
		init_cpe4_table(targ);
	}
}

struct task_init task_init_qinq_encap4_table = {
	.mode = QINQ_ENCAP4,
	.mode_str = "qinqencapv4",
	.early_init = early_init_table,
	.init = init_task_qinq_encap4,
	.handle = handle_qinq_encap4_bulk,
	.thread_x = thread_generic,
	.flag_req_data = REQ_DSCP,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_qinq_encap4)
};

struct task_init task_init_qinq_encap4_table_pe = {
	.mode = QINQ_ENCAP4,
	.mode_str = "qinqencapv4",
	.sub_mode_str = "pe",
	.early_init = early_init_table,
	.init = init_task_qinq_encap4,
	.handle = handle_qinq_encap4_bulk_pe,
	.thread_x = thread_generic,
	.flag_req_data = REQ_DSCP | REQ_CPE_TABLE,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_qinq_encap4)
};

struct task_init task_init_qinq_encap4_untag = {
	.mode = QINQ_ENCAP4,
	.sub_mode_str = "unmpls",
	.mode_str = "qinqencapv4",
	.init = init_task_qinq_encap4,
	.handle = handle_qinq_encap4_untag_bulk,
	.thread_x = thread_generic,
	.flag_req_data = REQ_DSCP,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_qinq_encap4)
};

void reg_task_qinq_encap4(void)
{
	reg_task(&task_init_qinq_encap4_table);
	reg_task(&task_init_qinq_encap4_table_pe);
	reg_task(&task_init_qinq_encap4_untag);
}

static inline void restore_cpe(struct cpe_pkt* packet, struct cpe_data *table);
static inline void handle_qinq_encap4(struct task_qinq_encap4 *task, struct cpe_pkt *cpe_pkt, struct rte_mbuf *mbuf, struct cpe_data *entry);

void handle_qinq_encap4_untag_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_encap4 *task = (struct task_qinq_encap4 *)tbase;
	prefetch_pkts(mbufs, n_pkts);

	for (uint16_t j = 0; j < n_pkts; ++j) {
		if (likely(mpls_untag(mbufs[j]))) {
			struct cpe_pkt* cpe_pkt = (struct cpe_pkt*) rte_pktmbuf_adj(mbufs[j], UPSTREAM_DELTA);
			handle_qinq_encap4(task, cpe_pkt, mbufs[j], NULL);
		}
		else {
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbufs[j]);
		}
	}

	task->tx_pkt(&task->base);
}


static inline void extract_key_bulk(struct task_qinq_encap4 *task, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	for (uint16_t j = 0; j < n_pkts; ++j) {
		extract_key_core(mbufs[j], &task->keys[j]);
	}
}

static void handle_error(__attribute__((unused)) struct task_qinq_encap4 *task, struct rte_mbuf *mbuf)
{
	struct core_net_pkt* core_pkt = rte_pktmbuf_mtod(mbuf, struct core_net_pkt *);
	mprintf_verbose("Core Unknown IP %x/gre_id %x\n",  core_pkt->ip_hdr.dst_addr, core_pkt->gre_hdr.gre_id);
	INCR_TX_DROP_COUNT(task->base.stats, 1);
	rte_pktmbuf_free(mbuf);
}

void handle_qinq_encap4_bulk_pe(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_encap4 *task = (struct task_qinq_encap4 *)tbase;
	uint64_t pkts_mask = RTE_LEN2MASK(n_pkts, uint64_t);
	struct cpe_data* entries[64];
	uint64_t lookup_hit_mask;

	prefetch_pkts(mbufs, n_pkts);

	if (task->runtime_flags & TASK_DO_SIG) {

		for (uint16_t j = 0; j < n_pkts; ++j) {
			struct ipv4_hdr* ip = (struct ipv4_hdr *)(rte_pktmbuf_mtod(mbufs[j], struct ether_hdr *) + 1);
			task->keys[j] = (uint64_t)ip->dst_addr;
		}
		rte_table_hash_key8_ext_dosig_ops.f_lookup(task->cpe_table, task->fake_packets, pkts_mask, &lookup_hit_mask, (void**)entries);
	}
	else {
		rte_table_hash_key8_ext_ops.f_lookup(task->cpe_table, mbufs, pkts_mask, &lookup_hit_mask, (void**)entries);
	}

	if (likely(lookup_hit_mask == pkts_mask)) {
		for (uint16_t j = 0; j < n_pkts; ++j) {
			struct cpe_pkt* cpe_pkt = (struct cpe_pkt*) rte_pktmbuf_prepend(mbufs[j], sizeof(struct qinq_hdr) - sizeof(struct ether_hdr));
			uint16_t padlen = rte_pktmbuf_pkt_len(mbufs[j]) - rte_be_to_cpu_16(cpe_pkt->ipv4_hdr.total_length) - offsetof(struct cpe_pkt, ipv4_hdr);

			if (padlen) {
				rte_pktmbuf_trim(mbufs[j], padlen);
			}
			handle_qinq_encap4(task, cpe_pkt, mbufs[j], entries[j]);
		}
	}
	else {
		for (uint16_t j = 0; j < n_pkts; ++j) {
			if (unlikely(!((lookup_hit_mask >> j) & 0x1))) {
				handle_error(task, mbufs[j]);
				continue;
			}
			struct cpe_pkt* cpe_pkt = (struct cpe_pkt*) rte_pktmbuf_prepend(mbufs[j], sizeof(struct qinq_hdr) - sizeof(struct ether_hdr));
			uint16_t padlen = rte_pktmbuf_pkt_len(mbufs[j]) - rte_be_to_cpu_16(cpe_pkt->ipv4_hdr.total_length) - offsetof(struct cpe_pkt, ipv4_hdr);

			if (padlen) {
				rte_pktmbuf_trim(mbufs[j], padlen);
			}
			handle_qinq_encap4(task, cpe_pkt, mbufs[j], entries[j]);
		}
	}

	task->tx_pkt(&task->base);
}
void handle_qinq_encap4_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_encap4 *task = (struct task_qinq_encap4 *)tbase;
	uint64_t pkts_mask = RTE_LEN2MASK(n_pkts, uint64_t);
	struct cpe_data* entries[64];
	uint64_t lookup_hit_mask;

	prefetch_pkts(mbufs, n_pkts);

	if (task->runtime_flags & TASK_DO_SIG) {
		extract_key_bulk(task, mbufs, n_pkts);
		rte_table_hash_key8_ext_dosig_ops.f_lookup(task->cpe_table, task->fake_packets, pkts_mask, &lookup_hit_mask, (void**)entries);
	}
	else {
		rte_table_hash_key8_ext_ops.f_lookup(task->cpe_table, mbufs, pkts_mask, &lookup_hit_mask, (void**)entries);
	}

	if (likely(lookup_hit_mask == pkts_mask)) {
		for (uint16_t j = 0; j < n_pkts; ++j) {
			struct cpe_pkt* cpe_pkt = (struct cpe_pkt*) rte_pktmbuf_adj(mbufs[j], UPSTREAM_DELTA);
			handle_qinq_encap4(task, cpe_pkt, mbufs[j], entries[j]);
		}
	}
	else {
		for (uint16_t j = 0; j < n_pkts; ++j) {
			if (unlikely(!((lookup_hit_mask >> j) & 0x1))) {
				handle_error(task, mbufs[j]);
				continue;
			}
			struct cpe_pkt* cpe_pkt = (struct cpe_pkt*) rte_pktmbuf_adj(mbufs[j], UPSTREAM_DELTA);
			handle_qinq_encap4(task, cpe_pkt, mbufs[j], entries[j]);
		}
	}

	task->tx_pkt(&task->base);
}

static inline void handle_qinq_encap4(struct task_qinq_encap4 *task, struct cpe_pkt *cpe_pkt, struct rte_mbuf *mbuf, struct cpe_data *entry)
{
	DPPD_ASSERT(cpe_pkt);

	if (cpe_pkt->ipv4_hdr.time_to_live) {
		cpe_pkt->ipv4_hdr.time_to_live--;
	}
	else {
		mprintf("TTL = 0 => Dropping\n");
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return;
	}
	cpe_pkt->ipv4_hdr.hdr_checksum = 0;

	restore_cpe(cpe_pkt, entry);

	if (task->runtime_flags & TASK_CLASSIFY) {
		uint8_t queue = task->dscp[cpe_pkt->ipv4_hdr.type_of_service >> 2] & 0x3;
		uint8_t tc = task->dscp[cpe_pkt->ipv4_hdr.type_of_service >> 2] >> 2;

		rte_sched_port_pkt_write(mbuf, 0, entry->user, tc, queue, 0);
	}
#ifdef HARD_CRC
	dppd_ip_cksum_hw(mbuf, CKSUM_ETH_IP);
#elif defined(SOFT_CRC)
	dppd_ip_cksum_sw(pip, 0, &pip->ip_sum);
#endif

	tx_buf_pkt_single(&task->base, mbuf, entry->mac_port.out_idx);
}

static inline void restore_cpe(struct cpe_pkt *packet, struct cpe_data *table)
{
#ifdef USE_QINQ
        struct qinq_hdr *pqinq = &packet->qinq_hdr;
	rte_memcpy(pqinq, &qinq_proto, sizeof(struct qinq_hdr));
	(*(uint64_t *)(&pqinq->d_addr)) = table->mac_port_8bytes;
	/* set source as well now */
	*((uint64_t *)(&pqinq->s_addr)) = *((uint64_t *)&if_cfg[table->mac_port.out_idx]);
	pqinq->svlan.vlan_tci = table->qinq_svlan;
	pqinq->cvlan.vlan_tci = table->qinq_cvlan;
	pqinq->svlan.eth_proto = dppd_cfg.QinQ_tag;
	pqinq->cvlan.eth_proto = ETYPE_VLAN;
	pqinq->ether_type = ETYPE_IPv4;
#else
	(*(uint64_t *)(&packet->ether_hdr.d_addr)) = table->mac_port_8bytes;
	/* set source as well now */
	*((uint64_t *)(&packet->ether_hdr.s_addr)) = *((uint64_t *)&if_cfg[table->mac_port.port]);
	packet->ether_hdr.ether_type = ETYPE_IPv4;

	packet->ipv4_hdr.dst_addr = rte_bswap32(10 << 24 | rte_bswap16(table->qinq_svlan) << 12 | rte_bswap16(table->qinq_cvlan));
#endif
}
