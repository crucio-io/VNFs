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
#include <rte_byteorder.h>
#include <rte_cycles.h>
#include <rte_hash.h>
#include <rte_ip.h>
#include <rte_malloc.h>

#include "handle_gre_decap_encap.h"
#include "defines.h"
#include "stats.h"
#include "tx_pkt.h"
#include "hash_entry_types.h"
#include "prefetch.h"
#include "dppd_cksum.h"
#include "gre.h"
#include "etypes.h"
#include "display.h"
#include "dppd_assert.h"
#include "pkt_prototypes.h"
#include "thread_generic.h"
#include "quit.h"

struct cpe_gre_key {
	struct ether_addr clt_mac;
	uint16_t          pad;
} __attribute__((__packed__));

struct cpe_gre_data {
	uint32_t gre_id;
	uint32_t cpe_ip;
	uint64_t tsc;
#ifdef GRE_TP
	uint64_t tp_tsc;
	double tp_tbsize;
#endif
} __attribute__((__packed__));

struct task_gre_decap {
	struct task_base base;
	void (*tx_pkt) (struct task_base *tbase);
	struct rte_hash *cpe_gre_hash;
	struct cpe_gre_data *cpe_gre_data;
	struct lcore_cfg *lconf;
	uint8_t runtime_flags;
	uint8_t mapping[DPPD_MAX_PORTS];
	uint32_t bucket_index;
	const void* key_ptr[16];
	struct cpe_gre_key key[16];
	uint64_t           cpe_timeout;
#ifdef GRE_TP
	double cycles_per_byte;
	uint32_t tb_size;
#endif
};

static void handle_gre_decap_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
static void handle_gre_encap_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

static inline void handle_gre_encap(struct task_gre_decap *task, struct rte_mbuf *mbuf, struct cpe_gre_data *table);
static inline void handle_gre_encap16(struct task_gre_decap *task, struct rte_mbuf **mbufs, uint16_t n_pkts);
static inline void handle_gre_decap(struct task_gre_decap *tbase, struct rte_mbuf *mbuf);

void update_arp_entries_gre(void *data);

static void init_cpe_gre_hash(struct task_args *targ)
{
	char name[64];
	uint8_t socket_id;
	uint8_t lcore_id;
	uint8_t table_part;

	/* Already set up by other task */
	if (targ->cpe_gre_hash) {
		return;
	}

	lcore_id = targ->lconf->id;
	socket_id = rte_lcore_to_socket_id(lcore_id);
	sprintf(name, "core_%u_CPE_GRE_Table", targ->lconf->id);
	table_part = targ->nb_slave_threads;

	if (table_part == 0)
		table_part = 1;
	if (!rte_is_power_of_2(table_part)) {
		table_part = rte_align32pow2(table_part) >> 1;
	}

	struct rte_hash_parameters hash_params = {
		.name = name,
		.entries = MAX_GRE / table_part,
		.bucket_entries = GRE_BUCKET_ENTRIES,
		.key_len = sizeof(struct cpe_gre_key),
		.hash_func_init_val = 0,
		.socket_id = socket_id
	};

	struct rte_hash* phash = rte_hash_create(&hash_params);
	struct cpe_gre_data *cpe_gre_data = rte_zmalloc(NULL, MAX_GRE / table_part, socket_id);

	DPPD_PANIC(phash == NULL, "Unable to allocate memory for IPv4 hash table on core %u\n", lcore_id);

	for (uint8_t task_id = 0; task_id < targ->lconf->nb_tasks; ++task_id) {
		enum task_mode smode = targ->lconf->targs[task_id].mode;
		if (smode == GRE_DECAP || smode == GRE_ENCAP) {
			targ->lconf->targs[task_id].cpe_gre_hash = phash;
			targ->lconf->targs[task_id].cpe_gre_data = cpe_gre_data;
		}
	}
}

static void init_task_gre_decap(struct task_base *tbase, struct task_args *targ)
{
	struct task_gre_decap *task = (struct task_gre_decap *)tbase;

	init_cpe_gre_hash(targ);
	task->cpe_gre_hash = targ->cpe_gre_hash;
	task->cpe_gre_data = targ->cpe_gre_data;
	task->runtime_flags = targ->runtime_flags;
	task->lconf = targ->lconf;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);
	task->cpe_timeout = rte_get_tsc_hz()/1000*targ->cpe_table_timeout_ms;

	targ->lconf->period_func = update_arp_entries_gre;
	targ->lconf->period_data = tbase;
	targ->lconf->period_timeout = (rte_get_tsc_hz() >> 1) / NUM_VCPES;

	for (uint8_t i = 0; i < 16; ++i) {
		task->key_ptr[i] = &task->key[i];
	}
}

static void init_task_gre_encap(struct task_base *tbase, struct task_args *targ)
{
	struct task_gre_decap *task = (struct task_gre_decap *)tbase;

	init_cpe_gre_hash(targ);
	task->cpe_gre_hash = targ->cpe_gre_hash;
	task->cpe_gre_data = targ->cpe_gre_data;
	task->runtime_flags = targ->runtime_flags;
	task->lconf = targ->lconf;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);

#ifdef GRE_TP
	if (targ->tb_rate) {
		task->cycles_per_byte = ((double)rte_get_tsc_hz()) / ((double)targ->tb_rate);
		task->tb_size = targ->tb_size != 0 ? targ->tb_size : 1520;
	}
	else {
		/* traffic policing disabled */
		task->cycles_per_byte = 0;
	}
#endif
}

static struct task_init task_init_gre_decap = {
	.mode = GRE_DECAP,
	.mode_str = "gredecap",
	.init = init_task_gre_decap,
	.handle = handle_gre_decap_bulk,
	.thread_x = thread_generic,
	.size = sizeof(struct task_gre_decap)
};

static struct task_init task_init_gre_encap = {
	.mode = GRE_ENCAP,
	.mode_str = "greencap",
	.init = init_task_gre_encap,
	.handle = handle_gre_encap_bulk,
	.thread_x = thread_generic,
	.size = sizeof(struct task_gre_decap)
};

void reg_task_gre(void)
{
	reg_task(&task_init_gre_decap);
	reg_task(&task_init_gre_encap);
}

void handle_gre_decap_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_gre_decap *task = (struct task_gre_decap *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_gre_decap(task, mbufs[j]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_gre_decap(task, mbufs[j]);
	}
#endif
	task->tx_pkt(&task->base);
}

struct gre_packet {
	struct ether_hdr eth;
	struct ipv4_hdr ip;
	struct gre_hdr gre;
	union {
		struct ether_hdr eth2;
		struct ipv4_hdr ip2;
	};
} __attribute__((__packed__));

#if 0
static const char *ntos(unsigned int nip)
{
	static char s[24];
	int hip = rte_bswap32(nip);
	sprintf(s, "%d.%d.%d.%d", (hip >> 24) & 0xFF, (hip >> 16) & 0xFF,
		(hip >> 8) & 0xFF, (hip) & 0xFF);
	return s;
}
#endif

/* Handle ipv4 over GRE and Ethernet over GRE. In case of ipv4 over
   GRE remove gre and ipv4 header and retain space for ethernet
   header. In case of Eth over GRE remove external eth, gre and ipv4
   headers and return pointer to payload */
static inline struct ether_hdr *gre_decap(struct gre_hdr *pgre, struct rte_mbuf *mbuf)
{
	int16_t hsize = 0;
	if (pgre->type == ETYPE_EoGRE) {
		hsize = sizeof(struct ether_hdr) + sizeof(struct ipv4_hdr) + sizeof(struct gre_hdr);
	}
	else if (pgre->type == ETYPE_IPv4) {
		/* retain sizeof(struct ether_hdr) */
		hsize = sizeof(struct ipv4_hdr) + sizeof(struct gre_hdr);
	}
	else {
		return NULL;
	}

	return (struct ether_hdr *)rte_pktmbuf_adj(mbuf, hsize);
}

static inline void handle_gre_decap(struct task_gre_decap *task, struct rte_mbuf *mbuf)
{
	struct ipv4_hdr *pip = (struct ipv4_hdr *)(rte_pktmbuf_mtod(mbuf, struct ether_hdr *) + 1);

#ifdef HARD_CRC
	dppd_ip_cksum_hw(mbuf, CKSUM_ETH_IP);
#endif

	if (pip->next_proto_id != IPPROTO_GRE) {
		mprintf("Invalid packet proto_id = 0x%x expect 0x%x\n",
			pip->next_proto_id, IPPROTO_GRE);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return;
	}

	struct cpe_gre_data data;
	struct cpe_gre_key key;
	struct gre_hdr *pgre = (struct gre_hdr *)(pip + 1);
	data.gre_id = pgre->gre_id;
	data.cpe_ip = pip->src_addr;

	struct ether_hdr *peth = gre_decap(pgre, mbuf);
	DPPD_ASSERT(peth);

	pip = (struct ipv4_hdr *)(peth + 1);

/* emulate client MAC for test purposes */
#if 1
	if (pgre->type == ETYPE_IPv4) {
		struct ether_hdr eth = {
			.d_addr = {.addr_bytes =
				   {0x0A, 0x02, 0x0A, 0x0A, 0x00, 0x01}},
			.s_addr = {.addr_bytes =
				   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
			.ether_type = ETYPE_IPv4
		};
		uint32_t hip = rte_bswap32(pip->src_addr);
		eth.s_addr.addr_bytes[2] = (hip >> 24) & 0xFF;
		eth.s_addr.addr_bytes[3] = (hip >> 16) & 0xFF;
		eth.s_addr.addr_bytes[4] = (hip >> 8) & 0xFF;
		eth.s_addr.addr_bytes[5] = (hip) & 0xFF;
		rte_memcpy(peth, &eth, sizeof(struct ether_hdr));
	}
	ether_addr_copy(&peth->s_addr, &key.clt_mac);
#endif

	data.tsc = rte_rdtsc() + task->cpe_timeout;

	int32_t hash_index = rte_hash_add_key(task->cpe_gre_hash, &key);
	if (unlikely(hash_index < 0)) {
		mprintf("Failed to add key, gre %x\n", data.gre_id);
	}
	else if (unlikely(hash_index >= MAX_GRE)) {
		mprintf("Failed to add: Invalid hash_index = 0x%x\n",
			hash_index);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return;
	}
	rte_memcpy(&task->cpe_gre_data[hash_index], &data, sizeof(data));
#ifndef HARD_CRC
#ifdef SOFT_CRC
	dppd_ip_cksum_sw(pip, 0, &pip->hdr_checksum);
#endif
#endif

	tx_buf_pkt_single(&task->base, mbuf, 0);
}

void handle_gre_encap_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_gre_decap *task = (struct task_gre_decap *)tbase;

	while (n_pkts) {
		uint16_t chopped = RTE_MIN(n_pkts, 16);
		prefetch_pkts(mbufs, chopped);
		handle_gre_encap16(task, mbufs, chopped);
		mbufs += chopped;
		n_pkts -= chopped;
	}

	task->tx_pkt(&task->base);
}

#define DO_ENC_ETH_OVER_GRE 1
#define DO_ENC_IP_OVER_GRE 0

static inline void handle_gre_encap16(struct task_gre_decap *task, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	for (uint8_t i = 0; i < n_pkts; ++i) {
		struct ether_hdr *peth = rte_pktmbuf_mtod(mbufs[i], struct ether_hdr *);
		ether_addr_copy(&peth->d_addr, &task->key[i].clt_mac);
	}

	int32_t hash_index[16];
	rte_hash_lookup_bulk(task->cpe_gre_hash, task->key_ptr, n_pkts, hash_index);
	for (uint8_t i = 0; i < n_pkts; ++i ) {
		if (unlikely(hash_index[i] < 0)) {
			mprintf("Invalid hash_index (<0) = 0x%x\n", hash_index[i]);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbufs[i]);
			continue;
		}
		else if (unlikely(hash_index[i] >= MAX_GRE)) {
			mprintf("Invalid hash_index = 0x%x\n", hash_index[i]);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbufs[i]);
			continue;
		}
		rte_prefetch0(&task->cpe_gre_data[hash_index[i]]);
	}

	for (uint8_t i = 0; i < n_pkts; ++i ) {
		if (unlikely(hash_index[i] < 0 || hash_index[i] >= MAX_GRE)) {
			continue;
		}
		handle_gre_encap(task, mbufs[i], &task->cpe_gre_data[hash_index[i]]);
	}
}

static inline void handle_gre_encap(struct task_gre_decap *task, struct rte_mbuf *mbuf, struct cpe_gre_data *table)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
	struct ipv4_hdr *pip = (struct ipv4_hdr *)(peth + 1);
	uint16_t ip_len = rte_be_to_cpu_16(pip->total_length);

	struct cpe_gre_key key;
	ether_addr_copy(&peth->d_addr, &key.clt_mac);


#ifdef GRE_TP
	/* policing enabled */
	if (task->cycles_per_byte) {
		const uint16_t pkt_size = rte_pktmbuf_pkt_len(mbuf) + ETHER_CRC_LEN;
		uint64_t tsc_now = rte_rdtsc();
		if (table->tp_tbsize < pkt_size) {
			uint64_t cycles_diff = tsc_now - table->tp_tsc;
			double dB = ((double)cycles_diff) / task->cycles_per_byte;
			if (dB > (double)task->tb_size) {
				dB = task->tb_size;
			}
			if ((table->tp_tbsize + dB) >= pkt_size) {
				table->tp_tbsize += dB;
				table->tp_tsc = tsc_now;
			}
			else {
				INCR_TX_DROP_COUNT(task->base.stats, 1);
				rte_pktmbuf_free(mbuf);
				return;
			}
		}
		table->tp_tbsize -= pkt_size;
	}
#endif /* GRE_TP */

#if DO_ENC_ETH_OVER_GRE
	peth = (struct ether_hdr *)rte_pktmbuf_prepend(mbuf,
						       sizeof(struct ether_hdr) +
						       sizeof(struct ipv4_hdr) +
						       sizeof(struct gre_hdr));
	PREFETCH0(peth);
	ip_len +=
	    (sizeof(struct ether_hdr) + sizeof(struct ipv4_hdr) +
	     sizeof(struct gre_hdr));
#elif DO_ENC_IP_OVER_GRE
	/* reuse ethernet header from payload, retain payload (ip) */
	peth = (struct ether_hdr *)rte_pktmbuf_prepend(mbuf,
						       sizeof(struct ipv4_hdr) +
						       sizeof(struct gre_hdr));
	PREFETCH0(peth);
	ip_len += (sizeof(struct ipv4_hdr) + sizeof(struct gre_hdr));
#endif

	pip = (struct ipv4_hdr *)(peth + 1);
	struct gre_hdr *pgre = (struct gre_hdr *)(pip + 1);

	struct ether_hdr eth = {
		.d_addr = {.addr_bytes = {0x0A, 0x0A, 0x0A, 0xC8, 0x00, 0x02}},
		.s_addr = {.addr_bytes = {0x0A, 0x0A, 0x0A, 0xC8, 0x00, 0x01}},
		.ether_type = ETYPE_IPv4
	};
	rte_memcpy(peth, &eth, sizeof(struct ether_hdr));

	rte_memcpy(pgre, &gre_hdr_proto, sizeof(struct gre_hdr));
#if DO_ENC_ETH_OVER_GRE
	pgre->type = ETYPE_EoGRE;
#elif DO_ENC_IP_OVER_GRE
	pgre->type = ETYPE_IPv4;
#endif
	pgre->gre_id = table->gre_id;

	rte_memcpy(pip, &tunnel_ip_proto, sizeof(struct ipv4_hdr));
	pip->src_addr = 0x02010a0a;	//emulate port ip
	pip->dst_addr = table->cpe_ip;
	pip->total_length = rte_cpu_to_be_16(ip_len);

#ifdef SOFT_CRC
	dppd_ip_cksum_sw(pip, 0, &pip->hdr_checksum);
#elif defined(HARD_CRC)
	dppd_ip_cksum_hw(mbuf, CKSUM_ETH_IP);
#endif

	tx_buf_pkt_single(&task->base, mbuf, 0);
}

void update_arp_entries_gre(void *data)
{
	uint64_t cur_tsc = rte_rdtsc();
	struct task_gre_decap *task = (struct task_gre_decap *)data;

	uint32_t *sig_bucket = (hash_sig_t *)&(task->cpe_gre_hash->sig_tbl[task->bucket_index * task->cpe_gre_hash->sig_tbl_bucket_size]);
	uint32_t table_index = task->bucket_index * task->cpe_gre_hash->bucket_entries;

	uint8_t *entry_bucket =
	    (uint8_t *) & task->cpe_gre_hash->key_tbl[task->bucket_index * task->cpe_gre_hash->bucket_entries * task->cpe_gre_hash->key_tbl_key_size];

	for (uint32_t pos = 0; pos < task->cpe_gre_hash->bucket_entries; ++pos, ++table_index) {
		struct cpe_gre_entry *key = (struct cpe_gre_entry *)&entry_bucket[pos * task->cpe_gre_hash->key_tbl_key_size];
		if (task->cpe_gre_data[table_index].tsc < cur_tsc) {
			sig_bucket[pos] = 0;
			task->cpe_gre_data[table_index].tsc = UINT64_MAX;
		}
	}
	++task->bucket_index;
	task->bucket_index &= task->cpe_gre_hash->bucket_bitmask;
}
