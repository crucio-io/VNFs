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
#include <rte_table_hash.h>
#include <rte_lpm.h>

#include "handle_qinq_decap4.h"
#include "handle_qinq_encap4.h"
#include "display.h"
#include "stats.h"
#include "tx_pkt.h"
#include "defines.h"
#include "handle_routing.h"
#include "dppd_assert.h"
#include "task_init.h"
#include "quit.h"
#include "pkt_prototypes.h"
#include "task_base.h"
#include "task_init.h"
#include "bng_pkts.h"
#include "thread_generic.h"
#include "dppd_cksum.h"
#include "expire_cpe.h"
#include "dppd_port_cfg.h"
#include "prefetch.h"

#ifdef MPLS_ROUTING
#define VLAN_MACIP_DATA CKSUM_ETH_MPLS_IP
#else
#define VLAN_MACIP_DATA CKSUM_ETH_IP
#endif

struct task_qinq_decap4 {
	struct task_base        base;
	void (*tx_pkt)(struct task_base *tbase);
	struct rte_table_hash         *cpe_table;
	struct rte_table_hash         *qinq_gre_table;
	struct qinq_gre_data    *qinq_gre_data;
	struct next_hop_struct  *next_hop;
	struct rte_lpm          *ipv4_lpm;
	uint32_t                local_ipv4;
	uint8_t                 runtime_flags;
	uint64_t                keys[64];
	uint64_t                src_mac[DPPD_MAX_PORTS];
	struct rte_mbuf*        fake_packets[64];
	struct expire_cpe       expire_cpe;
	uint64_t                cpe_timeout;
	uint8_t                 mapping[DPPD_MAX_PORTS];
};

/* Convert IPv4 packets to GRE and optionally store QinQ Tags */
static void handle_qinq_decap4_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
static void handle_qinq_decap4_bulk_pe(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
static void handle_error(__attribute__((unused)) struct task_qinq_decap4* task, struct rte_mbuf *mbuf);
static void handle_qinq_decap4_table(struct task_qinq_decap4 *task, struct rte_mbuf *mbuf, struct qinq_gre_data* entries);
static void handle_qinq_decap4(struct task_qinq_decap4 *task, struct rte_mbuf *mbuf, struct qinq_gre_data* entry);
static void arp_update(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
static void arp_msg(struct task_base *tbase, void **data, uint16_t n_msgs);

static void init_task_qinq_decap4(struct task_base *tbase, struct task_args *targ)
{
	struct task_qinq_decap4 *task = (struct task_qinq_decap4 *)tbase;
	task->cpe_table = targ->cpe_table;
	task->cpe_timeout = rte_get_tsc_hz()/1000*targ->cpe_table_timeout_ms;

	if (targ->dppd_shared) {
		task->next_hop = targ->dppd_shared->next_hop;
		task->ipv4_lpm = targ->dppd_shared->ipv4_lpm;
	}

	task->local_ipv4 = targ->local_ipv4;
	task->runtime_flags = targ->runtime_flags;
	if (strcmp(targ->task_init->sub_mode_str, "pe"))
		DPPD_PANIC(targ->qinq_gre_table == NULL, "can't set up qinq gre\n");
	task->qinq_gre_table = targ->qinq_gre_table;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);

	if (targ->cpe_table_timeout_ms) {
		targ->lconf->period_func = check_expire_cpe;
		task->expire_cpe.cpe_table = task->cpe_table;
		targ->lconf->period_data = &task->expire_cpe;
		targ->lconf->period_timeout = (rte_get_tsc_hz() >> 1) / NUM_VCPES;
	}

	for (uint32_t i = 0; i < 64; ++i) {
		task->fake_packets[i] = (struct rte_mbuf*)((uint8_t*)&task->keys[i] - sizeof (struct rte_mbuf));
	}
	if (task->runtime_flags & TASK_ROUTING)
		for (uint32_t i = 0; i < targ->nb_txrings || i < targ->nb_txports; ++i) {
			task->src_mac[i] = *(uint64_t*)&dppd_port_cfg[targ->tx_port_queue[i].port].eth_addr;
		}

	if (targ->runtime_flags & TASK_HANDLE_ARP) {
		targ->lconf->ctrl_func_p[targ->task] = arp_update;

		targ->lconf->ctrl_timeout = rte_get_tsc_hz()/1000;
	}

	/* Copy the mapping from a sibling task which is configured
	   with mode encap4. The mapping is constant, so it is faster
	   to apply it when entries are added (least common case)
	   instead of re-applying it for every packet (most common
	   case). */

	for (uint8_t task_id = 0; task_id < targ->lconf->nb_tasks; ++task_id) {
		enum task_mode smode = targ->lconf->targs[task_id].mode;
		if (QINQ_ENCAP4 == smode) {
			for (uint8_t i = 0; i < DPPD_MAX_PORTS; ++i) {
				task->mapping[i] = targ->lconf->targs[task_id].mapping[i];
			}
		}
	}

	targ->lconf->ctrl_timeout = rte_get_tsc_hz()/1000;
	targ->lconf->ctrl_func_m[targ->task] = arp_msg;
}

static void early_init_table(struct task_args *targ)
{
	if (!targ->qinq_gre_table && !targ->cpe_table) {
		init_qinq_gre_table(targ, targ->dppd_shared);
		init_cpe4_table(targ);
	}
}

struct task_init task_init_qinq_decapv4_table = {
	.mode = QINQ_DECAP4,
	.mode_str = "qinqdecapv4",
	.early_init = early_init_table,
	.init = init_task_qinq_decap4,
	.handle = handle_qinq_decap4_bulk,
	.thread_x = thread_generic,
	.flag_req_data = REQ_GRE_TABLE | REQ_NEXT_HOP | REQ_LPM4,
	.flag_features = TASK_ROUTING,
	.size = sizeof(struct task_qinq_decap4)
};

struct task_init task_init_qinq_decapv4_table_pe = {
	.mode = QINQ_DECAP4,
	.mode_str = "qinqdecapv4",
	.sub_mode_str = "pe",
	.init = init_task_qinq_decap4,
	.handle = handle_qinq_decap4_bulk_pe,
	.thread_x = thread_generic,
	/* not using shared data structures */
	.size = sizeof(struct task_qinq_decap4)
};

void reg_task_qinq_decap4(void)
{
	reg_task(&task_init_qinq_decapv4_table);
	reg_task(&task_init_qinq_decapv4_table_pe);
}

static inline void extract_key_bulk(struct rte_mbuf **mbufs, uint16_t n_pkts, struct task_qinq_decap4 *task)
{
	for (uint16_t j = 0; j < n_pkts; ++j) {
		extract_key_cpe(mbufs[j], &task->keys[j]);
	}
}

static void handle_error(__attribute__((unused)) struct task_qinq_decap4* task, struct rte_mbuf *mbuf)
{
        struct cpe_pkt *packet = rte_pktmbuf_mtod(mbuf, struct cpe_pkt *);
#ifdef USE_QINQ
	uint32_t svlan = packet->qinq_hdr.svlan.vlan_tci;
	uint32_t cvlan = packet->qinq_hdr.cvlan.vlan_tci;

	svlan = rte_be_to_cpu_16(svlan & 0xFF0F);
	cvlan = rte_be_to_cpu_16(cvlan & 0xFF0F);
	mprintf("Error converting qinq %d|%d to gre_id\n", svlan, cvlan);
#else
	mprintf("Error converting ip %x to gre_id\n", rte_bswap32(packet->ipv4_hdr.src_addr));
#endif

	rte_pktmbuf_free(mbuf);
	INCR_TX_DROP_COUNT(task->base.stats, 1);
}

static int add_arp_entry(struct rte_table_hash *hash, struct cpe_key *key, struct cpe_data *data)
{
	void* entry_in_hash;
	int ret, key_found = 0;

	ret = rte_table_hash_key8_ext_dosig_ops.
		f_add(hash, &key, &data, &key_found, &entry_in_hash);
	if (unlikely(ret)) {
		mprintf_once(1, "Failed to add key: ip %x, gre %x\n", key->ip, key->gre_id);
		return 1;
	}
	return 0;
}

static void extract_key_data_arp(struct rte_mbuf* mbuf, struct cpe_key* key, struct cpe_data* data, const struct qinq_gre_data* entry, uint64_t cpe_timeout, uint8_t* mapping)
{
	const struct cpe_packet_arp *packet = rte_pktmbuf_mtod(mbuf, const struct cpe_packet_arp *);
	uint32_t svlan = packet->qinq_hdr.svlan.vlan_tci & 0xFF0F;
	uint32_t cvlan = packet->qinq_hdr.cvlan.vlan_tci & 0xFF0F;
	uint8_t port_id;
	key->ip = packet->arp.data.spa;
	key->gre_id = entry->gre_id;

	data->mac_port_8bytes = *((const uint64_t *)(&packet->qinq_hdr.s_addr));
	data->qinq_svlan = svlan;
	data->qinq_cvlan = cvlan;
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	port_id = mbuf->port;

#else
	port_id = mbuf->pkt.in_port;
#endif
	uint8_t mapped = mapping[port_id];
	data->mac_port.out_idx = mapping[port_id];

	if (unlikely(mapped == 255)) {
		/* This error only occurs if the system is configured incorrectly */
		mprintf("Error adding packet: unknown mapping for port %d", port_id);
		data->mac_port.out_idx = 0;
	}

	data->user = entry->user;
	data->tsc = rte_rdtsc() + cpe_timeout;
}

void arp_msg_to_str(char *str, struct arp_msg *msg)
{
	sprintf(str, "%u %u %u %u %u.%u.%u.%u %x:%x:%x:%x:%x:%x %u\n",
		msg->data.mac_port.out_idx, msg->key.gre_id, msg->data.qinq_svlan, msg->data.qinq_cvlan,
		msg->key.ip_bytes[0], msg->key.ip_bytes[1], msg->key.ip_bytes[2], msg->key.ip_bytes[3],
		msg->data.mac_port_b[0], msg->data.mac_port_b[1], msg->data.mac_port_b[2],
		msg->data.mac_port_b[3], msg->data.mac_port_b[4], msg->data.mac_port_b[5], msg->data.user);
}

int str_to_arp_msg(struct arp_msg *msg, const char *str)
{
	uint32_t ip[4],	interface, gre_id, svlan, cvlan, mac[6], user;

	int ret = sscanf(str, "%u %u %u %u %u.%u.%u.%u %x:%x:%x:%x:%x:%x %u",
			 &interface, &gre_id, &svlan, &cvlan,
			 ip, ip + 1, ip + 2, ip + 3,
			 mac, mac + 1, mac + 2, mac + 3, mac + 4, mac + 5, &user);

	for (uint8_t i = 0; i < 4; ++i)
		msg->key.ip_bytes[i] = ip[i];
	msg->key.gre_id = gre_id;

	for (uint8_t i = 0; i < 4; ++i)
		msg->data.mac_port_b[i] = mac[i];
	msg->data.qinq_svlan = svlan;
	msg->data.qinq_cvlan = cvlan;
	msg->data.user = user;
	msg->data.mac_port.out_idx = interface;

	return ret != 15;
}

void arp_update_from_msg(struct rte_table_hash * cpe_table, struct arp_msg **msgs, uint16_t n_msgs, uint64_t cpe_timeout)
{
	int ret, key_found = 0;
	void* entry_in_hash;

	for (uint16_t i = 0; i < n_msgs; ++i) {
		msgs[i]->data.tsc = rte_rdtsc() + cpe_timeout;
		ret = rte_table_hash_key8_ext_dosig_ops.
			f_add(cpe_table, &msgs[i]->key, &msgs[i]->data, &key_found, &entry_in_hash);
		if (unlikely(ret)) {
			mprintf_once(1, "Failed to add key %x, gre %x\n", msgs[i]->key.ip, msgs[i]->key.gre_id);
		}
	}
}

static void arp_msg(struct task_base *tbase, void **data, uint16_t n_msgs)
{
	struct task_qinq_decap4 *task = (struct task_qinq_decap4 *)tbase;
	struct arp_msg **msgs = (struct arp_msg **)data;

	arp_update_from_msg(task->cpe_table, msgs, n_msgs, task->cpe_timeout);
}

static void arp_update(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_decap4 *task = (struct task_qinq_decap4 *)tbase;

	prefetch_pkts(mbufs, n_pkts);
	extract_key_bulk(mbufs, n_pkts, task);

	uint64_t pkts_mask = RTE_LEN2MASK(n_pkts, uint64_t);
	uint64_t lookup_hit_mask = 0;
	struct qinq_gre_data* entries[64];
	rte_table_hash_key8_ext_dosig_ops.f_lookup(task->qinq_gre_table, task->fake_packets, pkts_mask, &lookup_hit_mask, (void**)entries);


	for (uint16_t j = 0; j < n_pkts; ++j) {
		if (unlikely(!((lookup_hit_mask >> j) & 0x1))) {
			handle_error(task, mbufs[j]);
			continue;
		}

		struct cpe_key key;
		struct cpe_data data;

		extract_key_data_arp(mbufs[j], &key, &data, entries[j], task->cpe_timeout, task->mapping);

		void* entry_in_hash;
		int ret, key_found = 0;

		ret = rte_table_hash_key8_ext_dosig_ops.
			f_add(task->cpe_table, &key, &data, &key_found, &entry_in_hash);

		if (unlikely(ret)) {
			mprintf_once(1, "Failed to add key %x, gre %x\n", key.ip, key.gre_id);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
		}

		/* should do ARP reply */
		rte_pktmbuf_free(mbufs[j]);
	}
}

void handle_qinq_decap4_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_decap4 *task = (struct task_qinq_decap4 *)tbase;
	uint64_t pkts_mask = RTE_LEN2MASK(n_pkts, uint64_t);
	struct qinq_gre_data* entries[64];
	uint64_t lookup_hit_mask;

	prefetch_pkts(mbufs, n_pkts);

	if (task->runtime_flags & TASK_DO_SIG) {
		extract_key_bulk(mbufs, n_pkts, task);
		rte_table_hash_key8_ext_dosig_ops.f_lookup(task->qinq_gre_table, task->fake_packets, pkts_mask, &lookup_hit_mask, (void**)entries);
	}
	else {
		rte_table_hash_key8_ext_ops.f_lookup(task->qinq_gre_table, mbufs, pkts_mask, &lookup_hit_mask, (void**)entries);
	}

	if (likely(lookup_hit_mask == pkts_mask)) {
		for (uint16_t j = 0; j < n_pkts; ++j) {
			handle_qinq_decap4_table(task, mbufs[j], entries[j]);
		}
	}
	else {
		for (uint16_t j = 0; j < n_pkts; ++j) {
			if (unlikely(!((lookup_hit_mask >> j) & 0x1))) {
	                        handle_error(task, mbufs[j]);
				continue;
			}
			handle_qinq_decap4_table(task, mbufs[j], entries[j]);
		}
	}

	task->tx_pkt(&task->base);
}

void handle_qinq_decap4_bulk_pe(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_qinq_decap4 *task = (struct task_qinq_decap4 *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		rte_pktmbuf_adj(mbufs[j], sizeof(struct qinq_hdr) - sizeof(struct ether_hdr));
		/* No need to write etype (already correct) */
		tx_buf_pkt_single(&task->base, mbufs[j], 0);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		rte_pktmbuf_adj(mbufs[j], sizeof(struct qinq_hdr) - sizeof(struct ether_hdr));
		/* No need to write etype (already correct) */
		tx_buf_pkt_single(&task->base, mbufs[j], 0);
	}
#endif

	task->tx_pkt(&task->base);
}

/* add gre header */
static inline void gre_encap(uint32_t src_ipv4, struct rte_mbuf *mbuf, uint32_t gre_id)
{
	struct ipv4_hdr *pip = (struct ipv4_hdr *)(1 + rte_pktmbuf_mtod(mbuf, struct qinq_hdr *));
	uint16_t ip_len = rte_be_to_cpu_16(pip->total_length);


	uint16_t padlen = rte_pktmbuf_pkt_len(mbuf) - 20 - ip_len - sizeof(struct qinq_hdr);

	if (padlen) {
		rte_pktmbuf_trim(mbuf, padlen);
	}

	/* prepend only 20 bytes instead of 28, 8 bytes are present from the QinQ */
	struct ether_hdr *peth = (struct ether_hdr *)rte_pktmbuf_prepend(mbuf, 20);

	DPPD_ASSERT(peth);
	PREFETCH0(peth);
	/* calculate IP CRC here to avoid problems with -O3 flag with gcc */
#ifdef SOFT_CRC
	dppd_ip_cksum_sw(pip, 0, &pip->hdr_checksum);
#elif defined(HARD_CRC)
	dppd_ip_cksum_hw(mbuf, CKSUM_ETH_IP);
#endif


	DPPD_PANIC(rte_pktmbuf_data_len(mbuf) - padlen + 20 > ETHER_MAX_LEN,
	           "Would need to fragment packet new size = %u - not implemented\n",
	           rte_pktmbuf_data_len(mbuf) - padlen + 20);


	/* new IP header */
	struct ipv4_hdr *p_tunnel_ip = (struct ipv4_hdr *)(peth + 1);
	rte_memcpy(p_tunnel_ip, &tunnel_ip_proto, sizeof(struct ipv4_hdr));
	ip_len += sizeof(struct ipv4_hdr) + sizeof(struct gre_hdr);
	p_tunnel_ip->total_length = rte_cpu_to_be_16(ip_len);
	p_tunnel_ip->src_addr = src_ipv4;

	/* Add GRE Header values */
	struct gre_hdr *pgre = (struct gre_hdr *)(p_tunnel_ip + 1);

	rte_memcpy(pgre, &gre_hdr_proto, sizeof(struct gre_hdr));
	pgre->gre_id = gre_id;
	peth->ether_type = ETYPE_IPv4;
}

static inline uint16_t calc_padlen(const struct rte_mbuf *mbuf, const uint16_t ip_len)
{
	return rte_pktmbuf_pkt_len(mbuf) - DOWNSTREAM_DELTA - ip_len - offsetof(struct cpe_pkt, ipv4_hdr);
}

static inline uint8_t gre_encap_route(uint32_t src_ipv4, struct rte_mbuf *mbuf, uint32_t gre_id, struct task_qinq_decap4 *task)
{
	struct core_net_pkt_m *packet = (struct core_net_pkt_m *)rte_pktmbuf_prepend(mbuf, DOWNSTREAM_DELTA);
	DPPD_ASSERT(packet);
	PREFETCH0(packet);

	struct ipv4_hdr *pip = &((struct cpe_pkt_delta *)packet)->pkt.ipv4_hdr;
	uint16_t ip_len = rte_be_to_cpu_16(pip->total_length);

	uint8_t next_hop_index;
	/* returns 0 on success, returns -ENOENT of failure (or -EINVAL if first or last parameter is NULL) */
	if (unlikely(rte_lpm_lookup(task->ipv4_lpm, rte_bswap32(pip->dst_addr), &next_hop_index) != 0)) {
		mprintf("lpm_lookup failed for ip %x: rc = %d\n", rte_bswap32(pip->dst_addr), -ENOENT);
		return ROUTE_ERR;
	}
	PREFETCH0(&task->next_hop[next_hop_index]);

	/* calculate IP CRC here to avoid problems with -O3 flag with gcc */

#ifdef SOFT_CRC
	dppd_ip_cksum_sw((void *)pip, 1, &pip->hdr_checksum);
#elif defined(HARD_CRC)
	dppd_ip_cksum_hw(mbuf, VLAN_MACIP_DATA);
#endif

	const uint16_t padlen = calc_padlen(mbuf, ip_len);
	if (padlen) {
		rte_pktmbuf_trim(mbuf, padlen);
	}
	const uint8_t port_id = task->next_hop[next_hop_index].mac_port.out_idx;

	*((uint64_t *)(&packet->ether_hdr.d_addr)) = task->next_hop[next_hop_index].mac_port_8bytes;
	*((uint64_t *)(&packet->ether_hdr.s_addr)) = task->src_mac[task->next_hop[next_hop_index].mac_port.out_idx];

#ifdef MPLS_ROUTING
	packet->mpls_bytes = task->next_hop[next_hop_index].mpls | 0x00010000; // Set BoS to 1
	packet->ether_hdr.ether_type = ETYPE_MPLSU;
#else
	packet->ether_hdr.ether_type = ETYPE_IPv4;
#endif

	DPPD_PANIC(rte_pktmbuf_data_len(mbuf) + DOWNSTREAM_DELTA  > ETHER_MAX_LEN,
	           "Would need to fragment packet new size = %u - not implemented\n",
	           rte_pktmbuf_data_len(mbuf) + DOWNSTREAM_DELTA);

	/* New IP header */
	rte_memcpy(&packet->tunnel_ip_hdr, &tunnel_ip_proto, sizeof(struct ipv4_hdr));
	ip_len += sizeof(struct ipv4_hdr) + sizeof(struct gre_hdr);
	packet->tunnel_ip_hdr.total_length = rte_cpu_to_be_16(ip_len);
	packet->tunnel_ip_hdr.src_addr = src_ipv4;

	/* Add GRE Header values */
	rte_memcpy(&packet->gre_hdr, &gre_hdr_proto, sizeof(struct gre_hdr));
	packet->gre_hdr.gre_id = rte_be_to_cpu_32(gre_id);

	return port_id;
}

static void extract_key_data(struct rte_mbuf* mbuf, struct cpe_key* key, struct cpe_data* data, const struct qinq_gre_data* entry, uint64_t cpe_timeout, uint8_t *mapping)
{
	struct cpe_pkt *packet = rte_pktmbuf_mtod(mbuf, struct cpe_pkt *);
	uint8_t port_id;

#ifndef USE_QINQ
        const uint32_t tmp = rte_bswap32(packet->ipv4_hdr.src_addr) & 0x00FFFFFF;
	const uint32_t svlan = rte_bswap16(tmp >> 12);
	const uint32_t cvlan = rte_bswap16(tmp & 0x0FFF);
#endif

#ifdef USE_QINQ
	key->ip = packet->ipv4_hdr.src_addr;
#else
	key->ip = 0;
#endif
	key->gre_id = entry->gre_id;

#ifdef USE_QINQ
	data->mac_port_8bytes = *((const uint64_t *)(&packet->qinq_hdr.s_addr));
	data->qinq_svlan      = packet->qinq_hdr.svlan.vlan_tci & 0xFF0F;
	data->qinq_cvlan      = packet->qinq_hdr.cvlan.vlan_tci & 0xFF0F;
#else
	data->mac_port_8bytes = *((const uint64_t *)(&packet->ether_hdr.s_addr));
	data->qinq_svlan      = svlan;
	data->qinq_cvlan      = cvlan;
#endif

#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	port_id = mbuf->port;

#else
	port_id = mbuf->pkt.in_port;
#endif
	uint8_t mapped = mapping[port_id];
	data->mac_port.out_idx = mapped;

	if (unlikely(mapped == 255)) {
		/* This error only occurs if the system is configured incorrectly */
		mprintf("Error adding packet: unknown mapping for port %d", port_id);
		data->mac_port.out_idx = 0;
	}


#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	data->mac_port.out_idx = mapping[mbuf->port];
#else
	data->mac_port.out_idx = mapping[mbuf->pkt.in_port];
#endif
	data->user            = entry->user;
	data->tsc             = rte_rdtsc() + cpe_timeout;
}

static void handle_qinq_decap4_table(struct task_qinq_decap4 *task, struct rte_mbuf *mbuf, struct qinq_gre_data* entry)
{
	if (!(task->runtime_flags & TASK_HANDLE_ARP)) {
		struct cpe_key key;
		struct cpe_data data;
		extract_key_data(mbuf, &key, &data, entry, task->cpe_timeout, task->mapping);

		if (add_arp_entry(task->cpe_table, &key, &data)) {
			mprintf("Failed to add ARP entry\n");
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return;
		}
	}

	if (task->runtime_flags & TASK_ROUTING) {
		uint8_t tx_portid;

		if (ROUTE_ERR == (tx_portid = gre_encap_route(task->local_ipv4, mbuf, entry->gre_id, task))) {
			rte_pktmbuf_free(mbuf);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			return;
		}

		tx_buf_pkt_single(&task->base, mbuf, tx_portid);
	}
	else {
		gre_encap(task->local_ipv4, mbuf, entry->gre_id);
		tx_buf_pkt_single(&task->base, mbuf, 0);
	}
}
