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
#include <rte_udp.h>
#include <rte_tcp.h>
#include <rte_table_hash.h>
#include <rte_ether.h>
#include <rte_version.h>
#include <rte_byteorder.h>

#include "handle_ipv6_tunnel.h"
#include "tx_pkt.h"
#include "task_init.h"
#include "task_base.h"
#include "dppd_port_cfg.h"
#include "thread_basic.h"
#include "prefetch.h"
#include "quit.h"
#include "hash_utils.h"
#include "etypes.h"
#include "dppd_cksum.h"
#include "stats.h"
#include "defines.h"
#include "display.h"

struct ipv6_tun_dest {
        struct ipv6_addr  dst_addr;
	struct ether_addr dst_mac;
};

typedef enum ipv6_tun_dir_t {
        TUNNEL_DIR_ENCAP = 0,
        TUNNEL_DIR_DECAP = 1,
} ipv6_tun_dir_t;

struct task_ipv6_tun_base {
	struct task_base        base;
	void (*tx_pkt)(struct task_base* tbase);
	struct ether_addr       src_mac;
	uint8_t                 core_nb;
	struct lcore_cfg*       lconf;
	uint64_t                keys[64];
	struct rte_mbuf*        fake_packets[64];
	uint16_t                lookup_port_mask;  // Mask used before looking up the port
	void*                   lookup_table;      // Fast lookup table for bindings
};

struct task_ipv6_decap {
	struct task_ipv6_tun_base   base;
        struct ether_addr           dst_mac;
};

struct task_ipv6_encap {
	struct task_ipv6_tun_base   base;
	uint32_t                    ipaddr;
	struct ipv6_addr            local_endpoint_addr;
	uint8_t                     tunnel_hop_limit;
};

#define MAKE_KEY_FROM_FIELDS(ipv4_addr, port, port_mask) ( ((uint64_t)ipv4_addr << 16) | (port & port_mask) )

void handle_ipv6_decap_bulk(struct task_base* tbase, struct rte_mbuf** rx_mbuf, const uint16_t n_pkts);
void handle_ipv6_encap_bulk(struct task_base* tbase, struct rte_mbuf** rx_mbuf, const uint16_t n_pkts);

static void init_lookup_table(struct task_ipv6_tun_base* ptask, struct dppd_shared* shared)
{
	struct rte_table_hash_key8_ext_params table_hash_params = {
		.n_entries = (shared->ipv6_tun_binding_table->num_binding_entries * 4),
		.n_entries_ext = (shared->ipv6_tun_binding_table->num_binding_entries * 2) >> 1,
		.f_hash = hash_crc32,
		.seed = 0,
		.signature_offset = 8,  // Ignored for dosig tables
		.key_offset = 0,
	};

	int socket_id = rte_lcore_to_socket_id(ptask->lconf->id);

#ifdef IPV6_TUN_SHARED_LOOKUP
	if (shared->ipv6_tun_binding_table->lookup_table != NULL) {
		ptask->lookup_table = shared->ipv6_tun_binding_table->lookup_table;
		mprintf("\tCore %d: IPv6 Tunnel uses shared lookup table\n", ptask->lconf->id);
	}
#endif

	if (ptask->lookup_table == NULL) {
                mprintf("\tCore %d: IPv6 Tunnel allocating lookup table on socket %d\n", ptask->lconf->id, socket_id);
		ptask->lookup_table = rte_table_hash_key8_ext_dosig_ops.
				f_create(&table_hash_params, socket_id, sizeof(struct ipv6_tun_dest));
		DPPD_PANIC(ptask->lookup_table == NULL, "Error creating IPv6 Tunnel lookup table");
#ifdef IPV6_TUN_SHARED_LOOKUP
		shared->ipv6_tun_binding_table->lookup_table = ptask->lookup_table;  // Use for other tasks
#endif

		unsigned int idx;
		for (idx = 0; idx < shared->ipv6_tun_binding_table->num_binding_entries; idx++) {
			int key_found = 0;
			void* entry_in_hash = NULL;
			struct ipv6_tun_dest data;
			struct ipv6_tun_binding_entry* entry = &shared->ipv6_tun_binding_table->entry[idx];
                        uint64_t key = MAKE_KEY_FROM_FIELDS(rte_cpu_to_be_32(entry->public_ipv4), entry->public_port, ptask->lookup_port_mask);
			rte_memcpy(&data.dst_addr, &entry->endpoint_addr, sizeof(struct ipv6_addr));
			rte_memcpy(&data.dst_mac, &entry->next_hop_mac, sizeof(struct ether_addr));

			int ret = rte_table_hash_key8_ext_dosig_ops.f_add(ptask->lookup_table, &key, &data, &key_found, &entry_in_hash);
			DPPD_PANIC(ret, "Error adding entry (%d) to binding lookup table", idx);
			DPPD_PANIC(key_found, "key_found!!! for idx=%d\n", idx);

#ifdef DBG_IPV6_TUN_BINDING
			mprintf("Bind: %x:0x%x (port_mask 0x%x) key=0x%"PRIx64"\n", entry->public_ipv4, entry->public_port, ptask->lookup_port_mask, key);
			mprintf("  -> "IPv6_BYTES_FMT" ("MAC_BYTES_FMT")\n", IPv6_BYTES(entry->endpoint_addr.bytes), MAC_BYTES(entry->next_hop_mac.addr_bytes));
			mprintf("  -> "IPv6_BYTES_FMT" ("MAC_BYTES_FMT")\n", IPv6_BYTES(data.dst_addr.bytes), MAC_BYTES(data.dst_mac.addr_bytes));
			mprintf("  -> entry_in_hash=%p\n", entry_in_hash);
#endif
		}
                mprintf("\tCore %d: IPv6 Tunnel created %d lookup table entries\n", ptask->lconf->id, shared->ipv6_tun_binding_table->num_binding_entries);
	}
}

static void init_task_ipv6_tun_base(struct task_ipv6_tun_base* tun_base, struct task_args* targ)
{
	memcpy(&tun_base->src_mac, &dppd_port_cfg[tun_base->base.tx_params_hw.tx_port_queue[0].port].eth_addr, sizeof(tun_base->src_mac));

	tun_base->lookup_port_mask = targ->lookup_port_mask;  // Mask used before looking up the port
	tun_base->lconf = targ->lconf;
	tun_base->tx_pkt = tx_function(targ);

	init_lookup_table(tun_base, targ->dppd_shared);

	tun_base->base.flush_queues = flush_function(targ);

	for (uint32_t i = 0; i < 64; ++i) {
		tun_base->fake_packets[i] = (struct rte_mbuf*)((uint8_t*)&tun_base->keys[i] - sizeof (struct rte_mbuf));
	}

	mprintf("\tCore %d: IPv6 Tunnel MAC="MAC_BYTES_FMT" port_mask=0x%x\n", tun_base->lconf->id,
	                MAC_BYTES(tun_base->src_mac.addr_bytes), tun_base->lookup_port_mask);
}

static void init_task_ipv6_decap(struct task_base* tbase, struct task_args* targ)
{
	struct task_ipv6_decap* tun_task = (struct task_ipv6_decap*)tbase;
	struct task_ipv6_tun_base* tun_base = (struct task_ipv6_tun_base*)tun_task;

	init_task_ipv6_tun_base(tun_base, targ);

        memcpy(&tun_task->dst_mac, &targ->edaddr, sizeof(tun_task->dst_mac));

#ifndef IPV6_TUN_DECAP_ROUTE
        tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
        tbase->flags |= FLAG_NEVER_FLUSH;
#endif
}

static void init_task_ipv6_encap(struct task_base* tbase, struct task_args* targ)
{
	struct task_ipv6_encap* tun_task = (struct task_ipv6_encap*)tbase;
	struct task_ipv6_tun_base *tun_base = (struct task_ipv6_tun_base*)tun_task;

	init_task_ipv6_tun_base(tun_base, targ);

	rte_memcpy(&tun_task->local_endpoint_addr, &targ->local_ipv6, sizeof(tun_task->local_endpoint_addr));
	tun_task->tunnel_hop_limit = targ->tunnel_hop_limit;

#ifndef IPV6_TUN_ENCAP_ROUTE
        tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
        tbase->flags |= FLAG_NEVER_FLUSH;
#endif
}

struct task_init task_init_ipv6_decap = {
	.mode = IPV6_DECAP,
	.mode_str = "ipv6_decap",
	.init = init_task_ipv6_decap,
	.handle = handle_ipv6_decap_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_IPV6_TUNNEL,
	.size = sizeof(struct task_ipv6_decap)
};

struct task_init task_init_ipv6_encap = {
	.mode = IPV6_ENCAP,
	.mode_str = "ipv6_encap",
	.init = init_task_ipv6_encap,
	.handle = handle_ipv6_encap_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_IPV6_TUNNEL,
	.size = sizeof(struct task_ipv6_encap)
};

void reg_task_ipv6_decap(void)
{
	reg_task(&task_init_ipv6_decap);
}

void reg_task_ipv6_encap(void)
{
	reg_task(&task_init_ipv6_encap);
}


static inline void handle_ipv6_decap(struct task_ipv6_decap* ptask, struct rte_mbuf* rx_mbuf, struct ipv6_tun_dest* tun_dest);
static inline void handle_ipv6_encap(struct task_ipv6_encap* ptask, struct rte_mbuf* rx_mbuf, struct ipv6_tun_dest* tun_dest);


static inline int extract_key_fields( __attribute__((unused)) struct task_ipv6_tun_base* ptask, struct ipv4_hdr* pip4, ipv6_tun_dir_t dir, uint32_t* pAddr, uint16_t* pPort)
{
        *pAddr = (dir == TUNNEL_DIR_DECAP) ? pip4->src_addr : pip4->dst_addr;

        if (pip4->next_proto_id == IPPROTO_UDP) {
                struct udp_hdr* pudp = (struct udp_hdr *)(pip4 + 1);
                *pPort = rte_be_to_cpu_16((dir == TUNNEL_DIR_DECAP) ? pudp->src_port : pudp->dst_port);
        }
        else if (pip4->next_proto_id == IPPROTO_TCP) {
                struct tcp_hdr* ptcp = (struct tcp_hdr *)(pip4 + 1);
                *pPort = rte_be_to_cpu_16((dir == TUNNEL_DIR_DECAP) ? ptcp->src_port : ptcp->dst_port);
        }
        else {
                mprintf("IPv6 Tunnel: IPv4 packet of unexpected type proto_id=0x%x\n", pip4->next_proto_id);
                *pPort = 0xffff;
                return -1;
        }

        return 0;
}

static inline void extract_key(struct task_ipv6_tun_base* ptask, struct ipv4_hdr* pip4, ipv6_tun_dir_t dir, uint64_t* pkey)
{
        uint32_t lookup_addr;
        uint16_t lookup_port;

        if (unlikely( extract_key_fields(ptask, pip4, dir, &lookup_addr, &lookup_port))) {
                mprintf("IPv6 Tunnel: Unable to extract fields from packet\n");
                *pkey = 0xffffffffL;
                return;
        }

        *pkey = MAKE_KEY_FROM_FIELDS(lookup_addr, lookup_port, ptask->lookup_port_mask);
}

static inline struct ipv4_hdr* get_ipv4_decap(struct rte_mbuf *mbuf)
{
        struct ether_hdr* peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
        struct ipv6_hdr* pip6 = (struct ipv6_hdr *)(peth + 1);
        struct ipv4_hdr* pip4 = (struct ipv4_hdr*) (pip6 + 1);  // TODO - Skip Option headers

        return pip4;
}

static inline struct ipv4_hdr* get_ipv4_encap(struct rte_mbuf *mbuf)
{
        struct ether_hdr* peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
        struct ipv4_hdr* pip4 = (struct ipv4_hdr *)(peth + 1);

        return pip4;
}

static inline void extract_key_decap(struct task_ipv6_tun_base* ptask, struct rte_mbuf *mbuf, uint64_t* pkey)
{
        extract_key(ptask, get_ipv4_decap(mbuf), TUNNEL_DIR_DECAP, pkey);
}

static inline void extract_key_decap_bulk(struct task_ipv6_tun_base* ptask, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
        for (uint16_t j = 0; j < n_pkts; ++j) {
                extract_key_decap(ptask, mbufs[j], &ptask->keys[j]);
        }
}

static inline void extract_key_encap(struct task_ipv6_tun_base* ptask, struct rte_mbuf *mbuf, uint64_t* pkey)
{
        extract_key(ptask, get_ipv4_encap(mbuf), TUNNEL_DIR_ENCAP, pkey);
}

static inline void extract_key_encap_bulk(struct task_ipv6_tun_base* ptask, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
        for (uint16_t j = 0; j < n_pkts; ++j) {
                extract_key_encap(ptask, mbufs[j], &ptask->keys[j]);
        }
}


static void handle_error(struct task_ipv6_tun_base* ptask, struct rte_mbuf* mbuf, ipv6_tun_dir_t dir)
{
        uint32_t lookup_addr;
        uint16_t lookup_port;
        uint64_t key;

        struct ipv4_hdr* pip4 = (dir == TUNNEL_DIR_DECAP) ? get_ipv4_decap(mbuf) : get_ipv4_encap(mbuf);
        extract_key_fields(ptask, pip4, dir, &lookup_addr, &lookup_port);
        extract_key(ptask, pip4, dir, &key);

        mprintf("IPv6 Tunnel (%s) lookup failed for "IPv4_BYTES_FMT":%d [key=0x%"PRIx64"]\n",
                        (dir == TUNNEL_DIR_DECAP) ? "decap" : "encap",
                        IPv4_BYTES(((unsigned char*)&lookup_addr)), lookup_port, key);

        rte_pktmbuf_free(mbuf);
        INCR_TX_DROP_COUNT(ptask->base.stats, 1);
}

void handle_ipv6_decap_bulk(struct task_base* tbase, struct rte_mbuf** mbufs, const uint16_t n_pkts)
{
        struct task_ipv6_decap* task = (struct task_ipv6_decap *)tbase;
        uint64_t pkts_mask = RTE_LEN2MASK(n_pkts, uint64_t);
        struct ipv6_tun_dest* entries[64];
        uint64_t lookup_hit_mask;
        uint16_t n_kept = 0;

        prefetch_pkts(mbufs, n_pkts);

        // Lookup to verify packets are valid for their respective tunnels (their sending lwB4)
        extract_key_decap_bulk(&task->base, mbufs, n_pkts);
        rte_table_hash_key8_ext_dosig_ops.f_lookup(task->base.lookup_table, task->base.fake_packets, pkts_mask, &lookup_hit_mask, (void**)entries);

        if (likely(lookup_hit_mask == pkts_mask)) {
                for (uint16_t j = 0; j < n_pkts; ++j) {
                        mbufs[n_kept] = mbufs[j];
                        handle_ipv6_decap(task, mbufs[j], entries[j]);
                        n_kept++;
                }
        }
        else {
                for (uint16_t j = 0; j < n_pkts; ++j) {
                        if (unlikely(!((lookup_hit_mask >> j) & 0x1))) {
                                handle_error(&task->base, mbufs[j], TUNNEL_DIR_DECAP);
                                continue;
                        }
                        mbufs[n_kept] = mbufs[j];
                        handle_ipv6_decap(task, mbufs[j], entries[j]);
                        n_kept++;
                }
        }

#ifdef IPV6_TUN_DECAP_ROUTE
        task->base.tx_pkt(tbase);
#else  /* If we don't have to fan out to multiple queues, then we just send on the same set of mbufs that we received
        * with the exception of dropped packets. Those are just removed, in place, from the mbuf arrays as we go through it. */
        if (likely(n_kept)) {
                task->base.base.tx_pkt_no_buf(&task->base.base, mbufs, n_kept);
        }
        else {
                mprintf("IPv6 Tunnel Encap: discarded all mbufs of the bulk\n");
        }
#endif
}

void handle_ipv6_encap_bulk(struct task_base* tbase, struct rte_mbuf** mbufs, const uint16_t n_pkts)
{
	struct task_ipv6_encap* task = (struct task_ipv6_encap *)tbase;
        uint64_t pkts_mask = RTE_LEN2MASK(n_pkts, uint64_t);
        struct ipv6_tun_dest* entries[64];
        uint64_t lookup_hit_mask;
        uint16_t n_kept = 0;

	prefetch_first(mbufs, n_pkts);

        extract_key_encap_bulk(&task->base, mbufs, n_pkts);
        rte_table_hash_key8_ext_dosig_ops.f_lookup(task->base.lookup_table, task->base.fake_packets, pkts_mask, &lookup_hit_mask, (void**)entries);

        if (likely(lookup_hit_mask == pkts_mask)) {
                for (uint16_t j = 0; j < n_pkts; ++j) {
                        mbufs[n_kept] = mbufs[j];
                        handle_ipv6_encap(task, mbufs[j], entries[j]);
                        n_kept++;
                }
        }
        else {
                for (uint16_t j = 0; j < n_pkts; ++j) {
                        if (unlikely(!((lookup_hit_mask >> j) & 0x1))) {
                                handle_error(&task->base, mbufs[j], TUNNEL_DIR_ENCAP);
                                continue;
                        }
                        mbufs[n_kept] = mbufs[j];
                        handle_ipv6_encap(task, mbufs[j], entries[j]);
                        n_kept++;
                }
        }

#ifdef IPV6_TUN_ENCAP_ROUTE
        task->base.tx_pkt(tbase);
#else  /* If we don't have to fan out to multiple queues, then we just send on the same set of mbufs that we received
        * with the exception of dropped packets. Those are just removed, in place, from the mbuf arrays as we go through it. */
        if (likely(n_kept)) {
                task->base.base.tx_pkt_no_buf(&task->base.base, mbufs, n_kept);
        }
        else {
                mprintf("IPv6 Tunnel Encap: discarded all mbufs of the bulk\n");
        }
#endif
}

static inline void handle_ipv6_decap(struct task_ipv6_decap* ptask, struct rte_mbuf* rx_mbuf, __attribute__((unused)) struct ipv6_tun_dest* tun_dest)
{
	struct ether_hdr* peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);

	if (unlikely(peth->ether_type != ETYPE_IPv6)) {
		mprintf("Received non IPv6 packet on ipv6 tunnel port\n");
		// Drop packet
		INCR_TX_DROP_COUNT(ptask->base.base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	struct ipv6_hdr* pip6 = (struct ipv6_hdr *)(peth + 1);
	int ipv6_hdr_len = sizeof(struct ipv6_hdr);

	// TODO - Skip over any IPv6 Extension Header:
	//      If pip6->next_header is in (0, 43, 44, 50, 51, 60, 135), skip ahead pip->hdr_ext_len
	//      bytes and repeat. Increase ipv6_hdr_len with as much, each time.

	if (unlikely(pip6->proto != IPPROTO_IPV4)) {
		mprintf("Received non IPv4 content within IPv6 tunnel packet\n");
		// Drop packet
		INCR_TX_DROP_COUNT(ptask->base.base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

        // Discard IPv6 encapsulation
        rte_pktmbuf_adj(rx_mbuf, ipv6_hdr_len);
        peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);

        // Restore Ethernet header
        ether_addr_copy(&ptask->base.src_mac, &peth->s_addr);
        ether_addr_copy(&ptask->dst_mac, &peth->d_addr);
        peth->ether_type = ETYPE_IPv4;

#ifdef IPV6_TUN_DECAP_ROUTE
        tx_buf_pkt_single(&ptask->base.base, rx_mbuf, 0);
#endif
}

static inline void handle_ipv6_encap(struct task_ipv6_encap* ptask, struct rte_mbuf* rx_mbuf, __attribute__((unused)) struct ipv6_tun_dest* tun_dest)
{
	DBG_DUMP_PKT(&ptask->base.base.task_debug, "RX", rx_mbuf);

        //mprintf("Found tunnel endpoint:"IPv6_BYTES_FMT" ("MAC_BYTES_FMT")\n", IPv6_BYTES(tun_dest->dst_addr), MAC_BYTES(tun_dest->dst_mac.addr_bytes));

	struct ether_hdr* peth = (struct ether_hdr *)(rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *));
	struct ipv4_hdr* pip4 = (struct ipv4_hdr *)(peth + 1);
	uint16_t ipv4_length = rte_be_to_cpu_16(pip4->total_length);

	if (unlikely((pip4->version_ihl >> 4) != 4)) {
		mprintf("Received non IPv4 packet at ipv6 tunnel input\n");
		// Drop packet
		INCR_TX_DROP_COUNT(ptask->base.base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	if (pip4->time_to_live) {
		pip4->time_to_live--;
	}
	else {
		mprintf("TTL = 0 => Dropping\n");
		INCR_TX_DROP_COUNT(ptask->base.base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}
	pip4->hdr_checksum = 0;

	// Remove padding if any (we don't want to encapsulate garbage at end of IPv4 packet)
	int padding = rte_pktmbuf_pkt_len(rx_mbuf) - (ipv4_length + sizeof(struct ether_hdr));
	if (unlikely(padding > 0)) {
	        rte_pktmbuf_trim(rx_mbuf, padding);
	}

	// Encapsulate
	const int extra_space = sizeof(struct ipv6_hdr);
	peth = (struct ether_hdr *)rte_pktmbuf_prepend(rx_mbuf, extra_space);

	// Ethernet Header
	ether_addr_copy(&ptask->base.src_mac, &peth->s_addr);
	ether_addr_copy(&tun_dest->dst_mac, &peth->d_addr);
	peth->ether_type = ETYPE_IPv6;

	// Set up IPv6 Header
	struct ipv6_hdr* pip6 = (struct ipv6_hdr *)(peth + 1);
	pip6->vtc_flow = rte_cpu_to_be_32(IPv6_VERSION << 28);
	pip6->proto = IPPROTO_IPV4;
	pip6->payload_len = rte_cpu_to_be_16(ipv4_length);
	pip6->hop_limits = ptask->tunnel_hop_limit;
	rte_memcpy(pip6->dst_addr, &tun_dest->dst_addr, sizeof(pip6->dst_addr));
	rte_memcpy(pip6->src_addr, &ptask->local_endpoint_addr, sizeof(pip6->src_addr));

	// We modified the TTL in the IPv4 header, hence have to recompute the IPv4 checksum
#define TUNNEL_L2_LEN (sizeof(struct ether_hdr) + sizeof(struct ipv6_hdr))
#ifdef SOFT_CRC
	dppd_ip_cksum_sw(pip4, 0, &pip4->hdr_checksum);
#elif defined(HARD_CRC)
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	rx_mbuf->l2_l3_len = (TUNNEL_L2_LEN << 9) | (sizeof(struct ipv4_hdr));
#else
	rx_mbuf->pkt.vlan_macip.data = (TUNNEL_L2_LEN << 9) | (sizeof(struct ipv4_hdr));
#endif
	rx_mbuf->ol_flags |= PKT_TX_IP_CKSUM;
#endif

#ifdef IPV6_TUN_ENCAP_ROUTE
	tx_buf_pkt_single(&ptask->base.base, rx_mbuf, 0);
#endif
}
