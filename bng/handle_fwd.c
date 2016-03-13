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

#include <rte_lpm.h>

#include "defines.h"
#include "hash_entry_types.h"
#include "mpls.h"
#include "prefetch.h"
#include "handle_fwd.h"
#include "task_base.h"
#include "tx_pkt.h"
#include "stats.h"
#include "task_init.h"
#include "thread_basic.h"
#include "dppd_port_cfg.h"
#include "dppd_cksum.h"
#include "prefetch.h"
#include "dppd_cfg.h"

struct task_fwd {
	struct task_base                base;
	void (*tx_pkt)(struct task_base *tbase);
	struct lcore_cfg                *lconf;
	struct next_hop_struct          *next_hop;
	struct rte_lpm                  *ipv4_lpm;
	struct ether_addr 		edaddr;
	uint8_t                         runtime_flags;
	uint8_t                         core_nb;
	uint64_t                        src_mac[DPPD_MAX_PORTS];
};

/* Forward packets while updating the MAC, TTL and CRC. */
static void handle_fwd_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

static inline void handle_fwd(struct task_fwd *task, struct rte_mbuf *mbuf);

static void handle_fwd_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_fwd *task = (struct task_fwd *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_fwd(task, mbufs[j]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_fwd(task, mbufs[j]);
	}
#endif

	task->tx_pkt(&task->base);
}

/* LPM Routing based on IPv4 routing table */
static inline uint8_t route(struct rte_mbuf *mbuf, struct next_hop_struct *nh, struct rte_lpm* ipv4_lpm, uint32_t dst_ip, uint64_t *src_mac)
{
	uint8_t next_hop_index;
	if (unlikely(rte_lpm_lookup(ipv4_lpm, rte_bswap32(dst_ip), &next_hop_index) != 0)) {
		mprintf("lpm_lookup failed for ip %x: rc = %d\n", dst_ip, -ENOENT);
		return ROUTE_ERR;
	}
	prefetch_nta(&nh[next_hop_index]);

#ifdef MPLS_ROUTING
	struct ether_hdr *peth = mpls_encap(mbuf, nh[next_hop_index].mpls);
#else
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
#endif

	*((uint64_t *)(&peth->d_addr)) = nh[next_hop_index].mac_port_8bytes;
	uint8_t port = nh[next_hop_index].mac_port.out_idx;
	*((uint64_t *)(&peth->s_addr)) = src_mac[next_hop_index];

	return port;
}

static inline void handle_fwd(struct task_fwd *task, struct rte_mbuf *mbuf)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
	struct ipv4_hdr *pip;

	if (peth->ether_type == dppd_cfg.QinQ_tag) {
		struct vlan_hdr *psvlan, *pcvlan;
		/* Skip and Store SVLAN and CVLAN */
		psvlan = (struct vlan_hdr *)(peth + 1);
		if (((psvlan->eth_proto) & 0xFF) != ETYPE_VLAN) {
			uint16_t proto = (psvlan->eth_proto) & 0xFF;
			mprintf("Unexpected proto in QinQ = %#04x\n", proto);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return;
		}

		pcvlan = (struct vlan_hdr *)(psvlan + 1);
		pip = (struct ipv4_hdr *)(pcvlan + 1);
	}
	else {
		pip = (struct ipv4_hdr *)(peth + 1);
	}

	if ((pip->version_ihl >> 4) == 4) {
		if (pip->time_to_live) {
			pip->time_to_live--;
		}
		else {
			mprintf("TTL = 0 => Dropping\n");
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return;
		}
		pip->hdr_checksum = 0;

#ifdef SOFT_CRC
		dppd_ip_cksum_sw(pip, 0, &pip->hdr_checksum);
#elif defined(HARD_CRC)
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
		mbuf->l2_l3_len = (sizeof(struct ether_hdr) << 9) | (sizeof(struct ipv4_hdr));
#else
		mbuf->pkt.vlan_macip.data = (sizeof(struct ether_hdr) << 9) | (sizeof(struct ipv4_hdr));
#endif
		mbuf->ol_flags |= PKT_TX_IP_CKSUM;
#endif

		uint8_t tx_portid = 0;
		if (task->runtime_flags & TASK_ROUTING) {
			if ((tx_portid = route(mbuf, task->next_hop, task->ipv4_lpm, pip->dst_addr, task->src_mac)) == ROUTE_ERR) {
				rte_pktmbuf_free(mbuf);
				INCR_TX_DROP_COUNT(task->base.stats, 1);
				return;
			}
		}
		else {
			ether_addr_copy(&task->edaddr, &peth->d_addr);
		}

		tx_buf_pkt_single(&task->base, mbuf, tx_portid);
	}
	else if ((pip->version_ihl >> 4) == 6) {
		struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(peth + 1);
		ether_addr_copy(&task->edaddr, &peth->d_addr);
		/* Decrement TTL */
		if (pip6->hop_limits) {
			pip6->hop_limits--;
		}
		else {
			mprintf("TTL = 0 => Dropping\n");
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return;
		}
		tx_buf_pkt_single(&task->base, mbuf, 0);
		return;
	}
	else {
		mprintf("Core %u: Error in Fwd mode: version=%u, protocol = %u\n", task->lconf->id, pip->version_ihl, pip->next_proto_id);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return;
	}
}

static void init_task_fwd(struct task_base *tbase, struct task_args *targ)
{
	struct task_fwd *task = (struct task_fwd *)tbase;
	task->edaddr = targ->edaddr;
	task->runtime_flags = targ->runtime_flags;
	task->lconf = targ->lconf;
	task->next_hop = targ->dppd_shared->next_hop;
	task->ipv4_lpm = targ->dppd_shared->ipv4_lpm;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);

	for (uint32_t i = 0; i < targ->nb_txrings || i < targ->nb_txports; ++i) {
#ifdef MPLS_ROUTING
		task->src_mac[i] = (*(uint64_t*)&dppd_port_cfg[i].eth_addr) | ((uint64_t)ETYPE_MPLSU << 48);
#else
		task->src_mac[i] = (*(uint64_t*)&dppd_port_cfg[i].eth_addr) | ((uint64_t)ETYPE_IPv4 << 48);
#endif
	}
}

struct task_init task_init_fwd = {
	.mode_str = "fwd",
	.init = init_task_fwd,
	.handle = handle_fwd_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_NEXT_HOP | REQ_LPM4,
	.flag_features = TASK_ROUTING,
	.size = sizeof(struct task_fwd)
};

void reg_task_fwd(void)
{
	reg_task(&task_init_fwd);
}
