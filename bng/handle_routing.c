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
#include <rte_cycles.h>

#include "quit.h"
#include "display.h"
#include "handle_routing.h"
#include "tx_pkt.h"
#include "stats.h"
#include "gre.h"
#include "thread_generic.h"
#include "dppd_port_cfg.h"
#include "etypes.h"
#include "prefetch.h"
#include "hash_entry_types.h"
#include "mpls.h"
#include "qinq.h"
#include "dppd_cfg.h"

struct task_routing {
	struct task_base                base;
	void (*tx_pkt)(struct task_base *tbase);
	uint8_t                         runtime_flags;
	struct lcore_cfg                *lconf;
	struct next_hop_struct          *next_hop;
	struct rte_lpm                  *ipv4_lpm;
	uint32_t                        marking[4];
	uint64_t                        src_mac[DPPD_MAX_PORTS];
};

void handle_routing_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

static void routing_update(struct task_base *tbase, void **data, uint16_t n_msgs)
{
	struct task_routing *task = (struct task_routing *)tbase;
	struct route_msg *msg;

	for (uint16_t i = 0; i < n_msgs; ++i) {
		msg = (struct route_msg *)data[i];

		if (rte_lpm_add(task->ipv4_lpm, rte_bswap32(msg->ip), msg->prefix, msg->nh)) {
			mprintf("Error adding route: %u.%u.%u.%u/%u\n",
				msg->ip_bytes[0], msg->ip_bytes[1], msg->ip_bytes[2],
				msg->ip_bytes[3], msg->prefix);
		}
	}
}

static void init_task_routing(struct task_base *tbase, struct task_args *targ)
{
	struct task_routing *task = (struct task_routing *)tbase;
	task->lconf = targ->lconf;
	task->tx_pkt = tx_function(targ);
	task->runtime_flags = targ->runtime_flags;
	task->next_hop = targ->dppd_shared->next_hop;
	char name[60];
	snprintf(name, sizeof(name), "IPv4_lpm_s%u_%u_%u", rte_lcore_to_socket_id(targ->lconf->id), targ->lconf->id, targ->task);
	if (targ->flags & TASK_ARG_LOCAL_LPM) {
		task->ipv4_lpm = rte_lpm_create(name, rte_lcore_to_socket_id(targ->lconf->id), targ->lpm_size, RTE_LPM_HEAP);
		DPPD_PANIC(!task->ipv4_lpm, "Failed to create LPM on core %u task %u\n", targ->lconf->id, targ->task);
	}
	else {
		task->ipv4_lpm = targ->dppd_shared->ipv4_lpm;
	}

	tbase->flush_queues = flush_function(targ);

	for (uint32_t i = 0; i < targ->nb_txrings || i < targ->nb_txports; ++i) {
		if (task->runtime_flags & TASK_MPLS_TAGGING) {
			task->src_mac[i] = (*(uint64_t*)&dppd_port_cfg[i].eth_addr) | ((uint64_t)ETYPE_MPLSU << (64 - 16));
		}
		else {
			task->src_mac[i] = (*(uint64_t*)&dppd_port_cfg[i].eth_addr) | ((uint64_t)ETYPE_IPv4 << (64 - 16));
		}
	}

	for (uint32_t i = 0; i < 4; ++i) {
		task->marking[i] = rte_bswap32(targ->marking[i] << 9);
	}

	targ->lconf->ctrl_func_m[targ->task] = routing_update;
	targ->lconf->ctrl_timeout = rte_get_tsc_hz()/50;
}

struct task_init task_init_routing = {
	.mode = ROUTING,
	.mode_str = "routing",
	.init = init_task_routing,
	.handle = handle_routing_bulk,
	.thread_x = thread_generic,
	.flag_req_data = REQ_NEXT_HOP | REQ_LPM4,
	.flag_features = TASK_ROUTING,
	.size = sizeof(struct task_routing)
};

void reg_task_routing(void)
{
	reg_task(&task_init_routing);
}

static inline void handle_routing(struct task_routing *task, struct rte_mbuf *mbuf);

void handle_routing_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_routing *task = (struct task_routing *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_routing(task, mbufs[j]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_routing(task, mbufs[j]);
	}
#endif
	task->tx_pkt(&task->base);
}

static void set_l2(struct task_routing *task, struct rte_mbuf *mbuf, uint8_t nh_idx)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
	*((uint64_t *)(&peth->d_addr)) = task->next_hop[nh_idx].mac_port_8bytes;
	*((uint64_t *)(&peth->s_addr)) = task->src_mac[task->next_hop[nh_idx].mac_port.out_idx];
}

static void set_l2_mpls(struct task_routing *task, struct rte_mbuf *mbuf, uint8_t nh_idx)
{
	struct ether_hdr *peth = (struct ether_hdr *)rte_pktmbuf_prepend(mbuf, sizeof(struct mpls_hdr));

#ifdef HARD_CRC
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	mbuf->l2_l3_len += (4 << 9);
#else
	mbuf->pkt.vlan_macip.data += (4 << 9);
#endif
#endif

	*((uint64_t *)(&peth->d_addr)) = task->next_hop[nh_idx].mac_port_8bytes;
	*((uint64_t *)(&peth->s_addr)) = task->src_mac[task->next_hop[nh_idx].mac_port.out_idx];
	peth->ether_type = ETYPE_MPLSU;
	struct mpls_hdr *mpls = (struct mpls_hdr *)(peth + 1);

	if (task->runtime_flags & TASK_MARK) {
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	struct rte_sched_port_hierarchy *sched =
		(struct rte_sched_port_hierarchy *) &mbuf->hash.sched;
#else
	struct rte_sched_port_hierarchy *sched =
		(struct rte_sched_port_hierarchy *) &mbuf->pkt.hash.sched;
#endif

                *(uint32_t *)mpls = task->next_hop[nh_idx].mpls | task->marking[sched->color] | 0x00010000; // Set BoS to 1
	}
	else {
		*(uint32_t *)mpls = task->next_hop[nh_idx].mpls | 0x00010000; // Set BoS to 1
	}
}

static void route_ipv4(struct task_routing *task, uint8_t *beg, uint32_t ip_offset, struct rte_mbuf *mbuf)
{
	struct ipv4_hdr *ip = (struct ipv4_hdr*)(beg + ip_offset);
	struct ether_hdr *peth_out;
	uint8_t tx_port, next_hop_index;
	uint32_t dst_ip;

	if (unlikely(ip->version_ihl >> 4 != 4)) {
                mprintf("Offset: %d\n", ip_offset);
		mprintf("Expected to receive IPv4 packet but IP version was %d\n",
			ip->version_ihl >> 4);
		rte_pktmbuf_free(mbuf);
		return;
	}

	switch(ip->next_proto_id) {
	case IPPROTO_GRE: {
		struct gre_hdr *pgre = (struct gre_hdr *)(ip + 1);
		dst_ip = ((struct ipv4_hdr *)(pgre + 1))->dst_addr;
		break;
	}
	case IPPROTO_UDP:
		dst_ip = ip->dst_addr;
		break;
	default:
		/* Routing for other protocols is not implemented */
		rte_pktmbuf_free(mbuf);
		return;
	}

	if (unlikely(rte_lpm_lookup(task->ipv4_lpm, rte_bswap32(dst_ip), &next_hop_index) != 0)) {
		uint8_t* dst_ipp = (uint8_t*)&dst_ip;
		mprintf("lpm_lookup failed for ip %d.%d.%d.%d: rc = %d\n",
			dst_ipp[0], dst_ipp[1], dst_ipp[2], dst_ipp[3], -ENOENT);
		rte_pktmbuf_free(mbuf);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		return;
	}

	tx_port = task->next_hop[next_hop_index].mac_port.out_idx;
	if (task->runtime_flags & TASK_MPLS_TAGGING) {
	        uint16_t padlen = rte_pktmbuf_pkt_len(mbuf) - rte_be_to_cpu_16(ip->total_length) - ip_offset;
		if (padlen) {
			rte_pktmbuf_trim(mbuf, padlen);
                }

                set_l2_mpls(task, mbuf, next_hop_index);
        }
	else {
		set_l2(task, mbuf, next_hop_index);
        }
	tx_buf_pkt_single(&task->base, mbuf, tx_port);
}

static inline void handle_routing(struct task_routing *task, struct rte_mbuf *mbuf)
{
	struct qinq_hdr *qinq;
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);

	switch (peth->ether_type) {
	case ETYPE_8021ad: {
		struct qinq_hdr *qinq = (struct qinq_hdr *)peth;
		if ((qinq->cvlan.eth_proto != ETYPE_VLAN)) {
			mprintf("Unexpected proto in QinQ = %#04x\n", qinq->cvlan.eth_proto);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return;
		}

		route_ipv4(task, (uint8_t*)qinq, sizeof(*qinq), mbuf);
		return;
	}
	case ETYPE_IPv4:
		route_ipv4(task, (uint8_t*)peth, sizeof(*peth), mbuf);
		return;
	case ETYPE_MPLSU: {
		/* skip MPLS headers if any for routing */
		struct mpls_hdr *mpls = (struct mpls_hdr *)(peth + 1);
		uint32_t count = sizeof(struct ether_hdr);
		while (!(mpls->bytes & 0x00010000)) {
			mpls++;
			count += sizeof(struct mpls_hdr);
		}
		count += sizeof(struct mpls_hdr);

		route_ipv4(task, (uint8_t*)peth, count, mbuf);
		return;
	}
	default:
		if (peth->ether_type == dppd_cfg.QinQ_tag) {
			struct qinq_hdr *qinq = (struct qinq_hdr *)peth;
			if ((qinq->cvlan.eth_proto != ETYPE_VLAN)) {
				mprintf("Unexpected proto in QinQ = %#04x\n", qinq->cvlan.eth_proto);
				INCR_TX_DROP_COUNT(task->base.stats, 1);
				rte_pktmbuf_free(mbuf);
				return;
			}
	
			route_ipv4(task, (uint8_t*)qinq, sizeof(*qinq), mbuf);
			return;
		}
		mprintf("Error routing packet: ether_type %#06x is unknown\n", peth->ether_type);
		rte_pktmbuf_free(mbuf);
		return;
	}
}
