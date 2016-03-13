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

#include <rte_mbuf.h>
#include <rte_ip.h>
#include <rte_byteorder.h>

#include "handle_lb_net.h"
#include "task_base.h"
#include "defines.h"
#include "dppd_args.h"
#include "tx_pkt.h"
#include "display.h"
#include "stats.h"
#include "quit.h"
#include "mpls.h"
#include "etypes.h"
#include "gre.h"
#include "prefetch.h"
#include "thread_basic.h"
#include "qinq.h"
#include "dppd_cfg.h"

void handle_lb_net_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

struct task_lb_net {
	struct task_base                    base;
	void (*tx_pkt)(struct task_base *tbase);
	uint8_t				    bit_mask;
	uint8_t                             nb_worker_threads;
	uint8_t				    worker_byte_offset_ipv4;
	uint8_t				    worker_byte_offset_ipv6;
	uint8_t                             runtime_flags;
};

static void init_task_lb_net(struct task_base *tbase, struct task_args *targ)
{
	struct task_lb_net *task = (struct task_lb_net *)tbase;
	task->runtime_flags = targ->runtime_flags;
	task->worker_byte_offset_ipv6 = targ->flags & TASK_ARG_INET_SIDE ? 39 : 23;
	task->worker_byte_offset_ipv4 = targ->flags & TASK_ARG_INET_SIDE ? 19 : 15;
	task->nb_worker_threads       = targ->nb_worker_threads;
	/* The optimal configuration is when the number of worker threads
	   is a power of 2. In that case, a bit_mask can be used. Setting
	   the bitmask to 0xff disables the "optimal" usage of bitmasks
	   and the actual number of worker threads will be used instead. */
	task->bit_mask = rte_is_power_of_2(targ->nb_worker_threads) ? targ->nb_worker_threads - 1 : 0xff;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);
}

struct task_init task_init_lb_net = {
	.mode = LB_NET,
	.mode_str = "lbnetwork",
	.init = init_task_lb_net,
	.handle = handle_lb_net_bulk,
	.thread_x = thread_basic,
	.size = sizeof(struct task_lb_net)
};

void reg_task_lb_net(void)
{
	reg_task(&task_init_lb_net);
}

static inline uint8_t handle_lb_net(struct task_lb_net *task, struct rte_mbuf *mbuf,uint8_t *dest_wt);

void handle_lb_net_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_lb_net *task = (struct task_lb_net *)tbase;
	uint16_t not_dropped = 0;
	uint8_t dest_wt[MAX_RING_BURST];
	// process packet, i.e. decide if the packet has to be dropped or not and where the packet has to go
	uint16_t j;
	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		mbufs[not_dropped] = mbufs[j];
		not_dropped += handle_lb_net(task, mbufs[j], &dest_wt[not_dropped]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		mbufs[not_dropped] = mbufs[j];
		not_dropped += handle_lb_net(task, mbufs[j], &dest_wt[not_dropped]);
	}
#endif

	/* mbufs now contains the packets that have not been dropped */
	if (likely(not_dropped)) {
		for (j = 0; j < not_dropped; ++j) {
			tx_buf_pkt_single(&task->base, mbufs[j], dest_wt[j]);
		}
		task->tx_pkt(&task->base);
	}
}

static inline uint8_t worker_from_mask(struct task_lb_net *task, uint32_t val)
{
	if (task->bit_mask != 0xff) {
		return val & task->bit_mask;
	}
	else {
		return val % task->nb_worker_threads;
	}
}

static inline uint8_t lb_ip4(struct task_lb_net *task, struct ipv4_hdr *ip, uint8_t *dest_wt, struct rte_mbuf *mbuf)
{
	if (unlikely(ip->version_ihl >> 4 != 4)) {
		mprintf("Expected to receive IPv4 packet but IP version was %d\n",
			ip->version_ihl >> 4);
		rte_pktmbuf_free(mbuf);
		return 0;
	}

	if (ip->next_proto_id == IPPROTO_GRE) {
		struct gre_hdr *pgre = (struct gre_hdr *)(ip + 1);

		if (pgre->bits & GRE_KEY_PRESENT) {
			uint32_t gre_id;
			if (pgre->bits & (GRE_CRC_PRESENT | GRE_ROUTING_PRESENT)) {
				gre_id = *((uint32_t *)((uint8_t *)pgre + 8));
			}
			else {
				gre_id = *((uint32_t *)((uint8_t *)pgre + 4));
			}

			gre_id = rte_be_to_cpu_32(gre_id) & 0xFFFFFFF;
			uint8_t worker = worker_from_mask(task, gre_id);
			*dest_wt = worker + task->nb_worker_threads * IPV4;
			mprintf_verbose("gre_id = %u worker = %u\n", gre_id, worker);
		}
		else {
			mprintf("Key not present\n");
			*dest_wt;
		}
	}
	else if (ip->next_proto_id == IPPROTO_UDP) {
	        //uint8_t worker = worker_from_mask(task, *(uint8_t *)ip + task->worker_byte_offset_ipv4));
		uint8_t worker = worker_from_mask(task, rte_bswap32(ip->dst_addr));
		*dest_wt = worker + task->nb_worker_threads * IPV4;
	}

	return 1;
}

static inline uint8_t lb_ip6(struct task_lb_net *task, struct ipv6_hdr *ip, uint8_t *dest_wt, struct rte_mbuf *mbuf)
{
	if (unlikely((*(uint8_t*)ip) >> 4 != 6)) {
		mprintf("Expected to receive IPv6 packet but IP version was %d\n",
			*(uint8_t*)ip >> 4);
		rte_pktmbuf_free(mbuf);
		return 0;
	}

	uint8_t worker = worker_from_mask(task, *((uint8_t *)ip + task->worker_byte_offset_ipv6));
	*dest_wt = worker + task->nb_worker_threads * IPV6;
	return 1;
}

static inline uint8_t lb_mpls(struct task_lb_net *task, struct ether_hdr *peth, uint8_t *dest_wt, struct rte_mbuf *mbuf)
{
	struct mpls_hdr *mpls = (struct mpls_hdr *)(peth + 1);
	uint32_t mpls_len = 0;
	while (!(mpls->bytes & 0x00010000)) {
		mpls++;
		mpls_len += sizeof(struct mpls_hdr);
	}
	mpls_len += sizeof(struct mpls_hdr);
	struct ipv4_hdr *ip = (struct ipv4_hdr *)(mpls + 1);

	switch (ip->version_ihl >> 4) {
	case 4:
		if (task->runtime_flags & TASK_MPLS_TAGGING) {
			peth = (struct ether_hdr *)rte_pktmbuf_adj(mbuf, mpls_len);
			peth->ether_type = ETYPE_IPv4;
		}
		return lb_ip4(task, ip, dest_wt, mbuf);
	case 6:
		if (task->runtime_flags & TASK_MPLS_TAGGING) {
			peth = (struct ether_hdr *)rte_pktmbuf_adj(mbuf, mpls_len);
			peth->ether_type = ETYPE_IPv6;
		}
		return lb_ip6(task, (struct ipv6_hdr *)ip, dest_wt, mbuf);
	default:
		mprintf("Error Decoding MPLS Packet - neither IPv4 neither IPv6: version %u\n", ip->version_ihl);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return 0;
	}
}

static inline uint8_t lb_qinq(struct task_lb_net *task, struct qinq_hdr *qinq, uint8_t *dest_wt, struct rte_mbuf *mbuf)
{
	if (qinq->cvlan.eth_proto != ETYPE_VLAN) {
		mprintf("Unexpected proto in QinQ = %#04x\n", qinq->cvlan.eth_proto);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return 0;
	}
	uint32_t qinq_tags = rte_bswap16(qinq->cvlan.vlan_tci & 0xFF0F);
	*dest_wt = worker_from_mask(task, qinq_tags);
	return 1;
}

static inline uint8_t handle_lb_net(struct task_lb_net *task, struct rte_mbuf *mbuf, uint8_t *dest_wt)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);

	switch (peth->ether_type) {
	case ETYPE_MPLSU:
		return lb_mpls(task, peth, dest_wt, mbuf);
	case ETYPE_8021ad:
		return lb_qinq(task, (struct qinq_hdr *)peth, dest_wt, mbuf);
	case ETYPE_IPv4:
		return lb_ip4(task, (struct ipv4_hdr *)(peth + 1), dest_wt, mbuf);
	case ETYPE_IPv6:
		return lb_ip6(task, (struct ipv6_hdr *)(peth + 1), dest_wt, mbuf);
	case ETYPE_LLDP:
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return 0;
	default:
		if (peth->ether_type == dppd_cfg.QinQ_tag)
			return lb_qinq(task, (struct qinq_hdr *)peth, dest_wt, mbuf);
		mprintf("Unexpected frame Ether type = %#06x\n", peth->ether_type);
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return 0;
	}

	return 1;
}
