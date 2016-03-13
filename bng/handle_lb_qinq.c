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

#include "task_base.h"
#include "handle_lb_qinq.h"
#include "tx_pkt.h"
#include "etypes.h"
#include "display.h"
#include "stats.h"
#include "qinq.h"
#include "quit.h"
#include "prefetch.h"
#include "defines.h"
#include "thread_basic.h"
#include "dppd_cfg.h"

/* Load balancing based on one byte, figures out what type of packet
   is passed and depending on the type, pass the packet to the correct
   worker thread. If an unsupported packet type is used, the packet is
   simply dropped. This Load balancer can only handling QinQ packets
   (i.e. packets comming from the vCPE). */
void handle_lb_qinq_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

struct task_lb_qinq {
	struct task_base        base;
	void (*tx_pkt)(struct task_base *tbase);
	uint8_t                *worker_thread_table;
	uint8_t			bit_mask;
	uint8_t                 nb_worker_threads;
};

static void init_task_lb_qinq(struct task_base *tbase, struct task_args *targ)
{
	struct task_lb_qinq *task = (struct task_lb_qinq *)tbase;
	task->nb_worker_threads = targ->nb_worker_threads;
	task->bit_mask = rte_is_power_of_2(targ->nb_worker_threads) ? targ->nb_worker_threads - 1 : 0xff;
	task->worker_thread_table = targ->dppd_shared->worker_thread_table;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);
}

struct task_init task_init_lb_qinq = {
	.mode = LB_QINQ,
	.mode_str = "lbqinq",
	.init = init_task_lb_qinq,
	.handle = handle_lb_qinq_bulk,
	.thread_x = thread_basic,
	.flag_req_data = REQ_WT_TABLE | REQ_GRE_TABLE,
	.size = sizeof(struct task_lb_qinq)
};

void reg_task_lb_qinq(void)
{
	reg_task(&task_init_lb_qinq);
}

static inline uint8_t handle_lb_qinq(struct task_lb_qinq *task, struct rte_mbuf *mbuf, uint8_t *dest_wt);

void handle_lb_qinq_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_lb_qinq *task = (struct task_lb_qinq *)tbase;
	uint16_t j;
	uint8_t dest_wt[MAX_RING_BURST];
	uint16_t not_dropped = 0;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		mbufs[not_dropped] = mbufs[j];
		not_dropped += handle_lb_qinq(task, mbufs[j], &dest_wt[not_dropped]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		mbufs[not_dropped] = mbufs[j];
		not_dropped += handle_lb_qinq(task, mbufs[j], &dest_wt[not_dropped]);
	}
#endif

	if (likely(not_dropped)) {
		for (j = 0; j < not_dropped; ++j) {
			tx_buf_pkt_single(&task->base, mbufs[j], dest_wt[j]);
		}
		task->tx_pkt(&task->base);
	}
}

struct qinq_packet {
	struct qinq_hdr qinq_hdr;
	union {
		struct ipv4_hdr ipv4_hdr;
		struct ipv6_hdr ipv6_hdr;
	};
} __attribute__((packed));

struct ether_packet {
	struct ether_hdr ether_hdr;
	union {
		struct ipv4_hdr ipv4_hdr;
		struct ipv6_hdr ipv6_hdr;
	};
} __attribute__((packed));


struct cpe_packet {
	union {
		struct qinq_packet  qp;
		struct ether_packet ep;
	};
};

static inline uint8_t handle_lb_qinq(struct task_lb_qinq *task, struct rte_mbuf *mbuf, uint8_t *dest_wt)
{
	struct cpe_packet *packet = rte_pktmbuf_mtod(mbuf, struct cpe_packet*);
	if (packet->ep.ether_hdr.ether_type == ETYPE_IPv4) {
		if (unlikely((packet->ep.ipv4_hdr.version_ihl >> 4) != 4)) {
			mprintf("Invalid Version %u for ETYPE_IPv4\n", packet->ep.ipv4_hdr.version_ihl);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return 0;
		}
		/* use 24 bits from the IP, clients are from the 10.0.0.0/8 network */
		const uint32_t tmp = rte_bswap32(packet->ep.ipv4_hdr.src_addr) & 0x00FFFFFF;
		const uint32_t svlan = rte_bswap16(tmp >> 12);
		const uint32_t cvlan = rte_bswap16(tmp & 0x0FFF);
		prefetch_nta(&task->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)]);
		uint8_t worker = task->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)];
		*dest_wt = worker + IPV4 * task->nb_worker_threads;
		return 1;
	}
	else if (unlikely(packet->qp.qinq_hdr.svlan.eth_proto != dppd_cfg.QinQ_tag)) {
		/* might receive LLDP from the L2 switch... */
		if (packet->qp.qinq_hdr.svlan.eth_proto != ETYPE_LLDP) {
			mprintf("Core LB: Error getting non Q in Q packets in LB for Q in Q mode\n");
			DBG_DUMP_PKT(&task->base.task_debug, "DROP", mbuf);
		}
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return 0;
	}


	uint8_t worker = 0;
	uint8_t proto = 0xFF;
	switch (packet->qp.qinq_hdr.ether_type) {
	case ETYPE_IPv4: {
		if (unlikely((packet->qp.ipv4_hdr.version_ihl >> 4) != 4)) {
			mprintf("Invalid Version %u for ETYPE_IPv4\n", packet->qp.ipv4_hdr.version_ihl);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return 0;
		}
		uint16_t svlan = packet->qp.qinq_hdr.svlan.vlan_tci;
		uint16_t cvlan = packet->qp.qinq_hdr.cvlan.vlan_tci;
		prefetch_nta(&task->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)]);
		worker = task->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)];

		const size_t pos = offsetof(struct cpe_packet, qp.qinq_hdr.cvlan.vlan_tci);
		mprintf_verbose("qinq = %u, worker = %u, pos = %lu\n", rte_be_to_cpu_16(cvlan), worker, pos);
		proto = IPV4;
		break;
	}
	case ETYPE_IPv6: {
		if (unlikely((packet->qp.ipv4_hdr.version_ihl >> 4) != 6)) {
			mprintf("Invalid Version %u for ETYPE_IPv6\n", packet->qp.ipv4_hdr.version_ihl);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			rte_pktmbuf_free(mbuf);
			return 0;
		}
		/* Use IP Destination when IPV6 QinQ */
		if (task->bit_mask != 0xff) {
			worker = ((uint8_t *)packet)[61] & task->bit_mask;
		}
		else {
			worker = ((uint8_t *)packet)[61] % task->nb_worker_threads;
		}
		proto = IPV6;
		break;
	}
	case ETYPE_ARP: {
		uint16_t svlan = packet->qp.qinq_hdr.svlan.vlan_tci;
		uint16_t cvlan = packet->qp.qinq_hdr.cvlan.vlan_tci;
		prefetch_nta(&task->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)]);
		worker = task->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)];
		proto = ARP;
		break;
	}
	default:
		DPPD_PANIC(1, "Error in ETYPE_8021ad: ether_type = %#06x\n", packet->qp.qinq_hdr.ether_type);
	}

	*dest_wt = worker + proto * task->nb_worker_threads;
	return 1;
}
