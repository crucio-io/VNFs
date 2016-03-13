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

#include "dppd_args.h"
#include "handle_unmpls.h"
#include "tx_pkt.h"
#include "mpls.h"
#include "stats.h"
#include "defines.h"
#include "prefetch.h"
#include "thread_basic.h"

struct task_unmpls {
	struct task_base        base;
	void (*tx_pkt)(struct task_base *tbase);
	struct ether_addr 	edaddr;
	uint8_t                 core_nb;
	struct lcore_cfg        *lconf;
};

void handle_unmpls_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

static void init_task_unmpls(struct task_base *tbase, struct task_args *targ)
{
	struct task_unmpls *task = (struct task_unmpls *)tbase;
	task->edaddr = targ->edaddr;
	task->tx_pkt = tx_function(targ);
	task->lconf = targ->lconf;
	tbase->flush_queues = flush_function(targ);
}

struct task_init task_init_unmpls = {
	.mode_str = "unmpls",
	.init = init_task_unmpls,
	.handle = handle_unmpls_bulk,
	.thread_x = thread_basic,
	.size = sizeof(struct task_unmpls)
};

void reg_task_unmpls(void)
{
	reg_task(&task_init_unmpls);
}

static inline void handle_unmpls(struct task_unmpls *task, struct rte_mbuf *mbuf);

void handle_unmpls_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_unmpls *task = (struct task_unmpls *)tbase;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		handle_unmpls(task, mbufs[j]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		handle_unmpls(task, mbufs[j]);
	}
#endif
	task->tx_pkt(&task->base);
}

static inline void handle_unmpls(struct task_unmpls *task, struct rte_mbuf *mbuf)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);

	switch (peth->ether_type) {
	case ETYPE_MPLSU:
		/* MPLS Decapsulation */
		mpls_decap(mbuf);
		peth = rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
		ether_addr_copy(&task->edaddr, &peth->d_addr);
		break;
	case ETYPE_LLDP:
		INCR_TX_DROP_COUNT(task->base.stats, 1);
		rte_pktmbuf_free(mbuf);
		return;
	case ETYPE_IPv6:
		tx_buf_pkt_single(&task->base, mbuf, 0);
		break;
	case ETYPE_IPv4:
		tx_buf_pkt_single(&task->base, mbuf, 0);
		break;
	default:
		mprintf("Core %u Error Removing MPLS: ether_type = %#06x\n", task->lconf->id, peth->ether_type);
		rte_pktmbuf_free(mbuf);
	}
}
