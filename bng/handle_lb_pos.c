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

#include "handle_lb_pos.h"
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

void handle_lb_pos_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

struct task_lb_pos {
	struct task_base                    base;
	void (*tx_pkt)(struct task_base *tbase);
	uint16_t                            byte_offset;
	uint8_t                             nb_worker_threads;
};

static void init_task_lb_pos(struct task_base *tbase, struct task_args *targ)
{
	struct task_lb_pos *task = (struct task_lb_pos *)tbase;
	task->nb_worker_threads = targ->nb_worker_threads;
	task->byte_offset = targ->byte_offset;
	task->tx_pkt = tx_function(targ);
	tbase->flush_queues = flush_function(targ);
}

struct task_init task_init_lb_pos = {
	.mode_str = "lbpos",
	.init = init_task_lb_pos,
	.handle = handle_lb_pos_bulk,
	.thread_x = thread_basic,
	.size = sizeof(struct task_lb_pos)
};

void reg_task_lb_pos(void)
{
	reg_task(&task_init_lb_pos);
}

void handle_lb_pos_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_lb_pos *task = (struct task_lb_pos *)tbase;
	uint16_t offset = task->byte_offset;
	uint16_t j;

	prefetch_first(mbufs, n_pkts);

	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		uint8_t* pkt = rte_pktmbuf_mtod(mbufs[j], uint8_t*);
		uint16_t wt = pkt[offset] % task->nb_worker_threads;
		tx_buf_pkt_single(&task->base, mbufs[j], wt);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		uint8_t* pkt = rte_pktmbuf_mtod(mbufs[j], uint8_t*);
		uint16_t wt = pkt[offset] % task->nb_worker_threads;
		tx_buf_pkt_single(&task->base, mbufs[j], wt);
	}
#endif

	task->tx_pkt(&task->base);
}
