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

#include <rte_cycles.h>

#include "thread_basic.h"
#include "stats.h"
#include "control.h"
#include "defines.h"

/* This function is only run on low load (when no bulk was sent within
   last drain_timeout (16kpps if DRAIN_TIMEOUT = 2 ms) */
void flush_all(struct task_base **tasks, const uint8_t nb_tasks)
{
	for (uint8_t task_id = 0; task_id < nb_tasks; ++task_id) {
		/* Do not flush packets if packets were transmitted in
		   last drain_timeout (or for tasks that simply don't
		   flush packets */
		if (!(tasks[task_id]->flags & FLAG_TX_FLUSH) || (tasks[task_id]->flags & FLAG_NEVER_FLUSH)) {
			tasks[task_id]->flags |= FLAG_TX_FLUSH;
			continue;
		}
		tasks[task_id]->flush_queues(tasks[task_id]);
	}
}

int thread_basic(struct lcore_cfg *lconf)
{
	struct rte_mbuf *mbufs[MAX_RING_BURST] __rte_cache_aligned;
	struct task_base *tasks[MAX_TASKS_PER_CORE];
	uint64_t cur_tsc = rte_rdtsc();
	uint64_t term_tsc = cur_tsc + TERM_TIMEOUT;
	uint64_t drain_tsc = cur_tsc + DRAIN_TIMEOUT;
	const uint8_t nb_tasks = lconf->nb_tasks;

	for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
		tasks[task_id] = lconf->task[task_id];
	}

	for (;;) {
		cur_tsc = rte_rdtsc();
		if (cur_tsc > drain_tsc) {
			drain_tsc = cur_tsc + DRAIN_TIMEOUT;
			FLUSH_STATS(lconf);

			if (cur_tsc > term_tsc) {
				term_tsc = cur_tsc + TERM_TIMEOUT;
				if (is_terminated(lconf)) {
					break;
				}
			}

			flush_all(tasks, nb_tasks);
		}

		for (uint8_t task_id = 0; task_id < nb_tasks; ++task_id) {
			uint16_t nb_rx = tasks[task_id]->rx_pkt(tasks[task_id], mbufs);

			if (likely(nb_rx)) {
				INCR_NBRX(nb_rx);
				INCR_RX_PKT_COUNT(tasks[task_id]->stats, nb_rx);
				tasks[task_id]->handle_bulk(tasks[task_id], mbufs, nb_rx);
			}
		}
	}
	return 0;
}
