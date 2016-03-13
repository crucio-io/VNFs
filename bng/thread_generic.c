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
#include <rte_table_hash.h>

#include "thread_generic.h"
#include "thread_basic.h"
#include "display.h"
#include "stats.h"
#include "tx_pkt.h"
#include "control.h"
#include "hash_entry_types.h"
#include "defines.h"
#include "hash_utils.h"


int thread_generic(struct lcore_cfg *lconf)
{
	struct rte_mbuf *mbufs[MAX_RING_BURST] __rte_cache_aligned;
	struct task_base *tasks[MAX_TASKS_PER_CORE];
	uint64_t cur_tsc = rte_rdtsc();
	uint64_t term_tsc = cur_tsc + TERM_TIMEOUT;
	uint64_t drain_tsc = cur_tsc + DRAIN_TIMEOUT;
	uint64_t period_tsc = UINT64_MAX;
	uint64_t ctrl_tsc = UINT64_MAX;
	const uint8_t nb_tasks = lconf->nb_tasks;
	void *msgs[MAX_RING_BURST];

	for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
		tasks[task_id] = lconf->task[task_id];
	}

	if (lconf->period_func) {
		period_tsc = cur_tsc + lconf->period_timeout;
	}

	for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
		if (lconf->ctrl_func_m[task_id]) {
			ctrl_tsc = cur_tsc + lconf->ctrl_timeout;
			break;
		}
		if (lconf->ctrl_func_p[task_id]) {
			ctrl_tsc = cur_tsc + lconf->ctrl_timeout;
			break;
		}
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

			if (cur_tsc > period_tsc) {
				lconf->period_func(lconf->period_data);
				period_tsc += lconf->period_timeout;
			}

			flush_all(tasks, nb_tasks);
		}

		if (cur_tsc > ctrl_tsc) {
			ctrl_tsc = ctrl_tsc + lconf->ctrl_timeout;
			for (uint8_t task_id = 0; task_id < nb_tasks; ++task_id) {
				if (lconf->ctrl_rings_m[task_id] && lconf->ctrl_func_m[task_id]) {
					uint16_t n_msgs = rte_ring_sc_dequeue_burst(lconf->ctrl_rings_m[task_id], msgs, MAX_RING_BURST);
					if (n_msgs) {
						lconf->ctrl_func_m[task_id](tasks[task_id], msgs, n_msgs);
					}
				}
				if (lconf->ctrl_rings_p[task_id] && lconf->ctrl_func_p[task_id]) {
					uint16_t n_msgs = rte_ring_sc_dequeue_burst(lconf->ctrl_rings_p[task_id], msgs, MAX_RING_BURST);
					if (n_msgs) {
						lconf->ctrl_func_p[task_id](tasks[task_id], (struct rte_mbuf **)msgs, n_msgs);
					}
				}
			}
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
