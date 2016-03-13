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

#include "thread_none.h"
#include "handle_none.h"
#include "stats.h"
#include "control.h"
#include "defines.h"

int thread_none(struct lcore_cfg *lconf)
{
	struct rte_mbuf *mbuf[MAX_RING_BURST] __rte_cache_aligned;
	struct task_none *task[MAX_TASKS_PER_CORE];
	uint64_t cur_tsc = rte_rdtsc();
	uint64_t term_tsc = cur_tsc + TERM_TIMEOUT;
	uint64_t drain_tsc = cur_tsc + DRAIN_TIMEOUT;
	const uint8_t nb_tasks = lconf->nb_tasks;

	for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
		task[task_id] = (struct task_none *)lconf->task[task_id];
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
		}

		for (uint8_t task_id = 0; task_id < nb_tasks; ++task_id) {
			uint16_t nb_rx = task[task_id]->base.rx_pkt(&task[task_id]->base, mbuf);

			if (likely(nb_rx)) {
				INCR_NBRX(nb_rx);
				INCR_RX_PKT_COUNT(task[task_id]->base.stats, nb_rx);
				handle_none_bulk(&task[task_id]->base, mbuf, nb_rx);
			}
		}
	}

	return 0;
}
