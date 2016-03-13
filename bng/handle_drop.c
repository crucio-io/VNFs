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

#include "handle_drop.h"
#include "task_init.h"
#include "task_base.h"
#include "thread_basic.h"
#include "stats.h"

void handle_drop_bulk(__attribute__((unused)) struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);

void handle_drop_bulk(__attribute__((unused)) struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	for (uint16_t j = 0; j < n_pkts; ++j) {
		rte_pktmbuf_free(mbufs[j]);
	}
	INCR_TX_DROP_COUNT(tbase->stats, n_pkts);
}

static void init_task_drop(struct task_base *tbase, __attribute__((unused)) struct task_args *targ)
{
	tbase->flags |= FLAG_NEVER_FLUSH;
}

struct task_init task_init_drop = {
	.mode = DROP,
	.mode_str = "drop",
	.init = init_task_drop,
	.handle = handle_drop_bulk,
	.thread_x = thread_basic,
	.size = sizeof(struct task_base)
};

void reg_task_drop(void)
{
	reg_task(&task_init_drop);
}
