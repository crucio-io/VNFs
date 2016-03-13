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

#include "handle_l2fwd.h"
#include "task_init.h"
#include "task_base.h"
#include "dppd_port_cfg.h"
#include "thread_basic.h"

struct task_l2fwd {
	struct task_base base;
	uint8_t src_dst_mac[12];
};

static void handle_l2fwd_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_l2fwd *task = (struct task_l2fwd *)tbase;
	struct ether_hdr *hdr;

	for (uint16_t j = 0; j < n_pkts; ++j) {
		hdr = rte_pktmbuf_mtod(mbufs[j], struct ether_hdr *);
                rte_memcpy(hdr, task->src_dst_mac, sizeof(task->src_dst_mac));
	}
	task->base.tx_pkt_no_buf(&task->base, mbufs, n_pkts);
}

static void init_task_l2fwd(struct task_base *tbase, struct task_args *targ)
{
	struct task_l2fwd *task = (struct task_l2fwd *)tbase;
	struct ether_addr *src_addr, *dst_addr;

	src_addr = &targ->edaddr;
	dst_addr = &dppd_port_cfg[task->base.tx_params_hw.tx_port_queue[0].port].eth_addr;
	memcpy(&task->src_dst_mac[0], src_addr, sizeof(*src_addr));
	memcpy(&task->src_dst_mac[6], dst_addr, sizeof(*dst_addr));
	tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
	tbase->flags |= FLAG_NEVER_FLUSH;
}

struct task_init task_init_l2fwd = {
	.mode_str = "l2fwd",
	.init = init_task_l2fwd,
	.handle = handle_l2fwd_bulk,
	.thread_x = thread_basic,
	.size = sizeof(struct task_l2fwd)
};

void reg_task_l2fwd(void)
{
	reg_task(&task_init_l2fwd);
}
