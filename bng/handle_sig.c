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

#include <string.h>
#include <stdio.h>
#include <rte_version.h>
#include <rte_mbuf.h>
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
#include <rte_port.h>
#endif

#include "handle_sig.h"
#include "bng_pkts.h"
#include "hash_utils.h"
#include "hash_entry_types.h"
#include "task_init.h"
#include "thread_basic.h"

struct task_sig {
	struct task_base base;
};

static void handle_sig_bulk_qinq(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_sig *task = (struct task_sig*)tbase;

	for (uint16_t j = 0; j < n_pkts; ++j) {
		uint64_t key;
		extract_key_cpe(mbufs[j], &key);
		/* key is stored in first 8 bytes, signature is stored
		   in next 4 bytes following key */
		*RTE_MBUF_METADATA_UINT64_PTR(mbufs[j], 0) = key;
		*RTE_MBUF_METADATA_UINT32_PTR(mbufs[j], 8) = hash_crc32(&key, sizeof(key), 0);
	}

	task->base.tx_pkt_no_buf(&task->base, mbufs, n_pkts);
}

static void handle_sig_bulk_gre(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_sig *task = (struct task_sig*)tbase;

	for (uint16_t j = 0; j < n_pkts; ++j) {
		uint64_t key;
		extract_key_core_m(mbufs[j], &key);
		*RTE_MBUF_METADATA_UINT64_PTR(mbufs[j], 0) = key;
		*RTE_MBUF_METADATA_UINT32_PTR(mbufs[j], 8) = hash_crc32(&key, sizeof(key), 0);
	}

	task->base.tx_pkt_no_buf(&task->base, mbufs, n_pkts);
}

static void init_task_sig(struct task_base *tbase, struct task_args *targ)
{
	struct task_l2fwd *task = (struct task_l2fwd *)tbase;

	tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
	tbase->flags |= FLAG_NEVER_FLUSH;
}

struct task_init task_init_sig_qinq = {
	.mode_str = "sig",
        .sub_mode_str = "",
	.init = init_task_sig,
	.handle = handle_sig_bulk_qinq,
	.thread_x = thread_basic,
	.size = sizeof(struct task_sig)
};

struct task_init task_init_sig_gre = {
	.mode_str = "sig",
        .sub_mode_str = "gre",
	.init = init_task_sig,
	.handle = handle_sig_bulk_gre,
	.thread_x = thread_basic,
	.size = sizeof(struct task_sig)
};

void reg_task_sig(void)
{
	reg_task(&task_init_sig_qinq);
	reg_task(&task_init_sig_gre);
}
