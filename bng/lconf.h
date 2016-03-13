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

#ifndef _LCONF_H_
#define _LCONF_H_

#include "task_init.h"

struct task_base;
struct rte_table;

struct lcore_cfg {
	struct task_base	*task[MAX_TASKS_PER_CORE];
	uint8_t			nb_tasks;		// Used by ALL

#ifdef DPPD_CMD_STAT_RX
	rte_atomic64_t		nb_rx_buckets[MAX_RING_BURST + 1];
#endif

	void (*period_func)(void* data);
	void*                   period_data;
	uint64_t                period_timeout;       // call periodic_func after periodic_timeout cycles

	uint64_t                ctrl_timeout;
	void (*ctrl_func_m[MAX_TASKS_PER_CORE])(struct task_base *tbase, void **data, uint16_t n_msgs);
	struct rte_ring         *ctrl_rings_m[MAX_TASKS_PER_CORE];

	void (*ctrl_func_p[MAX_TASKS_PER_CORE])(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts);
	struct rte_ring         *ctrl_rings_p[MAX_TASKS_PER_CORE];

	// Following variables are not accessed in main loop
	uint32_t		flags;			// PCFG_* flags below
	uint8_t			active_task;
	uint8_t			id;
	char			name[MAX_NAME_SIZE];
	struct task_args        targs[MAX_TASKS_PER_CORE];
	int (*thread_x)(struct lcore_cfg* lconf);
} __rte_cache_aligned;


/* flags for lcore_cfg */
#define PCFG_TERMINATE		0x40000000 /* thread terminate flag */

#endif /* _LCONF_H_ */
