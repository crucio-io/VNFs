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

#include "handle_qos.h"
#include "display.h"
#include "quit.h"
#include "thread_qos.h"

static void init_task_qos(struct task_base *tbase, struct task_args *targ)
{
	struct task_qos *task = (struct task_qos *)tbase;
	task->sched_port = targ->sched_port;
	task->user_table = targ->dppd_shared->user_table;
	task->runtime_flags = targ->runtime_flags;
	tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
	tbase->flags |= FLAG_NEVER_FLUSH;
	task->dscp = targ->dppd_shared->dscp;

	DPPD_PANIC(targ->lconf->nb_tasks != 1, "Only support one task when doing QoS");
}

static void early_init(struct task_args *targ)
{
	char name[64];
	snprintf(name, sizeof(name), "qos_sched_port_%u_%u", targ->lconf->id, 0);

	targ->qos_conf.port_params.name = name;
	targ->qos_conf.port_params.socket = rte_lcore_to_socket_id(targ->lconf->id);
	targ->sched_port = rte_sched_port_config(&targ->qos_conf.port_params);

	DPPD_PANIC(targ->sched_port == NULL, "failed to create sched_port");

	mprintf("number of pipes: %d\n\n", targ->qos_conf.port_params.n_pipes_per_subport);
	int err = rte_sched_subport_config(targ->sched_port, 0, targ->qos_conf.subport_params);
	DPPD_PANIC(err != 0, "Failed setting up sched_port subport, error: %d", err);

	/* only single subport and single pipe profile is supported */
	for (uint32_t pipe = 0; pipe < targ->qos_conf.port_params.n_pipes_per_subport; ++pipe) {
		err = rte_sched_pipe_config(targ->sched_port, 0 , pipe, 0);
		DPPD_PANIC(err != 0, "failed setting up sched port pipe, error: %d", err);
	}
}

static struct task_init task_init_qos = {
	.mode_str = "qos",
	.early_init = early_init,
	.init = init_task_qos,
	.handle = handle_qos_bulk,
	.thread_x = thread_qos,
	.flag_req_data = REQ_USER_TABLE | REQ_DSCP,
	.flag_features = TASK_CLASSIFY,
	.size = sizeof(struct task_qos)
};

void reg_task_qos(void)
{
	reg_task(&task_init_qos);
}
