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

#include <rte_malloc.h>
#include "task_init.h"
#include "rx_pkt.h"
#include "tx_pkt.h"
#include "quit.h"
#include "display.h"

#include "handle_none.h"
#include "handle_drop.h"
#include "handle_qos.h"

#include "handle_qinq_decap4.h"
#include "handle_qinq_decap6.h"
#include "handle_qinq_encap4.h"
#include "handle_qinq_encap6.h"
#include "handle_routing.h"
#include "handle_unmpls.h"
#include "handle_fwd.h"
#include "handle_lb_qinq.h"
#include "handle_lb_net.h"
#include "handle_classify.h"
#include "handle_gre_decap_encap.h"
#include "handle_lb_pos.h"
#include "handle_l2fwd.h"
#include "handle_sig.h"
#include "handle_police.h"
#include "handle_acl.h"
#include "handle_ipv6_tunnel.h"

#define MAX_TASK_TYPES 30

#include "handle_pf_acl.h"

struct task_init task_init[MAX_TASK_TYPES];
uint32_t nb_task_types = 0;

static void call_if_exists(void (*reg_func)(void))
{
        if (reg_func) {
                reg_func();
        }
}

void reg_all_tasks(void)
{
        call_if_exists(reg_task_routing);
	call_if_exists(reg_task_qinq_encap4);
	call_if_exists(reg_task_qinq_decap4);
        call_if_exists(reg_task_qinq_encap6);
	call_if_exists(reg_task_qinq_decap6);
        call_if_exists(reg_task_fwd);
        call_if_exists(reg_task_lb_net);
        call_if_exists(reg_task_lb_qinq);
	call_if_exists(reg_task_lb_pos);
	call_if_exists(reg_task_classify);
        call_if_exists(reg_task_unmpls);
	call_if_exists(reg_task_drop);
	call_if_exists(reg_task_qos);
        call_if_exists(reg_task_gre);
	call_if_exists(reg_task_none);
	call_if_exists(reg_task_l2fwd);
	call_if_exists(reg_task_sig);
	call_if_exists(reg_task_police);
	call_if_exists(reg_task_acl);
        call_if_exists(reg_task_ipv6_decap);
        call_if_exists(reg_task_ipv6_encap);
	call_if_exists(reg_task_pf_acl);
}

void reg_task(const struct task_init* t)
{
	DPPD_PANIC(nb_task_types == MAX_TASK_TYPES, "Too many task types, only %d allowed\n", MAX_TASK_TYPES);
	DPPD_PANIC(t->handle == NULL, "No handle function specified for task with name %d\n", t->mode);
	DPPD_PANIC(t->thread_x == NULL, "No thread function specified\n");
	memcpy(&task_init[nb_task_types++], t, sizeof(*t));
}

struct task_init *to_task_init(const char *mode_str, const char *sub_mode_str)
{
	for (uint32_t i = 0; i < nb_task_types; ++i) {
		if (!strcmp(mode_str, task_init[i].mode_str) &&
		    !strcmp(sub_mode_str, task_init[i].sub_mode_str)) {
			return &task_init[i];
		}
	}
	return NULL;
}

static size_t calc_memsize(struct task_args *targ, size_t task_size)
{
	size_t memsize = task_size;

#ifdef BRAS_STATS
	memsize += sizeof(struct stats);
#endif

	if (targ->nb_rxrings != 0) {
		memsize += sizeof(struct rte_ring *)*targ->nb_rxrings;
	}

	if (targ->nb_txrings != 0) {
		memsize += (sizeof(struct mbuf_table_struct) + sizeof(struct rte_ring *)) * targ->nb_txrings;
	}
	else {
		memsize += (sizeof(struct mbuf_table_struct) + sizeof(struct port_queue)) * targ->nb_txports;
	}

	return memsize;
}

static size_t init_rx_tx_rings_ports(struct task_args *targ, struct task_base *tbase, size_t offset)
{
	if (targ->nb_rxrings != 0) {
		tbase->rx_pkt = rx_pkt_sw;
		tbase->rx_params_sw.nb_rxrings = targ->nb_rxrings;
		tbase->rx_params_sw.rx_rings = (struct rte_ring **)(((uint8_t *)tbase) + offset);
		offset += sizeof(struct rte_ring *)*tbase->rx_params_sw.nb_rxrings;

		for (uint8_t i = 0; i < tbase->rx_params_sw.nb_rxrings; ++i) {
			tbase->rx_params_sw.rx_rings[i] = targ->rx_rings[i];
		}
	}
	else {
		/* When receiving from a port, only a single port
		   can be used to receive packets. The related values
		   are saved inline. */
		tbase->rx_pkt = rx_pkt_hw;
		tbase->rx_params_hw.rx_port = targ->rx_port;
		tbase->rx_params_hw.rx_queue = targ->rx_queue;
	}


	if (targ->nb_txrings != 0) {
		tbase->tx_params_sw.nb_txrings = targ->nb_txrings;
		tbase->tx_mbuf = (struct mbuf_table_struct *)(((uint8_t *)tbase) + offset);
		offset += sizeof(struct mbuf_table_struct) * tbase->tx_params_sw.nb_txrings;
		tbase->tx_params_sw.tx_rings = (struct rte_ring **)(((uint8_t *)tbase) + offset);
		offset += sizeof(struct rte_ring *)*tbase->tx_params_sw.nb_txrings;

		for (uint8_t i = 0; i < tbase->tx_params_sw.nb_txrings; ++i) {
			tbase->tx_params_sw.tx_rings[i] = targ->tx_rings[i];
		}
	}
	else {
		tbase->tx_params_hw.nb_txports = targ->nb_txports;
		tbase->tx_mbuf = (struct mbuf_table_struct *)(((uint8_t *)tbase) + offset);
		offset += sizeof(struct mbuf_table_struct) * tbase->tx_params_hw.nb_txports;
		tbase->tx_params_hw.tx_port_queue = (struct port_queue *)(((uint8_t *)tbase) + offset);
		offset += sizeof(struct port_queue) * tbase->tx_params_hw.nb_txports;
		for (uint8_t i = 0; i < tbase->tx_params_hw.nb_txports; ++i) {
			tbase->tx_params_hw.tx_port_queue[i].port = targ->tx_port_queue[i].port;
			tbase->tx_params_hw.tx_port_queue[i].queue = targ->tx_port_queue[i].queue;
		}
	}
	return offset;
}

void *tx_function(struct task_args *targ)
{
	if (targ->flags & TASK_ARG_DROP) {
		return targ->nb_txrings ? tx_pkt_sw : tx_pkt_hw;
	}
	else {
		return targ->nb_txrings ? tx_pkt_no_drop_sw : tx_pkt_no_drop_hw;
	}
}

void *flush_function(struct task_args *targ)
{
	if (targ->flags & TASK_ARG_DROP) {
		return targ->nb_txrings ? flush_queues_sw : flush_queues_hw;
	}
	else {
		return targ->nb_txrings ? flush_queues_no_drop_sw : flush_queues_no_drop_hw;
	}
}

void *no_buf_tx_function(struct task_args *targ)
{
	if (targ->flags & TASK_ARG_DROP) {
		return targ->nb_txrings ? tx_pkt_no_buf_sw : tx_pkt_no_buf_hw;
	}
	else {
		return targ->nb_txrings ? tx_pkt_no_drop_no_buf_sw : tx_pkt_no_drop_no_buf_hw;
	}
}

struct task_base *init_task_struct(struct task_args *targ)
{
	struct task_init* t = targ->task_init;
	size_t offset = 0;
	size_t memsize = calc_memsize(targ, t->size);
	uint8_t task_socket = rte_lcore_to_socket_id(targ->lconf->id);
	struct task_base *tbase = rte_zmalloc_socket(NULL, memsize, CACHE_LINE_SIZE, task_socket);
	DPPD_PANIC(tbase == NULL, "Failed to allocate memory for task (%zu bytes)", memsize);
	offset += t->size;

	offset = init_rx_tx_rings_ports(targ, tbase, offset);


#ifdef BRAS_STATS
	tbase->stats = (struct stats *)(((uint8_t *)tbase) + offset);
	offset += sizeof(struct stats);
#endif

#ifdef DPPD_CMD_DUMP
	tbase->task_debug.core_id = targ->lconf->id;
	tbase->task_debug.task_id = targ->task;
#endif

	tbase->handle_bulk = t->handle;

	if (t->init) {
		t->init(tbase, targ);
	}

	return tbase;
}
