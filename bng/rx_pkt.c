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
#include <rte_ethdev.h>

#include "clock.h"
#include "rx_pkt.h"
#include "stats.h"

uint16_t rx_pkt_hw(struct task_base *tbase, struct rte_mbuf **mbufs)
{
	START_EMPTY_MEASSURE();
	uint16_t nb_rx = rte_eth_rx_burst(tbase->rx_params_hw.rx_port, tbase->rx_params_hw.rx_queue, mbufs, MAX_PKT_BURST);
	if (likely(nb_rx > 0)) {
		DBG_DUMP_PKT_BULK(&tbase->task_debug, "RX", mbufs, nb_rx);
		return nb_rx;
	}
	INCR_EMPTY_CYCLES(tbase->stats, rte_rdtsc() - cur_tsc);
	return 0;
}

uint16_t rx_pkt_sw(struct task_base *tbase, struct rte_mbuf **mbufs)
{
	START_EMPTY_MEASSURE();
#ifdef BRAS_RX_BULK
	if (unlikely (rte_ring_sc_dequeue_bulk(tbase->rx_params_sw.rx_rings[tbase->rx_params_sw.last_read_ring], (void **)mbufs, MAX_RING_BURST)) < 0) {
		++tbase->rx_params_sw.last_read_ring;
		if (unlikely(tbase->rx_params_sw.last_read_ring == tbase->rx_params_sw.nb_rxrings)) {
			tbase->rx_params_sw.last_read_ring = 0;
		}
		INCR_EMPTY_CYCLES(tbase->stats, rte_rdtsc() - cur_tsc);
		return 0;
	}
	else {
		DBG_DUMP_PKT_BULK(&tbase->task_debug, "RX", mbufs, MAX_RING_BURST);
		return MAX_RING_BURST;
	}
#else
	uint16_t nb_rx = rte_ring_sc_dequeue_burst(tbase->rx_params_sw.rx_rings[tbase->rx_params_sw.last_read_ring], (void **)mbufs, MAX_RING_BURST);
	++tbase->rx_params_sw.last_read_ring;
	if (unlikely(tbase->rx_params_sw.last_read_ring == tbase->rx_params_sw.nb_rxrings)) {
		tbase->rx_params_sw.last_read_ring = 0;
	}

	if (nb_rx != 0) {
		DBG_DUMP_PKT_BULK(&tbase->task_debug, "RX", mbufs, nb_rx);
		return nb_rx;
	}
	else {
		INCR_EMPTY_CYCLES(tbase->stats, rte_rdtsc() - cur_tsc);
		return 0;
	}
#endif
}
