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

#include <rte_ethdev.h>

#include "tx_pkt.h"
#include "stats.h"
#include "prefetch.h"
#include "dppd_assert.h"
#include "debug.h"

void tx_buf_pkt_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts, const uint8_t out)
{
	const uint16_t ntx = tbase->tx_mbuf[out].n_mbufs;
	tbase->tx_mbuf[out].n_mbufs = (ntx + n_pkts) & (MAX_PKT_BURST * 2 - 1);

	if (ntx + n_pkts < MAX_PKT_BURST * 2) {
		rte_memcpy(tbase->tx_mbuf[out].mbuf + ntx, mbufs, n_pkts * sizeof(*mbufs));
	}
	else {
		DPPD_ASSERT(ntx < MAX_PKT_BURST*2);
		const uint16_t offset = MAX_PKT_BURST * 2 - ntx;
		rte_memcpy(tbase->tx_mbuf[out].mbuf + ntx, mbufs, offset * sizeof(*mbufs));
		rte_memcpy(tbase->tx_mbuf[out].mbuf, mbufs + offset, (n_pkts - offset)*sizeof(*mbufs));
	}

	DBG_DUMP_PKT_BULK(&tbase->task_debug, "TX", mbufs, n_pkts);
}


void tx_buf_pkt_single(struct task_base *tbase, struct rte_mbuf *mbuf, const uint8_t out)
{
	const uint16_t ntx = tbase->tx_mbuf[out].n_mbufs;
	tbase->tx_mbuf[out].n_mbufs = (ntx + 1) & (MAX_PKT_BURST * 2 - 1);
	tbase->tx_mbuf[out].mbuf[ntx] = mbuf;
	DBG_DUMP_PKT(&tbase->task_debug, "TX", mbuf);
}


/* The following help functions also report stats. Therefore we need
   to pass the task_base struct. */
static inline void tx_drop(const struct port_queue *port_queue, struct rte_mbuf **mbufs, uint16_t n_pkts, __attribute__((unused)) struct task_base *tbase)
{
	uint16_t ntx = rte_eth_tx_burst(port_queue->port, port_queue->queue, mbufs, n_pkts);

	INCR_TX_PKT_COUNT(tbase->stats, ntx);
	if (ntx < n_pkts) {
		INCR_TX_DROP_COUNT(tbase->stats, n_pkts - ntx);
		do {
			rte_pktmbuf_free(mbufs[ntx++]);
		}
		while (ntx < n_pkts);
	}
}

static inline void tx_no_drop(const struct port_queue *port_queue, struct rte_mbuf **mbufs, uint16_t n_pkts, __attribute__((unused)) struct task_base *tbase)
{
	uint16_t ret;

	INCR_TX_PKT_COUNT(tbase->stats, n_pkts);

	do {
		ret = rte_eth_tx_burst(port_queue->port, port_queue->queue, mbufs, n_pkts);
		mbufs += ret;
		n_pkts -= ret;
	}
	while (n_pkts);
}

static inline void ring_enq_drop(struct rte_ring *ring, struct rte_mbuf *const *mbufs, uint16_t n_pkts, __attribute__((unused)) struct task_base *tbase)
{
	/* return 0 on succes, -ENOBUFS on failure */
	if (unlikely(rte_ring_sp_enqueue_bulk(ring, (void *const *)mbufs, n_pkts))) {
		for (uint16_t i = 0; i < n_pkts; ++i) {
			rte_pktmbuf_free(mbufs[i]);
		}
		INCR_TX_DROP_COUNT(tbase->stats, n_pkts);
	}
	else {
		INCR_TX_PKT_COUNT(tbase->stats, n_pkts);
	}
}

static inline void ring_enq_no_drop(struct rte_ring *ring, struct rte_mbuf *const *mbufs, uint16_t n_pkts, __attribute__((unused)) struct task_base *tbase)
{
	while (rte_ring_sp_enqueue_bulk(ring, (void *const *)mbufs, n_pkts));
	INCR_TX_PKT_COUNT(tbase->stats, n_pkts);
}

void flush_queues_sw(struct task_base *tbase)
{
	uint16_t ntx, last_sent;
	for (uint8_t i = 0; i < tbase->tx_params_sw.nb_txrings; ++i) {
		ntx = tbase->tx_mbuf[i].n_mbufs;
		last_sent = tbase->tx_mbuf[i].last_sent;
		if (!last_sent && ntx) {
			DPPD_ASSERT(ntx <= MAX_RING_BURST);
			ring_enq_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf, ntx, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;

		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			DPPD_ASSERT(ntx - MAX_PKT_BURST <= MAX_RING_BURST);
			ring_enq_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;
		}
	}

	tbase->flags &= ~FLAG_TX_FLUSH;
}

void flush_queues_hw(struct task_base *tbase)
{
	uint16_t ntx, last_sent;
	for (uint8_t i = 0; i < tbase->tx_params_hw.nb_txports; ++i) {
		ntx = tbase->tx_mbuf[i].n_mbufs;
		last_sent = tbase->tx_mbuf[i].last_sent;
		if (!last_sent && ntx) {
			tx_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf, ntx, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;

		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			tx_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;
		}
	}

	tbase->flags &= ~FLAG_TX_FLUSH;
}

void flush_queues_no_drop_hw(struct task_base *tbase)
{
	uint16_t ntx, last_sent;
	for (uint8_t i = 0; i < tbase->tx_params_hw.nb_txports; ++i) {
		ntx = tbase->tx_mbuf[i].n_mbufs;
		last_sent = tbase->tx_mbuf[i].last_sent;
		if (!last_sent && ntx) {
			tx_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf, ntx, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;
		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			tx_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;
		}
	}

	tbase->flags &= ~FLAG_TX_FLUSH;
}

void flush_queues_no_drop_sw(struct task_base *tbase)
{
	uint16_t ntx, last_sent;
	for (uint8_t i = 0; i < tbase->tx_params_sw.nb_txrings; ++i) {
		ntx = tbase->tx_mbuf[i].n_mbufs;
		last_sent = tbase->tx_mbuf[i].last_sent;
		if (!last_sent && ntx) {
			ring_enq_no_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf, ntx, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;
		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			ring_enq_no_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].n_mbufs = 0;
			tbase->tx_mbuf[i].last_sent = 0;
		}
	}
	tbase->flags &= ~FLAG_TX_FLUSH;
}

void tx_pkt_no_drop_no_buf_hw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts)
{
	/* tx_portid will always be 0, otherwise the no_buf would not make sense. */
	tx_no_drop(&tbase->tx_params_hw.tx_port_queue[0], mbufs, n_pkts, tbase);
}

void tx_pkt_no_drop_no_buf_sw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts)
{
	ring_enq_no_drop(tbase->tx_params_sw.tx_rings[0], mbufs, n_pkts, tbase);
}

void tx_pkt_no_buf_hw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts)
{
	/* tx_portid will always be 0, otherwise the no_buf would not make sense. */
	tx_drop(&tbase->tx_params_hw.tx_port_queue[0], mbufs, n_pkts, tbase);
}

void tx_pkt_no_buf_sw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts)
{
	ring_enq_drop(tbase->tx_params_sw.tx_rings[0], mbufs, n_pkts, tbase);
}

void tx_pkt_no_drop_hw(struct task_base *tbase)
{
	const uint8_t nb_bufs = tbase->tx_params_hw.nb_txports;
	for (uint8_t i = 0; i < nb_bufs; ++i) {
		const uint16_t ntx = tbase->tx_mbuf[i].n_mbufs;
		const uint16_t last_sent = tbase->tx_mbuf[i].last_sent;
		if (last_sent && ntx < MAX_PKT_BURST) {
			tx_no_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 0;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			tx_no_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 1;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
	}
}

void tx_pkt_no_drop_sw(struct task_base *tbase)
{
	const uint8_t nb_bufs = tbase->tx_params_sw.nb_txrings;
	for (uint8_t i = 0; i < nb_bufs; ++i) {
		const uint16_t ntx = tbase->tx_mbuf[i].n_mbufs;
		const uint16_t last_sent = tbase->tx_mbuf[i].last_sent;
		if (last_sent && ntx < MAX_PKT_BURST) {
			ring_enq_no_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 0;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			ring_enq_no_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 1;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
	}
}

void tx_pkt_hw(struct task_base *tbase)
{
	const uint8_t nb_bufs = tbase->tx_params_hw.nb_txports;
	for (uint8_t i = 0; i < nb_bufs; ++i) {
		const uint16_t ntx = tbase->tx_mbuf[i].n_mbufs;
		const uint16_t last_sent = tbase->tx_mbuf[i].last_sent;
		if (last_sent && ntx < MAX_PKT_BURST) {
			tx_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 0;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			tx_drop(&tbase->tx_params_hw.tx_port_queue[i], tbase->tx_mbuf[i].mbuf, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 1;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
	}
}

void tx_pkt_sw(struct task_base *tbase)
{
	const uint8_t nb_bufs = tbase->tx_params_sw.nb_txrings;
	for (uint8_t i = 0; i < nb_bufs; ++i) {
		const uint16_t ntx = tbase->tx_mbuf[i].n_mbufs;
		const uint16_t last_sent = tbase->tx_mbuf[i].last_sent;

		if (last_sent && ntx < MAX_PKT_BURST) {
			ring_enq_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf + MAX_PKT_BURST, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 0;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			ring_enq_drop(tbase->tx_params_sw.tx_rings[i], tbase->tx_mbuf[i].mbuf, MAX_PKT_BURST, tbase);
			tbase->tx_mbuf[i].last_sent = 1;
			tbase->flags &= ~FLAG_TX_FLUSH;
		}
	}
}
