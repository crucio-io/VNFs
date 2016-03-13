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

#ifndef _TASK_BASE_H_
#define _TASK_BASE_H_

#include <rte_common.h>

#include "debug.h"
#include "defaults.h"

#define TASK_MPLS_TAGGING	       0x01
#define TASK_ROUTING		       0x02
#define TASK_CLASSIFY                  0x04
#define TASK_HANDLE_ARP                0x08
#define TASK_DO_SIG                    0x10
#define TASK_MARK                      0x20

#define FLAG_TX_FLUSH                  0x01
#define FLAG_NEVER_FLUSH               0x02

#define NO_PORT_AVAIL	0xFF

struct mbuf_table_struct {
	uint16_t        n_mbufs;
	uint8_t         last_sent;
	struct rte_mbuf *mbuf[MAX_RING_BURST * 2];
};

struct port_queue {
	uint8_t port;
	uint8_t queue;
} __attribute__((packed));

struct rx_params_hw {
	uint8_t         rx_port;
	uint8_t         rx_queue;
	uint8_t         nb_rxbulk;
} __attribute__((packed));

struct rx_params_sw {
	uint8_t         nb_rxrings;
	uint8_t         last_read_ring;
	struct rte_ring **rx_rings;
} __attribute__((packed));

struct tx_params_hw {
	struct port_queue *tx_port_queue;
	uint8_t            nb_txports;
} __attribute__((packed));

struct tx_params_sw {
	struct rte_ring **tx_rings;
	uint8_t         nb_txrings;
} __attribute__((packed));

/* The task_base is accessed for _all_ task types. In case
   no debugging is needed, it has been optimized to fit
   into a single cache line to minimize cache pollution */
struct task_base {
	void (*handle_bulk)(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts);
	union {
		void (*flush_queues)(struct task_base *tbase);
		/* No need to support flushing if not buffering pkts. */
		void (*tx_pkt_no_buf)(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts);
	};
	uint16_t (*rx_pkt)(struct task_base *tbase, struct rte_mbuf **mbufs);

#ifdef BRAS_STATS
	struct stats *stats;
#endif

	/* Contains bits telling if an action is needed
	   in the thread itself (i.e. flushing of queues). */
	uint8_t flags;

	union {
		struct rx_params_hw rx_params_hw;
		struct rx_params_sw rx_params_sw;
	};

	/* The tx_mbuf inside the tx_params should only be used
	   if there is more than one output path for a packet.
	   Good examples are routing and load balancing. The
	   packets in this mode should be passed without
	   copying any intermediate buffering. */
	struct mbuf_table_struct *tx_mbuf;
	union {
		struct tx_params_hw tx_params_hw;
		struct tx_params_sw tx_params_sw;
	};

#ifdef DPPD_CMD_DUMP
	struct task_debug task_debug;
#endif
} __attribute__((packed)) __rte_cache_aligned;


#endif /* _TASK_BASE_H_ */
