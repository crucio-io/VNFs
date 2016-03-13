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

#ifndef _TX_PKT_H_
#define _TX_PKT_H_

#include <inttypes.h>

struct task_base;
struct rte_mbuf;

void flush_queues_hw(struct task_base *tbase);
void flush_queues_sw(struct task_base *tbase);

void flush_queues_no_drop_hw(struct task_base *tbase);
void flush_queues_no_drop_sw(struct task_base *tbase);

void tx_pkt_no_drop_no_buf_hw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts);
void tx_pkt_no_drop_no_buf_sw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts);

void tx_pkt_no_buf_hw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts);
void tx_pkt_no_buf_sw(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts);

void tx_pkt_no_drop_hw(struct task_base *tbase);
void tx_pkt_no_drop_sw(struct task_base *tbase);

void tx_pkt_hw(struct task_base *tbase);
void tx_pkt_sw(struct task_base *tbase);

void tx_buf_pkt_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, const uint16_t n_pkts, const uint8_t portid);
void tx_buf_pkt_single(struct task_base *tbase, struct rte_mbuf *mbuf, const uint8_t portid);


#endif /* _TX_PKT_H_ */
