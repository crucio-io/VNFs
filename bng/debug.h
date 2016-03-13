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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef DPPD_CMD_DUMP

#include <rte_atomic.h>

struct task_debug {
	rte_atomic32_t nb_print;
	uint8_t core_id;
	uint8_t task_id;
};

struct rte_mbuf;

void DBG_DUMP_PKT(struct task_debug *task_dbg, const char *msg, struct rte_mbuf *mbuf);
void DBG_DUMP_PKT_BULK(struct task_debug *task_dbg, const char *msg, struct rte_mbuf **mbufs, uint16_t n_pkts);
void DBG_PRINT_PKT(struct task_debug *task_dbg, struct rte_mbuf *mbuf);

#else

#define DBG_DUMP_PKT_BULK(task_dbg, msg, mbuf, n_pkts)
#define DBG_DUMP_PKT(task_dbg, msg, mbuf)
#define DBG_PRINT_PKT(task_dbg, mbuf)

#endif


#endif /* _DEBUG_H_ */
