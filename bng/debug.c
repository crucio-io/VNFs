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

#ifdef DPPD_CMD_DUMP

#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_mbuf.h>

#include "debug.h"
#include "display.h"

#define DUMP_PKT_LEN 128

void DBG_DUMP_PKT(struct task_debug *task_dbg, const char *msg, struct rte_mbuf *mbuf)
{
	DBG_DUMP_PKT_BULK(task_dbg, msg, &mbuf, 1);
}

static void dump_1pkt(uint8_t core_id, uint8_t task_id, const char *msg, struct rte_mbuf *mbuf)
{
	char buf[512];
	size_t str_len;
	const struct ether_hdr *peth = rte_pktmbuf_mtod(mbuf, const struct ether_hdr *);
	const struct ipv4_hdr *dpip = (const struct ipv4_hdr *)(peth + 1);
	const uint8_t *pkt_bytes = (const uint8_t *)peth;
	const uint16_t len = rte_pktmbuf_pkt_len(mbuf);

	str_len = 0;
	buf[0] = 0;

	str_len = snprintf(buf, sizeof(buf) - str_len, "Core %u Task %u %s (pkt_len: %u, Eth: %x, Proto: %#06x)",
			   core_id, task_id, msg, len, peth->ether_type, dpip->next_proto_id);

	for (uint16_t i = 0; i < len && i < DUMP_PKT_LEN; ++i) {
		if (i % 16 == 0) {
			if (str_len > (sizeof(buf) - 100)) {
				mprintf("%s", buf);
				str_len = 0;
				buf[0] = 0;
			}
			str_len += snprintf(buf + str_len, sizeof(buf) - str_len, "\n%04x  ", i);
		}
		else if (i % 8 == 0) {
			str_len += snprintf(buf + str_len, sizeof(buf) - str_len, " ");
		}
		str_len += snprintf(buf + str_len, sizeof(buf) - str_len, "%02x ", pkt_bytes[i]);
	}
	snprintf(buf + str_len, sizeof(buf) - str_len, "\n");
	mprintf("%s", buf);
}

void DBG_DUMP_PKT_BULK(struct task_debug *task_dbg, const char *msg, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	uint32_t nb_print = rte_atomic32_read(&task_dbg->nb_print);

	/* Minimize the performance impact of dumping packets */
	if (!nb_print) {
		return;
	}

	nb_print = nb_print < n_pkts? nb_print : n_pkts;
	rte_atomic32_sub(&task_dbg->nb_print, nb_print);

	for (uint16_t i = 0; i < nb_print; ++i) {
		dump_1pkt(task_dbg->core_id, task_dbg->task_id, msg, mbufs[i]);
	}
}

void DBG_PRINT_PKT(struct task_debug *task_dbg, struct rte_mbuf *mbuf)
{
	rte_atomic32_set(&task_dbg->nb_print, 1);
	DBG_DUMP_PKT_BULK(task_dbg, "", &mbuf, 1);
}

#endif
