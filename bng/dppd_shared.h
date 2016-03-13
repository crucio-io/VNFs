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

#ifndef _DPPD_SHARED_H_
#define _DPPD_SHARED_H_

#include <rte_ether.h>

#define REQ_NEXT_HOP	0x0001
#define REQ_GRE_TABLE	0x0002
#define REQ_USER_TABLE	0x0004
#define REQ_LPM4	0x0010
#define REQ_LPM6	0x0020
#define REQ_WT_TABLE	0x0040
#define REQ_DSCP	0x0080
#define REQ_CPE_TABLE	0x0100
#define REQ_IPV6_TUNNEL 0x0200

struct qinq_to_gre_lookup_entry {
	uint16_t svlan;
	uint16_t cvlan;
	uint32_t gre_id;
	uint32_t user;
};

struct cpe_table_entry {
	uint32_t port_idx;
	uint32_t gre_id;
	uint32_t svlan;
	uint32_t cvlan;
	uint32_t ip;
	struct ether_addr eth_addr;
	uint32_t user;
};

/* Global data shared amongst the task arguments (task_args struct) of all tasks */
struct dppd_shared {
	uint8_t *worker_thread_table;
	uint16_t *user_table;
	uint8_t *dscp;
	struct rte_lpm *ipv4_lpm;
	struct qinq_to_gre_lookup_entry* qinq_to_gre_lookup;
	uint32_t qinq_to_gre_lookup_count;
	struct next_hop_struct *next_hop;
	struct cpe_table_entry *cpe_table_entries;
	uint32_t cpe_table_entries_count;
        struct ipv6_tun_binding_table* ipv6_tun_binding_table;
};

#endif /* _DPPD_SHARED_H_ */
