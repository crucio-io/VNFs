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

#ifndef _READ_CONFIG_H_
#define _READ_CONFIG_H_

#include <rte_lpm.h>
#include <rte_acl.h>

struct rte_lpm *read_lpm4_config(const char *cfg_file, uint8_t socket);

void read_lpm6_config(const char *cfg_file, uint8_t socket);

struct next_hop_struct *read_next_hop_config(const char *cfg_file, uint8_t socket);

/* Need to pass wt_table. This is also filled in while the configuration is loaded. */
struct qinq_to_gre_lookup_entry *read_gre_table_config(const char *cfg_file, uint8_t socket, uint32_t* count);

uint8_t *read_dscp_config(const char *cfg_file, uint8_t socket);

/* If qinq_to_gre_lookup is a NULL pointer, the hash lookup will be created. */
uint16_t *read_user_table_config(const char *cfg_file, struct qinq_to_gre_lookup_entry **qinq_to_gre_lookup, uint32_t* qinq_to_gre_lookup_count, uint8_t socket);

struct ipv6_tun_binding_table * read_ipv6_tun_bindings(const char *cfg_file, uint8_t socket);

uint8_t get_nb_hop_ports(void);

uint8_t get_hop_port(uint8_t idx);

int read_rules_config(struct rte_acl_ctx* ctx, const char *cfg_file, uint32_t n_max_rules, int use_qinq);

struct cpe_table_entry;
void read_cpe_table_config(struct cpe_table_entry **cpe_table_entries, uint32_t* cpe_table_entries_count, const char *cfg_file, int32_t* port_defs, uint32_t n_port_defs);

#endif /* _READ_CONFIG_H_ */
