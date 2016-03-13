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

#ifndef _HANDLE_VXLAN_DECAP_H_
#define _HANDLE_VXLAN_DECAP_H_

#include "task_base.h"
#include "route.h"

#define IPv6_VERSION 6

struct ipv6_tun_binding_entry {
	struct ipv6_addr        endpoint_addr;  // IPv6 local addr
	struct ether_addr       next_hop_mac;   // mac addr of next hop towards lwB4
	uint32_t                public_ipv4;    // Public IPv4 address
	uint16_t                public_port;    // Public base port (together with port mask, defines the Port Set)
} __attribute__((__packed__));

struct ipv6_tun_binding_table {
	struct ipv6_tun_binding_entry* entry;
	uint32_t                num_binding_entries;
#ifdef IPV6_TUN_SHARED_LOOKUP
	void*                   lookup_table;   // Same data as binding_table but in a fast lookup structure
#endif
};

void reg_task_ipv6_decap(void) __attribute__((weak));
void reg_task_ipv6_encap(void) __attribute__((weak));

#endif /* _HANDLE_VXLAN_DECAP_H_ */
