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

#include "route.h"
#include "display.h"
#include "quit.h"

struct route_struct {
	uint32_t              mpls;
	struct route_struct   *nextp;
	uint8_t		      depth: 7;
	uint8_t		      used: 1;
};

struct route_struct MyRoute6[256];

void add_ipv6_route(const struct ipv6_addr *route, uint8_t depth, uint32_t *mpls, __attribute__((unused)) struct ether_addr *addr, uint8_t socket)
{
	int i = 0;
	struct route_struct *a_route_p = MyRoute6;
	uint8_t mask = 0xFF;
	uint8_t masked_route = route->bytes[i] & mask;
	for (i = 0; i < (depth - 1) / 8;) {
		if (a_route_p[masked_route].nextp == NULL) {
			a_route_p[masked_route].nextp = rte_zmalloc_socket(NULL, 256 * sizeof(struct route_struct), 0, socket);
		}
		DPPD_PANIC(a_route_p[masked_route].nextp == NULL, "\t\tError: cannot allocate array of route_struct\n");
		a_route_p = a_route_p[masked_route].nextp;
		masked_route = route->bytes[++i] & mask;
	}
	if (a_route_p[masked_route].depth <= depth) {
		a_route_p[masked_route].depth = depth;
		a_route_p[masked_route].mpls = *mpls;
		a_route_p[masked_route].used = 1;
	}
}

uint32_t find_ipv6_route(const struct ipv6_addr *routep, __attribute__((unused)) struct ether_addr *addr)
{
	struct route_struct *a_route_p = MyRoute6, *rc = NULL;
	int i = 0;
	uint8_t route_mask = routep->bytes[i] & 0xFF;
	while (a_route_p[route_mask].nextp) {
		if (a_route_p[route_mask].used) {
			rc = &a_route_p[route_mask];
		}
		a_route_p = a_route_p[route_mask].nextp;
		route_mask = routep->bytes[++i] & 0xFF;
	}
	if (a_route_p[route_mask].used) {
		return a_route_p[route_mask].mpls;
	}
	if (rc) {
		return rc->mpls;
	}

	if (a_route_p[0].depth == 0) {
		return a_route_p[0].mpls;
	}
	return 0xFFFFFFFF;
}
