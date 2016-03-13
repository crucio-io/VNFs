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

#ifndef _MPLS_H_
#define _MPLS_H_

#include <rte_version.h>

#include "etypes.h"
#include "route.h"
#include "dppd_args.h"
#include "display.h"
#include "dppd_assert.h"

struct mpls_hdr {
	union {
		struct {
			uint16_t lbl_h;     /* Label */
			uint8_t bos: 1;     /* Bottom of Stack */
			uint8_t cos: 3;     /* Class of Service */
			uint8_t lbl_l: 4;   /* Label */
			uint8_t ttl;        /* Time to Live, 64 */
		};
		uint32_t bytes;
	};
} __attribute__((__packed__));

static inline struct ether_hdr *mpls_encap_v6(struct rte_mbuf *mbuf)
{
	struct ether_hdr *peth = (struct ether_hdr *)rte_pktmbuf_mtod(mbuf, struct ether_hdr *);
	struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(peth + 1);
	peth = (struct ether_hdr *)rte_pktmbuf_prepend(mbuf, 4);
	DPPD_ASSERT(peth);
	peth->ether_type = ETYPE_MPLSU;
	*((uint32_t *)(peth + 1)) = find_ipv6_route((struct ipv6_addr*)pip6->dst_addr, &peth->d_addr) | 0x00010000; // Set BoS to 1
	return peth;
}

static inline struct ether_hdr *mpls_encap(struct rte_mbuf *mbuf, uint32_t mpls)
{
	struct ether_hdr *peth = (struct ether_hdr *)rte_pktmbuf_prepend(mbuf, 4);
	DPPD_ASSERT(peth);
	rte_prefetch0(peth);
#ifdef HARD_CRC
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	mbuf->l2_l3_len += (4 << 9);
#else
	mbuf->pkt.vlan_macip.data += (4 << 9);
#endif
#endif

	*((uint32_t *)(peth + 1)) = mpls | 0x00010000; // Set BoS to 1

	peth->ether_type = ETYPE_MPLSU;
	return peth;
}

static inline uint8_t mpls_decap(struct rte_mbuf *mbuf)
{
	struct ether_hdr *peth = (struct ether_hdr *)(rte_pktmbuf_mtod(mbuf, struct ether_hdr *));
	struct ether_hdr *pneweth = (struct ether_hdr *)rte_pktmbuf_adj(mbuf, 4);

	DPPD_ASSERT(pneweth);

	const struct mpls_hdr *mpls = (const struct mpls_hdr *)(peth + 1);
	if (mpls->bos == 0) {
		// Double MPLS tag
		pneweth = (struct ether_hdr *)rte_pktmbuf_adj(mbuf, 4);
		DPPD_ASSERT(pneweth);
	}
	const struct ipv4_hdr *pip = (const struct ipv4_hdr *)(pneweth + 1);
	if ((pip->version_ihl >> 4) == 4) {
		pneweth->ether_type = ETYPE_IPv4;
		return IPV4;
	}
	else if ((pip->version_ihl >> 4) == 6) {
		pneweth->ether_type = ETYPE_IPv6;
		return IPV6;
	}

	mprintf("Error Decoding MPLS Packet - neither IPv4 neither IPv6: version %u\n", pip->version_ihl);
	return 255;
}

static inline uint8_t mpls_untag(struct rte_mbuf *mbuf)
{
	uint16_t eth_type = rte_pktmbuf_mtod(mbuf, struct ether_hdr *)->ether_type;
	if (eth_type == ETYPE_MPLSU) {
		mpls_decap(mbuf);
		return 1;
	}
	if (eth_type != ETYPE_LLDP) {
		mprintf("Error Removing MPLS: ether_type = %#06x\n", eth_type);
	}
	return 0;
}

#endif /* _MPLS_H_ */
