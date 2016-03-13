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

#include "dppd_cksum.h"

/* compute IP 16 bit checksum */
void dppd_ip_cksum_sw(struct ipv4_hdr *buf, uint32_t cksum, uint16_t *res)
{
	const uint16_t size = sizeof(struct ipv4_hdr);
	uint32_t nb_dwords;
	uint32_t tail, mask;
	uint32_t *pdwd = (uint32_t *)buf;

	/* compute 16 bit checksum using hi and low parts of 32 bit integers */
	for (nb_dwords = (size >> 2); nb_dwords > 0; --nb_dwords) {
		cksum += (*pdwd >> 16);
		cksum += (*pdwd & 0xFFFF);
		++pdwd;
	}

	/* deal with the odd byte length */
	if (size & 0x03) {
		tail = *pdwd;
		/* calculate mask for valid parts */
		mask = 0xFFFFFFFF << ((size & 0x03) << 3);
		/* clear unused bits */
		tail &= ~mask;

		cksum += (tail >> 16) + (tail & 0xFFFF);
	}

	cksum = (cksum >> 16) + (cksum & 0xFFFF);
	cksum = (cksum >> 16) + (cksum & 0xFFFF);

	*res = ~((uint16_t)cksum);
}
