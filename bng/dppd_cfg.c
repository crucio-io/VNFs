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

#include "dppd_cfg.h"

struct dppd_cfg dppd_cfg;

static int dppd_core_active_m(uint32_t lcore_id)
{
	return (dppd_cfg.core_mask & (__UINT64_C(1) << lcore_id)) ||
		lcore_id == dppd_cfg.master;
}

int dppd_core_active(uint32_t lcore_id)
{
	return (dppd_cfg.core_mask & (__UINT64_C(1) << lcore_id)) &&
		lcore_id != dppd_cfg.master;
}

int dppd_core_next(uint32_t* lcore_id)
{
	for (uint32_t i = *lcore_id + 1; i < 64; ++i) {
		if (dppd_core_active(i) && i != dppd_cfg.master) {
			*lcore_id = i;
			return 0;
		}
	}
	return -1;
}

int dppd_core_next_m(uint32_t* lcore_id)
{
	for (uint32_t i = *lcore_id + 1; i < 64; ++i) {
		if (dppd_core_active_m(i)) {
			*lcore_id = i;
			return 0;
		}
	}
	return -1;
}
