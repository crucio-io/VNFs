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

#ifndef _DPPD_ARGS_H_
#define _DPPD_ARGS_H_

#include "lconf.h"

struct rte_cfg {
	/* DPDK standard options */
	uint64_t core_mask;
	uint32_t memory;	 /* amount of asked memory */
	uint32_t force_nchannel; /* force number of channels */
	uint32_t force_nrank;	 /* force number of ranks */
	uint32_t no_hugetlbfs;	 /* true to disable hugetlbfs */
	uint32_t no_pci;	 /* true to disable PCI */
	uint32_t no_hpet;	 /* true to disable HPET */
	uint32_t no_shconf;	 /* true if there is no shared config */
	char    *hugedir;	 /* dir where hugetlbfs is mounted */
	char    *eal;            /* any additional eal option */
	uint32_t no_output;	 /* disable EAL debug output */
	/* run-time options */
	uint8_t  nb_ports;       /* number of ports */
};

extern struct rte_cfg        rte_cfg;
extern struct ether_addr     if_cfg[];
extern struct lcore_cfg     *lcore_cfg;
extern struct lcore_cfg      lcore_cfg_init[];


int dppd_parse_args(int argc, char **argv);
int dppd_read_config_file(void);
int dppd_setup_rte(const char *prog_name);

#endif /* _DPPD_ARGS_H_ */
