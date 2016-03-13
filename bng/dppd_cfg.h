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

#ifndef _DPPD_CFG_H
#define _DPPD_CFG_H

#include <inttypes.h>

#include "dppd_globals.h"


#define TGSF_USE_VF            0x00000001      /* use SR-IOV virtual functions */
#define TGSF_AUTOSTART         0x00000002      /* start all ports automatically */
#define TGSF_CHECK_INIT        0x00000004      /* check initialization sequence and exit */
#define TGSF_CHECK_SYNTAX      0x00000008      /* check configuration file syntax and exit */
#define TGSF_SHUFFLE           0x00000010      /* shuffle memory addresses within memory pool */
#define TGSF_WAIT_ON_QUIT      0x00000020      /* wait for all cores to stop before exiting */

struct dppd_cfg {
	uint32_t        cfg_version_major;
	uint32_t        cfg_version_minor;
	uint32_t	flags;		/* TGSF_* flags above */
	uint32_t	master;		/* master core to run user interface on */
	uint64_t        core_mask;      /* active cores without master core */
	uint32_t	start_time;	/* if set (not 0), average pps will be calculated starting after start_time seconds */
	uint32_t	duration_time;      /* if set (not 0), dppd will exit duration_time seconds after start_time */
	uint32_t	QinQ_tag;	/* Tag used to identify QinQ. 88A8 by default, but sometimes 8100 */
	char            name[MAX_NAME_SIZE];
	char            path_gre_cfg[MAX_PATH_LEN];
	char            path_user_cfg[MAX_PATH_LEN];
	char            path_next_hop_cfg[MAX_PATH_LEN];
	char            path_ipv4_cfg[MAX_PATH_LEN];
	char            path_ipv6_cfg[MAX_PATH_LEN];
	char            path_dscp_cfg[MAX_PATH_LEN];
	char            path_cpe_table_cfg[MAX_PATH_LEN];
	int32_t         cpe_table_ports[DPPD_MAX_PORTS];
        char            path_ipv6_tunnel_cfg[MAX_PATH_LEN];
        char            path_acl_cfg[MAX_PATH_LEN];
};

extern struct dppd_cfg dppd_cfg;

/* master core is considered inactive */
int dppd_core_active(uint32_t lcore_id);

/* Returns non-zero if past lcore_id is the last active core. The
   first core can be found by setting *lcore_id == -1 */
int dppd_core_next(uint32_t *lcore_id);

/* includes master core */
int dppd_core_next_m(uint32_t *lcore_id);

#endif /* __DPPD_CFG_H_ */
