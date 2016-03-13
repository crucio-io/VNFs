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

#ifndef _DPPD_PORT_CFG_H
#define _DPPD_PORT_CFG_H

#include <rte_ether.h>
#include <rte_ethdev.h>

#include "dppd_globals.h"

enum addr_type {DPPD_PORT_MAC_HW, DPPD_PORT_MAC_SET, DPPD_PORT_MAC_RAND};

struct dppd_port_cfg {
	struct rte_mempool *pool[32];  /* Rx/Tx mempool */
	uint8_t promiscuous;
	uint8_t active;
	uint8_t socket;
	uint16_t max_rxq;         /* max number of Tx queues */
	uint16_t max_txq;         /* max number of Tx queues */
	uint16_t n_rxq;           /* number of used Rx queues */
	uint16_t n_txq;           /* number of used Tx queues */
	uint32_t n_rxd;
	uint32_t n_txd;
	uint8_t  link_up;
	uint32_t  link_speed;
	enum addr_type    type;
	struct ether_addr eth_addr;    /* port MAC address */
	char name[MAX_NAME_SIZE];
	char driver_name[MAX_NAME_SIZE];
	char rx_ring[MAX_NAME_SIZE];
	char tx_ring[MAX_NAME_SIZE];
	struct rte_eth_conf port_conf;
	struct rte_eth_rxconf rx_conf;
	struct rte_eth_txconf tx_conf;
};

int dppd_nb_active_ports(void);
int dppd_last_port_active(void);

extern struct dppd_port_cfg dppd_port_cfg[];
#endif /* __DPPD_PORT_CFG_H_ */
