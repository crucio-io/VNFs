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

#include <stdio.h>
#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_string_fns.h>

#include "cfgfile.h"
#include "read_config.h"
#include "display.h"
#include "quit.h"
#include "route.h"
#include "hash_entry_types.h"
#include "parse_utils.h"
#include "defines.h"
#include "qinq.h"
#include "etypes.h"
#include "dppd_globals.h"
#include "dppd_shared.h"
#include "handle_ipv6_tunnel.h"
#include "ip_subnet.h"
#include "dppd_cfg.h"

#define MAX_HOP_INDEX	128

static uint8_t hop_ports[DPPD_MAX_PORTS];
static uint8_t nb_hop_ports;

static uint32_t count_lines(const char *name, int num_fields)
{
	struct csv_file* file = csv_open(name, num_fields, num_fields);
	uint32_t lines = 0;
	int ret;
	while ((ret = csv_next(file, NULL)) > 0) {
		++lines;
	};
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: %d comma separated fields\n", file->line, name, num_fields);

	return lines;
}

uint8_t get_nb_hop_ports(void)
{
	return nb_hop_ports;
}

uint8_t get_hop_port(uint8_t idx)
{
	return hop_ports[idx];
}

struct rte_lpm *read_lpm4_config(const char *cfg_file, uint8_t socket)
{
	struct csv_file *file;
	struct rte_lpm *new_lpm;
	int ret;
	char *fields[3];
	uint32_t route;
	uint8_t depth;
	uint32_t next_hop_index;
	char name[64];
	int nroutes = 0;

	file = csv_open(cfg_file, 2, 2);
	DPPD_PANIC(NULL == file, "Could not open IPv4 config file %s\n", cfg_file);
	snprintf(name, sizeof(name), "IPv4_lpm_s%u", socket);
	new_lpm = rte_lpm_create(name, socket, 16 * 65536 * 32, RTE_LPM_HEAP);
	while ((ret = csv_next(file, fields)) > 0) {
		struct ipv4_subnet dst;

		DPPD_PANIC(parse_ip_cidr(&dst, fields[0]), "Error parsing destination subnet: %s\n", get_parse_err());
		DPPD_PANIC(parse_int(&next_hop_index, fields[1]), "Error parsing next hop index: %s\n", get_parse_err());

		ret = rte_lpm_add(new_lpm, dst.ip, dst.prefix, next_hop_index);

		if (ret != 0) {
			mprintf("Failed to add (%d) index %u ip %x/%u to lpm\n", ret, next_hop_index, dst.ip, dst.prefix);
		}
		else if (++nroutes % 10000 == 0) {
			mprintf("Route %d added\n", nroutes);
		}
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpected format: IPv4 address, depth, next_hop_index\n", file->line, name);
	mprintf("\t\tConfigured %d routes\n", nroutes);
	csv_close(file);
	return new_lpm;
}

void read_lpm6_config(const char *cfg_file, uint8_t socket)
{
	struct csv_file *file;
	struct ipv6_addr route;
	uint8_t depth;
	uint8_t out_if;
	struct ipv6_addr next_hop_addr;
	struct ether_addr mac;
	uint32_t mpls;
	int ret, offset = 0;
	char *fields[6];

	file = csv_open(cfg_file, 5, 6);
	DPPD_PANIC(file == NULL, "Could not open IPv6 config file %s\n", cfg_file);
	while ((ret = csv_next(file, fields)) > 0) {
		DPPD_PANIC(0 != parse_ip6(&route, fields[0]), "Error in routing IPv6 address: %s\n", fields[0]);
		depth = atoi(fields[1]);

		if (ret == 6) {
			out_if = atoi(fields[2]);
			offset = 1;
		}

		DPPD_PANIC(0 != parse_ip6(&next_hop_addr, fields[2 + offset]), "Error in next hop IPv6 address: %s\n", get_parse_err());
		DPPD_PANIC(0 != parse_mac(&mac, fields[3 + offset]), "Error in next hop MAC address: %s\n", get_parse_err());

		mpls = atoi(fields[4 + offset]);

		add_ipv6_route(&route, depth, &mpls, &mac, socket);
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format IPv6 address, Depth, [out_if, ] next hop IPv6 address, next hop MAC address, Next hop MPLS\n", file->line, cfg_file);
	csv_close(file);
}

struct next_hop_struct *read_next_hop_config(const char *cfg_file, uint8_t socket)
{
	struct csv_file *file;
	struct next_hop_struct *next_hop_init;
	char *fields[5];
	int ret;
	uint32_t next_hop_index, port_id;

	file = csv_open(cfg_file, 5, 5);
	DPPD_PANIC(file == NULL, "Could not open next hop config file %s\n", cfg_file);
	next_hop_init = rte_zmalloc_socket(NULL, sizeof(struct next_hop_struct) * MAX_HOP_INDEX, CACHE_LINE_SIZE, socket);
	DPPD_PANIC(next_hop_init == NULL, "Could not allocate memory for next hop\n");
	while ((ret = csv_next(file, fields)) > 0) {
		DPPD_PANIC(0 != parse_int(&next_hop_index, fields[0]), "Invalid next hop index: %s\n", get_parse_err());
		DPPD_PANIC(next_hop_index > MAX_HOP_INDEX - 1, "Invalid hop index: %d\n", next_hop_index);
		DPPD_PANIC(0 != parse_int(&port_id, fields[1]), "Invalid port number: %s\n", get_parse_err());
		DPPD_PANIC(0 != parse_ip(&next_hop_init[next_hop_index].ip_dst, fields[2]), "Invalid IP for next hop: %s\n", get_parse_err());
		DPPD_PANIC(0 != parse_mac(&next_hop_init[next_hop_index].mac_port.mac, fields[3]), "Invalid MAC for next hop: %s\n", get_parse_err());
		DPPD_PANIC(port_id >= DPPD_MAX_PORTS, "Port id too high (only supporting %d ports)\n", DPPD_MAX_PORTS);

		next_hop_init[next_hop_index].mac_port.out_idx = port_id;
		next_hop_init[next_hop_index].mpls = strtol(fields[4], NULL, 16);
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: hop index, port, IP address, MAC address, MPLS\n", file->line, cfg_file);
	csv_close(file);
	return next_hop_init;
}

struct qinq_to_gre_lookup_entry *read_gre_table_config(const char *cfg_file, uint8_t socket, uint32_t* count)
{
	struct csv_file *file;
	struct qinq_to_gre_lookup_entry* qinq_to_gre_lookup;
	char *fields[2];
	uint32_t gre_id, qinq;
	uint16_t svlan, cvlan;
	int ret;

	file = csv_open(cfg_file, 2, 2);
	DPPD_PANIC(NULL == file, "Could not open GRE table %s\n", cfg_file);
	qinq_to_gre_lookup = rte_zmalloc_socket(NULL, count_lines(cfg_file, 2) * sizeof(struct qinq_to_gre_lookup_entry), CACHE_LINE_SIZE, socket);
	DPPD_PANIC(qinq_to_gre_lookup == NULL, "Error creating qinq to gre lookup");
	*count = 0;
	while ((ret = csv_next(file, fields)) > 0) {
		gre_id = atoi(fields[0]);
		qinq = atoi(fields[1]) & 0xFFFFFF;
		svlan = rte_cpu_to_be_16((qinq & 0xFFF000) >> 12);
		cvlan = rte_cpu_to_be_16(qinq & 0xFFF);

		qinq_to_gre_lookup[*count].svlan = svlan;
		qinq_to_gre_lookup[*count].cvlan = cvlan;
		qinq_to_gre_lookup[*count].gre_id = gre_id;
		++(*count);
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: gre_id, qinq\n", file->line, cfg_file);
	csv_close(file);
	return qinq_to_gre_lookup;
}

uint16_t *read_user_table_config(const char *cfg_file, struct qinq_to_gre_lookup_entry **qinq_to_gre_lookup, uint32_t* qinq_to_gre_lookup_count, uint8_t socket)
{
	struct csv_file *file;
	uint8_t added, created_hash = 0;
	uint16_t *user_table_init;
	char *fields[3];
	int ret;
	uint32_t user, qinq;
	uint16_t svlan, cvlan;

	file = csv_open(cfg_file, 3, 3);
	DPPD_PANIC(NULL == file, "Could not open user table %s\n", cfg_file);
	user_table_init = rte_zmalloc_socket(NULL, 0x1000000 * sizeof(uint16_t), CACHE_LINE_SIZE, socket);
	DPPD_PANIC(user_table_init == NULL, "Error creating worker thread table");

	if (*qinq_to_gre_lookup == NULL) {
		*qinq_to_gre_lookup = rte_zmalloc_socket(NULL, count_lines(cfg_file, 3) * sizeof(struct qinq_to_gre_lookup_entry), CACHE_LINE_SIZE, socket);
		DPPD_PANIC(qinq_to_gre_lookup == NULL, "Error creating qinq to gre lookup");
		created_hash = 1;
		*qinq_to_gre_lookup_count = 0;
	}

	while ((ret = csv_next(file, fields)) > 0) {
		user = atoi(fields[2]);

		uint16_t svlan = rte_cpu_to_be_16(atoi(fields[0]));
		uint16_t cvlan = rte_cpu_to_be_16(atoi(fields[1]));

		struct vlans qinq2 = {
			.svlan = {.eth_proto = dppd_cfg.QinQ_tag, .vlan_tci = svlan},
			.cvlan = {.eth_proto = ETYPE_VLAN,   .vlan_tci = cvlan}
		};

		user_table_init[PKT_TO_LUTQINQ(svlan, cvlan)] = user;

		/* if the hash table has been created, we can reuse the same
		   entries and add the extra QoS related user lookup data */
		if (created_hash == 0) {
			added = 0;
			for (uint32_t i = 0; i < *qinq_to_gre_lookup_count; ++i) {
				if ((*qinq_to_gre_lookup)[i].svlan == svlan &&
				    (*qinq_to_gre_lookup)[i].cvlan == cvlan) {
					(*qinq_to_gre_lookup)[i].user = user;
					added = 1;
				}
			}
			DPPD_PANIC(!added, "Error finding correct entry in qinq_to_gre_lookup: qinq %d|%d\n", svlan, cvlan);
		}
		else {
			(*qinq_to_gre_lookup)[*qinq_to_gre_lookup_count].svlan = svlan;
			(*qinq_to_gre_lookup)[*qinq_to_gre_lookup_count].cvlan = cvlan;
			(*qinq_to_gre_lookup)[*qinq_to_gre_lookup_count].user = user;
			++(*qinq_to_gre_lookup_count);
		}
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: svlan, cvlan user\n", file->line, cfg_file);
	csv_close(file);

	return user_table_init;
}

uint8_t *read_dscp_config(const char *cfg_file, uint8_t socket)
{
	struct csv_file *file;
	int ret;
	char *fields[3];
	uint32_t dscp_bits, tc, queue;

	file = csv_open(cfg_file, 3, 3);
	DPPD_PANIC(NULL == file, "Could not open user table %s\n", cfg_file);
	uint8_t *dscp = rte_zmalloc_socket(NULL, 64, CACHE_LINE_SIZE, socket);
	DPPD_PANIC(dscp == NULL, "Error creating dscp table");

	while ((ret = csv_next(file, fields)) > 0) {
		dscp_bits = atoi(fields[0]);
		tc = atoi(fields[1]);
		queue = atoi(fields[2]);

		dscp[dscp_bits] = tc << 2 | queue;
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: dscp, tc, queue\n", file->line, cfg_file);
	csv_close(file);

	return dscp;
}

void read_cpe_table_config(struct cpe_table_entry **cpe_table_entries, uint32_t* cpe_table_entries_count, const char *cfg_file, int32_t* port_defs, uint32_t n_port_defs)
{
	struct csv_file *file, *file2;
	int ret;
	char *fields[7];
	uint32_t n_entries = 0;
	struct ipv4_subnet ip_subnet;
	struct cpe_table_entry cur_entry;
	uint32_t n_hosts;


	file = csv_open(cfg_file, 7, 7);
	DPPD_PANIC(NULL == file, "Could not open cpe table %s\n", cfg_file);
	*cpe_table_entries_count = 0;

	/* Need to figure out how much memory to allocate */
	file2 = csv_open(cfg_file, 7, 7);
	while ((ret = csv_next(file2, fields)) > 0) {
		DPPD_PANIC(0 != parse_ip_cidr(&ip_subnet, fields[4]), "Error parsing ip subnet: %s", get_parse_err());
		*cpe_table_entries_count += ip_subet_get_n_hosts(&ip_subnet);
	}
	csv_close(file2);

	*cpe_table_entries = rte_zmalloc_socket(NULL, (*cpe_table_entries_count) * sizeof(struct cpe_table_entry), CACHE_LINE_SIZE, 0);
	DPPD_PANIC(*cpe_table_entries == NULL, "Error creating cpe table entries\n");
	struct cpe_table_entry *entries = *cpe_table_entries;
	uint32_t port_idx;
	while ((ret = csv_next(file, fields)) > 0) {
		DPPD_PANIC(0 != parse_int(&port_idx, fields[0]), "Error parsing port index: %s", get_parse_err());
		DPPD_PANIC(0 != parse_int(&cur_entry.gre_id, fields[1]), "Error parsing port index: %s", get_parse_err());
		DPPD_PANIC(0 != parse_int(&cur_entry.svlan, fields[2]), "Error parsing svlan: %s", get_parse_err());
		DPPD_PANIC(0 != parse_int(&cur_entry.cvlan, fields[3]), "Error parsing cvlan: %s", get_parse_err());
		DPPD_PANIC(0 != parse_ip_cidr(&ip_subnet, fields[4]), "Error parsing ip subnet: %s", get_parse_err());
		DPPD_PANIC(0 != parse_mac(&cur_entry.eth_addr, fields[5]), "Error parsing ether addr: %s", get_parse_err());
		DPPD_PANIC(0 != parse_int(&cur_entry.user, fields[6]), "Error parsing user: %s", get_parse_err());

		DPPD_PANIC(port_idx >= n_port_defs || -1 == port_defs[port_idx], "Undefined port idx from file %d\n", port_idx);

		cur_entry.port_idx = port_defs[port_idx];
		cur_entry.svlan = rte_bswap16(cur_entry.svlan);
		cur_entry.cvlan = rte_bswap16(cur_entry.cvlan);

		n_hosts = ip_subet_get_n_hosts(&ip_subnet);

		for (uint32_t i = 0; i < n_hosts; ++i) {
			DPPD_PANIC(ip_subnet_to_host(&ip_subnet, i, &cur_entry.ip), "Invalid host in address\n");
			cur_entry.ip = rte_bswap32(cur_entry.ip);

			entries[n_entries] = cur_entry;
			n_entries++;
		}
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: port index, gre_id, svlan, cvlan, ip, mac, user", file->line, cfg_file);
	csv_close(file);
}

struct ipv6_tun_binding_table * read_ipv6_tun_bindings(const char *cfg_file, uint8_t socket)
{
	struct csv_file *file;
	int ret;
	char *fields[4];

	struct ipv6_tun_binding_table * data = rte_zmalloc_socket(NULL, sizeof(struct ipv6_tun_binding_table), CACHE_LINE_SIZE, socket);
	DPPD_PANIC(data == NULL, "Failed to alloc IPv6 Tunnel data!\n");
	memset(data, 0, sizeof(struct ipv6_tun_binding_table));

	// Read Binding Table
	file = csv_open(cfg_file, 4, 4);
	DPPD_PANIC(NULL == file, "Could not open IPv6 Tunnel lookup table %s\n", cfg_file);

	data->entry = rte_zmalloc_socket(NULL, count_lines(cfg_file, 4) * sizeof(struct ipv6_tun_binding_entry), CACHE_LINE_SIZE, socket);
	DPPD_PANIC(data->entry == NULL, "Error creating IPv6 binding table");

	int idx = 0;
	while ((ret = csv_next(file, fields)) > 0) {
		DPPD_PANIC(0 != parse_ip6(&data->entry[idx].endpoint_addr, fields[0]), "Error in routing IPv6 address: %s\n", get_parse_err());
		DPPD_PANIC(0 != parse_mac(&data->entry[idx].next_hop_mac, fields[1]), "Invalid MAC for next hop: %s\n", get_parse_err());
		DPPD_PANIC(0 != parse_ip(&data->entry[idx].public_ipv4, fields[2]), "Invalid Public IP: %s\n", get_parse_err());
		data->entry[idx].public_port = atoi(fields[3]);
		idx++;
	}
	data->num_binding_entries = idx;
	mprintf("\tRead %d IPv6 Tunnel Binding entries\n", idx);

	csv_close(file);

	return data;
}
