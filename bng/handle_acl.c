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

#include <rte_mbuf.h>
#include <rte_acl.h>
#include <rte_ip.h>
#include <rte_cycles.h>

#include "stats.h"
#include "thread_generic.h"
#include "parse_utils.h"
#include "cfgfile.h"
#include "ip_subnet.h"
#include "handle_acl.h"
#include "task_init.h"
#include "task_base.h"
#include "dppd_port_cfg.h"
#include "thread_basic.h"
#include "lconf.h"
#include "display.h"
#include "quit.h"
#include "bng_pkts.h"
#include "read_config.h"
#include "dppd_cfg.h"
#include "bng_pkts.h"
#include "prefetch.h"

struct rte_acl_field_def ipv4_defs[9] = {
	/* first input field - always one byte long. */
	{
		.type = RTE_ACL_FIELD_TYPE_BITMASK,
		.size = sizeof (uint8_t),
		.field_index = 0,
		.input_index = 0,
		.offset = offsetof (struct cpe_pkt, ipv4_hdr.next_proto_id),
	},
	/* IPv4 source address. */
	{
		.type = RTE_ACL_FIELD_TYPE_MASK,
		.size = sizeof (uint32_t),
		.field_index = 1,
		.input_index = 1,
		.offset = offsetof (struct cpe_pkt, ipv4_hdr.src_addr),
	},
	/* IPv4 destination address */
	{
		.type = RTE_ACL_FIELD_TYPE_MASK,
		.size = sizeof (uint32_t),
		.field_index = 2,
		.input_index = 2,
		.offset = offsetof (struct cpe_pkt, ipv4_hdr.dst_addr),
	},
	/* (L4 src/dst port) - 4 consecutive bytes. */
	{
		.type = RTE_ACL_FIELD_TYPE_RANGE,
		.size = sizeof (uint16_t),
		.field_index = 3,
		.input_index = 3,
		.offset = offsetof (struct cpe_pkt, udp_hdr.src_port),
	},
	{
		.type = RTE_ACL_FIELD_TYPE_RANGE,
		.size = sizeof (uint16_t),
		.field_index = 4,
		.input_index = 3,
		.offset = offsetof (struct cpe_pkt, udp_hdr.dst_port),
	},
	/* (SVLAN id + eth type) - 4 consecutive bytes. */
	{
		.type = RTE_ACL_FIELD_TYPE_BITMASK,
		.size = sizeof(uint16_t),
		.field_index = 5,
		.input_index = 4,
		.offset = offsetof (struct cpe_pkt, qinq_hdr.svlan.eth_proto),
	},
	{
		.type = RTE_ACL_FIELD_TYPE_BITMASK,
		.size = sizeof(uint16_t),
		.field_index = 6,
		.input_index = 4,
		.offset = offsetof (struct cpe_pkt, qinq_hdr.svlan.vlan_tci),
	},
	/* (CVLAN id + eth type) - 4 consecutive byates. */
	{
		.type = RTE_ACL_FIELD_TYPE_BITMASK,
		.size = sizeof(uint16_t),
		.field_index = 7,
		.input_index = 5,
		.offset = offsetof (struct cpe_pkt, qinq_hdr.cvlan.eth_proto),
	},
	{
		.type = RTE_ACL_FIELD_TYPE_BITMASK,
		.size = sizeof(uint16_t),
		.field_index = 8,
		.input_index = 5,
		.offset = offsetof (struct cpe_pkt, qinq_hdr.cvlan.vlan_tci),
	},
};

int read_rules_config(struct rte_acl_ctx* ctx, const char *cfg_file, uint32_t n_max_rules, int use_qinq)
{
	struct csv_file *file;
	int ret;
	char *fields[8];
	uint32_t n_rules = 0;
	char class_str[24];

	file = csv_open_delim(cfg_file, 8, 8, ' ');
	DPPD_PANIC(NULL == file, "Could not open ACL rules %s\n", cfg_file);

	while ((ret = csv_next(file, fields)) > 0) {
		DPPD_PANIC(n_rules++ > n_max_rules, "Too many rules in file %s. Maximum allowed is %d\n", cfg_file, n_max_rules);
		struct acl4_rule rule;
		str_to_rule(&rule, fields, n_rules, use_qinq);
		rte_acl_add_rules(ctx, (struct rte_acl_rule*) &rule, 1);
	}
	DPPD_PANIC(ret < 0, "Error at line %d while reading %s:\n\tExpecting format: svlan, cvlan, IP proto, src IP, dst IP, src port, dst port, class ", file->line, cfg_file);
	csv_close(file);
	return n_rules;
}

struct task_acl {
	struct task_base base;
	struct rte_acl_ctx *context;
	const uint8_t *ptuples[64];
	uint32_t       n_rules;
	uint32_t       n_max_rules;
};

static void set_tc(struct rte_mbuf *mbuf, uint32_t tc)
{
#if RTE_VERSION >= RTE_VERSION_NUM(1,8,0,0)
	struct rte_sched_port_hierarchy *sched =
		(struct rte_sched_port_hierarchy *) &mbuf->hash.sched;
#else
	struct rte_sched_port_hierarchy *sched =
		(struct rte_sched_port_hierarchy *) &mbuf->pkt.hash.sched;
#endif
	sched->traffic_class = tc;
}

static void handle_acl_bulk(struct task_base *tbase, struct rte_mbuf **mbufs, uint16_t n_pkts)
{
	struct task_acl *task = (struct task_acl *)tbase;
	uint32_t results[64];
	uint16_t j;

#ifdef BRAS_PREFETCH_OFFSET
	for (j = 0; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
		PREFETCH0(mbufs[j]);
	}
	for (j = 1; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j - 1], void *));
	}
#endif
	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(mbufs[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(mbufs[j + PREFETCH_OFFSET - 1], void *));
#endif
		/* TODO: detect version_ihl != 0x45. Extract relevant
		   fields of that packet and point ptuples[j] to the
		   extracted verion. Note that this is very unlikely. */
		task->ptuples[j] = rte_pktmbuf_mtod(mbufs[j], uint8_t *);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(mbufs[n_pkts - 1], void *));
	for (; j < n_pkts; ++j) {
		task->ptuples[j] = rte_pktmbuf_mtod(mbufs[j], uint8_t *);
	}
#endif

	rte_acl_classify(task->context, (const uint8_t **)task->ptuples, results, n_pkts, 1);

	struct rte_mbuf* to_send[64];
	uint16_t idx = 0;
	for (uint8_t i = 0; i < n_pkts; ++i) {

		switch (results[i]) {
		case 0:
		case 1:
			rte_pktmbuf_free(mbufs[i]);
			INCR_TX_DROP_COUNT(task->base.stats, 1);
			break;
		case 2:
			set_tc(mbufs[i], 3);
		case 3:
			to_send[idx] = mbufs[i];
			idx++;
			break;
		};
	}
	if (idx)
		task->base.tx_pkt_no_buf(&task->base, to_send, idx);
}

static void acl_msg(struct task_base *tbase, void **data, uint16_t n_msgs)
{
	struct task_acl *task = (struct task_acl *)tbase;
	struct acl4_rule **new_rules = (struct acl4_rule **)data;
	uint16_t i;

	for (i = 0; i < n_msgs; ++i) {
		if (task->n_rules == task->n_max_rules) {
			mprintf("Failed to add %d rule%s (already at maximum number of rules (%d))",
				n_msgs - i, (n_msgs - i)? "s" : "", task->n_max_rules);
			break;
		}

		new_rules[i]->data.priority = ++task->n_rules;
		rte_acl_add_rules(task->context, (struct rte_acl_rule*) new_rules[i], 1);
	}

	/* No need to rebuild if no rules have been added */
	if (!i) {
		return ;
	}

	struct rte_acl_config acl_build_param;
	/* Perform builds */
	acl_build_param.num_categories = 1;

	acl_build_param.num_fields = RTE_DIM(ipv4_defs);
	rte_memcpy(&acl_build_param.defs, ipv4_defs, sizeof(ipv4_defs));

	int ret;
	DPPD_PANIC((ret = rte_acl_build(task->context, &acl_build_param)),
		   "Failed to build ACL trie (%d)\n", ret);
}

static void init_task_acl(struct task_base *tbase, struct task_args *targ)
{
	struct task_acl *task = (struct task_acl *)tbase;

	tbase->tx_pkt_no_buf = no_buf_tx_function(targ);
	tbase->flags |= FLAG_NEVER_FLUSH;

	char name[PATH_MAX];
	struct rte_acl_param acl_param;

	/* Create ACL contexts */
	snprintf(name, sizeof(name), "acl-%d-%d", targ->lconf->id, targ->task);

	int dim = RTE_DIM(ipv4_defs);
	acl_param.name = name;
	acl_param.socket_id = rte_lcore_to_socket_id(targ->lconf->id);
	acl_param.rule_size = RTE_ACL_RULE_SZ(dim);
	acl_param.max_rule_num = targ->n_max_rules;

	task->n_max_rules = targ->n_max_rules;
	task->context = rte_acl_create(&acl_param);

	int use_qinq = targ->flags & TASK_ARG_QINQ_ACL;

	if (!use_qinq) {
		for (uint8_t i = 0; i < 5; ++i) {
			ipv4_defs[i].offset -= 2*sizeof(struct vlan_hdr);
		}
	}

	DPPD_PANIC(task->context == NULL, "Failed to create ACL context\n");
	task->n_rules = read_rules_config(task->context, dppd_cfg.path_acl_cfg, targ->n_max_rules, use_qinq);

	mprintf("Configured %d rules\n", task->n_rules);

	if (task->n_rules) {
		struct rte_acl_config acl_build_param;
		/* Perform builds */
		acl_build_param.num_categories = 1;

		acl_build_param.num_fields = dim;
		rte_memcpy(&acl_build_param.defs, ipv4_defs, sizeof(ipv4_defs));

		mprintf("Building trie structure\n");
		DPPD_PANIC(rte_acl_build(task->context, &acl_build_param),
			   "Failed to build ACL trie\n");
	}

	/* Restore fields */
	if (!use_qinq) {
		for (uint8_t i = 0; i < 5; ++i) {
			ipv4_defs[i].offset += 2*sizeof(struct vlan_hdr);
		}
	}

	targ->lconf->ctrl_timeout = rte_get_tsc_hz()/1000;
	targ->lconf->ctrl_func_m[targ->task] = acl_msg;
}

int str_to_rule(struct acl4_rule *rule, char** fields, int n_rules, int use_qinq)
{
	uint32_t svlan, svlan_mask;
	uint32_t cvlan, cvlan_mask;

	uint32_t ip_proto, ip_proto_mask;

	struct ipv4_subnet ip_src;
	struct ipv4_subnet ip_dst;

	uint32_t sport_lo, sport_hi;
	uint32_t dport_lo, dport_hi;

	uint32_t class = 0;
	char class_str[24];

	DPPD_PANIC(parse_int_mask(&svlan, &svlan_mask, fields[0]), "Error parsing svlan: %s\n", get_parse_err());
	DPPD_PANIC(parse_int_mask(&cvlan, &cvlan_mask, fields[1]), "Error parsing cvlan: %s\n", get_parse_err());
	DPPD_PANIC(parse_int_mask(&ip_proto, &ip_proto_mask, fields[2]), "Error parsing ip protocol: %s\n", get_parse_err());
	DPPD_PANIC(parse_ip_cidr(&ip_src, fields[3]), "Error parsing source IP subnet: %s\n", get_parse_err());
	DPPD_PANIC(parse_ip_cidr(&ip_dst, fields[4]), "Error parsing dest IP subnet: %s\n", get_parse_err());

	DPPD_PANIC(parse_range(&sport_lo, &sport_hi, fields[5]), "Error parsing source port range: %s\n", get_parse_err());
	DPPD_PANIC(parse_range(&dport_lo, &dport_hi, fields[6]), "Error parsing destination port range: %s\n", get_parse_err());

	DPPD_PANIC(parse_str(class_str, fields[7], sizeof(class_str)), "Error parsing action: %s\n", get_parse_err());

	if (!strcmp(class_str, "drop")) {
		class = 1;
	}
	else if (!strcmp(class_str, "allow")) {
		class = 2;
	}
	else if (!strcmp(class_str, "rate limit")) {
		class = 3;
	}
	else {
		mprintf("unknown class type: %s\n", class_str);
	}

	rule->data.userdata = class; /* allow, drop or ratelimit */
	rule->data.category_mask = 1;
	rule->data.priority = n_rules;

	/* Configuration for rules is done in little-endian so no bswap is needed here.. */

	rule->fields[0].value.u8 = ip_proto;
	rule->fields[0].mask_range.u8 = ip_proto_mask;
	rule->fields[1].value.u32 = ip_src.ip;
	rule->fields[1].mask_range.u32 = ip_src.prefix;

	rule->fields[2].value.u32 = ip_dst.ip;
	rule->fields[2].mask_range.u32 = ip_dst.prefix;

	rule->fields[3].value.u16 = sport_lo;
	rule->fields[3].mask_range.u16 = sport_hi;

	rule->fields[4].value.u16 = dport_lo;
	rule->fields[4].mask_range.u16 = dport_hi;

	if (use_qinq) {
		rule->fields[5].value.u16 = rte_bswap16(ETYPE_8021ad);
		rule->fields[5].mask_range.u16 = 0xffff;

		/* To mask out the TCI and only keep the VID, the mask should be 0x0fff */
		rule->fields[6].value.u16 = svlan;
		rule->fields[6].mask_range.u16 = svlan_mask;

		rule->fields[7].value.u16 = rte_bswap16(ETYPE_VLAN);
		rule->fields[7].mask_range.u16 = 0xffff;

		rule->fields[8].value.u16 = cvlan;
		rule->fields[8].mask_range.u16 = cvlan_mask;
	} else {
		/* Reuse first ethertype from vlan to check if packet is IPv4 packet */
		rule->fields[5].value.u16 =  rte_bswap16(ETYPE_IPv4);
		rule->fields[5].mask_range.u16 = 0xffff;

		/* Other fields are ignored */
		rule->fields[6].value.u16 = 0;
		rule->fields[6].mask_range.u16 = 0;
		rule->fields[7].value.u16 = 0;
		rule->fields[7].mask_range.u16 = 0;
		rule->fields[8].value.u16 = 0;
		rule->fields[8].mask_range.u16 = 0;
	}
	return 0;
}

struct task_init task_init_acl = {
	.mode_str = "acl",
	.init = init_task_acl,
	.handle = handle_acl_bulk,
	.thread_x = thread_generic,
	.size = sizeof(struct task_acl)
};

void reg_task_acl(void)
{
	reg_task(&task_init_acl);
}
