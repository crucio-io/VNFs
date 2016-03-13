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

#include <unistd.h>

#include <rte_sched.h>
#include <rte_string_fns.h>

#include "version.h"
#include "defines.h"
#include "dppd_args.h"
#include "dppd_assert.h"
#include "dppd_cfg.h"
#include "cfgfile.h"
#include "display.h"
#include "parse_utils.h"
#include "thread_none.h"
#include "thread_generic.h"
#include "thread_basic.h"
#include "thread_qos.h"
#include "dppd_port_cfg.h"
#include "defaults.h"

#define MAX_RTE_ARGV 64
#define MAX_ARG_LEN  32

/* Helper macro */
#define STR_EQ(s1, s2)	(!strcmp((s1), (s2)))

/* configuration files support */
static int get_rte_cfg(unsigned sindex, char *str, void *data);
static int get_global_cfg(unsigned sindex, char *str, void *data);
static int get_port_cfg(unsigned sindex, char *str, void *data);
static int get_defaults_cfg(unsigned sindex, char *str, void *data);
static int get_var_cfg(unsigned sindex, char *str, void *data);
static int get_core_cfg(unsigned sindex, char *str, void *data);

static struct cfg_section eal_default_cfg = {
	.name   = "eal options",
	.parser = get_rte_cfg,
	.data   = &rte_cfg,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section port_cfg = {
	.name   = "port #",
	.parser = get_port_cfg,
	.data   = &dppd_port_cfg,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section var_cfg = {
	.name   = "variables",
	.parser = get_var_cfg,
	.data   = 0,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section defaults_cfg = {
	.name   = "defaults",
	.parser = get_defaults_cfg,
	.data   = 0,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section settings_cfg = {
	.name   = "global",
	.parser = get_global_cfg,
	.data   = &dppd_cfg,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section core_cfg = {
	.name   = "core #",
	.parser = get_core_cfg,
	.data   = lcore_cfg_init,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static const char *cfg_file = DEFAULT_CONFIG_FILE;
struct rte_cfg    rte_cfg;
/* contains the source mac address for all the
   interfaces to be set when transmitting packets */
struct ether_addr if_cfg[DPPD_MAX_PORTS];
struct lcore_cfg *lcore_cfg;
// only used at initialization time
struct lcore_cfg  lcore_cfg_init[RTE_MAX_LCORE];

static char format_err_str[256];
static const char *err_str = "Unknown error";

static void set_errf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	vsnprintf(format_err_str, sizeof(format_err_str), format, ap);
	va_end(ap);
	err_str = format_err_str;
}

/* [eal options] parser */
static int get_rte_cfg(__attribute__((unused))unsigned sindex, char *str, void *data)
{
	struct rte_cfg *pconfig = (struct rte_cfg *)data;

	if (str == NULL || pconfig == NULL) {
		return -1;
	}

	char *pkey = get_cfg_key(str);
	if (pkey == NULL) {
		set_errf("Missing key after option");
		return -1;
	}

	if (STR_EQ(str, "-m")) {
		pconfig->memory = atoi(pkey);
		return 0;
	}
	if (STR_EQ(str, "-n")) {
		pconfig->force_nchannel = atoi(pkey);
		if (pconfig->force_nchannel == 0 || pconfig->force_nchannel > 4) {
			set_errf("Invalid number of memory channels");
			return -1;
		}
		return 0;
	}
	if (STR_EQ(str, "-r")) {
		pconfig->force_nrank = atoi(pkey);
		if (pconfig->force_nrank == 0 || pconfig->force_nrank > 16) {
			set_errf("Invalid number of memory ranks");
			return -1;
		}
		return 0;
	}
	/* debug options */
	if (STR_EQ(str, "no-pci")) {
		return parse_bool(&pconfig->no_pci, pkey);
	}
	if (STR_EQ(str, "no-hpet")) {
		return parse_bool(&pconfig->no_hpet, pkey);
	}
	if (STR_EQ(str, "no-shconf")) {
		return parse_bool(&pconfig->no_shconf, pkey);
	}
	if (STR_EQ(str, "no-huge")) {
		return parse_bool(&pconfig->no_hugetlbfs, pkey);
	}
	if (STR_EQ(str, "no-output")) {
		return parse_bool(&pconfig->no_output, pkey);
	}

	if (STR_EQ(str, "huge-dir")) {
		if (pconfig->hugedir) {
			free(pconfig->hugedir);
		}
		pconfig->hugedir = strdup(pkey);
		return 0;
	}

	if (STR_EQ(str, "eal")) {
		if (pconfig->eal) {
			free(pconfig->eal);
		}
		pconfig->eal = strdup(pkey);
		return 0;
	}


	/* fail on unknown keys */
	return -1;
}

/* [global] parser */
static int get_global_cfg(__attribute__((unused))unsigned sindex, char *str, void *data)
{
	struct dppd_cfg *pset = (struct dppd_cfg *)data;

	if (str == NULL || pset == NULL) {
		return -1;
	}

	char *pkey = get_cfg_key(str);
	if (pkey == NULL) {
		set_errf("Missing key after option");
		return -1;
	}

	if (STR_EQ(str, "version")) {
		if (2 != sscanf(pkey, "%d.%d", &pset->cfg_version_major, &pset->cfg_version_minor)) {
			set_errf("Invalid version format (expected x.y where x is the major version and y is the minor version)");
			return -1;
		}
		if (pset->cfg_version_major != VERSION_MAJOR ||
		    pset->cfg_version_minor != VERSION_MINOR) {
			set_errf("Configuration version mismatch (expected %d.%d but specified version was %d.%d)",
				 pset->cfg_version_major, pset->cfg_version_minor,
				 VERSION_MAJOR, VERSION_MINOR);
			return -1;
		}

		return 0;
	}

	if (pset->cfg_version_major == 0 && pset->cfg_version_minor == 0) {
		set_errf("First line in global section needs to specify version, for example: version=%d.%d", VERSION_MAJOR, VERSION_MINOR);
		return -1;
	}

	if (STR_EQ(str, "name")) {
		return parse_str(pset->name, pkey, sizeof(pset->name));
	}

	if (STR_EQ(str, "start time")) {
		return parse_int(&pset->start_time, pkey);
	}

	if (STR_EQ(str, "duration time")) {
		return parse_int(&pset->duration_time, pkey);
	}

	if (STR_EQ(str, "virtualization")) {
		return parse_flag(&pset->flags, TGSF_USE_VF, pkey);
	}

	if (STR_EQ(str, "shuffle")) {
		return parse_flag(&pset->flags, TGSF_SHUFFLE, pkey);
	}

	if (STR_EQ(str, "wait on quit")) {
		return parse_flag(&pset->flags, TGSF_WAIT_ON_QUIT, pkey);
	}

	if (STR_EQ(str, "cpe table map")) {
		/* The config defined ports through 0, 1, 2 ... which
		   need to be associated with ports. This is done
		   through defining it using "cpe table map=" */
		return parse_port_name_list((uint32_t*)pset->cpe_table_ports, NULL, DPPD_MAX_PORTS, pkey);
	}

	if (STR_EQ(str, "qinq_tag")) {
		return parse_int(&pset->QinQ_tag, pkey);
	}

	if (STR_EQ(str, "gre cfg")) {
		return parse_path(pset->path_gre_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "user cfg")) {
		return parse_path(pset->path_user_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "next hop cfg")) {
		return parse_path(pset->path_next_hop_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "ipv4 cfg")) {
		return parse_path(pset->path_ipv4_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "ipv6 cfg")) {
		return parse_path(pset->path_ipv6_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "dscp cfg")) {
		return parse_path(pset->path_dscp_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "cpe cfg")) {
		return parse_path(pset->path_cpe_table_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "ipv6 tunnel cfg")) {
		return parse_path(pset->path_ipv6_tunnel_cfg, pkey, MAX_PATH_LEN);
	}

	if (STR_EQ(str, "acl cfg")) {
		return parse_path(pset->path_acl_cfg, pkey, MAX_PATH_LEN);
	}

	/* fail on unknown keys */
	return -1;
}

/* [variable] parser */
static int get_var_cfg(__attribute__((unused)) unsigned sindex, char *str, __attribute__((unused)) void *data)
{
	return add_var(str, get_cfg_key(str), 0);
}

/* [defaults] parser */
static int get_defaults_cfg(__attribute__((unused)) unsigned sindex, char *str, __attribute__((unused)) void *data)
{
	uint32_t val;
	char *pkey;

	pkey = get_cfg_key(str);
	if (pkey == NULL) {
		set_errf("Missing key after option");
		return -1;
	}

	if (STR_EQ(str, "mempool size")) {

		if (parse_kmg(&val, pkey)) {
			return -1;
		}

		for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
			struct lcore_cfg *cur_lcore_cfg_init = &lcore_cfg_init[lcore_id];
			cur_lcore_cfg_init->id = lcore_id;
			for (uint8_t task_id = 0; task_id < MAX_TASKS_PER_CORE; ++task_id) {
				struct task_args *targ = &cur_lcore_cfg_init->targs[task_id];
				targ->nb_mbuf = val;
			}
		}
		return 0;
	}

	if (STR_EQ(str, "memcache size")) {

		if (parse_kmg(&val, pkey)) {
			return -1;
		}

		for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
			struct lcore_cfg *cur_lcore_cfg_init = &lcore_cfg_init[lcore_id];
			cur_lcore_cfg_init->id = lcore_id;
			for (uint8_t task_id = 0; task_id < MAX_TASKS_PER_CORE; ++task_id) {
				struct task_args *targ = &cur_lcore_cfg_init->targs[task_id];
				targ->nb_cache_mbuf = val;
			}
		}
		return 0;
	}

	set_errf("Option '%s' is not known", str);
	return -1;
}

/* [port] parser */
static int get_port_cfg(unsigned sindex, char *str, void *data)
{
	struct dppd_port_cfg *cfg = (struct dppd_port_cfg *)data;

	uint8_t cur_if = sindex & ~CFG_INDEXED;

	if (cur_if >= DPPD_MAX_PORTS) {
		set_errf("Port ID is too high (max allowed %d)", DPPD_MAX_PORTS - 1 );
		return -1;
	}

	cfg = &dppd_port_cfg[cur_if];

	if (str == NULL || data == NULL) {
		return -1;
	}

	char *pkey = get_cfg_key(str);

	if (pkey == NULL) {
		set_errf("Missing key after option");
		return -1;
	}


	if (STR_EQ(str, "mac")) {
		if (STR_EQ(pkey, "hardware")) {
			cfg->type = DPPD_PORT_MAC_HW;
		}
		else if (STR_EQ(pkey, "random")) {
			cfg->type = DPPD_PORT_MAC_RAND;
		}
		else {
			cfg->type = DPPD_PORT_MAC_SET;
			if (parse_mac(&cfg->eth_addr, pkey)) {
				return -1;
			}
		}
	}
	else if (STR_EQ(str, "name")) {
		uint32_t val;
		strncpy(cfg->name, pkey, MAX_NAME_SIZE);
		DPPD_ASSERT(cur_if < DPPD_MAX_PORTS);
		return add_port_name(cur_if, pkey);
	}
	else if (STR_EQ(str, "rx desc")) {
		return parse_int(&cfg->n_rxd, pkey);
	}
	else if (STR_EQ(str, "tx desc")) {
		return parse_int(&cfg->n_txd, pkey);
	}
	else if (STR_EQ(str, "promiscuous")) {
		uint32_t val;
		if (parse_bool(&val, pkey)) {
			return -1;
		}
		cfg->promiscuous = val;
	}
	else if (STR_EQ(str, "strip crc")) {
		uint32_t val;
		if (parse_bool(&val, pkey)) {
			return -1;
		}
		cfg->port_conf.rxmode.hw_strip_crc = val;
	}
	else if (STR_EQ(str, "rss")) {
		uint32_t val;
		if (parse_bool(&val, pkey)) {
			return -1;
		}
		if (val) {
			cfg->port_conf.rxmode.mq_mode = ETH_MQ_RX_RSS;
			cfg->port_conf.rx_adv_conf.rss_conf.rss_hf = ETH_RSS_IPV4;
		}
	}
	else if (STR_EQ(str, "rx_ring")) {
		parse_str(cfg->rx_ring, pkey, sizeof(cfg->rx_ring));
	}
	else if (STR_EQ(str, "tx_ring")) {
		parse_str(cfg->tx_ring, pkey, sizeof(cfg->tx_ring));
	}

	return 0;
}

static enum police_action str_to_color(const char *str)
{
	if (STR_EQ(str, "green"))
		return ACT_GREEN;
	if (STR_EQ(str, "yellow"))
		return ACT_YELLOW;
	if (STR_EQ(str, "red"))
		return ACT_RED;
	if (STR_EQ(str, "drop"))
		return ACT_DROP;
	return ACT_INVALID;
}

/* [core] parser */
static int get_core_cfg(unsigned sindex, char *str, void *data)
{
	char *pkey;
	struct lcore_cfg *lconf = (struct lcore_cfg *)data;

	if (str == NULL || lconf == NULL || !(sindex & CFG_INDEXED)) {
		return -1;
	}

	pkey = get_cfg_key(str);
	if (pkey == NULL) {
		set_errf("Missing key after option");
		return -1;
	}

	uint32_t ncore = sindex & ~CFG_INDEXED;
	if (ncore >= RTE_MAX_LCORE) {
		set_errf("Core index too high (max allowed %d)", RTE_MAX_LCORE - 1);
		return -1;
	}

	lconf = &lconf[ncore];

	if (STR_EQ(str, "task")) {

		uint32_t val = atoi(pkey);
		if (val >= MAX_TASKS_PER_CORE) {
			set_errf("Too many tasks for core (max allowed %d)", MAX_TASKS_PER_CORE - 1);
			return -1;
		}
		if (val != lconf->nb_tasks) {
			set_errf("Task ID skipped or defined twice");
			return -1;
		}

		lconf->active_task = val;

		lconf->targs[lconf->active_task].task = lconf->active_task;

		if (lconf->nb_tasks < lconf->active_task + 1) {
			lconf->nb_tasks = lconf->active_task + 1;
		}
		return 0;
	}

	struct task_args *targ = &lconf->targs[lconf->active_task];
	if (STR_EQ(str, "tx ports from routing table")) {
		uint32_t vals[DPPD_MAX_PORTS];
		uint32_t n_if;
		if (!(targ->task_init->flag_features & TASK_ROUTING)) {
			set_errf("tx port form route not supported mode %s",  targ->task_init->mode_str);
			return -1;
		}

		if (parse_port_name_list(vals, &n_if, DPPD_MAX_PORTS, pkey)) {
			return -1;
		}

		for (uint8_t i = 0; i < n_if; ++i) {
			dppd_port_cfg[vals[i]].active = 1;
			targ->tx_port_queue[i].port = vals[i];
			targ->nb_txports++;
		}
		targ->runtime_flags |= TASK_ROUTING;
		return 0;
	}
	if (STR_EQ(str, "tx ports from cpe table")) {
		uint32_t vals[DPPD_MAX_PORTS];
		int n_remap = -1;
		uint32_t ret;
		uint32_t val;
		char* mapping_str = strstr(pkey, " remap=");

		if (mapping_str != NULL) {
			*mapping_str = 0;
			*mapping_str += strlen(" remap=");
			n_remap = parse_remap(targ->mapping, mapping_str);
		}

		if (parse_port_name_list(vals, &ret, DPPD_MAX_PORTS, pkey)) {
			return -1;
		}


		if (n_remap != -1 && ret != (uint32_t)n_remap) {
			set_errf("Expected %d remap elements but had %d", n_remap, ret);
			return -1;
		}

		for (uint8_t i = 0; i < ret; ++i) {
			targ->tx_port_queue[i].port = vals[i];

			/* default mapping this case is port0 -> port0 */
			if (n_remap == -1) {
				targ->mapping[vals[i]] = i;
			}
		}

		targ->nb_txports = ret;

		return 0;
	}
	if (STR_EQ(str, "tx cores from routing table")) {
		if (!(targ->task_init->flag_features & TASK_ROUTING)) {
			set_errf("tx port form route not supported mode %s",  targ->task_init->mode_str);
			return -1;
		}

		struct thread_list_cfg *thread_list_cfg = &targ->thread_list[0];
		thread_list_cfg->dest_task = 0;
		thread_list_cfg->active = 1;
		int ret;

		if (STR_EQ(pkey, "self")) {
			ret = 1;
			thread_list_cfg->thread_id[0] = lconf->id;
		}
		else {
			ret = parse_list_set(thread_list_cfg->thread_id, pkey, MAX_WT_PER_LB);
			if (ret < 0) {
				return -1;
			}
		}

		thread_list_cfg->nb_threads = ret;
		targ->nb_worker_threads = ret;
		targ->nb_txrings = ret;
		uint32_t max_allowed = MAX_RINGS_PER_CORE > DPPD_MAX_PORTS? DPPD_MAX_PORTS: MAX_RINGS_PER_CORE;
		if (targ->nb_txrings > max_allowed) {
			set_errf("Maximum allowed TX rings is %u but have %u", max_allowed, targ->nb_txrings);
			return -1;
		}
		else if (thread_list_cfg->nb_threads > MAX_WT_PER_LB) {
			set_errf("Maximum worker threads allowed is %u but have %u", MAX_WT_PER_LB, thread_list_cfg->nb_threads);
			return -1;
		}

		targ->runtime_flags |= TASK_ROUTING;
		return 0;
	}
	if (STR_EQ(str, "tx cores from cpe table")) {
		struct thread_list_cfg *thread_list_cfg =  &targ->thread_list[0];
		int ret, ret2;
		char *mapping_str;

		mapping_str = strstr(pkey, " remap=");
		if (mapping_str == NULL) {
			set_errf("There is no default mapping for tx cores from cpe table. Please specify it through remap=");
			return -1;
		}
		*mapping_str = 0;
		mapping_str += strlen(" remap=");
		ret = parse_remap(targ->mapping, mapping_str);
		if (ret <= 0) {
			return -1;
		}

		ret2 = parse_list_set(thread_list_cfg->thread_id, pkey, MAX_RINGS_PER_CORE);
		if (ret2 <= 0) {
			return -1;
		}
		else if (ret2 > MAX_WT_PER_LB) {
			set_errf("Maximum cores to route to is %u\n", MAX_WT_PER_LB);
			return -1;
		}
		thread_list_cfg->nb_threads = ret2;
		targ->nb_txrings = ret2;
		thread_list_cfg->active = 1;

		if (ret != ret2) {
			set_errf("Expecting same number of remaps as cores\n", str);
			return -1;
		}
		return 0;
	}

	if (STR_EQ(str, "cpe table timeout ms")) {
		return parse_int(&targ->cpe_table_timeout_ms, pkey);
	}

	if (STR_EQ(str, "handle arp")) {
		return parse_flag(&targ->runtime_flags, TASK_HANDLE_ARP, pkey);
	}

	if (STR_EQ(str, "do sig")) {
		return parse_flag(&targ->runtime_flags, TASK_DO_SIG, pkey);
	}

	/* Using tx port name, only a _single_ port can be assigned to a task. */
	if (STR_EQ(str, "tx port")) {
		if (targ->nb_txports) {
			set_errf("Only one tx port can be defined per task. Use a LB task or routing instead.");
			return -1;
		}
		uint32_t val;
		if (parse_port_name(&val, pkey)) {
			return -1;
		}
		DPPD_ASSERT(val < DPPD_MAX_PORTS);
		targ->tx_port_queue[0].port = val;
		dppd_port_cfg[val].active = 1;
		targ->nb_txports = 1;
		return 0;
	}
	if (STR_EQ(str, "rx ring")) {
		uint32_t val;
		int err = parse_bool(&val, pkey);
		if (!err && val && targ->rx_port != NO_PORT_AVAIL) {
			set_errf("Can't read both from internal ring and external port from the same task. Use multiple tasks instead.");
			return -1;
		}

		return parse_flag(&targ->flags, TASK_ARG_RX_RING, pkey);
	}

	if (STR_EQ(str, "local lpm")) {
		return parse_flag(&targ->flags, TASK_ARG_LOCAL_LPM, pkey);
	}
	if (STR_EQ(str, "lpm size")) {
		return parse_int(&targ->lpm_size, pkey);
	}
	if (STR_EQ(str, "drop")) {
		return parse_flag(&targ->flags, TASK_ARG_DROP, pkey);
	}
	if (STR_EQ(str, "qinq")) {
		return parse_flag(&targ->flags, TASK_ARG_QINQ_ACL, pkey);
	}

	if (STR_EQ(str, "rx port")) {
		if (targ->flags & TASK_ARG_RX_RING) {
			set_errf("Can't read both from internal ring and external port from the same task. Use multiple tasks instead.");
			return -1;
		}
		uint32_t val;
		if (parse_port_name(&val, pkey)) {
			return -1;
		}
		DPPD_ASSERT(val < DPPD_MAX_PORTS);
		dppd_port_cfg[val].active = 1;
		targ->rx_port = val;
		return 0;
	}

	if (STR_EQ(str, "mode")) {

		/* master is a special mode that is always needed (cannot be turned off) */
		if (STR_EQ(pkey, "master")) {
			dppd_cfg.master = ncore;
			targ->mode = MASTER;
			if (lconf->nb_tasks > 1 || targ->task != 0) {
				set_errf("Master core can only have one task\n");
				return -1;
			}
			return 0;
		}

		struct task_init* task_init = to_task_init(pkey, "");
		if (task_init) {
			targ->mode = task_init->mode;
		}
		else {
			set_errf("Task mode '%s' is invalid", pkey);
			return -1;
		}
		targ->task_init = task_init;
		lconf->thread_x = task_init->thread_x;
		return 0;
	}
	if (STR_EQ(str, "users")) {
		return parse_int(&targ->n_users, pkey);
	}

	if (STR_EQ(str, "network side")) {
		return parse_flag(&targ->flags, TASK_ARG_INET_SIDE, pkey);
	}

	if (STR_EQ(str, "mark")) {
		return parse_flag(&targ->runtime_flags, TASK_MARK, pkey);
	}

	if (STR_EQ(str, "mark green")) {
		return parse_int(&targ->marking[0], pkey);
	}

	if (STR_EQ(str, "mark yellow")) {
		return parse_int(&targ->marking[1], pkey);
	}

	if (STR_EQ(str, "mark red")) {
		return parse_int(&targ->marking[2], pkey);
	}

	if (STR_EQ(str, "tx cores")) {
		uint8_t dest_task = 0;
		/* if user did not specify, dest_port is left at default (first type) */
		uint8_t dest_proto = 0;
		uint8_t ctrl = CTRL_TYPE_DP;
		char *task_str = strstr(pkey, "proto=");
		if (task_str) {
			task_str += strlen("proto=");

			if (STR_EQ(task_str, "ipv4")) {
				dest_proto = IPV4;
			}
			else if (STR_EQ(task_str, "arp")) {
				dest_proto = ARP;
			}
			else if (STR_EQ(task_str, "ipv6")) {
				dest_proto = IPV6;
			}
			else {
				set_errf("proto needs to be either ipv4, arp or ipv6");
				return -1;
			}

		}

		task_str = strstr(pkey, "task=");

		if (task_str) {
			--task_str;
			*task_str = 0;
			task_str++;
			task_str += strlen("task=");
			char *task_str_end = strstr(task_str, " ");
			if (task_str_end) {
				*task_str_end = 0;
			}
			if (0 == strlen(task_str)) {
				set_errf("Invalid task= syntax");
				return -1;
			}

			switch (task_str[strlen(task_str) - 1]) {
			case 'p':
				ctrl = CTRL_TYPE_PKT;
				break;
			case 'm':
				ctrl = CTRL_TYPE_MSG;
				break;
			case '\n':
			case 0:
				break;
			default:
				if (task_str[strlen(task_str) -1] < '0' ||
				    task_str[strlen(task_str) -1] > '9') {
					set_errf("Unknown ring type %c.\n",
						 task_str[strlen(task_str) - 1]);
					return -1;
				}
			}

			dest_task = atoi(task_str);
			if (dest_task >= MAX_TASKS_PER_CORE) {
				set_errf("Destination task too high (max allowed %d)", MAX_TASKS_PER_CORE - 1);
				return -1;
			}
		}
		else {
			dest_task = 0;
		}

		struct thread_list_cfg *thread_list_cfg = &targ->thread_list[dest_proto];
		thread_list_cfg->dest_task = dest_task;
		thread_list_cfg->active = 1;
		thread_list_cfg->type = ctrl;

		int ret;

		if (STR_EQ(pkey, "self")) {
			ret = 1;
			thread_list_cfg->thread_id[0] = lconf->id;
		}
		else {
			uint32_t max = MAX_RINGS_PER_CORE > MAX_WT_PER_LB? MAX_RINGS_PER_CORE: MAX_WT_PER_LB;

			ret = parse_list_set(thread_list_cfg->thread_id, pkey, max);
			if (ret < 0) {
				return -1;
			}
		}

		thread_list_cfg->nb_threads = ret;
		targ->nb_worker_threads = ret;
		targ->nb_txrings += ret;

		return 0;
	}
	if (STR_EQ(str, "ring size")) {
		return parse_int(&targ->ring_size, pkey);
	}

	if (STR_EQ(str, "mempool size")) {
		return parse_kmg(&targ->nb_mbuf, pkey);
	}

	if (STR_EQ(str, "memcache size")) {
		return parse_kmg(&targ->nb_cache_mbuf, pkey);
	}

	if (STR_EQ(str, "byte offset")) {
		return parse_int(&targ->byte_offset, pkey);
	}

	if (STR_EQ(str, "name")) {
		return parse_str(lconf->name, pkey, sizeof(lconf->name));
	}
	/* MPLS configuration */
	if (STR_EQ(str, "untag mpls")) {
		return parse_flag(&targ->runtime_flags, TASK_MPLS_TAGGING, pkey);
	}

	if (STR_EQ(str, "add mpls")) {
		return parse_flag(&targ->runtime_flags, TASK_MPLS_TAGGING, pkey);
	}

	if (STR_EQ(str, "sub mode")) {
		const char* mode_str = targ->task_init->mode_str;
		const char *sub_mode_str = pkey;

		targ->task_init = to_task_init(mode_str, sub_mode_str);
		if (!targ->task_init) {
			set_errf("sub mode %s not supported for mode %s", sub_mode_str, mode_str);
			return -1;
		}
		return 0;
	}

	if (STR_EQ(str, "dst mac")) { /* destination MAC address to be used for packets */
		return parse_mac(&targ->edaddr, pkey);
	}
	if (STR_EQ(str, "local ipv4")) { /* source IP address to be used for packets */
		return parse_ip(&targ->local_ipv4, pkey);
	}
        if (STR_EQ(str, "local ipv6")) { /* source IPv6 address to be used for packets */
                return parse_ip6(&targ->local_ipv6, pkey);
        }

	if (STR_EQ(str, "pipes")) {
		uint32_t val;
		int err = parse_int(&val, pkey);
		if (err)
			return -1;
		if (!val || !rte_is_power_of_2(val)) {
			set_errf("Number of pipes has to be power of 2 and not zero");
			return -1;
		}

		targ->qos_conf.port_params.n_pipes_per_subport = val;
		return 0;
	}
	if (STR_EQ(str, "queue size")) {
		uint32_t val;
		int err = parse_int(&val, pkey);
		if (err) {
			return -1;
		}

		targ->qos_conf.port_params.qsize[0] = val;
		targ->qos_conf.port_params.qsize[1] = val;
		targ->qos_conf.port_params.qsize[2] = val;
		targ->qos_conf.port_params.qsize[3] = val;
		return 0;
	}
	if (STR_EQ(str, "subport tb rate")) {
		return parse_int(&targ->qos_conf.subport_params[0].tb_rate, pkey);
	}
	if (STR_EQ(str, "subport tb size")) {
		return parse_int(&targ->qos_conf.subport_params[0].tb_size, pkey);
	}
	if (STR_EQ(str, "subport tc 0 rate")) {
		return parse_int(&targ->qos_conf.subport_params[0].tc_rate[0], pkey);
	}
	if (STR_EQ(str, "subport tc 1 rate")) {
		return parse_int(&targ->qos_conf.subport_params[0].tc_rate[1], pkey);
	}
	if (STR_EQ(str, "subport tc 2 rate")) {
		return parse_int(&targ->qos_conf.subport_params[0].tc_rate[2], pkey);
	}
	if (STR_EQ(str, "subport tc 3 rate")) {
		return parse_int(&targ->qos_conf.subport_params[0].tc_rate[3], pkey);
	}

	if (STR_EQ(str, "subport tc rate")) {
		uint32_t val;
		int err = parse_int(&val, pkey);
		if (err) {
			return -1;
		}

		targ->qos_conf.subport_params[0].tc_rate[0] = val;
		targ->qos_conf.subport_params[0].tc_rate[1] = val;
		targ->qos_conf.subport_params[0].tc_rate[2] = val;
		targ->qos_conf.subport_params[0].tc_rate[3] = val;

		return 0;
	}
	if (STR_EQ(str, "subport tc period")) {
		return parse_int(&targ->qos_conf.subport_params[0].tc_period, pkey);
	}
	if (STR_EQ(str, "pipe tb rate")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tb_rate, pkey);
	}
	if (STR_EQ(str, "pipe tb size")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tb_size, pkey);
	}
	if (STR_EQ(str, "pipe tc rate")) {
		uint32_t val;
		int err = parse_int(&val, pkey);
		if (err) {
			return -1;
		}

		targ->qos_conf.pipe_params[0].tc_rate[0] = val;
		targ->qos_conf.pipe_params[0].tc_rate[1] = val;
		targ->qos_conf.pipe_params[0].tc_rate[2] = val;
		targ->qos_conf.pipe_params[0].tc_rate[3] = val;
		return 0;
	}
	if (STR_EQ(str, "pipe tc 0 rate")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tc_rate[0], pkey);
	}
	if (STR_EQ(str, "pipe tc 1 rate")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tc_rate[1], pkey);
	}
	if (STR_EQ(str, "pipe tc 2 rate")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tc_rate[2], pkey);
	}
	if (STR_EQ(str, "pipe tc 3 rate")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tc_rate[3], pkey);
	}
	if (STR_EQ(str, "pipe tc period")) {
		return parse_int(&targ->qos_conf.pipe_params[0].tc_period, pkey);
	}
	if (STR_EQ(str, "police action")) {
		char *in = strstr(pkey, " io=");
		if (in == NULL) {
			set_errf("Need to specify io colors using io=in_color,out_color\n");
			return -1;
		}
		*in = 0;
		in += strlen(" io=");

		char *out = strstr(in, ",");
		if (out == NULL) {
			set_errf("Output color not specified\n");
		}
		*out = 0;
		out++;

		enum police_action in_color = str_to_color(in);
		enum police_action out_color = str_to_color(out);

		if (in_color == ACT_INVALID) {
			set_errf("Invalid input color %s. Expected green, yellow or red", in);
			return -1;
		}
		if (out_color == ACT_INVALID) {
			set_errf("Invalid output color %s. Expected green, yellow or red", out);
			return -1;
		}
		enum police_action action = str_to_color(pkey);
		if (action == ACT_INVALID) {
			set_errf("Error action %s. Expected green, yellow, red or drop", pkey);
			return -1;
		}
		targ->police_act[in_color][out_color] = action;

		return 0;
	}
	if (STR_EQ(str, "cir")) {
		return parse_int(&targ->cir, pkey);
	}
	if (STR_EQ(str, "cbs")) {
		return parse_int(&targ->cbs, pkey);
	}
	if (STR_EQ(str, "pir")) {
		return parse_int(&targ->pir, pkey);
	}
	if (STR_EQ(str, "pbs")) {
		return parse_int(&targ->pbs, pkey);
	}
	if (STR_EQ(str, "ebs")) {
		return parse_int(&targ->ebs, pkey);
	}
	uint32_t queue_id = 0;
	if (sscanf(str, "queue %d weight", &queue_id) == 1) {
		uint32_t val;
		int err = parse_int(&val, pkey);
		if (err) {
			return -1;
		}
		targ->qos_conf.pipe_params[0].wrr_weights[queue_id] = val;
		return 0;
	}
	if (STR_EQ(str, "classify")) {
		if (!(targ->task_init->flag_features & TASK_CLASSIFY)) {
			set_errf("Classify is not supported in '%s' mode", targ->task_init->mode_str);
			return -1;
		}

		return parse_flag(&targ->runtime_flags, TASK_CLASSIFY, pkey);
	}
#ifdef GRE_TP
	if (STR_EQ(str, "tbf rate")) {
		return parse_int(&targ->tb_rate, pkey);
	}
	if (STR_EQ(str, "tbf size")) {
		return parse_int(&targ->tb_size, pkey);
	}
#endif
	if (STR_EQ(str, "max rules")) {
		return parse_int(&targ->n_max_rules, pkey);
	}

        if (STR_EQ(str, "tunnel hop limit")) {
                uint32_t val;
                int err = parse_int(&val, pkey);
                if (err) {
                        return -1;
                }
                targ->tunnel_hop_limit = val;
                return 0;
        }

        if (STR_EQ(str, "lookup port mask")) {
                uint32_t val;
                int err = parse_int(&val, pkey);
                if (err) {
                        return -1;
                }
                targ->lookup_port_mask = val;
                return 0;
        }

	set_errf("Option '%s' is not known", str);
	/* fail on unknown keys */
	return -1;
}

/* command line parameters parsing procedure */
int dppd_parse_args(int argc, char **argv)
{
	/* Default settings */
	dppd_cfg.flags |= TGSF_AUTOSTART | TGSF_WAIT_ON_QUIT;

	mprintf("=== Parsing command line ===\n");
	int opt, ret;
	char *tmp, *tmp2;
	char tmp3[64];
	while ((opt = getopt(argc, argv, "f:aesiw:")) != EOF) {
		switch (opt) {
		case 'f':
			/* path to config file */
			cfg_file = optarg;
			size_t offset = 0;
			for (size_t i = 0; i < strlen(cfg_file); ++i) {
				if (cfg_file[i] == '/') {
					offset = i + 1;
				}
			}

			strncpy(dppd_cfg.name, cfg_file + offset, MAX_NAME_SIZE);
			mprintf("\tConfiguration file: %s\n", cfg_file);
			break;
		case 'a':
			/* autostart all ports */
			dppd_cfg.flags |= TGSF_AUTOSTART;
			break;
		case 'e':
			/* don't autostart */
			dppd_cfg.flags &= ~TGSF_AUTOSTART;
			break;
		case 's':
			/* check configuration file syntax and exit */
			dppd_cfg.flags |= TGSF_CHECK_SYNTAX;
			break;
		case 'i':
			/* check initialization sequence and exit */
			dppd_cfg.flags |= TGSF_CHECK_INIT;
			break;
		case 'w':
			tmp = optarg;
			tmp2 = 0;
			if (strlen(tmp) >= 3 &&
			    (tmp2 = strchr(tmp, '='))) {
				*tmp2 = 0;
				tmp3[0] = '$';
				strncpy(tmp3 + 1, tmp, 63);
				mprintf("\tAdding variable: %s = %s\n", tmp3, tmp2 + 1);
				ret = add_var(tmp3, tmp2 + 1, 1);
				if (ret == -2) {
					mprintf("\tError adding variable: too many variables defines\n");
					return -1;
				}
				else if(ret == -3) {
					mprintf("\tError adding variable: Each variable can only be defined once\n");
					return -1;
				}
				break;
			}
			/* fall-through */
		default:
			mprintf("\tError while parsing command line options\n");
			return -1;
		}
	}

	/* reset getopt lib for DPDK */
	optind = 0;

	return 0;
}

static int check_cfg(void)
{
	/* Sanity check */
#define RETURN_IF(cond, err)			\
	if (cond) {				\
		mprintf(err);			\
		return -1;			\
	};

	RETURN_IF(rte_cfg.force_nchannel == 0, "\tError: number of memory channels not specified in [eal options] section\n");
	RETURN_IF(dppd_cfg.master >= RTE_MAX_LCORE, "\tError: master core index not specified in [global] section\n");

	RETURN_IF(dppd_nb_active_ports() == 0, "\tError: No ports specified, check configuration ('tx port name' and 'rx port name')\n");
#undef RETURN_IF

	return 0;
}

int dppd_read_config_file(void)
{
	set_global_defaults(&dppd_cfg);
	set_path_defaults(&dppd_cfg, cfg_file);
	set_task_defaults(&dppd_cfg, lcore_cfg_init);
	set_port_defaults();
	mprintf("=== Parsing configuration file '%s' ===\n", cfg_file);
	struct cfg_file *pcfg = cfg_open(cfg_file);
	if (pcfg == NULL) {
		return -1;
	}

	struct cfg_section* config_sections[] = {
		&var_cfg          ,
		&eal_default_cfg  ,
		&port_cfg         ,
		&defaults_cfg     ,
		&settings_cfg     ,
		&core_cfg         ,
		NULL
	};

	for (struct cfg_section** section = config_sections; *section != NULL; ++section) {
		const char* name = (*section)->name;
		size_t len = strlen(name);
		mprintf("\t*** Reading [%s] section%s ***\n", name, name[len - 1] == '#'? "s": "");
		cfg_parse(pcfg, *section);

		if ((*section)->error) {
			mprintf("Error at line %u, section [%s], entry %u: %s\n\t%s\n"
			        , pcfg->err_line, pcfg->err_section, pcfg->err_entry + 1, strlen(get_parse_err())? get_parse_err() : err_str, pcfg->cur_line);
			cfg_close(pcfg); /* cannot close before printing error, print uses internal buffer */
			return -1;
		}
	}

	cfg_close(pcfg);

	return check_cfg();
}


static void failed_rte_eal_init(__attribute__((unused))const char *prog_name)
{
	mprintf("\tError in rte_eal_init()\n");
}

int dppd_setup_rte(const char *prog_name)
{
	char *rte_argv[MAX_RTE_ARGV];
	char  rte_arg[MAX_RTE_ARGV][MAX_ARG_LEN];

	/* create mask of used cores */
	mprintf("=== Setting up RTE EAL ===\n");
	rte_cfg.core_mask = 0;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (lcore_cfg_init[lcore_id].nb_tasks > 0 && lcore_cfg_init[lcore_id].targs[0].mode != MASTER) {
			rte_cfg.core_mask |= (__UINT64_C(1) << lcore_id);
		}
	}
	dppd_cfg.core_mask = rte_cfg.core_mask;

	mprintf("\tWorker threads core mask is 0x%lx\n", rte_cfg.core_mask);
	if (rte_cfg.core_mask & (__UINT64_C(1) << dppd_cfg.master)) {
		mprintf("\tError: master core index is reused for [core %u] section.\n", dppd_cfg.master);
		return -1;
	}
	if (rte_cfg.core_mask & ((__UINT64_C(1) << dppd_cfg.master) - 1)) {
		mprintf("\tInvalid master core index %u: DPDK would use the lowest index from [core #] sections.\n",
		        dppd_cfg.master);
		return -1;
	}
	rte_cfg.core_mask |= __UINT64_C(1) << dppd_cfg.master;
	mprintf("\tWith master core index %u, full core mask is 0x%lx\n", dppd_cfg.master, rte_cfg.core_mask);

	/* fake command line parameters for rte_eal_init() */
	int argc = 0;
	rte_argv[argc] = strdup(prog_name);
	sprintf(rte_arg[++argc], "-c%lx", rte_cfg.core_mask);
	rte_argv[argc] = rte_arg[argc];

	if (rte_cfg.memory) {
		sprintf(rte_arg[++argc], "-m%u", rte_cfg.memory);
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.force_nchannel) {
		sprintf(rte_arg[++argc], "-n%u", rte_cfg.force_nchannel);
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.force_nrank) {
		sprintf(rte_arg[++argc], "-r%u", rte_cfg.force_nrank);
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_hugetlbfs) {
		strcpy(rte_arg[++argc], "--no-huge");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_pci) {
		strcpy(rte_arg[++argc], "--no-pci");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_hpet) {
		strcpy(rte_arg[++argc], "--no-hpet");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_shconf) {
		strcpy(rte_arg[++argc], "--no-shconf");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.eal != NULL) {
		char *ptr = rte_cfg.eal;
		char *ptr2;
		while (ptr != NULL) {
			ptr2 = ptr;
			ptr = strchr(ptr, ' ');
			if (ptr) {
				*ptr++ = '\0';
			}
			strcpy(rte_arg[++argc], ptr2);
			rte_argv[argc] = rte_arg[argc];
		}
	}


	if (rte_cfg.hugedir != NULL) {
		strcpy(rte_arg[++argc], "--huge-dir");
		rte_argv[argc] = rte_arg[argc];
		rte_argv[++argc] = rte_cfg.hugedir;
	}

	if (rte_cfg.no_output) {
		rte_set_log_level(0);
	}
	/* init EAL */
	mprintf("\tEAL command line:");
	if (argc >= MAX_RTE_ARGV) {
		mprintf("too many arguments for EAL\n");
		return -1;
	}

	for (int h = 0; h <= argc; ++h) {
		mprintf(" %s", rte_argv[h]);
	}
	mprintf("\n");

	rte_set_application_usage_hook(failed_rte_eal_init);
	if (rte_eal_init(++argc, rte_argv) < 0) {
		mprintf("\tError in rte_eal_init()\n");
		return -1;
	}
	mprintf("\tEAL Initialized\n");

	/* check if all active cores are in enabled in DPDK */
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {

		if (lcore_id == dppd_cfg.master) {
			if (!rte_lcore_is_enabled(lcore_id))
				return -1;
		}
		else if (rte_lcore_is_enabled(lcore_id) != dppd_core_active(lcore_id)) {
			return -1;
		}
		else if (lcore_cfg_init[lcore_id].nb_tasks != 0 && !rte_lcore_is_enabled(lcore_id)) {
			mprintf("\tfailed to enable lcore %u\n", lcore_id);
			return -1;
		}
	}
	return 0;
}
