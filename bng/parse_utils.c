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

#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>

#include <rte_ether.h>
#include <rte_string_fns.h>

#include "quit.h"
#include "route.h"
#include "parse_utils.h"
#include "dppd_globals.h"

#define MAX_NB_PORT_NAMES 16
#define MAX_LEN_PORT_NAME 24
#define MAX_LEN_VAR_NAME  24
#define MAX_LEN_VAL       40
#define MAX_NB_VARS       16
#define MAX_STR_LEN_PROC  64

static uint8_t nb_cores_per_socket[2];
static uint8_t cores_per_socket[2][RTE_MAX_LCORE];

struct port_name {
	uint32_t id;
	char     name[MAX_LEN_PORT_NAME];
};

static struct port_name port_names[MAX_NB_PORT_NAMES];
static uint8_t nb_port_names;

struct var {
	uint8_t  cli;
	char     name[MAX_LEN_VAR_NAME];
	char     val[MAX_LEN_VAL];
};

static struct var vars[MAX_NB_PORT_NAMES];
static uint8_t nb_vars;

static char format_err_str[256];
static const char *err_str = "";

const char *get_parse_err(void)
{
	return err_str;
}

static void set_errf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	vsnprintf(format_err_str, sizeof(format_err_str), format, ap);
	va_end(ap);
	err_str = format_err_str;
}

static int is_var(const char *name)
{
        return name[0] == '$';
}

static int parse_var(const char **val, const char *name)
{
	for (uint8_t i = 0; i < nb_vars; ++i) {
		if (!strcmp(name, vars[i].name)) {
                        *val = vars[i].val;
			return 0;
		}
	}
	set_errf("Variable '%s' not defined!", name);
	return 1;
}

int parse_int_mask(uint32_t *val, uint32_t *mask, const char *str)
{
	char str_cpy[MAX_STR_LEN_PROC];
	char *mask_str;

	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	if (strlen(str) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(str_cpy, str, MAX_STR_LEN_PROC);


	mask_str = strchr(str_cpy, '&');

	if (mask_str == NULL) {
		set_errf("Missing '&' when parsing mask");
		return -2;
	}

	*mask_str = 0;

	if (parse_int(val, str))
		return -1;
	if (parse_int(mask, mask_str + 1))
		return -1;

	return 0;
}

int parse_range(uint32_t* lo, uint32_t* hi, const char *str)
{
	char str_cpy[MAX_STR_LEN_PROC];
	char *dash;

	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	if (strlen(str) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}

	strncpy(str_cpy, str, MAX_STR_LEN_PROC);

	dash = strstr(str_cpy, "-");

	if (dash == NULL) {
		set_errf("Missing '-' when parsing mask");
		return -2;
	}

	*dash = 0;

	if (parse_int(lo, str_cpy))
		return -1;
	if (parse_int(hi, dash + 1))
		return -1;


	int64_t tmp = strtol(str_cpy, 0, 0);
	if (tmp > UINT32_MAX) {
		set_errf("Integer is bigger than %u", UINT32_MAX);
		return -1;
	}
	if (tmp < 0) {
		set_errf("Integer is negative");
		return -2;
	}

	*lo = tmp;

	tmp = strtol(dash + 1, 0, 0);
	if (tmp > UINT32_MAX) {
		set_errf("Integer is bigger than %u", UINT32_MAX);
		return -1;
	}
	if (tmp < 0) {
		set_errf("Integer is negative");
		return -2;
	}

	*hi = tmp;

	if (*lo > *hi) {
		set_errf("Low boundary is above high boundary in range");
		return -2;
	}

	return 0;
}

int parse_ip(uint32_t *addr, const char *saddr)
{
	if (is_var(saddr)) {
		if (parse_var(&saddr, saddr))
			return -1;
	}

	char *ip_parts[5];

	char saddr_cpy[MAX_STR_LEN_PROC];
	if (strlen(saddr) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(saddr_cpy, saddr, MAX_STR_LEN_PROC);

	if (4 != rte_strsplit(saddr_cpy, strlen(saddr_cpy), ip_parts, 5, '.')) {
		set_errf("Expecting 4 octets in ip.");
		return -1;
	}

	uint32_t val;
	for (uint8_t i = 0; i < 4; ++i) {
		val = atoi(ip_parts[i]);
		if (val > 255) {
			set_errf("Maximum value for octet is 255 but octet %u is %u", i, val);
			return -1;
		}
		*addr = *addr << 8 | val;
	}
	return 0;
}

int parse_ip_cidr(struct ipv4_subnet *val, const char *saddr)
{
	char saddr_cpy[MAX_STR_LEN_PROC];
	char *slash;
	int prefix;

	if (is_var(saddr)) {
		if (parse_var(&saddr, saddr))
			return -1;
	}

	if (strlen(saddr) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}

	strncpy(saddr_cpy, saddr, MAX_STR_LEN_PROC);
	slash = strstr(saddr_cpy, "/");

	if (slash == NULL) {
		set_errf("Missing '/' when parsing CIDR notation");
		return -2;
	}

	*slash = 0;
	prefix = atoi(slash + 1);
	val->prefix = prefix;

	if (prefix > 32) {
		set_errf("Prefix %d is too big", prefix);
		return -2;
	}
	if (prefix < 1) {
		set_errf("Prefix %d is too small", prefix);
	}
	if (parse_ip(&val->ip, saddr_cpy))
		return -2;

	/* Apply mask making all bits outside the prefix zero */
	val->ip &= ((int)(1 << 31)) >> (prefix - 1);

	return 0;
}

int parse_ip6(struct ipv6_addr *addr, const char *saddr)
{
	if (is_var(saddr)) {
		if (parse_var(&saddr, saddr))
			return -1;
	}

	char *addr_parts[9];

	char saddr_cpy[MAX_STR_LEN_PROC];
	if (strlen(saddr) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(saddr_cpy, saddr, MAX_STR_LEN_PROC);

	uint8_t ret = rte_strsplit(saddr_cpy, strlen(saddr_cpy), addr_parts, 9, ':');

	if (ret == 9) {
		set_errf("Invalid IPv6 address");
		return -1;
	}

	uint8_t omitted = 0;

	for (uint8_t i = 0, j = 0; i < ret; ++i, ++j) {
		if (*addr_parts[i] == 0) {
			if (omitted == 0) {
				set_errf("Can only omit zeros once");
				return -1;
			}
			omitted = 1;
			j += 8 - ret;
		}
		else {
			uint16_t w = strtoll(addr_parts[i], NULL, 16);
			addr->bytes[j++] = (w >> 8) & 0xff;
			addr->bytes[j] = w & 0xff;
		}
	}
	return 0;
}

int parse_mac(struct ether_addr *ether_addr, const char *saddr)
{
	if (is_var(saddr)) {
		if (parse_var(&saddr, saddr))
			return -1;
	}

	char *addr_parts[7];

	char saddr_cpy[MAX_STR_LEN_PROC];
	if (strlen(saddr) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(saddr_cpy, saddr, MAX_STR_LEN_PROC);

	uint8_t ret = rte_strsplit(saddr_cpy, strlen(saddr_cpy), addr_parts, 7, ':');

	if (ret != 6) {
		set_errf("Invalid MAC address format");
		return -1;
	}

	for (uint8_t i = 0; i < 6; ++i) {
		if (2 != strlen(addr_parts[i])) {
			set_errf("Invalid MAC address format");
			return -1;
		}
		ether_addr->addr_bytes[i] = strtol(addr_parts[i], NULL, 16);
	}

	return 0;
}


char* get_cfg_key(char *str)
{
	char *pkey = strchr(str, '=');

	if (pkey == NULL) {
		return NULL;
	}
	*pkey++ = '\0';

	/* remove leading spaces */
	while (isspace(*pkey)) {
		pkey++;
	}
	if (*pkey == '\0') { /* an empty key */
		return NULL;
	}

	return pkey;
}

void strip_spaces(char *strings[], const uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i) {
		while (isspace(strings[i][0])) {
			++strings[i];
		}
		size_t len = strlen(strings[i]);

		while (len && isspace(strings[i][len - 1])) {
			strings[i][len - 1] = '\0';
			--len;

		}
	}
}

static int find_ht(uint32_t* dst, uint32_t core_id)
{
	uint32_t ret;
	char buf[1024];
	snprintf(buf, sizeof(buf), "/sys/devices/system/cpu/cpu%u/topology/thread_siblings_list", core_id);
	FILE* ht_fd = fopen(buf, "r");

	if (ht_fd == NULL) {
		set_errf("Could not open cpu topology %s", buf);
		return -1;
	}

	if (fgets(buf, sizeof(buf), ht_fd) == NULL) {
		set_errf("Could not read cpu topology");
		return -1;
	}

	uint32_t list[2] = {-1,-1};
	parse_list_set(list, buf, 2);

	if (core_id != list[0]) {
		set_errf("No hyper-thread found for core %u", core_id);
		return -1;
	}

	fclose(ht_fd);
	*dst = list[1];
	return 0;
}

static uint32_t find_socket(uint32_t core_id)
{
	int ret = -1;
	char buf[1024];
	snprintf(buf, sizeof(buf), "/sys/devices/system/cpu/cpu%u/topology/physical_package_id", core_id);
	FILE* fd = fopen(buf, "r");
	if (fgets(buf, sizeof(buf), fd) != NULL) {
		ret = atoi(buf);
	}
	fclose(fd);

	return ret == -1 ? 0 : ret;
}

int parse_list_set(uint32_t *list, const char *str, uint32_t max_list)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	char str_cpy[MAX_STR_LEN_PROC];
	char* str2 = str_cpy;
	if (strlen(str) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(str_cpy, str, MAX_STR_LEN_PROC);


	unsigned int i, j = 0;
	uint32_t range;
	while (*str) {

		uint32_t val;

		if (0 != parse_core(&val, str2, &str2)) {
			return -1;
		}

		list[j++] = val;
		if (*str2 == '-') {
			*str2++ = '\0';
			if (0 != parse_core(&range, str2, &str2)) {
				return -1;
			}

			if (range >= val) {
				for (i = val + 1; i <= range; ++i) {
					if (j == max_list) {
						set_errf("Too many elements in range (%u > %u)", range, max_list);
						return -1;
					}
					list[j++] = i;
				}
			}
			else {
				set_errf("Too little elements in range (%u < %u)", val, range);
				return -1;
			}
		}
		if (*str2 == ',') {
			++str2;
		}
		else {
			break;
		}
	}
	return j;
}

int parse_kmg(uint32_t* val, const char *str)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	char c = str[strlen(str) - 1];
	*val = atoi(str);

	switch (c) {
	case 'G':
		if (*val >> 22)
			return -2;
		*val <<= 10;
	case 'M':
		if (*val >> 22)
			return -2;
		*val <<= 10;
	case 'K':
		if (*val >> 22)
			return -2;
		*val <<= 10;
		break;
	default:
		/* only support optional KMG suffix */
		if (c < '0' || c > '9') {
			set_errf("Unknown syntax for KMG suffix '%c' (expected K, M or G)", c);
			return -1;
		}
	}

	return 0;
}

int parse_bool(uint32_t* val, const char *str)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	if (!strcmp(str, "yes")) {
		*val = 1;
		return 0;
	}
	else if (!strcmp(str, "no")) {
		*val = 0;
		return 0;
	}

	set_errf("Unknown syntax for bool '%s' (expected yes or no)", str);

	return -1;
}


int parse_flag(uint32_t* val, uint32_t flag, const char *str)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	uint32_t tmp;
	if (parse_bool(&tmp, str))
		return -1;

	if (tmp)
		*val |= flag;
	else
		*val &= ~flag;

	return 0;
}

int parse_int(uint32_t* val, const char *str)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	int64_t tmp = strtol(str, 0, 0);
	if (tmp > UINT32_MAX) {
		set_errf("Integer is bigger than %u", UINT32_MAX);
		return -1;
	}
	if (tmp < 0) {
		set_errf("Integer is negative");
		return -2;
	}
	*val = tmp;

	return 0;
}

int parse_str(char* dst, const char *str, size_t max_len)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}
	if (strlen(str) > max_len - 1) {
		set_errf("String too long (%u > %u)", strlen(str), max_len - 1);
		return -2;
	}

	strncpy(dst, str, max_len);
	return 0;
}

int parse_path(char *dst, const char *str, size_t max_len)
{
	if (parse_str(dst, str, max_len))
		return -1;
	if (access(dst, F_OK)) {
                set_errf("Invalid file '%s' (%s)", dst, strerror(errno));
		return -1;
	}
	return 0;
}

int parse_port_name(uint32_t *val, const char *str)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	for (uint8_t i = 0; i < nb_port_names; ++i) {
		if (!strcmp(str, port_names[i].name)) {
			*val = port_names[i].id;
			return 0;
		}
	}
	set_errf("Port with name %s not defined", str);
	return 1;
}

int parse_port_name_list(uint32_t *val, uint32_t* tot, uint8_t max_vals, const char *str)
{
	char *elements[DPPD_MAX_PORTS + 1];
	char str_cpy[MAX_STR_LEN_PROC];
	uint32_t cur;
	int ret;

	if (strlen(str) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(str_cpy, str, MAX_STR_LEN_PROC);

	ret = rte_strsplit(str_cpy, strlen(str_cpy), elements, DPPD_MAX_PORTS + 1, ',');

	if (ret == DPPD_MAX_PORTS + 1 || ret > max_vals) {
		set_errf("Too many ports in port list");
		return -1;
	}

	strip_spaces(elements, ret);
	for (uint8_t i = 0; i < ret; ++i) {
		if (parse_port_name(&cur, elements[i])) {
			return -1;
		}
		val[i] = cur;
	}
	if (tot) {
		*tot = ret;
	}
	return 0;
}

int parse_remap(uint8_t *mapping, const char *str)
{
	char *elements[DPPD_MAX_PORTS + 1];
	char *elements2[DPPD_MAX_PORTS + 1];
	char str_cpy[MAX_STR_LEN_PROC];
	uint32_t val;
	int ret, ret2;

	if (strlen(str) > MAX_STR_LEN_PROC) {
		set_errf("String too long (max supported: %d)", MAX_STR_LEN_PROC);
		return -2;
	}
	strncpy(str_cpy, str, MAX_STR_LEN_PROC);

	ret = rte_strsplit(str_cpy, strlen(str_cpy), elements, DPPD_MAX_PORTS + 1, ',');
	if (ret <= 0) {
		set_errf("Invalid remap syntax");
		return -1;
	}
	else if (ret > DPPD_MAX_PORTS) {
		set_errf("Too many remaps");
		return -2;
	}

	strip_spaces(elements, ret);
	for (uint8_t i = 0; i < ret; ++i) {
		ret2 = rte_strsplit(elements[i], strlen(elements[i]), elements2, DPPD_MAX_PORTS + 1, '|');
		strip_spaces(elements2, ret2);
		if (ret2 > DPPD_MAX_PORTS) {
			set_errf("Too many remaps");
			return -2;
		}
		for (uint8_t j = 0; j < ret2; ++j) {
			if (parse_port_name(&val, elements2[j])) {
				return -1;
			}

			/* This port will be mapped to the i'th
			   element specified before remap=. */
			mapping[val] = i;
		}
	}

	return ret;
}

int add_port_name(uint32_t val, const char *str)
{
	if (is_var(str)) {
		if (parse_var(&str, str))
			return -1;
	}

	struct port_name* pn;

	if (nb_port_names == MAX_NB_PORT_NAMES) {
		set_errf("Too many ports defined (can define %d)", MAX_NB_PORT_NAMES);
		return -1;
	}

	for (uint8_t i = 0; i < nb_port_names; ++i) {
		/* each port has to have a unique name*/
		if (!strcmp(str, port_names[i].name)) {
			set_errf("Port with name %s is already defined", str);
			return -2;
		}
	}

	pn = &port_names[nb_port_names];
	strncpy(pn->name, str, sizeof(pn->name));
	pn->id = val;

	++nb_port_names;
	return 0;
}

int add_var(const char* name, const char *val, uint8_t cli)
{
	struct var* v;

	if (strlen(name) == 0 || strlen(name) == 1) {
		set_errf("Can't define variables with empty name");
		return -1;
	}

	if (name[0] != '$') {
		set_errf("Each variable should start with the $ character");
		return -1;
	}

	if (nb_vars == MAX_NB_VARS) {
		set_errf("Too many variables defined (can define %d)", MAX_NB_VARS);
		return -2;
	}

	for (uint8_t i = 0; i < nb_vars; ++i) {
		if (!strcmp(name, vars[i].name)) {

			/* Variables defined through program arguments
			   take precedence. */
			if (!cli && vars[i].cli) {
				return 0;
			}

			set_errf("Variable with name %s is already defined", name);
			return -3;
		}
	}

	v = &vars[nb_vars];
	strncpy(v->name, name, sizeof(v->name));
	strncpy(v->val, val, sizeof(v->val));
	v->cli = cli;

	++nb_vars;
	return 0;
}

static int get_total_cores(uint32_t* val)
{
	FILE* fd = fopen("/sys/devices/system/cpu/present", "r");
	char buf[1024];

	if (fd == NULL) {
		set_errf("Could not opening file /sys/devices/system/cpu/present");
		return -1;
	}

	if (fgets(buf, sizeof(buf), fd) == NULL) {
		set_errf("Could not read cores range");
		return -1;
	}

	char* pos = strstr(buf, "-");
	uint8_t ret;
	if (pos == NULL) {
		ret = atoi(buf);
	}
	else {
		pos++;
		ret = atoi(pos) + 1;
	}

	fclose(fd);
	*val = ret;
	return 0;
}

int parse_core(uint32_t *val, const char* str, char** end)
{
	uint8_t socket_id = 255;
	uint32_t val2, n_cores;

	if (nb_cores_per_socket[0] == 0) {
		if (get_total_cores(&n_cores)) {
			set_errf("Failed to obtain total number of cores on system");
			return -1;
		}
		n_cores = n_cores > RTE_MAX_LCORE? RTE_MAX_LCORE : n_cores;

		for (uint8_t i = 0; i < n_cores; ++i) {
			cores_per_socket[find_socket(i)][nb_cores_per_socket[find_socket(i)]] = i;
			nb_cores_per_socket[find_socket(i)]++;
		}
	}

	*val = strtol(str, end, 10);
	val2 = *val;

	if (**end == 's') {
		++*end;
		socket_id = strtol(*end, end, 10);
		if (socket_id >= MAX_SOCKETS) {
			set_errf("Socket id %d to high (max allowed is %d)", socket_id, MAX_SOCKETS - 1);
			return -1;
		}
		if (*val >= nb_cores_per_socket[socket_id]) {
			set_errf("Core %us%u is not on socket %u", *val, socket_id, socket_id);
			return -2;
		}
		/* The value of the core id has to be specified as if
		   it would be on socket 0. When the socket syntax is
		   used, the hyper-thread syntax is a requirement. */
		if (find_socket(*val) != 0) {
			set_errf("Core %us%u is not on socket %u", *val, socket_id, socket_id);
			return -2;
		}
		*val = cores_per_socket[socket_id][*val];
	}

	if (**end == 'h') {
		++*end;
		if (find_ht(val, *val)) {
			return -1;
		}
	}

	return 0;
}
