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

#include <curses.h>
#include <rte_cycles.h>
#include <string.h>
#include <signal.h>

#include "cqm.h"
#include "msr.h"
#include "display.h"
#include "commands.h"
#include "main.h"
#include "stats.h"
#include "dppd_args.h"
#include "dppd_cfg.h"
#include "display_utils.h"
#include "dppd_assert.h"
#include "version.h"
#include "dppd_port_cfg.h"

/* Set up the display mutex  as recursive. This enables threads to use
   display_[un]lock() to lock  the display when multiple  calls to for
   instance mprintf() need to be made. */

static pthread_mutex_t disp_mtx = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static pthread_mutex_t file_mtx = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;

static void display_lock(void)
{
	pthread_mutex_lock(&disp_mtx);
}

static void display_unlock(void)
{
	pthread_mutex_unlock(&disp_mtx);
}

static void file_lock(void)
{
	pthread_mutex_lock(&file_mtx);
}

static void file_unlock(void)
{
	pthread_mutex_unlock(&file_mtx);
}

__attribute__((format(printf, 2, 3))) void mprintf_once(uint8_t idx, __attribute__((unused)) const char *format, ...)
{
	static __thread int printed[16];
	if (idx > 16 || printed[idx]) {
		return;
	}
	printed[idx] = 1;

	va_list ap;
	char buffer[1024];

	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	mprintf("%s", buffer);
	va_end(ap);
}

#ifdef BRAS_STATS

struct cqm_related {
	struct cqm_features	features;
	uint8_t			supported;
	uint32_t		last_rmid;
};

struct cqm_related cqm;

struct global_stats {
	uint64_t tx_pps;
	uint64_t rx_pps;
	uint64_t tx_tot;
	uint64_t rx_tot;
	uint64_t rx_tot_beg;
	uint64_t tx_tot_beg;
	uint64_t avg_start;
	uint8_t  started_avg;
	uint64_t rx_avg;
	uint64_t tx_avg;
};

struct port_stats {
	uint64_t tot_tx_pkt_count;
	uint64_t tot_tx_pkt_drop;
	uint64_t tot_rx_pkt_count;

	uint64_t prev_tsc;
	uint64_t cur_tsc;

	uint32_t diff_tx_pkt_count;
	uint32_t diff_tx_pkt_drop;
	uint32_t diff_rx_pkt_count;
	uint32_t diff_empty_cycles;

	uint32_t last_tx_pkt_count;
	uint32_t last_tx_pkt_drop;
	uint32_t last_rx_pkt_count;
	uint32_t last_empty_cycles;

};

struct core_port {
	struct stats *stats;
	struct port_stats *port_stats;
	uint8_t lcore_id;
	uint8_t port_id;
	/* flags set if total RX/TX values need to be reported set at
	   initialization time, only need to access stats values in port */
	uint8_t flags;
};

struct lcore_stats {
	struct port_stats port_stats[MAX_TASKS_PER_CORE];
	uint32_t rmid;
	uint64_t cqm_data;
	uint64_t cqm_bytes;
	uint64_t cqm_fraction;
	uint64_t cur_afreq;
	uint64_t prev_afreq;
	uint64_t cur_mfreq;
	uint64_t prev_mfreq;
};

struct eth_stats {
	uint64_t no_mbufs;
	uint64_t ierrors;
	uint64_t prev_no_mbufs;
	uint64_t prev_ierrors;

	uint64_t rx_tot;
	uint64_t tx_tot;
	uint64_t prev_rx_tot;
	uint64_t prev_tx_tot;
};

/* Advanced text output */
static WINDOW *scr = NULL, *win_txt, *win_cmd, *win_stat, *win_title, *win_help;

/* Stores all readed values from the cores, displaying is done afterwards because
   displaying introduces overhead. If displaying was done right after the values
   are read, inaccuracy is introduced for later cores */
static struct lcore_stats  lcore_stats[RTE_MAX_LCORE];
static struct core_port    core_ports[RTE_MAX_LCORE *MAX_TASKS_PER_CORE];
static struct core_port    *core_port_ordered[RTE_MAX_LCORE*MAX_TASKS_PER_CORE];
static struct global_stats global_stats;
static struct eth_stats    eth_stats[16];
static uint8_t nb_tasks_tot;
static uint8_t nb_interface;
static uint8_t nb_active_interfaces;
static uint16_t core_port_height;
static uint64_t start_tsc;
static int msr_support;
static int col_offset;

/* Colors used in the interface */
enum colors {
	INVALID_COLOR,
	WHITE_ON_BLUE,
	BLACK_ON_CYAN,
	BLACK_ON_WHITE,
	BLACK_ON_YELLOW,
	WHITE_ON_RED,
	GREEN_ON_BLUE,
	YELLOW_ON_BLUE
};

static WINDOW *create_subwindow(int height, int width, int y_pos, int x_pos)
{
	WINDOW *win = subwin(scr, height, width, y_pos, x_pos);
	touchwin(scr);
	return win;
}

/* Format string capable [mv]waddstr() wrappers */
__attribute__((format(printf, 4, 5))) static inline void mvwaddstrf(WINDOW* win, int y, int x, const char *fmt, ...)
{
	va_list ap;

	char buf[1024];

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	wmove(win, y, x);
	if (x > COLS - 1) {
		return ;
	}

	/* to prevent strings from wrapping and */
	if (strlen(buf) > (uint32_t)COLS - x) {
		buf[COLS - 1 - x] = 0;
	}
	waddstr(win, buf);
}


#define PORT_STATS_RX 0x01
#define PORT_STATS_TX 0x02


// Red: link down; Green: link up
static short link_color(const uint8_t if_port)
{
	return COLOR_PAIR(dppd_port_cfg[if_port].link_up? GREEN_ON_BLUE : WHITE_ON_RED);
}

static void init_core_port(struct core_port *core_port, uint8_t lcore_id, uint8_t port_id, struct stats *stats, uint8_t flags)
{
	core_port->lcore_id = lcore_id;
	core_port->port_id = port_id;
	core_port->stats = stats;

	core_port->port_stats = &lcore_stats[lcore_id].port_stats[port_id];
	core_port->flags |= flags;

	if (cqm.supported && lcore_stats[lcore_id].rmid == 0) {
		++cqm.last_rmid; // 0 not used (by default all cores are 0)
		mprintf("setting up rmid: %d\n", cqm.last_rmid);
		lcore_stats[lcore_id].rmid = cqm.last_rmid;
	}
}

static struct core_port *set_line_no(const uint8_t lcore_id, const uint8_t port_id)
{
	for (uint8_t active_core_port = 0; active_core_port < nb_tasks_tot; ++active_core_port) {
		struct core_port *core_port = &core_ports[active_core_port];
		if (lcore_id == core_port->lcore_id && port_id == core_port->port_id) {
			return core_port;
		}
	}
	return NULL;
}

static void init_active_eth_ports(void)
{
	nb_interface = rte_eth_dev_count();

	nb_active_interfaces = 0;
	for (uint8_t i = 0; i < nb_interface; ++i) {
		if (dppd_port_cfg[i].active) {
			nb_active_interfaces++;
		}
	}
}

/* Populate active_core_ports for stats reporting, the order of the cores matters
   for reporting the most accurate results. TX cores should updated first (to prevent
   negative Loss stats). This will also calculate the number of core ports used by
   other display functions. */
static void init_active_core_ports(void)
{
	struct lcore_cfg *lconf;
	uint32_t lcore_id;

	/* add cores that are receiving from and sending to physical ports first */
	lcore_id = -1;
	while(dppd_core_next(&lcore_id) == 0) {
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_args *targ = &lconf->targs[task_id];
			struct stats *stats = lconf->task[task_id]->stats;
			if (targ->nb_rxrings == 0 && targ->nb_txrings == 0) {
				init_core_port(&core_ports[nb_tasks_tot], lcore_id, task_id, stats, PORT_STATS_RX | PORT_STATS_TX);
				++nb_tasks_tot;
			}
		}
	}

	/* add cores that are sending to physical ports second */
	lcore_id = -1;
	while(dppd_core_next(&lcore_id) == 0) {
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_args *targ = &lconf->targs[task_id];
			struct stats *stats = lconf->task[task_id]->stats;
			if (targ->nb_rxrings != 0 && targ->nb_txrings == 0) {
				init_core_port(&core_ports[nb_tasks_tot], lcore_id, task_id, stats, PORT_STATS_TX);
				++nb_tasks_tot;
			}
		}
	}

	/* add cores that are receiving from physical ports third */
	lcore_id = -1;
	while(dppd_core_next(&lcore_id) == 0) {
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_args *targ = &lconf->targs[task_id];
			struct stats *stats = lconf->task[task_id]->stats;
			if (targ->nb_rxrings == 0 && targ->nb_txrings != 0) {
				init_core_port(&core_ports[nb_tasks_tot], lcore_id, task_id, stats, PORT_STATS_RX);
				++nb_tasks_tot;
			}
		}
	}

	/* add cores that are working internally (no physical ports attached) */
	lcore_id = -1;
	while(dppd_core_next(&lcore_id) == 0) {
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_args *targ = &lconf->targs[task_id];
			struct stats *stats = lconf->task[task_id]->stats;
			if (targ->nb_rxrings != 0 && targ->nb_txrings != 0) {
				init_core_port(&core_ports[nb_tasks_tot], lcore_id, task_id, stats, 0);
				++nb_tasks_tot;
			}
		}
	}
}

static void update_tsc_stats(void)
{
	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		const uint8_t lcore_id = core_ports[task_id].lcore_id;
		const uint8_t port_id = core_ports[task_id].port_id;
		struct port_stats *cur_port_stats = &lcore_stats[lcore_id].port_stats[port_id];
		cur_port_stats->prev_tsc = cur_port_stats->cur_tsc;
	}

	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (dppd_port_cfg[port_id].active) {
			eth_stats[port_id].prev_no_mbufs = eth_stats[port_id].no_mbufs;
			eth_stats[port_id].prev_ierrors = eth_stats[port_id].ierrors;
			eth_stats[port_id].prev_rx_tot = eth_stats[port_id].rx_tot;
			eth_stats[port_id].prev_tx_tot = eth_stats[port_id].tx_tot;
		}
	}

	if (msr_support) {
		for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
			lcore_stats[lcore_id].prev_afreq = lcore_stats[lcore_id].cur_afreq;
			lcore_stats[lcore_id].prev_mfreq = lcore_stats[lcore_id].cur_mfreq;
		}
	}
}

void init_display(unsigned avg_start)
{
	scr = initscr();
	start_color();
	/* Assign default foreground/background colors to color number -1 */
	use_default_colors();

	init_pair(WHITE_ON_BLUE,   COLOR_WHITE,  COLOR_BLUE);
	init_pair(BLACK_ON_CYAN,   COLOR_BLACK,  COLOR_CYAN);
	init_pair(BLACK_ON_WHITE,  COLOR_BLACK,  COLOR_WHITE);
	init_pair(BLACK_ON_YELLOW, COLOR_BLACK,  COLOR_YELLOW);
	init_pair(WHITE_ON_RED,    COLOR_WHITE,  COLOR_RED);
	init_pair(GREEN_ON_BLUE,   COLOR_GREEN,  COLOR_BLUE);
	init_pair(YELLOW_ON_BLUE,  COLOR_YELLOW, COLOR_BLUE);

	wbkgd(scr, COLOR_PAIR(WHITE_ON_BLUE));

	nodelay(scr, TRUE);
	noecho();

	/* Create fullscreen log window. When stats are displayed
	   later, it is recreated with appropriate dimensions. */
	win_txt = create_subwindow(0, 0, 0, 0);
	wbkgd(win_txt, COLOR_PAIR(0));

	idlok(win_txt, FALSE);
	/* Get scrolling */
	scrollok(win_txt, TRUE);
	/* Leave cursor where it was */
	leaveok(win_txt, TRUE);

	refresh();

	if ((msr_support = !msr_init())) {
		mprintf("Failed to open msr pseudo-file (missing msr kernel module?)\n");
	}

	if (cqm_is_supported()) {
		if (!msr_support) {
			mprintf("CPU supports CQM but msr module not loaded. Disabling CQM stats\n");

		}
		else {
			if (0 != cqm_get_features(&cqm.features)) {
				mprintf("Failed to get CQM features\n");
				cqm.supported = 0;
			}
			else {
				cqm_init_stat_core(rte_lcore_id());
				cqm.supported = 1;
			}
		}
	}


	init_active_core_ports();
	init_active_eth_ports();

	if (cqm.supported) {
		for (uint8_t i = 0; i < RTE_MAX_LCORE; ++i) {
			cqm_assoc(i, lcore_stats[i].rmid);
		}
	}

	core_port_height = (LINES - 5 - 2 - 3);
	if (core_port_height > nb_tasks_tot) {
		core_port_height = nb_tasks_tot;
	}
	start_tsc = rte_rdtsc();
	global_stats.avg_start = start_tsc + avg_start*rte_get_tsc_hz();
	update_stats();
	update_tsc_stats();
}

static void stats_display_eth_ports(void)
{
	display_lock();
	wattron(win_stat, A_BOLD);
	/* Labels */
	mvwaddstrf(win_stat, 3, 2,   "Port");
	mvwaddstrf(win_stat, 4, 0,   "  Nb");
	mvwvline(win_stat, 4, 4,  ACS_VLINE, nb_active_interfaces + 2);
	mvwaddstrf(win_stat, 4, 5,   "Name");

	mvwvline(win_stat, 2, 13,  ACS_VLINE, nb_active_interfaces + 3);
	mvwaddstrf(win_stat, 3, 14, "                Statistics per second");
	mvwaddstrf(win_stat, 4, 14, "   no mbufs (#)");
	mvwvline(win_stat, 4, 30,  ACS_VLINE, nb_active_interfaces + 1);
	mvwaddstrf(win_stat, 4, 31, "    ierrors (#)");
	mvwvline(win_stat, 4, 47 ,  ACS_VLINE, nb_active_interfaces + 2);

	mvwaddstrf(win_stat, 4, 48, "   RX (k)");
	mvwvline(win_stat, 4, 57,  ACS_VLINE, nb_active_interfaces + 1);
	mvwaddstrf(win_stat, 4, 58, "   TX (k)");
	mvwvline(win_stat, 3, 67,  ACS_VLINE, nb_active_interfaces + 2);

	mvwaddstrf(win_stat, 3, 68, "                          Total Statistics");
	mvwaddstrf(win_stat, 4, 68, "           RX (k)");
	mvwvline(win_stat, 4, 85,  ACS_VLINE, nb_active_interfaces + 1);
	mvwaddstrf(win_stat, 4, 86, "          TX (k)");
	mvwvline(win_stat, 4, 102,  ACS_VLINE, nb_active_interfaces + 1);
	mvwaddstrf(win_stat, 4, 103, "    no mbufs (#)");
	mvwvline(win_stat, 4, 119,  ACS_VLINE, nb_active_interfaces + 1);
	mvwaddstrf(win_stat, 4, 120, "     ierrors (#)");
	mvwvline(win_stat, 3, 136,  ACS_VLINE, nb_active_interfaces + 2);

	wattroff(win_stat, A_BOLD);
	uint8_t count = 0;
	for (uint8_t i = 0; i < nb_interface; ++i) {
		if (dppd_port_cfg[i].active) {
			mvwaddstrf(win_stat, 5 + count, 0, "%4u", i);
			mvwaddstrf(win_stat, 5 + count, 5, "%8s", dppd_port_cfg[i].name);
			count++;
		}
	}
	display_unlock();
}

static void stats_display_core_ports(unsigned chosen_page)
{
	display_lock();
	if (msr_support) {
		col_offset = 20;
	}
	/* Sub-section separator lines */
	mvwvline(win_stat, 4,  4,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 23,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 43,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 53,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 63,  ACS_VLINE, nb_tasks_tot + 1);
	if (msr_support){
		mvwvline(win_stat, 4, 73,  ACS_VLINE, nb_tasks_tot + 1);
		mvwvline(win_stat, 4, 83,  ACS_VLINE, nb_tasks_tot + 1);
	}
	mvwvline(win_stat, 4, 88 + col_offset,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 103 + col_offset, ACS_VLINE, nb_tasks_tot + 1);
	if (cqm.supported) {
		mvwvline(win_stat, 4, 133 + col_offset, ACS_VLINE, nb_tasks_tot + 1);
	}

	wattron(win_stat, A_BOLD);
	/* Section separators (bold) */
	mvwvline(win_stat, 3, 13, ACS_VLINE, nb_tasks_tot + 2);
	mvwvline(win_stat, 3, 33, ACS_VLINE, nb_tasks_tot + 2);
	mvwvline(win_stat, 3, 73 + col_offset, ACS_VLINE, nb_tasks_tot + 2);
	if (cqm.supported) {
		mvwvline(win_stat, 3, 118 + col_offset, ACS_VLINE, nb_tasks_tot + 2);
	}

	/* Labels */
	mvwaddstrf(win_stat, 3, 2,   "Core");
	mvwaddstrf(win_stat, 4, 0,   "  Nb");
	mvwaddstrf(win_stat, 4, 5,   "Name");

	mvwaddstrf(win_stat, 3, 14, " Port Nb/Ring Name");
	mvwaddstrf(win_stat, 4, 14, "       RX");
	mvwaddstrf(win_stat, 4, 24, "       TX");

	if (!msr_support) {
		mvwaddstrf(win_stat, 3, 34, "         Statistics per second         ");
	}
	else {
		mvwaddstrf(win_stat, 3, 34, "                   Statistics per second                   ");
	}
	mvwaddstrf(win_stat, 4, 34, "%s", " Idle (%)");
#ifdef FULL_PRECISION_STATS
	mvwaddstrf(win_stat, 4, 44, "   RX    ");
	mvwaddstrf(win_stat, 4, 54, "   TX    ");
	mvwaddstrf(win_stat, 4, 64, " Drop    ");
#else
	mvwaddstrf(win_stat, 4, 44, "   RX (k)");
	mvwaddstrf(win_stat, 4, 54, "   TX (k)");
	mvwaddstrf(win_stat, 4, 64, " Drop (k)");
#endif
	if (msr_support) {
		mvwaddstrf(win_stat, 4, 74, "      CPP");
		mvwaddstrf(win_stat, 4, 84, "Clk (GHz)");
	}

	mvwaddstrf(win_stat, 3, 74 + col_offset, "              Total Statistics             ");
	mvwaddstrf(win_stat, 4, 74 + col_offset, "            RX");
	mvwaddstrf(win_stat, 4, 89 + col_offset, "            TX");
	mvwaddstrf(win_stat, 4, 104 + col_offset, "          Drop");


	if (cqm.supported) {
		mvwaddstrf(win_stat, 3, 119 + col_offset, "  Cache QoS Monitoring  ");
		mvwaddstrf(win_stat, 4, 119 + col_offset, "occupancy (KB)");
		mvwaddstrf(win_stat, 4, 134 + col_offset, " fraction");
	}
	wattroff(win_stat, A_BOLD);

	uint16_t line_no = 0;
	uint32_t lcore_id = -1;
	while(dppd_core_next(&lcore_id) == 0) {
		const struct lcore_cfg *const cur_core = &lcore_cfg[lcore_id];

		for (uint8_t task_id = 0; task_id < cur_core->nb_tasks; ++task_id) {
			const struct task_args *const targ = &cur_core->targs[task_id];

			if (line_no >= core_port_height * chosen_page && line_no < core_port_height * (chosen_page + 1)) {
				// Core number and name
				mvwaddstrf(win_stat, line_no % core_port_height + 5, 0, "%2u/%1u", lcore_id, task_id);
				mvwaddstrf(win_stat, line_no % core_port_height + 5, 5, "%s", task_id == 0 ? cur_core->name : "");

				// Rx port information
				if (targ->nb_rxrings == 0) {
					wbkgdset(win_stat, link_color(targ->rx_port));
					mvwaddstrf(win_stat, line_no % core_port_height + 5, 16, "%2u", targ->rx_port);
					wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
				}

				for (uint8_t ring_id = 0; ring_id < targ->nb_rxrings; ++ring_id) {
					mvwaddstrf(win_stat, line_no % core_port_height + 5, 14 + ring_id, "%s", targ->rx_rings[ring_id]->name);
				}

				// Tx port information
				if (targ->runtime_flags & TASK_ROUTING) {
					wbkgdset(win_stat, COLOR_PAIR(YELLOW_ON_BLUE));
					mvwaddstrf(win_stat, line_no % core_port_height + 5, 25, "(r:");
					uint8_t pos = 28;
					for (uint8_t i = 0; i < targ->nb_txports; ++i) {
						if (i) {
							mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, ",");
							++pos;
						}
						wbkgdset(win_stat, link_color(targ->tx_port_queue[i].port));
						mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, "%u", targ->tx_port_queue[i].port);
						wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
						++pos;
					}

					for (uint8_t ring_id = 0; ring_id < targ->nb_txrings; ++ring_id) {
						mvwaddstrf(win_stat, line_no % core_port_height + 5, pos++, "%s", targ->tx_rings[ring_id]->name);
					}

					wbkgdset(win_stat, COLOR_PAIR(YELLOW_ON_BLUE));
					mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, ")");
					wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
				}
				else {
					uint8_t pos = 26;
					for (uint8_t i = 0; i < targ->nb_txports; ++i) {
						if (i) {
							mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, ",");
							++pos;
						}

						wbkgdset(win_stat, link_color(targ->tx_port_queue[i].port));
						mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, "%u", targ->tx_port_queue[i].port);
						wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
						pos++;
					}

					for (uint8_t ring_id = 0; ring_id < targ->nb_txrings; ++ring_id) {
						mvwaddstrf(win_stat, line_no % core_port_height + 5, 24 + ring_id, "%s", targ->tx_rings[ring_id]->name);
					}
				}
			}
			DPPD_ASSERT(line_no < RTE_MAX_LCORE*MAX_TASKS_PER_CORE);
			core_port_ordered[line_no] = set_line_no(lcore_id, task_id);
			++line_no;
		}
	}
	display_unlock();
}

void stats_display_layout(struct screen_state screen_state, uint8_t in_place)
{
	uint8_t cur_stats_height;

	if (screen_state.chosen_screen == 0) {
		cur_stats_height = core_port_height;
	}
	else {
		cur_stats_height = nb_active_interfaces;
	}

	display_lock();
	if (!in_place) {
		// moving existing windows does not work
		delwin(win_txt);
		delwin(win_title);
		delwin(win_cmd);
		delwin(win_txt);
		delwin(win_help);

		clear();
	}

	if (!in_place) {
		win_stat = create_subwindow(cur_stats_height + 5, 0, 0, 0);
		win_title = create_subwindow(3, 40, 0, 41);
		win_cmd = create_subwindow(1, 0, cur_stats_height + 5,  0);
		win_txt = create_subwindow(LINES - cur_stats_height - 5 - 2, 0, cur_stats_height + 5 + 1, 0);
		win_help = create_subwindow(1, 0, LINES - 1, 0);
	}
	/* Title box */
	wbkgd(win_title, COLOR_PAIR(BLACK_ON_CYAN));
	box(win_title, 0, 0);
	char title_str[40];
	snprintf(title_str, 40, "%s %s: %s", PROGRAM_NAME, VERSION_STR, dppd_cfg.name);

	mvwaddstrf(win_title, 1, (40 - strlen(title_str))/2, "%s", title_str);

	/* Stats labels and separator lines */
	/* Upper left stats block */
	mvwaddstrf(win_stat, 0, 0, "Time:");
	mvwaddstrf(win_stat, 0, 12, "%s", "%:");
	mvwaddstrf(win_stat, 1, 0, "Rx: %10s pps (%10s avg)", "", "");
	mvwaddstrf(win_stat, 2, 0, "Tx: %10s pps (%10s avg)", "", "");

	/* Upper right stats block */
	mvwaddstrf(win_stat, 0, 100, "Rx:");
	mvwaddstrf(win_stat, 1, 100, "Tx:");
	mvwaddstrf(win_stat, 2, 100, "Diff:");

	/* Command line */
	wbkgd(win_cmd, COLOR_PAIR(BLACK_ON_YELLOW));
	idlok(win_cmd, FALSE);
	/* Move cursor at insertion point */
	leaveok(win_cmd, FALSE);

	/* Help/status bar */
	wbkgd(win_help, COLOR_PAIR(BLACK_ON_WHITE));
	waddstr(win_help, "Enter 'help' or command, <ESC> or 'quit' to exit, <F1> and <F2> to switch screens...");
	wrefresh(win_help);

	/* Log window */
	idlok(win_txt, FALSE);
	/* Get scrolling */
	scrollok(win_txt, TRUE);

	/* Leave cursor where it was */
	leaveok(win_txt, TRUE);

	wbkgd(win_txt, COLOR_PAIR(BLACK_ON_CYAN));
	wrefresh(win_txt);

	/* Draw everything to the screen */
	refresh();
	display_unlock();

	if (screen_state.chosen_screen == 0) {
		stats_display_core_ports(screen_state.chosen_page);
	}
	else {
		stats_display_eth_ports();
	}
}

void end_display(void)
{
	pthread_mutex_destroy(&disp_mtx);

	if (scr != NULL) {
		endwin();
	}
}


static void update_global_stats(uint8_t task_id, struct global_stats *global_stats)
{
	const struct port_stats *port_stats = core_ports[task_id].port_stats;
	const uint64_t delta_t = port_stats->cur_tsc - port_stats->prev_tsc;

	if (core_ports[task_id].flags & PORT_STATS_RX) {
		global_stats->rx_tot += port_stats->diff_rx_pkt_count;
		global_stats->rx_pps += (port_stats->diff_rx_pkt_count) * rte_get_tsc_hz() / delta_t;
	}

	if (core_ports[task_id].flags & PORT_STATS_TX) {
		global_stats->tx_tot += port_stats->diff_tx_pkt_count;
		global_stats->tx_pps += (port_stats->diff_tx_pkt_count) * rte_get_tsc_hz() / delta_t;
	}
}

static void display_core_port_stats(uint8_t task_id)
{
	const int line_no = task_id % core_port_height;

	const struct port_stats *port_stats = core_port_ordered[task_id]->port_stats;

	/* delta_t in units of clock ticks */
	uint64_t delta_t = port_stats->cur_tsc - port_stats->prev_tsc;

	uint64_t empty_cycles = port_stats->diff_empty_cycles;

	if (empty_cycles > delta_t) {
		empty_cycles = 10000;
	}
	else {
		empty_cycles = empty_cycles * 10000 / delta_t;
	}

	// empty_cycles has 2 digits after point, (usefull when only a very small idle time)
	mvwaddstrf(win_stat, line_no + 5, 37, "%3lu.%02lu", empty_cycles / 100, empty_cycles % 100);

#ifdef  HIGH_PREC_STATS
	mvwaddstrf(win_stat, line_no + 5, 44, "%9lu", port_stats->diff_rx_pkt_count * rte_get_tsc_hz() / delta_t);
	mvwaddstrf(win_stat, line_no + 5, 54, "%9lu", port_stats->diff_tx_pkt_count * rte_get_tsc_hz() / delta_t);
	mvwaddstrf(win_stat, line_no + 5, 64, "%9lu", port_stats->diff_tx_pkt_drop * rte_get_tsc_hz() / delta_t);
#else
	// Display per second statistics in Kpps unit
	delta_t *= 1000;

	uint64_t nb_pkt;
	nb_pkt = port_stats->diff_rx_pkt_count * rte_get_tsc_hz();
	if (nb_pkt && nb_pkt < delta_t) {
		mvwaddstrf(win_stat, line_no + 5, 44, "    0.%03lu", nb_pkt * 1000 / delta_t);
	}
	else {
		mvwaddstrf(win_stat, line_no + 5, 44, "%9lu", nb_pkt / delta_t);
	}

	nb_pkt = port_stats->diff_tx_pkt_count * rte_get_tsc_hz();
	if (nb_pkt && nb_pkt < delta_t) {
		mvwaddstrf(win_stat, line_no + 5, 54, "    0.%03lu", nb_pkt * 1000 / delta_t);
	}
	else {
		mvwaddstrf(win_stat, line_no + 5, 54, "%9lu", nb_pkt / delta_t);
	}

	nb_pkt = port_stats->diff_tx_pkt_drop * rte_get_tsc_hz();
	if (nb_pkt && nb_pkt < delta_t) {
		mvwaddstrf(win_stat, line_no + 5, 64, "    0.%03lu", nb_pkt * 1000 / delta_t);
	}
	else {
		mvwaddstrf(win_stat, line_no + 5, 64, "%9lu", nb_pkt / delta_t);
	}
#endif
	if (msr_support) {
		uint8_t lcore_id = core_port_ordered[task_id]->lcore_id;
		uint64_t adiff = lcore_stats[lcore_id].cur_afreq - lcore_stats[lcore_id].prev_afreq;
		uint64_t mdiff = lcore_stats[lcore_id].cur_mfreq - lcore_stats[lcore_id].prev_mfreq;

		if (port_stats->diff_rx_pkt_count && mdiff) {
			mvwaddstrf(win_stat, line_no + 5, 74, "%9lu", delta_t/port_stats->diff_rx_pkt_count*adiff/mdiff/1000);
		}
		else {
			mvwaddstrf(win_stat, line_no + 5, 74, "%9lu", 0L);
		}

		uint64_t mhz;
		if (mdiff)
			mhz = rte_get_tsc_hz()*adiff/mdiff/1000000;
		else
			mhz = 0;

		mvwaddstrf(win_stat, line_no + 5, 84, "%5lu.%03lu", mhz/1000, mhz%1000);
	}

	// Total statistics (packets)
	mvwaddstrf(win_stat, line_no + 5, 74 + col_offset, "%14lu", port_stats->tot_rx_pkt_count);
	mvwaddstrf(win_stat, line_no + 5, 89 + col_offset, "%14lu", port_stats->tot_tx_pkt_count);
	mvwaddstrf(win_stat, line_no + 5, 104 + col_offset, "%14lu", port_stats->tot_tx_pkt_drop);

	if (cqm.supported) {
		uint8_t lcore_id = core_port_ordered[task_id]->lcore_id;
		mvwaddstrf(win_stat, line_no + 5, 119 + col_offset, "%14lu", lcore_stats[lcore_id].cqm_bytes >> 10);
		mvwaddstrf(win_stat, line_no + 5, 134 + col_offset, "%6lu.%02lu", lcore_stats[lcore_id].cqm_fraction/100, lcore_stats[lcore_id].cqm_fraction%100);
	}
}

void update_stats(void)
{
	if (nb_tasks_tot == 0) {
		return;
	}

	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		struct stats *stats = core_ports[task_id].stats;
		struct port_stats *cur_port_stats = core_ports[task_id].port_stats;

		/* Read TX first and RX second, in order to prevent displaying
		   a negative packet loss. Depending on the configuration
		   (when forwarding, for example), TX might be bigger than RX. */

		cur_port_stats->cur_tsc = rte_rdtsc();
		cur_port_stats->diff_tx_pkt_count = rte_atomic32_read(&stats->tx_pkt_count) - cur_port_stats->last_tx_pkt_count;
		cur_port_stats->diff_tx_pkt_drop  = rte_atomic32_read(&stats->tx_pkt_drop)  - cur_port_stats->last_tx_pkt_drop;
		cur_port_stats->diff_rx_pkt_count = rte_atomic32_read(&stats->rx_pkt_count) - cur_port_stats->last_rx_pkt_count;
		cur_port_stats->diff_empty_cycles = rte_atomic32_read(&stats->empty_cycles) - cur_port_stats->last_empty_cycles;
	}

	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		struct port_stats *cur_port_stats = core_ports[task_id].port_stats;

		/* no total stats for empty loops */
		cur_port_stats->tot_rx_pkt_count  += cur_port_stats->diff_rx_pkt_count;
		cur_port_stats->last_rx_pkt_count += cur_port_stats->diff_rx_pkt_count;
		cur_port_stats->tot_tx_pkt_count  += cur_port_stats->diff_tx_pkt_count;
		cur_port_stats->last_tx_pkt_count += cur_port_stats->diff_tx_pkt_count;
		cur_port_stats->tot_tx_pkt_drop   += cur_port_stats->diff_tx_pkt_drop;
		cur_port_stats->last_tx_pkt_drop  += cur_port_stats->diff_tx_pkt_drop;

		cur_port_stats->last_empty_cycles += cur_port_stats->diff_empty_cycles;
	}

	uint64_t cqm_data_core0 = 0;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (lcore_stats[lcore_id].rmid) {
			cqm_read_ctr(&lcore_stats[lcore_id].cqm_data, lcore_stats[lcore_id].rmid);
		}
		if (msr_support) {
			msr_read(&lcore_stats[lcore_id].cur_afreq, lcore_id, 0xe8);
			msr_read(&lcore_stats[lcore_id].cur_mfreq, lcore_id, 0xe7);
		}
	}
	cqm_read_ctr(&cqm_data_core0, 0);

	struct rte_eth_stats eth_stat;

	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (dppd_port_cfg[port_id].active) {
			rte_eth_stats_get(port_id, &eth_stat);

			eth_stats[port_id].no_mbufs = eth_stat.rx_nombuf;
			eth_stats[port_id].ierrors = eth_stat.ierrors;
			eth_stats[port_id].rx_tot = eth_stat.ipackets;
			eth_stats[port_id].tx_tot = eth_stat.opackets;
		}
	}

	global_stats.tx_pps = 0;
	global_stats.rx_pps = 0;
	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		update_global_stats(task_id, &global_stats);
	}

	if (cqm.supported) {
		// update CQM stats (calucate fraction and bytes reported) */
		uint64_t total_monitored = cqm_data_core0*cqm.features.upscaling_factor;

		for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
			if (lcore_stats[lcore_id].rmid) {
				lcore_stats[lcore_id].cqm_bytes = lcore_stats[lcore_id].cqm_data*cqm.features.upscaling_factor;
				total_monitored += lcore_stats[lcore_id].cqm_bytes;
			}
		}
		for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
			if (lcore_stats[lcore_id].rmid && total_monitored) {
				lcore_stats[lcore_id].cqm_fraction = lcore_stats[lcore_id].cqm_bytes*10000/total_monitored;
			}
			else
				lcore_stats[lcore_id].cqm_fraction = 0;
		}
	}
}

static void display_stats_general(void)
{
	uint64_t cur_tsc = rte_rdtsc();
	// Upper left stats block
	wattron(win_stat, A_BOLD);
	mvwaddstrf(win_stat, 0,  6, "%4lu", (cur_tsc - start_tsc)/rte_get_tsc_hz());
	if (global_stats.tx_tot > global_stats.rx_tot) {
		mvwaddstrf(win_stat, 0, 15, "%.4f", 100.0);
	}
	else {
		mvwaddstrf(win_stat, 0, 15, "%.4f", global_stats.tx_tot * 100.0 / global_stats.rx_tot);
	}
	mvwaddstrf(win_stat, 1,  4, "%10lu", global_stats.rx_pps);
	mvwaddstrf(win_stat, 2,  4, "%10lu", global_stats.tx_pps);

	if (cur_tsc > global_stats.avg_start) {
		if (!global_stats.started_avg) {
			global_stats.rx_tot_beg = global_stats.rx_tot;
			global_stats.tx_tot_beg = global_stats.tx_tot;
			global_stats.started_avg = 1;
		}
		else {
			uint64_t avg_tsc_passed = cur_tsc - global_stats.avg_start;
			uint64_t thresh = ((uint64_t)-1)/rte_get_tsc_hz();
			/* Use only precise arithmetic when there is no overflow */
			if (global_stats.rx_tot - global_stats.rx_tot_beg < thresh) {
				global_stats.rx_avg = (global_stats.rx_tot - global_stats.rx_tot_beg)*rte_get_tsc_hz()/avg_tsc_passed;
			}
			else {
				global_stats.rx_avg = (global_stats.rx_tot - global_stats.rx_tot_beg)/(avg_tsc_passed/rte_get_tsc_hz());
			}

			if (global_stats.tx_tot - global_stats.tx_tot_beg < thresh) {
				global_stats.tx_avg = (global_stats.tx_tot - global_stats.tx_tot_beg)*rte_get_tsc_hz()/avg_tsc_passed;
			}
			else {
				global_stats.tx_avg = (global_stats.tx_tot - global_stats.tx_tot_beg)/(avg_tsc_passed/rte_get_tsc_hz());
			}

			mvwaddstrf(win_stat, 1,  20, "%10lu", global_stats.rx_avg);
			mvwaddstrf(win_stat, 2,  20, "%10lu", global_stats.tx_avg);
		}
	}

	// Upper right stats block
	mvwaddstrf(win_stat, 0, 106, "%12lu", global_stats.rx_tot);
	mvwaddstrf(win_stat, 1, 106, "%12lu", global_stats.tx_tot);

	uint64_t loss = 0;
	if (global_stats.rx_tot > global_stats.tx_tot)
		loss = global_stats.rx_tot - global_stats.tx_tot;
	mvwaddstrf(win_stat, 2, 106, "%12lu", loss);

	wattroff(win_stat, A_BOLD);
}

void display_stats_core_ports(__attribute__((unused)) unsigned chosen_page)
{
	display_lock();
	for (uint8_t active_core = core_port_height * chosen_page; active_core < nb_tasks_tot && active_core < core_port_height * (chosen_page + 1); ++active_core) {
		display_core_port_stats(active_core);
	}
	display_stats_general();
	wrefresh(win_stat);
	display_unlock();
	update_tsc_stats();
}

void display_stats_eth_ports(void)
{
	display_lock();

	uint8_t count = 0;
	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (dppd_port_cfg[port_id].active) {
			mvwaddstrf(win_stat, 5 + count, 14, "%16lu", eth_stats[port_id].no_mbufs - eth_stats[port_id].prev_no_mbufs);
			mvwaddstrf(win_stat, 5 + count, 31, "%16lu", eth_stats[port_id].ierrors - eth_stats[port_id].prev_ierrors);

			mvwaddstrf(win_stat, 5 + count, 48, "%9lu", (eth_stats[port_id].rx_tot - eth_stats[port_id].prev_rx_tot)/1000);
			mvwaddstrf(win_stat, 5 + count, 58, "%9lu", (eth_stats[port_id].tx_tot - eth_stats[port_id].prev_tx_tot)/1000);

			mvwaddstrf(win_stat, 5 + count, 69, "%16lu", eth_stats[port_id].rx_tot);
			mvwaddstrf(win_stat, 5 + count, 86, "%16lu", eth_stats[port_id].tx_tot);

			mvwaddstrf(win_stat, 5 + count, 103, "%16lu", eth_stats[port_id].no_mbufs);
			mvwaddstrf(win_stat, 5 + count, 120, "%16lu", eth_stats[port_id].ierrors);
			count++;
		}
	}

	display_stats_general();
	wrefresh(win_stat);
	display_unlock();
	update_tsc_stats();
}

char *get_key(struct screen_state *screen_state, int *resize)
{
#define MAX_STRID      64
#define MAX_CMDLEN     256
	static int lastid = 0;
	static int readid = -1;
	static char str[MAX_CMDLEN] = {0};
	static char last_strs[MAX_STRID][MAX_CMDLEN] = {{0}};
	static unsigned len = 0;
	*resize = 0;
	int testid;
	int refresh_str = 0;
	struct key_val key_val;
	char *ret = NULL;

	display_lock();
	poll_key(&key_val);
	display_unlock();

	switch (key_val.type) {
	case TYPE_SPECIAL_KEY:
		switch (key_val.val) {
		case KEY_ENTER:
			readid = -1;
			if (len == 0) {
				break;
			}
			len = 0;
			if (strstr(str, "quit") == NULL) {
				lastid = (lastid + 1) % MAX_STRID;
				strncpy(last_strs[lastid], str, MAX_CMDLEN);
				memset(str, 0, sizeof(str));
				refresh_str = 1;
				ret = last_strs[lastid];
			} else {
				mprintf("Leaving...\n");
				stop_core(dppd_cfg.core_mask);
				stop_dppd = 1;
			}
			break;
		case KEY_RESIZE:
			*resize = 1;
			break;
		case KEY_BACKSPACE:

			if (len > 0) {
				str[--len] = '\0';
				refresh_str = 1;
			}

			break;
		case KEY_ESC:
			/* ESC */
			mprintf("Leaving...\n");
			stop_core(dppd_cfg.core_mask);
			stop_dppd = 1;

			break;
		case KEY_UP:
			if (readid == -1) {
				testid = lastid;
			}
			else {
				testid = (readid + MAX_STRID - 1) % MAX_STRID;
			}
			if (last_strs[testid][0] != '\0') {
				readid = testid;
				strcpy(str, last_strs[readid]);
				len = strlen(str);
				refresh_str = 1;
			}

			break;
		case KEY_DOWN:
			if (readid == -1) {
				testid = (lastid + 1) % MAX_STRID;
			}
			else {
				testid = (readid + 1) % MAX_STRID;
			}
			if (last_strs[testid][0] != '\0') {
				readid = testid;
				strcpy(str, last_strs[readid]);
				len = strlen(str);
				refresh_str = 1;
			}
			break;
		case KEY_F(1):
			screen_state->chosen_screen = 0;
			break;
		case KEY_F(2):
			screen_state->chosen_screen = 1;
			break;
		case KEY_F(12):
			reset_stats();
			break;
		case KEY_PPAGE:
			if (screen_state->chosen_page) {
				--screen_state->chosen_page;
			}
			break;
		case KEY_NPAGE:
			if (nb_tasks_tot > core_port_height * (screen_state->chosen_page + 1)) {
				++screen_state->chosen_page;
			}
			break;
		}
		break;
	case TYPE_NORMAL_KEY:
		if (len < sizeof(str) - 1) {
			str[len++] = key_val.val;
			refresh_str = 1;
		}
		break;
	case TYPE_NOT_SUPPORTED:
		mprintf("ESC2 car %i:'%c' not supported\n", key_val.val, key_val.val);
		break;
	case TYPE_NO_KEY:
		return NULL;
		break;
	}

	if (refresh_str == 1) {
		display_lock();
		werase(win_cmd);
		waddstr(win_cmd, str);
		wrefresh(win_cmd);
		display_unlock();
	}
	return ret;
}

void reset_stats(void)
{
	memset(&global_stats, 0, sizeof(struct global_stats));

	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		struct port_stats *cur_port_stats = core_ports[task_id].port_stats;
		cur_port_stats->tot_rx_pkt_count = 0;
		cur_port_stats->tot_tx_pkt_count = 0;
		cur_port_stats->tot_tx_pkt_drop = 0;
	}

	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (dppd_port_cfg[port_id].active) {
			rte_eth_stats_reset(port_id);
			memset(&eth_stats[port_id], 0, sizeof(struct eth_stats));
		}
	}

	start_tsc = rte_rdtsc();
	global_stats.avg_start = rte_rdtsc();
}

uint64_t global_total_tx(void)
{
	return global_stats.tx_tot;
}

uint64_t global_total_rx(void)
{
	return global_stats.rx_tot;
}

uint64_t global_avg_tx(void)
{
	return global_stats.tx_avg;
}

uint64_t global_avg_rx(void)
{
	return global_stats.rx_avg;
}

#endif

__attribute__((format(printf, 1, 2))) void mprintf(const char *format, ...)
{

	if (format == NULL) {
		return;
	}

	va_list ap;
	char buffer[1024];

	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);

	file_lock();
	/* Output to log file */
	static FILE *fp = NULL;
	if (fp == NULL) {
		fp = fopen("dppd.log", "w");
	}
	if (fp != NULL) {
		fputs(buffer, fp);
		fflush(fp);
	}
	file_unlock();


#ifdef BRAS_STATS
	display_lock();
	/* Output to screen */
	if (scr == NULL) {
		// ncurses is not yet initialized
		fputs(buffer, stdout);
		fflush(stdout);
	}
	else {
		waddstr(win_txt, buffer);
		wrefresh(win_txt);
	}
	display_unlock();
#endif
}



#ifndef BRAS_STATS

void init_display(__attribute__((unused)) unsigned avg_start){}
void stats_display_layout(__attribute__((unused)) struct screen_state screen_state){}
void end_display(void){}

char *get_key(__attribute__((unused)) struct screen_state *screen_state){return 0;}

void reset_stats(void){}
void update_stats(void){}
void display_stats_core_ports(__attribute((unused)) unsigned chosen_page){}
void display_stats_eth_ports(void){}

uint64_t global_total_tx(void) {return 0;}
uint64_t global_total_rx(void) {return 0;}
uint64_t global_avg_tx(void) {return 0;}
uint64_t global_avg_rx(void) {return 0;}

#endif
