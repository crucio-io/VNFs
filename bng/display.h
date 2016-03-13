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

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

struct screen_state {
	char chosen_screen;
	int chosen_page;
};

void init_display(unsigned avg_start);
void stats_display_layout(struct screen_state screen_state, uint8_t in_place);
void end_display(void);

char *get_key(struct screen_state *screen_state, int *resize);

void reset_stats(void);
void update_stats(void);
void display_stats_core_ports(unsigned chosen_page);
void display_stats_eth_ports(void);


void mprintf(const char *format, ...) __attribute__((format(printf, 1, 2)));

#ifdef ENABLE_VERBOSITY
extern uint8_t verbose;
#endif

__attribute__((format(printf, 1, 2))) static inline void mprintf_verbose(__attribute__((unused)) const char *format, ...)
{
#ifdef ENABLE_VERBOSITY
	if (!verbose) {
		return;
	}

	va_list ap;
	char buffer[1024];

	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	mprintf("%s",buffer);
	va_end(ap);
#endif
}

__attribute__((format(printf, 2, 3))) void mprintf_once(uint8_t idx, __attribute__((unused)) const char *format, ...);

uint64_t global_total_tx(void);
uint64_t global_total_rx(void);
uint64_t global_avg_tx(void);
uint64_t global_avg_rx(void);


#endif /* _DISPLAY_H_ */
