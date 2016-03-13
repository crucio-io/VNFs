##
# Copyright(c) 2010-2014 Intel Corporation.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#   * Neither the name of Intel Corporation nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##

ifeq ($(RTE_SDK),)
$(error "Please define RTE_SDK environment variable")
endif

# Default target, can be overriden by command line or environment
RTE_TARGET ?= x86_64-native-linuxapp-gcc

include $(RTE_SDK)/mk/rte.vars.mk

# binary name
APP = dppd

CFLAGS += -O2
CFLAGS += -fno-stack-protector

ifeq ($(BNG_QINQ),)
	CFLAGS += -DUSE_QINQ
else ifeq ($(BNG_QINQ),y)
	CFLAGS += -DUSE_QINQ
endif

ifeq ($(MPLS_ROUTING),)
	CFLAGS += -DMPLS_ROUTING
else ifeq ($(MPLS_ROUTING),y)
	CFLAGS += -DMPLS_ROUTING
endif

ifeq ($(dbg),y)
CFLAGS += -DDPPD_CMD_DUMP
EXTRA_CFLAGS += -ggdb
endif

CFLAGS += -DBRAS_PREFETCH_OFFSET=2
CFLAGS += -DHARD_CRC
CFLAGS += -DBRAS_STATS
#CFLAGS += -DBRAS_RX_BULK
#CFLAGS += -DASSERT
#CFLAGS += -DFULL_PRECISION_STATS
#CFLAGS += -DDPPD_CMD_STAT_RX
#CFLAGS += -DDPPD_CMD_DUMP
#CFLAGS += -DENABLE_VERBOSITY
CFLAGS += -DGRE_TP
CFLAGS += -std=gnu99
CFLAGS += -D_GNU_SOURCE                # for PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP
CFLAGS += $(WERROR_FLAGS)
CFLAGS += -Wno-unused
LDFLAGS += -lncurses -lncursesw

# if symbols are not needed then comment out next line
# EXTRA_CFLAGS += -ggdb

# all source are stored in SRCS-y
SRCS-y := handle_none.c
SRCS-y += handle_drop.c
SRCS-y += handle_qos.c
SRCS-y += handle_qinq_decap4.c
SRCS-y += handle_routing.c
SRCS-y += handle_unmpls.c
SRCS-y += handle_qinq_decap6.c
SRCS-y += handle_gre_decap_encap.c
SRCS-y += handle_fwd.c
SRCS-y += handle_lb_qinq.c
SRCS-y += handle_lb_pos.c
SRCS-y += handle_lb_net.c
SRCS-y += handle_qinq_encap4.c
SRCS-y += handle_qinq_encap6.c
SRCS-y += handle_classify.c
SRCS-y += handle_l2fwd.c
SRCS-y += handle_sig.c
SRCS-y += handle_police.c
SRCS-y += handle_acl.c
SRCS-y += handle_ipv6_tunnel.c
SRCS-$(CONFIG_RTE_LIBRTE_PIPELINE) += handle_pf_acl.c


SRCS-y += thread_none.c
SRCS-y += thread_basic.c
SRCS-y += thread_qos.c
SRCS-y += thread_generic.c
SRCS-$(CONFIG_RTE_LIBRTE_PIPELINE) += thread_pipeline.c

SRCS-y += dppd_args.c dppd_cfg.c dppd_cksum.c dppd_port_cfg.c

SRCS-y += cfgfile.c clock.c commands.c cqm.c msr.c
SRCS-y += debug.c defaults.c display.c display_utils.c
SRCS-y += hash_utils.c main.c parse_utils.c
SRCS-y += read_config.c route.c run.c rx_pkt.c
SRCS-y += task_init.c tx_pkt.c tx_worker.c expire_cpe.c ip_subnet.c

include $(RTE_SDK)/mk/rte.extapp.mk
