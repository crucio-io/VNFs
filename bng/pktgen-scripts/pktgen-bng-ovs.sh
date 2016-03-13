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

# This configuration generates test traffic for the Open vSwitch
# comatible vBNG use-case. Note that the port0, port1, port2, port3
# variables need to reflect your setup.

rate_cpe=23
rate_inet=20
port0=0
port1=1
port2=2
port3=3

echo "
 set mac $port0 90:e2:ba:46:46:c0
 set mac $port2 90:e2:ba:46:3b:54
 set mac $port1 90:e2:ba:53:7c:7c
 set mac $port3 00:1b:21:b1:23:6c


### CPE traffic (sent from port 0 && port 2) ###########################

type ipv4 $port0
proto udp $port0
set $port0 rate $rate_cpe
set $port0 sport 5000
set $port0 dport 5000


rnd $port0 0 26 00000000/00000X00/00000000/000X00XX # src ip
rnd $port0 1 30 0000101X/00000000/0XXX0000/00000000 # dst ip
rnd $port0 2 16 00000000/00011010/00000000/00000000 # fixed IP size

type ipv4 $port2
proto udp $port2
set $port2 rate $rate_cpe
set $port2 sport 5000
set $port2 dport 5000

rnd $port2 0 26 00000000/00001X00/00000000/00XX00XX # src ip
rnd $port2 1 30 0000101X/00000000/0XXX0000/00000000 # dst ip
rnd $port2 2 16 00000000/00011010/00000000/00000000 # fixed IP size

### Core traffic (sent from port 1 && port 3) #########################
type ipv4 $port1
proto udp $port1
set ip src $port1 10.1.2.3/24
set ip dst $port1 10.0.0.0
set $port1 size 74

gre $port1 on

set $port1 sport 5000
set $port1 dport 5000

# randomize lower 2 bytes of the GRE ID
rnd 1 0 38 00000000/00000000/XX000000/000000XX

type ipv4 $port3
proto udp $port3
set ip src $port3 10.1.2.3/24
set ip dst $port3 10.0.0.0
set $port3 size 74

gre $port3 on

set $port3 sport 5000
set $port3 dport 5000

# randomize lower 2 bytes of the GRE ID
rnd $port3 0 38 00000000/00000000/XX000000/000000XX

### Start all ports #########################
sleep 1
#start 0-3
stop 0-3
sleep 1
set $port0,$port2 rate $rate_cpe
set $port1,$port3 rate $rate_inet

str
 cls
" > /tmp/pkt_gen_cfg.pkt

# pkt-gen expects to find lua files relative to the working directory
cd $PKTGEN_DIR/dpdk/examples/pktgen/;

./app/build/app/pktgen -c ffffffffff -n 4 -- -T -P -p 0xffffff -m "[1:2].$port0,[3:4].$port1,[5:6].$port2,[7:8].$port3" -f /tmp/pkt_gen_cfg.pkt
