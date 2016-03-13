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

# This configuration generates test traffic for the BNG and vBNG use case.
# Note that the port0, port1, port2, port3 variables need to reflect your
# setup.

rate_cpe=85
rate_inet=100
port0=0
port1=1
port2=2
port3=3

echo "

# This configuration generates test traffic for the BNG use case.
# On interfaces 0 and 2, CPE traffic is generated. CPE traffic consists
# of 64 byte QinQ packets. The internal tables in the SUT are loaded
# by first sending ARP packets (during 5 seconds).
#
# On interfaces 1 and 3 Core traffic is generated. The traffic consists
# of 78 byte MPLS/GRE packets. 2 bytes of the GRE ID are randomized.
#
#     CPE    |     Core
#    traffic |    traffic
#            |
#
#       -----------
#   +---2         3---+
#   |   | pkt-gen |   |
#   | +-0         1-+ |
#   | | ----------- | |
#   | |             | |
#   | | ----------- | |
#   | +-0         1-+ |
#   |   |   SUT   |   |
#   +---2         3---+
#       -----------


### CPE (ARP) traffic (sent from port 0) ###############################

### ARP on port $port0
set ip src $port0 10.168.0.1/24
set ip dst $port0 10.168.0.3
type arp $port0
qinq $port0 on

# QinQ inside ARP: set ID's
rnd $port0 0 14 00000000/0XXXXXXX
rnd $port0 1 18 0000XXXX/00XX00XX


### CPE (ARP) traffic (sent from port 2) ###############################
set ip src $port2 10.168.0.1/24
set ip dst $port2 10.168.0.1
type arp $port2
qinq $port2 on

# QinQ inside ARP: set ID's
rnd $port2 0 14 00000000/1XXXXXXX
rnd $port2 1 18 0000XXXX/00XX00XX

########################################################################
start $port0,$port2
sleep 5
stop $port0,$port2
########################################################################

### CPE (UDP) traffic (sent from port 0) ###############################

type ipv4 $port0
proto udp $port0
set $port0 rate $rate_cpe
qinq $port0 on
set $port0 sport 5000
set $port0 dport 5000

# QinQ for UDP packets (set vlans)
rnd $port0 0 14 00000000/0XXXXXXX
rnd $port0 1 18 0000XXXX/00XX00XX
rnd $port0 2 24 00000000/00011100
rnd $port0 3 38 0000101X/XXXXXXXX/XXXX0000/XXXXXXXX
rnd $port0 4 46 00000000/00001000

### CPE (UDP) traffic (sent from port 2) ###############################

type ipv4 $port2
proto udp $port2
set $port2 rate $rate_cpe
qinq $port2 on
set $port2 sport 5000
set $port2 dport 5000
rnd $port2 0 14 00000000/1XXXXXXX
rnd $port2 1 18 0000XXXX/00XX00XX
rnd $port2 2 24 00000000/00011100
rnd $port2 3 38 0000101X/XXXXXXXX/XXXX0000/XXXXXXXX
rnd $port2 4 46 00000000/00001000


### Core (MPLS/GRE) traffic (sent from port 1) #########################
set ip src $port1 10.168.0.3/24
set ip dst $port1 10.168.0.1
set $port1 size 78
mpls $port1 on
mpls_entry $port1 00051140

gre $port1 on

set $port1 sport 5000
set $port1 dport 5000

# randomize lower 2 bytes of the GRE ID
rnd $port1 0 42 00000000/00000000/XXXXXXXX/XXXXXXXX

### Core (MPLS/GRE) traffic (sent from port 3) #########################
set ip src $port3 10.168.0.2/24
set ip dst $port3 10.168.0.1
set $port3 size 78
mpls $port3 on
mpls_entry $port3 00051140

gre $port3 on

set $port3 sport 5000
set $port3 dport 5000

# randomize lower 2 bytes of the GRE ID
rnd $port3 0 42 00000000/00000000/XXXXXXXX/XXXXXXXX

start $port0,$port1,$port2,$port3
sleep 1
set $port0,$port2 rate $rate_cpe
set $port1,$port3 rate $rate_inet
str

" > /tmp/pkt_gen_cfg.pkt

# pkt-gen expects to find lua files relative to the working directory
cd $PKTGEN_DIR/dpdk/examples/pktgen/;

./app/build/app/pktgen -c ffffffffff -n 4 -- -T -P -p 0xffffff -m "[1:2].$port0,[3:4].$port1,[5:6].$port2,[7:8].$port3" -f /tmp/pkt_gen_cfg.pkt
