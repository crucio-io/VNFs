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

rate=100
port0=0
port1=1
port2=2
port3=3

port0_pkt_size=64
port1_pkt_size=64
port2_pkt_size=64
port3_pkt_size=64

echo "
 set mac $port0 00:00:01:00:00:00
 set mac $port1 00:00:02:00:00:01
 set mac $port2 00:00:03:00:00:02
 set mac $port3 00:00:04:00:00:03

 type ipv4 $port0
 type ipv4 $port1
 type ipv4 $port2
 type ipv4 $port3

 proto udp $port0
 proto udp $port1
 proto udp $port2
 proto udp $port3

 set $port0 size $port0_pkt_size
 set $port1 size $port2_pkt_size
 set $port2 size $port3_pkt_size
 set $port3 size $port3_pkt_size


 set $port0 rate $rate
 set $port1 rate $rate
 set $port2 rate $rate
 set $port3 rate $rate

 start all
 cls
" > /tmp/pkt_gen_cfg.pkt

# pkt-gen expects to find lua files relative to the working directory

cd $PKTGEN_DIR/dpdk/examples/pktgen/;

./app/build/app/pktgen -c ffffffffff -n 4 -- -T -P -p 0xffffff -m "[1:2].$port0,[3:4].$port1,[5:6].$port2,[7:8].$port3" -f /tmp/pkt_gen_cfg.pkt
