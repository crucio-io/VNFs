#!/bin/sh
#
#  Copyright(c) 2010-2014 Intel Corporation.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#    * Neither the name of Intel Corporation nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Helper script for vBNG in SRT 1.2
# To be executed on the host
# Starting the VM for the use-case with user space virtio / "vBNG on OVS"

taskset 0x3e1003e1 qemu-kvm -cpu host -enable-kvm -m 4096 -smp 12,cores=12,threads=1,sockets=1 \
 -name VM1 -hda fc20min.qcow2 -mem-path /dev/hugepages -mem-prealloc -vnc :2 -daemonize\
 -net nic,model=virtio,macaddr=00:1e:77:68:09:fd \
 -net tap,ifname=tap1,script=no,downscript=no,vhost=on \
 -netdev type=tap,id=net1,script=no,downscript=no,ifname=dpdk0,vhost=on \
 -device virtio-net-pci,netdev=net1,mac=00:00:01:00:00:01,csum=off,gso=off,guest_tso4=off,guest_tso6=off,guest_ecn=off\
 -netdev type=tap,id=net2,script=no,downscript=no,ifname=dpdk1,vhost=on \
 -device virtio-net-pci,netdev=net2,mac=00:00:01:00:00:02,csum=off,gso=off,guest_tso4=off,guest_tso6=off,guest_ecn=off\
 -netdev type=tap,id=net3,script=no,downscript=no,ifname=dpdk2,vhost=on \
 -device virtio-net-pci,netdev=net3,mac=00:00:01:00:00:03,csum=off,gso=off,guest_tso4=off,guest_tso6=off,guest_ecn=off\
 -netdev type=tap,id=net4,script=no,downscript=no,ifname=dpdk3,vhost=on \
 -device virtio-net-pci,netdev=net4,mac=00:00:01:00:00:04,csum=off,gso=off,guest_tso4=off,guest_tso6=off,guest_ecn=off\


