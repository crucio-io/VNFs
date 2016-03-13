#!/bin/bash


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
# Unbinding 82599 Network Devices from PCI drivers for passing them to VM using PCI PT

if [ -n "$RTE_UNBIND" ] && [ -x $RTE_UNBIND ]; then

  echo "Unbinding using DPDK script"
  lspci | grep 82599 | cut -d' ' -f 1 | sudo xargs -I {} python2.7 $RTE_UNBIND -u {}
  $RTE_UNBIND --status

else

  echo "Unbinding from IXGBE driver"
  lspci | grep 82599 | cut -d' ' -f 1 | sudo xargs -I {} sh -c "echo 0000:{}" 
  lspci | grep 82599 | cut -d' ' -f 1 | sudo xargs -I {} sh -c "echo -n 0000:{} > /sys/bus/pci/drivers/ixgbe/unbind" > /dev/null 2>&1

fi
