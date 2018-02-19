#!/bin/bash

iface=$1
channel=4
essid="exploration"
ip=192.168.1.$2

echo "Disable network manager"
sudo service network-manager stop

echo "Put the interface $iface down"
sudo ip link set $iface down

echo "Switch $iface to ad-hoc mode"
sudo iwconfig $iface mode ad-hoc

echo "Set the channel of $iface to $channel"
sudo iwconfig $iface channel $channel

echo "Set network ESSID to $essid"
sudo iwconfig $iface essid $essid

echo "Bring the interface $iface back up"
sudo ip link set $iface up

echo "Set IP to $ip"
sudo ip addr flush $iface
sudo ip addr add $ip/24  broadcast 192.168.1.255 dev $iface

sudo ip link set $iface down
sleep 2
sudo ip link set $iface up

iwconfig

echo "finish"