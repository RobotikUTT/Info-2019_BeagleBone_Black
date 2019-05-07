#!/bin/bash
if ! ip a | grep vcan0;
then
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan
	sudo ip link set up vcan0
fi