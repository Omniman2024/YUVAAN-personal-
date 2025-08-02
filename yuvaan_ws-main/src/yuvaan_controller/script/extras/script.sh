#!/bin/bash

# Define the specific IP address to watch
target_ip="192.168.2.100"
count=0

# Listen for ICMP packets from the specific IP
# NO roscore here!
sudo tcpdump -l -i any icmp and src $target_ip | while read -r line; do
    ((count++))

    if [ "$count" -ge 5 ]; then
        echo "Received 5 ICMP packets from $target_ip - executing command." & ./executables.sh # This script will also be updated
        count=0
	    break
    fi
done

