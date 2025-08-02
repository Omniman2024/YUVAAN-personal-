#!/bin/bash

# Define the specific IP address to watch
target_ip="192.168.2.100"  # Replace with the IP address you want to monitor
count=0

# Listen for ICMP packets from the specific IP
sudo tcpdump -l -i any icmp and src $target_ip | while read -r line; do
    ((count++))  # Increment count for each ICMP packet from target_ip

    # Check if we've received 5 packets from target_ip
    if [ "$count" -ge 5 ]; then
        echo "Received 5 ICMP packets from $target_ip - executing command." & ./executables2.sh

        count=0
	break
    fi
done
