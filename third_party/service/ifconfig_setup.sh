#!/bin/bash

desired_subnet="10.5.5.1"  # Replace with your desired subnet

while true; do
	current_subnet=$(ifconfig eth0 | grep -oP '(?<=inet )[0-9.]+')
	echo $current_subnet
	if [ "$current_subnet" != "$desired_subnet" ]; then
		echo "Changing subnet..."
		ifconfig eth0 10.5.5.1 broadcast 10.5.5.255 netmask 255.255.255.0	
		echo "Subnet changed to 10.5.5.1"
	else
		echo "Subnet is already correct."
		sleep 10
	fi
done
