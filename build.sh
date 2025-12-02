#!/bin/bash

# Replace the udev rule with the UUID of the SD card
sudo apt-get install exfat-fuse exfat-utils

cp third_party/udev/99-voxl2-sd-card.rules /tmp
cp third_party/service/ifconfig_setup.sh /usr/bin

sed -i 's/KERNEL=="sd\*", ENV{ID_FS_UUID}=="", SYMLINK+="voxl2"/KERNEL=="sd\*", ENV{ID_FS_UUID}=="'"$1"'", SYMLINK+="voxl2"/' /tmp/99-voxl2-sd-card.rules

# Make directory in media to mount to on reboot
target_directory="/media/ouster"
if [ ! -d "$target_directory" ]; then
	mkdir -p "$target_directory"
fi

if [ -x "$(command -v docker)" ]; then
	echo "Docker already installed!"
else
	./third_party/install_docker.bash
fi

docker build -t voxl-os1-server .

cp /tmp/99-voxl2-sd-card.rules /etc/udev/rules.d/
cp third_party/service/media-ouster-watchdog.service /lib/systemd/system
cp third_party/service/media-ouster.mount /lib/systemd/system
cp third_party/service/ifconfig_setup.service /lib/systemd/system
cp third_party/service/voxl-os1-server.service /lib/systemd/system

udevadm control --reload-rules && udevadm trigger

systemctl enable media-ouster-watchdog.service
systemctl enable media-ouster.mount
systemctl enable ifconfig_setup.service
systemctl enable voxl-os1-server.service

echo "Please reboot voxl2!"
