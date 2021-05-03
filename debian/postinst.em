#!/bin/bash

mkdir -p /etc/udev/rules.d

cat << EOF > /etc/udev/rules.d/80-movidius.rules

SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"

EOF

chmod 644 /etc/udev/rules.d/80-movidius.rules

exit 0
