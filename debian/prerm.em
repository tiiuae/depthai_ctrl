#!/bin/bash

if [ -e /etc/udev/rules.d/80-movidius.rules ]; then
    rm /etc/udev/rules.d/80-movidius.rules
fi

/usr/bin/udevadm control --reload-rules
/usr/bin/udevadm trigger

exit 0
