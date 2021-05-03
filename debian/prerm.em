#!/bin/bash

if [ -e /etc/udev/rules.d/80-movidius.rules ]; then
    rm /etc/udev/rules.d/80-movidius.rules
fi

exit 0
