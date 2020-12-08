#!/bin/bash

# Get script directory
INIT_DIR=$PWD
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${SCRIPT_DIR}

# Check file already exists
if test -f "/etc/udev/rules.d/99-uvc.rules"; then
    echo "/etc/udev/rules.d/99-uvc.rules exists."
else
    # Copy uvc rules to udev
    sudo cp 99-uvc.rules /etc/udev/rules.d/
fi

# Check file already exists
if test -f "/lib/udev/rules.d/99-uvc.rules"; then
    echo "/lib/udev/rules.d/99-uvc.rules exists."
else
    # Copy uvc rules to udev
    sudo cp 99-uvc.rules /etc/udev/rules.d/
fi

# Restore inital working directory
cd ${INIT_DIR}