#!/bin/bash

# Get script directory
INIT_DIR=$PWD
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${SCRIPT_DIR}

# Copy uvc rules to udev
sudo cp 99-uvc.rules /etc/udev/rules.d/
sudo cp 99-uvc.rules /lib/udev/rules.d/

# Restore inital working directory
cd ${INIT_DIR}