#!/bin/bash

# Get script directory
INIT_DIR=$PWD
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${SCRIPT_DIR}

sudo cp 99-uvc.rules /etc/udev/rules.d/

# Restore inital working directory
cd ${INIT_DIR}