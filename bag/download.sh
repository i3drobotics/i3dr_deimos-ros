#!/bin/bash

# Get script directory
INIT_DIR=$PWD
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${SCRIPT_DIR}

# Download sample bag file
curl -o scene001.bag https://github.com/i3drobotics/i3dr_deimos-ros/releases/download/v1.0.0/scene001.bag