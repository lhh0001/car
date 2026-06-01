#!/bin/bash
# Entrypoint for vehicle-sim Docker container
set -e

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

exec "$@"
