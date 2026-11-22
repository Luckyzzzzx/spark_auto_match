#!/usr/bin/env bash
set -eo pipefail
source /home/spark/spark_noeticbisai/devel/setup.bash
set -u
exec /usr/bin/python3 /home/spark/cake_detect_ros.py
