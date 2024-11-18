#!/bin/bash

count="$1"

git config --global --add safe.directory /PX4-Autopilot

cd /src/PX4-Autopilot/

make px4_sitl none_iris > /dev/null 2>&1 &

#while true; do
#  if pgrep -x "px4-simulator" > /dev/null; then
#    ps -ef | grep -E 'px4|px4-simulator' | grep -v grep | awk '{print $2}' | xargs kill -9
#       break
#  fi
#  sleep 1
#done

#./Tools/sitl_multiple_run.sh "$count"

sleep infinity

