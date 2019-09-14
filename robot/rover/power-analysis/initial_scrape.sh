#!/usr/bin/env bash

if [ -z "$1" ]; then
  # default location for PdsNode.py logs
  LOG=~/.ros/log/pds_node.log
else
  LOG=$1
  echo "$1"
fi

while read p; do
  if [[ $p =~ "voltage=" ]]; then
    # some kind of grep here, save into variable
    echo "$p";
  elif [[ $p =~ "temps=" ]]; then
    echo "$p";
  elif [[ $p =~ "currents=" ]]; then
    echo "$p";
  fi
done <$LOG
