#!/usr/bin/env bash
while read p; do
  if [[ $p =~ "voltage=" ]] ; then
    # some kind of grep here, save into variable
    echo "$p";
  elif [[ $p =~ "temps=" ]] ; then
    echo "$p";
  elif [[ $p =~ "currents=" ]]; then
    echo "$p";
  fi
done <~/.ros/log/pds_node.log
