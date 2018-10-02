#!/usr/bin/env bash

# get odroid temp, format, and print
function get_temp0() {
    raw_temp="$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)"
    echo "${raw_temp%???}.${raw_temp#??} C"
}
