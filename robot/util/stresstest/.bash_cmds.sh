#!/usr/bin/env bash

# get odroid temp, format, and print
function get_temp0() {
    raw_temp="$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)"
    echo "CPU temp: ${raw_temp%???}.${raw_temp#??} C"
}

# get current CPU frequency, format, and print
function get_cpu_freq() {
    cpu_freq_dirty=$(lscpu | grep 'CPU MHz')
    cpu_freq=$(echo $cpu_freq_dirty | rev | cut -d" " -f1 | rev)
    echo "CPU freq: $cpu_freq MHz"
}

function get_cpu_temp_freq() {
    get_temp0 && get_cpu_freq
}
