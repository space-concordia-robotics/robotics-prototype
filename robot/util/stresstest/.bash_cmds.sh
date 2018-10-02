#!/usr/bin/env bash
# if running on ARM processor, you need to run this script as super user

# get odroid temp, format, and print
function get_temp0() {
    raw_temp=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
    echo "CPU temp: ${raw_temp%???}.${raw_temp#??} C"
}

# get current CPU frequency, format, and print
function get_cpu_freq() {
    # if on ARM architecture (i.e. odroid)
    architecture_mixed_case=$(uname -a)
    # to lower case
    architecture=${architecture_mixed_case,,}

    if [[ $architecture = *"arm"* ]]; then
        # this cmd will only work if script is run with sudo
        raw_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq)
        echo "CPU freq: $raw_freq KHz"
    else
        freq_dirty=$(lscpu | grep 'CPU MHz')
        freq=$(echo $freq_dirty | rev | cut -d" " -f1 | rev)
        echo "CPU freq: $freq MHz"
    fi
}

function get_cpu_temp_freq() {
    get_temp0 && get_cpu_freq
}
