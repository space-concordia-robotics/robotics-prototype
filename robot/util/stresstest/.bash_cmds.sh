#!/usr/bin/env bash
# if running on ARM processor, you need to run this script as super user

# get current CPU temp
function get_cpu_temp() {
    raw_temp=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
    echo $raw_temp
}

# get current CPU frequency
function get_cpu_freq() {
    # if on ARM architecture (i.e. odroid)
    architecture_mixed_case=$(uname -a)
    # to lower case
    architecture=${architecture_mixed_case,,}

    if [[ $architecture = *"arm"* ]]; then
        # this cmd will only work if script is run with sudo
        raw_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq)
    else
        freq_dirty=$(lscpu | grep 'CPU MHz')
        raw_freq=$(echo $freq_dirty | rev | cut -d" " -f1 | rev)
    fi

    echo $raw_freq
}

# format and print CPU temp
function print_cpu_temp() {
    raw_temp=`get_cpu_temp`
    echo "CPU temp: ${raw_temp%???}.${raw_temp#??} C"
}

# format and print CPU freq
function print_cpu_freq() {
    raw_freq=`get_cpu_freq`

    # if on ARM architecture (i.e. odroid)
    architecture_mixed_case=$(uname -a)
    # to lower case
    architecture=${architecture_mixed_case,,}

    if [[ $architecture = *"arm"* ]]; then
        echo "CPU freq: $raw_freq KHz"
    else
        echo "CPU freq: $raw_freq MHz"
    fi
}

function print_cpu_temp_freq() {
    print_cpu_temp `get_cpu_temp` && print_cpu_freq `get_cpu_freq`
}
