#!/usr/bin/env python3
from re import search
from time import time, sleep
import datetime
import os
import subprocess
import sys


def get_arch():
    output = subprocess.check_output(["uname", "-a"]).decode()
    output = output.lower()

    if "arm" in output:
        return "arm"

    if "x86" in output:
        return "x86"

    return "unknown architecture:\n" + output


def get_cpu_temp():
    temp_file = open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r")
    raw_temp = temp_file.read().rstrip()
    temp_file.close()

    return raw_temp


def get_cpu_freq():
    arch = get_arch()
    raw_freq = -1

    if arch == "arm":
        # this requires sudo priveleges to open
        freq_file = open("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq", "r")
        raw_freq = freq_file.read().rstrip()
        freq_file.close()
    elif arch == "x86":
        cmd = "lscpu | grep 'CPU MHz'"
        output = subprocess.check_output(cmd, shell=True).decode()
        raw_freq = output.split("\n")[0]
        raw_freq = search(r'\d+\.\d+', raw_freq).group(0)

    return raw_freq


def report_temp(temp, prepend=""):
    temp = str(temp)

    if "." in temp:
        return prepend + "CPU temp: " + temp + " C"

    return prepend + "CPU temp: " + temp[:2] + "." + temp[2:] + " C"


def report_freq(freq, arch, prepend=""):
    if arch == "arm":
        return prepend + "CPU freq: " + str(freq) + " KHz"

    if arch == "x86":
        return prepend + "CPU freq: " + str(freq) + " MHz"

    return "Unkown architecture: " + arch


# average is to be calculated from sum, min and max values already supplied
def final_report(freq_min_max_sum, temp_min_max_sum, runtime, log_file):
    arch = get_arch()
    avg_freq = freq_min_max_sum[2] / (runtime * 1.0)
    avg_temp = temp_min_max_sum[2] / (runtime * 1.0)
    # scale down to account for extra trailing 0's
    avg_temp = avg_temp / 1000

    print("\nTemperature stats:")
    log_file.write("\nTemperature stats:\n")

    print_and_log(report_temp(temp_min_max_sum[0], "Min "), log_file)
    print_and_log(report_temp(temp_min_max_sum[1], "Max "), log_file)
    print_and_log(report_temp(avg_temp, "Avg "), log_file)

    print("\nClock speed stats:")
    log_file.write("\nClock speed stats:\n")

    print_and_log(report_freq(freq_min_max_sum[0], arch, "Min "), log_file)
    print_and_log(report_freq(freq_min_max_sum[1], arch, "Max "), log_file)
    print_and_log(report_freq(avg_freq, arch, "Avg "), log_file)


def check_dependencies():
    print("Checking for dependencies...")

    devnull = open(os.devnull, "w")
    return_val = subprocess.call(["dpkg", "-s", "stress"], stdout=devnull, stderr=subprocess.STDOUT)
    devnull.close()

    if return_val != 0:
        print("Dependency not found: stress")
        print("To install on debian: sudo apt-get install stress")
        sys.exit(1)


def print_and_log(msg, log_file):
    print(msg)
    log_file.write(msg + "\n")


def run_stress_test():
    # default 60 seconds run time
    runtime = 60
    end = time() + runtime
    arch = get_arch()

    print()

    # get user defined runtime, if given
    if len(sys.argv) == 2:
        runtime = int(sys.argv[1])
        end = time() + runtime

    max_temp = int(get_cpu_temp()) + 5000
    min_temp = max_temp - 5000
    max_freq = float(get_cpu_freq()) + 50
    min_freq = max_freq - 50
    temp_sum = 0
    freq_sum = 0
    i = 0

    try:
        # create log file
        time_stamp = time()
        date_stamp = datetime.datetime.fromtimestamp(time_stamp).strftime("%Y-%m-%d_%H:%M:%S")
        log_file = open("stress-test-results_" + date_stamp + ".log", "w")
        log_file.write("Started stress test: " + date_stamp + "\n\n")

        subprocess.Popen(("stress -c 4 -t " + str(runtime) + "s").split(" "))

        while time() < end:
            i += 1
            temp = int(get_cpu_temp())
            freq = float(get_cpu_freq())
            temp_sum += temp
            freq_sum += freq

            print_and_log(report_temp(temp), log_file)
            print_and_log(report_freq(freq, arch), log_file)

            if temp > max_temp:
                max_temp = temp

            if temp < min_temp:
                min_temp = temp

            if freq > max_freq:
                max_freq = freq

            if freq < min_freq:
                min_freq = freq

            sleep(1)

    except KeyboardInterrupt:
        print("\nTerminating stress test early")
        final_report([min_freq, max_freq, freq_sum], [min_temp, max_temp, temp_sum], i, log_file)
        log_file.close()
        sys.exit(0)

    final_report([min_freq, max_freq, freq_sum], [min_temp, max_temp, temp_sum], i, log_file)
    log_file.close()


if __name__ == "__main__":
    run_stress_test()
