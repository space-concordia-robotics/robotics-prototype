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


def final_report(freq_sum, freq_min, freq_max, temp_sum, temp_min, temp_max, runtime, log_file):
    arch = get_arch()
    avg_freq = freq_sum / (runtime * 1.0)
    avg_temp = temp_sum / (runtime * 1.0)
    # scale down to account for extra trailing 0's
    avg_temp = avg_temp / 1000

    print("\nTemperature stats:")
    log_file.write("\nTemperature stats:\n")

    min_temp_report = report_temp(temp_min, "Min ")
    max_temp_report = report_temp(temp_max, "Max ")
    avg_temp_report = report_temp(avg_temp, "Avg ")

    print(min_temp_report)
    print(max_temp_report)
    print(avg_temp_report)

    log_file.write(min_temp_report + "\n")
    log_file.write(max_temp_report + "\n")
    log_file.write(avg_temp_report + "\n")

    print("\nClock speed stats:")
    log_file.write("\nClock speed stats:\n")

    min_freq_report = report_freq(freq_min, arch, "Min ")
    max_freq_report = report_freq(freq_max, arch, "Max ")
    avg_freq_report = report_freq(avg_freq, arch, "Avg ")

    print(min_freq_report)
    print(max_freq_report)
    print(avg_freq_report)

    log_file.write(min_freq_report + "\n")
    log_file.write(max_freq_report + "\n")
    log_file.write(avg_freq_report + "\n")


def run_stress_test():
    print("Checking for dependencies...")

    devnull = open(os.devnull, "w")
    return_val = subprocess.call(["dpkg", "-s", "stress"], stdout=devnull, stderr=subprocess.STDOUT)
    devnull.close()

    if return_val != 0:
        print("Dependency not found: stress")
        print("To install on debian: sudo apt-get install stress")
        sys.exit(1)

    # default 60 seconds run time
    runtime = 60
    end = time() + runtime

    print()

    arch = get_arch()

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

        stress_test_cmd = "stress -c 4 -t " + str(runtime) + "s"
        subprocess.Popen(stress_test_cmd.split(" "))

        while time() < end:
            i += 1
            temp = int(get_cpu_temp())
            freq = float(get_cpu_freq())
            temp_sum += temp
            freq_sum += freq

            temp_report = report_temp(temp)
            print(temp_report)
            log_file.write(temp_report + "\n")

            freq_report = report_freq(freq, arch)
            print(freq_report)
            log_file.write(freq_report + "\n")

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
        final_report(freq_sum, min_freq, max_freq, temp_sum, min_temp, max_temp, i, log_file)
        log_file.close()
        sys.exit(0)

    final_report(freq_sum, min_freq, max_freq, temp_sum, min_temp, max_temp, i, log_file)
    log_file.close()


if __name__ == "__main__":
    run_stress_test()
