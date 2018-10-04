#!/usr/bin/env python
from subprocess import check_output, Popen
from re import search
from sys import exit, argv, stdout
from time import time, sleep
import traceback

def get_arch():
    output = check_output(["uname", "-a"]).decode()
    output = output.lower()

    if "arm" in output:
        return "arm"
    elif "x86" in output:
        return "x86"
    else:
        return "unknown architecture:\n" + output

def get_cpu_temp():
    f = open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r")
    raw_temp = f.read().rstrip()
    f.close()

    return raw_temp

def get_cpu_freq():
    arch = get_arch()
    raw_freq = -1

    if arch == "arm":
        # this requires sudo priveleges to open
        f = open("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq", "r")
        raw_freq = f.read.rstrip()
        f.close()
    elif arch == "x86":
        cmd = "lscpu | grep 'CPU MHz'"
        output = check_output(cmd, shell=True).decode()
        raw_freq = output.split("\n")[0]
        raw_freq = search('\d+\.\d+', raw_freq).group(0)

    return raw_freq

def report_temp(freq):
    print("CPU temp: " + freq[:2] + "." + freq[2:] + " C")

def report_freq(temp, arch):
    if arch == "arm":
        print("CPU freq: " + str(temp) + " KHz")
    elif arch == "x86":
        print("CPU freq: " + str(temp) + " MHz")
    else:
        print("Uknown architecture:")
        print(arch)


def report_temp_freq(temp, freq, arch):
    report_temp(temp)
    report_freq(freq, arch)

def report_freq_temp(freq, temp, arch):
    report_freq(freq, arch)


print("Checking for dependencies...")
PKG_OK="dpkg-query -W --showformat='${Status}\n' stress | grep 'install ok installed'"

output = check_output(PKG_OK, shell=True).decode()

if output == "":
    print("Dependency not found: stress")
    print("To install on debian: sudo apt-get install stress")
    exit(1)

# default 60 seconds run time
end = time() + 60

print()

arch = get_arch()

if len(argv) == 2:
    runtime = int(argv[1])
    end = time() + runtime

try:
    stress_test_cmd = "stress -c 4 -t " + str(runtime) + "s"
    Popen(stress_test_cmd.split(" "))

    while time() < end:
        report_temp_freq(get_cpu_temp(), get_cpu_freq(), arch)
        sleep(1)

except:
    print("\nTerminating stress test")

# self assurance tests
"""
report_temp_(temp)

cpu_arch = get_arch()
print("get_arch: " + cpu_arch)
cpu_temp = get_cpu_temp()
print("get_cpu_temp: " + cpu_temp)
cpu_freq = get_cpu_freq()
print("get_cpu_freq: " + cpu_freq + "\n")

report_temp(cpu_temp)
report_freq(cpu_freq, cpu_arch)

print("")

report_temp_freq(cpu_temp, cpu_freq, cpu_arch)
report_freq_temp(cpu_freq, cpu_temp, cpu_arch)
"""
