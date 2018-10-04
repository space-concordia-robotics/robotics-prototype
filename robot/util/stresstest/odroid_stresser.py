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

def report_temp(freq, prepend=""):
    freq = str(freq)

    if "." in freq:
        print(prepend + "CPU temp: " + freq + " C")
    else:
        print(prepend + "CPU temp: " + freq[:2] + "." + freq[2:] + " C")


def report_freq(temp, arch, prepend=""):
    if arch == "arm":
        print(prepend + "CPU freq: " + str(temp) + " KHz")
    elif arch == "x86":
        print(prepend + "CPU freq: " + str(temp) + " MHz")
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
runtime = 60
end = time() + runtime

print()

arch = get_arch()

if len(argv) == 2:
    runtime = int(argv[1])
    end = time() + runtime

max_temp = int(get_cpu_temp()) + 5000
min_temp = max_temp - 5000
max_freq = float(get_cpu_freq()) + 50
min_freq = max_freq - 50
temp_sum = 0
freq_sum = 0

try:
    stress_test_cmd = "stress -c 4 -t " + str(runtime) + "s"
    p = Popen(stress_test_cmd.split(" "))

    while time() < end:
        temp = int(get_cpu_temp())
        freq = float(get_cpu_freq())
        temp_sum += temp
        freq_sum += freq

        report_temp_freq(temp, freq, arch)

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
    print("\nTerminating stress test")
except:
    print("Exception in user code:")
    print("-"*60)
    traceback.print_exc(file=stdout)
    print("-"*60)

# paranoia
#while p.poll() is None:
    # keep waiting
#    sleep(1)

avg_freq = freq_sum / (runtime * 1.0)
avg_temp = temp_sum / (runtime * 1.0)
# scale down to account for extra trailing 0's
avg_temp = avg_temp / 1000

print("\nTemperature stats:")

report_temp(min_temp, "Min ")
report_temp(max_temp, "Max ")
report_temp(avg_temp, "Avg ")

print("\nClock speed stats:")

report_freq(min_freq, arch, "Min ")
report_freq(max_freq, arch, "Max ")
report_freq(avg_freq, arch, "Avg ")

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
