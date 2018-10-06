#!/usr/bin/env python3
from subprocess import check_output, Popen
from re import search
from sys import exit, argv, stdout
from time import time, sleep
import traceback
import datetime

#TODO:
# - kill process on early exit
# - look into avg bug
# - make summation/average calculation more space efficient
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
        raw_freq = f.read().rstrip()
        f.close()
    elif arch == "x86":
        cmd = "lscpu | grep 'CPU MHz'"
        output = check_output(cmd, shell=True).decode()
        raw_freq = output.split("\n")[0]
        raw_freq = search('\d+\.\d+', raw_freq).group(0)

    return raw_freq

def report_temp(temp, prepend=""):
    temp = str(temp)

    if "." in temp:
        return prepend + "CPU temp: " + temp + " C"
    else:
        return prepend + "CPU temp: " + temp[:2] + "." + temp[2:] + " C"


def report_freq(freq, arch, prepend=""):
    if arch == "arm":
        return prepend + "CPU freq: " + str(freq) + " KHz"
    elif arch == "x86":
        return prepend + "CPU freq: " + str(freq) + " MHz"
    else:
        return "Unkown architecture: " + arch


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
    # create log file
    time_stamp = time()
    date_stamp = datetime.datetime.fromtimestamp(time_stamp).strftime("%Y-%m-%d_%H:%M:%S")
    log_file = open("stress-test-results_" + date_stamp + ".log", "w")
    log_file.write("Started stress test: " + date_stamp + "\n\n")

    stress_test_cmd = "stress -c 4 -t " + str(runtime) + "s"
    p = Popen(stress_test_cmd.split(" "))

    while time() < end:
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
log_file.write("\nTemperature stats:\n")

min_temp_report = report_temp(min_temp, "Min ")
max_temp_report = report_temp(max_temp, "Max ")
avg_temp_report = report_temp(avg_temp, "Avg ")

print(min_temp_report)
print(max_temp_report)
print(avg_temp_report)

log_file.write(min_temp_report + "\n")
log_file.write(max_temp_report + "\n")
log_file.write(avg_temp_report + "\n")

print("\nClock speed stats:")
log_file.write("\nClock speed stats:\n")

min_freq_report = report_freq(min_freq, arch, "Min ")
max_freq_report = report_freq(max_freq, arch, "Max ")
avg_freq_report = report_freq(avg_freq, arch, "Avg ")

print(min_freq_report)
print(max_freq_report)
print(avg_freq_report)

log_file.write(min_freq_report + "\n")
log_file.write(max_freq_report + "\n")
log_file.write(avg_freq_report + "\n")

log_file.close()

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

