import subprocess
import re
import os 

def main():
    result = subprocess.run(["sudo", "lshw", "-class", "network"], capture_output=True)
    result_str = result.stdout.decode("utf-8")
    interfaces = re.split("  \\*-network", result_str)

    interface_name = None

    num_interfaces = subprocess.run("ifconfig -a | grep eth[0-9] | wc -l", capture_output=True, shell=True)
    num_interfaces_result = int(num_interfaces.stdout.decode("utf-8"))

    if num_interfaces_result < 2:
        print(f"ERROR: only {num_interfaces_result} network interface(s) exist(s), need at least 2. Ensure the USB-ethernet adapter is plugged in. Exiting.")
        return


    for i in interfaces:
        # Use bus info to detect if it's usb
        if len(i) > 0 and re.search("(bus info: usb)", i):
            # Find interface name
            interface_matches = re.findall("logical name: (eth[0-9]+)", i)
            if len(interface_matches) > 0:
                interface_name = interface_matches[0]
                print("Interface name: " + interface_name)
                break

    if interface_name is None:
        interface_name = "eth0"
        print(f"Using default interface name {interface_name}")

    dir_path = os.path.dirname(os.path.realpath(__file__))

    subprocess.run(["bash", os.path.join(dir_path, "./lidar_setup.sh"), interface_name, "non-interactive"])

if __name__ == "__main__":
    main()