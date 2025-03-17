connect_lidar(){
	sudo ip addr flush dev $interface_arg
	ip addr show dev $interface_arg
	sudo ip addr add 10.5.5.1/24 dev $interface_arg
	echo "-------------------CONNECT LIDAR-------------------"
	echo "------------(Press any key to continue)------------"
	read -n 1 -s
	sudo ip link set $interface_arg up
	ip addr show dev $interface_arg
}
stop_dnsmasq(){
	sudo systemctl stop dnsmasq
}
do_dnsmasq(){
	echo "--------------------RUN DNSMASQ--------------------"
	echo "------------(Press any key to continue)------------"
	echo '---(Press CTRL-C when seeing "os1-992005000098")---'
	read -n 1 -s
	sudo dnsmasq -C /dev/null -kd -F 10.5.5.96,10.5.5.96 -i $interface_arg --bind-dynamic
}
ping_lidar(){
	ping -c1 os1-992005000098.local
}
launch_ouster(){
	ros2 launch ouster_ros driver.launch.py viz:=false
	#sensor_hostname:=os1-992005000098.local metadata:=metadata.json
}

non_interactive_connection(){
	sudo ip addr flush dev $interface_arg
	ip addr show dev $interface_arg
	sudo ip addr add 10.5.5.1/24 dev $interface_arg
	sudo ip link set $interface_arg up
	ip addr show dev $interface_arg

	sudo systemctl stop dnsmasq

	sudo dnsmasq -C /dev/null -F 10.5.5.96,10.5.5.96 -i $interface_arg --bind-dynamic
}

# Take arg which is the ethernet interface to use (and have default)
interface_arg=$1
if ((${#interface_arg} == 0))
then
	interface_arg="eth0"
	echo "Took default interface of eth0"
fi

# If "non-interactive" is the second argument, run automatically
# Does not quit automatically, since it runs dnsmasq
non_interactive_arg=$2
if [ "$non_interactive_arg" = "non-interactive" ]; then
	non_interactive_connection
	exit
fi

while true; do
	echo "Connect lidar:	Press 1"
	echo "Stop dnsmasq:	Press 2"
	echo "Run dnsmasq:	Press 3"
	echo "Ping lidar:	Press 4"
	echo "Launch ouster:	Press 5"
	echo "Exit:		Press 6"
	read -n 1 choice
	echo ""
	case $choice in
		"1") connect_lidar;;
		"2") stop_dnsmasq;;
		"3") do_dnsmasq;;
		"4") ping_lidar;;
		"5") launch_ouster;;
		"6") break;;
		*) echo "";	
	esac
done