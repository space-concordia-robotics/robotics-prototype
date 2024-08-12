connect_lidar(){
	sudo ip addr flush dev eth1
	ip addr show dev eth1
	sudo ip addr add 10.5.5.1/24 dev eth1
	echo "-------------------CONNECT LIDAR-------------------"
	echo "------------(Press any key to continue)------------"
	read -n 1 -s
	sudo ip link set eth1 up
	ip addr show dev eth1
}
stop_dnsmasq(){
	sudo systemctl stop dnsmasq
}
do_dnsmasq(){
	echo "--------------------RUN DNSMASQ--------------------"
	echo "------------(Press any key to continue)------------"
	echo '---(Press CTRL-C when seeing "os1-992005000098")---'
	read -n 1 -s
	sudo dnsmasq -C /dev/null -F 10.5.5.96,10.5.5.96 -i eth1 --bind-dynamic
}
ping_lidar(){
	ping -c1 os1-992005000098.local
}
launch_ouster(){
	ros2 launch ouster_ros driver.launch.py
	#sensor_hostname:=os1-992005000098.local metadata:=metadata.json
}
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
