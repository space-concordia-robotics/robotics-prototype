#!/usr/bin/env sh

# since only one person can run the flask server at a time
# get current flask server host IP address from ifconfig, filter out only the IP address
IP=`ifconfig | grep "inet addr:" | head -1 | grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" | head -1`

# if string is empty, set to localhost by default
if [ -z "$IP" ] ;
then
    IP="localhost"
fi

echo "env = {"
echo "  HOST_IP: '$IP'"
echo "}"
