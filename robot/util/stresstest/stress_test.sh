#!/usr/bin/env bash

# check if stress is installed
echo "Checking for dependencies..."
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' stress|grep "install ok installed")

if [ "" == "$PKG_OK" ]; then
    echo "Dependency not found: stress"
    echo "To install: sudo apt-get install stress"
    exit 1
fi

# source functions containing script
. bash_cmds.sh

# end after 60 seconds (default case)
end=$((SECONDS+60))

if [ "$#" -ne 2 ]; then
    end=$((SECONDS+$1))
fi

echo

while [ $SECONDS -lt $end ];
do
print_cpu_temp_freq && sleep 1;
test $? -gt 128 && break;
done& stress -c 4 -t `echo $end`s
