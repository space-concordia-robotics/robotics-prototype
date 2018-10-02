#!/usr/bin/env bash
# need to install "stress" via "apt-get install stress" before running this script

# source functions containing script
. .bash_cmds.sh

# end after 60 seconds (default case)
end=$((SECONDS+60))

if [ "$#" -ne 2 ]; then
    end=$((SECONDS+$1))
fi

while [ $SECONDS -lt $end ];
do
get_cpu_temp_freq && sleep 1;
test $? -gt 128 && break;
done& stress -c 4 -t `echo $end`s
