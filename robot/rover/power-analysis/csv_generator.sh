#!/usr/bin/env bash

# WARNING: this script will overwrite your existing logs, if you have any

if [ -z "$1" ]; then
  # default location for PdsNode.py logs
  LOG=~/.ros/log/pds_node.log
else
  LOG=$PWD/$1
fi

# check if log exists first, exit if DNE
if test -f "$LOG"; then
    echo "$LOG exists"
else
    echo "$LOG DNE, exiting script"
    exit 1
fi

# get first dirty master file to scrape
# NOTE: sh would _not_ work
bash $PWD/initial_scrape.sh $LOG > first.txt

cat first.txt | grep -o '[0-2][0-9]:[0-5][0-9]:[0-5][0-9].*temps.*' > temps.txt
cat first.txt | grep -o '[0-2][0-9]:[0-5][0-9]:[0-5][0-9].*voltage.*' > voltage.txt
cat first.txt | grep -o '[0-2][0-9]:[0-5][0-9]:[0-5][0-9].*currents.*' > currents.txt

echo "Time(s),T1 Temp(C),T2 Temp(C),T3 Temp(C)" > temps_final.csv
sed 's/,[0-9]*: temps=/,/' temps.txt >> temps_final.csv
echo "Time(s),Voltage(V)" > voltage_final.csv
sed 's/,[0-9]*: voltage=/,/' voltage.txt >> voltage_final.csv
echo "Time(s),M1 Current(A),M2 Current(A),M3 Current(A),M4 Current(A),M5 Current(A),M6 Current(A)" > currents_final.csv
sed 's/,[0-9]*: currents=/,/' currents.txt >> currents_final.csv

# clean up garbage
rm $PWD/first.txt
rm $PWD/temps.txt
rm $PWD/voltage.txt
rm $PWD/currents.txt
