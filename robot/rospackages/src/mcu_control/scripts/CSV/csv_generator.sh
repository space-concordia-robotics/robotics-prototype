#!/usr/bin/env bash

# check if log exists first
LOG=~/.ros/log/pds_node.log

# exit if DNE
if test -f "$LOG"; then
    echo "$LOG exists"
else
    echo "$LOG DNE, exiting script"
    exit 1
fi

# get first dirty master file to scrape
./initial_scrape.sh > first.txt
cat first.txt | grep -o '[0-2][0-9]:[0-5][0-9]:[0-5][0-9].*temps.*' > temps.txt
cat first.txt | grep -o '[0-2][0-9]:[0-5][0-9]:[0-5][0-9].*voltage.*' > voltage.txt
cat first.txt | grep -o '[0-2][0-9]:[0-5][0-9]:[0-5][0-9].*currents.*' > currents.txt

sed 's/,[0-9]*: temps=/,/' temps.txt > temps_final.csv
sed 's/,[0-9]*: voltage=/,/' voltage.txt > voltage_final.csv
sed 's/,[0-9]*: currents=/,/' currents.txt > currents_final.csv

