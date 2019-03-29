#!/usr/bin/env bash
(stty raw; while IFS= read -r line; do
printf '%s\n' "$line" > /home/odroid/received.txt; done) < /dev/ttySAC0
