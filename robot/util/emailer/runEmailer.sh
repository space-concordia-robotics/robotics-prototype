#!/usr/bin/env bash

# when adding this script to be run on startup whether using cronjob or systemd services
# make sure to replace the beginning "node" with the output of `which node`
# otherwise it will not work
node /home/odroid/emailer/emailIPAddress.js > /home/odroid/emailer/emailer.log
