#!/usr/bin/env bash

EMAILER_LOG="/home/odroid/emailer/emailer.log"

activeConnection=$(nmcli connection show --active)
echo "activeConnection: $activeConnection" > $EMAILER_LOG

if [[ "$activeConnection" =~ "RoverOBC" ]]; then
    echo "Rover configuration selected, exiting script" >> $EMAILER_LOG
    exit 1
else
    # WARNING: when adding this script to be run on startup whether using cronjob or systemd services
    # make sure to replace the beginning "node" with the direct path which you can check with the output of `which node`
    # otherwise it will not work. Example: "/home/odroid/.nvm/versions/node/v10.13.0/bin/node"
    node /home/odroid/emailer/emailIPAddress.js >> $EMAILER_LOG
fi
