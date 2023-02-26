#!/bin/bash
nmcli monitor |
    while read -r line; do
        echo $line
        if [[ "$line" == "wlan0: disconnected" ]]; then
            nmcli dev wifi hotspot ifname wlan0 ssid McQueen password _kachow_
        elif [[ "$line" == "wlan0: connected" ]]; then
            python /home/mcqueen/main_program.py
        elif [[ "$line" == "wlan0: device removed" ]]; then
            pkill -f main_program.py
        fi
    done
