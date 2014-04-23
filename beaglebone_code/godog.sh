#!/bin/bash

# godog.sh - Shell script to start the beaglefetch server.  If the beaglefetch
# server dies for some reason, this script will monitor beaglefetch using 
# ps -ef | grep and try to restart it:
#
# RPI Rock Raiders
# 4/23/14
#
# Last Updated: Bryant Pong: 4/23/14 - 5:39 PM

while :
do
	if ps -ef | grep "[b]eaglefetch" > /dev/null
		then
			echo "Beaglefetch exists!"
		else
			echo "Beaglefetch died!"
			./beaglefetch
	fi

done
