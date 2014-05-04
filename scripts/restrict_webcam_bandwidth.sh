#!/bin/sh

rmmod uvcvideo
modprobe uvcvideo quirks=128

echo "rmmod uvcvideo" >> /etc/rc.local
echo "modprobe uvcvideo quirks=128" >> /etc/rc.local
