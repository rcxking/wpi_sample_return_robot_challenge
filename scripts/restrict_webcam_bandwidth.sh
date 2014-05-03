#!/bin/sh

rmmod uvcvideo
modprobe uvcvideo quirks=128
