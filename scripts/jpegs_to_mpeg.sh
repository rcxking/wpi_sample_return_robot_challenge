#!/bin/bash

cd ../rockie_code/src/stereo_historian/scripts/images/right

mencoder "mf://*.jpg" -mf type=jpg:fps=15 -o output.mpg -speed 1 -ofps 30 -ovc lavc -lavcopts vcodec=mpeg2video:vbitrate=2500 -oac copy -of mpeg
