#!/usr/bin/python
import time
import configuration
import drive

while 1:
  time.sleep(5)
  with drive.lock:
    drive.velocity=drive.velocity+1
    drive.omega=drive.omega+1
    drive.new_data=True
