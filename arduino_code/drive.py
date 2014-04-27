#!/usr/bin/python
import configuration
import threading

lock=threading.Lock()
new_data=True
velocity=0 #linear velocity in meters per second
omega=0 #radial velocity in rad/s

def drive_loop():
  global new_data
  threading.Timer(configuration.drive_loop_time,drive_loop).start()
  print "Hello world"
  with lock:
    if new_data==True:
      print "Velocity:",velocity,"Omega:",omega
    new_data=False


drive_loop()



