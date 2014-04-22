#!/usr/bin/python

'''
beaglefetch.py - Main server code for the Beaglebone Black.

RPI Rock Raiders
4/16/14

Last Updated: Bryant Pong: 4/22/14 - 12:59 PM
'''

# Socket Library  
import socket

'''
Socket Settings:
The hostname being set to '' means listen on the localhost port (192.168.7.2).
The port number is 9001 because IT'S OVER 9000!
The backlog is the number of clients to be queued.  We only have 1 client.
'''
host = '192.168.7.2'
port = 9001
backlog = 1
size = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Starting up on localhost port %s" % port)
s.bind((host, port))
s.listen(backlog)
while 1:
	client, address = s.accept()

	try:
		print("Connection from " + str(address))

		while True:
			data = client.recv(size)
			if data:
				client.send(data)
	finally:
		client.close()
