#!/usr/bin/python

import socket

host = ''
port = 9001
backlog = 5
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
