#!/usr/bin/python

import socket

TCP_IP = '127.0.0.1'
TCP_PORT = 9001
MESSAGE = "HELLO, WORLD!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

while True:
	s.send(MESSAGE)
	data = s.recv(1024)
	print("Received data: " + str(data))
