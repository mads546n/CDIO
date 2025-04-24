#!/usr/bin/env python3
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank
import socket

tank = MoveTank(OUTPUT_A, OUTPUT_B)

HOST = ''
PORT = 12345

server_socket = socket.socket()
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Waiting for connection...")

client, addr = server_socket.accept()
print("Connected to {}".format(addr))

try:
    while True:
        data = client.recv(1)
        if not data:
            break

        cmd = data.decode().strip()
        print(f"Received command: {cmd}")

        if cmd == 'F':
            tank.on(30, 30)
        elif cmd == 'L':
            tank.on(10, 30)
        elif cmd == 'R':
            tank.on(30, 10)
        elif cmd == 'S':
            tank.off()
finally:
    tank.off()
    client.close()
    server_socket.close()
