#!/usr/bin/env python

#For testing without the robot
import socket
import struct

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 10100  # Port to listen on (non-privileged ports are > 1023)
 
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST,PORT))
sock.listen(5)

print('Listening at:', HOST)

while True:
    sockClient, addrClient = sock.accept()
    print('Got connection from:', addrClient)

    if not sockClient:
        continue

    data = b''
    try:
        while True:
            # Read at least the header
        
            packet = sockClient.recv(4096)
            if not packet:
                break
            data += packet
            print(data)
            sockClient.sendall(data)

    except struct.error as e:
            print('Lost connection from:', addrClient)
            # sockClient.shutdown(socket.SHUT_RDWR)
            sockClient.close()

    except KeyboardInterrupt:
            print('Shutting down socket')
            # sockClient.shutdown(socket.SHUT_RDWR)
            sockClient.close()
            exit()
            
def main_null():
    print("This function is not meant to have a main function")
