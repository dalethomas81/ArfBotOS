import socket
import sys
import json
import struct

HOST = 'localhost'
PORT = 50007

clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientsocket.connect((HOST, PORT))

data = json.dumps(sys.argv)
data_len = len(data)
payload = struct.pack('!i', data_len) + data.encode()

clientsocket.send(payload)