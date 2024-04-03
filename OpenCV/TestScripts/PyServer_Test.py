# https://stackoverflow.com/questions/72483906/sending-and-receiving-lists-in-socket
# https://stackoverflow.com/questions/16373510/improving-speed-of-python-module-import
# https://stackoverflow.com/questions/68345812/is-there-a-way-to-pass-flags-to-runpy

import socket
import runpy

import cv2
import datetime
import time
import numpy
import getopt
import sys
import math

import json
import struct

def receive_list(buffer):
    # determine how many bytes are in the prefix
    prefix_len = struct.calcsize("!i")

    # we really should be more careful about indexes here ;)
    data_len = struct.unpack("!i", buffer[:prefix_len])[0]
    data = buffer[prefix_len: data_len + prefix_len]

    l = json.loads(data)

    return l

HOST = 'localhost'
PORT = 50007

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    serversocket.bind((HOST, PORT))
except:
    print("Server is already running")
    exit(1)

# Start server with maximum 100 connections
serversocket.listen(100)

while True:
    connection, address = serversocket.accept()
    buf = connection.recv(4096)
    if len(buf) > 0:
        list = receive_list(buf)
        now = time.time()
        sys.argv = list[1:]
        runpy.run_path(path_name=list[1], run_name="__main__")
        after = time.time()
        duration = after - now
        print("I received " + list[1] + " script and it took " + str(duration) + " seconds to execute it")
