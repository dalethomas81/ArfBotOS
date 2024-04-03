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

import os

#https://stackoverflow.com/questions/1218933/can-i-redirect-the-stdout-into-some-sort-of-string-buffer
import io
from contextlib import redirect_stdout

import string

import subprocess

import json
import struct

HOST = 'localhost'
#PORT = 50011

def receive_list(buffer):
    # determine how many bytes are in the prefix
    prefix_len = struct.calcsize("!i")

    # we really should be more careful about indexes here ;)
    data_len = struct.unpack("!i", buffer[:prefix_len])[0]
    data = buffer[prefix_len: data_len + prefix_len]

    l = json.loads(data)

    return l

def remove_non_ascii(a_str):
# https://bobbyhadz.com/blog/python-remove-non-ascii-characters-from-string
    ascii_chars = set(string.printable)

    return ''.join(
        filter(lambda x: x in ascii_chars, a_str)
    )

def is_socket_closed(sock: socket.socket) -> bool:
    try:
        # this will try to read bytes without blocking and also without removing them from buffer (peek only)
        data = sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
        if len(data) == 0:
            return True
    except BlockingIOError:
        return False  # socket is open and reading from it would block
    except ConnectionResetError:
        return True  # socket was closed for some other reason
    except Exception as e:
        #logger.exception("unexpected exception when checking if a socket is closed")
        return False
    return False

def main(PORT):

    # set up server
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # this will open the socket even if it open?
        serversocket.bind((HOST, PORT))
    except Exception as e: 
        print(e)
        exit(1)

    # Start server with maximum 100 connections
    serversocket.listen(100)

    print("listening on port: " + str(PORT))
    
    # start timer
    ping_time = 5 # seconds
    start_time = time.time();
    
    #
    #exception_count = 0
    
    # hang out and listen
    while True:
        #
        #serversocket.setblocking(0)
        connection, address = serversocket.accept()
        print(address)
        
        #
        while not is_socket_closed(connection):
           
            try:
                
                # heartbeat timer
                if (time.time() - start_time) > ping_time:
                    start_time = time.time()
                    #time_string = str(start_time)
                    #print(time_string)
                    #connection.send(time_string.encode())
                    #print("sent")
                
                buf = connection.recv(4096, 0x40) # 0x40 = MSG_DONTWAIT a.k.a. O_NONBLOCK
                #buf = connection.recv(4096)
                if len(buf) > 0:
                    #
                    command_rx = buf.decode()
                    
                    #
                    command_filtered = remove_non_ascii(command_rx)
                    
                    #
                    print("Command: " + command_filtered)
                    
                    # sudo python /var/opt/codesys/PlcLogic/Application/CaptureImage.py -s /var/opt/codesys/PlcLogic/Application/input.jpg -w 2016 -h 2000
                    # use runpy
                    try:
                        list = command_filtered.split(" ")
                        sys.argv = list[2:]
                        with io.StringIO() as buf, redirect_stdout(buf):
                            runpy.run_path(path_name=list[2], run_name="__main__")
                            output = buf.getvalue()
                        print(output)
                        connection.send(output.encode())
                    except Exception as e: 
                        print(e)
                    '''
                    # use subprocess (this is much slower as it has to load the modules on each "call')
                    try:
                        now = time.time()
                        process = subprocess.run(command_filtered, shell=True, capture_output=True, text=True)
                        after = time.time()
                        duration = after - now
                        print("subprocess took " + str(duration) + " seconds")
                        #print("stdout: " + process.stdout)
                        #print("returncode: " + process.returncode)
                        #print("stderr: " + process.stderr)
                        connection.send(process.stdout.encode())
                    except Exception as e: 
                        print(e)
                    '''
            except:
                pass
                
if __name__ == '__main__':

    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "p:", ["port ="])
    except:
        print('pass the arguments like -p <port>')

    # set defaults
    #port = 30000
    
    for o,v in opts:
        if o in ['-p','--port']:
            port = int(v)

    main(port)