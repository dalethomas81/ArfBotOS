# https://stackoverflow.com/questions/72483906/sending-and-receiving-lists-in-socket
# https://stackoverflow.com/questions/16373510/improving-speed-of-python-module-import
# https://stackoverflow.com/questions/68345812/is-there-a-way-to-pass-flags-to-runpy

import socket
import runpy
import threading, queue
import cv2
import datetime
import time
import numpy
import getopt
import sys
import math
import os
import io
from contextlib import redirect_stdout
import string
import subprocess
import json
import struct
import multiprocessing

HOST = '' # leave this blank so that we listen on all interfaces. this is needed if you are hosting on a separate machine.
PORT = 50011

print_lock = threading.Lock()
def print_with_lock(string):
    print_lock.acquire()
    print(string)
    print_lock.release()

queue_lock = threading.Lock()
def peek_queue(queue):
    queue_lock.acquire()
    value = queue.get()
    queue.put(value)
    queue_lock.release()
    return value
    
connection_lock = threading.Lock()
def send_client_with_lock(connection, data):
    connection_lock.acquire()
    connection.send(data)
    connection_lock.release()

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

def handle_command(buf, connection):
    
    my_thread_id = threading.get_ident()
    print_with_lock("created command thread with id: " + str(my_thread_id))
    uid = "command uid: " + str(my_thread_id)
    send_client_with_lock(connection, uid.encode())
    
    #
    command_rx = buf.decode()
    
    #
    command_filtered = remove_non_ascii(command_rx)
    
    #
    print_with_lock("command: " + command_filtered)
    
    # use runpy
    try:
        list = command_filtered.split(" ")
        sys.argv = list[2:]
        with io.StringIO() as buf, redirect_stdout(buf):
            runpy.run_path(path_name=list[2], run_name="__main__")
            output = buf.getvalue()
        print(output)
        result = "res: " + output
        send_client_with_lock(connection, result.encode())
    except Exception as e:
        print_with_lock(e)
    '''
    # use subprocess (this is much slower as it has to load the modules on each "call')
    try:
        now = time.time()
        process = subprocess.run(command_filtered, shell=True, capture_output=True, text=True)
        after = time.time()
        duration = after - now
        print_with_lock("subprocess took " + str(duration) + " seconds")
        #print_with_lock("stdout: " + process.stdout)
        #print_with_lock("returncode: " + process.returncode)
        #print_with_lock("stderr: " + process.stderr)
        send_client_with_lock(connection, process.stdout.encode())
    except Exception as e: 
        print_with_lock(e)
    '''
    #print_with_lock("exiting command thread with id: " + str(my_thread_id))
            
thread_id_to_kill = queue.Queue()
def handle_worker(buf, connection):
    #
    my_thread_id = threading.get_ident()
    print_with_lock("created worker thread with id: " + str(my_thread_id))
    uid = "worker uid: " + str(my_thread_id)
    send_client_with_lock(connection, uid.encode())
    
    #
    #command_thread = multiprocessing.Process(target = handle_command, args = (buf, connection))
    #command_thread.start()
    command_thread = threading.Thread(target = handle_command, args = (buf, connection))
    command_thread.start()
    
    #
    while True:
        time.sleep(0.1)
        id = peek_queue(thread_id_to_kill)
        #print_with_lock("thread to kill: " + str(id))
        #print_with_lock("my thread id: " + str(my_thread_id))
        if (my_thread_id == id) or not command_thread.is_alive():
            if (my_thread_id == id):
                thread_id_to_kill.put(0)
                #command_thread.terminate()
                #command_thread.kill()
                #command_thread.join()
            #command_thread.close()
            print_with_lock("exiting worker thread with id: " + str(my_thread_id))
            break
    
    
def handle_connection(connection):
    
    my_thread_id = threading.get_ident()
    print_with_lock("created connection thread with id: " + str(my_thread_id))
    uid = "connection uid: " + str(my_thread_id)
    send_client_with_lock(connection, uid.encode())
    
    while True:
    
        time.sleep(0.001)
        
        try:
            
            buf = connection.recv(4096, 0x40) # 0x40 = MSG_DONTWAIT a.k.a. O_NONBLOCK
            #buf = connection.recv(4096)
            if len(buf) > 0:
                rx = buf.decode()
                if rx[0:6] == 'kill: ': # kill: 1234567890
                    id = int(rx[6:])
                    thread_id_to_kill.put(id)
                    print_with_lock("received kill command for thread with id: " + str(id))
                    result = "res: killing thread with id: " + str(id)
                    send_client_with_lock(connection, result.encode())
                    
                else:
                    #worker_thread = threading.Thread(target = handle_worker, args = (buf, connection))
                    #worker_thread.start()
                    command_thread = threading.Thread(target = handle_command, args = (buf, connection))
                    command_thread.start()
                    
        except ValueError:
            result = "no uid provided for kill command"
            send_client_with_lock(connection, result.encode())
            
        except socket.error as e:
            pass
            
        except:
            pass

def main():
    
    try:
        #
        thread_id_to_kill.put(0)
        
        # set up  and start server
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # this will open the socket even if it open?
        serversocket.bind((HOST, PORT))
        serversocket.listen(5) # max number of connections
        print_with_lock("listening on port: " + str(PORT))
        
    except:
        print_with_lock("could not start server")
        exit(1)
 
    # hang out and listen for connections
    while True:
    
        time.sleep(0.001)
        
        try:
            connection, address = serversocket.accept()
        except Exception as e:
            continue
            
        print_with_lock('connection address:')
        print_with_lock(address)
        
        #connection.settimeout(2)
        connection_thread = threading.Thread(target = handle_connection, args = (connection,))
        connection_thread.start()

if __name__ == '__main__':

    main()