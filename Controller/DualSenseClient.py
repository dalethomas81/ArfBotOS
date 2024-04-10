'''
    this is just a test client for running locally and debugging the dualsenseserver.py
'''

# https://stackoverflow.com/questions/70058132/how-do-i-make-a-timer-in-python

import socket
import time

PORT = 50013

def main():
    try:
        #
        clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        clientsocket.connect(('localhost', PORT))
        
        #
        start = time.time()
        
        while True:
        
            # send a cmd every 5 seconds
            now = time.time()
            if now - start > 5:
                start = time.time()
                data = 'cmd: setColorI:0,0,255'
                clientsocket.send(data.encode())
            
            # continuously read the buffer
            buf = clientsocket.recv(4096)
            if len(buf) > 0:
                #stringdata = buf.decode('utf-8')
                #print (stringdata)
                for x in buf:
                    print(buf.hex())

    except Exception as e:
        print("main thread")
        print(e)

if __name__ == '__main__':
    main()