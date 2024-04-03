'''
    this is just a test client for running locally and debugging the dualsenseserver.py
'''

import socket

PORT = 50013

def main():
    try:
        clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        clientsocket.connect(('localhost', PORT))
        while True:
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