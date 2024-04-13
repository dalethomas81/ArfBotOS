'''
    this is just a test client for running locally and debugging
'''

# https://stackoverflow.com/questions/70058132/how-do-i-make-a-timer-in-python

import socket
import time
import signal

PORT = 50011

def runcleanup():
    global shutdown
    shutdown = True
    sys.exit(1)

def cleanup(*args):
    runcleanup()

# https://docs.python.org/3/library/signal.html
signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

shutdown = False
def main():
    try:
        global shutdown
        
        #
        clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        clientsocket.connect(('192.168.1.73', PORT))
        
        #data = 'sudo python /var/opt/codesys/PlcLogic/Application/Vision/CaptureImage.py -s /var/opt/codesys/PlcLogic/Application/Vision/CapturedImage.jpg -w 640 -h 400 2> ./vision_log.txt'
        
        data = 'sudo python /var/opt/codesys/PlcLogic/Application/Vision/FastTemplateMatching.py -s /var/opt/codesys/PlcLogic/Application/Vision/outputimage.bmp -t /var/opt/codesys/PlcLogic/Application/Vision/Templates/battery.jpg -i 5 -j 0.0 -k 0.8 -l 90.0 -d True'
        
        clientsocket.send(data.encode())
        
        #
        start = time.time()
        
        while not shutdown:
        
            # send a cmd every 5 seconds
            #now = time.time()
            #if now - start > 5:
            #    start = time.time()
            #    data = ''
            #    clientsocket.send(data.encode())
            
            # continuously read the buffer
            buf = clientsocket.recv(4096)
            if len(buf) > 0:
                stringdata = buf.decode('utf-8')
                print (stringdata)

    except Exception as e:
        print("main thread")
        print(e)
        
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()