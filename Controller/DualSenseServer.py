import time
import getopt
import sys
from pydualsense import *
import socket
import threading, queue
import signal

HOST = 'localhost'
PORT = 50013

print_lock = threading.Lock()
def print_with_lock(string):
    print_lock.acquire()
    try:
        print(string)
    finally:
        print_lock.release()

connection_lock = threading.Lock()
def send_client_with_lock(connection, data):
    connection_lock.acquire()
    try:
        connection.send(data, 0x40) # 0x40 = MSG_DONTWAIT a.k.a. O_NONBLOCK
    finally:
        connection_lock.release()

disconnect_controller = False
def handle_connection(connection):

    try:
    
        # in case we want the client to shut us down
        #my_thread_id = threading.get_ident()
        #print_with_lock("created connection thread with id: " + str(my_thread_id))
        #uid = "connection uid: " + str(my_thread_id)
        #send_client_with_lock(connection, uid.encode())
        
        #
        global disconnect_controller
        disconnect_controller = False
        
        #
        while not disconnect_controller:
            
            try:

                # test connection to client
                try:
                    buf = connection.recv(1024, 0x40) # 0x40 = MSG_DONTWAIT a.k.a. O_NONBLOCK
                    if len(buf) == 0:
                        #print("client disconnected")
                        break
                except socket.error as e:
                    pass

                # create dualsense
                dualsense = pydualsense()
                
                # find device and initialize
                dualsense.init()

                # set color around touchpad to red
                dualsense.light.setColorI(255, 0, 0)
                dualsense.setLeftMotor(100)
                dualsense.setRightMotor(100)
                time.sleep(0.250)
                dualsense.setLeftMotor(0)
                dualsense.setRightMotor(0)
                
                # hang out here
                while not disconnect_controller:
                    
                    #
                    time.sleep(0.1) # take a nap so you dont overload the cpu

                    try:
                        #
                        if not dualsense.connected:
                            break
                        
                        # test connection to client
                        try:
                            buf = connection.recv(1024, 0x40) # 0x40 = MSG_DONTWAIT
                            if len(buf) == 0:
                                break
                        
                        except socket.error as e:
                            pass
                        
                        #
                        send_client_with_lock(connection, bytearray(dualsense.states))
                        
                    except Exception as e:
                        #print(e)
                        continue # continue will keep running the while loop
                
                # set color around touchpad to red
                dualsense.light.setColorI(0, 255, 0)
                dualsense.setLeftMotor(100)
                dualsense.setRightMotor(100)
                time.sleep(0.250)
                dualsense.setLeftMotor(0)
                dualsense.setRightMotor(0)
                
                # close device
                dualsense.close()
                
            except Exception as e:
                #if e == 'No device detected':
                #print(e)
                pass
                
    except Exception as e:
        print(e)

def str_to_bool (val):
    """Convert a string representation of truth to true (1) or false (0).
    True values are 'y', 'yes', 't', 'true', 'on', and '1'; false values
    are 'n', 'no', 'f', 'false', 'off', and '0'.  Raises ValueError if
    'val' is anything else.
    """
    val = val.lower()
    if val in ('y', 'yes', 't', 'true', 'on', '1'):
        return 1
    elif val in ('n', 'no', 'f', 'false', 'off', '0'):
        return 0
    else:
        raise ValueError("invalid truth value %r" % (val,))

def runcleanup():
    global disconnect_controller
    global shutdown
    disconnect_controller = True
    shutdown = True
    sys.exit(1)

def cleanup(*args):
    runcleanup()

# https://docs.python.org/3/library/signal.html
signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

shutdown = False
def main():

    global shutdown

    try:
    
        try:
            # set up  and start server
            #print_with_lock("trying port: " + str(PORT))
            serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            serversocket.bind((HOST, PORT))
            serversocket.listen(5) # max number of connections
            #print_with_lock("listening on port: " + str(PORT))
            
        except Exception as e:
            print_with_lock("could not start server")
            print_with_lock(e)
            exit(1)    
            
        # hang out and listen for connections
        while not shutdown:
            
            #
            time.sleep(0.001)
            
            #
            try:
                connection, address = serversocket.accept()
            except Exception as e:
                continue
            
            # once a connection is made
            #print_with_lock('connection address:')
            #print_with_lock(address)
            connection_thread = threading.Thread(target = handle_connection, args = (connection,))
            connection_thread.name = 'connection thread'
            connection_thread.start()

    except Exception as e:
        print(e)

if __name__ == '__main__':

    try:

        # fetch all the arguments from sys.argv except the script name
        argv = sys.argv[1:]

        # get option and value pair from getopt
        try:
            opts,argv = getopt.getopt(argv, "1:2:3:", ["option1 =","option2 =","option3 ="])
        except:
            print('pass the arguments like -1 <option 1> -2 <option 2> -3 <option 3>')
            
        # set defaults
        option1 = '' # string
        option2 = 0 # integer
        option3 = False # boolean
        
        for o,v in opts:
            if o in ['-1','--option1']:
                option1 = v
            elif o in ['-2','--option2']:
                option2 = int(v)
            elif o in ['-3','--option3']:
                option3 = str_to_bool(v)
        
        # example to get a relative file
        #dir_path = os.path.dirname(os.path.realpath(__file__))
        #file_path = str(os.path.join(dir_path, 'test.txt'))
        
        main()

    except KeyboardInterrupt as e:
        # note this doesnt currently work as expected so forget it for now
        # was hoping to cleanly exit when testing this script from command line
        # now will need to pidof python then sudo kill -9 <pid number>
        runcleanup()