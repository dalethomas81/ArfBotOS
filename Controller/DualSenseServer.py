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

 # 'setColorI:0,0,255' 0-255,0-255,0-255 r,g,b
def handle_setcolor_cmd(cmdParameters ,dualsense):
    '''
    dualsense.light.setColorI(0, 0, 255) 'cmd: setColorI:0,0,255'
    '''
    print('command currently not supported')
    
 # 'setMicrophoneState:True' True/False
def handle_setmicrophonestate_cmd(cmdParameters ,dualsense):
    '''
    dualsense.audio.setMicrophoneState(True)
    '''
    print('command currently not supported')
    
 # 'setPlayerID:4' 4/10/21/27/31 1/2/3/4/all
def handle_setplayerid_cmd(cmdParameters ,dualsense):
    '''
    dualsense.light.setPlayerID(PlayerID.PLAYER_2)
    class PlayerID(IntFlag):
        PLAYER_1 = 4
        PLAYER_2 = 10
        PLAYER_3 = 21
        PLAYER_4 = 27
        ALL = 31
    '''
    print('command currently not supported')
    
 # 'setBrightness:2' 0/1/2 low/med/high
def handle_setbrightness_cmd(cmdParameters ,dualsense):
    '''        
        dualsense.light.setBrightness(Brightness.high)
        class Brightness(IntFlag):
            high = 0x0
            medium = 0x1
            low = 0x2
    '''
    print('command currently not supported')
    
 # 'setRightMotor:255' 0-255 intensity
def handle_setrightmotor_cmd(cmdParameters ,dualsense):
    '''
    '''
    '''
    dualsense.setLeftMotor(255)
    dualsense.setRightMotor(100)
    '''
    print('command currently not supported')
    
 # 'setLeftMotor:255' 0-255 intensity
def handle_setleftmotor_cmd(cmdParameters ,dualsense):
    '''
    '''
    print('command currently not supported')
    
 # 'setRightTriggerMode:0' see notes in function
def handle_setrighttriggermode_cmd(cmdParameters ,dualsense):
    '''
    '''
    print('command currently not supported')
    
 # 'setLeftTriggerMode:0' see notes in function
def handle_setlefttriggermode_cmd(cmdParameters ,dualsense):
    '''
    dualsense.triggerR.setMode(TriggerModes.Pulse_A)
    class TriggerModes(IntFlag):
        Off = 0x0  # no resistance
        Rigid = 0x1  # continous resistance
        Pulse = 0x2  # section resistance
        Rigid_A = 0x1 | 0x20
        Rigid_B = 0x1 | 0x04
        Rigid_AB = 0x1 | 0x20 | 0x04
        Pulse_A = 0x2 | 0x20
        Pulse_B = 0x2 | 0x04
        Pulse_AB = 0x2 | 0x20 | 0x04
        Calibration = 0xFC
    '''
    print('command currently not supported')
    
 # 'setRightTriggerForce:0,255' see notes in function
def handle_setrighttriggerforce_cmd(cmdParameters ,dualsense):
    '''
    dualsense.triggerR.setForce(0, 200)
    dualsense.triggerR.setForce(1, 255)
    dualsense.triggerR.setForce(2, 175)
    '''
    print('command currently not supported')
    
 # 'setLeftTriggerForce:0,255' see notes in function
def handle_setlefttriggerforce_cmd(cmdParameters ,dualsense):
    '''
    '''
    print('command currently not supported')

def handle_client_message(buf, dualsense):
    
    try:
        message = buf.decode()
        if message[0:5] == 'cmd: ': # [0:5] means start at 0 index and return 5
            cmd = message[5:] # [5:] means start at index 5 and return the rest
            colon = cmd.find(':')
            cmdType = cmd[0:colon]
            cmdParameters = cmd[colon+1:]
            
            if cmdType == 'setColorI':
                handle_setcolor_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setMicrophoneState':
                handle_setmicrophonestate_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setPlayerID':
                handle_setplayerid_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setBrightness':
                handle_setbrightness_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setRightMotor':
                handle_setrightmotor_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setLeftMotor':
                handle_setleftmotor_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setRightTriggerMode':
                handle_setrighttriggermode_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setLeftTriggerMode':
                handle_setlefttriggermode_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setRightTriggerForce':
                handle_setrighttriggerforce_cmd(cmdParameters ,dualsense)
                
            elif cmdType == 'setLeftTriggerForce':
                handle_setlefttriggerforce_cmd(cmdParameters ,dualsense)
                
            else:
                print('unknown command: ' + cmdType)
                
    except Exception as e:
        print(e)

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
                                
                            else:
                                handle_client_message(buf, dualsense)
                        
                        except socket.error as e:
                            pass
                        
                        #
                        send_client_with_lock(connection, bytearray(dualsense.states))
                        
                    except Exception as e:
                        #print(e)
                        continue # continue will keep running the while loop
                
                # set color around touchpad to green
                dualsense.light.setColorI(0, 255, 0)
                dualsense.setLeftMotor(100)
                dualsense.setRightMotor(100)
                time.sleep(0.250)
                dualsense.setLeftMotor(0)
                dualsense.setRightMotor(0)
                time.sleep(0.250)
                
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