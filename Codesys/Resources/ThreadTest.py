import threading, queue

print_lock = threading.Lock()

def print_with_lock(s):
    print_lock.acquire()
    print(s)
    print_lock.release()

def handle_command(buf, connection):
    
    my_thread_id = threading.get_ident()
    print_with_lock("created command thread with id: " + str(my_thread_id))
    uid = "command uid: " + str(my_thread_id)

    print_with_lock(e)

            
thread_id_to_kill = queue.Queue()
def handle_worker(buf, connection):
    #
    my_thread_id = threading.get_ident()
    print_with_lock("created worker thread with id: " + str(my_thread_id))
    uid = "worker uid: " + str(my_thread_id)
    
    #
    command_thread = threading.Thread(target = handle_command, args = (buf, connection))
    command_thread.start()
    # CAUTION - YOU HAVE ALREADY PASSED THE CONNECTION TO THE TARGET
    # IF YOU USE IT NOW KEEP IN MIND MESSAGES WILL BE OUT OF ORDER
    
    #
    while (1):
        time.sleep(0.01)
        print_with_lock("thread to kill: " + str(thread_id_to_kill.get()))
        print_with_lock("my thread id: " + str(my_thread_id))
        if (my_thread_id == thread_id_to_kill.get()):# or not command_thread.is_alive():
            thread_id_to_kill.put(0)
            print_with_lock("exiting worker thread with id: " + str(my_thread_id))
            break
    
    
def handle_connection(connection):
    
    my_thread_id = threading.get_ident()
    print_with_lock("created connection thread with id: " + str(my_thread_id))
    uid = "connection uid: " + str(my_thread_id)
    
    worker_thread = threading.Thread(target = handle_worker, args = (buf, connection))
    worker_thread.start()
    
    while True:
    
        time.sleep(0.001)

def main():
    
    thread_id_to_kill.put(0)
    
    connection_thread = threading.Thread(target = handle_connection, args = (connection,))
    connection_thread.start()
 
    # hang out and listen for connections
    while True:
    
        time.sleep(0.001)

if __name__ == '__main__':

    main()