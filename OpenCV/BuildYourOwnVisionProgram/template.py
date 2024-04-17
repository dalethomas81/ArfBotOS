import datetime
import time
import numpy
import getopt
import sys
import math
import os

if __name__ == '__main__':

    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "1:2:3:", ["option1 =","option2 =","option3 ="])
    except:
        print('pass the arguments like -1 <option 1> -2 <option 2> -3 <option 3>')
        
    # set defaults
    option1 = ''
    option2 = 0
    option3 = False
    
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
    
    #
    print("res:" + str(datetime.datetime.now()) + " " + "some response data")