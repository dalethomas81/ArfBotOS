import sys
import subprocess
  
if __name__ == '__main__':

    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    p = subprocess.Popen(['python', argv[0]] + sys.argv[2:], shell=False)
    print (p.pid)