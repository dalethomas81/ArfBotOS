import cv2
import datetime
import time
import numpy
from multiprocessing import shared_memory

import getopt
import sys

def detect(frame, file_name):
	#Convert source image to hsv image
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	#Define the range of color blue
	#lower_boarder = numpy.array([110, 50, 50])
	#upper_boarder = numpy.array([130, 255, 255])
	#Define the range of color red
	lower_boarder = numpy.array([170, 100, 100])
	upper_boarder = numpy.array([180, 255, 255])

	#Show the blue values in white, all others black
	mask = cv2.inRange(hsv, lower_boarder, upper_boarder)

	#Get the treshold and contours of the blue elements
	ret, threshed_img = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)
	contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	#Bounding rect around every blue element and get coordinates
	s=''
	for c in contours:
		(x,y,w,h) = cv2.boundingRect(c)
		pt = (x, y+h)
		cv2.rectangle(frame, (x, y), (x+w, y+h), (255,0,0), 2) #bgr
		s += str(pt) + ' '
		#cv2.imwrite(file_name,frame)
		#result string in PLC is 255
		if len(s) > 200:
			break
	cv2.rectangle(frame, (100, 100), (2000, 2000), (0,0,255), 3)
	cv2.imwrite(file_name,frame)

	return s

if __name__ == '__main__':

	#lets fetch all the arguments from sys.argv except the script name
	argv = sys.argv[1:]

	#get option and value pair from getopt
	try:
		opts,argv = getopt.getopt(argv, "f:t:", ["filename =","test ="])
	except:
		print('pass the arguments like -f <file name> -t <test> or --filename <file name> and --test <test>')

	for o,v in opts:
		if o in ['-f','--filename']:
			file_name = v
		elif o in ['-t','--test']:
			test = v
	#print(file_name + ' ' + test)

	#Get the image of an file
	frame = cv2.imread(file_name)

	#cv2.rectangle(frame, (100, 100), (200, 200), (255,0,0), 2)
	#cv2.imwrite(file_name,frame)

	#Add a time stamp to the string
	s = detect(frame,file_name) + str(datetime.datetime.now())
	print(s)
