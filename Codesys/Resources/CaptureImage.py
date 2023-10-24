import cv2
import datetime
import time
import numpy
import getopt
import sys
import math
import os

def load_cal_data(path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    checkerboard = cv_file.getNode("checkerboard").mat()
    squaresize = cv_file.getNode("squaresize").real()
    camera_matrix = cv_file.getNode("matrix").mat()
    dist_matrix = cv_file.getNode("distortion").mat()
    pixel_ratio = cv_file.getNode("pixelratio").real()
    rotation_offset = cv_file.getNode("rotation_offset").real()
    top_left = cv_file.getNode("top_left").mat()
    bot_right = cv_file.getNode("bot_right").mat()
    cv_file.release()
    
    return [checkerboard, squaresize, camera_matrix, dist_matrix, pixel_ratio, rotation_offset, top_left, bot_right]

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
    
def filter_image(image, mtx, dist, crop=False):
    
    # get the size of the image
    h,  w = image.shape[:2]
    clone = image.copy()

    # apply the calibration data
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
    dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
    
    if crop == True:
        x,y,w,h = roi
        image = dst[y:y+h, x:x+w]
    
    return image
    
def rotate_image(image, angle):
    h,  w = image.shape[:2]
    cX, cY = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D((cX, cY), angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))

    return rotated

def crop_image(image, top_left, bot_right):
    y = int(top_left[1])
    x = int(top_left[0])
    h = int(bot_right[1] - top_left[1])
    w = int(bot_right[0] - top_left[0])
    cropped = image[y:y+h, x:x+w].copy() # image slicing creates a pointer. So use copy()
    return cropped
    
def capture_image(debug, width, height):
    if debug:
        image = cv2.imread("Input.jpg") # temporary while we run on windows to develop

    else:
        #import libcamera
        from picamera2 import Picamera2
        camera = Picamera2()
        config = camera.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        #config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        camera.configure(config)
        camera.start()
        #time.sleep(0.1)
        image = camera.capture_array()
        camera.stop()
        camera.close()
        
        
        '''
        from picamera.array import PiRGBArray
        from picamera import PiCamera
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        rawCapture = PiRGBArray(camera)

        # allow the camera to warmup
        time.sleep(0.1)

        # grab an image from the camera
        camera.resolution = (width, height) #max is(3280, 2464)
        #camera.annotate_text = "ArfBot Vision v1.0"
        camera.capture(rawCapture, format="bgr")#, use_video_port=True)
        image = rawCapture.array
        
        # release resources
        camera.close()
        '''

    return image

if __name__ == '__main__':

    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "s:w:h:d:", ["savelocation =","width =","height =","debug ="])
    except:
        print('pass the arguments like -s <save location> -w <width> -h <height> -d <debug>')
        
    # set defaults
    savelocation = 'image.jpg'
    width = 3280
    height = 2464
    debug = False
    
    for o,v in opts:
        if o in ['-s','--savelocation']:
            savelocation = v
        elif o in ['-w','--width']:
            width = int(v)
        elif o in ['-h','--height']:
            height = int(v)
        elif o in ['-d','--debug']:
            debug = str_to_bool(v)
    
    # load the calibration data
    dir_path = os.path.dirname(os.path.realpath(__file__))
    cal_file_path = str(os.path.join(dir_path, 'cal.yaml'))
    checkerboard, squaresize, mtx, dist, pixel_ratio, rotation_offset, top_left, bot_right = load_cal_data(cal_file_path)
    
    # capture the image
    image = capture_image(debug, width, height)
    
    # filter image through calibration data
    if debug == False:
        image = filter_image(image, mtx, dist, False)
        image = rotate_image(image, rotation_offset)
        image = crop_image(image, top_left, bot_right)
    
    #
    cv2.imwrite(savelocation, image)
    
    #
    print("CAP t:" + str(datetime.datetime.now()) + " ")