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

def rotate_image(image, angle):
    image_center = tuple(numpy.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result
    
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
        time.sleep(0.1)
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

def FindTemplate(image, SaveLocation, TemplateLocation, MinThreshold, MaxThreshold):

    # save image to disk
    #cv2.imwrite(SaveLocation, image)
    
    # convert image to grey
    image_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # get template image
    #methods = ['cv2.IMREAD_COLOR', 'cv2.IMREAD_GRAYSCALE', 'cv2.IMREAD_UNCHANGED']
    template = cv2.imread(TemplateLocation, cv2.IMREAD_GRAYSCALE)

    # get the shape of the template for when we draw the rectangle
    w, h = template.shape[1::-1]

    # match the template
    #methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]
    #methods = [cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF_NORMED]
    methods = [cv2.TM_CCOEFF, cv2.TM_CCORR, cv2.TM_SQDIFF]
    #methods = [cv2.TM_CCOEFF]
    high_val = 0
    high_val_last = 0
    for j in range(len(methods)):
        result = cv2.matchTemplate(image_grey, template, methods[j])
        cv2.normalize( result, result, 0, 1, cv2.NORM_MINMAX, -1 )
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if methods[j] in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            high_val = min_val
        else:
            high_val = max_val
        if (high_val > high_val_last) or (methods[j] == methods[0]):
            high_val_last = high_val
            high_val_meth = methods[j]
            min_loc_last = min_loc
            max_loc_last = max_loc

    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    if high_val_meth in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        top_left = min_loc_last
    else:
        top_left = max_loc_last
    bottom_right = (top_left[0] + w, top_left[1] + h)

    # draw rectangle around found template
    cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 6) #colour of rectangle(b,g,r)

    # draw crosshair
    crosshair_length = 20
    pt1 = (int(top_left[0] + ((bottom_right[0] - top_left[0]) / 2)), int(top_left[1] + ((bottom_right[1] - top_left[1]) / 2) - crosshair_length))
    pt2 = (int(top_left[0] + ((bottom_right[0] - top_left[0]) / 2)), int(top_left[1] + ((bottom_right[1] - top_left[1]) / 2) + crosshair_length))
    cv2.line(image, pt1, pt2, (0, 255, 0), 3, cv2.LINE_AA)

    pt1 = (int(top_left[0] + ((bottom_right[0] - top_left[0]) / 2) - crosshair_length), int(top_left[1] + (bottom_right[1] - top_left[1]) / 2))
    pt2 = (int(top_left[0] + ((bottom_right[0] - top_left[0]) / 2) + crosshair_length), int(top_left[1] + (bottom_right[1] - top_left[1]) / 2))
    cv2.line(image, pt1, pt2, (0, 255, 0), 3, cv2.LINE_AA)


    # find edges
    #dst = cv2.Canny(image_grey, 50, 200, None, 3)

    # Copy edges to the images that will display the results in BGR
    #cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    #cdstP = numpy.copy(cdst)

    #lines = cv2.HoughLines(dst, 1, numpy.pi / 180, 150, None, 0, 0)

    #if lines is not None:
    #    for i in range(0, len(lines)):
    #        rho = lines[i][0][0]
    #        theta = lines[i][0][1]
    #        a = math.cos(theta)
    #        b = math.sin(theta)
    #        x0 = a * rho
    #        y0 = b * rho
    #        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    #        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    #        cv2.line(image, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)


    # save image to disk
    cv2.imwrite(SaveLocation, image)

    return top_left

if __name__ == '__main__':

    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "s:t:i:j:d:w:h:", ["savelocation =","templatelocation =","minthreshold =","maxthreshold =","debug =","width =","height ="])
    except:
        print('pass the arguments like -s <save location> -t <template location> -i <min threshold> -j <max threshold> -d <debug> -w <width> -h <height>')

    # set defaults
    savelocation = 'image.jpg'
    templatelocation = 'template.jpg'
    minthreshold = 0.9
    maxthreshold = 1.0
    width = 3280
    height = 2464
    debug = False
    
    for o,v in opts:
        if o in ['-s','--savelocation']:
            savelocation = v
        elif o in ['-t','--templatelocation']:
            templatelocation = v
        elif o in ['-i','--minthreshold']:
            minthreshold = float(v)
        elif o in ['-j','--maxthreshold']:
            maxthreshold = float(v)
        elif o in ['-d','--debug']:
            debug = str_to_bool(v)
        elif o in ['-w','--width']:
            width = int(v)
        elif o in ['-h','--height']:
            height = int(v)
    
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
    s = FindTemplate(image, savelocation, templatelocation, minthreshold, maxthreshold)
    
    #
    print(s)
    
    #print(str(datetime.datetime.now()))