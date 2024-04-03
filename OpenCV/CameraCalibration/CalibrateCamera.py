# https://www.geeksforgeeks.org/camera-calibration-with-python-opencv/
# https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
# https://stackoverflow.com/questions/37310210/camera-calibration-with-opencv-how-to-adjust-chessboard-square-size
# Import required modules
import cv2
import numpy as np
import os
import getopt
import sys
import time
import datetime
import os
import math
#import multiprocessing

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
        
def save_cal_data(checkerboard, squaresize, matrix, distortion, pixelratio, rotation_offset, top_left, bot_right, origin, path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("checkerboard", checkerboard)
    cv_file.write("squaresize", squaresize)
    cv_file.write("matrix", matrix)
    cv_file.write("distortion", distortion)
    cv_file.write("pixelratio", pixelratio)
    cv_file.write("rotation_offset", rotation_offset)
    cv_file.write("top_left", top_left)
    cv_file.write("bot_right", bot_right)
    cv_file.write("origin", origin)
    cv_file.release()

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
    origin = cv_file.getNode("origin").mat()
    cv_file.release()
    
    return [checkerboard, squaresize, camera_matrix, dist_matrix, pixel_ratio, rotation_offset, top_left, bot_right, origin]
    
def load_roi_data(path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    top_left = cv_file.getNode("top_left").mat()
    bot_right = cv_file.getNode("bot_right").mat()
    cv_file.release()
    
    return [top_left, bot_right]


# get the average pixel count between corners
# use this to get the pixel per mm ratio
def get_pixel_average(corners, checkerboard):
    count = 0
    total = 0
    average = 0
    for x in range(0, len(corners)-1, checkerboard[0]):
        for x in range(0, checkerboard[0]-1, 1):
            val0 = corners[x][0][0]
            val1 = corners[x + 1][0][0]
            diff = val1 - val0
            total = total + diff
            count = count + 1
            #print("v0:" + str(val0) + " v1:" + str(val1) + " diff:" + str(diff))
    average = total / count
    return average
    
def get_index_value(array, x, y, column_count):
    # X + (Y*W)
    value = array[x + (y*column_count)][0]
    return value
    
def get_rotation_offset(corners, checkerboard):
    
    x1 = get_index_value(corners,0,0,checkerboard[0])
    x2 = get_index_value(corners,0,checkerboard[1]-1,checkerboard[0])
    rotation_offset = 0
    
    if x1[0] == x2[0]:
        rotation_offset = 0
    elif x1[0] > x2[0]:
        rotation_offset = math.tan((x1[0]-x2[0])/(x2[1]-x1[1]))
        rotation_offset = math.degrees(rotation_offset)
    elif x1[0] < x2[0]:
        rotation_offset = math.tan((x2[0]-x1[0])/(x2[1]-x1[1]))
        rotation_offset = math.degrees(rotation_offset)
        rotation_offset = -rotation_offset
        
    return rotation_offset
    
def get_top_left(corners, checkerboard):
    coordinates = get_index_value(corners,0,0,checkerboard[0])
    return coordinates

def get_bot_right(corners, checkerboard):
    coordinates = get_index_value(corners,checkerboard[0]-1,checkerboard[1]-1,checkerboard[0])
    return coordinates

def main(checkerboard, squaresize, resultfile, debug, width, height):
    # stop the iteration when specified
    # accuracy, epsilon, is reached or
    # specified number of iterations are completed.
    criteria = (cv2.TERM_CRITERIA_EPS +
                cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Vector for 3D points
    threedpoints = []

    # Vector for 2D points
    twodpoints = []

    # 3D points real world coordinates
    objectp3d = np.zeros((1, checkerboard[0]
                        * checkerboard[1],
                        3), np.float32)
    objectp3d[0, :, :2] = np.mgrid[0:checkerboard[0],
                                0:checkerboard[1]].T.reshape(-1, 2)
    objectp3d = objectp3d * squaresize
    prev_img_shape = None

    image = capture_image(debug, width, height)
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    # If desired number of corners are
    # found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(
                    grayColor, checkerboard,
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK +
                    cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If desired number of corners can be detected then,
    # refine the pixel coordinates and display
    # them on the images of checker board
    if ret == True:

        pixel_average = get_pixel_average(corners, checkerboard)
        rotation_offset = get_rotation_offset(corners, checkerboard)
        origin = get_top_left(corners, checkerboard)
        top_left = origin # leaving for backwards compatibility for now.
        bot_right = get_bot_right(corners, checkerboard)
        
        threedpoints.append(objectp3d)

        # Refining pixel coordinates
        # for given 2d points.
        corners2 = cv2.cornerSubPix(
            grayColor, corners, (11, 11), (-1, -1), criteria)

        twodpoints.append(corners2)
        
        # Draw and display the corners
        image = cv2.drawChessboardCorners(image, 
                                          checkerboard, 
                                          corners2, ret)

        # Perform camera calibration by
        # passing the value of above found out 3D points (threedpoints)
        # and its corresponding pixel coordinates of the
        # detected corners (twodpoints)
        reprojection_error, mtx, dist, r_vecs, t_vecs = cv2.calibrateCamera(
            threedpoints, twodpoints, grayColor.shape[::-1], None, None)

        # save so we can pull them in later from other scripts
        pixel_ratio = pixel_average / squaresize
        dir_path = os.path.dirname(os.path.realpath(__file__))
        cal_file_path = str(os.path.join(dir_path, 'cal.yaml'))
        save_cal_data(checkerboard, squaresize, mtx, dist, pixel_ratio, rotation_offset, top_left, bot_right, origin, cal_file_path)

        # filter image through new calibration data
        image = filter_image(image, mtx, dist, False)
        #image = rotate_image(image, rotation_offset)
        #image = crop_image(image, top_left, bot_right)
        
        # save the file so we can see if the squares were detected and see the results of the calibration
        cv2.imwrite(resultfile, image)
        
    else:
        pixel_ratio = -1
        rotation_offset = -1
        reprojection_error = -1
        
    return pixel_ratio, rotation_offset, reprojection_error

if __name__ == "__main__":
    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]
    
    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "r:l:x:y:d:w:h:", ["resultfile =","squaresize =","checkerboard_x =","checkerboard_y =","debug =","width =","height ="])
    except:
        print('pass the arguments like -r <result file> -l <squaresize> -x <checkerboard_x> -y <max checkerboard_y> -d <debug> -w <width> -h <height>')
    
    # set defaults
    resultfile = 'result.jpg'
    squaresize = 25 # mm
    checkerboard_x = 6
    checkerboard_y = 9
    debug = False
    width = 3280
    height = 2464
    
    # parse arguments
    for o,v in opts:
        if o in ['-r','--resultfile']:
            resultfile = v
        elif o in ['-l','--squaresize']:
            squaresize = float(v)
        elif o in ['-x','--checkerboard_x']:
            checkerboard_x = int(v)
        elif o in ['-y','--checkerboard_y']:
            checkerboard_y = int(v)
        elif o in ['-d','--debug']:
            debug = str_to_bool(v)
        elif o in ['-w','--width']:
            width = int(v)
        elif o in ['-h','--height']:
            height = int(v)
    
    checkerboard = (checkerboard_x, checkerboard_y)    
    
    # load the calibration data
    #dir_path = os.path.dirname(os.path.realpath(__file__))
    #cal_file_path = str(os.path.join(dir_path, 'cal.yaml'))
    #checkerboard, squaresize, mtx, dist, pixel_ratio, rotation_offset, top_left, bot_right, origin = load_cal_data(cal_file_path)
    
    # load the roi data
    #dir_path = os.path.dirname(os.path.realpath(__file__))
    #roi_file_path = str(os.path.join(dir_path, 'roi.yaml'))
    #roi_top_left, roi_bot_right = load_roi_data(roi_file_path)
    
    # run main calibration program
    pixel_ratio, rotation_offset, reprojection_error = main(checkerboard, squaresize, resultfile, debug, width, height)
        
    # print cal data
    precision = 2
    cal_data = "CAL "
    cal_data = cal_data + "rat:" + str(round(pixel_ratio,precision)) + " "
    cal_data = cal_data + "rot:" + str(round(rotation_offset,precision)) + " "
    cal_data = cal_data + "err:" + str(round(reprojection_error,precision)) + " "
    print(cal_data)