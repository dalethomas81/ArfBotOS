import cv2
import numpy as np
import math
import datetime
import time
import getopt
import sys
import os

box_points = []
button_down = False

SQUARESIZE = 24.5

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

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, -angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

def scale_image(image, percent, maxwh):
    max_width = maxwh[1]
    max_height = maxwh[0]
    max_percent_width = max_width / image.shape[1] * 100
    max_percent_height = max_height / image.shape[0] * 100
    max_percent = 0
    if max_percent_width < max_percent_height:
        max_percent = max_percent_width
    else:
        max_percent = max_percent_height
    if percent > max_percent:
        percent = max_percent
    width = int(image.shape[1] * percent / 100)
    height = int(image.shape[0] * percent / 100)
    result = cv2.resize(image, (width, height), interpolation = cv2.INTER_AREA)
    return result, percent

def invariantMatchTemplate(rgbimage, rgbtemplate, method, matched_thresh, rgbdiff_thresh, rot_range, rot_interval, scale_range, scale_interval, rm_redundant, minmax, count):
    """
    rgbimage: RGB image where the search is running.
    rgbtemplate: RGB searched template. It must be not greater than the source image and have the same data type.
    method: [String] Parameter specifying the comparison method
    matched_thresh: [Float] Setting threshold of matched results(0~1).
    rgbdiff_thresh: [Float] Setting threshold of average RGB difference between template and source image.
    rot_range: [Integer] Array of range of rotation angle in degrees. Example: [0,360]
    rot_interval: [Integer] Interval of traversing the range of rotation angle in degrees.
    scale_range: [Integer] Array of range of scaling in percentage. Example: [50,200]
    scale_interval: [Integer] Interval of traversing the range of scaling in percentage.
    rm_redundant: [Boolean] Option for removing redundant matched results based on the width and height of the template.
    minmax:[Boolean] Option for finding points with minimum/maximum value.

    Returns: List of satisfied matched points in format [[point.x, point.y], angle, scale].
    """
    
    #
    current_count = 0
    count_satisfied = False
    img_gray = cv2.cvtColor(rgbimage, cv2.COLOR_RGB2GRAY)
    template_gray = cv2.cvtColor(rgbtemplate, cv2.COLOR_RGB2GRAY)
    image_maxwh = img_gray.shape
    height, width = template_gray.shape
    all_points = []
    
    if minmax == False:
    
        # will return matches found that meet the threshold setting
        for next_angle in range(rot_range[0], rot_range[1], rot_interval):
        
            for next_scale in range(scale_range[0], scale_range[1], scale_interval):
            
                #
                scaled_template_gray, actual_scale = scale_image(template_gray, next_scale, image_maxwh)
                
                #
                if next_angle == 0:
                    rotated_template = scaled_template_gray
                else:
                    rotated_template = rotate_image(scaled_template_gray, next_angle)
                
                #
                if method == "TM_CCOEFF":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCOEFF)
                    satisfied_points = np.where(matched_points >= matched_thresh)
                    
                elif method == "TM_CCOEFF_NORMED":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCOEFF_NORMED)
                    satisfied_points = np.where(matched_points >= matched_thresh)
                    
                elif method == "TM_CCORR":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCORR)
                    satisfied_points = np.where(matched_points >= matched_thresh)
                    
                elif method == "TM_CCORR_NORMED":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCORR_NORMED)
                    satisfied_points = np.where(matched_points >= matched_thresh)
                    
                elif method == "TM_SQDIFF":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_SQDIFF)
                    satisfied_points = np.where(matched_points <= matched_thresh)
                    
                elif method == "TM_SQDIFF_NORMED":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_SQDIFF_NORMED)
                    satisfied_points = np.where(matched_points <= matched_thresh)
                    
                else:
                    raise MethodError("There's no such comparison method for template matching.")
                
                #
                for pt in zip(*satisfied_points[::-1]):
                    all_points.append([pt, next_angle, actual_scale])
                    current_count = current_count + 1
                    if current_count > count and count > 0:
                        count_satisfied = True;
                        break
                    
                if count_satisfied == True and count > 0:
                    break
            
            if count_satisfied == True and count > 0:
                break
    else:
    
        # will return matches filtered through cv2.minMaxLoc()
        for next_angle in range(rot_range[0], rot_range[1], rot_interval):
        
            for next_scale in range(scale_range[0], scale_range[1], scale_interval):
                
                #
                scaled_template_gray, actual_scale = scale_image(template_gray, next_scale, image_maxwh)
                
                #
                if next_angle == 0:
                    rotated_template = scaled_template_gray
                else:
                    rotated_template = rotate_image(scaled_template_gray, next_angle)
                
                #
                if method == "TM_CCOEFF":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCOEFF)
                    #cv2.normalize( matched_points, matched_points, 0, 1, cv2.NORM_MINMAX, -1 )
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matched_points)
                    if max_val >= matched_thresh:
                        all_points.append([max_loc, next_angle, actual_scale, max_val])
                        current_count = current_count + 1
                        if current_count > count:
                            count_satisfied = True;
                        
                elif method == "TM_CCOEFF_NORMED":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCOEFF_NORMED)
                    #cv2.normalize( matched_points, matched_points, 0, 1, cv2.NORM_MINMAX, -1 )
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matched_points)
                    if max_val >= matched_thresh:
                        all_points.append([max_loc, next_angle, actual_scale, max_val])
                        current_count = current_count + 1
                        if current_count > count:
                            count_satisfied = True;
                        
                elif method == "TM_CCORR":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCORR)
                    #cv2.normalize( matched_points, matched_points, 0, 1, cv2.NORM_MINMAX, -1 )
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matched_points)
                    if max_val >= matched_thresh:
                        all_points.append([max_loc, next_angle, actual_scale, max_val])
                        current_count = current_count + 1
                        if current_count > count:
                            count_satisfied = True;
                        
                elif method == "TM_CCORR_NORMED":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_CCORR_NORMED)
                    #cv2.normalize( matched_points, matched_points, 0, 1, cv2.NORM_MINMAX, -1 )
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matched_points)
                    if max_val >= matched_thresh:
                        all_points.append([max_loc, next_angle, actual_scale, max_val])
                        current_count = current_count + 1
                        if current_count > count:
                            count_satisfied = True;
                        
                elif method == "TM_SQDIFF":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_SQDIFF)
                    #cv2.normalize( matched_points, matched_points, 0, 1, cv2.NORM_MINMAX, -1 )
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matched_points)
                    if min_val <= matched_thresh:
                        all_points.append([min_loc, next_angle, actual_scale, min_val])
                        current_count = current_count + 1
                        if current_count > count:
                            count_satisfied = True;
                        
                elif method == "TM_SQDIFF_NORMED":
                    matched_points = cv2.matchTemplate(img_gray,rotated_template,cv2.TM_SQDIFF_NORMED)
                    #cv2.normalize( matched_points, matched_points, 0, 1, cv2.NORM_MINMAX, -1 )
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matched_points)
                    if min_val <= matched_thresh:
                        all_points.append([min_loc, next_angle, actual_scale, min_val])
                        current_count = current_count + 1
                        if current_count > count:
                            count_satisfied = True;
                        
                else:
                    raise MethodError("There's no such comparison method for template matching.")
                    
                if count_satisfied == True and count > 0:
                    break
            
            if count_satisfied == True and count > 0:
                break
        
        #
        if method == "TM_CCOEFF":
            all_points = sorted(all_points, key=lambda x: -x[3])
            
        elif method == "TM_CCOEFF_NORMED":
            all_points = sorted(all_points, key=lambda x: -x[3])
            
        elif method == "TM_CCORR":
            all_points = sorted(all_points, key=lambda x: -x[3])
            
        elif method == "TM_CCORR_NORMED":
            all_points = sorted(all_points, key=lambda x: -x[3])
            
        elif method == "TM_SQDIFF":
            all_points = sorted(all_points, key=lambda x: x[3])
            
        elif method == "TM_SQDIFF_NORMED":
            all_points = sorted(all_points, key=lambda x: x[3])
            
    if rm_redundant == True:
    
        #
        lone_points_list = []
        visited_points_list = []
        
        #
        for point_info in all_points:
            point = point_info[0]
            scale = point_info[2]
            all_visited_points_not_close = True
            
            #
            if len(visited_points_list) != 0:
                for visited_point in visited_points_list:
                    if ((abs(visited_point[0] - point[0]) < (width * scale / 100)) and (abs(visited_point[1] - point[1]) < (height * scale / 100))):
                        all_visited_points_not_close = False
                if all_visited_points_not_close == True:
                    lone_points_list.append(point_info)
                    visited_points_list.append(point)
                    
            else:
                lone_points_list.append(point_info)
                visited_points_list.append(point)
                
        points_list = lone_points_list
        
    else:
        points_list = all_points
    
    #
    color_filtered_list = []
    template_channels = cv2.mean(rgbtemplate)
    template_channels = np.array([template_channels[0], template_channels[1], template_channels[2]])
    
    #
    for point_info in points_list:
        point = point_info[0]
        cropped_img = rgbimage[point[1]:point[1]+height, point[0]:point[0]+width]
        cropped_channels = cv2.mean(cropped_img)
        cropped_channels = np.array([cropped_channels[0], cropped_channels[1], cropped_channels[2]])
        diff_observation = cropped_channels - template_channels
        total_diff = np.sum(np.absolute(diff_observation))
        #print(total_diff)
        
        #
        if total_diff < rgbdiff_thresh:
            color_filtered_list.append([point_info[0],point_info[1],point_info[2],point_info[3],total_diff])
    
    #
    print(color_filtered_list)
    
    return color_filtered_list

def draw_angled_rec(x0, y0, width, height, angle, img):

    _angle = angle * math.pi / 180.0
    b = math.cos(_angle) * 0.5
    a = math.sin(_angle) * 0.5
    pt0 = (int(x0 - a * height - b * width),
           int(y0 + b * height - a * width))
    pt1 = (int(x0 + a * height - b * width),
           int(y0 - b * height - a * width))
    pt2 = (int(2 * x0 - pt0[0]), int(2 * y0 - pt0[1]))
    pt3 = (int(2 * x0 - pt1[0]), int(2 * y0 - pt1[1]))

    cv2.line(img, pt0, pt1, (0, 255, 0), 3)
    cv2.line(img, pt1, pt2, (0, 255, 0), 3)
    cv2.line(img, pt2, pt3, (0, 255, 0), 3)
    cv2.line(img, pt3, pt0, (0, 255, 0), 3)

def main(image, savelocation, templatelocation, minthreshold, maxthreshold, method, count):
    #
    img_bgr = image
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    
    #
    template_bgr = cv2.imread(templatelocation)
    template_rgb = cv2.cvtColor(template_bgr, cv2.COLOR_BGR2RGB)
    
    #
    cropped_template_rgb = template_rgb
    cropped_template_rgb = np.array(cropped_template_rgb)
    cropped_template_gray = cv2.cvtColor(cropped_template_rgb, cv2.COLOR_RGB2GRAY)
    height, width = cropped_template_gray.shape
    
    # TM_CCOEFF, TM_CCOEFF_NORMED, TM_CCORR, TM_CCORR_NORMED, TM_SQDIFF, TM_SQDIFF_NORMED
    points_list = invariantMatchTemplate(img_rgb, cropped_template_rgb, method, minthreshold, maxthreshold, [0,360], 5, [100,150], 5, True, True, count)
    
    #
    centers_list = []
    for point_info in points_list:
        point = point_info[0] #point
        #print("Point:", point)
        angle = point_info[1] #angle
        #print("Corresponding angle:", angle)
        scale = point_info[2] #scale
        #print("Corresponding scale:", scale)
        
        draw_angled_rec(point_info[0][0] + width/2, point_info[0][1] + height/2, width * point_info[2]/100, height * point_info[2]/100, point_info[1], img_bgr) # x0, y0, width, height, angle, img

    # save image to disk
    cv2.imwrite(savelocation, img_bgr)

if __name__ == "__main__":
    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "s:t:i:j:k:d:w:h:c:", ["savelocation =","templatelocation =","minthreshold =","maxthreshold =","method =","debug =","width =","height =","count ="])
    except:
        print('pass the arguments like -s <save location> -t <template location> -i <min threshold> -j <max threshold> -k <method> -d <debug> -w <width> -h <height> -c <count>')

    # set defaults
    savelocation = 'image.jpg'
    templatelocation = 'template.jpg'
    minthreshold = 0.1
    maxthreshold = 500
    method = 'TM_CCOEFF_NORMED'
    debug = False
    width = 3280
    height = 2464
    count = 0
    
    for o,v in opts:
        if o in ['-s','--savelocation']:
            savelocation = v
        elif o in ['-t','--templatelocation']:
            templatelocation = v
        elif o in ['-i','--minthreshold']:
            minthreshold = float(v)
        elif o in ['-j','--maxthreshold']:
            maxthreshold = float(v)
        elif o in ['-k','--method']:
            method = v
        elif o in ['-d','--debug']:
            debug = str_to_bool(v)
        elif o in ['-w','--width']:
            width = int(v)
        elif o in ['-h','--height']:
            height = int(v)
        elif o in ['-c','--count']:
            count = int(v)
    
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
    main(image, savelocation, templatelocation, minthreshold, maxthreshold, method, count)

    #
    print(str(datetime.datetime.now()))