# https://www.geeksforgeeks.org/camera-calibration-with-python-opencv/
# https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
# https://stackoverflow.com/questions/37310210/camera-calibration-with-opencv-how-to-adjust-chessboard-square-size
# Import required modules
import cv2
import numpy
import os
import getopt
import sys
import time
import datetime
import os
import math

def capture_image(debug, width, height):
    if debug:
        image = cv2.imread("Input.jpg") # temporary while we run on windows to develop

    else:
        from picamera2 import Picamera2
        camera = Picamera2()
        config = camera.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        camera.configure(config)
        camera.start()
        # First frames are often dark while auto-exposure settles.
        time.sleep(0.4)
        camera.capture_array()
        image = camera.capture_array()
        camera.stop()
        camera.close()

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
    average = abs(total / count)
    return average
    
def calculate_average_square_size(found, corners, pattern_size):
    if found:
        # Calculate the distances between all adjacent corners
        square_sizes = []
        
        for i in range(pattern_size[1] - 1):
            for j in range(pattern_size[0] - 1):
                # Horizontal distance
                dx = corners[i * pattern_size[0] + j + 1][0][0] - corners[i * pattern_size[0] + j][0][0]
                dy = corners[i * pattern_size[0] + j + 1][0][1] - corners[i * pattern_size[0] + j][0][1]
                horizontal_distance = numpy.sqrt(dx**2 + dy**2)
                
                # Vertical distance
                dx = corners[(i + 1) * pattern_size[0] + j][0][0] - corners[i * pattern_size[0] + j][0][0]
                dy = corners[(i + 1) * pattern_size[0] + j][0][1] - corners[i * pattern_size[0] + j][0][1]
                vertical_distance = numpy.sqrt(dx**2 + dy**2)
                
                # Add distances to the list
                square_sizes.append(horizontal_distance)
                square_sizes.append(vertical_distance)
        
        # Calculate the average square size
        average_square_size = numpy.mean(square_sizes)
        
        return average_square_size
    else:
        return None
    
def get_index_value(array, x, y, column_count):
    # X + (Y*W)
    value = array[x + (y*column_count)][0]
    return value
    
def get_rotation_offset(corners, checkerboard):
    
    x1 = get_index_value(corners,0,0,checkerboard[0])
    x2 = get_index_value(corners,0,checkerboard[1]-1,checkerboard[0])
    
    rotation_offset = 0
    
    if x1[0] == x2[0]:
        if x1[1] > x2[1]:
            rotation_offset = 180
        else:
            rotation_offset = 0
            
    if x1[1] == x2[1]:
        if x1[0] > x2[0]:
            rotation_offset = 90
        else:
            rotation_offset = -90
            
    elif x1[0] > x2[0]:
        if x1[1] < x2[1]:   # quadrant III
            rotation_offset = math.atan((x1[0]-x2[0])/(x2[1]-x1[1]))
            rotation_offset = math.degrees(rotation_offset)
            
        else:               # quadrant II
            rotation_offset = math.atan((x1[0]-x2[0])/(x1[1]-x2[1]))
            rotation_offset = math.degrees(rotation_offset)
            rotation_offset = -rotation_offset
            
    elif x1[0] < x2[0]:
        if x1[1] < x2[1]:   # quadrant IV
            rotation_offset = math.atan((x2[0]-x1[0])/(x2[1]-x1[1]))
            rotation_offset = math.degrees(rotation_offset)
            rotation_offset = -rotation_offset
            
        else:               # quadrant I
            rotation_offset = math.atan((x2[0]-x1[0])/(x1[1]-x2[1]))
            rotation_offset = math.degrees(rotation_offset)
        
    return rotation_offset
    
def get_top_left(corners, checkerboard):
    coordinates = get_index_value(corners,0,0,checkerboard[0])
    return coordinates

def get_bot_right(corners, checkerboard):
    coordinates = get_index_value(corners,checkerboard[0]-1,checkerboard[1]-1,checkerboard[0])
    return coordinates

# https://savvycalculator.com/rotation-calculator-new-coordinates-by-rotation/
# formula to rotate coordinates around 0,0 counter-clockwise
# x’ = x * cos(θ) – y * sin(θ) 
# y’ = x * sin(θ) + y * cos(θ)
def ptRotatePt2f(ptInput, ptOrg, dAngle):
    # Calculate width and height based on origin coordinates
    dWidth = ptOrg[0] * 2
    dHeight = ptOrg[1] * 2
    # Adjust y-coordinates to work with the origin at the bottom left
    dY1 = dHeight - ptInput[1]
    dY2 = dHeight - ptOrg[1]
    # Apply rotation matrix
    dX = (ptInput[0] - ptOrg[0]) * math.cos(dAngle) - (dY1 - ptOrg[1]) * math.sin(dAngle) + ptOrg[0]
    dY = (ptInput[0] - ptOrg[0]) * math.sin(dAngle) + (dY1 - ptOrg[1]) * math.cos(dAngle) + dY2
    # Adjust back to the original coordinate system
    dY = -dY + dHeight
    # Return new coordinates
    return numpy.array([dX, dY]) 
    
def rotate_coordinates_counterclockwise(CoordinatesToRotate, CoordinatesToRotateAround, AngleToRotateInRadians):
    # Translate the coordinates to the origin
    translated_x = CoordinatesToRotate[0] - CoordinatesToRotateAround[0]
    translated_y = CoordinatesToRotate[1] - CoordinatesToRotateAround[1]
    # Apply the rotation matrix
    rotated_x = translated_x * numpy.cos(AngleToRotateInRadians) - translated_y * numpy.sin(AngleToRotateInRadians)
    rotated_y = translated_x * numpy.sin(AngleToRotateInRadians) + translated_y * numpy.cos(AngleToRotateInRadians)
    # Translate the coordinates back to the original position
    new_x = rotated_x + CoordinatesToRotateAround[0]
    new_y = rotated_y + CoordinatesToRotateAround[1]
    return numpy.array([new_x, new_y])
    
def rotate_coordinates_clockwise(CoordinatesToRotate, CoordinatesToRotateAround, AngleToRotateInRadians):
    # Translate the coordinates to the origin
    translated_x = CoordinatesToRotate[0] - CoordinatesToRotateAround[0]
    translated_y = CoordinatesToRotate[1] - CoordinatesToRotateAround[1]
    # Apply the clockwise rotation matrix
    rotated_x = translated_x * numpy.cos(AngleToRotateInRadians) + translated_y * numpy.sin(AngleToRotateInRadians)
    rotated_y = -translated_x * numpy.sin(AngleToRotateInRadians) + translated_y * numpy.cos(AngleToRotateInRadians)
    # Translate the coordinates back to the original position
    new_x = rotated_x + CoordinatesToRotateAround[0]
    new_y = rotated_y + CoordinatesToRotateAround[1]
    # Return the new coordinates as a list of floats
    return [float(new_x), float(new_y)]
    
def _log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()

def _as_corner_array(corners):
    corners = numpy.asarray(corners, dtype=numpy.float32)
    if corners.ndim == 2:
        corners = corners.reshape(-1, 1, 2)
    elif corners.ndim == 3 and corners.shape[1] != 1:
        corners = corners.reshape(-1, 1, 2)
    return corners

def _gray_views(gray):
    views = [("gray", gray)]
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    views.append(("clahe", clahe.apply(gray)))
    views.append(("invert", cv2.bitwise_not(gray)))
    return views

def _sb_flags(exhaustive):
    flags = 0
    if hasattr(cv2, "CALIB_CB_NORMALIZE_IMAGE"):
        flags |= cv2.CALIB_CB_NORMALIZE_IMAGE
    if hasattr(cv2, "CALIB_CB_ACCURACY"):
        flags |= cv2.CALIB_CB_ACCURACY
    if exhaustive and hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
        flags |= cv2.CALIB_CB_EXHAUSTIVE
    return flags

def _try_sb(image, pattern_size, exhaustive):
    if not hasattr(cv2, "findChessboardCornersSB"):
        return False, None
    flags = _sb_flags(exhaustive)
    try:
        ret, corners = cv2.findChessboardCornersSB(image, pattern_size, flags=flags)
    except TypeError:
        ret, corners = cv2.findChessboardCornersSB(image, pattern_size, flags)
    if ret:
        return True, _as_corner_array(corners)
    return False, None

def _try_classic(image, pattern_size):
    # FAST_CHECK is omitted: it false-negatives dense and square boards.
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(image, pattern_size, flags=flags)
    if ret:
        return True, _as_corner_array(corners)
    return False, None

def _find_at_scale(gray, pattern_size, use_exhaustive):
    views = _gray_views(gray)

    for name, img in views:
        found, corners = _try_sb(img, pattern_size, False)
        if found:
            _log("checkerboard found with findChessboardCornersSB (%s)" % name)
            return True, corners, hasattr(cv2, "CALIB_CB_ACCURACY")

    for name, img in views:
        found, corners = _try_classic(img, pattern_size)
        if found:
            _log("checkerboard found with findChessboardCorners (%s)" % name)
            return True, corners, False

    if use_exhaustive:
        for name, img in views[:2]:
            found, corners = _try_sb(img, pattern_size, True)
            if found:
                _log("checkerboard found with findChessboardCornersSB exhaustive (%s)" % name)
                return True, corners, hasattr(cv2, "CALIB_CB_ACCURACY")

    return False, None, False

def estimate_inner_corners(gray):
    """Rough inner-corner count from checker period. Overlay hint only."""
    h, w = gray.shape[:2]
    k = 15
    gray_f = gray.astype(numpy.float32)
    mu = cv2.blur(gray_f, (k, k))
    var = cv2.blur(gray_f * gray_f, (k, k)) - mu * mu
    var_n = cv2.normalize(var, None, 0, 255, cv2.NORM_MINMAX).astype(numpy.uint8)
    ys, xs = numpy.where(var_n > 40)
    if xs.size < 100:
        return None
    crop = gray[int(ys.min()):int(ys.max()) + 1, int(xs.min()):int(xs.max()) + 1]
    if crop.size == 0 or min(crop.shape[:2]) < 20:
        return None
    row = cv2.GaussianBlur(crop[crop.shape[0] // 2:crop.shape[0] // 2 + 1, :].astype(numpy.float32), (1, 5), 0).ravel()
    row = row - row.mean()
    corr = numpy.correlate(row, row, mode="full")
    corr = corr[corr.size // 2:]
    period = None
    for i in range(4, min(80, corr.size - 1)):
        if corr[i] > corr[i - 1] and corr[i] > corr[i + 1] and corr[i] > 0.2 * corr[0]:
            period = i
            break
    if period is None:
        return None
    square_px = period / 2.0
    if square_px < 3:
        return None
    inner_x = max(1, int(round(crop.shape[1] / square_px)) - 1)
    inner_y = max(1, int(round(crop.shape[0] / square_px)) - 1)
    return square_px, inner_x, inner_y, (w, h)

def find_checkerboard(gray, pattern_size):
    """Locate inner chessboard corners.

    Square high-count boards often fail findChessboardCorners with
    CALIB_CB_FAST_CHECK. Try the sector-based detector first, then classic
    detection without FAST_CHECK. On small captures, retry at 2x/3x because
    5 mm squares at 640x400 are only a few pixels.
    """
    inner = pattern_size[0] * pattern_size[1]
    square = pattern_size[0] == pattern_size[1]
    use_exhaustive = square or inner >= 80
    patterns = [pattern_size]
    if pattern_size[0] != pattern_size[1]:
        patterns.append((pattern_size[1], pattern_size[0]))

    scales = [1.0]
    if min(gray.shape[:2]) < 1000:
        scales.extend([2.0, 3.0])

    for scale in scales:
        if scale == 1.0:
            img = gray
        else:
            img = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            _log("retrying checkerboard detect at %.1fx (%sx%s)" % (scale, img.shape[1], img.shape[0]))
        for pattern in patterns:
            found, corners, sub = _find_at_scale(img, pattern, use_exhaustive and scale == 1.0)
            if found:
                if scale != 1.0:
                    corners = corners / scale
                if pattern != pattern_size:
                    _log("found swapped inner corners %sx%s" % (pattern[0], pattern[1]))
                return True, corners, sub, pattern

    _log("checkerboard not found for inner corners %sx%s" % (pattern_size[0], pattern_size[1]))
    return False, None, False, pattern_size

def subpix_window(corners, pattern_size):
    avg = calculate_average_square_size(True, corners, pattern_size)
    if avg is None or avg <= 0:
        return (5, 5)
    size = int(round(avg / 3.0))
    if size % 2 == 0:
        size -= 1
    size = max(3, min(11, size))
    return (size, size)

def drawOrientation(image, origin, rotation):
    # y-green x-red z-blue
    start_point = (int(origin[0]), int(origin[1]))
    
    x = origin.copy()
    x[0] += 100
    y = origin.copy()
    y[1] += 100
    
    xr = ptRotatePt2f(x, origin, math.radians(-rotation))
    xe = (int(xr[0]), int(xr[1]))
    yr = ptRotatePt2f(y, origin, math.radians(-rotation))
    ye = (int(yr[0]), int(yr[1]))

    thickness = 3
    color = (0, 0, 255)
    image = cv2.arrowedLine(image, start_point, xe, color, thickness)
    color = (0, 255, 0)
    image = cv2.arrowedLine(image, start_point, ye, color, thickness)
    
    return image

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

    prev_img_shape = None

    image = capture_image(debug, width, height)
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    ret, corners, already_subpixel, found_pattern = find_checkerboard(grayColor, checkerboard)

    # If desired number of corners can be detected then,
    # refine the pixel coordinates and display
    # them on the images of checker board
    if ret == True:
        checkerboard = found_pattern
        objectp3d = numpy.zeros((1, checkerboard[0]
                            * checkerboard[1],
                            3), numpy.float32)
        objectp3d[0, :, :2] = numpy.mgrid[0:checkerboard[0],
                                    0:checkerboard[1]].T.reshape(-1, 2)
        objectp3d = objectp3d * squaresize
        
        threedpoints.append(objectp3d)

        # SB with CALIB_CB_ACCURACY is already sub-pixel. A fixed 11x11
        # cornerSubPix window is too large for 5 mm squares.
        if already_subpixel:
            corners2 = corners
        else:
            win = subpix_window(corners, checkerboard)
            corners2 = cv2.cornerSubPix(
                grayColor, corners, win, (-1, -1), criteria)
        
        # get pixel average so we can calulate how many pixels per user units
        #pixel_average = get_pixel_average(corners2, checkerboard)
        pixel_average = calculate_average_square_size(True, corners2, checkerboard)
        
        #
        rotation_offset = get_rotation_offset(corners2, checkerboard)
        origin = get_top_left(corners2, checkerboard)
        top_left = origin # leaving for backwards compatibility for now.
        bot_right = get_bot_right(corners2, checkerboard)

        #
        twodpoints.append(corners2)
        
        # Draw and display the corners
        image = cv2.drawChessboardCorners(image, 
                                          checkerboard, 
                                          corners2, ret)
        
        image = drawOrientation(image, origin, rotation_offset)

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
        fail = image.copy()
        h, w = fail.shape[:2]
        scale = max(0.5, min(w, h) / 640.0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        label = "checkerboard not found %sx%s" % (checkerboard[0], checkerboard[1])
        cv2.putText(fail, label, (16, int(32 * scale) + 8), font, 0.7 * scale, (0, 0, 255), max(1, int(2 * scale)))
        hint = "%sx%s capture" % (w, h)
        estimate = estimate_inner_corners(grayColor)
        if estimate is not None:
            square_px, inner_x, inner_y, _ = estimate
            hint = "%sx%s  ~%.0fpx/sq  try %sx%s inner" % (w, h, square_px, inner_x, inner_y)
        cv2.putText(fail, hint, (16, int(64 * scale) + 8), font, 0.55 * scale, (0, 0, 255), max(1, int(2 * scale)))
        cv2.imwrite(resultfile, fail)
        
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