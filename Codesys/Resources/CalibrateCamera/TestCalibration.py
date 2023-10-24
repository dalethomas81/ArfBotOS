import cv2
import numpy as np
import os

def load_cal_data(path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    checkerboard = cv_file.getNode("checkerboard").mat()
    squaresize = cv_file.getNode("squaresize").real()
    camera_matrix = cv_file.getNode("matrix").mat()
    dist_matrix = cv_file.getNode("distortion").mat()   
    pixel_ratio = cv_file.getNode("pixelratio").real()
    cv_file.release()
    
    return [checkerboard, squaresize, camera_matrix, dist_matrix, pixel_ratio]

def filter_image(image, mtx, dist, crop):
    
    # get the size of the image
    h,  w = image.shape[:2]

    # apply the calibration data
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
    dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

    if crop == True:
        # crop the image
        x,y,w,h = roi
        image = dst[y:y+h, x:x+w]
    
    return image

# load the calibration data
dir_path = os.path.dirname(os.path.realpath(__file__))
cal_file_path = str(os.path.join(dir_path, 'cal.yaml'))
checkerboard, squaresize, mtx, dist, pixel_ratio = load_cal_data(cal_file_path)

# capture the image
image = cv2.imread('input.jpg')

# filter image through calibration data
image = filter_image(image, mtx, dist, True)

# calculate line size from calibration data
line_size = squaresize * pixel_ratio # mm

# get the shape to you know where to draw the lines
h,  w = image.shape[:2]

#
pt0 = (int(w/2), int(h/2))
pt1 = (int(w/2 + line_size), int(h/2))
cv2.line(image, pt0, pt1, (0, 255, 0), 2)

#
pt2 = (int(w/2), int(h/2))
pt3 = (int(w/2), int(h/2 + line_size))
cv2.line(image, pt2, pt3, (0, 255, 0), 2)

#
cv2.imwrite('result.png',image)

#print (" ")
#input("Press enter to exit;")