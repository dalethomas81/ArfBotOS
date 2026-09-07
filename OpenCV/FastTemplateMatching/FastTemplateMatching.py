import math
import numpy
import cv2
import getopt
import sys
import os
import datetime
import time
from concurrent.futures import ThreadPoolExecutor

# fundamental constants
CV_PI   = 3.1415926535897932384626433832795
CV_2PI  = 6.283185307179586476925286766559
CV_LOG2 = 0.69314718055994530941723212145818
DBL_EPSILON = 2.2204460492503131e-016 # smallest such that 1.0+DBL_EPSILON != 1.0

VISION_TOLERANCE = 0.0000001
D2R = (CV_PI / 180.0)
R2D = (180.0 / CV_PI)
MATCH_CANDIDATE_NUM = 10

SUBITEM_INDEX = 0
SUBITEM_SCORE = 1
SUBITEM_ANGLE = 2
SUBITEM_POS_X = 3
SUBITEM_POS_Y = 4

MAX_SCALE_TIMES = 10
MIN_SCALE_TIMES = 0
SCALE_RATIO = 1.25

colorWaterBlue = (230, 255, 102)
colorBlue = (255, 0, 0)
colorYellow = (0, 255, 255)
colorRed = (0, 0, 255)
colorBlack = (0, 0, 0)
colorGray = (200, 200, 200)
colorSystem = (240, 240, 240)
colorGreen = (0, 255, 0)
colorWhite = (255, 255, 255)
colorPurple = (214, 112, 218)
colorGoldenrod = (15, 185, 255)

class s_VecPtAngle:
    def __init__(self, first, second):
        self.first = first
        self.second = second

class Block:
    def __init__(self, rect_, dMax_, ptMaxLoc_):
        self.rect = rect_
        self.dMax = dMax_
        self.ptMaxLoc = ptMaxLoc_

class s_SingleTargetMatch:
    def __init__(self):
        self.ptLT = numpy.array([0.0, 0.0])
        self.ptRT = numpy.array([0.0, 0.0])
        self.ptRB = numpy.array([0.0, 0.0])
        self.ptLB = numpy.array([0.0, 0.0])
        self.ptCenter = numpy.array([0.0, 0.0])
        self.dMatchedAngle = 0.0
        self.dMatchScore = 0.0
    
    def clear(self):
        self.ptLT.fill(0)
        self.ptRT.fill(0)
        self.ptRB.fill(0)
        self.ptLB.fill(0)
        self.ptCenter.fill(0)
        self.dMatchedAngle = 0.0
        self.dMatchScore = 0.0

class s_TemplData:
    def __init__(self):
        #mat = cv.CreateMat(3, 5, cv.CV_32FC1)
        self.vecPyramid = numpy.array([])
        self.vecTemplMean = numpy.array([])
        self.vecTemplNorm = numpy.array([])
        self.vecInvArea = numpy.array([])
        self.vecResultEqual1 = numpy.array([])
        self.bIsPatternLearned = False
        self.iBorderColor = 0
    
    def clear(self):
        #self.vecPyramid.clear()
        self.vecTemplNorm.fill(0)
        self.vecInvArea.fill(0)
        self.vecTemplMean.fill(0)
        self.vecResultEqual1.fill(False)
    
    def resize(self, iSize):
        self.vecTemplMean = numpy.resize(self.vecTemplMean, (iSize,4))
        self.vecTemplNorm = numpy.resize(self.vecTemplNorm, iSize)
        self.vecInvArea = numpy.resize(self.vecInvArea, iSize)
        self.vecResultEqual1 = numpy.resize(self.vecResultEqual1, iSize)
        self.vecResultEqual1.fill(0)
        self.vecInvArea.fill(1)
        self.vecResultEqual1.fill(False)

class s_MatchParameter:
    def __init__(self, ptMinMax=None, dScore=0, dAngle=0):
        if ptMinMax is None:
            self.pt = numpy.array([0.0, 0.0])
        else:
            self.pt = numpy.array([ptMinMax[0], ptMinMax[1]])
        self.dMatchScore = dScore
        self.dMatchAngle = dAngle
        self.bDelete = False
        self.dNewAngle = 0.0
        self.bPosOnBorder = False
        self.vecResult = numpy.eye(3)
    def __del__(self):
        pass

class CMatchToolDlg:
    def SortPtWithCenter(self, vecSort):
        iSize = len(vecSort)
        ptCenter = numpy.array([0.0, 0.0])
        for i in range(iSize):
            ptCenter += vecSort[i]
        ptCenter /= iSize
        vecX = numpy.array([1, 0])
        vecPtAngle = []
        for i in range(iSize):
            vecPtAngle.append((vecSort[i], 0))
            vec1 = numpy.array([vecSort[i][0] - ptCenter[0], vecSort[i][1] - ptCenter[1]])
            fNormVec1 = vec1[0] * vec1[0] + vec1[1] * vec1[1]
            fDot = vec1[0]
            if vec1[1] < 0:
                vecPtAngle[i] = (vecPtAngle[i][0], math.acos(fDot / fNormVec1) * 180 / math.pi)
            elif vec1[1] > 0:
                vecPtAngle[i] = (vecPtAngle[i][0], 360 - math.acos(fDot / fNormVec1) * 180 / math.pi)
            else:
                if vec1[0] - ptCenter[0] > 0:
                    vecPtAngle[i] = (vecPtAngle[i][0], 0)
                else:
                    vecPtAngle[i] = (vecPtAngle[i][0], 180)
        vecPtAngle.sort(key=lambda x: x[1])
        for i in range(iSize):
            vecSort[i] = vecPtAngle[i][0]

class s_BlockMax:
    class Block:
        def __init__(self, rect_=None, dMax_=0, ptMaxLoc_=None):
            self.rect = rect_
            self.dMax = dMax_
            self.ptMaxLoc = ptMaxLoc_
    def __init__(self, matSrc_=None, sizeTemplate=None):
        self.vecBlock = []
        self.matSrc = matSrc_
        if matSrc_ is None or sizeTemplate is None:
            return
        iBlockW = sizeTemplate[0] * 2
        iBlockH = sizeTemplate[1] * 2
        iCol = matSrc_.shape[1] // iBlockW
        bHResidue = matSrc_.shape[1] % iBlockW != 0
        iRow = matSrc_.shape[0] // iBlockH
        bVResidue = matSrc_.shape[0] % iBlockH != 0
        if iCol == 0 or iRow == 0:
            self.vecBlock.clear()
            return
        self.vecBlock = [self.Block() for i in range(iCol * iRow)]
        iCount = 0
        for y in range(iRow):
            for x in range(iCol):
                rectBlock = (x * iBlockW, y * iBlockH, iBlockW, iBlockH)
                self.vecBlock[iCount].rect = rectBlock
                self.vecBlock[iCount].dMax, _, self.vecBlock[iCount].ptMaxLoc, _ = cv2.minMaxLoc(matSrc_[rectBlock])
                self.vecBlock[iCount].ptMaxLoc = tuple(numpy.array(self.vecBlock[iCount].ptMaxLoc) + numpy.array(rectBlock[:2]))
                iCount += 1
        if bHResidue and bVResidue:
            rectRight = (iCol * iBlockW, 0, matSrc_.shape[1] - iCol * iBlockW, matSrc_.shape[0])
            blockRight = self.Block(rectRight)
            blockRight.dMax, _, blockRight.ptMaxLoc, _ = cv2.minMaxLoc(matSrc_[rectRight])
            blockRight.ptMaxLoc = tuple(numpy.array(blockRight.ptMaxLoc) + numpy.array(rectRight[:2]))
            self.vecBlock.append(blockRight)
            rectBottom = (0, iRow * iBlockH, iCol * iBlockW, matSrc_.shape[0] - iRow * iBlockH)
            blockBottom = self.Block(rectBottom)
            blockBottom.dMax, _, blockBottom.ptMaxLoc, _ = cv2.minMaxLoc(matSrc_[rectBottom])
            blockBottom.ptMaxLoc = tuple(numpy.array(blockBottom.ptMaxLoc) + numpy.array(rectBottom[:2]))
            self.vecBlock.append(blockBottom)
        elif bHResidue:
            rectRight = (iCol * iBlockW, 0, matSrc_.shape[1] - iCol * iBlockW, matSrc_.shape[0])
            blockRight = self.Block(rectRight)
            blockRight.dMax, _, blockRight.ptMaxLoc, _ = cv2.minMaxLoc(matSrc_[rectRight])
            blockRight.ptMaxLoc = tuple(numpy.array(blockRight.ptMaxLoc) + numpy.array(rectRight[:2]))
            self.vecBlock.append(blockRight)
        else:
            rectBottom = (0, iRow * iBlockH, matSrc_.shape[1], matSrc_.shape[0] - iRow * iBlockH)
            blockBottom = self.Block(rectBottom)
            blockBottom.dMax, _, blockBottom.ptMaxLoc, _ = cv2.minMaxLoc(matSrc_[rectBottom])
            blockBottom.ptMaxLoc = tuple(numpy.array(blockBottom.ptMaxLoc) + numpy.array(rectBottom[:2]))
            self.vecBlock.append(blockBottom)
    def UpdateMax(self, rectIgnore):
        if len(self.vecBlock) == 0:
            return
        iSize = len(self.vecBlock)
        for i in range(iSize):
            rectIntersec = rectIgnore & self.vecBlock[i].rect
            if rectIntersec[2] == 0 and rectIntersec[3] == 0:
                continue
            self.vecBlock[i].dMax, _, self.vecBlock[i].ptMaxLoc, _ = cv2.minMaxLoc(self.matSrc[self.vecBlock[i].rect])
            self.vecBlock[i].ptMaxLoc = tuple(numpy.array(self.vecBlock[i].ptMaxLoc) + numpy.array(self.vecBlock[i].rect[:2]))
    def GetMaxValueLoc(self):
        iSize = len(self.vecBlock)
        if iSize == 0:
            dMax, _, ptMaxLoc, _ = cv2.minMaxLoc(self.matSrc)
            return dMax, ptMaxLoc
        iIndex = 0
        dMax = self.vecBlock[0].dMax
        for i in range(1, iSize):
            if self.vecBlock[i].dMax >= dMax:
                iIndex = i
                dMax = self.vecBlock[i].dMax
        ptMaxLoc = self.vecBlock[iIndex].ptMaxLoc
        return dMax, ptMaxLoc

m_iMinReduceArea = 256
m_iMessageCount = 0
m_ckBitwiseNot = False
m_bToleranceRange = False
m_dTolerance1 = 0.0
m_dTolerance2 = 0.0
m_dTolerance3 = 0.0
m_dTolerance4 = 0.0
bSubPixelEstimation = False
m_bShowResult = True
#m_matSrc = cv2.Mat
#m_matDst = cv2.Mat
m_iScaleTimes = 10
m_dNewScale = 1.0
m_dSrcScale = 1.0
m_dDstScale = 1.0
m_TemplData = s_TemplData() #s_TemplData
m_vecSingleTargetData = [] #s_SingleTargetMatch
vecBlock = Block # array of type Block
UseSIMD = False

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

def capture_image(debug, width, height):
    if debug:
        image = cv2.imread("Input.jpg",cv2.IMREAD_GRAYSCALE)

    else:
        #import libcamera
        from picamera2 import Picamera2
        camera = Picamera2()
        #config = camera.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        config = camera.create_preview_configuration(main={"size": (width, height), "format": "YUV420"})
        #config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        camera.configure(config)
        camera.start()
        time.sleep(0.1)
        image = camera.capture_array()
        camera.stop()
        camera.close()
        image = image[:height, :width] # grey
 
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

def _xy(pt):
    arr = numpy.asarray(pt, dtype=numpy.float64).reshape(-1)
    return float(arr[0]), float(arr[1])

def crop_image(image, top_left, bot_right):
    x0, y0 = _xy(top_left)
    x1, y1 = _xy(bot_right)
    x, y = int(x0), int(y0)
    w, h = int(x1 - x0), int(y1 - y0)
    cropped = image[y:y+h, x:x+w].copy()
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

def sort_pt_with_center(vecSort):
    iSize = len(vecSort)
    ptCenter = (0.0, 0.0)
    for i in range(iSize):
        ptCenter += vecSort[i][0]
    ptCenter /= iSize

    vecPtAngle = []# * iSize

    for i in range(iSize):
        vecPtAngle.append(s_VecPtAngle((vecSort[i][0][0],vecSort[i][0][1]), 0.0)) # .first = vecSort[i][0]
        vec1 = (vecSort[i][0][0] - ptCenter[0], vecSort[i][0][1] - ptCenter[1])
        fNormVec1 = vec1[0] * vec1[0] + vec1[1] * vec1[1]
        fDot = vec1[0]
        if vec1[1] < 0: # If the point is above the center
            vecPtAngle[i].second = numpy.arccos(fDot / fNormVec1) * 180 / numpy.pi
        elif vec1[1] > 0: # below
            vecPtAngle[i].second = 360 - numpy.arccos(fDot / fNormVec1) * 180 / numpy.pi
        else: # Point and center are in the same Y
            if vec1[0] - ptCenter[0][0] > 0:
                vecPtAngle[i].second = 0
            else:
                vecPtAngle[i].second = 180

    vecPtAngle.sort(key=lambda x: x.second, reverse=True)

    vecSorted = []
    for i in range(iSize):
        vecSorted.append(vecPtAngle[i].first)
    vecSortedNp = numpy.array(vecSorted)

    return vecSortedNp

def _pt2i(pt):
    x, y = _xy(pt)
    return (int(round(x)), int(round(y)))

def draw_line(img,pt1,pt2,color = colorGreen,thickness=1,style='dotted',gap=1):
    t = max(1, int(round(thickness)))
    p1 = _pt2i(pt1)
    p2 = _pt2i(pt2)
    if style == 'dotted':
        gap = max(1.0, float(gap))
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 1:
            cv2.circle(img, p1, t, color, -1)
            return img
        n = max(1, int(dist / gap))
        for i in range(0, n + 1, 2):
            x = int(p1[0] + dx * i / n)
            y = int(p1[1] + dy * i / n)
            cv2.circle(img, (x, y), t, color, -1)
        return img
    cv2.line(img, p1, p2, color, t, cv2.LINE_AA)
    return img

def draw_dash_line(matDraw, ptStart, ptEnd, color1=(0, 0, 255), color2=(255, 255, 255)):
    #Scalar color1 = Scalar (0, 0, 255), Scalar color2 = Scalar::all (255)
    _pt1 = (int(ptStart[1]),int(ptStart[0]))
    _pt2 = (int(ptEnd[1]),int(ptEnd[0]))
    itLine = cv2.line(img=matDraw, 
                      pt1=_pt1, 
                      pt2=_pt2,
                      color=(255, 255, 255),
                      thickness=8, 
                      lineType=0)

    iCount = itLine.count
    for i in range(0, iCount, 1):
        if i % 3 == 0:
            itLine[i] = (color2[0], color2[1], color2[2])
        else:
            itLine[i] = (color1[0], color1[1], color1[2])

    return itLine

def draw_mark_cross(matDraw, iX, iY, iLength, color, iThickness):
    if matDraw.size == 0:
        return
    ptC = (iX, iY)
    #matDraw = cv2.line(matDraw, ptC - (iLength, 0), ptC + (iLength, 0), color, iThickness)
    #matDraw = cv2.line(matDraw, ptC - (0, iLength), ptC + (0, iLength), color, iThickness)
    matDraw = draw_line(img=matDraw,
                        pt1=(ptC[0] - iLength, ptC[1]),
                        pt2=(ptC[0] + iLength, ptC[1]),
                        color=color,thickness=iThickness,style='none',gap=0)
    matDraw = draw_line(img=matDraw,
                        pt1=(ptC[0], ptC[1] - iLength),
                        pt2=(ptC[0], ptC[1] + iLength),
                        color=color,thickness=iThickness,style='none',gap=0)

    return matDraw

def IM_Conv_SIMD(pCharKernel, pCharConv, iLength):
    iBlockSize = 16
    Block = iLength // iBlockSize
    SumV = numpy.zeros(16, dtype=numpy.int32)
    Zero = numpy.zeros(16, dtype=numpy.int32)
    for Y in range(0, Block * iBlockSize, iBlockSize):
        SrcK = numpy.frombuffer(pCharKernel[Y:Y+iBlockSize], dtype=numpy.uint8)
        SrcC = numpy.frombuffer(pCharConv[Y:Y+iBlockSize], dtype=numpy.uint8)
        SrcK_L = numpy.reshape(SrcK[:8], (8, 1))
        SrcK_H = numpy.reshape(SrcK[8:], (8, 1))
        SrcC_L = numpy.reshape(SrcC[:8], (8, 1))
        SrcC_H = numpy.reshape(SrcC[8:], (8, 1))
        SumT = numpy.add(numpy.multiply(SrcK_L, SrcC_L), numpy.multiply(SrcK_H, SrcC_H)).sum(axis=0)
        SumV = numpy.add(SumV, SumT)
    Sum = numpy.sum(SumV)
    for Y in range(Block * iBlockSize, iLength):
        Sum += pCharKernel[Y] * pCharConv[Y]
    return Sum

def CCOEFF_Denominator(matSrc, pTemplData, matResult, iLayer):
    if pTemplData.vecResultEqual1[iLayer]:
        return numpy.ones(matResult.shape, dtype=numpy.float32)

    sum_img, sqsum = cv2.integral2(matSrc, sdepth=cv2.CV_64F, sqdepth=cv2.CV_64F)
    h, w = pTemplData.vecPyramid[iLayer].shape[:2]
    rh, rw = matResult.shape[:2]

    p0 = sum_img[0:rh, 0:rw]
    p1 = sum_img[0:rh, w:w + rw]
    p2 = sum_img[h:h + rh, 0:rw]
    p3 = sum_img[h:h + rh, w:w + rw]
    q0 = sqsum[0:rh, 0:rw]
    q1 = sqsum[0:rh, w:w + rw]
    q2 = sqsum[h:h + rh, 0:rw]
    q3 = sqsum[h:h + rh, w:w + rw]

    window_sum = p0 - p1 - p2 + p3
    window_sqsum = q0 - q1 - q2 + q3
    dTemplMean0 = float(pTemplData.vecTemplMean[iLayer][0])
    dTemplNorm = float(pTemplData.vecTemplNorm[iLayer])
    dInvArea = float(pTemplData.vecInvArea[iLayer])

    num = matResult.astype(numpy.float64, copy=False) - window_sum * dTemplMean0
    wndMean2 = window_sum * window_sum * dInvArea
    diff2 = numpy.maximum(window_sqsum - wndMean2, 0.0)
    eps = numpy.finfo(numpy.float64).eps
    too_small = diff2 <= numpy.minimum(0.5, 10.0 * eps * window_sqsum)
    t = numpy.sqrt(diff2, dtype=numpy.float64) * dTemplNorm
    t = numpy.where(too_small, 0.0, t)

    abs_num = numpy.abs(num)
    out = numpy.zeros(matResult.shape, dtype=numpy.float32)
    mask_div = abs_num < t
    mask_clip = (~mask_div) & (abs_num < t * 1.125)
    numpy.divide(num, t, out=out, where=mask_div)
    out[mask_clip] = numpy.sign(num[mask_clip])
    return out

def FilterWithRotatedRect(vec, iMethod, dMaxOverLap):
    iMatchSize = len(vec)
    for i in range(iMatchSize - 1):
        if vec[i].bDelete:
            continue
        for j in range(i + 1, iMatchSize):
            if vec[j].bDelete:
                continue
            rect1 = vec[i].rectR
            rect2 = vec[j].rectR
            vecInterSec = []
            iInterSecType, vecInterSec = cv2.rotatedRectangleIntersection(rect1, rect2)
            if iInterSecType == cv2.INTERSECT_NONE:
                continue
            elif iInterSecType == cv2.INTERSECT_FULL:
                if iMethod == cv2.TM_SQDIFF:
                    iDeleteIndex = j if vec[i].dMatchScore <= vec[j].dMatchScore else i
                else:
                    iDeleteIndex = j if vec[i].dMatchScore >= vec[j].dMatchScore else i
                vec[iDeleteIndex].bDelete = True
            else:
                if len(vecInterSec) < 3:
                    continue
                else:
                    vecInterSec = sort_pt_with_center(vecInterSec)
                    dArea = cv2.contourArea(vecInterSec)
                    rect1_size = rect1[1][0] * rect1[1][1]
                    dRatio = dArea / (rect1_size)
                    if dRatio > dMaxOverLap:
                        if iMethod == cv2.TM_SQDIFF:
                            iDeleteIndex = j if vec[i].dMatchScore <= vec[j].dMatchScore else i
                        else:
                            iDeleteIndex = j if vec[i].dMatchScore >= vec[j].dMatchScore else i
                        vec[iDeleteIndex].bDelete = True
    vec = [x for x in vec if not x.bDelete]

    return vec

def RefreshSrcView(matSrc, matDst, vecSingleTargetData, boxRatio):

    global m_dNewScale
    solidLineThickness = max(1, int(round(0.9 * m_dNewScale * boxRatio)))
    fontScale = max(0.4, 0.675 * m_dNewScale * boxRatio)
    fontthickness = max(1, int(round(0.9 * m_dNewScale * boxRatio)))
    markCrossLength = 4.5 * m_dNewScale * boxRatio
    srcRows = matSrc.shape[0]
    srcCols = matSrc.shape[1]
    dstRows = matDst.shape[0]
    dstCols = matDst.shape[1]
    size = (int(m_dNewScale * srcCols), int(m_dNewScale * srcRows))
    matColorSrc = cv2.cvtColor(src=matSrc, code=cv2.COLOR_GRAY2BGR)
    matResize = cv2.resize(src=matColorSrc, dsize=size)
    iSize = len(vecSingleTargetData)
    if m_bShowResult:
        for i in range(iSize):
            ptLT = vecSingleTargetData[i].ptLT * m_dNewScale
            ptLB = vecSingleTargetData[i].ptLB * m_dNewScale
            ptRB = vecSingleTargetData[i].ptRB * m_dNewScale
            ptRT = vecSingleTargetData[i].ptRT * m_dNewScale
            ptC = vecSingleTargetData[i].ptCenter * m_dNewScale

            box = numpy.array([_pt2i(ptLT), _pt2i(ptLB), _pt2i(ptRB), _pt2i(ptRT)], dtype=numpy.int32)
            cv2.polylines(matResize, [box], True, colorGreen, solidLineThickness, cv2.LINE_AA)

            if dstCols > dstRows:
                ptDis1 = (ptLB - ptLT) / 3
                ptDis2 = (ptRT - ptLT) / 3 * (dstRows / float(dstCols))
            else:
                ptDis1 = (ptLB - ptLT) / 3 * (dstCols / float(dstRows))
                ptDis2 = (ptRT - ptLT) / 3
            cv2.line(matResize, _pt2i(ptLT), _pt2i(ptLT + ptDis1 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptLT), _pt2i(ptLT + ptDis2 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptRT), _pt2i(ptRT + ptDis1 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptRT), _pt2i(ptRT - ptDis2 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptRB), _pt2i(ptRB - ptDis1 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptRB), _pt2i(ptRB - ptDis2 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptLB), _pt2i(ptLB - ptDis1 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptLB), _pt2i(ptLB + ptDis2 / 2), colorGreen, solidLineThickness, cv2.LINE_AA)
            cv2.line(matResize, _pt2i(ptLT + ptDis1), _pt2i(ptLT + ptDis2), colorGreen, solidLineThickness, cv2.LINE_AA)

            matResize = draw_mark_cross(matResize, ptC[0], ptC[1], markCrossLength, colorGreen, solidLineThickness)

            label = f"{i}"
            _ptText = (ptLT + ptRT) / 2
            matResize = cv2.putText(img=matResize, text=label, org=_pt2i(_ptText), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=fontScale, color=colorGreen, thickness=fontthickness)

    return matResize

def GetBestRotationSize(sizeSrc, sizeDst, dRAngle):
    
    dRAngle_radian = dRAngle * math.pi / 180
    ptLT = (0, 0)
    ptLB = (0, sizeSrc[0] - 1)
    ptRB = (sizeSrc[1] - 1, sizeSrc[0] - 1)
    ptRT = (sizeSrc[1] - 1, 0)
    ptCenter = ((sizeSrc[1] - 1) / 2.0, (sizeSrc[0] - 1) / 2.0)
    ptLT_R = ptRotatePt2f(numpy.array(ptLT), numpy.array(ptCenter), dRAngle_radian)
    ptLB_R = ptRotatePt2f(numpy.array(ptLB), numpy.array(ptCenter), dRAngle_radian)
    ptRB_R = ptRotatePt2f(numpy.array(ptRB), numpy.array(ptCenter), dRAngle_radian)
    ptRT_R = ptRotatePt2f(numpy.array(ptRT), numpy.array(ptCenter), dRAngle_radian)
    fTopY = max(ptLT_R[1], ptLB_R[1], ptRB_R[1], ptRT_R[1])
    fBottomY = min(ptLT_R[1], ptLB_R[1], ptRB_R[1], ptRT_R[1])
    fRightX = max(ptLT_R[0], ptLB_R[0], ptRB_R[0], ptRT_R[0])
    fLeftX = min(ptLT_R[0], ptLB_R[0], ptRB_R[0], ptRT_R[0])
    if dRAngle > 360:
        dRAngle -= 360
    elif dRAngle < 0:
        dRAngle += 360
    if abs(abs(dRAngle) - 90) < VISION_TOLERANCE or abs(abs(dRAngle) - 270) < VISION_TOLERANCE:
        return (sizeSrc[0], sizeSrc[1])
    elif abs(dRAngle) < VISION_TOLERANCE or abs(abs(dRAngle) - 180) < VISION_TOLERANCE:
        return sizeSrc
    dAngle = dRAngle
    if dAngle > 0 and dAngle < 90:
        pass
    elif dAngle > 90 and dAngle < 180:
        dAngle -= 90
    elif dAngle > 180 and dAngle < 270:
        dAngle -= 180
    elif dAngle > 270 and dAngle < 360:
        dAngle -= 270
    fH1 = sizeDst[1] * math.sin(dAngle * math.pi / 180) * math.cos(dAngle * math.pi / 180)
    fH2 = sizeDst[0] * math.sin(dAngle * math.pi / 180) * math.cos(dAngle * math.pi / 180)
    iHalfHeight = int(math.ceil(fTopY - ptCenter[1] - fH1))
    iHalfWidth = int(math.ceil(fRightX - ptCenter[0] - fH2))
    sizeRet = (iHalfWidth * 2, iHalfHeight * 2)
    #bWrongSize = (sizeDst[1] < sizeRet[0] and sizeDst[0] > sizeRet[1]) or (sizeDst[1] > sizeRet[0] and sizeDst[0] < sizeRet[1]) or (sizeDst[1] * sizeDst[0] > sizeRet[0] * sizeRet[1])
    bWrongSize = sizeDst[0] < sizeRet[0] or sizeDst[1] < sizeRet[1]
    if bWrongSize:
        sizeRet = (int(fRightX - fLeftX + 0.5), int(fTopY - fBottomY + 0.5))
    return sizeRet

def GetNextMaxLoc(matResult, ptMaxLoc, sizeTemplate, dMaxValue, dMaxOverlap, blockMax):
    # The area to be compared needs to consider the overlap ratio
    iStartX = int(ptMaxLoc[0] - sizeTemplate[0] * (1 - dMaxOverlap))
    iStartY = int(ptMaxLoc[1] - sizeTemplate[1] * (1 - dMaxOverlap))
    # blacked out
    rectIgnore = (iStartX, iStartY, int(2 * sizeTemplate[0] * (1 - dMaxOverlap)), int(2 * sizeTemplate[1] * (1 - dMaxOverlap)))
    cv2.rectangle(matResult, rectIgnore, -1, cv2.FILLED)
    blockMax.UpdateMax(rectIgnore)
    ptReturn = blockMax.GetMaxValueLoc(dMaxValue)
    return ptReturn

def GetNextMaxLocNoBlockMax(matResult, ptMaxLoc, sizeTemplate, dMaxOverlap):
    # The area to be compared needs to consider the overlap ratio
    iStartX = int(ptMaxLoc[0] - sizeTemplate[1] * (1 - dMaxOverlap))
    iStartY = int(ptMaxLoc[1] - sizeTemplate[0] * (1 - dMaxOverlap))
    # blacked out
    rectIgnore = (iStartX, iStartY, int(2 * sizeTemplate[1] * (1 - dMaxOverlap)), int(2 * sizeTemplate[0] * (1 - dMaxOverlap)))
    cv2.rectangle(matResult, rectIgnore, -1, cv2.FILLED)
    # get the next maximum
    minVal, dMaxValue, minLoc, ptNewMaxLoc = cv2.minMaxLoc(matResult)
    return dMaxValue, ptNewMaxLoc

# https://savvycalculator.com/rotation-calculator-new-coordinates-by-rotation/
# formula to rotate coordinates around 0,0 counter-clockwise
# x’ = x * cos(θ) – y * sin(θ) 
# y’ = x * sin(θ) + y * cos(θ)
def ptRotatePt2f(ptInput, ptOrg, dAngle):
    x, y = _xy(ptInput)
    ox, oy = _xy(ptOrg)
    dWidth = ox * 2
    dHeight = oy * 2
    dY1 = dHeight - y
    dY2 = dHeight - oy
    dX = (x - ox) * math.cos(dAngle) - (dY1 - oy) * math.sin(dAngle) + ox
    dY = (x - ox) * math.sin(dAngle) + (dY1 - oy) * math.cos(dAngle) + dY2
    dY = -dY + dHeight
    return numpy.array([dX, dY], dtype=numpy.float64) 
    
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

def GetRotatedROI(matSrc, size, ptLT, dAngle):
    dAngle_radian = dAngle * math.pi / 180
    ptC = ((matSrc.shape[1] - 1) / 2.0, (matSrc.shape[0] - 1) / 2.0)
    ptLT_rotate = ptRotatePt2f(numpy.array(ptLT), numpy.array(ptC), dAngle_radian)
    sizePadding = (size[1] + 6, size[0] + 6)
    rMat = cv2.getRotationMatrix2D(ptC, dAngle, 1)
    rMat[0, 2] -= ptLT_rotate[0] - 3
    rMat[1, 2] -= ptLT_rotate[1] - 3
    matROI = cv2.warpAffine(src=matSrc, M=rMat, dsize=sizePadding)
    return matROI

def compareScoreBig2Small(lhs, rhs):
    return lhs.dMatchScore > rhs.dMatchScore

def get_top_layer(matTempl, iMinDstLength):
    iTopLayer = 0
    iMinReduceArea = iMinDstLength * iMinDstLength
    iArea = matTempl.shape[1] * matTempl.shape[0]
    while iArea > iMinReduceArea:
        iArea //= 4
        iTopLayer += 1
    return iTopLayer

def match_template(matSrc, pTemplData, matResult, iLayer, bUseSIMD):
    templ = pTemplData.vecPyramid[iLayer]
    if matSrc is None or templ is None:
        return numpy.zeros((1, 1), dtype=numpy.float32)
    if matSrc.shape[0] < templ.shape[0] or matSrc.shape[1] < templ.shape[1]:
        return numpy.zeros((1, 1), dtype=numpy.float32)
    if pTemplData.vecResultEqual1[iLayer]:
        return numpy.ones(
            (matSrc.shape[0] - templ.shape[0] + 1, matSrc.shape[1] - templ.shape[1] + 1),
            dtype=numpy.float32,
        )
    # Native TM_CCOEFF_NORMED is the same score the C++ matcher computed via
    # TM_CCORR + CCOEFF_Denominator, but runs in optimized C instead of Python.
    return cv2.matchTemplate(matSrc, templ, cv2.TM_CCOEFF_NORMED)

def filter_with_score(vec, dScore):
    vec = [x for x in vec if x.dMatchScore >= dScore]
    vec.sort(key=lambda x: x.dMatchScore, reverse=True)
    return vec

def sub_pix_estimation(vec, dNewX, dNewY, dNewAngle, dAngleStep, iMaxScoreIndex):
    matA = numpy.zeros((27, 10), dtype=numpy.float64)
    matZ = numpy.zeros((10, 1), dtype=numpy.float64)
    matS = numpy.zeros((27, 1), dtype=numpy.float64)
    dX_maxScore = vec[iMaxScoreIndex].pt[0]
    dY_maxScore = vec[iMaxScoreIndex].pt[1]
    dTheata_maxScore = vec[iMaxScoreIndex].dMatchAngle
    iRow = 0
    for theta in range(3):
        for y in range(-1, 2):
            for x in range(-1, 2):
                dX = dX_maxScore + x
                dY = dY_maxScore + y
                dT = (dTheata_maxScore + (theta - 1) * dAngleStep) * D2R
                matA[iRow, 0] = dX * dX
                matA[iRow, 1] = dY * dY
                matA[iRow, 2] = dT * dT
                matA[iRow, 3] = dX * dY
                matA[iRow, 4] = dX * dT
                matA[iRow, 5] = dY * dT
                matA[iRow, 6] = dX
                matA[iRow, 7] = dY
                matA[iRow, 8] = dT
                matA[iRow, 9] = 1.0
                matS[iRow, 0] = vec[iMaxScoreIndex + (theta - 1)].vecResult[x + 1][y + 1]
                iRow += 1
    matZ = numpy.linalg.inv(matA.T @ matA) @ matA.T @ matS
    matK1 = numpy.array([[2 * matZ[0], matZ[3], matZ[4]], [matZ[3], 2 * matZ[1], matZ[5]], [matZ[4], matZ[5], 2 * matZ[2]]])
    matK2 = numpy.array([[-matZ[6]], [-matZ[7]], [-matZ[8]]])
    matDelta = numpy.linalg.inv(matK1) @ matK2
    dNewX[0] = matDelta[0, 0]
    dNewY[0] = matDelta[1, 0]
    dNewAngle[0] = matDelta[2, 0] * R2D

def get_max_value_loc(dMax, ptMaxLoc):
    iSize = len(vecBlock)
    if iSize == 0:
        minVal, dMax, minLoc, ptMaxLoc = cv2.minMaxLoc(matSrc)
        return
    iIndex = 0
    dMax = vecBlock[0].dMax
    for i in range(1, iSize):
        if vecBlock[i].dMax >= dMax:
            iIndex = i
            dMax = vecBlock[i].dMax
    ptMaxLoc = vecBlock[iIndex].ptMaxLoc

def buildPyramid(Source, Layers):
    Pyramid = [None] * (Layers+1)
    Pyramid[0] = Source
    for i in range(Layers):
        Source = cv2.pyrDown(src=Source,borderType=cv2.BORDER_DEFAULT)
        Pyramid[i+1] = Source

    return Pyramid

def learn_pattern(m_matDst):
    TemplData = s_TemplData()
    #UpdateData (1)
    TemplData.clear()
    iTopLayer = get_top_layer(m_matDst, int(math.sqrt(float(m_iMinReduceArea))))
    TemplData.vecPyramid = buildPyramid (m_matDst, iTopLayer)
    #templData.iBorderColor = math.mean(m_matDst).val[0] < 128 ? 255 : 0;
    if cv2.mean(m_matDst)[0] < 128:
        TemplData.iBorderColor = 255
    else:
        TemplData.iBorderColor = 0
    iSize = len(TemplData.vecPyramid)
    TemplData.resize(iSize);

    for i in range(iSize):
        rows = TemplData.vecPyramid[i].shape[0]
        cols = TemplData.vecPyramid[i].shape[1]
        invArea = 1.0 / float(rows * cols)
        templMean = None
        templSdv = None
        templNorm = 0.0
        templSum2 = 0.0

        templMean, templSdv = cv2.meanStdDev(TemplData.vecPyramid[i])
        mean0 = float(templMean[0, 0])
        sdv0 = float(templSdv[0, 0])
        templNorm = sdv0 * sdv0

        if templNorm < DBL_EPSILON:
            TemplData.vecResultEqual1[i] = True

        templSum2 = templNorm + mean0 * mean0
        templSum2 /= invArea
        templNorm = math.sqrt(templNorm)
        templNorm /= math.sqrt(invArea)
        
        TemplData.vecInvArea[i] = invArea
        TemplData.vecTemplMean[i][0] = mean0
        TemplData.vecTemplNorm[i] = templNorm
	
        TemplData.bIsPatternLearned = True

    return TemplData

def overlay_images(top_image, bottom_image, origin):
    # get shape of top image
    h,  w = top_image.shape[:2]
    result_image = bottom_image.copy()
    # replace values at coordinates
    result_image[origin[0]:origin[0]+h, origin[1]:origin[1]+w] = top_image[0:h, 0:w]
    return result_image

def _coarse_match_angle(srcTop, pTemplData, iTopLayer, angle, ptCenter, iMaxPos, dMaxOverlap, minScore):
    sizeBest = GetBestRotationSize(srcTop.shape, pTemplData.vecPyramid[iTopLayer].shape, angle)
    fTranslationY = (sizeBest[0] - 1) / 2.0 - ptCenter[1]
    fTranslationX = (sizeBest[1] - 1) / 2.0 - ptCenter[0]
    matR = cv2.getRotationMatrix2D(ptCenter, angle, 1)
    matR[0, 2] += fTranslationX
    matR[1, 2] += fTranslationY
    border = int(pTemplData.iBorderColor)
    matRotatedSrc = cv2.warpAffine(
        srcTop,
        matR,
        (sizeBest[1], sizeBest[0]),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(border, border, border),
    )
    matResult = match_template(matRotatedSrc, pTemplData, None, iTopLayer, False)
    if matResult is None or matResult.size == 0:
        return []
    _minVal, dMaxVal, minLoc, ptMaxLoc = cv2.minMaxLoc(matResult)
    if dMaxVal < minScore:
        return []
    peaks = [s_MatchParameter((ptMaxLoc[0] - fTranslationX, ptMaxLoc[1] - fTranslationY), dMaxVal, angle)]
    nExtra = max(0, iMaxPos + MATCH_CANDIDATE_NUM - 1)
    sizeTemplate = pTemplData.vecPyramid[iTopLayer].shape
    for _ in range(nExtra):
        dValue, ptMaxLoc = GetNextMaxLocNoBlockMax(matResult, ptMaxLoc, sizeTemplate, dMaxOverlap)
        if dValue < minScore:
            break
        peaks.append(s_MatchParameter((ptMaxLoc[0] - fTranslationX, ptMaxLoc[1] - fTranslationY), dValue, angle))
    return peaks
    
######################################################################################################################################################

def main(m_matSrc, m_matDst, savelocation, iMaxPos, dMaxOverlap, dScore, dToleranceAngle, pixel_ratio, rotation_offset, origin, roi_top_left, roi_bot_right, debug):
    global m_vecSingleTargetData
    m_vecSingleTargetData = []

    if m_matSrc is None or m_matDst is None:
        return False

    m_TemplData = learn_pattern(m_matDst)
    
    # make a copy of just the region of interest
    m_matRoi = crop_image(m_matSrc, roi_top_left, roi_bot_right)

    #cv2.imshow("m_matRoi", m_matRoi)
    #cv2.waitKey(0)

    #if m_matRoi.empty() or m_matDst.empty():
     #   return False
    #if (m_matDst.cols < m_matRoi.cols and m_matDst.rows > m_matRoi.rows) or (m_matDst.cols > m_matRoi.cols and m_matDst.rows < m_matRoi.rows):
    #    return False
    #if m_matDst.size().area() > m_matRoi.size().area():
    #    return False
    if not m_TemplData.bIsPatternLearned:
       return False

    #Determine the number of pyramid layers a total of 1 + iLayer layer
    iTopLayer = get_top_layer(m_matDst, int(math.sqrt(float(m_iMinReduceArea))))

    #build a pyramid
    #vecMatSrcPyr = cv2.Mat
    vecMatSrcPyr = numpy.array([])
    if m_ckBitwiseNot:
        ############################
        # TODO this path not tested
        ############################
        matNewSrc = 255 - m_matRoi
        vecMatSrcPyr = buildPyramid(matNewSrc, iTopLayer)
        #cv2.imshow("matNewSrc", matNewSrc)
        #cv2.moveWindow("1", 0, 0)
    else:
        #cv2.imshow("m_matRoi", m_matRoi)
        #cv2.waitKey(0)
        vecMatSrcPyr = buildPyramid(m_matRoi, iTopLayer)
        #cv2.imshow("vecMatSrcPyr", vecMatSrcPyr)
        #cv2.waitKey(0)

    pTemplData = m_TemplData

    #The first stage uses the topmost layer to find out the approximate angle and ROI
    dAngleStep = math.atan(2.0 / max(pTemplData.vecPyramid[iTopLayer].shape[0],pTemplData.vecPyramid[iTopLayer].shape[1])) * R2D
    vecAngles = []
    if m_bToleranceRange:
        ############################
        # TODO this path not tested
        ############################
        if m_dTolerance1 >= m_dTolerance2 or m_dTolerance3 >= m_dTolerance4:
            #messagebox.showerror("Error", "The angle range setting is abnormal, the left value must be smaller than the right value")
            return False
        for dAngle in numpy.arange(m_dTolerance1, m_dTolerance2 + dAngleStep, dAngleStep):
            vecAngles.append(dAngle)
        for dAngle in numpy.arange(m_dTolerance3, m_dTolerance4 + dAngleStep, dAngleStep):
            vecAngles.append(dAngle)
    else:
        if dToleranceAngle < VISION_TOLERANCE:
            vecAngles.append(0.0)
        else:
            for dAngle in numpy.arange(0, dToleranceAngle + dAngleStep, dAngleStep):
                vecAngles.append(dAngle)
            for dAngle in numpy.arange(-dAngleStep, -dToleranceAngle - dAngleStep, -dAngleStep):
                vecAngles.append(dAngle)

    iTopSrcW = vecMatSrcPyr[iTopLayer].shape[1]
    iTopSrcH = vecMatSrcPyr[iTopLayer].shape[0]
    ptCenter = ((iTopSrcW - 1) / 2.0, (iTopSrcH - 1) / 2.0)

    vecLayerScore = [dScore] * (iTopLayer + 1)
    for iLayer in range(1, iTopLayer + 1):
        vecLayerScore[iLayer] = vecLayerScore[iLayer - 1] * 0.9

    srcTop = vecMatSrcPyr[iTopLayer]
    minScoreTop = vecLayerScore[iTopLayer]
    vecMatchParameter = []

    def _run_coarse_sequential():
        found = []
        for angle in vecAngles:
            found.extend(
                _coarse_match_angle(srcTop, pTemplData, iTopLayer, angle, ptCenter, iMaxPos, dMaxOverlap, minScoreTop)
            )
        return found

    workers = min(4, os.cpu_count() or 1, max(1, len(vecAngles)))
    if workers <= 1 or len(vecAngles) < 4:
        vecMatchParameter = _run_coarse_sequential()
    else:
        # Raspberry Pi OpenCV/OpenBLAS already uses multiple cores per
        # matchTemplate. Limit OpenCV to one thread while we parallelize angles
        # so the Pi is not oversubscribed.
        prev_threads = 0
        try:
            prev_threads = cv2.getNumThreads()
            cv2.setNumThreads(1)
        except Exception:
            prev_threads = 0
        try:
            with ThreadPoolExecutor(max_workers=workers) as pool:
                jobs = [
                    pool.submit(
                        _coarse_match_angle, srcTop, pTemplData, iTopLayer, angle, ptCenter, iMaxPos, dMaxOverlap, minScoreTop
                    )
                    for angle in vecAngles
                ]
                for job in jobs:
                    vecMatchParameter.extend(job.result())
        except Exception:
            vecMatchParameter = _run_coarse_sequential()
        finally:
            if prev_threads:
                try:
                    cv2.setNumThreads(prev_threads)
                except Exception:
                    pass

    vecMatchParameter.sort(key=lambda x: x.dMatchScore, reverse=True)
    iSearchSize = min(iMaxPos + MATCH_CANDIDATE_NUM, len(vecMatchParameter))
    iDstW = pTemplData.vecPyramid[iTopLayer].shape[1]
    iDstH = pTemplData.vecPyramid[iTopLayer].shape[0]

    iStopLayer = 0
    vecAllResult = []
    for i in range(iSearchSize):
        dRAngle = -vecMatchParameter[i].dMatchAngle * D2R
        ptLT = ptRotatePt2f(vecMatchParameter[i].pt, ptCenter, dRAngle)
        dAngleStep = math.atan(2.0 / max(iDstW, iDstH)) * R2D
        vecMatchParameter[i].dAngleStart = vecMatchParameter[i].dMatchAngle - dAngleStep
        vecMatchParameter[i].dAngleEnd = vecMatchParameter[i].dMatchAngle + dAngleStep
        if iTopLayer <= iStopLayer:
            vecMatchParameter[i].pt = ptLT * (1.0 if iTopLayer == 0 else 2.0)
            vecAllResult.append(vecMatchParameter[i])
        else:
            for iLayer in range(iTopLayer - 1, iStopLayer - 1, -1):
                dAngleStep = math.atan(2.0 / max(pTemplData.vecPyramid[iLayer].shape)) * R2D
                dMatchedAngle = vecMatchParameter[i].dMatchAngle
                if dToleranceAngle < VISION_TOLERANCE:
                    refine_angles = [0.0]
                else:
                    refine_angles = [dMatchedAngle + dAngleStep * j for j in range(-1, 2)]

                ptSrcCenter = ((vecMatSrcPyr[iLayer].shape[1] - 1) / 2.0, (vecMatSrcPyr[iLayer].shape[0] - 1) / 2.0)
                vecNewMatchParameter = []
                iMaxScoreIndex = 0
                dBigValue = -1
                for j, ang in enumerate(refine_angles):
                    matRotatedSrc = GetRotatedROI(vecMatSrcPyr[iLayer], pTemplData.vecPyramid[iLayer].shape, ptLT * 2, ang)
                    matResult = match_template(matRotatedSrc, pTemplData, None, iLayer, False)
                    minVal, dMaxValue, minLoc, ptMaxLoc = cv2.minMaxLoc(matResult)
                    cand = s_MatchParameter(ptMaxLoc, dMaxValue, ang)
                    if bSubPixelEstimation:
                        if ptMaxLoc[0] == 0 or ptMaxLoc[1] == 0 or ptMaxLoc[0] == matResult.shape[1] - 1 or ptMaxLoc[1] == matResult.shape[0] - 1:
                            cand.bPosOnBorder = True
                        if not cand.bPosOnBorder:
                            for y in range(-1, 2):
                                for x in range(-1, 2):
                                    cand.vecResult[y + 1][x + 1] = matResult[ptMaxLoc[1] + x, ptMaxLoc[0] + y]
                    vecNewMatchParameter.append(cand)
                    if cand.dMatchScore > dBigValue:
                        iMaxScoreIndex = j
                        dBigValue = cand.dMatchScore

                if vecNewMatchParameter[iMaxScoreIndex].dMatchScore < vecLayerScore[iLayer]:
                    break
                if bSubPixelEstimation and iLayer == 0 and not vecNewMatchParameter[iMaxScoreIndex].bPosOnBorder and iMaxScoreIndex != 0 and iMaxScoreIndex != 2:
                    dNewX, dNewY, dNewAngle = [0], [0], [0]
                    sub_pix_estimation(vecNewMatchParameter, dNewX, dNewY, dNewAngle, dAngleStep, iMaxScoreIndex)
                    vecNewMatchParameter[iMaxScoreIndex].pt = numpy.array([dNewX[0], dNewY[0]], dtype=numpy.float64)
                    vecNewMatchParameter[iMaxScoreIndex].dMatchAngle = dNewAngle[0]

                dNewMatchAngle = vecNewMatchParameter[iMaxScoreIndex].dMatchAngle
                ptPaddingLT = ptRotatePt2f(ptLT * 2, ptSrcCenter, dNewMatchAngle * D2R) - numpy.array([3.0, 3.0])
                pt = numpy.array([
                    vecNewMatchParameter[iMaxScoreIndex].pt[0] + ptPaddingLT[0],
                    vecNewMatchParameter[iMaxScoreIndex].pt[1] + ptPaddingLT[1],
                ], dtype=numpy.float64)
                pt = ptRotatePt2f(pt, ptSrcCenter, -dNewMatchAngle * D2R)
                if iLayer == iStopLayer:
                    vecNewMatchParameter[iMaxScoreIndex].pt = pt * (1.0 if iStopLayer == 0 else 2.0)
                    vecAllResult.append(vecNewMatchParameter[iMaxScoreIndex])
                else:
                    vecMatchParameter[i].dMatchAngle = dNewMatchAngle
                    vecMatchParameter[i].dAngleStart = vecMatchParameter[i].dMatchAngle - dAngleStep / 2
                    vecMatchParameter[i].dAngleEnd = vecMatchParameter[i].dMatchAngle + dAngleStep / 2
                    ptLT = pt

    vecAllResult = filter_with_score(vecAllResult, dScore)

    #Finally filter out overlapping
    iDstW = pTemplData.vecPyramid[iStopLayer].shape[1]
    iDstH = pTemplData.vecPyramid[iStopLayer].shape[0]
    for i in range(len(vecAllResult)):
        ptLT, ptRT, ptRB, ptLB = numpy.zeros((4, 2))
        dRAngle = -vecAllResult[i].dMatchAngle * D2R
        ptLT = vecAllResult[i].pt
        ptRT = numpy.array([ptLT[0] + iDstW * numpy.cos(dRAngle), ptLT[1] - iDstW * numpy.sin(dRAngle)])
        ptLB = numpy.array([ptLT[0] + iDstH * numpy.sin(dRAngle), ptLT[1] + iDstH * numpy.cos(dRAngle)])
        ptRB = numpy.array([ptRT[0] + iDstH * numpy.sin(dRAngle), ptRT[1] + iDstH * numpy.cos(dRAngle)])

        # record rotated rectangle
        ptRectCenter = ((ptLT[0] + ptRT[0] + ptLB[0] + ptRB[0]) / 4.0, (ptLT[1] + ptRT[1] + ptLB[1] + ptRB[1]) / 4.0)
        
        # https://stackoverflow.com/questions/18207181/opencv-python-draw-minarearect-rotatedrect-not-implemented
        vecAllResult[i].rectR = (ptRectCenter, pTemplData.vecPyramid[iStopLayer].shape, vecAllResult[i].dMatchAngle)

    vecAllResult = FilterWithRotatedRect(vecAllResult, cv2.TM_CCOEFF_NORMED, dMaxOverlap)
    #Finally filter out overlapping

    vecAllResult.sort(key=lambda x: x.dMatchScore, reverse=True)

    iMatchSize = len(vecAllResult)
    if iMatchSize <= 0:
        return False

    iW, iH = pTemplData.vecPyramid[0].shape[::-1]
    for i in range(iMatchSize):
        sstm = s_SingleTargetMatch()
        dRAngle = -vecAllResult[i].dMatchAngle * D2R
        
        sstm.ptLT = numpy.array([vecAllResult[i].pt[0],vecAllResult[i].pt[1]])
        sstm.ptRT = numpy.array([sstm.ptLT[0] + iW * numpy.cos(dRAngle), sstm.ptLT[1] - iW * numpy.sin(dRAngle)])
        sstm.ptLB = numpy.array([sstm.ptLT[0] + iH * numpy.sin(dRAngle), sstm.ptLT[1] + iH * numpy.cos(dRAngle)])
        sstm.ptRB = numpy.array([sstm.ptRT[0] + iH * numpy.sin(dRAngle), sstm.ptRT[1] + iH * numpy.cos(dRAngle)])
        sstm.ptCenter = numpy.array([(sstm.ptLT[0] + sstm.ptRT[0] + sstm.ptRB[0] + sstm.ptLB[0]) / 4, (sstm.ptLT[1] + sstm.ptRT[1] + sstm.ptRB[1] + sstm.ptLB[1]) / 4])

        sstm.dMatchedAngle = -vecAllResult[i].dMatchAngle
        sstm.dMatchScore = vecAllResult[i].dMatchScore
        if sstm.dMatchedAngle < -180:
            sstm.dMatchedAngle += 360
        if sstm.dMatchedAngle > 180:
            sstm.dMatchedAngle -= 360
        m_vecSingleTargetData.append(sstm)
        if i + 1 == iMaxPos:
            break

    floatPrecision = 2
    numpy.set_printoptions(precision=floatPrecision)
    roi_x, roi_y = _xy(roi_top_left)
    ox, oy = _xy(origin)
    origin_xy = numpy.array([ox, oy], dtype=numpy.float64)
    for i in range(len(m_vecSingleTargetData)):
        cx, cy = _xy(m_vecSingleTargetData[i].ptCenter)
        ptCenterWithRoi = numpy.array([cx + roi_x, cy + roi_y], dtype=numpy.float64)
        trans_XY = ptRotatePt2f(ptCenterWithRoi, origin_xy, math.radians(float(rotation_offset)))
        trans_X = (trans_XY[0] - ox) / pixel_ratio
        trans_Y = (trans_XY[1] - oy) / pixel_ratio

        result = "LOC "
        result = result + "obj:" + str(i) + " "
        result = result + "cx:" + str(round(trans_X,floatPrecision)) + " "
        result = result + "cy:" + str(round(trans_Y,floatPrecision)) + " "
        result = result + "a:" + str(round(m_vecSingleTargetData[i].dMatchedAngle,floatPrecision)) + " "
        result = result + "s:" + str(round(m_vecSingleTargetData[i].dMatchScore,floatPrecision)) + " "
        print(result)
    
    markedRoi = RefreshSrcView(m_matRoi, m_matDst, m_vecSingleTargetData, pixel_ratio)
    cv2.imwrite(savelocation, markedRoi)

# testing command line
# python FastTemplateMatching.py -s outputimage.bmp -t Template.jpg -i 5 -j 0.0 -k 0.8 -l 90.0 -d True
if __name__ == "__main__":
    # fetch all the arguments from sys.argv except the script name
    argv = sys.argv[1:]

    # get option and value pair from getopt
    try:
        opts,argv = getopt.getopt(argv, "s:t:w:h:i:j:k:l:d:", ["savelocation =","templatelocation =","width =","height =","iMaxPos =","dMaxOverlap =","dScore =","dToleranceAngle =","debug ="])
    except:
        print('incorrect arguments')

    # set defaults
    savelocation = '/var/opt/codesys/PlcLogic/visu/outputimage.jpg'
    templatelocation = 'Template.bmp'
    width = 640
    height = 400
    iMaxPos = 2
    dMaxOverlap = 0.0
    dScore = 0.6
    dToleranceAngle = 90.0
    debug = False

    for o,v in opts:
        if o in ['-s','--savelocation']:
            savelocation = v
        elif o in ['-t','--templatelocation']:
            templatelocation = v
        elif o in ['-w','--width']:
            width = int(v)
        elif o in ['-h','--height']:
            height = int(v)
        elif o in ['-i','--iMaxPos']:
            iMaxPos = int(v)
        elif o in ['-j','--dMaxOverlap']:
            dMaxOverlap = float(v)
        elif o in ['-k','--dScore']:
            dScore = float(v)
        elif o in ['-l','--dToleranceAngle']:
            dToleranceAngle = float(v)
        elif o in ['-d','--debug']:
            debug = str_to_bool(v)
    
    # load the calibration data
    dir_path = os.path.dirname(os.path.realpath(__file__))
    cal_file_path = str(os.path.join(dir_path, 'cal.yaml'))
    checkerboard, squaresize, mtx, dist, pixel_ratio, rotation_offset, top_left, bot_right, origin = load_cal_data(cal_file_path)
    
    # load the roi data
    dir_path = os.path.dirname(os.path.realpath(__file__))
    roi_file_path = str(os.path.join(dir_path, 'roi.yaml'))
    roi_top_left, roi_bot_right = load_roi_data(roi_file_path)

    # capture the image
    image = capture_image(debug, width, height)
    
    # filter image through calibration data
    if debug == False:
        image = filter_image(image, mtx, dist, False)
        #image = rotate_image(image, rotation_offset)
        #image = crop_image(image, top_left, bot_right)
 
    # read in the template
    template = cv2.imread(templatelocation, cv2.IMREAD_GRAYSCALE)

    #
    main(image, template, savelocation, iMaxPos, dMaxOverlap, dScore, dToleranceAngle, pixel_ratio, rotation_offset, origin, roi_top_left, roi_bot_right, debug)

    #
    #print("time:" + str(datetime.datetime.now()))