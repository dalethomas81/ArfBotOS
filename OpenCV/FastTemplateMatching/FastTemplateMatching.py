import math
import numpy
import cv2
import getopt
import sys
import os
import datetime
import time

# fundamental constants
CV_PI   = 3.1415926535897932384626433832795
CV_2PI  = 6.283185307179586476925286766559
CV_LOG2 = 0.69314718055994530941723212145818
DBL_EPSILON = 2.2204460492503131e-016 # smallest such that 1.0+DBL_EPSILON != 1.0

VISION_TOLERANCE = 0.0000001
D2R = (CV_PI / 180.0)
R2D = (180.0 / CV_PI)
MATCH_CANDIDATE_NUM = 5

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

def draw_line(img,pt1,pt2,color = colorGreen,thickness=1,style='dotted',gap=1):
    if gap <= 0:
        gap = 1

    if thickness < 1:
        thickness = 1

    dist =((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)**.5
    pts= []
    for i in  numpy.arange(0,dist,gap):
        r=i/dist
        x=int((pt1[0]*(1-r)+pt2[0]*r)+.5)
        y=int((pt1[1]*(1-r)+pt2[1]*r)+.5)
        p = (x,y)
        pts.append(p)

    if style=='dotted':
        for p in pts:
            img = cv2.circle(img,p,int(thickness),color,-1)
    else:
        s=pts[0]
        e=pts[0]
        i=0
        for p in pts:
            s=e
            e=p
            if i%2==1:
                img = cv2.line(img,s,e,color,int(thickness))
            i+=1
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
        matResult = numpy.ones(matResult.shape, dtype=numpy.float32)
        return

    sum, sqsum = cv2.integral2(matSrc, sdepth=cv2.CV_64F, sqdepth=cv2.CV_64F)

    vecPyramid_rows = pTemplData.vecPyramid[iLayer].shape[0]
    vecPyramid_cols = pTemplData.vecPyramid[iLayer].shape[1]
    sqsum_step = sqsum.shape[1]
    sum_step = sum.shape[1]

    _q0 = numpy.array(sqsum[0:, 0:])
    q0 = _q0.flatten()
    q1 = q0[vecPyramid_cols:]
    q2 = q0[vecPyramid_rows * sqsum_step:]
    q3 = q2[vecPyramid_cols:]
    
    _p0 = numpy.array(sum[0:, 0:])
    p0 = _p0.flatten()
    p1 = p0[vecPyramid_cols:]
    p2 = p0[vecPyramid_rows * sum_step:]
    p3 = p2[vecPyramid_cols:]
    
    dTemplMean0 = pTemplData.vecTemplMean[iLayer][0]
    dTemplNorm = pTemplData.vecTemplNorm[iLayer]
    dInvArea = pTemplData.vecInvArea[iLayer]
    #_matResult = matResult.flatten()
    for i in range(matResult.shape[0]):
        rrow = matResult[i]
        idx = i * sum.shape[1]
        idx2 = i * sqsum.shape[1]
        for j in range(matResult.shape[1]):
            try:
                num = rrow[j]
                # X + (Y*W)
                t = p0[idx] - p1[idx] - p2[idx] + p3[idx]
                wndMean2 = t * t
                num -= t * dTemplMean0
                wndMean2 *= dInvArea
                t = q0[idx2] - q1[idx2] - q2[idx2] + q3[idx2]
                wndSum2 = t
                diff2 = max(wndSum2 - wndMean2, 0)
                if diff2 <= min(0.5, 10 * numpy.finfo(float).eps * wndSum2):
                    t = 0
                else:
                    t = numpy.sqrt(diff2) * dTemplNorm
                if abs(num) < t:
                    num /= t
                elif abs(num) < t * 1.125:
                    num = 1 if num > 0 else -1
                else:
                    num = 0
                rrow[j] = num

                idx += 1
                idx2 += 1

            except:
                pass

    return matResult

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
    dashGap = 5.4 * m_dNewScale * boxRatio
    solidLineThickness = 0.9 * m_dNewScale * boxRatio
    dashLineThickness = 1.35 * m_dNewScale * boxRatio
    fontScale = 0.675 * m_dNewScale * boxRatio
    fontthickness = int(0.9 * m_dNewScale * boxRatio)
    markCrossLength = 4.5 * m_dNewScale  * boxRatio
    srcRows = matSrc.shape[0]
    srcCols = matSrc.shape[1]
    dstRows = matDst.shape[0]
    dstCols = matDst.shape[1]
    #matResize = cv2.Mat
    #matColorSrc = cv2.Mat
    size = (int(m_dNewScale * srcCols), int(m_dNewScale * srcRows))
    matColorSrc = cv2.cvtColor(src=matSrc, code=cv2.COLOR_GRAY2BGR)
    matResize = cv2.resize(src=matColorSrc, dsize=size)
    #iPosX = m_hScrollBar.GetScrollPos()
    #iPosY = m_vScrollBar.GetScrollPos()
    iW = int(srcCols * m_dSrcScale)
    iH = int(srcRows * m_dSrcScale)
    #rectShow = cv2.Rect(cv2.Point(iPosX, iPosY), cv2.Size(iW, iH))
    iSize = len(vecSingleTargetData)
    if m_bShowResult:
        for i in range(iSize):
            ptLT = vecSingleTargetData[i].ptLT * m_dNewScale
            ptLB = vecSingleTargetData[i].ptLB * m_dNewScale
            ptRB = vecSingleTargetData[i].ptRB * m_dNewScale
            ptRT = vecSingleTargetData[i].ptRT * m_dNewScale
            ptC = vecSingleTargetData[i].ptCenter * m_dNewScale
            
            # draw rectangles
            matResize = draw_line(img=matResize,pt1=ptLT,pt2=ptLB,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize,pt1=ptLB,pt2=ptRB,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize,pt1=ptRB,pt2=ptRT,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize,pt1=ptRT,pt2=ptLT,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)

            matResize = draw_line(img=matResize,pt1=ptLT,pt2=ptLB,color=colorRed,thickness=dashLineThickness,style='dotted',gap=dashGap)
            matResize = draw_line(img=matResize,pt1=ptLB,pt2=ptRB,color=colorRed,thickness=dashLineThickness,style='dotted',gap=dashGap)
            matResize = draw_line(img=matResize,pt1=ptRB,pt2=ptRT,color=colorRed,thickness=dashLineThickness,style='dotted',gap=dashGap)
            matResize = draw_line(img=matResize,pt1=ptRT,pt2=ptLT,color=colorRed,thickness=dashLineThickness,style='dotted',gap=dashGap)
            
            # draw corners
            ptDis1 = numpy.array([0,0])
            ptDis2 = numpy.array([0,0])
            if dstCols > dstRows:
                ptDis1 = (ptLB - ptLT) / 3
                ptDis2 = (ptRT - ptLT) / 3 * (dstRows / float(dstCols))
            else:
                ptDis1 = (ptLB - ptLT) / 3 * (dstCols / float(dstRows))
                ptDis2 = (ptRT - ptLT) / 3
            matResize = draw_line(img=matResize, pt1=ptLT, pt2=ptLT + ptDis1 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptLT, pt2=ptLT + ptDis2 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptRT, pt2=ptRT + ptDis1 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptRT, pt2=ptRT - ptDis2 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptRB, pt2=ptRB - ptDis1 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptRB, pt2=ptRB - ptDis2 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptLB, pt2=ptLB - ptDis1 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)
            matResize = draw_line(img=matResize, pt1=ptLB, pt2=ptLB + ptDis2 / 2,color=colorGreen,thickness=solidLineThickness,style='none',gap=0)

            # draw corner line
            matResize = draw_line(img=matResize, pt1=ptLT + ptDis1, pt2=ptLT + ptDis2, color=colorGreen, thickness=solidLineThickness, style='none', gap=0)
            matResize = draw_line(img=matResize, pt1=ptLT + ptDis1, pt2=ptLT + ptDis2, color=colorRed, thickness=dashLineThickness, style='dotted', gap=dashGap)

            # matDraw, iX, iY, iLength, color, iThickness
            matResize = draw_mark_cross(matResize, ptC[0], ptC[1], markCrossLength, colorGreen, solidLineThickness)

            str = f"{i}"
            _ptText = (ptLT + ptRT) / 2
            _ptText_i = (numpy.rint(_ptText)).astype(int)
            matResize = cv2.putText(img=matResize, text=str, org=(_ptText_i[0],_ptText_i[1]), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=fontScale, color=colorGreen, thickness=fontthickness)

    #cv2.namedWindow("SrcView", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("SrcView", int(size[0]), int(size[1]))
    ##cv2.imshow("SrcView", matResize(rectShow))
    #cv2.imshow("SrcView", matResize)
    #cv2.waitKey(0)

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
    else:
        print("Unknown")
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
# x’ = x * cos(θ) – y * sin(θ) y’ = x * sin(θ) + y * cos(θ)
def ptRotatePt2f(ptInput, ptOrg, dAngle):
    dWidth = ptOrg[0] * 2
    dHeight = ptOrg[1] * 2
    dY1 = dHeight - ptInput[1]
    dY2 = dHeight - ptOrg[1]
    dX = (ptInput[0] - ptOrg[0]) * math.cos(dAngle) - (dY1 - ptOrg[1]) * math.sin(dAngle) + ptOrg[0]
    dY = (ptInput[0] - ptOrg[0]) * math.sin(dAngle) + (dY1 - ptOrg[1]) * math.cos(dAngle) + dY2
    dY = -dY + dHeight
    return numpy.array([dX, dY])

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
    if bUseSIMD:
        matResult = numpy.zeros((matSrc.shape[0] - pTemplData.vecPyramid[iLayer].shape[0] + 1, matSrc.shape[1] - pTemplData.vecPyramid[iLayer].shape[1] + 1), dtype=numpy.float32)
        matTemplate = pTemplData.vecPyramid[iLayer]
        t_r_end = matTemplate.shape[0]
        for r in range(matResult.shape[0]):
            r_matResult = matResult[r, :]
            r_source = matSrc[r, :]
            for c in range(matResult.shape[1]):
                r_template = matTemplate
                r_sub_source = r_source
                for t_r in range(t_r_end):
                    r_matResult[c] += IM_Conv_SIMD(r_template, r_sub_source, matTemplate.shape[1])
                    r_template += matTemplate.shape[1]
                    r_sub_source += matSrc.shape[1]
    else:
        #cv2.imshow("1", matSrc)
        #cv2.waitKey(0)
        #cv2.imshow("1", pTemplData.vecPyramid[iLayer])
        #cv2.waitKey(0)
        matResult = cv2.matchTemplate(image=matSrc, templ=pTemplData.vecPyramid[iLayer],  method=cv2.TM_CCORR)
    
    # the hell does this do?
    matResult = CCOEFF_Denominator(matSrc, pTemplData, matResult, iLayer)

    return matResult

def filter_with_score(vec, dScore):
    vec.sort(key=lambda x: x.dMatchScore, reverse=True)
    iSize = len(vec)
    iIndexDelete = iSize + 1
    for i in range(iSize):
        if vec[i].dMatchScore < dScore:
            iIndexDelete = i
            break
    if iIndexDelete == iSize + 1:
        return
    vecFiltered = vec[:iIndexDelete]
    return vecFiltered

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
        # TODO convert to for loop the size of array so we can use different image imputs
        templNorm = templSdv[0] * templSdv[0]# + templSdv[1] * templSdv[1] + templSdv[2] * templSdv[2] + templSdv[3] * templSdv[3]
        #for sdvIdx in range(len(templSdv)):
        #    templNorm = templNorm + templSdv[sdvIdx] * templSdv[sdvIdx]

        if templNorm < DBL_EPSILON:
            TemplData.vecResultEqual1[i] = True

        # TODO convert to for loop the size of array so we can use different image imputs
        templSum2 = templNorm + templMean[0] * templMean[0]# + templMean[1] * templMean[1] + templMean[2] * templMean[2] + templMean[3] * templMean[3]
        #templSum2 = templNorm
        #for meanIdx in range(len(templMean)):
        #    templSum2 = templSum2 + templMean[meanIdx] * templMean[meanIdx]
            
        templSum2 /= invArea
        templNorm = math.sqrt(templNorm)
        templNorm /= math.sqrt(invArea) # care of accuracy here
        
        TemplData.vecInvArea[i] = invArea
        for j in range(len(templMean)):
            TemplData.vecTemplMean[i][j] = templMean[j]
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
    
######################################################################################################################################################

def main(m_matSrc, m_matDst, savelocation, iMaxPos, dMaxOverlap, dScore, dToleranceAngle, pixel_ratio, rotation_offset, origin, roi_top_left, roi_bot_right, debug):
    
    #cv2.imshow("m_matDst", m_matDst)
    #cv2.waitKey(0)
    #cv2.imshow("m_matSrc", m_matSrc)
    #cv2.waitKey(0)

    #
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
        cv2.imshow("1", matNewSrc)
        cv2.moveWindow("1", 0, 0)
    else:
        #cv2.imshow("1", m_matRoi)
        #cv2.waitKey(0)
        vecMatSrcPyr = buildPyramid(m_matRoi, iTopLayer)
        #cv2.imshow("1", vecMatSrcPyr)
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
    #vector<s_MatchParameter> vecMatchParameter (iSize * (m_iMaxPos + MATCH_CANDIDATE_NUM));
    iSize = len(vecAngles)

    vecMatchParameter = [s_MatchParameter()]
    #Caculate lowest score at every layer
    vecLayerScore = [dScore] * (iTopLayer + 1)
    for iLayer in range(1, iTopLayer + 1):
        vecLayerScore[iLayer] = vecLayerScore[iLayer - 1] * 0.9

    sizePat = pTemplData.vecPyramid[iTopLayer].shape
    sizePatArea = sizePat[0] * sizePat[1]
    bCalMaxByBlock = (vecMatSrcPyr[iTopLayer].size / sizePatArea > 500) and (iMaxPos > 10)

    for i in range(iSize):
        matRotatedSrc = numpy.zeros_like(vecMatSrcPyr[iTopLayer])
        matR = cv2.getRotationMatrix2D(ptCenter, vecAngles[i], 1)
        matResult = numpy.zeros_like(vecMatSrcPyr[iTopLayer])
        ptMaxLoc = (0, 0)
        dValue = 0
        dMaxVal = 0
        sizeBest = GetBestRotationSize(vecMatSrcPyr[iTopLayer].shape, pTemplData.vecPyramid[iTopLayer].shape, vecAngles[i])
        fTranslationY = (sizeBest[0] - 1) / 2.0 - ptCenter[1]
        fTranslationX = (sizeBest[1] - 1) / 2.0 - ptCenter[0]
        matR[0, 2] += fTranslationX
        matR[1, 2] += fTranslationY

        matRotatedSrc = cv2.warpAffine(src=vecMatSrcPyr[iTopLayer], 
                       dst=matRotatedSrc, 
                       M=matR, 
                       dsize=(sizeBest[1],sizeBest[0]), 
                       flags=cv2.INTER_LINEAR, 
                       borderMode=cv2.BORDER_CONSTANT, 
                       borderValue=(pTemplData.iBorderColor, pTemplData.iBorderColor, pTemplData.iBorderColor))
        
        matResult = match_template(matRotatedSrc, pTemplData, matResult, iTopLayer, False)
        
        if bCalMaxByBlock:
            ############################
            # TODO this path not tested
            ############################
            blockMax = s_BlockMax(matResult, pTemplData.vecPyramid[iTopLayer].shape)
            blockMax.GetMaxValueLoc(dMaxVal, ptMaxLoc)
            if dMaxVal < vecLayerScore[iTopLayer]:
                continue
            vecMatchParameter.append(s_MatchParameter((ptMaxLoc[0] - fTranslationX, ptMaxLoc[1] - fTranslationY), dMaxVal, vecAngles[i]))
            for j in range(iMaxPos + MATCH_CANDIDATE_NUM - 1):
                ptMaxLoc = GetNextMaxLoc(matResult, ptMaxLoc, pTemplData.vecPyramid[iTopLayer].shape, dValue, dMaxOverlap, blockMax)
                if dMaxVal < vecLayerScore[iTopLayer]:
                    continue
                vecMatchParameter.append(s_MatchParameter((ptMaxLoc[0] - fTranslationX, ptMaxLoc[1] - fTranslationY), dValue, vecAngles[i]))
        else:
            _minVal, dMaxVal, minLoc, ptMaxLoc = cv2.minMaxLoc(matResult)
            if dMaxVal < vecLayerScore[iTopLayer]:
                continue
            vecMatchParameter.append(s_MatchParameter((ptMaxLoc[0] - fTranslationX, ptMaxLoc[1] - fTranslationY), dMaxVal, vecAngles[i]))
            for j in range(iMaxPos + MATCH_CANDIDATE_NUM - 1):
                dValue, ptMaxLoc = GetNextMaxLocNoBlockMax(matResult, ptMaxLoc, pTemplData.vecPyramid[iTopLayer].shape, dMaxOverlap)
                if dMaxVal < vecLayerScore[iTopLayer]:
                    continue
                vecMatchParameter.append(s_MatchParameter((ptMaxLoc[0] - fTranslationX, ptMaxLoc[1] - fTranslationY), dValue, vecAngles[i]))

    vecMatchParameter.sort(key=lambda x: x.dMatchScore, reverse=True)

    ###############################################################
    # TODO iMatchSize comes up short when comparing to c++ version
    ###############################################################
    iMatchSize = len(vecMatchParameter)
    iDstW = pTemplData.vecPyramid[iTopLayer].shape[1]
    iDstH = pTemplData.vecPyramid[iTopLayer].shape[0]
    #end of first phase

    iStopLayer = 0
    #int iSearchSize = min (iMaxPos + MATCH_CANDIDATE_NUM, (int)vecMatchParameter.size ());//It may not be necessary to search all of them, it is too time-consuming
    vecAllResult = [s_MatchParameter()]
    for i in range(iMatchSize):
    #for (int i = 0; i < iSearchSize; i++)
        dRAngle = -vecMatchParameter[i].dMatchAngle * D2R
        ptLT = ptRotatePt2f(vecMatchParameter[i].pt, ptCenter, dRAngle)
        dAngleStep = math.atan(2.0 / max(iDstW, iDstH)) * R2D
        vecMatchParameter[i].dAngleStart = vecMatchParameter[i].dMatchAngle - dAngleStep
        vecMatchParameter[i].dAngleEnd = vecMatchParameter[i].dMatchAngle + dAngleStep
        if iTopLayer <= iStopLayer:
            ##################################################
            # TODO this path not tested. no point2D in python
            ##################################################
            vecMatchParameter[i].pt = cv2.Point2d(ptLT * (1 if iTopLayer == 0 else 2))
            vecAllResult.append(vecMatchParameter[i])
        else:
            for iLayer in range(iTopLayer - 1, iStopLayer - 1, -1):
                #search angle
                dAngleStep = math.atan(2.0 / max(pTemplData.vecPyramid[iLayer].shape)) * R2D #min changed to max
                vecAngles = []
                #double dAngleS = vecMatchParameter[i].dAngleStart, dAngleE = vecMatchParameter[i].dAngleEnd;
                dMatchedAngle = vecMatchParameter[i].dMatchAngle
                if m_bToleranceRange:
                    ############################
                    # TODO this path not tested
                    ############################
                    for j in range(-1, 2):
                        vecAngles.append(dMatchedAngle + dAngleStep * j)
                else:
                    if dToleranceAngle < VISION_TOLERANCE:
                        ############################
                        # TODO this path not tested
                        ############################
                        vecAngles.append(0.0)
                    else:
                        for j in range(-1, 2):
                            vecAngles.append(dMatchedAngle + dAngleStep * j)
                
                ptSrcCenter = ((vecMatSrcPyr[iLayer].shape[1] - 1) / 2.0, (vecMatSrcPyr[iLayer].shape[0] - 1) / 2.0)
                iSize = len(vecAngles)
                vecNewMatchParameter = [s_MatchParameter() for _ in range(iSize)]
                iMaxScoreIndex = 0
                dBigValue = -1
                for j in range(iSize):
                    matResult = None
                    matRotatedSrc = None
                    dMaxValue = 0
                    ptMaxLoc = None
                    matRotatedSrc = GetRotatedROI(vecMatSrcPyr[iLayer], pTemplData.vecPyramid[iLayer].shape, ptLT * 2, vecAngles[j])
                    matResult = match_template(matRotatedSrc, pTemplData, matResult, iLayer, UseSIMD)
                    minVal, dMaxValue, minLoc, ptMaxLoc = cv2.minMaxLoc(matResult)
                    vecNewMatchParameter[j] = s_MatchParameter(ptMaxLoc, dMaxValue, vecAngles[j])
                    if vecNewMatchParameter[j].dMatchScore > dBigValue:
                        iMaxScoreIndex = j
                        dBigValue = vecNewMatchParameter[j].dMatchScore

                    #subpixel estimation
                    if ptMaxLoc[0] == 0 or ptMaxLoc[1] == 0 or ptMaxLoc[0] == matResult.shape[1] - 1 or ptMaxLoc[1] == matResult.shape[0] - 1:
                        vecNewMatchParameter[j].bPosOnBorder = True
                    if not vecNewMatchParameter[j].bPosOnBorder:
                        for y in range(-1, 2):
                            for x in range(-1, 2):
                                vecNewMatchParameter[j].vecResult[y + 1][x + 1] = matResult[ptMaxLoc[1] + x, ptMaxLoc[0] + y]
                    #subpixel estimation

                if vecNewMatchParameter[iMaxScoreIndex].dMatchScore < vecLayerScore[iLayer]:
                    break
                if bSubPixelEstimation and iLayer == 0 and not vecNewMatchParameter[iMaxScoreIndex].bPosOnBorder and iMaxScoreIndex != 0 and iMaxScoreIndex != 2:
                    ############################
                    # TODO this path not tested
                    ############################
                    dNewX, dNewY, dNewAngle = 0, 0, 0
                    sub_pix_estimation(vecNewMatchParameter, dNewX, dNewY, dNewAngle, dAngleStep, iMaxScoreIndex)
                    vecNewMatchParameter[iMaxScoreIndex].pt = cv2.Point2d(dNewX, dNewY)
                    vecNewMatchParameter[iMaxScoreIndex].dMatchAngle = dNewAngle

                dNewMatchAngle = vecNewMatchParameter[iMaxScoreIndex].dMatchAngle
                ptPaddingLT = ptRotatePt2f(ptLT * 2, ptSrcCenter, dNewMatchAngle * D2R) - (3, 3)
                pt = (vecNewMatchParameter[iMaxScoreIndex].pt[0] + ptPaddingLT[0], vecNewMatchParameter[iMaxScoreIndex].pt[1] + ptPaddingLT[1])
                pt = ptRotatePt2f(pt, ptSrcCenter, -dNewMatchAngle * D2R)
                if iLayer == iStopLayer:
                    vecNewMatchParameter[iMaxScoreIndex].pt = pt * (1 if iStopLayer == 0 else 2)
                    vecAllResult.append(vecNewMatchParameter[iMaxScoreIndex])
                else:
                    vecMatchParameter[i].dMatchAngle = dNewMatchAngle
                    vecMatchParameter[i].dAngleStart = vecMatchParameter[i].dMatchAngle - dAngleStep / 2
                    vecMatchParameter[i].dAngleEnd = vecMatchParameter[i].dMatchAngle + dAngleStep / 2
                    ptLT = pt
                    
    ###########################################################################################
    # TODO vecAllResult has one extra index here compared to c++ version. looks to be index 0.
    ###########################################################################################
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
    for i in range(len(m_vecSingleTargetData)):
        
        # translate the points around the origin taken from calibration data
        ptCenterWithRoi = (m_vecSingleTargetData[i].ptCenter[0] + roi_top_left[0][0], m_vecSingleTargetData[i].ptCenter[1] + roi_top_left[1][0])
        trans_XY = ptRotatePt2f(ptCenterWithRoi, origin, math.radians(rotation_offset))
        trans_XY = trans_XY - origin
        trans_X = trans_XY[0][0] / pixel_ratio # add back roi and convert to user units
        trans_Y = trans_XY[1][0] / pixel_ratio # add back roi and convert to user units

        # create string to send as result
        # LOC obj:0 cx:123.45 cy:678.90 a:55.94 s:0.95
        result = "LOC "
        result = result + "obj:" + str(i) + " "
        result = result + "cx:" + str(round(trans_X,floatPrecision)) + " "
        result = result + "cy:" + str(round(trans_Y,floatPrecision)) + " "
        result = result + "a:" + str(round(-(m_vecSingleTargetData[i].dMatchedAngle+rotation_offset),floatPrecision)) + " "
        result = result + "s:" + str(round(m_vecSingleTargetData[i].dMatchScore,floatPrecision)) + " "
        print(result)
    
    # mark region of interest image with found objects
    markedRoi = RefreshSrcView(m_matRoi, m_matDst, m_vecSingleTargetData, pixel_ratio)
    #cv2.imshow("markedRoi", markedRoi)
    #cv2.waitKey(0)
    
    # add region of interest back on top of original
    matColorSrc = cv2.cvtColor(src=m_matSrc, code=cv2.COLOR_GRAY2BGR)
    #cv2.imshow("matColorSrc", matColorSrc)
    #cv2.waitKey(0)
    org = [round(roi_top_left[1][0]),round(roi_top_left[0][0])]
    resultImage = overlay_images(markedRoi, matColorSrc, org)
    #cv2.imshow("resultImage", resultImage)
    #cv2.waitKey(0)

    # save image to disk
    #cv2.imwrite(savelocation, resultImage)
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