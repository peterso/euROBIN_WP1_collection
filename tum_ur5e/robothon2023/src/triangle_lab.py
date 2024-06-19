'''
    @file triangle_lab.py
    @authors Adrian Müller (adrian.mueller@study.thws.de), 
            Maximilian Hornauer (maximilian.hornauer@study.thws.de),
            Usama Ali (usama.ali@study.thws.de)
    @brief Program to detect triangles on a M5 screen
    @version 0.1
    @date 2023-03-29

    @copyright Copyright (c) 2023
'''


import rospy

import cv2
import numpy as np
import math
import time

import cv_bridge
from cv2 import WINDOW_NORMAL
from cv2 import WND_PROP_FULLSCREEN
from cv2 import WINDOW_FULLSCREEN

import sensor_msgs.msg
from robothon2023.srv import GetTriangles, GetTrianglesResponse

from robothon2023.srv import GetFinished, GetFinishedResponse 

def realsenseImgCallback(data):
    global realBuf
    global bufIndex
    realBuf[bufIndex] = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    bufIndex = (bufIndex +1)%5



def service_callback(req):
    delta = 0
    status = -1
    
    screenMask, width = detectM5()
    if(screenMask is not None):
        positions = detectTriangles(screenMask)
        if(checkDone(screenMask)<9000 and positions[0] is None):
            done = True
            for i in range(3):
                rospy.sleep(0.03)
                screenMask, width = detectM5()
                if(checkDone(screenMask)>9000):
                    done = False
            if(done):
                return GetTrianglesResponse(0,2)

        
        if(positions[0] is None): 
            status = -1
        else:
            if(positions[2] is None): #no green Triangle: Delta between red and yellow
                if(positions[1] is None):
                    status = -1
                else:
                    delta = positions[0][0] - positions[1][0]
            else:
                delta = positions[0][0] - positions[2][0]
            delta = delta/(width/2) #normalize delta to screen size
            status = 1

    return GetTrianglesResponse(delta,status)


def service_callback_finished(req): #/*
    #print("start message")
    screenMask, width = detectM5()
    #cv2.imshow("image", screenMask)
    #cv2.waitKey(0)
    
    if screenMask is not None:
        #print("image ok")
        if(checkDone(screenMask)>6000):
            return GetFinishedResponse(True)
        else:
            return GetFinishedResponse(False)


def detectM5():
    global realBuf
    if(realBuf[0] is not None):
        meanImg = np.mean(realBuf, axis=0).astype(np.uint8)

        labimg = cv2.cvtColor(meanImg, cv2.COLOR_BGR2LAB)
        #cv2.imshow("ImageLAB", labimg)
        #cv2.waitKey(10)

        loweryellow = np.array([0, 147, 115])
        upperyellow = np.array([110, 193, 179])
        m5Mask = cv2.inRange(labimg, loweryellow, upperyellow)
        #cv2.imshow("massk", m5Mask)
        kernel = np.ones((10, 10), np.uint8)
        m5Mask = cv2.erode(m5Mask, kernel=kernel)
        m5Mask = cv2.dilate(m5Mask, kernel=kernel, iterations= 2)
        # cv2.imshow("m5Mask", m5Mask)
        #cv2.waitKey(0)
        #cv2.imshow("ImageLAB", m5Mask)
        cnt, hierarchy = cv2.findContours(m5Mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        displayMaskCnt = None
        maxArea = 0
        selCnt = []
        if(len(cnt) == 0):
            return None, 0
        # TODO remove every contour without child

        # take every contour that is a child and get the biggest
        for idx,h in enumerate(hierarchy[0]):
            if(h[3] != -1):
                selCnt.append(cnt[idx])
                if(cv2.contourArea(cnt[idx]) > maxArea):
                   maxArea = cv2.contourArea(cnt[idx])
                   displayMaskCnt = cnt[idx]
        img = meanImg.copy()
        if(displayMaskCnt is None):
            return None, 0
        
        # cv2.drawContours(img, [displayMaskCnt], -1, (255,255,255), 3)
        # cv2.imshow("real", img)
        # cv2.waitKey(10)
        
        screenMask = np.zeros_like(m5Mask)
        cv2.drawContours(screenMask, [displayMaskCnt], -1, (255), -1)
        _,_,w,_ = cv2.boundingRect(displayMaskCnt)
        
        screenLab = cv2.bitwise_and(labimg, labimg, mask=screenMask)

        return screenLab, w
    else:
        return None, 0


def getTrianglePos(triangleMask, yLimit):
    cnts, _ = cv2.findContours(triangleMask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if(len(cnts) == 0):
        return None
    filteredPos = []
    if(yLimit is not None):
        for cnt in cnts:
            #get contour centers
            moments = cv2.moments(cnt)
            cntX = int(moments["m10"] / moments["m00"])
            cntY = int(moments["m01"] / moments["m00"])
            #filter out anything thats below the limit
            if(cntY > yLimit):
                continue
            filteredPos.append((cntX, cntY,cv2.contourArea(cnt)))
    else:
        moments = cv2.moments(cnts[0])
        cntX = int(moments["m10"] / moments["m00"])
        cntY = int(moments["m01"] / moments["m00"])
        filteredPos.append((cntX, cntY, cv2.contourArea))
        
    if(len(filteredPos) > 1):
        print("Too many contours! Found: " + str(len(filteredPos)))
        maxAreaPos = filteredPos[np.argmax(list(map(lambda t : t[2], filteredPos)))]
        return maxAreaPos[:2]
    if(len(filteredPos) == 0): #no green triangle
        return None
    
    #return center of triangle
    return filteredPos[0][:2]


def detectTriangles(labimg):
    global realBuf
    
    meanImg = np.mean(realBuf, axis=0).astype(np.uint8)
    # cv2.imshow("meanImg", meanImg)
    trianglePoints = []
    #Cut out our mask
    drawImg = meanImg.copy()

    
    #threshold red
    # lowerred = np.array([12, 172, 129])       # werte Samstag nacht
    lowerred = np.array([12, 172, 110])         # werte Sonntag früh
    upperred = np.array([205, 214, 222])
    redMask = cv2.inRange(labimg, lowerred, upperred)
    kernel = np.ones((5, 5), np.uint8)
    redMask = cv2.erode(redMask, kernel=kernel)
    redMask = cv2.dilate(redMask, kernel=kernel)
    # cv2.imshow("RedMask", redMask)
    # cv2.waitKey(12)

    #threshold yellow
    loweryellow = np.array([154, 109, 102])
    upperyellow = np.array([233, 140, 178])
    #old [233, 123, 178]
    yellowMask = cv2.inRange(labimg, loweryellow, upperyellow)
    #cv2.imshow("yellowMaskNoErode", yellowMask)
    kernel = np.ones((5, 5), np.uint8)
    #no erode because we would erode too much
    yellowMask = cv2.erode(yellowMask, kernel=np.ones((3, 3)))
    yellowMask = cv2.dilate(yellowMask, kernel=np.ones((5, 5)))
    # cv2.imshow("YellowMask", yellowMask)
    # cv2.waitKey(10)

    #threshold green
    lowergreen = np.array([0, 81, 103])
    uppergreen = np.array([220, 107, 170])
    #old [220, 107, 170]
    greenMask = cv2.inRange(labimg, lowergreen, uppergreen)
    #no erode because we would erode too much
    greenMask = cv2.dilate(greenMask, kernel=np.ones((5, 5)))
    # cv2.imshow("GreenMask", greenMask)
    # cv2.waitKey(10)

    #join all three for visualization
    totalMask = cv2.bitwise_or(redMask,yellowMask)
    totalMask = cv2.bitwise_or(totalMask,greenMask)
    # cv2.imshow("Total", cv2.resize(totalMask,(720,480)))
    # cv2.waitKey(10)
    redPos = getTrianglePos(redMask,None)
    if(redPos is None):
        return [None, None, None]
    trianglePoints.append(redPos)
    cv2.circle(drawImg, redPos, 10, (0,0,255), -1)

    yellowPos = getTrianglePos(yellowMask,redPos[1])
    trianglePoints.append(yellowPos)
    cv2.circle(drawImg, yellowPos, 10, (0,255,255), -1)
    
    greenPos = getTrianglePos(greenMask,redPos[1])
    trianglePoints.append(greenPos)
    cv2.circle(drawImg, greenPos, 10, (0,255,0), -1)
    
    print(trianglePoints)
    # cv2.imshow("Drawn Img", cv2.resize(drawImg,(720,480)))
    # cv2.waitKey(10)

    return trianglePoints

def checkDone(labimg):
    # lowerblack = np.array([0, 0, 90])         # werte Samstag nacht
    # upperblack = np.array([78, 151, 121])     # werte Samstag nacht
    #lowerblack = np.array([0, 0, 60])           # werte Sontage früh
    #upperblack = np.array([78, 166, 121])       # werte Sontage früh
    lowerblack = np.array([0, 0, 60])           # werte 05.05 mit black screen detection
    upperblack = np.array([78, 166, 160])       # werte 05.05 mit lack screen detection

    blackMask = cv2.inRange(labimg, lowerblack, upperblack)
    # cv2.imshow("Black", blackMask)
    # cv2.waitKey(10)
    print(cv2.countNonZero(blackMask))
    return cv2.countNonZero(blackMask)

if __name__ == "__main__":
    #Setup Ros
    rospy.init_node('triangle_detection')
    rospy.Subscriber("/camera/color/image_raw", sensor_msgs.msg.Image, realsenseImgCallback)
    bridge = cv_bridge.CvBridge()
    rate = rospy.Rate(1)
    realBuf = [None]*5
    bufIndex = 0
    #wait till buffer is filled for the first time
    while(True):
        bufferFull = True
        for i in realBuf:
            if(i is None):
                bufferFull = False
        if(bufferFull):
            break
        
    s = rospy.Service('/triangle_detection', GetTriangles, service_callback)
    a = rospy.Service('/finished_detection', GetFinished, service_callback_finished) #/*



    #while(not rospy.is_shutdown()):
    #     labimg, width = detectM5()
    #     cv2.imshow("mask", labimg)
    #     cv2.waitKey(10)
    #     if labimg is not None:
    #        if(checkDone(labimg)<5000):
    #            print("true")
    #        else:
    #            print("false")
    #     if(labimg is not None):
    #         if(checkDone(labimg)<500):
    #             print("Done")
    #         else:
    #             positions = detectTriangles(labimg)
    #     rate.sleep()


    rospy.spin()