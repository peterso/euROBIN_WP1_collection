#!/usr/bin/env python
'''
    @file board_detection.py
    @authors Adrian Müller (adrian.mueller@study.thws.de), 
            Maximilian Hornauer (maximilian.hornauer@study.thws.de),
            Usama Ali (usama.ali@study.thws.de)
    @brief Program to detect the taskboard
    @version 0.1
    @date 2023-03-29

    @copyright Copyright (c) 2023
'''

import rospy

import cv2
import numpy as np
import math

import cv_bridge
from cv2 import WINDOW_NORMAL
from cv2 import WND_PROP_FULLSCREEN
from cv2 import WINDOW_FULLSCREEN

import ros_numpy

import tf2_ros
from tf import transformations

import sensor_msgs.msg
from robothon2023.srv import AddTf2
from robothon2023.srv import GetBoardLocation, GetBoardLocationResponse
from geometry_msgs.msg import TransformStamped 
from geometry_msgs.msg import Transform

from scipy.spatial.transform import Rotation

### ROS setup stuff
idsBuf = None

def service_callback(req):
    global tfBuffer

    if(idsBuf is not None):
        M, angle = BoardDetetction()

        cam = transformStamped_to_numpy(tfBuffer.lookup_transform('base', 'ids_cam', rospy.Time()))
        cam[2,3] += 0.009 
        
        tb  = cam @ M

        tb_euler = Rotation.from_matrix(tb[:3,:3]).as_euler("zyx")

        tb_euler[1] = 0
        tb_euler[2] = 0
        tb[:3,:3] = Rotation.from_euler("zyx", tb_euler).as_matrix()
        tb[2][3] = 0.092

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base'
        t.child_frame_id = 'task_board'
        t.transform = ros_numpy.msgify(Transform, tb)
        addTf = rospy.ServiceProxy('/store_tf', AddTf2)
        addTf.call(t, False)

        print("board angle " +str(angle))
        config = int((angle+45)/90)%4

        return GetBoardLocationResponse(True,config)
    else:
        return GetBoardLocationResponse(False,0)

#def realsenseImgCallback(data):
#    global realBuf
#    realBuf = data

def tcpPoseCallback(data):
    global pose
    pose = transformStamped_to_numpy(data)

def transformStamped_to_numpy(msg):
    return np.dot(
        transformations.translation_matrix(np.array([msg.transform.translation.x, msg.transform.translation.y,  msg.transform.translation.z])),
        transformations.quaternion_matrix(np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]))
    )


def idsImgCallBack(data):
    global idsBuf
    idsBuf = data

### Image Detection 
def findBlueBnt(image):
    try:
        img = image.copy()
        hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lowerblue = np.array([95, 55, 70])
        upperblue = np.array([128, 255, 255])
        bluemask = cv2.inRange(hsvimg, lowerblue, upperblue)

        kernelSize = 5
        opIterations = 2
        morphKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize))
        bluemask = cv2.morphologyEx(bluemask, cv2.MORPH_CLOSE, morphKernel, None, None, opIterations, cv2.BORDER_REFLECT101)

        kernel = np.ones((3, 3), np.uint8)
        bluemask = cv2.erode(bluemask, kernel=kernel, iterations=opIterations)
        bluemask = cv2.dilate(bluemask, kernel=kernel, iterations=opIterations)

        bluecircle = cv2.HoughCircles(bluemask, cv2.HOUGH_GRADIENT, 1, minDist=5000, param1=50, param2=10, minRadius=45, maxRadius=49)

        cv2.circle(image, (int(bluecircle[0][0][0]), int(bluecircle[0][0][1])), 3, (0,165,255), 3, cv2.LINE_AA)
        cv2.circle(image, (int(bluecircle[0][0][0]), int(bluecircle[0][0][1])), int(bluecircle[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
        cv2.putText(image, "1", (int(bluecircle[0][0][0]), int(bluecircle[0][0][1])), cv2.LINE_AA, 2, (0, 165,255), 6)

        #cv2.imshow("Time "+str(j) , cv2.resize(image, (1384, 923)))
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()

        bluecircle = np.reshape(bluecircle, (1,3))
        
        return bluecircle
    except Exception as e:
        print("No blue button -->" + str(e) )
        #cv2.imshow("Time "+str(j) , cv2.resize(image, (1384, 923)))
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()
        
        return None

def findRedCircels(image):
    try:
        img = image.copy()
        hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #lowerred = np.array([0, 75, 50])
        #upperred = np.array([10, 255, 255])
        #redmask = cv2.inRange(hsvimg, lowerred, upperred)

        lowerredlow = np.array([0, 70, 50])
        upperredlow = np.array([13, 255, 255])
        lowerredup = np.array([155,77,0])
        upperredup = np.array([255,255,255])
        redmasklow = cv2.inRange(hsvimg, lowerredlow, upperredlow)
        redmaskup = cv2.inRange(hsvimg, lowerredup, upperredup)
        redmask = cv2.bitwise_or(redmasklow, redmaskup)

        kernelSize = 5
        opIterations = 2
        morphKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize))
        redmask = cv2.morphologyEx(redmask, cv2.MORPH_CLOSE, morphKernel, None, None, opIterations, cv2.BORDER_REFLECT101)

        #cv2.imshow("a" , cv2.resize(redmask, (1384, 923)))
        #cv2.waitKey(500)

        kernelerode = np.ones((3,3), np.uint8)
        redmask = cv2.erode(redmask, kernel=kernelerode, iterations=opIterations)
        kerneldilate = np.ones((3,3), np.uint8)
        redmask = cv2.dilate(redmask, kernel=kerneldilate, iterations=opIterations)

        redcircle = cv2.HoughCircles(redmask, cv2.HOUGH_GRADIENT, 1, minDist=5000, param1=50, param2=10, minRadius=45, maxRadius=49)

        redconnectorbig = cv2.HoughCircles(redmask, cv2.HOUGH_GRADIENT, 1, minDist = 5000, param1 = 50, param2 = 10, minRadius= 53, maxRadius=61)

        redconnectorsmall = cv2.HoughCircles(redmask, cv2.HOUGH_GRADIENT, 1, minDist = 5000, param1 = 50, param2 = 10, minRadius= 10, maxRadius=19)

        #redbutton
        if redcircle is not None:
            cv2.circle(image, (int(redcircle[0][0][0]), int(redcircle[0][0][1])), 3, (0,165,255), 3, cv2.LINE_AA)
            cv2.circle(image, (int(redcircle[0][0][0]), int(redcircle[0][0][1])), int(redcircle[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
            cv2.putText(image, "2", (int(redcircle[0][0][0]), int(redcircle[0][0][1])), cv2.LINE_AA, 2, (0, 165,255), 6)
        
        #connector
        #caclulate
        if redconnectorbig is not None and redconnectorsmall is not None:
            if redconnectorbig[0][0][0] < redconnectorsmall[0][0][0]-2 or redconnectorbig[0][0][0] > redconnectorsmall[0][0][0]+2 or redconnectorbig[0][0][1] < redconnectorsmall[0][0][1]-2 or redconnectorbig[0][0][1] > redconnectorsmall[0][0][1]+2:
                #print("big: -x " + str(redconnectorbig[0][0][0]) + " -y " + str(redconnectorbig[0][0][1]))
                cv2.circle(image, (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), 3, (0,165,255), 3, cv2.LINE_AA)
                cv2.circle(image, (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), int(redconnectorbig[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
                cv2.putText(image, "3", (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), cv2.LINE_AA, 2, (0, 165,255), 6)
                redconnector = redconnectorbig
            else:
                redconnector = np.array([(redconnectorbig[0][0][0] + redconnectorsmall[0][0][0])/2, (redconnectorbig[0][0][1] + redconnectorsmall[0][0][1])/2, redconnectorbig[0][0][2]])
                #print("small: -x " + str(redconnectorsmall[0][0][0]) + " -y " + str(redconnectorsmall[0][0][1]))
                #print("big: -x " + str(redconnectorbig[0][0][0]) + " -y " + str(redconnectorbig[0][0][1]))
                cv2.circle(image, (int(redconnector[0]), int(redconnector[1])), 3, (0,165,255), 3, cv2.LINE_AA)
                cv2.circle(image, (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), int(redconnectorbig[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
                cv2.circle(image, (int(redconnectorsmall[0][0][0]), int(redconnectorsmall[0][0][1])), int(redconnectorsmall[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
                cv2.putText(image, "3", (int(redconnector[0]), int(redconnector[1])), cv2.LINE_AA, 2, (0, 165,255), 6)
        elif redconnectorbig is not None:
            #print("big: -x " + str(redconnectorbig[0][0][0]) + " -y " + str(redconnectorbig[0][0][1]))
            cv2.circle(image, (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), 3, (0,165,255), 3, cv2.LINE_AA)
            cv2.circle(image, (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), int(redconnectorbig[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
            cv2.putText(image, "3", (int(redconnectorbig[0][0][0]), int(redconnectorbig[0][0][1])), cv2.LINE_AA, 2, (0, 165,255), 6)
            redconnector = redconnectorbig
        elif redconnectorsmall is not None:
            #print("small: -x " + str(redconnectorsmall[0][0][0]) + " -y " + str(redconnectorsmall[0][0][1]))
            cv2.circle(image, (int(redconnectorsmall[0][0][0]), int(redconnectorsmall[0][0][1])), 3, (0,165,255), 3, cv2.LINE_AA)
            cv2.circle(image, (int(redconnectorsmall[0][0][0]), int(redconnectorsmall[0][0][1])), int(redconnectorsmall[0][0][2]), (0,255,0), 3, cv2.LINE_AA)
            cv2.putText(image, "2", (int(redconnectorsmall[0][0][0]), int(redconnectorsmall[0][0][1])), cv2.LINE_AA, 2, (0, 165,255), 6)
            redconnector = redconnectorsmall
        else:
            redconnector = None

        #cv2.imshow("" , cv2.resize(image, (1384, 923)))
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()
        #print(np.shape(redcircle))
        #print(redcircle)
        #print(redconnector)
        redconnector = np.reshape(redconnector, (1,3))
        redcircle = np.reshape(redcircle, (1,3))
        
        return redcircle, redconnector
    except Exception as e:
        print("No red button --> " + str(e))
        #cv2.imshow("Time "+str(j) , cv2.resize(image, (1384, 923)))
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()
        
        return None, None
    
def findRedM5(image):
    try:
        img = image.copy()
        hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #blackimg = np.zeros_like(crop) 
        #lowerred = np.array([0, 70, 50])
        #upperred = np.array([13, 255, 255])
        #redmask = cv2.inRange(hsvimg, lowerred, upperred)

        lowerredlow = np.array([0, 70, 50])
        upperredlow = np.array([13, 255, 255])
        lowerredup = np.array([155,77,0])
        upperredup = np.array([255,255,255])
        redmasklow = cv2.inRange(hsvimg, lowerredlow, upperredlow)
        redmaskup = cv2.inRange(hsvimg, lowerredup, upperredup)
        redmask = cv2.bitwise_or(redmasklow, redmaskup)

        kernelSize = 5
        opIterations = 2
        morphKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize))
        redmask = cv2.morphologyEx(redmask, cv2.MORPH_CLOSE, morphKernel, None, None, opIterations, cv2.BORDER_REFLECT101)

        kernel = np.ones((10, 10), np.uint8)
        redmask = cv2.erode(redmask, kernel=kernel, iterations=opIterations)
        redmask = cv2.dilate(redmask, kernel=kernel, iterations=opIterations)

        #cv2.imshow("red m5" , cv2.resize(redmask, (692,462)))
        #cv2.waitKey(500)

        cnts1, _ = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts1:
            area = cv2.contourArea(cnt)
            
            if area > 65000 and area < 80000:
                #print("area cont " + str(area))
                rect= cv2.minAreaRect(cnt)
                arearect = rect[1][0] * rect[1][1]
                if arearect <90000 and arearect > 70000:
                    #print("area rect " + str(arearect))
                    #print(np.shape(rect))
                    #print(rect[0][0])
                    #print("area of rect: -x " +str(rect[0][0]) + " y " + str(rect[0][1]) + " w " + str(rect[1][0]) + " h " + str(rect[1][1]))
                    redbox = np.intp(cv2.boxPoints(rect))
                    middleredbox = np.mean(redbox, axis=0)
                    box = np.array([middleredbox[0], middleredbox[1], 0])

                    cv2.drawContours(image, [redbox], -1, (0,255,0), 3)
                    cv2.circle(image, (int(box[0]), int(box[1])), 3, (0,165,255), 3)
                    cv2.putText(image, "4", (int(box[0]), int(box[1])), cv2.LINE_AA, 2, (0, 165,255), 6)
                    #cv2.drawContours(blackimg, [redbox], -1, (255,255,255), cv2.FILLED)

                    middleredbox = np.reshape(box, (1,3))
                   
                    return middleredbox
                
        #redbox = np.reshape(redbox, (4,2))

        #cv2.imshow("red m5" , cv2.resize(blackimg, (1384, 923)))
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()
        
        #, blackimg
        return None
    except Exception as e:
        print("No red M5 " + str(e))
        #cv2.imshow("Time "+str(j) , cv2.resize(image, (1384, 923)))
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()
        
        return None#, crop
    
def caculateRotation(bluecircle, redcircle, redM5, image):
    #bluecircle and redcircle as reference
    try:
        if bluecircle is not None and redcircle is not None:
            angle = np.degrees(math.atan2(bluecircle[0][1]-redcircle[0][1], bluecircle[0][0]-redcircle[0][0]))
            angle = (angle +360) % 360
            cv2.line(image, (int(bluecircle[0][0]), int(bluecircle[0][1])), (int(redcircle[0][0]), int(redcircle[0][1])),(255,0,0),8)
            cv2.line(image, (int(bluecircle[0][0]-100), int(bluecircle[0][1])),  (int(bluecircle[0][0]), int(bluecircle[0][1])), (255,0,0),8)
            cv2.putText(image, str(int(angle)) + "deg", (int(bluecircle[0][0] -350), int(bluecircle[0][1] +20)), cv2.LINE_AA, 2, (255, 0,0), 6)

        elif bluecircle is not None and redM5 is not None:
            angle = np.degrees(math.atan2(bluecircle[0][1]-redM5[0][1], bluecircle[0][0]-redM5[0][0]))
            angle = (angle +360) % 360
            cv2.line(image, (int(bluecircle[0][0]), int(bluecircle[0][1])), (int(redM5[0][0]), int(redM5[0][1])),(255,0,0),8)
            cv2.line(image, (int(bluecircle[0][0]-100), int(bluecircle[0][1])),  (int(bluecircle[0][0]), int(bluecircle[0][1])), (255,0,0),8)
            cv2.putText(image, str(int(angle)) + "deg", (int(bluecircle[0][0] -350), int(bluecircle[0][1] +20)), cv2.LINE_AA, 2, (255, 0,0), 6)

        elif redcircle is not None and redM5 is not None:
            angle = np.degrees(math.atan2(redcircle[0][1]-redM5[0][1], redcircle[0][0]-redM5[0][0]))
            angle = (angle +360) % 360
            cv2.line(image, (int(redcircle[0][0]), int(redcircle[0][1])), (int(redM5[0][0]), int(redM5[0][1])),(255,0,0),8)
            cv2.line(image, (int(redcircle[0][0]-100), int(redcircle[0][1])),  (int(redcircle[0][0]), int(redcircle[0][1])), (255,0,0),8)
            cv2.putText(image, str(int(angle)) + "deg", (int(redcircle[0][0] -350), int(redcircle[0][1] +20)), cv2.LINE_AA, 2, (255, 0,0), 6)

        return angle

    except Exception as e:
        print("No angle found --> " + str(e))
        return None

    
    #if bluecircle is not None and redconnector is not None:
    #    angle = np.degrees(math.atan2(bluecircle[0][1]-redconnector[0][1], bluecircle[0][0]-redconnector[0][0]))
    #    angle = (angle +360) % 360
    #    brcc = angle
    #if redcircle is not None and redconnector is not None:
    #    angle = np.degrees(math.atan2(redcircle[0][1]-redconnector[0][1], redcircle[0][0]-redconnector[0][0]))
    #    angle = (angle +360) % 360    
    #if redconnector is not None and redM5 is not None:
    #    angle = np.degrees(math.atan2(redconnector[0][1]-redM5[0][1], redconnector[0][0]-redM5[0][0]))
    #    angle = (angle +360) % 360
    #    
    #    rcrm5 = angle
    
def caculateMask(bluecircle, redcircle, angle, image):
    circlesize = 80
    
    try:
        blackimg1 = np.zeros_like(image)
        #blackimg2 = np.zeros_like(image)

        if bluecircle is not None:
            #distance 226.9423 distance 775.1134 distance 867.61676 distance 1139.5376 distance 1673.3059 distance 1872.0208
            
            x1 = bluecircle[0][0] + 227 * np.cos(np.radians(angle+43))
            y1 = bluecircle[0][1] + 227 * np.sin(np.radians(angle+43))
            x2 = bluecircle[0][0] + 775 * np.cos(np.radians(angle-77))
            y2 = bluecircle[0][1] + 775 * np.sin(np.radians(angle-77))
            x3 = bluecircle[0][0] + 1673.3 * np.cos(np.radians(angle-84))
            y3 = bluecircle[0][1] + 1673.3 * np.sin(np.radians(angle-84))
            x4 = bluecircle[0][0] + 1872 * np.cos(np.radians(angle-117))
            y4 = bluecircle[0][1] + 1872 * np.sin(np.radians(angle-117))
            x5 = bluecircle[0][0] + 1139.5 * np.cos(np.radians(angle-139))
            y5 = bluecircle[0][1] + 1139.5 * np.sin(np.radians(angle-139))
            x6 = bluecircle[0][0] + 867.6 * np.cos(np.radians(angle+170))
            y6 = bluecircle[0][1] + 867.6 * np.sin(np.radians(angle+170))

            #cv2.circle(image, (int(x1), int(y1)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x2), int(y2)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x3), int(y3)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x4), int(y4)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x5), int(y5)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x6), int(y6)), 3, (0,165,255), 3)

            #create the mask 
            cv2.circle(blackimg1, (int(x1), int(y1)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x2), int(y2)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x3), int(y3)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x4), int(y4)), circlesize, (255,255,255), cv2.FILLED)
            #cv2.circle(blackimg2, (int(x4), int(y4)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x5), int(y5)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x6), int(y6)), circlesize, (255,255,255), cv2.FILLED)

            newimg1 = cv2.bitwise_and(blackimg1, image)
            #newimg2 = cv2.bitwise_and(blackimg2, image)

            #cv2.imshow("blackimg1", cv2.resize(newimg1, (1280,720)))
            #cv2.waitKey(10)
            #cv2.imshow("blackimg2", cv2.resize(newimg2, (1280,720)))
            #cv2.waitKey(0)

        elif redcircle is not None:
            #distance 317.1101 distance 759.98346 distance 806.2238 distance 1059.7273 distance 1687.8118 distance 1824.4368
            
            x1 = redcircle[0][0] + 317 * np.cos(np.radians(angle+30))
            y1 = redcircle[0][1] + 317 * np.sin(np.radians(angle+30))
            x2 = redcircle[0][0] + 806.2  * np.cos(np.radians(angle-70))
            y2 = redcircle[0][1] + 806.2 * np.sin(np.radians(angle-70))
            x3 = redcircle[0][0] + 1687.8 * np.cos(np.radians(angle-81))
            y3 = redcircle[0][1] + 1687.8 * np.sin(np.radians(angle-81))
            x4 = redcircle[0][0] + 1824.4 * np.cos(np.radians(angle-114))
            y4 = redcircle[0][1] + 1824.4 * np.sin(np.radians(angle-114))
            x5 = redcircle[0][0] + 1059.7 * np.cos(np.radians(angle-135))
            y5 = redcircle[0][1] + 1059.7 * np.sin(np.radians(angle-135))
            x6 = redcircle[0][0] + 763 * np.cos(np.radians(angle+169))
            y6 = redcircle[0][1] + 763 * np.sin(np.radians(angle+169))

            #cv2.circle(image, (int(x1), int(y1)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x2), int(y2)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x3), int(y3)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x4), int(y4)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x5), int(y5)), 3, (0,165,255), 3)
            #cv2.circle(image, (int(x6), int(y6)), 3, (0,165,255), 3)

            #create the mask 
            cv2.circle(blackimg1, (int(x1), int(y1)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x2), int(y2)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x3), int(y3)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x4), int(y4)), circlesize, (255,255,255), cv2.FILLED)
            #cv2.circle(blackimg2, (int(x4), int(y4)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x5), int(y5)), circlesize, (255,255,255), cv2.FILLED)
            cv2.circle(blackimg1, (int(x6), int(y6)), circlesize, (255,255,255), cv2.FILLED)

            newimg1 = cv2.bitwise_and(blackimg1, image)
            #newimg2 = cv2.bitwise_and(blackimg2, image)

            #äcv2.imshow("blackimg1", cv2.resize(newimg1, (1280,720)))
            #cv2.waitKey(10)
            #cv2.imshow("blackimg2", cv2.resize(newimg2, (1280,720)))
            #cv2.waitKey(0)
            
        return newimg1 #, newimg2
    except Exception as e:
        print("Error mask image --> " + str(e))
        return image

def findBlaCircels(Image, mask1, bluecircle, redcircle):

    

    graymask = cv2.cvtColor(mask1, cv2.COLOR_BGR2GRAY)
    #graymask = cv2.bitwise_not(graymask)
    graymask1 = cv2.inRange(graymask, 0, 20)
    kerneldilate = np.ones((4,4), np.uint8)
    kernelerode = np.ones((3,3), np.uint8)
    kernel = np.ones((3,3), np.uint8)
    iter = 3
    graymask1 = cv2.dilate(graymask1, kernel =  kerneldilate, iterations=iter )
    graymask1 = cv2.erode(graymask1, kernel = kernelerode, iterations=iter)
    graymask1 = cv2.morphologyEx(graymask1, cv2.MORPH_CLOSE, kernel, iterations=1)
    #cv2.imshow("black mask " , cv2.resize(graymask1, (1280, 720)))
    #cv2.waitKey(0)
        
    blackcircels = cv2.HoughCircles(graymask1, cv2.HOUGH_GRADIENT, 1, minDist=300, param1=50 , param2=5, minRadius=23, maxRadius=32 )
    #print("black circles: " + str(blackcircels))
    blackcircels = np.reshape(blackcircels, (np.shape(blackcircels)[1],3))
    #print("shape black circels: " + str(np.shape(blackcircels)))

    sortedpoints = None
    if blackcircels is not None:
        i = 1    
        distances = []
        centers = []
        if bluecircle is not None:
            for blackcircle in blackcircels:
                #print("black circle -x " + str(blackcircle[0]) + " -y " + str(blackcircle[1]) + " -r " + str(blackcircle[2]))
                distance = np.linalg.norm(bluecircle-blackcircle)
                #print("distance " +str(i) + " " + str(distance))
                distances.append(distance)
                centers.append([blackcircle[0], blackcircle[1]])
                i = i +1
            sortedpoints = np.empty((6,3))
            mindstances = np.array([217, 765, 857.6, 1129.5, 1663.3, 1862])
            maxidstances = np.array([237, 785, 877.6, 1149.5, 1683.3, 1882])
            i = 0
            for i in range(6):
                #print(distances)
                pos = np.argmin(distances)
                #print(pos)
                #print(pos)
                if mindstances[i] < distances[pos] and maxidstances[i] > distances[pos]:
                    if i == 1:
                        sortedpoints[i+1][:] = blackcircels[pos][:]
                        cv2.circle(Image, (int(sortedpoints[i+1][0]), int(sortedpoints[i+1][1])), 3, (0,165,255), 3, cv2.LINE_AA)
                        cv2.circle(Image, (int(sortedpoints[i+1][0]), int(sortedpoints[i+1][1])), int(blackcircels[pos][2]), (0,255,0), 3, cv2.LINE_AA) 
                        cv2.putText(Image, str(i+6), (int(sortedpoints[i+1][0]), int(sortedpoints[i+1][1])), cv2.LINE_AA, 2, (0, 165, 255), 6)
                    elif i == 2:
                        sortedpoints[i-1][:] = blackcircels[pos][:]
                        cv2.circle(Image, (int(sortedpoints[i-1][0]), int(sortedpoints[i-1][1])), 3, (0,165,255), 3, cv2.LINE_AA)
                        cv2.circle(Image, (int(sortedpoints[i-1][0]), int(sortedpoints[i-1][1])), int(blackcircels[pos][2]), (0,255,0), 3, cv2.LINE_AA) 
                        cv2.putText(Image, str(i+4), (int(sortedpoints[i-1][0]), int(sortedpoints[i-1][1])), cv2.LINE_AA, 2, (0, 165, 255), 6)
                    else:
                        sortedpoints[i][:] = blackcircels[pos][:]
                        cv2.circle(Image, (int(sortedpoints[i][0]), int(sortedpoints[i][1])), 3, (0,165,255), 3, cv2.LINE_AA)
                        cv2.circle(Image, (int(sortedpoints[i][0]), int(sortedpoints[i][1])), int(blackcircels[pos][2]), (0,255,0), 3, cv2.LINE_AA) 
                        cv2.putText(Image, str(i+5), (int(sortedpoints[i][0]), int(sortedpoints[i][1])), cv2.LINE_AA, 2, (0, 165, 255), 6)
                    #cv2.imshow("black mask " , cv2.resize(Image, (1280, 720)))
                    #cv2.waitKey(0)
                    #print(sortedpoints)
                    
                    
                    distances[pos] = 10000
                
                else:
                    if i ==1:
                        sortedpoints[i+1][:] = None
                    elif i ==2:
                        sortedpoints[i-1][:] = None
                    else:
                        sortedpoints[i][:] = None
                    #mindstances[i] = mindstances[i+1]
                    #maxidstances[i] = maxidstances[i+1]
                    
                
            #print("sorted points " + str(sortedpoints) )
        
        elif redcircle is not None:  
            for blackcircle in blackcircels:
                #print("black circle -x " + str(blackcircle[0]) + " -y " + str(blackcircle[1]) + " -r " + str(blackcircle[2]))
                distance = np.linalg.norm(redcircle-blackcircle)
                #print("distance " +str(i) + " " + str(distance))
                distances.append(distance)
                centers.append([blackcircle[0], blackcircle[1]])
                i = i +1
            sortedpoints = np.empty((np.shape(blackcircels)))
            mindstances = np.array([307, 753, 796.2, 1049.7, 1677.8, 1814.4])
            maxidstances = np.array([327, 773, 816.2, 1069.7, 1697.8, 1834.4])
            for i in range(np.shape(blackcircels)[0]):
                #print(distances)
                pos = np.argmin(distances)
                #print(pos)
                if mindstances[i] < distances[pos] and maxidstances[i] > distances[pos]:
                    sortedpoints[i][:] = blackcircels[pos][:]
                    cv2.circle(Image, (int(sortedpoints[i][0]), int(sortedpoints[i][1])), 3, (0,165,255), 3, cv2.LINE_AA)
                    cv2.circle(Image, (int(sortedpoints[i][0]), int(sortedpoints[i][1])), int(blackcircels[pos][2]), (0,255,0), 3, cv2.LINE_AA) 
                    cv2.putText(Image, str(i+5), (int(sortedpoints[i][0]), int(sortedpoints[i][1])), cv2.LINE_AA, 2, (0, 165, 255), 6)
                    distances[pos] = 10000
        
                else:
                    sortedpoints[i][:] = None
                
            #print("sorted points " + str(sortedpoints) )

    return sortedpoints

def caculateRealPoints(bluecircels, redcircels, redconnector, blackcircels):
    
    #newpoints = np.array([[points[0][0][0], points[0][0][1]],[points[1][0][0], points[1][0][1]], [points[2][0][0], points[2][0][1]], [points[3][0][0], points[3][0][1]], [points[4][0][0], points[4][0][1]], [points[5][0][1], points[5][0][1]], [points[6][0][1], points[6][0][1]], [points[7][0][0], points[7][0][1]], [points[8][0][0], points[8][0][1]], [points[9][0][0], points[9][0][1]]])
    #print(newpoints)
    #realPositions = np.array([[0.30445,0.12035,0.003], [0.30445, 0.10635, 0.003], [0.2464, 0.0947, 0.002], [0.3037, 0.053, 0.014], [0.32415,0.14185,-0.002], [0.32415, 0.01055,-0.002], [0.12685, 0.14185, -0.002], [0.12685, 0.01055,-0.002], [0.01055, 0.14185,-0.002], [0.01055, 0.01055,-0.002]])
    #realPositions = np.array([[0.30445,0.12035,0.000], [0.30445, 0.10635, 0.000], [0.2464, 0.0947, 0.000], [0.3037, 0.053, 0.014], [0.32415,0.14185,0.0], [0.32415, 0.01055,0.0], [0.12685, 0.14185, 0.0], [0.12685, 0.01055,0.0], [0.01055, 0.14185,0.0], [0.01055, 0.01055,0.0]])
    realPositions = np.array([[0.2232, 0.1199,0.003],
                              [0.2232, 0.1058, 0.003],
                              [0.1653, 0.0947, 0.002], 
                              [0.22365, 0.0528, 0.014], 
                              [0.2432, 0.1412,-0.002], 
                              [0.2432, 0.0108,-0.002], 
                              [0.127, 0.1412, -0.002], 
                              [0.127, 0.0108,-0.002], 
                              [0.0108, 0.1412,-0.002], 
                              [0.0108, 0.0108,-0.002]])
    picpoints = []
    realpoints = []
    
    if bluecircels is not None:
        picpoints.append([bluecircels[0][0], bluecircels[0][1]])
        realpoints.append([realPositions[0][0], realPositions[0][1], realPositions[0][2]])
    else:
        print("point blue button not detected for pnp")
    if redcircels is not None:
        picpoints.append([redcircels[0][0], redcircels[0][1]])
        realpoints.append([realPositions[1][0], realPositions[1][1], realPositions[1][2]])
    else:
        print("point red button not detected for pnp")
    """
    if redconnector is not None:
        picpoints.append([redconnector[0][0], redconnector[0][1]])
        realpoints.append([realPositions[2][0], realPositions[2][1], realPositions[2][2]])
    else:
        print("point red connector not detected for pnp")
    
    if redM5 is not None:
        picpoints.append([redM5[0][0], redM5[0][1]])
        realpoints.append([realPositions[3][0], realPositions[3][1], realPositions[3][2]])
    else:
        print("point red M5 not detected for pnp")
    """

    for i in range(np.shape(blackcircels)[0]):
        #print("balckcircles " + str(blackcircels[i]))
        if np.isnan(blackcircels[i][0]) == False:
            picpoints.append([blackcircels[i][0], blackcircels[i][1]])
            realpoints.append([realPositions[i+4][0], realPositions[i+4][1], realPositions[i+4][2]])
        else:
            print("black circle " + str(i+4) + " not detected for pnp")


    outPicPositions = np.array(picpoints)
    outRealPositions = np.array(realpoints)

    if np.shape(outPicPositions)[0] == np.shape(outRealPositions)[0]:
        return outPicPositions, outRealPositions
    else:
        print("worng array dimensions")

def caculatePosition2Base(pic, real, image):
    Kmat = np.array([[6.59120557e+03, 0.00000000e+00, 2.72294039e+03],[ 0.00000000e+00, 6.59485693e+03, 1.83543881e+03], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
    Dmat = np.array([0.0,0.0,0.0,0.0,0.0])

    #rvecCam = np.array([1.99538456e+000, 1.05741672e+250, -1.77548583e-003])
    #cam position ?? good
    #rvecCam = np.array([1.99538456e+000, 0, 0])
    #tvecCam = np.array([5.78500449e+000, -5.80422371e+002,  9.39442875e+002])

    #matCam =  np.array([[-0.99968356, -0.021374,    0.00264996,-0.01135023],[-0.02138098,  0.99973277,  0.00644169,  0.57806456],[-0.0027896,   0.00639422, -0.99988333,  0.92307161],[ 0., 0., 0., 1.        ]])
    matCam =  np.array([[-9.99818480e-01, -1.87744792e-02, -3.24429211e-03, -2.85266318e-04],[-1.87919161e-02,  9.99808669e-01,  5.43045494e-03, -5.86241360e-01],[ 3.14171741e-03,  5.49043567e-03, -9.99979992e-01,  9.09037476e-01],[ 0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
    _, rvec, tvec = cv2.solvePnP(real, pic, Kmat, Dmat, useExtrinsicGuess= False, flags = cv2.SOLVEPNP_EPNP)
    #cv2.drawFrameAxes(image, Kmat, Dmat, rvec, tvec, length = 0.1, thickness=10)

    _, rvecTask, tvecTask = cv2.solvePnP(real, pic, Kmat, Dmat, rvec= rvec, tvec=tvec, useExtrinsicGuess= True ,flags = cv2.SOLVEPNP_ITERATIVE)

    cv2.drawFrameAxes(image, Kmat, Dmat, rvecTask, tvecTask, length = 0.1, thickness=5)

    realCorners = np.array([[0.0,0.0,0], [0.0,0.152,0], [0.254,0.152,0], [0.254,0.0,0]])
    imgCorners, _ = cv2.projectPoints(realCorners, rvecTask, tvecTask, Kmat, Dmat)
    imgCorners = imgCorners.astype(int)
    cv2.line(image,imgCorners[0][0],imgCorners[1][0], [0,255,255],5)
    cv2.line(image,imgCorners[1][0],imgCorners[2][0], [0,255,255],5)
    cv2.line(image,imgCorners[2][0],imgCorners[3][0], [0,255,255],5)
    cv2.line(image,imgCorners[3][0],imgCorners[0][0], [0,255,255],5)
    #print("tvec " + str(tvecTask) + " rvec " +str(rvecTask))
   

    # Taskboard
    rotmatTask, _ = cv2.Rodrigues(rvecTask)
    transmatTask = tvecTask
    matTask = np.eye(4)
    matTask[:3,:3] = rotmatTask
    matTask[:3,3:] = transmatTask
    #matTask = np.linalg.inv(matTask)

    # Camera
    #rotmatCam, _ = cv2.Rodrigues(np.reshape(rvecCam,(3,1))) 
    #print(rotmatCam)
    #transmatCam = np.reshape(tvecCam, (3,1))
    #matCam = np.eye(4)
    #matCam[:3, :3] = rotmatCam
    #matCam[:3, 3:] = transmatCam
    #matCam = np.linalg.inv(matCam)

    #print(matTask)
    #print(matCam)

    #M = matCam @ matTask
   
    #print(M)

    return matTask



    """
        for i in range(np.shape(points)[0]):
            
            if np.isnan(points[i]) == True:
                
            else:
                #outPicPositions = np.append(outPicPositions, [points[i][0], points[i][1]], axis= 0)
                #outRealPositions = np.append(outRealPositions, realPositions[i][:], axis = 0)
                outPicPositions.append([points[i][0], points[i][1]])
                outRealPositions.append([realPositions[i][0], realPositions[i][1], realPositions[i][2]])
                
    else:
        print("worng array dimensions")
        newPic = np.array(outPicPositions)
        newReal = np.array(outRealPositions)

    return newPic, newReal
    """

def BoardDetetction():
    idsImage = bridge.imgmsg_to_cv2(idsBuf, desired_encoding='passthrough')
    loadImage = idsImage.copy()
    

    bluecircle = findBlueBnt(idsImage)
    #print("blue button " +str(bluecircle))
    redcircle, redconnector = findRedCircels(idsImage)
    #print("red button " +str(redcircle))
    #find the M5 and Diplay more precise
    redM5 = findRedM5(idsImage)      
    #Caculate Mask
    angle = caculateRotation(bluecircle, redcircle, redM5, idsImage)
    maskedImage = caculateMask(bluecircle, redcircle, angle, loadImage)
   
    blackcircels = findBlaCircels(idsImage, maskedImage, bluecircle, redcircle)

    picPoints, realworldPoints = caculateRealPoints(bluecircle, redcircle, redconnector, blackcircels)
    #print(picPoints)
    #print(realworldPoints)
    M = caculatePosition2Base(picPoints, realworldPoints, idsImage)
    
    #cv2.imshow("result", cv2.resize(idsImage, (1362, 923)))
    #cv2.waitKey(10)

    return M, angle
    #return Transform(translation=np.Vector3(*trans),rotation=np.Quaternion(*quat))
                

if __name__ == "__main__":
    #Setup Ros
    rospy.init_node('board_detection')
    #rospy.Subscriber("/camera/color/image_raw", sensor_msgs.msg.Image, realsenseImgCallback)
    #rospy.Subscriber("/task_board", TransformStamped, tcpPoseCallback)
    idsBuf = None
    rospy.Subscriber("/ids/rgb", sensor_msgs.msg.Image, idsImgCallBack)
    s = rospy.Service("/board_detection", GetBoardLocation, service_callback)
    
    bridge = cv_bridge.CvBridge()

    #br = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(20)
    #Setup variables
    #triangle = True
    #start = True
    #Observation = False
    #Detection = False
    #Cropimage = False
    
    cnt = 0
    cntTrue = 0
    ids = True
    detection = False
    if(detection == False):
        rospy.spin() #spin infinitely and wait for callbacks
    while(not rospy.is_shutdown()):
        if ids == True and detection == True:
            if(idsBuf is not None):
                M = BoardDetetction()

            #     # Calibrate camera with taskboard
                
            #     # trans = transformStamped_to_numpy(tfBuffer.lookup_transform('base', 'task_board', rospy.Time()))
            #     # trans[2,3] -= 0.009 
            #     # camBase = trans @ np.linalg.inv(M)
            #     # t = TransformStamped()
            #     # t.header.stamp = rospy.Time.now()
            #     # t.header.frame_id = 'base'
            #     # t.child_frame_id = 'ids_cam'
            #     # t.transform = ros_numpy.msgify(Transform, camBase)
            #     # addTf = rospy.ServiceProxy('/store_tf', AddTf2)
            #     # addTf.call(t, False)
            #     # exit()
            #     # t.child_frame_id = 'task_board_pnp'
            #     # transl = transformations.translation_from_matrix(M)
            #     # rot = transformations.quaternion_from_matrix(M)
            #     #t.transform = ros_numpy.msgify(Transform, M) 
            #     #print("pose: " + str(pose))
                
            #     cam = transformStamped_to_numpy(tfBuffer.lookup_transform('base', 'ids_cam', rospy.Time()))
            #     cam[2,3] += 0.009 
                
            #     tb  = cam @ M

            #     tb_euler = Rotation.from_matrix(tb[:3,:3]).as_euler("zyx")
            #     #print(tb_euler)
            #     tb_euler[1] = 0
            #     tb_euler[2] = 0
            #     tb[:3,:3] = Rotation.from_euler("zyx", tb_euler).as_matrix()
            #     tb[2][3] = 0.092
            #     #print(tb)

            #     t = TransformStamped()
            #     t.header.stamp = rospy.Time.now()
            #     t.header.frame_id = 'base'
            #     t.child_frame_id = 'task_board'
            #     t.transform = ros_numpy.msgify(Transform, tb)
            #     addTf = rospy.ServiceProxy('/store_tf', AddTf2)
            #     addTf.call(t, False)
            #     exit()
            # else:
                #print("No ids image")
                continue

        elif ids == False and detection == True:
                    
            for i in range(0,60):
                j = i/2
                #loadImage = cv2.imread('/home/fe-n346/Documents/Pics/4_triangle_ids/' + str(j) + '.png')
                loadImage = cv2.imread('/home/fe-n346/Documents/Pics/10_all_board_pics/' + str(i) + '.png')
                idsImage = loadImage.copy()
                print("Image " + str(i))


                bluecircle = findBlueBnt(idsImage)
                #print("blue button " +str(bluecircle))
                redcircle, redconnector = findRedCircels(idsImage)
                #print("red button " +str(redcircle))
                #find the M5 and Diplay more precise
                #redM5 = findRedM5(idsImage)      
                redM5 = None
                #Caculate Mask
                angle = caculateRotation(bluecircle, redcircle, redM5, idsImage)

                maskedImage = caculateMask(bluecircle, redcircle, angle, loadImage)

                blackcircels = findBlaCircels(idsImage, maskedImage, bluecircle, redcircle)
                
                picPoints, realworldPoints = caculateRealPoints(bluecircle, redcircle, redconnector, redM5, blackcircels)

                M = caculatePosition2Base(picPoints, realworldPoints)


                cv2.imshow("result", cv2.resize(idsImage, (1000, 680)))
                cv2.waitKey(10)
                

                #point = redcircle[0][:2] + np.array([18, -75])
                #cv2.circle(idsImage, (int(point[0]), int(point[1])), 3, (255,0,0), 3)

                if bluecircle is not None and redcircle is not None and redM5 is not None and redconnector is not None and blackcircels is not None:
                    cntTrue = cntTrue + 1

                cnt = cnt + 1
                             
                

                if i == 59:
                    print("Sucessrate: " + str(cntTrue/cnt))
                    exit()
                
            
        rate.sleep()