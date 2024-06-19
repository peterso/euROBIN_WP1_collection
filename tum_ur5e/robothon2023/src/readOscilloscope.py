'''
    @file readOscilloscope.py
    @authors Adrian Müller (adrian.mueller@study.thws.de), 
            Maximilian Hornauer (maximilian.hornauer@study.thws.de),
            Usama Ali (usama.ali@study.thws.de)
    @brief Program to read data from an Oscilloscope and detect beep lengths
    @version 0.1
    @date 2023-03-29

    @copyright Copyright (c) 2023
'''

import usbtmc
import math
import time
import matplotlib.pyplot as plt
import numpy as np
import rospy
from robothon2023.srv import GetBeep, GetBeepResponse

def serviceCB(data):
    global beepStatus
    return GetBeepResponse(beepStatus)

def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    return (cumsum[N:] - cumsum[:-N]) / float(N)

def detectBeep(sig):
    trigger = 4.8
    beeps = []
    #print(sig)
    t = time.time()
    
    falling = np.flatnonzero((sig[:-1] > trigger) & (sig[1:] < trigger))+1
    rising = np.flatnonzero((sig[:-1] < trigger) & (sig[1:] > trigger))+1
    
    # for i in range(len(sig)-1):
    #     if(sig[i] > trigger and sig[i+1] < trigger):
    #         edges.append(i)

    for i in range(min(len(falling), len(rising))):
        beeps.append((falling[i], rising[i]))
    return beeps

def classifyBeep(beepL):
    classifiedList = []
    for b in beepL:
        if(b < 0.02):
            classifiedList.append(0)
        elif(b <0.15):
            classifiedList.append(1)
        elif(b < 0.2):
            classifiedList.append(2)
        elif(b < 0.6):
            classifiedList.append(3)
        else:
            classifiedList.append(0)
    #print(classifiedList)
    return classifiedList

def checkInList(A, B):
    for i in range(len(B)-len(A)+1):
        for j in range(len(A)):
            if B[i + j] != A[j]:
                break
        else:
            return True
    return False

def initOszi():
    device = usbtmc.Instrument(0x0699, 0x0364) # VendorId and ProductId of Tektronix TDS 2002B

    try:
        print("Opened device: " + device.ask("*IDN?"))
    except Exception as e: #sometimes this will read a old buffered message
        print (e)
    device.write("HEADER 0") #no headers with response

    device.write("DATa:Source CH1") #set source
    device.write("DATa:ENCdg RPBinary") #set encoding to binary data
    device.write("Data:Width 1") #set data width to 1 byte
    
    return device

def readPreamble(device):
    preamble = device.ask("WFMPre?").split(";")
    numberPoints = int(preamble[5])
    xInc = float(preamble[8])
    yParams = np.array(preamble[12:15], float) #mult, zero, off
    xSteps = np.arange(numberPoints)*xInc
    
    return numberPoints, xSteps, yParams

def readData(device, numPoints, yParams):
    
    numDigits = int(math.log10(numPoints))+1
    device.write("Curve?")
    return ((np.array(list(device.read_raw()[numDigits+2:-1])) - yParams[2]) * yParams[0]) + yParams[1] #P198 Programmers Manual: ((data-offset)*mult)+zero

if __name__ == "__main__":
    rospy.init_node('oscilloscope_node')
    beepStatus = -1
    s = rospy.Service('/beep_detection', GetBeep, serviceCB)
    device = initOszi()
    
    numPoints, xSteps, yParams = readPreamble(device)
    data = readData(device, numPoints, yParams)
   
    #plt.ion()
    #graph = plt.plot(xSteps, data)[0]

    #plt.ylim(((0- yParams[2]) * yParams[0]) + yParams[1], ((255 - yParams[2]) * yParams[0]) + yParams[1])
    #plt.xlabel("Time in s")
    #plt.ylabel("Voltage in V")
    #plt.title(device.ask("*IDN?"))
    lastTime = time.perf_counter()
    
    while(not rospy.is_shutdown()):
        if(time.perf_counter() - lastTime > 5): ##check and update params
            yParams = np.array(device.ask("WFMPre:YMUlt?;YZEro?;YOFf?;").split(";"),float)
            #plt.ylim(((0- yParams[2]) * yParams[0]) + yParams[1], ((255 - yParams[2]) * yParams[0]) + yParams[1])
            xInc = float(device.ask("WFMPre:XINcr?"))
            xSteps = np.arange(numPoints)*xInc
            #graph.set_xdata(xSteps)
            #plt.xlim(0, numPoints*xInc)
            lastTime = time.perf_counter()
        
        data = readData(device, numPoints, yParams)
        #print(data)
        if(np.count_nonzero(data<1) > 100): # turned off
            print("Device off")
            beepStatus = 0
            #graph.set_xdata(xSteps)
            #graph.set_ydata(data)

        else:
            mean = running_mean(data, 10)
            beeps = detectBeep(mean)
            beepLengths = []
            for b in beeps:
                beepLengths.append(xSteps[b[1]]-xSteps[b[0]])
            classifiedBeeps = classifyBeep(beepLengths)
            #print(classifiedBeeps)
            if(classifiedBeeps == [1]):
                beepStatus = 3
                print("WORKING FINE!")
            if(checkInList([3,1,1], classifiedBeeps)):
                beepStatus = 2
                print("GPU ERROR!")
            if(checkInList([2,2], classifiedBeeps)):
                beepStatus = 1
                print("RAM ERROR!")
        rospy.sleep(0.5)
            #graph.set_xdata(xSteps[5:-4])
            #graph.set_ydata(mean)
        #plt.draw()
        #plt.pause(0.01)

