import cv2
import numpy as np
from AcquisitionKinect import AcquisitionKinect
from Frame import Frame
import HandTrackingClass as handtrack
import pyautogui
import time

detector = handtrack.handDetector()
kinect = AcquisitionKinect()
frame = Frame()
firstTime = 0
startLength = 0
firstZoomIn = False
firstZoomOut = False

while True:
    # take frame
    kinect.get_frame(frame)
    kinect.get_color_frame()
    image = kinect._frameRGB
    image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    hands, img = detector.findHands(image)  # with draw

    if hands:
        hand1 = hands[0]
        lmList1 = hand1["lmList"]  # List of 21 Landmark points
        bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
        centerPoint1 = hand1['center']  # center of the hand cx,cy
        handType1 = hand1["type"]  # Handtype Left or Right
        #print(handType1)
        fingers1 = detector.fingersUpForMultiple(hand1)

        if len(hands) == 2:
            # Hand 2
            hand2 = hands[1]
            lmList2 = hand2["lmList"]
            bbox2 = hand2["bbox"]
            centerPoint2 = hand2['center']
            handType2 = hand2["type"]

            fingers2 = detector.fingersUpForMultiple(hand2)
            if detector.isAllUp(fingers2) and detector.isAllUp(fingers1):
                length, info, img = detector.findDistanceforTwoHands(lmList1[8], lmList2[8], img)  # with draw
                if firstTime == 0:
                    startLength = length
                    print("start", startLength)
                    firstTime = firstTime + 1
                    #time.sleep(5)
                    firstZoomIn = True
                    firstZoomOut = True
                else:
                    if length >= startLength * (105 / 100):
                        x = round((100 * (length - startLength)) / startLength)
                        if firstZoomIn:
                            print("first zoom in amount", x)
                            pyautogui.keyDown('ctrl')
                            pyautogui.scroll(x)
                            pyautogui.keyUp('ctrl')
                            firstZoomIn = False
                            prev = x
                        else:
                            print("x",x)
                            print("prev",prev)

                            newAmount = x - prev
                            if newAmount>0:
                                print("new zoom in amount", newAmount)
                                pyautogui.keyDown('ctrl')
                                pyautogui.scroll(newAmount)
                                pyautogui.keyUp('ctrl')
                                prev = newAmount

                    elif length <= startLength * (95 / 100):
                        x = round((100 * (length - startLength)) / startLength)
                        if firstZoomOut:
                            print("first zoom out amount", x)
                            pyautogui.keyDown('ctrl')
                            pyautogui.scroll(x)
                            pyautogui.keyUp('ctrl')
                            firstZoomOut = False
                            prev = x
                        else:
                            print("x",x)
                            print("prev",prev)

                            newAmount = x - prev
                            if newAmount<0:
                                print("new zoom out amount", newAmount)
                                pyautogui.keyDown('ctrl')
                                pyautogui.scroll(newAmount)
                                pyautogui.keyUp('ctrl')
                                prev = newAmount

                    elif startLength * (99 / 100) < length < startLength * (101 / 100):
                        print("stop zoom slow change operation")
                        firstTime = 0
                        startLength=0
                        firstZoomIn = False
                       # time.sleep(8)
            else:
                print("stop zoom operation")
                firstTime = 0
                startLength = 0
                firstZoomIn = False
                #time.sleep(8)

    cv2.imshow("Zoom Operation", image)
    cv2.waitKey(1)
