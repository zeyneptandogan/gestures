import cv2
import numpy as np
from AcquisitionKinect import AcquisitionKinect
from Frame import Frame
import HandTrackingClass as handtrack
import pyautogui

detector = handtrack.handDetector()
kinect = AcquisitionKinect()
frame = Frame()
firstPosition=0
indicator=False
tryOther=False
while True:
    # take frame
    kinect.get_frame(frame)
    kinect.get_color_frame()
    image = kinect._frameRGB
    image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    hands, img = detector.findHands(image)
    lmList, bbox = detector.findPosition(image)
    tryOther=False
    if hands:
        hand1 = hands[0]
        lmList1 = hand1["lmList"]  # List of 21 Landmark points
        bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
        centerPoint1 = hand1['center']  # center of the hand cx,cy
        handType1 = hand1["type"]  # Handtype Left or Right
        print(handType1)
        fingers1 = detector.fingersUpForMultiple(hand1)
        if len(hands) == 2:
            # Hand 2
            hand2 = hands[1]
            lmList2 = hand2["lmList"]
            bbox2 = hand2["bbox"]
            centerPoint2 = hand2['center']
            handType2 = hand2["type"]
            fingers2 = detector.fingersUpForMultiple(hand2)

        if lmList1:
            if (fingers1[1] == 1 and fingers1[2] == 1 and fingers1[3] == 1 and fingers1[0] == 0 and fingers1[4] == 0 and indicator == False):
                firstPosition = lmList1[16][0]
                remaining = firstPosition
                indicator = True
                print("first pos:", firstPosition)

            elif(firstPosition!=0):
                        print("for one hand")
                        print(firstPosition)
                        print(lmList1[16][0])
                        if (firstPosition - lmList1[16][0] > 500 and handType1=="Left"):
                            pyautogui.press('right')
                            firstPosition = 0
                            indicator = False
                            print("right clicked!!")
                        elif (lmList1[16][0] - firstPosition> 500 and handType1=="Right"):
                            pyautogui.press('left')
                            firstPosition = 0
                            indicator = False
                            print("left clicked!!")
            else:
                tryOther=True

        if len(hands) == 2 and lmList2 and tryOther==True:
            if (fingers2[1] == 1 and fingers2[2] == 1 and fingers2[3] == 1 and fingers2[0] == 0 and fingers2[4] == 0 and indicator == False):
                firstPosition = lmList2[16][0]
                remaining = firstPosition
                indicator = True
                print("first pos:", firstPosition)

            elif (firstPosition != 0):
                print("for two hand")
                print(firstPosition)
                print(lmList2[16][0])
                if (firstPosition - lmList2[16][0] > 500 and handType2 == "Left"):
                    pyautogui.press('right')
                    firstPosition = 0
                    indicator = False
                    print("right clicked!!")
                elif (lmList2[16][0] - firstPosition > 500 and handType2 == "Right"):
                    pyautogui.press('left')
                    firstPosition = 0
                    indicator = False
                    print("left clicked!!")
            tryOther=False

    cv2.imshow("Sliding Operation", image)
    cv2.waitKey(1)

