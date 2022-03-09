import cv2
import numpy as np
from pynput.mouse import Button, Controller
import wx
from AcquisitionKinect import AcquisitionKinect
from Frame import Frame
import HandTrackingClass as handtrack
import pyautogui
import time

wCam, hCam =1920, 1080
frameR = 200     #Frame Reduction
smoothening = 7  #random value
plocX, plocY = 0, 0
clocX, clocY = 0, 0
movX, movY= 0, 0
clicked=False
globalClock=0

detector = handtrack.handDetector()
#mouse=Controller()
kinect = AcquisitionKinect()
frame = Frame()
#app=wx.App(False)
#(wScr,hScr) =  wx.GetDisplaySize()
(wScr,hScr) = pyautogui.size()
#print( wScr, hScr)
count=1

while True:
    # take frame
    kinect.get_frame(frame)
    kinect.get_color_frame()
    image = kinect._frameRGB
    image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)

    hands, img = detector.findHands(image)
    lmList, bbox = detector.findPosition(image, draw=False)
    if len(lmList) != 0:
        x1, y1 = lmList[8][1:]
        x2, y2 = lmList[12][1:]

        fingers = detector.fingersUp()
        cv2.rectangle(img, (frameR, frameR), (wCam - frameR, hCam - frameR),
                      (255, 0, 255), 2)

        # Step4: Only Index Finger: Moving Mode
        if fingers[1] == 1 and fingers[2] == 0 and clicked==False:
            # Step5: Convert the coordinates
            globalClock=0
            clicked = False
            x3 = np.interp(x1, (frameR, wCam - frameR), (0, wScr))
            y3 = np.interp(y1, (frameR, hCam - frameR), (0, hScr))

            # Step6: Smooth Values
            clocX = plocX + (x3 - plocX) / smoothening
            clocY = plocY + (y3 - plocY) / smoothening

            # Step7: Move Mouse
            #mouse.move(wScr - clocX, clocY)
          #  print(wScr-clocX)
            pyautogui.moveTo(clocX, clocY)
            cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
            plocX, plocY = clocX, clocY
            movX, movY = clocX, clocY
            # Step8: Both Index and middle are up: Clicking Mode
        elif fingers[1] == 1 and fingers[2] == 1 and clicked==False:

            # Step9: Find distance between fingers
            length, lineInfo, img = detector.findDistance(8, 12, img)

            # Step10: Click mouse if distance short
            if length < 40:
                cv2.circle(img, (lineInfo[4], lineInfo[5]), 15, (0, 255, 0), cv2.FILLED)
                pyautogui.click()
                clicked=True

        elif fingers[0] == 0 and fingers[1] == 0 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:
            print("hold")
            count = count + 1
            if clicked == False:
                print("first click with hold")
                print("clicked:",clicked)
                pyautogui.moveTo(movX, movY)
                #pyautogui.click()
                pyautogui.mouseDown()
                clicked = True
            else:
                print("hold drag")
                start = time.time()
                x3 = np.interp(x1, (frameR, wCam - frameR), (0, wScr))
                y3 = np.interp(y1, (frameR, hCam - frameR), (0, hScr))
                # Step6: Smooth Values
                clocX = plocX + (x3 - plocX) / smoothening
                clocY = plocY + (y3 - plocY) / smoothening
                pyautogui.moveTo(clocX, clocY)
                cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
                plocX, plocY = clocX, clocY
                end = time.time()
                globalClock = globalClock + (end - start)

        elif clicked == True:
            print("leave")
            #pyautogui.dragTo(clocX, clocY, globalClock, button='left')
            pyautogui.mouseUp(button='left', x=clocX, y=clocY)
            print("measuredTime: ",globalClock)
            clicked = False
            globalClock = 0

    cv2.imshow("Click-Panning Operation", image)
    cv2.waitKey(1)

