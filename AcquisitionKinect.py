import ctypes
import _ctypes
import sys
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import numpy as np
import cv2
import mediapipe as mp
import time
import mediapipe as mp

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread


class AcquisitionKinect():
    # Create a constructor to initialize different types of array and frame objects
    def __init__(self, resolution_mode=1.0):
        self.resolution_mode = resolution_mode

        self._done = False

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_Depth)

        # here we will store skeleton data
        self._bodies = None
        self.body_tracked = False
        self.joint_points = np.array([])
        self.joint_points3D = np.array([])
        self.joint_points_RGB = np.array([])
        self.joint_state = np.array([])

        self._frameRGB = None
        self._frameDepth = None
        self._frameDepthQuantized = None
        self._frameSkeleton = None
        self.frameNum = 0

    def get_frame(self, frame):
        self.acquireFrame()
        frame.ts = int(round(time.time() * 1000))

        self.frameNum += 1

        frame.frameRGB = self._frameRGB
        frame.frameDepth = self._frameDepth

        frame.frameDepthQuantized = self._frameDepthQuantized
        frame.frameSkeleton = self._frameSkeleton

    def getDepth(self, results):

        z = 0
        print(results.multi_hand_landmarks)
        if results.multi_hand_landmarks != None:
            print("entered")
            # for handLms in results.multi_hand_landmarks:
            # mpDraw.draw_landmarks(image,handLms,mpHands.HAND_CONNECTIONS)
            joints = results.multi_hand_landmarks[0]
            joint_points_depth = self._kinect.body_joints_to_depth_space(joints)
            x = round(int(joint_points_depth[PyKinectV2.JointType_SpineMid].x))
            y = round(int(joint_points_depth[PyKinectV2.JointType_SpineMid].y))
            z = int(self._depth[y * 512 + x])
        return z

    # Get a color frame object
    def get_color_frame(self):
        self._frameRGB = self._kinect.get_last_color_frame()
        self._frameRGB = self._frameRGB.reshape((1080, 1920, -1)).astype(np.uint8)
        self._frameRGB = cv2.resize(self._frameRGB, (0, 0), fx=1 / self.resolution_mode, fy=1 / self.resolution_mode)

    # Acquire the type of frame required
    def acquireFrame(self):
        if self._kinect.has_new_color_frame():
            self.get_color_frame()

    def close(self):
        self._kinect.close()
        self._frameDepth = None
        self._frameRGB = None
        self._frameSkeleton = None