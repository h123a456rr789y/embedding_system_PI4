#!/usr/bin/python
#+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#|R|a|s|p|b|e|r|r|y|P|i|.|c|o|m|.|t|w|
#+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
# Copyright (c) 2014, raspberrypi.com.tw
# All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
# Author : sosorry
# Date   : 05/31/2015
# Origin : http://blog.miguelgrinberg.com/post/video-streaming-with-flask

import cv2
from imutils.video.pivideostream import PiVideoStream
import imutils
import time

class Camera(object):
    def __init__(self):
        # self.video = cv2.VideoCapture(0)
        
        self.vs = PiVideoStream().start()
        # frame = vs.read()
        # frame = imutils.resize(frame, width=400)
        
        #self.flip = flip
        time.sleep(2.0)

        #self.video = cv2.VideoCapture(1)
        #self.video.set(PROP_FRAME_WIDTH, 640)
        #self.video.set(PROP_FRAME_HEIGHT, 480)
        #self.video.set(PROP_FRAME_WIDTH, 320)
        #self.video.set(PROP_FRAME_HEIGHT, 240)
    
    def __del__(self):
        self.vs.stop()
    
    def get_frame(self):
        frame = self.vs.read()
        frame = imutils.resize(frame, width=400)
        text = 'Hello, OpenCV!'
        frame = cv2.putText(frame,text,(50,150),cv2.FONT_HERSHEY_COMPLEX,50,(0,0,255),25) 
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()

