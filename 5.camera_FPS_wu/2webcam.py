# import the necessary packages
from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS

import imutils
import time
import cv2
import datetime

try:
    # created a *threaded *video stream, allow the camera sensor to warmup,
    # and start the FPS counter
    print("[INFO] sampling frames from WebcamVideoStream module...")
    vs = WebcamVideoStream(src=0).start()
    time.sleep(2.0)
    fps = FPS().start()
    oldtime = datetime.datetime.now()

    # loop over some frames...this time using the threaded stream
    while True:
        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # update the FPS counter
        fps.update()
        newtime = datetime.datetime.now()
        nowfps = 1 / (newtime - oldtime).total_seconds()
        oldtime = newtime

        cv2.putText(frame,str(nowfps),(20,200),cv2.FONT_HERSHEY_PLAIN,10,(255,0,0),1)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

except KeyboardInterrupt:
    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    # do a bit of cleanup
    cv2.destroyAllWindows()
    vs.stop()