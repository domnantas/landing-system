from collections import deque
from imutils.video import VideoStream
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from argparse import ArgumentParser
import numpy as np
import cv2
import imutils
import time


class ColorTracker:
    def __init__(self):
        ap = ArgumentParser()
        ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        args = vars(ap.parse_args())

        # if a video path was supplied, we're working with a video file
        self.isProcessingVideoFile = args.get("video", False)

        # define the lower and upper boundaries of the target in the HSV color space, then initialize the list of tracked points
        # TODO: Make GUI sliders, enabled with a flag
        self.targetLowerBoundary = (0, 0, 70)
        self.targetUpperBoundary = (180, 108, 255)
        self.pointsBuffer = deque(maxlen=64)

        if self.isProcessingVideoFile:
            # grab the reference to video file
            self.videoStream = cv2.VideoCapture(args["video"])
        else:
            # otherwise, start Pi camera capture on separate thread
            self.videoStream = PiVideoStream().start()

        # allow the camera or video file to warm up
        time.sleep(2)
        # start calculating fps
        self.fps = FPS().start()

    def processFrame(self):
        # grab the current frame
        self.frame = self.videoStream.read()
        # handle the frame from VideoCapture or VideoStream
        self.frame = self.frame[1] if self.isProcessingVideoFile else self.frame
        # if args.get("video", False):
        #     frame = frame[1]
        # if we are viewing a video and we did not grab a frame, then we have reached the end of the video
        if self.frame is None:
            return

        # resize the frame, blur it, and convert it to the HSV
        # color space
        self.frame = imutils.resize(self.frame, width=400)
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "uv", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.targetLowerBoundary,
                           self.targetUpperBoundary)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        self.center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(self.frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(self.frame, self.center, 5, (0, 0, 255), -1)
        # update the points queue
        self.pointsBuffer.appendleft(self.center)
        # loop over the set of tracked points
        for i in range(1, len(self.pointsBuffer)):
            # if either of the tracked points are None, ignore
            # them
            if self.pointsBuffer[i - 1] is None or self.pointsBuffer[i] is None:
                continue
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(64 / float(i + 1)) * 2)
            cv2.line(self.frame, self.pointsBuffer[i - 1],
                     self.pointsBuffer[i], (171, 26, 118), thickness)
        # update fps counter
        self.fps.update()
        self.fps.stop()
        # write fps on the frame
        cv2.putText(self.frame, "FPS: {:.2f}".format(
            self.fps.fps()), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # show the frame to our screen
        cv2.imshow("Frame", self.frame)

    def release(self):
        print("[INFO] elasped time: {:.2f}".format(self.fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
        # if we are not using a video file, stop the camera video stream
        if not self.isProcessingVideoFile:
            self.videoStream.stop()
        # otherwise, release the camera
        else:
            self.videoStream.release()
        # close all windows
        cv2.destroyAllWindows()
