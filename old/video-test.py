# import the necessary packages
from collections import deque
from imutils.video import VideoStream
from imutils.video import FileVideoStream
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from argparse import ArgumentParser
import numpy as np
import cv2
import imutils
import time

# construct the argument parse and parse the arguments
ap = ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
args = vars(ap.parse_args())

# if a video path was supplied, we're working with a video file
isProcessingVideoFile = args.get("video", False)

# define the lower and upper boundaries of the target in the HSV color space, then initialize the list of tracked points
# TODO: Make GUI sliders, enabled with a flag
targetLowerBoundary = (0, 0, 70)
targetUpperBoundary = (180, 108, 255)
pointsBuffer = deque(maxlen=64)

if isProcessingVideoFile:
    # grab the reference to video file
    videoStream = FileVideoStream(args["video"]).start()
else:
    # otherwise, start Pi camera capture on separate thread
    videoStream = PiVideoStream().start()

# allow the camera or video file to warm up
time.sleep(2.0)
# start calculating fps
fps = FPS().start()

while True:
    # grab the current frame
    frame = videoStream.read()
    # handle the frame from VideoCapture or VideoStream
    # if not videoStream.more():
    #     break
    # frame = frame[1] if args.get("video", False) else frame

    # if args.get("video", False):
    #     frame = frame[1]

    # if we are viewing a video and we did not grab a frame, then we have reached the end of the video
    if frame is None:
        break
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=400)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    # hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "uv", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    # mask = cv2.inRange(hsv, targetLowerBoundary, targetUpperBoundary)
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    # cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    # cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    # center = None
    # only proceed if at least one contour was found
    # if len(cnts) > 0:
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    # c = max(cnts, key=cv2.contourArea)
    # ((x, y), radius) = cv2.minEnclosingCircle(c)
    # M = cv2.moments(c)
    # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    # only proceed if the radius meets a minimum size
    # if radius > 10:
    # draw the circle and centroid on the frame,
    # then update the list of tracked points
    # cv2.circle(frame, (int(x), int(y)), int(radius),
    #    (0, 255, 255), 2)
    #         cv2.circle(frame, center, 5, (0, 0, 255), -1)
    # # update the points queue
    # pointsBuffer.appendleft(center)
    # # loop over the set of tracked points
    # for i in range(1, len(pointsBuffer)):
    #     # if either of the tracked points are None, ignore
    #     # them
    #     if pointsBuffer[i - 1] is None or pointsBuffer[i] is None:
    #         continue
    #     # otherwise, compute the thickness of the line and
    #     # draw the connecting lines
    #     thickness = int(np.sqrt(64 / float(i + 1)) * 2)
    #     cv2.line(frame, pointsBuffer[i - 1],
    #              pointsBuffer[i], (171, 26, 118), thickness)
    # update fps counter
    fps.update()
    fps.stop()
    # write fps on the frame
    cv2.putText(frame, "FPS: {:.2f}".format(
        fps.fps()), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
videoStream.stop()
# close all windows
cv2.destroyAllWindows()
