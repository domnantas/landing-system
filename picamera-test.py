from picamera import PiCamera
from picamera.array import PiRGBArray
import time

frames = 400

camera = PiCamera(framerate=40)
time.sleep(2)
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=camera.resolution)
start = time.time()
for frame, i in zip(camera.capture_continuous(rawCapture, format="bgr", use_video_port=True), range(frames)):
    image = frame.array
    rawCapture.truncate(0)
print("Time for {0} frames: {1} seconds".format(frames, time.time()-start))
