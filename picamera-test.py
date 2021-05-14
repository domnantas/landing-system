from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import FPS
import atexit
import time
import cv2

fps_counter = FPS()


def handle_exit():
    fps_counter.stop()
    print(f'FPS: {fps_counter.fps()}')


atexit.register(handle_exit)

camera = PiCamera(
    resolution=(320, 240), framerate=40, sensor_mode=4)
rawCapture = PiRGBArray(camera, size=(320, 240))

time.sleep(2)

awb_gains = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = awb_gains

fps_counter.start()
print('Video started')

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    fps_counter.update()
    rawCapture.truncate(0)

    cv2.imshow('camera', image)
    cv2.waitKey(1)
