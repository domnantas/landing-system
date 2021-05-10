from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
import atexit
import time
import cv2

fps_counter = FPS()


def handle_exit():
    fps_counter.stop()
    print(f'FPS: {fps_counter.fps()}')


atexit.register(handle_exit)

videoStream = PiVideoStream(
    resolution=(640, 480), framerate=40, sensor_mode=4).start()
camera = videoStream.camera

time.sleep(2)

awb_gains = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = awb_gains

fps_counter.start()
print('Video started')

while True:
    frame = videoStream.read()
    fps_counter.update()

    cv2.imshow('camera', frame)
    cv2.waitKey(1)
