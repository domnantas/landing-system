from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
import time
import cv2

fps_counter = FPS()
videoStream = PiVideoStream(
    resolution=(640, 480), framerate=40, sensor_mode=4).start()
camera = videoStream.camera

time.sleep(2)

# awb_gains = camera.awb_gains
# camera.awb_mode = 'off'
# camera.awb_gains = awb_gains
fps_counter.start()

while True:
    frame = videoStream.read()
    fps_counter.update()
    fps_counter.stop()
    frame = cv2.putText(frame, f"{fps_counter.fps()}",
                        (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('camera', frame)
    cv2.waitKey(5)
