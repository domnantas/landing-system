import time
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 40)
start = time.time()
frames = 400

for i in range(frames):
    ret, img = cap.read()
print("Time for {0} frames: {1} seconds".format(frames, time.time() - start))
