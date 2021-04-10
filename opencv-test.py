cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 40)
start = time.time()
for i in range(400):
    ret, img = cap.read()
print("Time for {0} frames: {1} seconds".format(frames, time.time() - start))
