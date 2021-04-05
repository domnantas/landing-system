from oled import TrackerOled
from color_tracker import ColorTracker
import cv2
from threading import Thread

tracker_oled = TrackerOled()
color_tracker = ColorTracker()


def write_fps():
    tracker_oled.writeTextCenter("FPS: {:.2f}".format(color_tracker.fps.fps()))


tracker_oled.writeTextCenter("READY")
while True:
    color_tracker.processFrame()

    # t = Thread(target=write_fps, args=(), daemon=True)
    # t.start()
    (frame_height, frame_width, frame_channels) = color_tracker.frame.shape
    if color_tracker.center is not None:
        (point_x, point_y) = color_tracker.center
        draw_x = int(round(point_x * tracker_oled.oled.width / frame_width))
        draw_y = int(round(point_y * tracker_oled.oled.height / frame_height))
        tracker_oled.drawPoint(draw_x, draw_y)
    # else:
    #     tracker_oled.writeTextCenter("NOT FOUND")
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        color_tracker.release()
        out.release()
        tracker_oled.clearDisplay()
        break
