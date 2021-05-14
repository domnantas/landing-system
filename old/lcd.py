# Create ST7789 LCD display class.
import time
import ST7789
import cv2
from PIL import Image
from picamera.array import PiRGBArray
from picamera import PiCamera

# Create TFT LCD display class.
disp = ST7789.ST7789(
    port=0,
    cs=ST7789.BG_SPI_CS_FRONT,  # BG_SPI_CS_BACK or BG_SPI_CS_FRONT
    dc=17,
    rst=22,
    backlight=27,               # 18 for back BG slot, 19 for front BG slot.
    # rotation=90,
    spi_speed_hz=80 * 1000 * 1000,
    offset_left=0
)

# Initialize display.
disp.begin()

width = disp.width
height = disp.height

camera = PiCamera()
camera.resolution = (disp.width, disp.height)
camera.framerate = 60


rawCapture = PiRGBArray(camera, size=camera.resolution)

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image_array = frame.array
    # show the frame
    cv2.imshow("Frame", image_array)
    print("asd")

    # image = Image.fromarray(image_array)
    # disp.display(image)
    rawCapture.truncate(0)
