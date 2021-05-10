import cv2
import argparse
from operator import xor
import time
import configparser
import atexit

config = configparser.ConfigParser()


def read_config():
    with open('tracker.cfg', 'r') as configfile:
        config.read_file(configfile)


def write_config():
    with open('tracker.cfg', 'w') as configfile:
        config.write(configfile)


def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-s', '--source',
                    help='Choose either camera or simulator as source',
                    required=True,
                    choices=['camera', 'simulator'])
    args = vars(ap.parse_args())

    return args


def callback(value):
    pass


def setup_trackbars():
    cv2.namedWindow("Trackbars", 0)

    initial_iso = config['camera'].getint('iso')
    cv2.createTrackbar("ISO", "Trackbars", initial_iso, 800, callback)
    initial_shutter_speed = config['camera'].getint('shutter_speed')
    cv2.createTrackbar("SHUTTER_SPEED", "Trackbars",
                       initial_shutter_speed, 10000, callback)
    for i in ["MIN", "MAX"]:

        for j in "HSV":
            initial_threshold = config['threshold'].getint(f'{j}_{i}')
            cv2.createTrackbar(f'{j}_{i}', "Trackbars",
                               initial_threshold, 255, callback)


def get_threshold_values():
    values = []

    for i in ["MIN", "MAX"]:
        for j in "HSV":
            v = cv2.getTrackbarPos(f'{j}_{i}', "Trackbars")
            config.set('threshold', f'{j}_{i}', str(v))
            values.append(v)

    return values


def process_image(image, camera=None):
    v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_threshold_values()

    if camera:
        iso = cv2.getTrackbarPos("ISO", "Trackbars")
        camera.iso = iso
        config.set('camera', 'iso', str(iso))

        shutter_speed = cv2.getTrackbarPos("SHUTTER_SPEED", "Trackbars")
        camera.shutter_speed = shutter_speed
        config.set('camera', 'shutter_speed', str(shutter_speed))

    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    threshold = cv2.inRange(
        hsv_frame, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

    preview = cv2.bitwise_and(image, image, mask=threshold)
    cv2.imshow("Preview", preview)
    cv2.imshow("Original", image)
    cv2.imshow("Threshold", threshold)

    cv2.waitKey(5)


def main():
    read_config()
    atexit.register(write_config)
    args = get_arguments()

    setup_trackbars()

    if args['source'] == 'camera':

        from imutils.video.pivideostream import PiVideoStream
        videoStream = PiVideoStream(resolution=(
            640, 480), framerate=40).start()
        camera = videoStream.camera
        # Wait for automatic gain control to settle
        time.sleep(2)
        camera.exposure_mode = 'spotlight'
        awb_gains = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = awb_gains

        while True:
            image = videoStream.read()
            process_image(image, camera)

    elif args['source'] == 'simulator':
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import rospy

        def ros_image_callback(ros_image, bridge):
            image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
            process_image(image)

        # Initialize the ros node
        rospy.init_node('cv_bridge_node', anonymous=True)
        bridge = CvBridge()

        # Subscribe to image topic
        image_topic = '/webcam/image_raw'
        rospy.Subscriber(image_topic, Image,
                         ros_image_callback, callback_args=(bridge))
        print(f'Subscribed to {image_topic}')
        rospy.spin()


if __name__ == '__main__':
    main()
