import cv2
import argparse
from operator import xor
import time
import configparser

def read_config(parser):
    with open('tracker.cfg', 'r') as configfile:
        parser.read_file(configfile)
        
def write_config(parser):
    with open('tracker.cfg', 'w') as configfile:
        parser.write(configfile)

def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-s', '--source',
                        help='Choose either camera or simulator as source',
                        required=True
                        choices=['camera', 'simulator'])
    args = vars(ap.parse_args())

    return args


def callback(value):
    pass


def setup_trackbars(config):
    cv2.namedWindow("Trackbars", 0)
    
    cv2.createTrackbar("ISO", "Trackbars", config['camera'].get('iso'), 800, callback)
    cv2.createTrackbar("SHUTTER_SPEED", "Trackbars", config['camera'].get('shutter_speed, 10000, callback)
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
        
        for j in "HSV":
            cv2.createTrackbar(f'{j}_{i}', "Trackbars", config['threshold'].get(f'{j}_{i}', v), 255, callback)


def get_trackbar_values():
    values = []

    for i in ["MIN", "MAX"]:
        for j in "HSV":
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def process_image(image, camera=None):
    v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values()

    if camera:
        camera.iso = cv2.getTrackbarPos("ISO", "Trackbars")
        camera.shutter_speed = cv2.getTrackbarPos("SHUTTER_SPEED", "Trackbars")
    
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    threshold = cv2.inRange(
        hsv_frame, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

    preview = cv2.bitwise_and(image, image, mask=threshold)
    cv2.imshow("Preview", preview)
    cv2.imshow("Original", image)
    cv2.imshow("Threshold", threshold)

    cv2.waitKey(5)


def main():
    config = configparser.ConfigParser()
    read_config(config)
    args = get_arguments()

    setup_trackbars(config)

    if args['source'] == 'camera':
        
        from imutils.video.pivideostream import PiVideoStream
        videoStream = PiVideoStream(resolution=(
            640, 480), framerate=40).start()
        camera = videoStream.camera
        # Wait for automatic gain control to settle
        time.sleep(2)
        # camera.shutter_speed = 4000
        camera.exposure_mode = 'spotlight'
        camera.iso = config['camera'].get('iso', 400)
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
            process_image(image, range_filter)

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
