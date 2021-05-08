#!/usr/bin/env python
# -*- coding: utf-8 -*-

# USAGE: You need to specify a filter and "only one" image source
#
# (python) range-detector --filter RGB --image /path/to/image.png
# or
# (python) range-detector --filter HSV --webcam

import cv2
import argparse
from operator import xor
import time


def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filter', required=True,
                    help='Range filter. RGB or HSV')
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('-i', '--image',
                        help='Path to the image')
    source.add_argument('-w', '--webcam',
                        help='Use webcam',
                        action='store_true')
    source.add_argument('-s', '--simulator',
                        help='Use camera input from Gazebo',
                        action='store_true',)
    args = vars(ap.parse_args())

    if not args['filter'].upper() in ['RGB', 'HSV']:
        ap.error("Filter should be either RGB or HSV.")

    return args


def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    cv2.createTrackbar("SHUTTER_SPEED", "Trackbars", 5000, 10000, callback)
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def process_image(image, range_filter, camera=None):
    v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(
        range_filter)

    if camera:
        pass
        # camera.shutter_speed = cv2.getTrackbarPos("SHUTTER_SPEED", "Trackbars")

    if range_filter == 'RGB':
        frame_to_thresh = image.copy()
    else:
        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    thresh = cv2.inRange(
        frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

    preview = cv2.bitwise_and(image, image, mask=thresh)
    cv2.imshow("Preview", preview)
    cv2.imshow("Original", image)
    cv2.imshow("Thresh", thresh)

    cv2.waitKey(5)


def main():
    args = get_arguments()

    range_filter = args['filter'].upper()

    setup_trackbars(range_filter)

    if args['image']:
        image = cv2.imread(args['image'])

        process_image(image, range_filter)
    elif args['webcam']:
        from imutils.video.pivideostream import PiVideoStream
        videoStream = PiVideoStream(resolution=(
            640, 480), framerate=20).start()
        camera = videoStream.camera
        # Wait for automatic gain control to settle
        time.sleep(2)
        # camera.shutter_speed = 4000
        camera.exposure_mode = 'spotlight'
        camera.iso = 800
        awb_gains = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = awb_gains

        while True:
            image = videoStream.read()
            process_image(image, range_filter, camera)

    elif args['simulator']:
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
