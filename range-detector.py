import cv2
import argparse
from operator import xor
import time


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


def setup_trackbars():
    cv2.namedWindow("Trackbars", 0)

    cv2.createTrackbar("SHUTTER_SPEED", "Trackbars", 5000, 10000, callback)
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in "HSV":
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_trackbar_values():
    values = []

    for i in ["MIN", "MAX"]:
        for j in "HSV":
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def process_image(image, camera=None):
    v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values()

#     if camera:
#         camera.shutter_speed = cv2.getTrackbarPos("SHUTTER_SPEED", "Trackbars")
    
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

    setup_trackbars()

    if args['source'] == 'camera':
        from imutils.video.pivideostream import PiVideoStream
        videoStream = PiVideoStream(resolution=(
            640, 480), framerate=40).start()
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
