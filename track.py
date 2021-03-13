import rospy
import sys
import cv2

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class track():
    def __init__(self):
        # Initialize the ros node
        rospy.init_node("cv_bridge_node", anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # Bridge is required to convert ROS message to cv2 image
        self.bridge = CvBridge()

        # Subscribe to image topic
        image_topic = "/webcam/image_raw"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to {image_topic}")

    def image_callback(self, ros_image):
        # Convert ROS message to cv2 image
        frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

        cv2.imshow("input", frame)
        processed_image = self.process_image(frame)

        cv2.imshow("processed image", processed_image)
        # waitKey is necessary for imshow to work
        cv2.waitKey(5)

    def process_image(self, frame):
        # Convert to grayscale
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Blur the image
        grey = cv2.blur(grey, (7, 7))

        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        return edges

    def cleanup(self):
        rospy.loginfo("Shutting down tracking")
        cv2.destroyAllWindows()


def main(args):
    track()
    rospy.spin()


main(sys.argv)
