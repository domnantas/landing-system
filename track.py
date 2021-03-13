import numpy as np
import rospy
import sys
import time
import cv2

from pymavlink import quaternion
from dronekit import connect
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Tracker:
    def __init__(self):
        # Connect to vehicle
        self.vehicle = connect('127.0.0.1:14551', wait_ready=True)
        rospy.loginfo("Connected to vehicle")
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

        # cv2.imshow("input", frame)
        processed_image = self.process_image(frame)

        cv2.imshow("processed image", processed_image)
        # waitKey is necessary for imshow to work
        cv2.waitKey(5)

    def process_image(self, frame):
        # Blur the image to filter out high frequency noise
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        # Convert to HSV colorspace because it makes tresholding based on color easier
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Create a mask for selected HSV color
        target_min_threshold = (0, 10, 10)
        target_max_threshold = (20, 255, 255)
        mask = cv2.inRange(hsv, target_min_threshold, target_max_threshold)
        # Remove small blobs in mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # Find contours in the mask
        contours, hierarchy = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Find center point of contour
            M = cv2.moments(largest_contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # Draw largest contour and center of it
            frame = cv2.drawContours(
                frame, [largest_contour], 0, (0, 255, 0), 2)
            frame = cv2.circle(frame, center, 5, (255, 0, 0), -1)

        frame = self.draw_crosshair(frame)

        return frame

    def send_set_attitude_target(self, roll=0, pitch=0, yaw=0, thrust=0.5):
        attitude = [np.radians(roll), np.radians(pitch), np.radians(yaw)]
        attitude_quaternion = quaternion.QuaternionBase(attitude)

        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, 0, 0,  # time_boot_ms, target_system, target_component
            0b10111000,  # type_mask https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET_TYPEMASK
            attitude_quaternion,
            0, 0, 0,  # Rotation rates (ignored)
            thrust  # Between 0.0 and 1.0
        )

        self.vehicle.send_mavlink(msg)

    def draw_crosshair(self, frame):
        (frame_height, frame_width) = frame.shape[:2]
        frame_width_middle = int(frame_width / 2)
        frame_height_middle = int(frame_height / 2)
        crosshair_offset = 15
        frame = cv2.line(frame,
                         (frame_width_middle - crosshair_offset, frame_height_middle),
                         (frame_width_middle + crosshair_offset, frame_height_middle), (0, 255, 0), 1)
        frame = cv2.line(frame,
                         (frame_width_middle, frame_height_middle - crosshair_offset),
                         (frame_width_middle, frame_height_middle + crosshair_offset), (0, 255, 0), 1)
        return frame

    def cleanup(self):
        rospy.loginfo("Shutting down tracking")
        cv2.destroyAllWindows()


def main(args):
    Tracker()
    rospy.spin()


main(sys.argv)
