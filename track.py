import numpy as np
import rospy
import sys
from datetime import datetime
from math import sqrt
import cv2
import csv

from pymavlink import quaternion
from dronekit import connect
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetModelState
from cv_bridge import CvBridge


class Tracker:
    def __init__(self):
        # Connect to vehicle
        self.vehicle = connect('127.0.0.1:14551', wait_ready=True)
        print("Connected to vehicle")
        # Initialize the ros node
        rospy.init_node("cv_bridge_node", anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # Bridge is required to convert ROS message to cv2 image
        self.bridge = CvBridge()

        self.target = None
        self.normalized_target_horizontal = None
        self.normalized_target_vertical = None

        self.target_height_offset = -60  # Raise the aiming point

        # Subscribe to image topic
        image_topic = "/webcam/image_raw"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to {image_topic}")

        # Subscribe to odometry topic (position)

        # Create telemetry file
        current_datetime = datetime.now()
        timestamp = current_datetime.strftime("%Y-%m-%d_%H:%M:%S")
        self.telemetry_file = open(f'telemetry/{timestamp}.csv', mode='w')
        fieldnames = ['timestamp', 'normalized_target_horizontal',
                      'normalized_target_vertical', 'distance_to_target']
        self.telemetry = csv.DictWriter(self.telemetry_file, fieldnames)
        self.telemetry.writeheader()

    def image_callback(self, ros_image):
        # Convert ROS message to cv2 image
        frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # cv2.imshow("input", frame)
        processed_image = self.process_image(frame)

        cv2.imshow("processed image", processed_image)

        get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        aircraft_position = get_model_state("zephyr", "")

        aircraft_distance_to_target = sqrt(
            aircraft_position.pose.position.x ** 2 + aircraft_position.pose.position.y ** 2)

        # Write telemetry
        self.telemetry.writerow(
            {
                "timestamp": datetime.now(),
                "normalized_target_horizontal": self.normalized_target_horizontal,
                "normalized_target_vertical": self.normalized_target_vertical,
                "distance_to_target": aircraft_distance_to_target
            })
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
            self.target = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # Draw largest contour
            # frame = cv2.drawContours(
            #     frame, [largest_contour], 0, (0, 255, 0), 2)
            # Draw a dot on target center
            frame = cv2.circle(frame, self.target, 5, (255, 0, 0), -1)
        else:
            self.target = None
            self.normalized_target_horizontal = None
            self.normalized_target_vertical = None
            frame = cv2.putText(frame, "No target", (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        frame = self.draw_crosshair(frame)

        if self.target is not None:
            (frame_height, frame_width) = frame.shape[:2]
            frame_width_middle = int(frame_width / 2)
            frame_height_middle = int(frame_height / 2)
            target_horizontal, target_vertical = self.target

            self.normalized_target_horizontal = (
                target_horizontal - frame_width_middle) / frame_width_middle

            if target_vertical <= frame_height_middle + self.target_height_offset:
                divider = frame_height_middle + self.target_height_offset
            else:
                divider = frame_height_middle - self.target_height_offset

            self.normalized_target_vertical = (
                target_vertical - frame_height_middle - self.target_height_offset) / divider

            # Rotate towards target
            roll_limit = 20
            pitch_limit = 10
            target_size = cv2.contourArea(largest_contour)
            throttle = 0.3 if target_size > 400 else 0.5

            frame = cv2.putText(frame, f"Throttle: {throttle}", (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            self.send_set_attitude_target(
                roll=exp_root(
                    3/5, self.normalized_target_horizontal) * roll_limit,
                pitch=-exp_root(3/5, self.normalized_target_vertical) * pitch_limit, thrust=throttle)
            frame = self.draw_input(
                frame,
                normalized_roll=exp_root(
                    3/5, self.normalized_target_horizontal),
                normalized_pitch=-exp_root(3/5, self.normalized_target_vertical))

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
        crosshair_width = 15
        frame = cv2.line(frame,
                         (frame_width_middle - crosshair_width,
                          frame_height_middle + self.target_height_offset),
                         (frame_width_middle + crosshair_width,
                          frame_height_middle + self.target_height_offset), (0, 255, 0), 1)
        frame = cv2.line(frame,
                         (frame_width_middle, frame_height_middle -
                          crosshair_width + self.target_height_offset),
                         (frame_width_middle, frame_height_middle +
                          crosshair_width + self.target_height_offset), (0, 255, 0), 1)
        return frame

    def draw_input(self, frame, normalized_roll=0, normalized_pitch=0):
        (frame_height, frame_width) = frame.shape[:2]
        frame_width_middle = int(frame_width / 2)
        frame_height_middle = int(frame_height / 2)
        box_width = 40
        frame = cv2.rectangle(frame,
                              (frame_width_middle - box_width,
                               frame_height_middle - box_width + self.target_height_offset),
                              (frame_width_middle + box_width,
                                  frame_height_middle + box_width + self.target_height_offset), (0, 255, 0), 1)
        frame = cv2.circle(frame,
                           (frame_width_middle + int(normalized_roll * box_width), frame_height_middle +
                            self.target_height_offset - int(normalized_pitch * box_width)), 2, (255, 0, 255), -1)
        return frame

    def cleanup(self):
        rospy.loginfo("Shutting down tracking")
        self.telemetry_file.close()
        cv2.destroyAllWindows()


def exp_root(exp, x):
    if x >= 0:
        return x**(exp)
    elif x < 0:
        return -(abs(x)**(exp))


def main(args):
    Tracker()
    rospy.spin()


main(sys.argv)
