from argparse import ArgumentParser
from pymavlink import quaternion
from dronekit import connect
from datetime import datetime
from imutils.video import FPS
from simple_pid import PID
from math import sqrt
import numpy as np
import time
import csv
import cv2

argument_parser = ArgumentParser(
    description='Target tracker for precision landing system')
argument_parser.add_argument(
    '--simulator', action='store_true', help='run tracker in Gazebo simulation mode')
argument_parser.add_argument(
    '--record', action='store_true', help='record processed video')


class Tracker:
    def __init__(self, simulator=False, record=False):
        vehicle_address = '127.0.0.1:14551' if simulator else '/dev/serial0'
        print(f'Connecting to {vehicle_address}')
        self.vehicle = connect(vehicle_address, wait_ready=False, baud=57600)
        print('Connected to vehicle')

        self.normalized_target = None
        self.aircraft_distance_to_target = None

        self.init_telemetry()
        self.init_control_PID()
        self.fps = FPS().start()
        self.record = record

        if self. record:
            self.init_record()

        if simulator:
            self.track_gazebo()
        else:
            self.track_pixhawk()

    def init_telemetry(self):
        # Filename is curent timestamp
        current_datetime = datetime.now()
        timestamp = current_datetime.strftime('%Y-%m-%d_%H:%M:%S')

        telemetry_file = open(f'telemetry/{timestamp}.csv', mode='w')
        fieldnames = ['timestamp', 'normalized_target_horizontal',
                      'normalized_target_vertical', 'distance_to_target']
        self.telemetry = csv.DictWriter(telemetry_file, fieldnames)
        self.telemetry.writeheader()

    def init_record(self):
        # Filename is curent timestamp
        current_datetime = datetime.now()
        timestamp = current_datetime.strftime('%Y-%m-%d_%H:%M:%S')

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(
            f'recordings/{timestamp}.avi', fourcc, 40.0, (640, 480))  # TODO: get resolution from frame ;-;

    def init_control_PID(self):
        self.roll_pid = PID(1.2, 0.07, 0.05, setpoint=0)
        self.pitch_pid = PID(0.6, 0.1, 0.05, setpoint=0)

    def track_gazebo(self):
        from gazebo_msgs.srv import GetModelState
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import rospy
        # Initialize the ros node
        rospy.init_node('cv_bridge_node', anonymous=True)
        bridge = CvBridge()

        # Subscribe to image topic
        image_topic = '/webcam/image_raw'
        rospy.Subscriber(image_topic, Image,
                         self.ros_image_callback, callback_args=(bridge))
        print(f'Subscribed to {image_topic}')

    def ros_image_callback(self, ros_image, bridge):
        self.find_distance_to_target()
        # Convert ROS message to cv2 image
        frame = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        self.track(frame)

    def track_pixhawk(self):
        from imutils.video.pivideostream import PiVideoStream
        videoStream = PiVideoStream(resolution=(
            640, 480), framerate=20).start()
        camera = videoStream.camera
        camera.iso = 30
        # Wait for automatic gain control to settle
        time.sleep(2)
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        awb_gains = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = awb_gains

        while True:
            frame = videoStream.read()
            # frame = imutils.resize(frame, width=400)
            self.track(frame)

    def track(self, frame):
        (frame_height, frame_width) = frame.shape[:2]
        # frame = cv2.resize(frame, (int(frame_width/2), int(frame_height/2)))

        self.find_target(frame)
        frame = self.draw_crosshair(frame)
        frame = self.draw_fps(frame)

        if self.normalized_target:
            self.control_aircraft()
            frame = self.draw_target(frame)
            frame = self.draw_input(frame)

        if self.record:
            self.video_writer.write(frame)

        cv2.imshow('camera', frame)
        self.write_telemetry()

        # waitKey is necessary for imshow to work
        cv2.waitKey(5)

    def find_target(self, frame):
        # Blur the image to filter out high frequency noise
        frame = cv2.GaussianBlur(frame, (11, 11), 0)
        # Convert to HSV colorspace because it makes tresholding based on color easier
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Create a mask for selected HSV color
        target_min_threshold = (0, 0, 10)
        target_max_threshold = (255, 160, 255)
        mask = cv2.inRange(
            frame, target_min_threshold, target_max_threshold)
        # Remove small blobs in mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # Find contours in the mask
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        if len(contours) > 0:
            if self.normalized_target is None:
                print('Target detected')
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Find center point of contour
            M = cv2.moments(largest_contour)
            target = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
            # Find normalized offsets
            (frame_height, frame_width) = frame.shape[:2]
            frame_width_middle = int(frame_width / 2)
            frame_height_middle = int(frame_height / 2)
            target_horizontal, target_vertical = target

            normalized_target_horizontal = (
                target_horizontal - frame_width_middle) / frame_width_middle
            normalized_target_vertical = (
                target_vertical - frame_height_middle) / frame_height_middle

            self.normalized_target = [
                normalized_target_horizontal,
                normalized_target_vertical
            ]
        elif self.normalized_target is not None:
            print('Target lost')
            self.normalized_target = None
            # Reset PID
            self.init_control_PID()

    def control_aircraft(self):
        normalized_target_horizontal, normalized_target_vertical = self.normalized_target
        throttle = (-normalized_target_vertical + 2) * 0.2

        self.roll_input = self.roll_pid(normalized_target_horizontal)
        self.pitch_input = self.pitch_pid(normalized_target_vertical)

        roll_limit = 20
        pitch_limit = 10

        self.send_set_attitude_target(
            roll=-self.roll_input * roll_limit,
            pitch=self.pitch_input * pitch_limit,
            thrust=throttle
        )

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

    def find_distance_to_target(self):
        get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        aircraft_position = get_model_state("zephyr", "")

        self.aircraft_distance_to_target = sqrt(
            aircraft_position.pose.position.x ** 2 + aircraft_position.pose.position.y ** 2)

    def draw_crosshair(self, frame):
        (frame_height, frame_width) = frame.shape[:2]
        frame_width_middle = int(frame_width / 2)
        frame_height_middle = int(frame_height / 2)
        crosshair_width = 15
        frame = cv2.line(frame,
                         (frame_width_middle - crosshair_width, frame_height_middle),
                         (frame_width_middle + crosshair_width, frame_height_middle), (0, 255, 0), 1)
        frame = cv2.line(frame,
                         (frame_width_middle, frame_height_middle - crosshair_width),
                         (frame_width_middle, frame_height_middle + crosshair_width), (0, 255, 0), 1)
        return frame

    def draw_target(self, frame):
        (frame_height, frame_width) = frame.shape[:2]
        normalized_target_horizontal, normalized_target_vertical = self.normalized_target
        target_position = (int(frame_width * (normalized_target_horizontal + 1) / 2),
                           int(frame_height * (normalized_target_vertical + 1) / 2))
        frame = cv2.circle(frame, target_position, 5, (255, 0, 0), -1)
        return frame

    def draw_input(self, frame):
        (frame_height, frame_width) = frame.shape[:2]
        frame_width_middle = int(frame_width / 2)
        frame_height_middle = int(frame_height / 2)
        box_width = 40
        frame = cv2.rectangle(frame,
                              (frame_width_middle - box_width,
                               frame_height_middle - box_width),
                              (frame_width_middle + box_width,
                               frame_height_middle + box_width), (0, 255, 0), 1)
        frame = cv2.circle(frame,
                           (frame_width_middle + int(self.roll_input * box_width),
                            frame_height_middle - int(self.pitch_input * box_width)), 2, (255, 0, 255), -1)
        return frame

    def draw_fps(self, frame):
        self.fps.update()
        self.fps.stop()
        cv2.putText(frame, f"{int(self.fps.fps())}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return frame

    def write_telemetry(self):
        if self.normalized_target:
            normalized_target_horizontal, normalized_target_vertical = self.normalized_target
        else:
            normalized_target_horizontal = None
            normalized_target_vertical = None

        self.telemetry.writerow(
            {
                "timestamp": datetime.now(),
                "normalized_target_horizontal": normalized_target_horizontal,
                "normalized_target_vertical": normalized_target_vertical,
                "distance_to_target": self.aircraft_distance_to_target
            })


def main():
    args = argument_parser.parse_args()

    Tracker(simulator=args.simulator, record=args.record)

    if args.simulator:
        rospy.spin()


main()
