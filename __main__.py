from argparse import ArgumentParser
from pymavlink import quaternion
from datetime import datetime
from imutils.video import FPS
from dronekit import connect
from simple_pid import PID
from math import sqrt
import configparser
import numpy as np
import atexit
import time
import csv
import cv2
import os

argument_parser = ArgumentParser(
    description='Target tracker for precision landing system')
argument_parser.add_argument('-s', '--source',
                             help='Choose either camera or simulator as source',
                             required=True,
                             choices=['camera', 'simulator'])
argument_parser.add_argument(
    '--telemetry', action='store_true', help='track aircraft telemetry')
argument_parser.add_argument(
    '--record', action='store_true', help='record processed video')
argument_parser.add_argument(
    '--no-vehicle', action='store_true', help='do not connect to a vehicle')

config = configparser.ConfigParser()


def read_config():
    with open('tracker.cfg', 'r') as configfile:
        config.read_file(configfile)


project_dir = os.path.dirname(__file__)


class Tracker:
    def __init__(self, source, telemetry=False, record=False, no_vehicle=False):

        if no_vehicle:
            self.vehicle = False
        else:
            self.init_vehicle(source)

        self.normalized_target = None
        self.aircraft_distance_to_target = None
        self.roll_input = 0
        self.pitch_input = 0

        self.fps_counter = FPS()

        self.camera_fps = 35
        self.camera_resolution = (320, 240)

        self.init_control_PID()

        self.telemetry_enabled = telemetry
        if self.telemetry_enabled:
            self.init_telemetry()

        self.record_enabled = record
        if self.record_enabled:
            self.init_record()

        atexit.register(self.handle_exit)

        if source == 'simulator':
            self.track_gazebo()
        elif source == 'camera':
            self.track_pixhawk()

    def init_vehicle(self, source):
        vehicle_address = '127.0.0.1:14551' if source == 'simulator' else '/dev/serial0'
        print(f'Connecting to {vehicle_address}')
        self.vehicle = connect(vehicle_address, wait_ready=False, baud=57600)
        print('Connected to vehicle')

    def init_telemetry(self):
        # Filename is curent timestamp
        current_datetime = datetime.now()
        timestamp = current_datetime.strftime('%Y-%m-%d_%H:%M:%S')

        telemetry_path = os.path.join(
            project_dir, f'telemetry/{timestamp}.csv')
        telemetry_file = open(telemetry_path, mode='w')
        fieldnames = ['timestamp', 'normalized_target_horizontal',
                      'normalized_target_vertical', 'distance_to_target']
        self.telemetry = csv.DictWriter(telemetry_file, fieldnames)
        self.telemetry.writeheader()

    def init_record(self):
        # Filename is curent timestamp
        current_datetime = datetime.now()
        timestamp = current_datetime.strftime('%Y-%m-%d_%H:%M:%S')

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        recording_path = os.path.join(
            project_dir, f'recordings/{timestamp}.avi')
        self.video_writer = cv2.VideoWriter(
            recording_path, fourcc, self.camera_fps, self.camera_resolution)

    def init_control_PID(self):
        self.roll_pid = PID(1.2, 0.07, 0.05, setpoint=0,
                            output_limits=(-1.0, 1.0))
        self.pitch_pid = PID(0.6, 0.1, 0.05, setpoint=0,
                             output_limits=(-1.0, 1.0))

    def track_gazebo(self):
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
        rospy.spin()

    def ros_image_callback(self, ros_image, bridge):
        self.find_distance_to_target()
        # Convert ROS message to cv2 image
        frame = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        self.track(frame)

    def track_pixhawk(self):
        from imutils.video.pivideostream import PiVideoStream
        from picamera.array import PiRGBArray
        from picamera import PiCamera

        camera = PiCamera(resolution=self.camera_resolution,
                          framerate=self.camera_fps, sensor_mode=4)
        rawCapture = PiRGBArray(camera, size=self.camera_resolution)
        # videoStream = PiVideoStream(
        #     resolution=self.camera_resolution, framerate=self.camera_fps, sensor_mode=4).start()
        # camera = videoStream.camera
        camera.exposure_mode = 'spotlight'
        # Wait for automatic gain control to settle
        time.sleep(2)
        awb_gains = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = awb_gains

        camera.shutter_speed = config['camera'].getint('shutter_speed')
        camera.iso = config['camera'].getint('iso')

        self.fps_counter.start()

        print('Starting tracking')

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            self.fps_counter.update()
            self.track(image)
            rawCapture.truncate(0)
        # while True:
        #     frame = videoStream.read()
        #     self.fps_counter.update()
        #     self.track(frame)

    def track(self, frame):
        self.find_target(frame)

        hud_frame = self.draw_crosshair(frame.copy())

        if self.normalized_target:
            self.control_aircraft()
            hud_frame = self.draw_target(hud_frame)
            hud_frame = self.draw_input(hud_frame)

        if self.record_enabled:
            self.video_writer.write(hud_frame)

        cv2.imshow('camera', hud_frame)
        if self.telemetry_enabled:
            self.write_telemetry()

        # waitKey is necessary for imshow to work
        cv2.waitKey(1)

    def find_target(self, frame):
        # Blur the image to filter out high frequency noise
        # blurred_frame = cv2.GaussianBlur(frame, (11, 11), 0)
        # Convert to HSV colorspace because it makes tresholding based on color easier
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Create a mask for selected HSV color
        h_min = config['threshold'].getint('h_min')
        s_min = config['threshold'].getint('s_min')
        v_min = config['threshold'].getint('v_min')
        target_min_threshold = (h_min, s_min, v_min)
        h_max = config['threshold'].getint('h_max')
        s_max = config['threshold'].getint('s_max')
        v_max = config['threshold'].getint('v_max')
        target_max_threshold = (h_max, s_max, v_max)
        white_threshold = cv2.inRange(hsv_frame, (0, 0, 253), (255, 255, 255))
        color_threshold = cv2.inRange(
            hsv_frame, target_min_threshold, target_max_threshold)
        mask = cv2.bitwise_or(white_threshold, color_threshold)
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
        else:
            # Reset PID
            self.roll_pid(0)
            self.pitch_pid(0)

            if self.normalized_target is not None:
                print('Target lost')
                self.normalized_target = None

    def control_aircraft(self):
        normalized_target_horizontal, normalized_target_vertical = self.normalized_target

        throttle = (-normalized_target_vertical + 2) * 0.2
        self.roll_input = self.roll_pid(normalized_target_horizontal)
        self.pitch_input = self.pitch_pid(normalized_target_vertical)

        roll_limit = 20
        pitch_limit = 10

        if self.vehicle:
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
        import rospy
        from gazebo_msgs.srv import GetModelState

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
        half_box_width = 40
        frame = cv2.rectangle(frame,
                              (frame_width_middle - half_box_width,
                               frame_height_middle - half_box_width),
                              (frame_width_middle + half_box_width,
                               frame_height_middle + half_box_width), (0, 255, 0), 1)
        frame = cv2.circle(frame,
                           (frame_width_middle - int(self.roll_input * half_box_width),
                            frame_height_middle - int(self.pitch_input * half_box_width)),
                           2, (255, 0, 255), -1)
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

    def handle_exit(self):
        self.fps_counter.stop()
        print(f'FPS: {self.fps_counter.fps()}')


def main():
    args = argument_parser.parse_args()

    read_config()

    Tracker(source=args.source,
            telemetry=args.telemetry,
            record=args.record,
            no_vehicle=args.no_vehicle)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
