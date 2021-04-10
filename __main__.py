from argparse import ArgumentParser
from dronekit import connect
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from datetime import datetime
import csv
from simple_pid import PID

argument_parser = ArgumentParser(
    description='Target tracker for precision landing system')
argument_parser.add_argument(
    '--simulator', action='store_true', help='run tracker in Gazebo simulation mode')


def init_gazebo():
    # Initialize the ros node
    rospy.init_node("cv_bridge_node", anonymous=True)
    # rospy.on_shutdown(self.cleanup)

    # Subscribe to image topic
    image_topic = "/webcam/image_raw"
    rospy.Subscriber(image_topic, RosImage, self.image_callback)
    rospy.loginfo(f"Subscribed to {image_topic}")

    # Bridge is required to convert ROS message to cv2 image
    bridge = CvBridge()
    return bridge


def init_telemetry():
    # Filename is curent timestamp
    current_datetime = datetime.now()
    timestamp = current_datetime.strftime("%Y-%m-%d_%H:%M:%S")

    telemetry_file = open(f'telemetry/{timestamp}.csv', mode='w')
    fieldnames = ['timestamp', 'normalized_target_horizontal',
                  'normalized_target_vertical', 'distance_to_target']
    telemetry = csv.DictWriter(telemetry_file, fieldnames)
    telemetry.writeheader()
    return telemetry


def main():
    args = argument_parser.parse_args()

    vehicle_address = '127.0.0.1:14551' if args.simulator else '/dev/serial0'
    print(f'Connecting to {vehicle_address}')
    vehicle = connect(vehicle_address, wait_ready=True, baud=57600)
    print('Connected to vehicle')

    telemetry = init_telemetry()

    roll_pid = PID(1.2, 0.07, 0.05, setpoint=0)
    pitch_pid = PID(0.6, 0.1, 0.05, setpoint=0)

    if (args.simulator):
        bridge = init_gazebo()


main()
