from argparse import ArgumentParser
from dronekit import connect

argument_parser = ArgumentParser(
    description='Target tracker for precision landing system')
argument_parser.add_argument(
    '--simulator', action='store_true', help='run tracker in Gazebo simulation mode')


def main():
    args = argument_parser.parse_args()

    vehicle_address = '127.0.0.1:14551' if args.simulator else '/dev/serial0'
    print(f'Connecting to {vehicle_address}')
    vehicle = connect(vehicle_address, wait_ready=True, baud=57600)
    print('Connected to vehicle')

    # if (args.simulator):


main()
