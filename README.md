# Accurate landing system

This system is based on visually tracking a target and sending commands to Ardupilot in order to steer aircraft towards it.

## Running the simulation

```sh
# Launch Gazebo and ROS
./launch_gazebo.sh

# Launch Ardupilot SITL
sim_vehicle.py

# Launch tracker script
python3 __main__.py --simulator
```

## Running with pixhawk

```sh
# Launch tracker script
python3 __main__.py
```
