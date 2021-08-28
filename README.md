# underwater_rov_main

## Setup
1. Install ROS melodic
2. Install mavros
3. Clone and build this repo
```
mkdir uw_rov_catkin_ws
cd uw_rov_catkin_ws
git clone git@github.com:Sea-Swindlers/underwater_rov_main.git src
catkin build
```

## Running
```
roslaunch mavros apm.launch 
rosservice call /mavros/set_stream_rate 0 10 1 # To have apm publish messages to ros.
```

## Dependencies
`pip3 install roboticstoolbox-python`

## Useful Commands
```
rosrun mavros mavsafety arm
rosrun mavros mavsys mode -c MANUAL
rosrun mavros mavsys mode -c STABILIZE
roslaunch mavros_extras teleop.launch teleop_args:="-rc"
```
## Network Setup
On the jetson:
```
export ROS_MASTER_URI=http://192.168.4.104:11311
```
On the laptop:
```
export ROS_MASTER_URI=http://192.168.4.104:11311
export ROS_IP=192.168.4.104 # or whatever your laptop ip is
```
