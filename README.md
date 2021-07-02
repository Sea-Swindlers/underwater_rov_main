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

## Useful Commands
```
rosrun mavros mavsafety arm
rosrun mavros mavsys mode -c MANUAL
rosrun mavros mavsys mode -c STABILIZE
```
