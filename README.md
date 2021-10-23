# sentient-vehicle

## Controlling a vehicle using brain waves

High level:

EEG -> Computer -> Robot Onboard Computer -> Vehicle 

Low level:

EEG live data -> ROS Subsystem -> ML API -> ROS Subsystem -> Onboard controllers 

ML API:
- Must be able to take processed EEG data as input 
- Must be able to run inference on EEG data within a suitable time frame 
- Must be able to return result with a suitable degree of accuracy 
- Could run model on the edge

ROS Subsystem:
- Must take as input EEG stream data
- Must preprocess data and pass this onto the ML API 
- Must retrieve the results from the ML API, interpret these, and send the equivalent response to the hardware 
- Should implement core robotic behaviours 

Robot:
- Must implement basic commands, including and not limited to: forward, back, left, right 
- Should feature an LED to tell user if EEG is connected; red for catching an error, amber for connecting, green for connected

Ideas:
- ROS System preprocessing of EEG data; may be unnecessary to pass in data to ML model in real time. Instead, sample at a certain frequency (eg. 2hz), such as by using the average value, and send this. The faster the model, the higher the possible frequency.
- ROS System robotic behaviours; idea to implement Braitenburgh's vehicles to allow for collision detection 
- Onboard computer will need to run roscore and package as well as the EEG streamer 
- Study tflite and edge computing to optimise calls to ML model 

Prototype design:
1. Demonstrate full solution within Gazebo simulations
2. Implement on real robot; use an Arduino or raspberry pi to connect to computer 
3. Migrate all systems onto onboard computer 

Tasks:
- ~~Write git~~
- Review BCI EEG data
- Review models
- ~~Set up a simulation and select robot~~
- Write ROS subsystem completely
- Implement basic robot behaviours 
- Run inference on model by sampling from the EEG distribution
- Research which controls to use, and the relationship between thoughts and actions

## ROS:

# Frequently used commands:

Must run every tab:
$ cd ~/catkin_ws/
$ export TURTLEBOT3_MODEL=waffle_pi
$ catkin_make
$ source devel/setup.bash

$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Setting up environment

sudo apt-get install ros-melodic-catkin

source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/

catkin_make

(inside catkin_ws/src) $ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

source devel/setup.bash

Ran following commands from https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/:

$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers \
  ros-melodic-dynamixel-sdk ros-melodic-turtlebot3 ros-melodic-turtlebot3-msgs \
  ros-melodic-dynamixel-sdk

$ export TURTLEBOT3_MODEL=waffle_pi

$ catkin_create_pkg sentient_vehicle std_msgs rospy roscpp
