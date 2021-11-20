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

# ROS:

## Frequently used commands:

Must run every tab:

$ cd ~/catkin_ws/

$ export TURTLEBOT3_MODEL=waffle_pi

$ catkin_make

$ source devel/setup.bash

$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

## Setting up environment

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

# Implementation

https://git.shefcompsci.org.uk/acb18ga/team13.git



Task 5 - how i created an action server:
- Created a .action file in action folder using touch
- Copied standard action variables into this file
- Create python files for action server and client, being sure not to use "action" in their names
- chmod +x them to make exe
- Open up and paste code into them, change permissions in vscode
- Open up the CMakeLists.txt file and under add_action_files add the name of the action file
- Inside CMakeLists.txt under catkin_package -> LIBRARIES, add the name of the action server library name that is to be imported, if it is not there already
- Run catkin_make from catkin_ws to complie everything
- Change to server
- Add this line:
	from team13.msg import task2Feedback, task2Result, task2Action
	     [packageName].msg  [actionFileName]Feedback, [actionFileName]Result, [actionFileName]Action
- Under init, inside SimpleActionServer, change name of topic to "/topic_name", and [name]Action
- At bottom, change name of node 
- Switch to client
- In the imports, change to that of the action server
- In init, change name of node
- In init, change name of topic to that of the server 
- In init, change [name]Action
- Set goals in send_goal() (you could do this later)
- Create launch file in launch folder
- Copy and paste launch code into there
- Make sure launch file points to server and client



Action client sends:
- goal; sendGoal(), sendGoalAndWait()
- cancel goal
- cancel all goals

Action server sends:
- state
- status
- feedback 
- result


Current goal is to make a simple action server client that we can copy and paste for future uses. Robot should move a few paces, spin, print out some stuff and stop.

Finish writing comments
Implement robot behaviours
Sort out git
Sort out wiki
Revise master plan
