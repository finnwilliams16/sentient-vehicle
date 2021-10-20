# sentient-vehicle

Controlling a vehicle using brain waves

## High level:
EEG -> Computer -> Robot Onboard Computer -> Vehicle 

## Low level:
EEG live data -> ROS Subsystem -> ML API -> ROS Subsystem -> Onboard controllers 

ML API:
- Must be able to take processed EEG data as input 
- Must be able to run inference on EEG data within a suitable time frame 
- Must be able to return result with a suitable degree of accuracy 

ROS Subsystem:
- Must take as input EEG stream data
- Must preprocess data and pass this onto the ML API 
- Must retrieve the results from the ML API, interpret these, and send the equivalent response to the hardware 
- Should implement core robotic behaviours 

Robot:
- Must implement basic commands, including and not limited to: forward, back, left, right 


Ideas:
- ROS System preprocessing of EEG data; may be unnecessary to pass in data to ML model in real time. Instead, sample at a certain frequency (eg. 2hz), such as by using the average value, and send this. The faster the model, the higher the possible frequency.
- ROS System robotic behaviours; idea to implement Braitenburgh's vehicles to allow for collision detection 
- Heavy documentation and wiki on git 
- LED on robot to tell user if EEG is connected; red for catching an error, amber for connecting, green for connected.
- Onboard computer will need to run roscore and package as well as the EEG streamer 
- Study tflite and edge computing to optimise calls to ML model 

Prototype design:
1. Demonstrate full solution within Gazebo simulations
2. Implement on real robot; use an Arduino or raspberry pi to connect to computer 
3. Migrate all systems onto onboard computer 

Tasks I can do right now:
- Write git 
- Find and train a model on EEG data
- Set up a simulation and select robot
- Write ROS subsystem completely
- Implement basic robot behaviours 
- Run inference on model by sampling from the EEG distribution
- Research which controls to use, and the relationship between thoughts and actions
