# Bystander Intervention Study
COPYRIGHT(C) 2018 - CMU Assitive Robots Lab - Code released under MIT.\
Contact - Zhi - zhi.tan@ri.cmu.edu

This repository contains the code that ran the user study described in *Inducing Bystander Interventions During Robot Abuse with Social Mechanisms* by Tan et al. (2018). Due to the experimental nature of the work, the code does not meet industry coding guidelines / best practices and would need modifications to work on different machines. We provided a brief guide and pointers on how the system work and requires hardwares.

### Hardware
- ANKI Cozmo Robot
- Smartphone that can run Cozmo App (We used an Iphone 5C in the original study)
- Tablet (Samsung Galaxy Tab 10.1 in the original study)
- Micosoft Kinect 2
- Laptop with Ubuntu 16.04 (i7 Lenova laptop in the original study)

### Software/ROS Dependencies
- ROS Kinetic
- Nodejs
- [CMU_ARM/lab_ros_speech_to_text](https://github.com/CMU-ARM/lab_ros_speech_to_text)
- [CMU_ARM/lab_ros_aruco](https://github.com/CMU-ARM/lab-ros-aruco)
- [iai_kinect2](https://github.com/code-iai/iai_kinect2)
- [audio_common](http://wiki.ros.org/audio_common)
- [our heavily modified cozmo_driver](https://github.com/CMU-ARM/cozmo_driver)

You will also need a google cloud account that has speech to text API enabled. 

### Installation
1. Install ROS and all the dependencies above.
2. Install all python packages listed in the [requirement.txt](requirement.txt)
3. Download this ROS package.

### Basic Instructions on Running

1. use roslaunch to start the base launch file. `roslaunch bystander_intervention_study base.launch`
    - This starts the kinect, rosbridge and other dependencies.
2. use rosparam to set the conditions
    - the parameter `bully_reaction` is a string that sets to the types of responses:`none`,`shutdown` and `emotional`.
    - the parameter `empathy_flag` is a bool that when set to `true` makes the empathetic during the task
3. use roslaunch to start the cozmo launch file. `roslaunch bystander_intervention_study cozmo.launch`
    - This starts the robot.
4. Start the nodejs server that runs the learning application with `nodejs learning_server/server.js`
5. start the study by running the python script, `study.py` with `rosrun bystander_intervention_study study.py`
6. You should be able to connect to the learning application at `128.0.0.1:3000` on your local machine.

We included a startup script,`start.sh` that we used in our original study to start the program. There are several hard links in the script that will need to be modified to work correctly. 
