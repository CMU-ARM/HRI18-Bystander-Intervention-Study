#!/bin/bash

behavior='Invalid'
empathy='Invalid'
ROS_DIRECTORY=~/Dev/cozmo_ws
PACKAGE_PATH=~/Dev/cozmo_ws/src/HRI18-Bystander-Intervention-Study

#enter participant ID
echo "Enter Participant ID:"
read PARTICIPANT_ID
BAG_PATH=~/Bags/$PARTICIPANT_ID
mkdir -p $BAG_PATH


#select the conditions
echo "Select Robot Behavior"
PS3='Please Enter Response: '
options=("n - None" "s - Shutdown" "r - Responsive")
select opt in "${options[@]}"
do
    case $opt in
        "n - None")
            behavior='none'
            break
            ;;
        "s - Shutdown")
            behavior="shutdown"
            break
            ;;
        "r - Responsive")
            behavior="response"
            break
            ;;
        *) echo invalid option;;
    esac
done

echo "enable Robot Empathy?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) empathy="True"; break;;
        No ) empathy="False";break;;
    esac
done

echo "Comfirming Settings"
COMMAND="rosrun bystander_intervention_study study.py _empathy_flag:=$empathy _bully_reaction:=$behavior"
echo -e "\x1b[0;31m $COMMAND \x1b[0m"
echo "Do you wish to run it?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) exit;;
    esac
done
S_NAME="STUDY_SESSION"

tmux new-session -d -s $S_NAME "echo window-1 pane-1; nodejs $PACKAGE_PATH/learning_server/server.js $BAG_PATH" 
tmux  split-window -t $S_NAME -h\;\
    send-keys -t $S_NAME:0.1 source\ $ROS_DIRECTORY/devel/setup.bash C-m \;\
    send-keys -t $S_NAME:0.1 roslaunch\ bystander_intervention_study\ base.launch\ bag_path:=$BAG_PATH C-m \;\
    
echo "\n RUNNING BACKGROUND PROCESS \n"
sleep 15
source $ROS_DIRECTORY/devel/setup.bash
rostopic list | grep kinect


echo "Is the phone black? and has the SDK screen on?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) exit;;
    esac
done

echo "Do you wish to run the main program?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) exit;;
    esac
done    

tmux new-window -t $S_NAME -n cozmo_launch 
tmux send-keys -t cozmo_launch source\ $ROS_DIRECTORY/devel/setup.bash Enter
tmux send-keys -t cozmo_launch roslaunch\ bystander_intervention_study\ cozmo.launch\ bag_path:=$BAG_PATH Enter

tmux split-window -t cozmo_launch -h
tmux send-keys -t cozmo_launch.1 source\ $ROS_DIRECTORY/devel/setup.bash Enter
tmux send-keys -t cozmo_launch.1 sleep\ 5 C-m Enter
tmux send-keys -t cozmo_launch.1 cd\ $ROS_DIRECTORY/src/bystander_intervention_study/scripts Enter
tmux send-keys -t cozmo_launch.1 rosrun\ bystander_intervention_study\ study.py\ _empathy_flag:=$empathy\ _bully_reaction:=$behavior C-m Enter
tmux attach-session -t $S_NAME