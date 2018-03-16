#!/bin/bash

behavior='Invalid'
empathy='Invalid'

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

tmux new-session -d -s $S_NAME "echo window-1 pane-1; nodejs ~/Dev/ros_ws/src/bystander_intervention_study/learning_server/server.js $BAG_PATH"
tmux  split-window -t $S_NAME -h\;\
    send-keys -t $S_NAME:0.1 source\ ~/Dev/ros_ws/devel/setup.bash C-m \;\
    send-keys -t $S_NAME:0.1 roslaunch\ bystander_intervention_study\ base.launch\ bag_path:=$BAG_PATH C-m \;\
    
echo "\n RUNNING BACKGROUND PROCESS \n"
sleep 15
source ~/Dev/ros_ws/devel/setup.bash
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
tmux split-window -h -t $S_NAME:0 'source ~/Dev/ros_ws/devel/setup.bash;roslaunch bystander_intervention_study cozmo.launch' \; \
    split-window -h\;\
    send-keys -t $S_NAME:0.3 source\ ~/Dev/ros_ws/devel/setup.bash C-m \;\
    send-keys -t $S_NAME:0.3 sleep\ 5 C-m \;\
    send-keys -t $S_NAME:0.3 cd\ ~/Dev/ros_ws/src/bystander_intervention_study/scripts C-m \; \
    send-keys -t $S_NAME:0.3 rosrun\ bystander_intervention_study\ study.py\ _empathy_flag:=$empathy\ _bully_reaction:=$behavior C-m \; \
    #\n  \n $CMD \n
            #split-window -d 'echo window-1 pane-2; '
tmux attach-session -t $S_NAME
# 19
# down vote
# accepted
# tmux new -d -s my-session 'echo window-1 pane-1; sleep 8' \; \
#           split-window -d 'echo window-1 pane-2; sleep 6' \; down-pane \; \
#             new-window -d 'echo window-2;        sleep 4' \; next-window \; \
#                 attach \;
