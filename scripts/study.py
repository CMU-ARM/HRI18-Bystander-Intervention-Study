#!/usr/bin/env python

import rospy
from std_msgs.msg import (
    String,
    Float64,
    ColorRGBA,
    Int8,
    Bool
)
from sensor_msgs.msg import (
    Image,
    CameraInfo,
    BatteryState,
    Imu,
    JointState
)

from geometry_msgs.msg import (
    Pose2D
)


from success_ros_msgs.msg import Speech as Speech_msg
from cozmo_driver.msg import(
    AnimAction,
    AnimGoal
)
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import SetBool
from std_msgs.msg import String
import yaml
import os
import random
from CozmoNav import CozmoNav
from bystander_intervention_study.msg import(
    EmpathyResult,
    EmpathyAction,
    LearnStatus,
    StudyOut
)
from rospkg import RosPack
import threading


class bullyObj(object):

    def loginfo(self, txt):
        self._log_lock.acquire()
        msg = StudyOut()
        msg.header.seq = self._seq
        msg.header.stamp = rospy.Time.now()
        msg.msg = txt
        self._study_out_pub.publish(msg)
        self._seq += 1
        rospy.loginfo(txt)
        self._log_lock.release()

    def __init__(self):

        #variables
        #current joint state
        self._joint_state = None
        self._run_flag = False 
        self._playing_anim_flag = False
        self._shutdown_flag = False
        self._start = False
        self._kill_switch = False

        #the position where cozmo will be 
        self._target_pose = Pose2D()
        self._target_pose.x = 0 
        self._target_pose.y = 0.15
        #different angles (in degree, -179->180)
        self._origin_theta = 180
        self._player_1_theta = 160
        self._player_2_theta = -160
        self._target_pose.theta = self._origin_theta

        #read the configuration file
        self._config_file = None
        rp = RosPack()
        dirpath = rp.get_path('bystander_intervention_study')
        with open(os.path.join(dirpath,'res','config.yaml'),'r') as f:
            self._config_file = yaml.load(f)


        rospy.init_node('bystander_intervention_study')
        #initialize the fake study out
        self._log_lock = threading.RLock()
        self._seq = 0
        self._study_out_pub = rospy.Publisher('/study_out',StudyOut, queue_size=1)
        #add a 5 second sleep so that it will actually start logging 
        rospy.sleep(5)
        self.loginfo("starting bully study node")
        

        self._bully_reaction = rospy.get_param('~bully_reaction',default="response")
        #self._bully_reaction = rospy.get_param('bully_reaction',default="response")
        #type is shutdown/response/none
        self._empathy_flag = rospy.get_param('~empathy_flag',default=True)
        #self.anim_pub = rospy.Publisher('/exec_anim',String, queue_size=1)

        self._anim_asc = actionlib.SimpleActionClient('cozmo/exec_anim', AnimAction)
        self._behavior_asc = actionlib.SimpleActionClient('cozmo/exec_behavior',AnimAction)
        self._sleep_asc = actionlib.SimpleActionClient('cozmo/sleep_anim',AnimAction)

        self._empathy_server =actionlib.SimpleActionServer('empathy_channel', EmpathyAction, self._empathy_callback, auto_start = False)
        self._empathy_server.start()
        rospy.loginfo("waiting for behavior server")
        self._behavior_asc.wait_for_server()
        rospy.loginfo("found behavior server")
        rospy.loginfo("waiting for animation server")
        self._anim_asc.wait_for_server()
        rospy.loginfo("found animation server")
        #subscribe to channels
        rospy.Subscriber('kill_switch', Bool, self._kill_switch_callback, queue_size=1)
        rospy.Subscriber('cozmo/imu', Imu, self._imu_callback, queue_size=1)
        rospy.Subscriber('/success_google_stt/stt', Speech_msg, self._stt_callback, queue_size=1)
        #rospy.Subscriber('empathy_channel', String, self._empathy_callback, queue_size=1)
        rospy.Subscriber('study_round', LearnStatus, self._stage_update, queue_size=10)
        rospy.Subscriber('cozmo/joint_states', JointState, self._joint_callback, queue_size=1)

        #speech toggl
        rospy.wait_for_service('success_google_stt/toggle_stt')
        self._stt_toggle = rospy.ServiceProxy('success_google_stt/toggle_stt', SetBool)
        rospy.loginfo("Google STT initialized")

        self._oled_pub = rospy.Publisher('cozmo/oled_face',Image,queue_size=1)
        self._lift_pub = rospy.Publisher('cozmo/lift_height',Float64, queue_size=1)
        self._head_pub = rospy.Publisher('cozmo/head_angle', Float64, queue_size=1)
        self._say_pub = rospy.Publisher('cozmo/say',String, queue_size=1)
        #
        self._coznav = CozmoNav()
        self._coznav._agent.stop()
        self.loginfo("CozNav Initialized")
        #counter for when it was last bullied
        self._last_bullied_time = rospy.Time.now()
        self._current_stage = 0 #a numerical value that say which stage of the round we are in
        self._level = 0
        self._player = 0
        self._last_bad_word = ""
        self._last_bad_word_time = rospy.Time.now()
        # goal = AnimGoal()
        # goal.anim_name = "FindFaces"
        # self._behavior_asc.send_goal(goal)
        self._initialize()

        
        self.loginfo('system:reaction-{}-empathy-{}'.format(self._bully_reaction, self._empathy_flag))
        self.loginfo('system:system-ready')

    def _initialize(self):
        pass

    def _stage_update(self, msg):
        self._current_stage = msg.stage
        #self.loginfo('update:stage:{}'.format(msg.stage))
        self._player = msg.player
        #0 = level 1
        #1  = level 2
        #2 = level 3
        self._level = msg.level
        if self._level > 1 and not self._start:
            print(self._level)
            self._run_flag = True
            self._start = True
            self._stt_toggle(True) #start STT
            self.loginfo("system:robot-start-moving")
        #we look at the participant when it's their turn
        if(self._player == 0):
            self._target_pose.theta = self._origin_theta
        if(self._player == 1):        
            self._target_pose.theta = self._player_1_theta
        if(self._player == 2):
            self._target_pose.theta = self._player_2_theta

    def _kill_switch_callback(self, msg):
        self.loginfo("kill switch activated")
        self._kill_switch = msg.data
        self._coznav.stop()
        #also stops STT
        self._stt_toggle(False)


    def _joint_callback(self, msg):
        self._joint_state = msg
        if(self._run_flag and not self._playing_anim_flag):
            ###Disabled lift based bullying
            #if(msg.position[1] > self._config_file['lift_limit'] and self._run_flag):
            #    #self.loginfo("bullied:lift-detected")            
            #    #self._bullied()
            #    return
            if(msg.position[0] < self._config_file["head_limit"] and self._run_flag):
                self.loginfo("bullied:head-detected")            
                self._bullied()
                return 

    def _empathy_callback(self, msg):

        result = EmpathyResult()
        result.response = True

    
        if self._empathy_flag and self._start and (not self._kill_switch):
            #if shutdown, then no empathy but consider success.
            if self._shutdown_flag:
                return self._empathy_server.set_succeeded(result)
            else:
                #attempt to execute the animation
                #select behavior based on the message
                if msg.answer in self._config_file['anim']:
                    selected_anim = random.choice(self._config_file['anim'][msg.answer])
                else:
                    selected_anim = 'majorfail'
                if not self._play_anim(selected_anim):
                    #animation didn't play
                    result.response = False
                    return self._empathy_server.set_succeeded(result)
                self._recovery_behavior()         
                #return success
                return self._empathy_server.set_succeeded(result)
        else:
            #no empathy at this condition, consider success
            return self._empathy_server.set_succeeded(result)

    def _stt_callback(self, msg):

        txt = msg.text #msg.confidence
        for bad_word in self._config_file['bad_list']:
            if bad_word in txt:
                self.loginfo('bullied:text-detected:{}'.format(bad_word))

                #no response if the same bad word is said in one second
                if(bad_word == self._last_bad_word):
                    if (rospy.Time.now() - self._last_bad_word_time).to_sec() < 2:
                        return
                
                self._last_bad_word_time = rospy.Time.now()

                self._bullied()

                self._last_bad_word_time = rospy.Time.now()
                self._last_bad_word = bad_word                

                return
        #txt_arr = txt.split(' ')

        # #for word in txt_arr:
        #     if word.strip() in self._config_file['bad_list']:
        #         self.loginfo('bullied:text-detected:{}'.format(word.strip()))
        #         self._bullied()
        #         return

    def _imu_callback(self, msg):

        if(msg.linear_acceleration.z > 12 or msg.linear_acceleration.z < 5):
            #only react if the system is not playing aninmations
            if not self._playing_anim_flag:
                self.loginfo("bullied:imu-detected")
                self._bullied()


    def _play_anim(self, name):

        #do nothing if shutdown
        if self._shutdown_flag:
            return True

        # #wait for an action to finish
        # state = self._anim_asc.get_state()
        # if(state == GoalStatus.ACTIVE):
        #     self._anim_asc.wait_for_result()

        #disable bully_flag
        self._run_flag = False 
        self._playing_anim_flag = True
        self.loginfo("anim:playing animation {}".format(name))
        goal = AnimGoal()
        goal.anim_name = name

        self._anim_asc.send_goal(goal)
        #we need to wait for the animation to finish
        self._anim_asc.wait_for_result()
        result = self._anim_asc.get_result()
        #reenable bully flag
        self._run_flag = True
        self._playing_anim_flag = False
        #return whether the playing of the animation was successful
        return result.success

    def _bullied(self):

        if self._kill_switch:
            return

        #check status
        cur_time = rospy.Time.now()
        if((cur_time - self._last_bullied_time).to_sec() < self._config_file['last_bully_time']):
            return

        #will not do anything if still shutdown
        if self._shutdown_flag or (not self._start):
            return

        #record when was the last time it was shut down.
        self._last_bullied_time = cur_time
        self._coznav.stop() #stop the movement
        
        #consider bully only if those are not true
        self.loginfo("bullied:system-reacting to bullying")

        self._behavior_asc.cancel_all_goals()
        print(self._bully_reaction)
        if self._bully_reaction.startswith("shutdown"):
            self._reaction_shutdown()
        elif self._bully_reaction.startswith("response"):
            self._reaction_respond()  
        else:
            self._reaction_none()
            #do nothing
            return

    def _recovery_behavior(self):
        #restore the head and lift positions
        self._play_anim("anim_neutral_eyes_01")
        msg = Float64()
        msg.data = 0
        self._lift_pub.publish(msg)
        self._head_pub.publish(msg)
        #something got stuck
        self.loginfo("cozmo running recovery behavior")
        pass

    def _restore_post_shutdown(self):
        self._shutdown_flag = False
        #do recovery behavior 
        self.loginfo("reaction:shutdown ended")
        self._recovery_behavior()

    def _reaction_shutdown(self):

        #disable any possible actions and set shutdown flag to true
        self._run_flag = False 
        self._shutdown_flag = True        
        self._playing_anim_flag = True
        #send the sleeping animation to it
        goal = AnimGoal()
        goal.anim_name = "anim_gotosleep_off_01"
        self._anim_asc.send_goal(goal)
        #we need to wait for the animation to finish
        self._anim_asc.wait_for_result()
        result = self._anim_asc.get_result()

        #no matter what, reenable run flag and playing flag
        self._playing_anim_flag = False
        self._run_flag = True
        

        if(not result.success):
            #this means that the shutdown behavior has failed
            #we ignore it for now
            self._shutdown_flag = False
            return 
        
        self.loginfo("reaction:shutdown reaction")
        #stop all movements
        self._coznav.stop()

        #create a shutdown timer that we will run again when shutdown is complete
        self._timer = threading.Timer(self._config_file["shutdown_time"], self._restore_post_shutdown)
        self._timer.start() #start

    def _reaction_none(self):
        self.loginfo("reaction:No reaction")
        self._recovery_behavior()

    def _reaction_respond(self):
        #The robot will respond to the bully through behavior
        if self._level <= self._config_file["response_level"]:
            selected_anim = random.choice(self._config_file['responses']['sad_responses'])
            self._play_anim(selected_anim)
        elif self._level > self._config_file["response_level"]:
            selected_anim = random.choice(self._config_file['responses']['angry_responses'])
            self._play_anim(selected_anim) 
        self.loginfo("reaction:responsive reaction")        
        self._recovery_behavior()


    def loop(self):
        #self._target_pose = Pose2D()
        #self._target_pose.theta = 180

        r = rospy.Rate(1)
        self._recovery_behavior()
        while not rospy.is_shutdown():
            

            if self._kill_switch:
                pass

            #check if we are outside of area
            elif not self._coznav.lossly_position(self._target_pose) and self._start:
                self.loginfo("cozmo detected out of position")

                #we will only move if we haven't been bullied in the last 1.5 seconds
                if not ((rospy.Time.now() - self._last_bullied_time).to_sec() < self._config_file['last_bully_time'] * 1.5):
                    #move back to original position if not shutdown
                    if not self._shutdown_flag:
                        #pause everything
                        self._run_flag = False
                        self._coznav.move_to_pose(self._target_pose)
                        self._run_flag = True
                        #self._recovery_behavior()
                        self.loginfo("cozmo moved back to target pose")
            else:
                pass
                #print("pose okay")
            r.sleep()


if __name__ == '__main__':
    b = bullyObj()
    b.loop()