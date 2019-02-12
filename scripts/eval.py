#!/usr/bin/env python

import rospy
from std_msgs.msg import (
    String,
    Float64,
    ColorRGBA,
    Int8
)
from sensor_msgs.msg import (
    Image,
    CameraInfo,
    BatteryState,
    Imu,
    JointState,
)

from geometry_msgs.msg import (
    Pose2D
)


from cozmo_driver.msg import(
    AnimAction,
    AnimGoal
)
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
import yaml
import random
from CozmoNav import CozmoNav
from bystander_intervention_study.msg import LearnStatus

class EvalObj(object):

    def __init__(self):


        #read the configuration file
        self._config_file = None
        with open('../res/eval_list.yaml','r') as f:
            self._config_file = yaml.load(f)

        self._anim_list = self._config_file["list"]
        self._index = 0

        rospy.init_node('eval_list')

        self._anim_asc = actionlib.SimpleActionClient('cozmo/exec_anim', AnimAction)
        self._anim_asc.wait_for_server()

        self._lift_pub = rospy.Publisher('cozmo/lift_height',Float64, queue_size=1)
        self._head_pub = rospy.Publisher('cozmo/head_angle', Float64, queue_size=1)
        self._say_pub = rospy.Publisher('cozmo/say',String, queue_size=1)

        rospy.loginfo('system ready')

    def done(self):
        return self._index >= len(self._anim_list)

    def repeat(self):
        self._index -= 1
        self.next()

    def next(self):
        name = self._anim_list[self._index]
        print("playing:{}".format(name))
        start_time = rospy.Time.now()
        self._play_anim(name)
        duration = rospy.Time.now() - start_time

        print("duration:{}".format(duration.to_sec()))
        self._index += 1
        self._play_anim("anim_neutral_eyes_01")

        msg = Float64()
        msg.data = 0
        self._lift_pub.publish(msg)
        self._head_pub.publish(msg)

        print("done")

    def _play_anim(self, name):

        goal = AnimGoal()
        goal.anim_name = name

        self._anim_asc.send_goal(goal)
        #we need to wait for the animation to finish
        self._anim_asc.wait_for_result()
        result = self._anim_asc.get_result()


if __name__ == '__main__':
    eval = EvalObj()
    while not eval.done():
        eval.next()
        x = raw_input()
        while x.startswith('r'):
            eval.repeat()
            x = raw_input()
        