import rospy

from geometry_msgs.msg import(
    Twist,
    Point,
    Pose2D
)
from nav_msgs.msg import(
    Odometry
)

import numpy as np
import time
from rospkg import RosPack
from lab_ros_perception.ArucoTagModule import ArucoTagModule
import math
import copy
import roslaunch
import ros
import os

from kalman_filter import KLF

def Quaternion_toEulerianAngle(x, y, z, w):
    ysqr = y*y
    
    t0 = +2.0 * (w * x + y*z)
    t1 = +1.0 - 2.0 * (x*x + ysqr)
    X = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w*y - z*x)
    t2 =  1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    Z = math.degrees(math.atan2(t3, t4))
    
    
    return X, Y, Z 

class RobotAgent():


    def __init__(self):

        self._aruco_launch = None
        self.restart_aruco()

        #small fix for the threading issue
        self._last_pose_time = rospy.Time.now()        
        self._klf = KLF()
        self._cur_pose = Pose2D()    

        self._tag_module = ArucoTagModule()
        self._tag_module.register_callback(self._tag_callback)
        self._tag_module.waitForID(0)    
        
        #probably will use Kinect instead
        #self._cur_pose = self.get_pose()

        #find the roslaunch file that launches kinect2
        rp = RosPack()
        dirpath = rp.get_path("lab_ros_perception")
        self._aruco_kinect_launchfile_dir = os.path.join(dirpath, 'launch','aruco_kinect2.launch')


        self.odom_pose = copy.deepcopy(self._cur_pose)        

        rospy.Subscriber('cozmo/pose',Pose2D,self.pose_update)

        self._pub = rospy.Publisher('/cozmo/cmd_vel',Twist,queue_size=1)

    def restart_aruco(self):
     
        if self._aruco_launch is not None:
            self._aruco_launch.shutdown()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        roslaunch.configure_logging(uuid)

        self._aruco_launch = roslaunch.parent.ROSLaunchParent(uuid, [self._aruco_kinect_launchfile_dir])
        self._aruco_launch.start()


    def pose_update(self, msg):

        self.odom_pose.x = msg.x
        self.odom_pose.y = msg.y
        self.odom_pose.theta = msg.theta
        pass


    def _tag_callback(self, id, pose):
    
        if id == 0:



            # #HACK 1, we ignore any poses Z that are beyond a range
            p = pose.pose            
            if(p.position.z < 0 or p.position.z > 0.25):
                #rospy.loginfo("sys:restarting aruco")
                self.restart_aruco()
                return
            #     print("Incorrect TAG 0 found")
            #print(p)
            #     return 

             #get dt
            dt = (pose.header.stamp - self._last_pose_time).to_sec()
            self._last_pose_time = pose.header.stamp
            #get the current state
            yaw, pitch, roll = Quaternion_toEulerianAngle(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            state = np.array([p.position.x, p.position.y, np.deg2rad(roll)])
            #run kalman filter
            filtered_state = self._klf.filter(state, dt)

            self._cur_pose.x = p.position.x#filtered_state[0]
            self._cur_pose.y = p.position.y#filtered_state[1]
            self._cur_pose.theta = roll#np.rad2deg(filtered_state[2])      
            #print(self._cur_pose)
            #print(p.position.z)

    def get_pose(self):

        return self._cur_pose


    def apply_cmd(self, p_cmd):

        cmd = Twist()
        cmd.linear.x = p_cmd.x
        cmd.angular.z = p_cmd.theta

        self._pub.publish(cmd)


    def stop(self):
        r = rospy.Rate(2)
        cmd = Twist()
        self._pub.publish(cmd)
        r.sleep()


class CozmoNav(object):

    def __init__(self):
        self._agent = RobotAgent()
        self._rotation_threshold = 2
        self._position_threshold = 0.05
        self._rot_lossly_threshold = 5
        self._pos_lossly_threshold = 0.075

    def rotate_to_angle(self, target_rot):
        self._run_flag = True

        if not self._run_flag:
            return

        #Hz of command inputs
        r = rospy.Rate(10)
        #find the rotation difference
        cur_pose = self._agent.get_pose()
        diff = find_rotation_diff(cur_pose.theta, target_rot)

        #Do nothing if the difference is small
        if np.abs(diff) < self._rotation_threshold:
            return 
        self._agent.stop()
        #the rotation will depend on the odomotry change instead
        start_theta = self._agent.odom_pose.theta
        target_theta = start_theta + diff
        if target_theta > 180:
            target_theta = target_theta - 360
        if target_theta < -180:
            target_theta = target_theta + 360
        
        cur_pose = self._agent.odom_pose
        diff = find_rotation_diff(cur_pose.theta, target_theta)

        #rotate to that position until we are within the goal threshold
        while(np.abs(diff) > self._rotation_threshold) and self._run_flag:

            #calculate the control values
            power = np.min([np.abs(diff) * 0.07, 0.75,])
            power = np.max([power, 0.01])
            cmd = Pose2D()
            cmd.theta = power * np.sign(diff)
            #print("current theta:{}, target theta:{}, power:{}".format(cur_pose.theta, target_theta, cmd.theta))

            #apply the command and wait
            self._agent.apply_cmd(cmd)
            r.sleep()

            #check the new position and get difference
            cur_pose = self._agent.odom_pose
            diff = find_rotation_diff(cur_pose.theta, target_theta)

        if not self._run_flag:
            return 
        #stop
        self._agent.stop()

    def stop(self):
        self._run_flag = False
        self._agent.stop()

    def move_to_pose(self, target_pose):
        #outer_loop = rospy.Rate(0.5)
        motion_loop = rospy.Rate(5)
        self._run_flag = True
        
        #check if we are at destination
        while(find_position_diff(self._agent.get_pose(), target_pose) > self._position_threshold and self._run_flag):
            #loop
            #Step 1 - find direction & move to that direction
            target_rot = calculate_rotation(self._agent.get_pose(), target_pose)
            self.rotate_to_angle(target_rot)
            #Step 2 - move slightly in that direction for 1 second
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rospy.Duration(1.5) and self._run_flag:
                diff = find_position_diff(self._agent.get_pose(), target_pose)
                cmd = Pose2D()
                cmd.x = np.min([0.06, diff*0.6])
                self._agent.apply_cmd(cmd)
                #print("applied cmd.x:{}".format(cmd.x))
                motion_loop.sleep()
            #time.sleep(2)
            #outer_loop.sleep()
        if not self._run_flag:
            return
        #stop movement
        self._agent.stop()
        #rotate to correct angle
        self.rotate_to_angle(target_pose.theta)
    
    def in_position(self, target_pose):
        cur_pose = self._agent.get_pose()
        diff = find_position_diff(cur_pose, target_pose)
        rot_diff = find_rotation_diff(cur_pose.theta, target_pose.theta)

        return (np.abs(diff) < self._position_threshold) and (np.abs(rot_diff) < self._rotation_threshold), diff, rot_diff
    
    def lossly_position(self, target_pose):
        pos_flag, diff, rot_dff = self.in_position(target_pose)
        #print("diff:{}, rot_diff:{}".format(diff,rot_dff))
        if(not pos_flag):
            if np.abs(diff) < (self._pos_lossly_threshold) and np.abs(rot_dff) < (self._rot_lossly_threshold):
                return True
            else:
                #print("DIFF")
                #print(self._agent.get_pose())
                #print("DIFF END")
                return False 
            
        return pos_flag


def calculate_dist(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)


def calculate_rotation(cur_pose, end_pose):

    #calculate the rotation
    dx = end_pose.x - cur_pose.x
    dy = end_pose.y - cur_pose.y
    rot = np.arctan2(dx,dy)
    #convert to degrees
    rot = np.rad2deg(rot)
    return rot  * -1
    #return rot%360

def move_angle_to_pi(angle):
    tau = 6.28318530718
    if angle >= tau:
        angle = angle - np.floor(angle/tau) * tau
    elif angle <= -tau:
        angle = angel + np.floor(-angle/tau)*tau

    if angle < -np.pi:
        angle = 2*np.pi + angle
    elif angle > np.pi:
        angle = -2*np.pi + angle

    return angle

def find_angle(x, y):
    tau = 6.28318530718
    d = np.fmod(y-x, tau)
    if d < -np.pi:
        d = d + tau
    if d > np.pi:
        d = d - tau
    return -d

def find_rotation_diff(current_rot, target_rot):


    r1 = np.deg2rad(target_rot)
    r2 = np.deg2rad(current_rot)
    r1 = move_angle_to_pi(r1)
    r2 = move_angle_to_pi(r2)
    degree = np.rad2deg(find_angle(r1,r2))
    return degree


def rotate_to_angle_pure(agent, target_rot):
    #Hz of command inputs
    r = rospy.Rate(10)
    #find the rotation difference
    cur_pose = agent.get_pose()
    diff = find_rotation_diff(cur_pose.theta, target_rot)
    #rotate to that position until we are within the goal threshold
    while(np.abs(diff) > rotation_threshold):

        #calculate the control values
        power = np.min([np.abs(diff) * 0.02, 0.3])
        cmd = Pose2D()
        cmd.theta = power * np.sign(diff)
        #print("current theta:{}, target theta:{}, power:{}".format(cur_pose.theta, target_rot, cmd.theta))

        #apply the command and wait
        agent.apply_cmd(cmd)
        r.sleep()

        #check the new position and get difference
        cur_pose = agent.get_pose()
        diff = find_rotation_diff(cur_pose.theta, target_rot)
    #stop
    agent.stop()


def find_position_diff(cur_pose, target_pose):
    return np.sqrt((target_pose.x - cur_pose.x)**2 + (target_pose.y - cur_pose.y)**2)



def main():
    rospy.init_node('test_node')
    r = rospy.Rate(1)
    target_pose = Pose2D()
    target_pose.x = 0
    target_pose.y = 0.25
    target_pose.theta = 0
    rotation_threshold = 3
    cozNav = CozmoNav()
    time.sleep(1)
    cozNav._agent.stop()
    while True:

        pos_flag, diff, rot_diff = cozNav.in_position(target_pose)
        while pos_flag:
            r.sleep()
            pos_flag, diff, rot_diff = cozNav.in_position(target_pose)

        #move and rotate to the desire position        
        cozNav.move_to_pose(target_pose)
        




if __name__ == '__main__':
    main()
