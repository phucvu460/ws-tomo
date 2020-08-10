#!/usr/bin/env python
import pysoem
import sys
import copy
import rospy
import math  
import time 
from mailbox import Mailbox
import moveit_msgs.msg 
import trajectory_msgs.msg 
from both_arm_head.msg import Num

rospy.init_node('display_planned_path',anonymous=True)

# print(display_trajectory_publisher)


class trajectory_tracking (object):

    def __init__(self,pub):
        self._pub = pub

    def callback(self, data):
        global goal_joint0_R, goal_joint1_R, goal_joint2_R, goal_joint3_R, goal_joint4_R, goal_joint5_R
        global goal_joint0_L, goal_joint1_L, goal_joint2_L, goal_joint3_L, goal_joint4_L, goal_joint5_L

        
        trajectory = data.trajectory[0].joint_trajectory.points[-1].positions
        trajecttory_list = list(trajectory)
        time = str(data.trajectory[0].joint_trajectory.points[-1].time_from_start)
        name = data.trajectory[0].joint_trajectory.joint_names

        print(name)
        print(type(time))

        trajec = Num()
        trajec.positions    = trajecttory_list
        trajec.time         = int(time)
        trajec.joint_names  = name

        self._pub.publish(trajec)
        
        #Right_arm
        if (name == ['joint_1_right', 'joint_2_right', 'joint_3_right', 'joint_4_right','joint_5_right','hand_joint']):

            goal_joint0_R     =       -trajec.positions[0]    #right_angle
            goal_joint1_R     =       -trajec.positions[1]
            goal_joint2_R     =       -trajec.positions[2]
            goal_joint3_R     =       -trajec.positions[3]
            goal_joint4_R     =       -trajec.positions[4]
            goal_joint5_R     =       trajec.positions[5]

            print('right')
            print(trajecttory_list)
            print(goal_joint0_R)
            print(goal_joint1_R)
            print(goal_joint2_R)
            print(goal_joint3_R)
            print(goal_joint4_R) 
            print(goal_joint5_R)
        
        #Left_arm
        elif (name == ['joint_1_left', 'joint_2_left', 'joint_3_left', 'joint_4_left','joint_5_left','joint_6_left']):

            
            goal_joint0_L     =       -trajec.positions[0]     #left_angle
            goal_joint1_L     =       -trajec.positions[1]
            goal_joint2_L     =       -trajec.positions[2]
            goal_joint3_L     =       trajec.positions[3]
            goal_joint4_L     =       -trajec.positions[4]
            goal_joint5_L     =       trajec.positions[5]
        
            print('left')
            print(goal_joint0_L)
            print(goal_joint1_L)
            print(goal_joint2_L)
            print(goal_joint3_L)
            print(goal_joint4_L) 
            print(goal_joint5_L)

        print(type(trajec.time))
        print(float(trajec.time)/10**9)
    
def listener():

    print('Hello')
    print('My name Hong gia Bao')

    pub = rospy.Publisher('/tracking_trajectory',Num,queue_size=100000)
    monitor = trajectory_tracking(pub)
    rospy.Subscriber('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, monitor.callback)
    rospy.spin()




if __name__ == '__main__':
    
    listener()
    
    
