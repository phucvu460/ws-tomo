#!/usr/bin/env python3.7
from Mailbox import mail_box
import math
import time
import copy
import rospy
from both_arm_head.msg import Num
import moveit_msgs.msg 
import threading
import sensor_msgs.msg

rospy.init_node('main_control_both_arm',anonymous=True)
mail = mail_box.MAIL_BOX()

#define temps
temp5_R                =       0 
temp4_R                =       0
temp3_R                =       0 
temp2_R                =       0 
temp1_R                =       0
temp0_R                =       0 

temp5_L                =       0 
temp4_L                =       0
temp3_L                =       0 
temp2_L                =       0 
temp1_L                =       0
temp0_L                =       0 
status                 =       0

actual_position_L = []
actual_position_R = []

#Thread
class TomoThread(threading.Thread):

    def __init__(self):

        threading.Thread.__init__(self)

    def run(self):
        
        #Subscriber
        rospy.Subscriber('/tracking_trajectory',Num, self.callback) 
        rospy.Subscriber('/move_group/fake_controller_joint_states',sensor_msgs.msg.JointState, self.callback2)
        rospy.spin()

    # positions in ROS into goals for real actuators      
    def callback(self,data):

        global goal_joint0_R, goal_joint1_R, goal_joint2_R, goal_joint3_R, goal_joint4_R, goal_joint5_R
        global goal_joint0_L, goal_joint1_L, goal_joint2_L, goal_joint3_L, goal_joint4_L, goal_joint5_L
        global time
        global joint_names

        joint_names = data.joint_names
        
        #Change to control real robot
        #Right_arm
        if (joint_names[0] == 'joint_1_right'):

            goal_joint0_R     =       -data.positions[0]    #right_angle
            goal_joint1_R     =       -data.positions[1]
            goal_joint2_R     =       -data.positions[2]
            goal_joint3_R     =       -data.positions[3]
            goal_joint4_R     =       -data.positions[4]
            goal_joint5_R     =       data.positions[5]

            print('right')

        #Left_arm
        elif (joint_names[0] == 'joint_1_left'):

            goal_joint0_L     =       -data.positions[0]     #left_angle
            goal_joint1_L     =       -data.positions[1]
            goal_joint2_L     =       -data.positions[2]
            goal_joint3_L     =       data.positions[3]
            goal_joint4_L     =       -data.positions[4]
            goal_joint5_L     =       data.positions[5]

            print('left')
            
        time            =       float(data.time)/1000000000

        print(time)
    
        
    #When button execute in ROS is clicked, real system moves too
    def callback2(self,data):

        global status
        global temp5_R, temp4_R, temp3_R, temp2_R, temp1_R, temp0_R
        global temp5_L, temp4_L, temp3_L, temp2_L, temp1_L, temp0_L
        
        status = data.header.seq
        if status > 0:

            #When joints goal is changed, TOMO moves
            if (joint_names[0] == 'joint_1_right'):

                if (temp0_R != goal_joint0_R) or \
                   (temp1_R != goal_joint1_R) or \
                   (temp2_R != goal_joint2_R) or \
                   (temp3_R != goal_joint3_R) or \
                   (temp4_R != goal_joint4_R) or \
                   (temp5_R != goal_joint5_R):

                    mail.move_to_position(18,goal_joint0_R,time)    
                    mail.move_to_position(19,goal_joint1_R,time)  
                    mail.move_to_position(20,goal_joint2_R,time)  
                    mail.move_to_position(21,goal_joint3_R,time)  
                    mail.move_to_position(22,goal_joint4_R,time)  
                    mail.move_to_position(23,goal_joint5_R,time) 

                    temp0_R        =       goal_joint0_R
                    temp1_R        =       goal_joint1_R
                    temp2_R        =       goal_joint2_R
                    temp3_R        =       goal_joint3_R
                    temp4_R        =       goal_joint4_R
                    temp5_R        =       goal_joint5_R
                    
                     
            if (joint_names[0] == 'joint_1_left'):

                if (temp0_L != goal_joint0_L) or \
                   (temp1_L != goal_joint1_L) or \
                   (temp2_L != goal_joint2_L) or \
                   (temp3_L != goal_joint3_L) or \
                   (temp4_L != goal_joint4_L) or \
                   (temp5_L != goal_joint5_L):
                    
                    mail.move_to_position(0,goal_joint0_L,time)    
                    mail.move_to_position(1,goal_joint1_L,time)  
                    mail.move_to_position(2,goal_joint2_L,time)  
                    mail.move_to_position(3,goal_joint3_L,time)  
                    mail.move_to_position(4,goal_joint4_L,time)  
                    mail.move_to_position(5,goal_joint5_L,time)  

                    temp0_L        =       goal_joint0_L
                    temp1_L        =       goal_joint1_L
                    temp2_L        =       goal_joint2_L
                    temp3_L        =       goal_joint3_L
                    temp4_L        =       goal_joint4_L
                    temp5_L        =       goal_joint5_L 
            
            #Show the actual posions of both arm to compare with ROS state
            print(" ")

            actual_position_L = [mail.actual_position_in_rad(0),mail.actual_position_in_rad(1),
                                 mail.actual_position_in_rad(2),mail.actual_position_in_rad(3),
                                 mail.actual_position_in_rad(4),mail.actual_position_in_rad(5)]

            actual_position_R = [mail.actual_position_in_rad(18),mail.actual_position_in_rad(19),
                                 mail.actual_position_in_rad(20),mail.actual_position_in_rad(21),
                                 mail.actual_position_in_rad(22),mail.actual_position_in_rad(23)]

            print('left: ',actual_position_L)
            print('right: ',actual_position_R)

    def control(self,goal,id,time):
        
        mail.move_to_position(id,goal,time)

if __name__ == "__main__":

    mail.servo_on(0)
    mail.servo_on(1)
    mail.servo_on(2)
    mail.servo_on(3)
    mail.servo_on(4)
    mail.servo_on(5)
    mail.servo_on(18)
    mail.servo_on(19)
    mail.servo_on(20)
    mail.servo_on(21)
    mail.servo_on(22)
    mail.servo_on(23)

    try:
        control = TomoThread()
        control.start()

    except KeyboardInterrupt:
        
        control.__stop()
    
    




