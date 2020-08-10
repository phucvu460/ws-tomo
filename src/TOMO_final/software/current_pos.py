#!/usr/bin/env python3.7
from Mailbox import mail_box
import math
import time
import copy
import rospy
import math  
from TOMO_final.msg import Num
import moveit_msgs.msg 
import sensor_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading

rospy.init_node('current_pos')
mail = mail_box.MAIL_BOX()
pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)

#Thread
class myThread(threading.Thread):

    def __init__(self,pub):
        threading.Thread.__init__(self)
        self.pub = pub

    def run(self):

        rate = rospy.Rate(1) # 10hz
        hello_str = JointState() 
        hello_str.header = Header()

        #Joint names
        hello_str.name              = ['joint_1_right', 'joint_2_right', 'joint_3_right', 'joint_4_right','joint_5_right','hand_joint',
                                       'joint_1_left', 'joint_2_left', 'joint_3_left', 'joint_4_left','joint_5_left','joint_6_left']

        #Joint positions
        hello_str.position          = [0,0,
                                       0,0,
                                       0,0,
                                       -mail.actual_position_in_rad(0),-mail.actual_position_in_rad(1),
                                       -mail.actual_position_in_rad(2),mail.actual_position_in_rad(3),
                                       -mail.actual_position_in_rad(4),-mail.actual_position_in_rad(5) ]

        hello_str.velocity = []
        hello_str.effort = []
        print(hello_str)

        while not rospy.is_shutdown():

            hello_str.header.stamp = rospy.Time.now()
            self.pub.publish(hello_str)
            rate.sleep()

        print("Bao")
        

if __name__ == '__main__':

    try:

        talker = myThread(pub)
        talker.start()
        
    except rospy.ROSInterruptException:
        pass