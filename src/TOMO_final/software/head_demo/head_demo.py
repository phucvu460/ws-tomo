#!/usr/bin/env python3.7
from fin_tracking import fin_detect

# from Mailbox import mail_box
import math
import time
# import copy
# import rospy
# from both_arm_head.msg import Num
# import moveit_msgs.msg 
import threading
# import sensor_msgs.msg
# import matplotlib as plt
# import actionlib_msgs.msg
# import threading
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

# global mail 
# mail = mail_box.MAIL_BOX()
# time.sleep(3)
# mail.servo_on(0)
# mail.servo_on(1)
float_multi_msg = Float64MultiArray()
pub  = rospy.Publisher('center_point_position',Float64MultiArray, queue_size=1)

rospy.init_node('head_control',anonymous=True)
rate = rospy.Rate(10) # 10hz
time.sleep(6)

class Tomo_Head_Control(threading.Thread):

    def __init__(self,mail,head_1_id,head_2_id):

        threading.Thread.__init__(self)
        self._fin                       = fin_detect.tracking()
        self._move_angle_x              = 0
        self._move_angle_y              = 0
        self.head_1                     = head_1_id
        self.head_2                     = head_2_id
        self._mail                      = mail


    def run(self):
        try:
            while not rospy.is_shutdown():
                self._center_point                   = self._fin.get_center_point()
                # print(self._center_point)
                
                if( self._center_point != (0,0)):
                    dis_x                 =  300/2 - self._center_point[0]
                    dis_y                 =  self._center_point[1] - 150/2   
                    self._move_angle_x    = ( dis_x/30 * 5 ) * math.pi/180
                    self._move_angle_y    = ( dis_y/40 * 5 ) * math.pi/180
                else:
                    self._move_angle_x    = 0.0
                    self._move_angle_y    = 0.0
                
                # float_multi_msg.layout = 1


                float_multi_msg.data = [self._move_angle_x,self._move_angle_y]
                
                pub.publish(float_multi_msg)
                # pub_2.publish('abcjglaskd')
                # print(self._move_angle_y)
                # # cur_pos_x                 = self._mail.actual_position_in_rad(self.head_1)
                # # cur_pos_y                 = self._mail.actual_position_in_rad(self.head_2)
                # cur_pos_x                 = 0
                # cur_pos_y                 = 0

                # if( (cur_pos_x + self._move_angle_x < math.pi/3) and 
                #     (cur_pos_x + self._move_angle_x > -math.pi/3) and     
                #     (cur_pos_x + self._move_angle_x != cur_pos_x  )        ):
                #     # self._mail.move_to_position(self.head_1 , cur_pos_x + self._move_angle_x,0.5)
                #     pass
                #     # print('im here')
                # if( (cur_pos_y + self._move_angle_y < math.pi/3) and 
                #     (cur_pos_y + self._move_angle_y > -math.pi/3) and     
                #     (cur_pos_y + self._move_angle_y != cur_pos_y  )        ):
                #     # self._mail.move_to_position(self.head_2, cur_pos_y + self._move_angle_y,0.5)
                #     pass
                # time.sleep(0.5)
                # print('woking')
                rate.sleep()
            
            self.join()
        #Subscriber
        finally:
            
            # Stop thread           
            self.join()






if __name__ == '__main__':
    # pass
    # mail.servo_on(6)
    # mail.servo_on(0)
    # mail.servo_on(7)

    # mail.move_to_position(0, 0.3,1.5)
    # mail.move_to_position(1, 0.3,0.5)

    # finger_detection                    =  fin_detect.tracking()
    head                                = Tomo_Head_Control(1,6,7)
    head.start()