#!/usr/bin/env python3.7
from Mailbox import mail_box
import math
import time
import copy
import rospy
from TOMO_final.msg import Num
import moveit_msgs.msg 
import threading
import sensor_msgs.msg
from sensor_msgs.msg import JointState
import matplotlib as plt
import actionlib_msgs.msg
from Serial_Communication import Serial_controller
#Init_ROS_node
#Main_controller
rospy.init_node('main_control_trajec',anonymous=True) 

#Initial mailbox
mail = mail_box.MAIL_BOX()               
                          
#Initial Hand
ser_left = Serial_controller.SERIAL_MAIN_CONTROLLER('/dev/ttyUSB0') 

def call_back_init_angle(data):
    global angles
    angles = data.position
    print(angles)

rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, call_back_init_angle)

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
status_hand            =      -1
temp = 0 
actual_position_L = []
actual_position_R = []
#Thread
class Tomo():

    def __init__(self):
        pass

    def run(self):
        #Subscriber

        #Reach to object - open/close hand
        rospy.Subscriber('/status_hand', Num, self.callback_status_hand)               

        #Get trajectory from 'Display planned path'
        rospy.Subscriber('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, self.callback) 

        #Push Execute button
        rospy.Subscriber('/execute_trajectory/feedback',moveit_msgs.msg.ExecuteTrajectoryActionFeedback, self.callback2)          
        rospy.spin()

    # positions in ROS into goals for real actuators      
    def callback(self, data):
        #Init
        global joint_names
        global position, travel_time, velocity
        travel_time = []
        position = []
        velocity = []
        joint_names = []

        ########## Add trajectory to list  ###########

        #Get position 
        trajectory = data.trajectory[0].joint_trajectory.points[0].positions                
        trajecttory_list = list(trajectory)

        #Get time duration
        time = str(data.trajectory[0].joint_trajectory.points[0].time_from_start)           
        name = data.trajectory[0].joint_trajectory.joint_names

        #Get velocities
        trajec_velocity = list(data.trajectory[0].joint_trajectory.points[0].velocities)    
        
        position.append(trajecttory_list) 
        velocity.append(trajec_velocity)
        travel_time.append(float(time)/10**9)
        joint_names  = name

        for i in range(1,len(data.trajectory[0].joint_trajectory.points)):

            trajectory = data.trajectory[0].joint_trajectory.points[i].positions
            trajecttory_list = list(trajectory)
            time = str(data.trajectory[0].joint_trajectory.points[i].time_from_start)
            name = data.trajectory[0].joint_trajectory.joint_names
            trajec_velocity = list(data.trajectory[0].joint_trajectory.points[i].velocities)

            position.append(trajecttory_list) 
            travel_time.append(float(time)/10**9)
            velocity.append(trajec_velocity)
            joint_names  = name

        #############################################################################

    #status == 1 - close hand ; status == 0 - open hand     
    def callback_status_hand(self,data):
        global status_hand
        status_hand = data.status_hand
        print('status_hand: ', status_hand)

    #When button execute in ROS is clicked, real system moves too
    def callback2(self,data):

        global status, block, status_hand
        global temp 
        Trajec = Planning(position,velocity,travel_time)
        status = data.status.status
        #status = 1 : Running...
        #status = 3 : Done 
        if status == 3 :
            temp = 0

        if status != temp and status != 3:
            temp = status

            i0 = 0
            i1 = 0
            i2 = 0
            i3 = 0
            i4 = 0
            i5 = 0

            #Run to other waypoint
            while(status != 3):


                # ############################################
                # if((mail.get_status_word(0) & 0x1000) != 0x1000) and i0 == ( len(Trajec.joint_0) - 3):
                #     mail.move_to_position_blended_test_edge( id = 0,
                #                                             current_pos = -Trajec.joint_0[i0][0],
                #                                             target_position = -Trajec.joint_0[i0+2][0] ,
                #                                             speed_current = abs(Trajec.joint_0[i0][1]),
                #                                             speed_final = abs(Trajec.joint_0[i0+2][1]),
                #                                             time_to_reach = Trajec.joint_0[i0+2][2] - Trajec.joint_0[i0][2])
                #     i0 += 2
                #     sleep = 1

                # if((mail.get_status_word(1) & 0x1000) != 0x1000) and i1 == ( len(Trajec.joint_1) - 3):   
                #     mail.move_to_position_blended_test_edge( id = 1,
                #                                             current_pos = -Trajec.joint_1[i1][0],
                #                                             target_position = -Trajec.joint_1[i1+2][0] ,
                #                                             speed_current = abs(Trajec.joint_1[i1][1]),
                #                                             speed_final = abs(Trajec.joint_1[i1+2][1]),
                #                                             time_to_reach = Trajec.joint_1[i1+2][2] - Trajec.joint_1[i1][2])
                #     i1 += 2
                #     sleep = 1

                # if((mail.get_status_word(2) & 0x1000) != 0x1000) and i2 == ( len(Trajec.joint_2) - 3):
                #     mail.move_to_position_blended_test_edge( id = 2,
                #                                             current_pos = -Trajec.joint_2[i2][0],
                #                                             target_position = -Trajec.joint_2[i2+2][0] ,
                #                                             speed_current = abs(Trajec.joint_2[i2][1]),
                #                                             speed_final = abs(Trajec.joint_2[i2+2][1]),
                #                                             time_to_reach = Trajec.joint_2[i2+2][2] - Trajec.joint_2[i2][2])
                #     i2 += 2
                #     sleep = 1

                # if((mail.get_status_word(3) & 0x1000) != 0x1000) and i3 == ( len(Trajec.joint_3) - 3):                    
                #     mail.move_to_position_blended_test_edge( id = 3,
                #                                             current_pos = Trajec.joint_3[i3][0],
                #                                             target_position = Trajec.joint_3[i3+2][0] ,
                #                                             speed_current = abs(Trajec.joint_3[i3][1]),
                #                                             speed_final = abs(Trajec.joint_3[i3+2][1]),
                #                                             time_to_reach = Trajec.joint_3[i3+2][2] - Trajec.joint_3[i3][2])
                #     i3 += 2
                #     sleep = 1

                # if((mail.get_status_word(4) & 0x1000) != 0x1000) and i4 == ( len(Trajec.joint_4) - 3):                            
                #     mail.move_to_position_blended_test_edge( id = 4,
                #                                             current_pos = -Trajec.joint_4[i4][0],
                #                                             target_position = -Trajec.joint_4[i4+2][0] ,
                #                                             speed_current = abs(Trajec.joint_4[i4][1]),
                #                                             speed_final = abs(Trajec.joint_4[i4+2][1]),
                #                                             time_to_reach = Trajec.joint_4[i4+2][2] - Trajec.joint_4[i4][2])
                #     i4 += 2
                #     sleep = 1

                # if((mail.get_status_word(5) & 0x1000) != 0x1000) and i5 == (len(Trajec.joint_5) - 3):
                #     mail.move_to_position_blended_test_edge( id = 5,
                #                                     current_pos = -Trajec.joint_5[i5][0],
                #                                     target_position = -Trajec.joint_5[i5+2][0] ,
                #                                     speed_current = abs(Trajec.joint_5[i5][1]),
                #                                     speed_final = abs(Trajec.joint_5[i5+2][1]),
                #                                     time_to_reach = Trajec.joint_5[i5+2][2] - Trajec.joint_5[i5][2])
                #     i5 += 2
                #     sleep = 1
                # ############################################

                print('i0: ',i0, 'i1: ',i1,'i2: ',i2, 'i3: ',i3, 'i4: ',i4, 'i5: ',i5 )

                if((mail.get_status_word(0) & 0x1000) != 0x1000) and i0 < ( len(Trajec.joint_0) - 1):
                    mail.move_to_position_blended_test_edge( id = 0,
                                                            current_pos = -Trajec.joint_0[i0][0],
                                                            target_position = -Trajec.joint_0[i0+1][0] ,
                                                            speed_current = abs(Trajec.joint_0[i0][1]),
                                                            speed_final = abs(Trajec.joint_0[i0+1][1]),
                                                            time_to_reach = Trajec.joint_0[i0+1][2] - Trajec.joint_0[i0][2])
                    i0 += 1
                    sleep = 1

                if((mail.get_status_word(1) & 0x1000) != 0x1000) and i1 < ( len(Trajec.joint_1) - 1):   
                    mail.move_to_position_blended_test_edge( id = 1,
                                                            current_pos = -Trajec.joint_1[i1][0],
                                                            target_position = -Trajec.joint_1[i1+1][0] ,
                                                            speed_current = abs(Trajec.joint_1[i1][1]),
                                                            speed_final = abs(Trajec.joint_1[i1+1][1]),
                                                            time_to_reach = Trajec.joint_1[i1+1][2] - Trajec.joint_1[i1][2])
                    i1 += 1
                    sleep = 1

                if((mail.get_status_word(2) & 0x1000) != 0x1000) and i2 < ( len(Trajec.joint_2) - 1):
                    mail.move_to_position_blended_test_edge( id = 2,
                                                            current_pos = -Trajec.joint_2[i2][0],
                                                            target_position = -Trajec.joint_2[i2+1][0] ,
                                                            speed_current = abs(Trajec.joint_2[i2][1]),
                                                            speed_final = abs(Trajec.joint_2[i2+1][1]),
                                                            time_to_reach = Trajec.joint_2[i2+1][2] - Trajec.joint_2[i2][2])
                    i2 += 1
                    sleep = 1

                if((mail.get_status_word(3) & 0x1000) != 0x1000) and i3 < ( len(Trajec.joint_3) - 1):                    
                    mail.move_to_position_blended_test_edge( id = 3,
                                                            current_pos = Trajec.joint_3[i3][0],
                                                            target_position = Trajec.joint_3[i3+1][0] ,
                                                            speed_current = abs(Trajec.joint_3[i3][1]),
                                                            speed_final = abs(Trajec.joint_3[i3+1][1]),
                                                            time_to_reach = Trajec.joint_3[i3+1][2] - Trajec.joint_3[i3][2])
                    i3 += 1
                    sleep = 1

                if((mail.get_status_word(4) & 0x1000) != 0x1000) and i4 < ( len(Trajec.joint_4) - 1):                            
                    mail.move_to_position_blended_test_edge( id = 4,
                                                            current_pos = -Trajec.joint_4[i4][0],
                                                            target_position = -Trajec.joint_4[i4+1][0] ,
                                                            speed_current = abs(Trajec.joint_4[i4][1]),
                                                            speed_final = abs(Trajec.joint_4[i4+1][1]),
                                                            time_to_reach = Trajec.joint_4[i4+1][2] - Trajec.joint_4[i4][2])
                    i4 += 1
                    sleep = 1

                if((mail.get_status_word(5) & 0x1000) != 0x1000) and i5 < (len(Trajec.joint_5) - 1):
                    mail.move_to_position_blended_test_edge( id = 5,
                                                    current_pos = -Trajec.joint_5[i5][0],
                                                    target_position = -Trajec.joint_5[i5+1][0] ,
                                                    speed_current = abs(Trajec.joint_5[i5][1]),
                                                    speed_final = abs(Trajec.joint_5[i5+1][1]),
                                                    time_to_reach = Trajec.joint_5[i5+1][2] - Trajec.joint_5[i5][2])
                    i5 += 1
                    sleep = 1
                    

                #If any joint added 1 waypoint to buffer, sleep...
                if sleep == 1:
                    time.sleep(0.0008)
                    sleep = 0 

                #Function in picking up blister (TOMO's hand close when it's on blister)
                if i0 == (len(Trajec.joint_0)-1) and            \
                   i1 == (len(Trajec.joint_0)-1) and            \
                   i2 == (len(Trajec.joint_0)-1) and            \
                   i3 == (len(Trajec.joint_0)-1) and            \
                   i4 == (len(Trajec.joint_0)-1):

                    if status_hand == 2: 
                        #right_side
                        ser_left.set_motor_data(6,-1200,1000,900,True) #index finger up -> +
                        ser_left.set_motor_data(6,-1200,1000,900,True) #index finger up -> +
                        ser_left.set_motor_data(6,-1200,1000,900,True) #index finger up -> +
                        time.sleep(3)
                        ser_left.set_motor_data(6,-600,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-600,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-600,900,700,True) #index finger up -> +
                        time.sleep(1)
                        ser_left.set_motor_data(2,900,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,900,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,900,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1000,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1000,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1000,900,700,True) #index finger up -> +
                        time.sleep(3)
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                    elif status_hand == 3: 
                        #right_side
                        ser_left.set_motor_data(6,-600,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-600,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-600,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,900,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,900,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,900,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1000,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1000,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1000,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                    elif status_hand == 1: 
                        #left_side
                        ser_left.set_motor_data(6,-500,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-500,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-500,900,700,True) #index finger up -> +
                        time.sleep(2)
                        ser_left.set_motor_data(2,910,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,910,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,910,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1010,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1010,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1010,900,700,True) #index finger up -> +
                        time.sleep(3)
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,700,True) #index finger up -> +
                    elif status_hand == 4: 
                        #left_side
                        ser_left.set_motor_data(6,-500,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-500,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,-500,900,700,True) #index finger up -> +
                        
                        ser_left.set_motor_data(2,910,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,910,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,910,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1010,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1010,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,1010,900,700,True) #index finger up -> +
                        
                        ser_left.set_motor_data(1,50,700,200,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,200,True) #index finger up -> +
                        ser_left.set_motor_data(1,50,700,200,True) #index finger up -> +
                        
                    elif status_hand == 0: 
                        ser_left.set_motor_data(2,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(2,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(4,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(1,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,0,900,700,True) #index finger up -> +
                        ser_left.set_motor_data(6,0,900,700,True) #index finger up -> +

                #Done trajectory, Stop loop
                if i0 >= (len(Trajec.joint_0) - 1) and                       \
                i1 >= (len(Trajec.joint_1) - 1) and                          \
                i2 >= (len(Trajec.joint_2) - 1) and                          \
                i3 >= (len(Trajec.joint_3) - 1) and                          \
                i4 >= (len(Trajec.joint_4) - 1) and                          \
                i5 >= (len(Trajec.joint_5) - 1):
                    
                    break

            #Show the actual posions of both arm to compare with ROS state
            actual_position_L = [mail.actual_position_in_rad(0), mail.actual_position_in_rad(1),
                                 mail.actual_position_in_rad(2), mail.actual_position_in_rad(3),
                                 mail.actual_position_in_rad(4), mail.actual_position_in_rad(5)] 

            # actual_position_R = [mail.actual_position_in_rad(6), mail.actual_position_in_rad(7),
            #                      mail.actual_position_in_rad(8), mail.actual_position_in_rad(9),
            #                      mail.actual_position_in_rad(10), mail.actual_position_in_rad(11)]
    

#Module PLanning Trajectory
class Planning():

    def __init__(self, position, velocity, travel_time):

        self.position       =       position
        self.velocity       =       velocity
        self.travel_time    =       travel_time

        self.joint_0            =       []
        self.joint_1            =       []
        self.joint_2            =       []
        self.joint_3            =       []
        self.joint_4            =       []
        self.joint_5            =       []
        self.trajec_joint()

    #Change from po[joint1,joint2] velo[joint1,joint2] ... to joint1[po,velo,...] joint2[po,velo,...]
    def trajec_joint(self):

        for i in range (len(travel_time)):
            self.joint_0.append([round(position[i][0],6), round(velocity[i][0],6), travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_1.append([round(position[i][1],6), round(velocity[i][1],6), travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_2.append([round(position[i][2],6), round(velocity[i][2],6), travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_3.append([round(position[i][3],6), round(velocity[i][3],6), travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_4.append([round(position[i][4],6), round(velocity[i][4],6), travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_5.append([round(position[i][5],6), round(velocity[i][5],6), travel_time[i]])

        self.modify_velocity_reverse()

        # print("traject 5 >>>>>>>>>>>>>>> : \n",self.joint_5[-1])

    #Situation velocity is reverse with the next step, add one more step between that with velocity equals to zero
    def modify_velocity_reverse(self):

        k0      =       0
        k1      =       0
        k2      =       0   
        k3      =       0
        k4      =       0
        k5      =       0

        for i in range (1,len(travel_time)):

            for j in range (6):

                if ((self.velocity[i][j] > 0 and self.velocity[i-1][j] < 0) or (self.velocity[i][j] < 0 and self.velocity[i-1][j] > 0)):

                    if abs(self.velocity[i-1][j]) <= abs(self.velocity[i][j]):
                        self.velocity[i-1][j] = 0 
                    else:
                        self.velocity[i][j] = 0

if __name__ == "__main__":

    #Motors on
    mail.servo_on(0)
    mail.servo_on(1)
    mail.servo_on(2)
    mail.servo_on(3)
    mail.servo_on(4)
    mail.servo_on(5)

    #Init angles of arm (as TOMO in ROS)
    mail.move_to_position(5,round(-angles[5],6),5)
    mail.move_to_position(4,round(-angles[4],6),5)
    mail.move_to_position(3,round(angles[3],6),5)
    mail.move_to_position(2,round(-angles[2],6),5)
    mail.move_to_position(1,round(-angles[1],6),5)
    time.sleep(5)
    mail.move_to_position(0,round(-angles[0],6),5)

    try:
        control = Tomo()
        control.run()

    except KeyboardInterrupt:
        
        control.join()
    
