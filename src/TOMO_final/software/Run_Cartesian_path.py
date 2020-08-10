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
import matplotlib as plt
import actionlib_msgs.msg

rospy.init_node('main_control_trajec',anonymous=True)
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
temp = 0 
block = 0 
actual_position_L = []
actual_position_R = []

#Thread
class TomoThread(threading.Thread):

    def __init__(self):

        threading.Thread.__init__(self)

    def run(self):
        #Subscriber
        if block == 0:
            rospy.Subscriber('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, self.callback)
        rospy.Subscriber('/execute_trajectory/status',actionlib_msgs.msg.GoalStatusArray, self.callback2)
        rospy.spin()

    #Function finds acce, deacce when position, velo are gotten 
    #Note: Velocity reverse case is not prepare in this function (Show in Class Planning )
    def find_acce_deacce(self, position, velocity, travel_time):

        acceleration.append([2058.87416,2058.87416,2573.592701,2573.592701, 4117.748321, 4117.748321])
        deacceleration.append([2058.87416,2058.87416,2573.592701,2573.592701, 4117.748321, 4117.748321])

        for i in range (1,len(velocity)-1):

            trajec_accelerate = []
            trajec_deaccelerate = []

            for j in range (6):

                if ((velocity[i][j] >= 0 and velocity[i+1][j] >= 0) or (velocity[i][j] <= 0 and velocity[i+1][j] <= 0)):
                    
                    if (abs(velocity[i][j])-abs(velocity[i+1][j])) <= 0:
                        
                        acce = (abs(velocity[i+1][j])-abs(velocity[i][j]))/(float(travel_time[i]))

                        if j == 0 or j == 1:
                            deacce = 2058.87416
                        elif j == 2 or j == 3:
                            deacce = 2573.592701
                        else:
                            deacce = 4117.748321

                        if acce <= 0.000001 :
                            if j == 0 or j == 1:
                                acce = 2058.87416
                            elif j == 2 or j == 3:
                                acce = 2573.592701
                            else:
                                acce = 4117.748321
                        # print('acce:', acce)
                            

                    elif (abs(velocity[i][j])-abs(velocity[i+1][j])) > 0:

                        print('acce:',acce)

                        if j == 0 or j == 1:
                            acce = 2058.87416
                        elif j == 2 or j == 3:
                            acce = 2573.592701
                        else:
                            acce = 4117.748321

                        deacce = (abs(velocity[i][j])-abs(velocity[i+1][j]))/(float(travel_time[i]))

                        if deacce <= 0.000001 :
                            if j == 0 or j == 1:
                                deacce = 2058.87416
                            elif j == 2 or j == 3:
                                deacce = 2573.592701
                            else:
                                deacce = 4117.748321

                    trajec_accelerate.append(acce)
                    trajec_deaccelerate.append(deacce)


                elif ((velocity[i][j] > 0 and velocity[i+1][j] < 0) or (velocity[i][j] < 0 and velocity[i+1][j] > 0)):

                    if j == 0 or j == 1:
                        acce = 2058.87416
                        deacce = 2573.592701
                    elif j == 2 or j == 3:
                        acce = 2573.592701
                        deacce = 2573.592701
                    else:
                        acce =4117.748321
                        deacce = 4117.748321
                    trajec_accelerate.append(acce)
                    trajec_deaccelerate.append(deacce)

                    
            acceleration.append(trajec_accelerate)
            deacceleration.append(trajec_deaccelerate)

        return acceleration, deacceleration

    # positions in ROS into goals for real actuators      
    def callback(self, data):
        global joint_names
        global position, travel_time, velocity, acceleration,deacceleration
        travel_time = []
        position = []
        velocity = []
        acceleration = []
        deacceleration = []
        # print(display_trajectory_publisher)
        joint_names = []


########## Add trajectory to list  #########################################

        trajectory = data.trajectory[0].joint_trajectory.points[0].positions
        trajecttory_list = list(trajectory)
        time = str(data.trajectory[0].joint_trajectory.points[0].time_from_start)
        name = data.trajectory[0].joint_trajectory.joint_names
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

        
        acceleration, deacceleration  = self.find_acce_deacce(position,velocity, travel_time)

    #When button execute in ROS is clicked, real system moves too
    def callback2(self,data):

        global status, block
        global temp 

        Trajec = Planning(position,velocity,acceleration,deacceleration,travel_time)
        status = data.status_list[-1].status

        if status == 3 :
            temp = 0

        if status != temp and status != 3:
            # print('temp: ', temp)
            temp = status
            # print('status: ', status)

            # mail.move_to_position_ROS(0, -Trajec.joint_0[1][0], abs(Trajec.joint_0[1][1]), abs(Trajec.joint_0[1][2]), abs(Trajec.joint_0[1][3]))
            # mail.move_to_position_ROS(1, -Trajec.joint_1[1][0], abs(Trajec.joint_1[1][1]), abs(Trajec.joint_1[1][2]), abs(Trajec.joint_1[1][3]))
            # mail.move_to_position_ROS(2, -Trajec.joint_2[1][0], abs(Trajec.joint_2[1][1]), abs(Trajec.joint_2[1][2]), abs(Trajec.joint_2[1][3]))
            # mail.move_to_position_ROS(3, Trajec.joint_3[1][0], abs(Trajec.joint_3[1][1]), abs(Trajec.joint_3[1][2]), abs(Trajec.joint_3[1][3]))
            # mail.move_to_position_ROS(4, -Trajec.joint_4[1][0], abs(Trajec.joint_4[1][1]), abs(Trajec.joint_4[1][2]), abs(Trajec.joint_4[1][3]))
            # mail.move_to_position_ROS(5, Trajec.joint_5[1][0], abs(Trajec.joint_5[1][1]), abs(Trajec.joint_5[1][2]), abs(Trajec.joint_5[1][3]))

            mail.move_to_position_blended_test_edge( id = 0,
                                                     current_pos = -Trajec.joint_0[0][0],
                                                     target_position = -Trajec.joint_0[1][0] ,
                                                     speed_current = abs(Trajec.joint_0[0][1]),
                                                     speed_final = abs(Trajec.joint_0[1][1]),
                                                     time_to_reach = Trajec.joint_0[1][4] - Trajec.joint_0[0][4])
            
            mail.move_to_position_blended_test_edge( id = 1,
                                                     current_pos = -Trajec.joint_1[0][0],
                                                     target_position = -Trajec.joint_1[1][0] ,
                                                     speed_current = abs(Trajec.joint_1[0][1]),
                                                     speed_final = abs(Trajec.joint_1[1][1]),
                                                     time_to_reach = Trajec.joint_1[1][4] - Trajec.joint_1[0][4])

            mail.move_to_position_blended_test_edge( id = 2,
                                                     current_pos = -Trajec.joint_2[0][0],
                                                     target_position = -Trajec.joint_2[1][0] ,
                                                     speed_current = abs(Trajec.joint_2[0][1]),
                                                     speed_final = abs(Trajec.joint_2[1][1]),
                                                     time_to_reach = Trajec.joint_2[1][4] - Trajec.joint_2[0][4])

            mail.move_to_position_blended_test_edge( id = 3,
                                                     current_pos = Trajec.joint_3[0][0],
                                                     target_position = Trajec.joint_3[1][0] ,
                                                     speed_current = abs(Trajec.joint_3[0][1]),
                                                     speed_final = abs(Trajec.joint_3[1][1]),
                                                     time_to_reach = Trajec.joint_3[1][4] - Trajec.joint_3[0][4])
            

            mail.move_to_position_blended_test_edge( id = 4,
                                                     current_pos = -Trajec.joint_4[0][0],
                                                     target_position = -Trajec.joint_4[1][0] ,
                                                     speed_current = abs(Trajec.joint_4[0][1]),
                                                     speed_final = abs(Trajec.joint_4[1][1]),
                                                     time_to_reach = Trajec.joint_4[1][4] - Trajec.joint_4[0][4])
                                                     
            mail.move_to_position_blended_test_edge( id = 5,
                                                     current_pos = Trajec.joint_5[0][0],
                                                     target_position = Trajec.joint_5[1][0] ,
                                                     speed_current = abs(Trajec.joint_5[0][1]),
                                                     speed_final = abs(Trajec.joint_5[1][1]),
                                                     time_to_reach = Trajec.joint_5[1][4] - Trajec.joint_5[0][4])
            # time.sleep(0.00001)

            i0 = 1
            i1 = 1
            i2 = 1
            i3 = 1
            i4 = 1
            i5 = 1
            # print(Trajec.joint_5)
            
            # while(status != 3):
                
            #     # print('mail._slave_set[0].STATUS_WORD: ', mail._slave_set[0].STATUS_WORD)
            #     # print('mail._slave_set[1].STATUS_WORD: ', mail._slave_set[1].STATUS_WORD)
            #     # print('mail._slave_set[2].STATUS_WORD: ', mail._slave_set[2].STATUS_WORD)
            #     # print('mail._slave_set[3].STATUS_WORD: ', mail._slave_set[3].STATUS_WORD)
            #     # print('mail._slave_set[4].STATUS_WORD: ', mail._slave_set[4].STATUS_WORD)
            #     # print('mail._slave_set[5].STATUS_WORD: ', mail._slave_set[5].STATUS_WORD)

            #     if((mail._slave_set[0].STATUS_WORD & 0x1000) != 0x1000) and i0 < len(Trajec.joint_0):
            #         # print('joint 0')
            #         if abs(Trajec.joint_0[i0][1]) <= 0.0001:
            #             Trajec.joint_0[i0][1] = abs(Trajec.joint_0[i0-1][1])
            #             # print('velocity',abs(Trajec.joint_0[i0-1][1]))
            #         mail.move_to_position_ROS(0, -Trajec.joint_0[i0][0], abs(Trajec.joint_0[i0][1]), abs(Trajec.joint_0[i0][2]), abs(Trajec.joint_0[i0][3]))
            #         i0 += 1
            #     if((mail._slave_set[1].STATUS_WORD & 0x1000) != 0x1000) and i1 < len(Trajec.joint_1):   
            #         # print('joint 1')
            #         if abs(Trajec.joint_1[i1][1]) <= 0.0001:
            #             Trajec.joint_1[i1][1] = abs(Trajec.joint_1[i1-1][1])
            #         mail.move_to_position_ROS(1, -Trajec.joint_1[i1][0], abs(Trajec.joint_1[i1][1]), abs(Trajec.joint_1[i1][2]), abs(Trajec.joint_1[i1][3]))
            #         i1 += 1
            #     if((mail._slave_set[2].STATUS_WORD & 0x1000) != 0x1000) and i2 < len(Trajec.joint_2):
            #         # print('joint 2')
            #         if abs(Trajec.joint_2[i2][1]) <= 0.0001:
            #             Trajec.joint_2[i2][1] = abs(Trajec.joint_2[i2-1][1])
            #         mail.move_to_position_ROS(2, -Trajec.joint_2[i2][0], abs(Trajec.joint_2[i2][1]), abs(Trajec.joint_2[i2][2]), abs(Trajec.joint_2[i2][3]))
            #         i2 += 1
            #     if((mail._slave_set[3].STATUS_WORD & 0x1000) != 0x1000) and i3 < len(Trajec.joint_3):
            #         # print('joint 3')
            #         if abs(Trajec.joint_3[i3][1]) <= 0.0001:
            #             Trajec.joint_3[i3][1] = abs(Trajec.joint_3[i3-1][1])
            #         mail.move_to_position_ROS(3, Trajec.joint_3[i3][0], abs(Trajec.joint_3[i3][1]), abs(Trajec.joint_3[i3][2]), abs(Trajec.joint_3[i3][3]))
            #         i3 += 1
            #     if((mail._slave_set[4].STATUS_WORD & 0x1000) != 0x1000) and i4 < len(Trajec.joint_4):
            #         # print('joint 4')
            #         if abs(Trajec.joint_4[i4][1]) <= 0.0001:
            #             Trajec.joint_4[i4][1] = abs(Trajec.joint_4[i4-1][1])                
            #         mail.move_to_position_ROS(4, -Trajec.joint_4[i4][0], abs(Trajec.joint_4[i4][1]), abs(Trajec.joint_4[i4][2]), abs(Trajec.joint_4[i4][3]))
            #         i4 += 1
            #     if((mail._slave_set[5].STATUS_WORD & 0x1000) != 0x1000) and i5 < len(Trajec.joint_5):
            #         # print('joint 5')
            #         if abs(Trajec.joint_5[i5][1]) <= 0.0001:
            #             Trajec.joint_5[i5][1] = abs(Trajec.joint_5[i5-1][1])
            #         mail.move_to_position_ROS(5, Trajec.joint_5[i5][0], abs(Trajec.joint_5[i5][1]), abs(Trajec.joint_5[i5][2]), abs(Trajec.joint_5[i5][3]))
            #         i5 += 1
                
            #     #Done trajectory, Stop loop
            #     if i0 >= len(Trajec.joint_0) and i1 >= len(Trajec.joint_1) and i2 >= len(Trajec.joint_2) and i3 >= len(Trajec.joint_3) and i4 >= len(Trajec.joint_4) and i5 >= len(Trajec.joint_5):
            #         break


            while(status != 3):
                # if ((mail._slave_set[0].STATUS_WORD & 0x1000) != 0x1000) or          \
                #    ((mail._slave_set[1].STATUS_WORD & 0x1000) != 0x1000) or          \
                #    ((mail._slave_set[2].STATUS_WORD & 0x1000) != 0x1000) or          \
                #    ((mail._slave_set[3].STATUS_WORD & 0x1000) != 0x1000) or          \
                #    ((mail._slave_set[4].STATUS_WORD & 0x1000) != 0x1000) or          \
                #    ((mail._slave_set[5].STATUS_WORD & 0x1000) != 0x1000):
                    # print('mail._slave_set[0].STATUS_WORD: ', mail._slave_set[0].STATUS_WORD)
                    # print('mail._slave_set[1].STATUS_WORD: ', mail._slave_set[1].STATUS_WORD)
                    # print('mail._slave_set[2].STATUS_WORD: ', mail._slave_set[2].STATUS_WORD)
                    # print('mail._slave_set[3].STATUS_WORD: ', mail._slave_set[3].STATUS_WORD)
                    # print('mail._slave_set[4].STATUS_WORD: ', mail._slave_set[4].STATUS_WORD)
                    # print('mail._slave_set[5].STATUS_WORD: ', mail._slave_set[5].STATUS_WORD)
                print('i0: ',i0, 'i1: ',i1,'i2: ',i2, 'i3: ',i3, 'i4: ',i4, 'i5: ',i5 )
                # print('i2:',i2)
                # print('len_joint0: ', len(Trajec.joint_0), 
                #     'len_joint1: ', len(Trajec.joint_1),
                #     'len_joint2: ', len(Trajec.joint_2),
                #     'len_joint3: ', len(Trajec.joint_3),
                #     'len_joint4: ', len(Trajec.joint_4),
                #     'len_joint5: ', len(Trajec.joint_5))
                if((mail._slave_set[0].STATUS_WORD & 0x1000) != 0x1000) and i0 < ( len(Trajec.joint_0) - 1):
                    # print('joint 0')

                    # mail.move_to_position_ROS(0, -Trajec.joint_0[i0][0], abs(Trajec.joint_0[i0][1]), abs(Trajec.joint_0[i0][2]), abs(Trajec.joint_0[i0][3]))
                    mail.move_to_position_blended_test_edge( id = 0,
                                                            current_pos = -Trajec.joint_0[i0][0],
                                                            target_position = -Trajec.joint_0[i0+1][0] ,
                                                            speed_current = abs(Trajec.joint_0[i0][1]),
                                                            speed_final = abs(Trajec.joint_0[i0+1][1]),
                                                            time_to_reach = Trajec.joint_0[i0+1][4] - Trajec.joint_0[i0][4])
                    i0 += 1
                    sleep = 1
                if((mail._slave_set[1].STATUS_WORD & 0x1000) != 0x1000) and i1 < ( len(Trajec.joint_1) - 1):   
                    # print('joint 1')
                    # mail.move_to_position_ROS(1, -Trajec.joint_1[i1][0], abs(Trajec.joint_1[i1][1]), abs(Trajec.joint_1[i1][2]), abs(Trajec.joint_1[i1][3]))
                    mail.move_to_position_blended_test_edge( id = 1,
                                                            current_pos = -Trajec.joint_1[i1][0],
                                                            target_position = -Trajec.joint_1[i1+1][0] ,
                                                            speed_current = abs(Trajec.joint_1[i1][1]),
                                                            speed_final = abs(Trajec.joint_1[i1+1][1]),
                                                            time_to_reach = Trajec.joint_1[i1+1][4] - Trajec.joint_1[i1][4])
                    
                    i1 += 1
                    sleep = 1
                if((mail._slave_set[2].STATUS_WORD & 0x1000) != 0x1000) and i2 < ( len(Trajec.joint_2) - 1):
                    # print('joint 2')
                    
                    # mail.move_to_position_ROS(2, -Trajec.joint_2[i2][0], abs(Trajec.joint_2[i2][1]), abs(Trajec.joint_2[i2][2]), abs(Trajec.joint_2[i2][3]))
                    mail.move_to_position_blended_test_edge( id = 2,
                                                            current_pos = -Trajec.joint_2[i2][0],
                                                            target_position = -Trajec.joint_2[i2+1][0] ,
                                                            speed_current = abs(Trajec.joint_2[i2][1]),
                                                            speed_final = abs(Trajec.joint_2[i2+1][1]),
                                                            time_to_reach = Trajec.joint_2[i2+1][4] - Trajec.joint_2[i2][4])

                    i2 += 1
                    sleep = 1
                if((mail._slave_set[3].STATUS_WORD & 0x1000) != 0x1000) and i3 < ( len(Trajec.joint_3) -1 ):
                    # print('joint 3')
                    
                    # mail.move_to_position_ROS(3, Trajec.joint_3[i3][0], abs(Trajec.joint_3[i3][1]), abs(Trajec.joint_3[i3][2]), abs(Trajec.joint_3[i3][3]))
                    mail.move_to_position_blended_test_edge( id = 3,
                                                            current_pos = Trajec.joint_3[i3][0],
                                                            target_position = Trajec.joint_3[i3+1][0] ,
                                                            speed_current = abs(Trajec.joint_3[i3][1]),
                                                            speed_final = abs(Trajec.joint_3[i3+1][1]),
                                                            time_to_reach = Trajec.joint_3[i3+1][4] - Trajec.joint_3[i3][4])
                    i3 += 1
                    sleep = 1
                if((mail._slave_set[4].STATUS_WORD & 0x1000) != 0x1000) and i4 < ( len(Trajec.joint_4) - 1):
                    # print('joint 4')
                            
                    # mail.move_to_position_ROS(4, -Trajec.joint_4[i4][0], abs(Trajec.joint_4[i4][1]), abs(Trajec.joint_4[i4][2]), abs(Trajec.joint_4[i4][3]))
                    mail.move_to_position_blended_test_edge( id = 4,
                                                            current_pos = -Trajec.joint_4[i4][0],
                                                            target_position = -Trajec.joint_4[i4+1][0] ,
                                                            speed_current = abs(Trajec.joint_4[i4][1]),
                                                            speed_final = abs(Trajec.joint_4[i4+1][1]),
                                                            time_to_reach = Trajec.joint_4[i4+1][4] - Trajec.joint_4[i4][4])
                    i4 += 1
                    sleep = 1
                if((mail._slave_set[5].STATUS_WORD & 0x1000) != 0x1000) and i5 < (len(Trajec.joint_5)-1):
                    # print('joint 5')
                    
                    # mail.move_to_position_ROS(5, Trajec.joint_5[i5][0], abs(Trajec.joint_5[i5][1]), abs(Trajec.joint_5[i5][2]), abs(Trajec.joint_5[i5][3]))
                    mail.move_to_position_blended_test_edge( id = 5,
                                                    current_pos = Trajec.joint_5[i5][0],
                                                    target_position = Trajec.joint_5[i5+1][0] ,
                                                    speed_current = abs(Trajec.joint_5[i5][1]),
                                                    speed_final = abs(Trajec.joint_5[i5+1][1]),
                                                    time_to_reach = Trajec.joint_5[i5+1][4] - Trajec.joint_5[i5][4])
                    # print("commonnnnnnnnmmmmmmmmmmmmmmmmmm: ",Trajec.joint_5[i5+1][0])
                    i5 += 1
                #     sleep = 1
                # if sleep == 1:
                #     time.sleep(0.00001)
                #     sleep = 0 
                # else:
                #     pass
                #Done trajectory, Stop loop
                if i0 >= (len(Trajec.joint_0) - 1) and                          \
                i1 >= (len(Trajec.joint_1) - 1) and                           \
                i2 >= (len(Trajec.joint_2) - 1) and                          \
                i3 >= (len(Trajec.joint_3) - 1) and                          \
                i4 >= (len(Trajec.joint_4) - 1) and                          \
                i5 >= (len(Trajec.joint_5) - 1):
                    break
                # if i2 >= (len(Trajec.joint_2) - 1):
                #     break



            #Show the actual posions of both arm to compare with ROS state
            # print(" ")

            actual_position_L = [mail.actual_position_in_rad(0), mail.actual_position_in_rad(1),
                                 mail.actual_position_in_rad(2), mail.actual_position_in_rad(3),
                                 mail.actual_position_in_rad(4), mail.actual_position_in_rad(5)] 

            # actual_position_R = [mail.actual_position_in_rad(6), mail.actual_position_in_rad(7),
            #                      mail.actual_position_in_rad(8), mail.actual_position_in_rad(9),
            #                      mail.actual_position_in_rad(10), mail.actual_position_in_rad(11)]
            
            # print('left: ',actual_position_L)
            # print('enddddddddddddddddddddddddddddddddddddddddddd')
            block = 0

#Module PLanning Trajectory
class Planning():

    def __init__(self, position, velocity, acceleration, deacceleration, travel_time):

        self.position       =       position
        self.velocity       =       velocity
        self.acceleration   =       acceleration
        self.deacceleration =       deacceleration
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
            self.joint_0.append([position[i][0], velocity[i][0], acceleration[i][0], deacceleration[i][0], travel_time[i] ])

        for i in range (len(travel_time)):
            self.joint_1.append([position[i][1], velocity[i][1], acceleration[i][1], deacceleration[i][1],travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_2.append([position[i][2], velocity[i][2], acceleration[i][2], deacceleration[i][2],travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_3.append([position[i][3], velocity[i][3], acceleration[i][3], deacceleration[i][3],travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_4.append([position[i][4], velocity[i][4], acceleration[i][4], deacceleration[i][4],travel_time[i]])

        for i in range (len(travel_time)):
            self.joint_5.append([position[i][5], velocity[i][5], acceleration[i][5], deacceleration[i][5],travel_time[i]])

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
                    #Caculate time, position,deacce add
                    # time_add = (abs(self.velocity[i-1][j]))*(self.travel_time[i]-float(self.travel_time[i-1]))/abs(self.velocity[i][j]-self.velocity[i-1][j])
                    
                    # deacce_add = (abs(0-self.velocity[i-1][j]))/(time_add)
                    # position_add = abs(self.velocity[i-1][j])*time_add + 0.5*deacce_add*time_add**2

                    # if position[i-1][j] < position[i-2][j]:
                    #     position_added = position[i-1][j] - position_add
                    # else:
                    #     position_added = position[i-1][j] + position_add
                    # #Add to list
                    # if j == 0:
                    #     # self.joint_0[i+k0] = [position[i][0], velocity[i][0], abs(self.velocity[i][j])/(self.travel_time[i]-float(self.travel_time[i-1])-time_add), 958.7379924]
                    #     self.joint_0.insert(i+k0,[position_added, 0, 958.7379924, deacce_add, time_add + travel_time[i-1] ])
                    #     # self.joint_0[i-1+k0][1]                                 = 0 
                    #     k0 += 1
                    # elif j == 1:
                    #     self.joint_1.insert(i+k1,[position_added, 0, 958.7379924, deacce_add, time_add + travel_time[i-1]])
                    #     # self.joint_1[i-1+k0][1]                                 = 0
                    #     k1 += 1
                    # elif j == 2:
                    #     self.joint_2.insert(i+k2,[position_added, 0, 1198.422491, deacce_add, time_add + travel_time[i-1]])
                    #     # self.joint_2[i-1+k0][1]                                 = 0
                    #     k2 += 1
                    # elif j == 3:
                    #     self.joint_3.insert(i+k3,[position_added, 0, 1198.422491, deacce_add,time_add + travel_time[i-1]])
                    #     # self.joint_3[i-1+k0][1]                                 = 0
                    #     k3 += 1
                    # elif j == 4:
                    #     self.joint_4.insert(i+k4,[position_added, 0, 1917.475985, deacce_add, time_add + travel_time[i-1]])             
                    #     # self.joint_4[i-1+k0][1]                                 = 0
                    #     k4 += 1                           
                    # elif j == 5:
                    #     self.joint_5.insert(i+k5,[position_added, 0, 1917.475985, deacce_add, time_add + travel_time[i-1]])
                    #     # self.joint_5[i-1+k0][1]                                 = 0                        
                    #     k5 += 1
                    if abs(self.velocity[i-1][j]) <= abs(self.velocity[i][j]):
                        self.velocity[i-1][j] = 0 
                    else:
                        self.velocity[i][j] = 0

if __name__ == "__main__":

    mail.servo_on(0)
    mail.servo_on(1)
    mail.servo_on(2)
    mail.servo_on(3)
    mail.servo_on(4)
    mail.servo_on(5)

    mail.move_to_position(5,0,10)
    mail.move_to_position(4,0,10)
    mail.move_to_position(3,0,10)
    mail.move_to_position(2,0,10)
    mail.move_to_position(1,0,10)
    time.sleep(5)
    mail.move_to_position(0,0,10)

    try:
        control = TomoThread()
        control.start()

    except KeyboardInterrupt:
        
        control.__stop()
    
    