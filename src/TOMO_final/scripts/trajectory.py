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
from TOMO_final.msg import Num
import matplotlib.pyplot as plt
import actionlib_msgs.msg


rospy.init_node('display_trajectory',anonymous=True)

# print(display_trajectory_publisher)

temp = 0 
status = 0
class trajectory_tracking (object):

    def __init__(self,pub):
        self._pub = pub

    def callback(self, data):
        global goal_joint0_R, goal_joint1_R, goal_joint2_R, goal_joint3_R, goal_joint4_R, goal_joint5_R
        global goal_joint0_L, goal_joint1_L, goal_joint2_L, goal_joint3_L, goal_joint4_L, goal_joint5_L
        global position, travel_time, velocity, acceleration,deaccelerate

        travel_time = []
        position = []
        velocity = []
        acceleration = []
        deaccelerate = []
        trajectory = data.goal.trajectory.joint_trajectory.points[0].positions
        trajecttory_list = list(trajectory)
        time = str(data.goal.trajectory.joint_trajectory.points[0].time_from_start)
        name = data.goal.trajectory.joint_trajectory.joint_names
        trajec_velocity = list(data.goal.trajectory.joint_trajectory.points[0].velocities)
        trajec_acceleration = list(data.goal.trajectory.joint_trajectory.points[0].accelerations)

        position.append(trajecttory_list) 
        velocity.append(trajec_velocity)
        # acceleration.append(trajec_acceleration)
        travel_time.append(time)
        joint_names  = name

        print(name)
        print(type(time))

        trajec = Num()
        trajec.positions    = trajecttory_list
        trajec.time         = int(time)
        trajec.joint_names  = name
        self._pub.publish(trajec)


        
        # #Right_arm
        # if (name == ['joint_1_right', 'joint_2_right', 'joint_3_right', 'joint_4_right','joint_5_right','hand_joint']):

        #     goal_joint0_R     =       -trajec.positions[0]    #right_angle
        #     goal_joint1_R     =       -trajec.positions[1]
        #     goal_joint2_R     =       -trajec.positions[2]
        #     goal_joint3_R     =       -trajec.positions[3]
        #     goal_joint4_R     =       -trajec.positions[4]
        #     goal_joint5_R     =       trajec.positions[5]


        #     print('right')
        #     print(trajecttory_list)
        #     print(goal_joint0_R)
        #     print(goal_joint1_R)
        #     print(goal_joint2_R)
        #     print(goal_joint3_R)
        #     print(goal_joint4_R) 
        #     print(goal_joint5_R)
        
        # #Left_arm
        # elif (name == ['joint_1_left', 'joint_2_left', 'joint_3_left', 'joint_4_left','joint_5_left','joint_6_left']):

            
        #     goal_joint0_L     =       -trajec.positions[0]       #left_angle
        #     goal_joint1_L     =       -trajec.positions[1]
        #     goal_joint2_L     =       -trajec.positions[2]
        #     goal_joint3_L     =       trajec.positions[3]
        #     goal_joint4_L     =       -trajec.positions[4]
        #     goal_joint5_L     =       trajec.positions[5]
        
        #     print('left')
        #     print(goal_joint0_L)
        #     print(goal_joint1_L)
        #     print(goal_joint2_L)
        #     print(goal_joint3_L)
        #     print(goal_joint4_L) 
        #     print(goal_joint5_L)

        print(type(trajec.time))
        print(float(trajec.time)/10**9)

        for i in range (1,len(data.goal.trajectory.joint_trajectory.points)):

            trajectory = data.goal.trajectory.joint_trajectory.points[i].positions
            trajecttory_list = list(trajectory)
            time = str(data.goal.trajectory.joint_trajectory.points[i].time_from_start)
            name = data.goal.trajectory.joint_trajectory.joint_names
            trajec_velocity = list(data.goal.trajectory.joint_trajectory.points[i].velocities)
            trajec_acceleration = list(data.goal.trajectory.joint_trajectory.points[i].accelerations)

            print(name)
            print(type(time))

            position.append(trajecttory_list) 
            travel_time.append(time)
            velocity.append(trajec_velocity)
            # acceleration.append(trajec_acceleration)
            joint_names  = name

            trajec = Num()
            trajec.positions    = trajecttory_list
            trajec.time         = int(time)
            trajec.joint_names  = name
            self._pub.publish(trajec)

            
            # #Right_arm
            # if (name == ['joint_1_right', 'joint_2_right', 'joint_3_right', 'joint_4_right','joint_5_right','hand_joint']):

            #     goal_joint0_R     =       -trajec.positions[0]    #right_angle
            #     goal_joint1_R     =       -trajec.positions[1]
            #     goal_joint2_R     =       -trajec.positions[2]
            #     goal_joint3_R     =       -trajec.positions[3]
            #     goal_joint4_R     =       -trajec.positions[4]
            #     goal_joint5_R     =       trajec.positions[5]

            #     print('right')
            #     print(trajecttory_list)
            #     print(goal_joint0_R)
            #     print(goal_joint1_R)
            #     print(goal_joint2_R)
            #     print(goal_joint3_R)
            #     print(goal_joint4_R) 
            #     print(goal_joint5_R)
            
            # #Left_arm
            # elif (name == ['joint_1_left', 'joint_2_left', 'joint_3_left', 'joint_4_left','joint_5_left','joint_6_left']):

                
            #     goal_joint0_L     =       -trajec.positions[0]       #left_angle
            #     goal_joint1_L     =       -trajec.positions[1]
            #     goal_joint2_L     =       -trajec.positions[2]
            #     goal_joint3_L     =       trajec.positions[3]
            #     goal_joint4_L     =       -trajec.positions[4]
            #     goal_joint5_L     =       trajec.positions[5]
            
            #     print('left')
            #     print(goal_joint0_L)
            #     print(goal_joint1_L)
            #     print(goal_joint2_L)
            #     print(goal_joint3_L)
            #     print(goal_joint4_L) 
            #     print(goal_joint5_L)

        print(type(trajec.time))
        print(float(trajec.time)/10**9)

        print(velocity)
        acceleration.append([0,0,0,0,0,0])
        deaccelerate.append([0,0,0,0,0,0])

        for i in range (len(velocity)-1):
            trajec_accelerate = []
            trajec_deaccelerate = []
            for j in range (6):

                if ((velocity[i][j] >= 0 and velocity[i+1][j] >= 0) or (velocity[i][j] <= 0 and velocity[i+1][j] <= 0)):
                    
                    if (abs(velocity[i][j])-abs(velocity[i+1][j])) <= 0:
                        
                        acce = (abs(velocity[i+1][j])-abs(velocity[i][j]))/((float(travel_time[i+1])-float(travel_time[i]))*10**(-9))
                        deacce = 0
                        print('acce:', acce)
                          

                    elif (abs(velocity[i][j])-abs(velocity[i+1][j])) > 0:
                        print('acce:',acce)
                        acce = 0
                        deacce = (abs(velocity[i][j])-abs(velocity[i+1][j]))/((float(travel_time[i+1])-float(travel_time[i]))*10**(-9))

                    trajec_accelerate.append(acce)
                    trajec_deaccelerate.append(deacce)
                    print('trajec_acce: ',trajec_accelerate)
                    print('trajec_deacce: ',trajec_deaccelerate)
                else:
                    acce = 0 
                    deacce = 0
                 ###################################################################

                # elif ((velocity[i][j] >= 0 and velocity[i+1][j] <= 0) or (velocity[i][j] <= 0 and velocity[i+1][j] >= 0)):
                #     print(type(position[i][j]))
                #     travel_time.insert(i+1,travel_time[i]+(abs(velocity[i][j]))*(travel_time[i+1]-travel_time[i])/abs(velocity[i][j]-velocity[i+1][j]))
                #     position_add = (((abs(velocity[i][j]))*(travel_time[i+1]-travel_time[i])/abs(velocity[i][j]-velocity[i+1][j]))*abs(position[i][j]-position[i+1][j]))/(travel_time[i+1]-travel_time[i])
                #     velocity.insert(i+1,0.0)

                #     if position[i+1][j] <= position[i][j]:
                #         position.insert(i+1,position[i][j]-position_add)
                #     elif position[i+1][j] > position[i][j]:
                #         position.insert(i+1,position[i][j]+position_add)

                #     if (abs(velocity[i][j])-abs(velocity[i+1][j])) <= 0.0:
                        
                #         acce = (abs(velocity[i+1][j])-abs(velocity[i][j]))/(float(travel_time[i+1])-float(travel_time[i]))

                #         if j == 0 or j == 1:
                #             deacce = 0
                #         elif j == 2 or j == 3:
                #             deacce = 0
                #         else:
                #             deacce = 0

                #         if acce <= 0.000001 :
                #             if j == 0 or j == 1:
                #                 acce = 0
                #             elif j == 2 or j == 3:
                #                 acce = 0
                #             else:
                #                 acce = 0
                #         # print('acce:', acce)
                            

                #     elif (abs(velocity[i][j])-abs(velocity[i+1][j])) > 0.0:
                #         print('acce:',acce)

                #         if j == 0 or j == 1:
                #             acce = 0
                #         elif j == 2 or j == 3:
                #             acce = 0
                #         else:
                #             acce = 0
                #         deacce = (abs(velocity[i][j])-abs(velocity[i+1][j]))/(float(travel_time[i+1])-float(travel_time[i]))
                #         if deacce <= 0.000001 :
                #             if j == 0 or j == 1:
                #                 deacce = 0
                #             elif j == 2 or j == 3:
                #                 deacce = 0
                #             else:
                #                 deacce = 0
                    trajec_accelerate.append(acce)
                    trajec_deaccelerate.append(deacce)

            acceleration.append(trajec_accelerate)
            deaccelerate.append(trajec_deaccelerate)
            
        
        print('acce: ', acceleration)
        print('deacce:  ',deaccelerate)
        print('test: ', (abs(velocity[1][0])-abs(velocity[0][0]))/(float(travel_time[1])-float(travel_time[0])))







            # #Right_arm
            # if (name == ['joint_1_right', 'joint_2_right', 'joint_3_right', 'joint_4_right','joint_5_right','hand_joint']):

            #     goal_joint0_R     =       -trajec.positions[0]    #right_angle
            #     goal_joint1_R     =       -trajec.positions[1]
            #     goal_joint2_R     =       -trajec.positions[2]
            #     goal_joint3_R     =       -trajec.positions[3]
            #     goal_joint4_R     =       -trajec.positions[4]
            #     goal_joint5_R     =       trajec.positions[5]

            #     print('right')
            #     print(trajecttory_list)
            #     print(goal_joint0_R)
            #     print(goal_joint1_R)
            #     print(goal_joint2_R)
            #     print(goal_joint3_R)
            #     print(goal_joint4_R) 
            #     print(goal_joint5_R)
            
            # #Left_arm
            # elif (name == ['joint_1_left', 'joint_2_left', 'joint_3_left', 'joint_4_left','joint_5_left','joint_6_left']):

        # for i in range (2):
        #     pass
            #     goal_joint0_L     =       -trajec.positions[0]       #left_angle
            #     goal_joint1_L     =       -trajec.positions[1]
            #     goal_joint2_L     =       -trajec.positions[2]
            #     goal_joint3_L     =       trajec.positions[3]
            #     goal_joint4_L     =       -trajec.positions[4]
            #     goal_joint5_L     =       trajec.positions[5]
            
            #     print('left')
            #     print(goal_joint0_L)
            #     print(goal_joint1_L)
            #     print(goal_joint2_L)
            #     print(goal_joint3_L)
            #     print(goal_joint4_L) 
            #     print(goal_joint5_L)

    def callback2(self, data):
        global status, block
        global temp5_R, temp4_R, temp3_R, temp2_R, temp1_R, temp0_R
        global temp5_L, temp4_L, temp3_L, temp2_L, temp1_L, temp0_L
        global temp 
        # status = data.status_list[-1].status
        # if status == 3 :
        #     temp = 0
        # print(status)
        # # print(travel_time)
        # # print(type(travel_time))

        # if status != temp and status != 3:
        #     print('temp: ', temp)
        #     temp = status
        #     print('ok')

def listener():

    print('Hello')
    print('My name Hong gia Bao')

    pub = rospy.Publisher('/tracking_trajectory',Num,queue_size=10000)
    monitor = trajectory_tracking(pub)
    rospy.Subscriber('/execute_trajectory/goal',moveit_msgs.msg.ExecuteTrajectoryActionGoal, monitor.callback)
    rospy.Subscriber('/execute_trajectory/status',actionlib_msgs.msg.GoalStatusArray, monitor.callback2)

    rospy.spin()




if __name__ == '__main__':
    global position, travel_time, velocity, acceleration
    global travel_time_joint0

    velocity_joint0 = []
    velocity_joint1 = []
    position_joint0 = []
    position_joint1 = []

    velocity_joint2 = []
    velocity_joint3 = []
    position_joint2 = []
    position_joint3 = []

    velocity_joint4 = []
    velocity_joint5 = []
    position_joint4 = []
    position_joint5 = []

    acceleration_joint0 = []
    acceleration_joint1 = []
    deaccelerate_joint0 = []
    deaccelerate_joint1 = []

    acceleration_joint2 = []
    acceleration_joint3 = []
    deaccelerate_joint2 = []
    deaccelerate_joint3 = []

    acceleration_joint4 = []
    acceleration_joint5 = []
    deaccelerate_joint4 = []
    deaccelerate_joint5 = []
    listener()

    for i in range (len(velocity)):
        velocity_joint0.append(velocity[i][0])
        velocity_joint1.append(velocity[i][1])
        position_joint0.append(position[i][0])
        position_joint1.append(position[i][1])

        velocity_joint2.append(velocity[i][2])
        velocity_joint3.append(velocity[i][3])
        position_joint2.append(position[i][2])
        position_joint3.append(position[i][3])

        velocity_joint4.append(velocity[i][4])
        velocity_joint5.append(velocity[i][5])
        position_joint4.append(position[i][4])
        position_joint5.append(position[i][5])

        acceleration_joint0.append(acceleration[i][0])
        acceleration_joint1.append(acceleration[i][1])
        deaccelerate_joint0.append(deaccelerate[i][0])
        deaccelerate_joint1.append(deaccelerate[i][1])

        acceleration_joint2.append(acceleration[i][2])
        acceleration_joint3.append(acceleration[i][3])
        deaccelerate_joint2.append(deaccelerate[i][2])
        deaccelerate_joint3.append(deaccelerate[i][3])

        acceleration_joint4.append(acceleration[i][4])
        acceleration_joint5.append(acceleration[i][5])
        deaccelerate_joint4.append(deaccelerate[i][4])
        deaccelerate_joint5.append(deaccelerate[i][5])

    for i in range (len(travel_time)):
        travel_time[i] = round(int(travel_time[i])/10**9,2)

    #Joint_0
    plt.figure()
    plt.subplot(2,2,1)    
    plt.plot(travel_time, velocity_joint0 )
    plt.grid()
    plt.title('Joint 0')
    plt.ylabel('velocity (rad/s))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,2)    
    plt.plot(travel_time, position_joint0)
    plt.grid()
    plt.title('Joint 0')
    plt.ylabel('position (rad))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,3)    
    plt.plot(travel_time, acceleration_joint0 )
    plt.grid()
    plt.title('Joint 0')
    plt.ylabel('accelerate (rad/s^2)')
    plt.xlabel('time (s)')

    plt.subplot(2,2,4)    
    plt.plot(travel_time, deaccelerate_joint0 )
    plt.grid()
    plt.title('Joint 0')
    plt.ylabel('deaccelerate (rad/s^2)')
    plt.xlabel('time (s)')

    #Joint_1
    plt.figure()
    plt.subplot(2,2,1)    
    plt.plot(travel_time, velocity_joint1 )
    plt.grid()
    plt.title('Joint 1')
    plt.ylabel('velocity (rad/s))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,2)    
    plt.plot(travel_time, position_joint1)
    plt.grid()
    plt.title('Joint 1')
    plt.ylabel('position (rad))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,3)    
    plt.plot( travel_time, acceleration_joint1 )
    plt.grid()
    plt.title('Joint 1')
    plt.ylabel('accelerate (rad/s^2)')
    plt.xlabel('time (s)')

    plt.subplot(2,2,4)    
    plt.plot(travel_time, deaccelerate_joint1 )
    plt.grid()
    plt.title('Joint 1')
    plt.ylabel('deaccelerate (rad/s^2)')
    plt.xlabel('time (s)')


    #Joint_2
    plt.figure()
    plt.subplot(2,2,1)    
    plt.plot(travel_time, velocity_joint2 )
    plt.grid()
    plt.title('Joint 2')
    plt.ylabel('velocity (rad/s))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,2)    
    plt.plot( travel_time, position_joint2)
    plt.grid()
    plt.title('Joint 2')
    plt.ylabel('position (rad))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,3)    
    plt.plot( travel_time, acceleration_joint2 )
    plt.grid()
    plt.title('Joint 2')
    plt.ylabel('accelerate (rad/s^2)')
    plt.xlabel('time (s)')

    plt.subplot(2,2,4)    
    plt.plot( travel_time, deaccelerate_joint2  )
    plt.grid()
    plt.title('Joint 2')
    plt.ylabel('deaccelerate (rad/s^2)')
    plt.xlabel('time (s)')

    #Joint_3
    plt.figure()
    plt.subplot(2,2,1)    
    plt.plot(travel_time, velocity_joint3 )
    plt.grid()
    plt.title('Joint 3')
    plt.ylabel('velocity (rad/s))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,2)    
    plt.plot( travel_time, position_joint3)
    plt.grid()
    plt.title('Joint 3')
    plt.ylabel('position (rad))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,3)    
    plt.plot( travel_time, acceleration_joint3 )
    plt.grid()
    plt.title('Joint 3')
    plt.ylabel('accelerate (rad/s^2)')
    plt.xlabel('time (s)')

    plt.subplot(2,2,4)    
    plt.plot( travel_time, deaccelerate_joint3  )
    plt.grid()
    plt.title('Joint 3')
    plt.ylabel('deaccelerate (rad/s^2)')
    plt.xlabel('time (s)')


    #Joint_4
    plt.figure()
    plt.subplot(2,2,1)    
    plt.plot(travel_time, velocity_joint4 )
    plt.grid()
    plt.title('Joint 4')
    plt.ylabel('velocity (rad/s))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,2)    
    plt.plot( travel_time, position_joint4)
    plt.grid()
    plt.title('Joint 4')
    plt.ylabel('position (rad))')
    plt.xlabel('time (s)')
    
    plt.subplot(2,2,3)    
    plt.plot( travel_time,acceleration_joint4 )
    plt.grid()
    plt.title('Joint 4')
    plt.ylabel('accelerate (rad/s^2)')
    plt.xlabel('time (s)')

    plt.subplot(2,2,4)    
    plt.plot( travel_time, deaccelerate_joint4 )
    plt.grid()
    plt.title('Joint 4')
    plt.ylabel('deaccelerate (rad/s^2)')
    plt.xlabel('time (s)')

    plt.show()