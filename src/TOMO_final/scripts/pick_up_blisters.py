#!/usr/bin/python2.7
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import Add_ring as ring
from TOMO_final.msg import position_blisters , Num
import transformation as tf
import math
moveit_commander.roscpp_initialize(sys.argv)
pub = rospy.Publisher('/status_hand', Num, queue_size=10)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "tomo_left_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=30)

group.set_goal_tolerance(0.0001)

def cartesian_run(po_x, po_y, po_z, or_x, or_y, or_z, or_w):
    group.clear_pose_targets()
    waypoints = []
    wpose = group.get_current_pose().pose

    wpose.position.x =  po_x
    wpose.position.y =  po_y
    wpose.position.z =  po_z
    wpose.orientation.x = or_x
    wpose.orientation.y = or_y
    wpose.orientation.z = or_z
    wpose.orientation.w = or_w
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    1,           # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions


    # Note: We are just planning, not asking move_group to actually move the root yet:
    plan = group.retime_trajectory(robot.get_current_state(),plan,1,1)
    print(plan)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    group.execute(plan,wait = True)

    return plan, fraction
def cartesian_step_1(po_x, po_y, po_z, or_x, or_y, or_z, or_w):
    group.clear_pose_targets()
    waypoints = []
    wpose = group.get_current_pose().pose

    #Set goal traject for Cartesian planner                        
    wpose.position.x =  0.31036828333 #mediate pose
    wpose.position.y =  0.15943643252
    wpose.position.z =  0.184034235235
    wpose.orientation.x = 0.0101582770739
    wpose.orientation.y = -0.642686360084
    wpose.orientation.z = -0.62584923323
    wpose.orientation.w = 0.441773459179
    waypoints.append(copy.deepcopy(wpose))

    
    wpose.position.x =  po_x      #reach to blister and pick up 
    wpose.position.y =  po_y
    wpose.position.z =  po_z
    wpose.orientation.x = or_x
    wpose.orientation.y = or_y
    wpose.orientation.z = or_z
    wpose.orientation.w = or_w
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    1,           # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions


    # Note: We are just planning, not asking move_group to actually move the root yet:
    plan = group.retime_trajectory(robot.get_current_state(),plan,1,1)
    print(plan)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    group.execute(plan,wait = True)

    return plan, fraction

def cartesian_step_2(num_blis,id):
    group.clear_pose_targets()
    waypoints = []
    wpose = group.get_current_pose().pose

    #Set goal traject for Cartesian planner                        
    wpose.position.x =  0.318927836539 #mediate pose
    wpose.position.y =  0.258896454792
    wpose.position.z =  0.19566134024
    wpose.orientation.x = -0.0352220159923
    wpose.orientation.y = -0.778008207318
    wpose.orientation.z = -0.258371979902
    wpose.orientation.w = 0.57158250405
    waypoints.append(copy.deepcopy(wpose))

    if id % 2 == 0:
        wpose.position.x =  0.311555279681 + num_blis*0.05      #reach to blister and pick up 
        wpose.position.y =  0.350783563244 
        wpose.position.z =  0.161352795951
        wpose.orientation.x = 0.462180594718
        wpose.orientation.y = -0.53510846888
        wpose.orientation.z = -0.462238904111
        wpose.orientation.w = 0.53514784866
        waypoints.append(copy.deepcopy(wpose))
    else:
        wpose.position.x =  0.311555279681 + num_blis*0.05      #reach to blister and pick up 
        wpose.position.y =  0.350783563244 + 0.07
        wpose.position.z =  0.161352795951
        wpose.orientation.x = 0.462180594718
        wpose.orientation.y = -0.53510846888
        wpose.orientation.z = -0.462238904111
        wpose.orientation.w = 0.53514784866
        waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    1,           # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions


    # Note: We are just planning, not asking move_group to actually move the root yet:
    plan = group.retime_trajectory(robot.get_current_state(),plan,1,1)
    print(plan)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    group.execute(plan,wait = True)

    return plan, fraction

def callback(data):
    global _po_x, _po_y, _po_z, _ori_x, _ori_y, _ori_z, _ori_w
    _po_x = data.po_x
    _po_y = data.po_y
    _po_z = data.po_z
    _ori_x = data.ori_x
    _ori_y = data.ori_y
    _ori_z = data.ori_z
    _ori_w = data.ori_w

if __name__ == '__main__':
    while(True):
        _po_x = []
        status_hand = Num()

        while _po_x == []:
            rospy.Subscriber('/position_orientation_blisters',position_blisters, callback) 
            time.sleep(1)
            if _po_x != []:
                break 

        if len(_po_x) > 0:
            time.sleep(3)
            load_scene = ring.Add_ring(_po_x,_po_y,_po_z,_ori_x,_ori_y,_ori_z,_ori_w)
            load_scene.add_rings()
            try:
                num_blis = 0
                for i in range (len(load_scene.Hole)):
                    
                    #Caculate euler angles of blisters from quaternions
                    angles = tf.euler_from_quaternion([load_scene.Hole[i].pose.orientation.w,             \
                                                        load_scene.Hole[i].pose.orientation.x,             \
                                                        load_scene.Hole[i].pose.orientation.y,             \
                                                        load_scene.Hole[i].pose.orientation.z]) 
                    print("X: ", angles[0])
                    print("Y: ", angles[1])
                    print("Z: ", angles[2])
                    angle_Z = angles[2]*180/math.pi

                    if i % 2 == 0 and i != 0:
                        num_blis += 1
                    
                    # #status_hand != 0 : close hand
                    # #status_hand == 0 : open hand
                    # status_hand.status_hand = 0 #1
                    # status_hand.positions = []
                    # status_hand.time = 0
                    # status_hand.joint_names = []

                    # #publish to ROS message
                    # pub.publish(status_hand)
                    # pub.publish(status_hand)
                    # pub.publish(status_hand)

                    # #mediate arm pose 
                    # cartesian_run(  po_x = 0.31036828333,
                    #                 po_y = 0.15943643252,
                    #                 po_z = 0.184034235235,
                    #                 or_x = 0.0101582770739,
                    #                 or_y = -0.642686360084,
                    #                 or_z = -0.62584923323,
                    #                 or_w = 0.441773459179)
                    # time.sleep(0.5)

                    # 3 poses corresponding to 3 directions of blisters 
                    # right, left, straight
                    #Right direction: 
                    if (angle_Z > -20 and angle_Z < 70) or  (angle_Z > -180 and angle_Z < -110):

                        #close hand
                        status_hand.status_hand = 2 
                        status_hand.positions = []
                        status_hand.time = 0
                        status_hand.joint_names = []

                        #publish to ROS message
                        pub.publish(status_hand)
                        pub.publish(status_hand)
                        pub.publish(status_hand)

                        #Reach to blister and pick
                        cartesian_step_1( po_x =  load_scene.Hole[i].pose.position.x -0.02022964683 ,
                                    po_y = load_scene.Hole[i].pose.position.y + 0.07093467509, #thumbfinger from touching sensor position to intial position = 160
                                    po_z = 0.143465951548  ,        
                                    or_x = 0.137029954955,
                                    or_y = -0.548901214555,
                                    or_z = -0.788602125413,
                                    or_w = 0.240908563358)
                        time.sleep(9)

                        #close hand
                        status_hand.status_hand = 3
                        status_hand.positions = []
                        status_hand.time = 0
                        status_hand.joint_names = []
                        pub.publish(status_hand)
                        pub.publish(status_hand)
                        pub.publish(status_hand)

                        #place blister to outside FOV
                        cartesian_step_2(num_blis,i)
                        time.sleep(0.5)

                        #open hand
                        status_hand.status_hand = 0
                        status_hand.positions = []
                        status_hand.time = 0
                        status_hand.joint_names = []
                        pub.publish(status_hand)
                        pub.publish(status_hand)

                        #place blister in 2 collumn
                        if i % 2 == 0:
                            cartesian_run( po_x =  0.312134551051 +num_blis*0.05,
                                        po_y = 0.345645462779 ,  
                                        po_z = 0.130465951548  ,        
                                        or_x = 0.407634526264,
                                        or_y = -0.487170013717,
                                        or_z = -0.512572340938,
                                        or_w = 0.577727501542)
                            time.sleep(2)
                        else:
                            cartesian_run( po_x =  0.312134551051 + num_blis*0.05, 
                                        po_y = 0.345645462779 + 0.07,
                                        po_z = 0.130465951548  ,
                                        or_x = 0.407634526264,
                                        or_y = -0.487170013717,
                                        or_z = -0.512572340938,
                                        or_w = 0.577727501542)
                            time.sleep(2)

                    #Like above sequence with left direction of blisters)
                    else:
                        status_hand.status_hand = 1 #1
                        status_hand.positions = []
                        status_hand.time = 0
                        status_hand.joint_names = []
                        pub.publish(status_hand)
                        pub.publish(status_hand)
                        cartesian_step_1( po_x =  load_scene.Hole[i].pose.position.x -0.02114895293, 
                                    po_y = load_scene.Hole[i].pose.position.y + 0.09226573438,  
                                    po_z = 0.139389886928  ,        
                                    or_x = 0.142115057078,
                                    or_y = -0.565574940312,
                                    or_z = -0.776718070038,
                                    or_w = 0.237986001939)
                        time.sleep(7)

                        status_hand.status_hand = 4 
                        status_hand.positions = []
                        status_hand.time = 0
                        status_hand.joint_names = []
                        pub.publish(status_hand)
                        pub.publish(status_hand)

                        cartesian_step_2(num_blis,i)
                        time.sleep(0.5)


                        status_hand.status_hand = 0
                        status_hand.positions = []
                        status_hand.time = 0
                        status_hand.joint_names = []
                        pub.publish(status_hand)
                        pub.publish(status_hand)
                        if i % 2 == 0:
                            cartesian_run( po_x =  0.312134551051 + num_blis*0.05, 
                                        po_y = 0.345645462779 , 
                                        po_z = 0.130465951548  ,        
                                        or_x = 0.407634526264,
                                        or_y = -0.487170013717,
                                        or_z = -0.512572340938,
                                        or_w = 0.577727501542)
                            time.sleep(2)
                        else:
                            cartesian_run( po_x =  0.312134551051 + num_blis*0.05, 
                                        po_y = 0.345645462779 + 0.07,  
                                        po_z = 0.130465951548  ,        
                                        or_x = 0.407634526264,
                                        or_y = -0.487170013717,
                                        or_z = -0.512572340938,
                                        or_w = 0.577727501542)
                            time.sleep(2)
                    
                # Home
                cartesian_run( po_x =  0.321522657778 , 
                            po_y =  0.257962775733,  
                            po_z = 0.0840144933305,        
                            or_x = -0.510860586206,
                            or_y = -0.488975544678,
                            or_z = 0.510753614117,
                            or_w = 0.488932637318)
                time.sleep(7)
            except KeyboardInterrupt:
                break
                # pass




