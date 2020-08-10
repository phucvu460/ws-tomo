#!/usr/bin/env python
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
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "tomo_left_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=200000)
group.set_goal_tolerance(0.0001)

# # We can get the name of the reference frame for this robot:
# planning_frame = group.get_planning_frame()
# print "============ Reference frame: %s" % planning_frame379669p:
# eef_link = group.get_end_effector_link()
# print "============ End effector: %s" % eef_link

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print "============ Robot Groups:", robot.get_group_names()
# ,
#                   po_x2,po_y2,po_z2,or_x2,or_y2,or_z2,or_w2,
#                   po_x3,po_y3,po_z3,or_x3,or_y3,or_z3,or_w3,
#                   po_x4,po_y4,po_z4,or_x4,or_y4,or_z4,or_w4,
#                   po_x5,po_y5,po_z5,or_x5,or_y5,or_z5,or_w5,

# def cartesian_run(po_x1,po_y1,po_z1,or_x1,or_y1,or_z1,or_w1):
#     waypoints = []
#     waypoints.append(copy.deepcopy(group.get_current_pose().pose))
#     wpose = geometry_msgs.msg.Pose()
#     #Pose_1
#     wpose.position.x = po_x1
#     wpose.position.z = po_z1  # First move up (z)
#     wpose.position.y = po_y1  # and sideways (y)
#     wpose.orientation.x = or_x1  # Third move sideways (y)
#     wpose.orientation.y = or_y1  # Third move sideways (y)
#     wpose.orientation.z = or_z1 # Third move sideways (y)
#     wpose.orientation.w = or_w1  # Third move sideways (y)
#     waypoints.append(copy.deepcopy(wpose))
#     # # #Pose_2
#     # wpose.position.x = po_x2
#     # wpose.position.z = po_z2  # First move up (z)
#     # wpose.position.y = po_y2  # and sideways (y)
#     # wpose.orientation.x = or_x2  # Third move sideways (y)
#     # wpose.orientation.y = or_y2  # Third move sideways (y)
#     # wpose.orientation.z = or_z2 # Third move sideways (y)
#     # wpose.orientation.w = or_w2  # Third move sideways (y)
#     # waypoints.append(copy.deepcopy(wpose))
#     # # #Pose_3
#     # wpose.position.x = po_x3
#     # wpose.position.z = po_z3  # First move up (z)
#     # wpose.position.y = po_y3  # and sideways (y)
#     # wpose.orientation.x = or_x3  # Third move sideways (y)
#     # wpose.orientation.y = or_y3  # Third move sideways (y)
#     # wpose.orientation.z = or_z3 # Third move sideways (y)
#     # wpose.orientation.w = or_w3  # Third move sideways (y)
#     # waypoints.append(copy.deepcopy(wpose))
#     # #Pose_4
#     # wpose.position.x = po_x4
#     # wpose.position.z = po_z4  # First move up (z)
#     # wpose.position.y = po_y4  # and sideways (y)
#     # wpose.orientation.x = or_x4  # Third move sideways (y)
#     # wpose.orientation.y = or_y4  # Third move sideways (y)
#     # wpose.orientation.z = or_z4 # Third move sideways (y)
#     # wpose.orientation.w = or_w4  # Third move sideways (y)
#     # waypoints.append(copy.deepcopy(wpose))
#     # #Pose_5
#     # wpose.position.x = po_x5
#     # wpose.position.z = po_z5  # First move up (z)
#     # wpose.position.y = po_y5  # and sideways (y)
#     # wpose.orientation.x = or_x5  # Third move sideways (y)
#     # wpose.orientation.y = or_y5  # Third move sideways (y)
#     # wpose.orientation.z = or_z5 # Third move sideways (y)
#     # wpose.orientation.w = or_w5  # Third move sideways (y)
#     # waypoints.append(copy.deepcopy(wpose))
#     # group.set_goal_tolerance(0.0001);    # We want the Cartesian path to be interpolated at a resolution of 1 cm
#     group.set_start_state_to_current_state()
#     (plan, fraction) = group.compute_cartesian_path (
#                                     waypoints,   # waypoint poses
#                                     0.1,        # eef_step
#                                     0.0,         # jump_threshold
#                                     True)        # avoid_collisions

#         # Increment the number of attempts
#     # (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01,0.0)       

#     # Note: We are just planning, not asking move_group to actually move the root yet:
#     plan = group.retime_trajectory(robot.get_current_state(),plan,0.1,0.5)
#     print(plan)
#     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#     # display_trajectory.trajectory_start = robot.get_current_state()
#     display_trajectory.trajectory.append(plan)
#     # Publish
#     display_trajectory_publisher.publish(display_trajectory)
#     # time.sleep(5)
#     if fraction == 1:
#         group.execute(plan,wait = True)
#     return plan, fraction
# #LEFT_ARM


def cartesian_run(point_list):
    waypoints = []
    waypoints.append(copy.deepcopy(group.get_current_pose().pose))
    wpose = geometry_msgs.msg.Pose()



    for i in range(len(point_list)):

    #Pose_1
        wpose.position.x = point_list[i][0]
        wpose.position.y = point_list[i][1]  # and sideways (y)
        wpose.position.z = point_list[i][2]  # First move up (z)
        wpose.orientation.x = point_list[i][3]  # Third move sideways (y)
        wpose.orientation.y = point_list[i][4]  # Third move sideways (y)
        wpose.orientation.z = point_list[i][5] # Third move sideways (y)
        wpose.orientation.w = point_list[i][6]  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

    # group.set_goal_tolerance(0.0001);    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    group.set_start_state_to_current_state()
    (plan_1, fraction) = group.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    1,        # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions

        # Increment the number of attempts
    # (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01,0.0)       

    # Note: We are just planning, not asking move_group to actually move the root yet:
    plan = group.retime_trajectory(robot.get_current_state(),plan_1,0.1,0.1)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    print(display_trajectory)

    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    # time.sleep(5)
    group.execute(plan,wait = True)
    return plan, fraction

if __name__ == '__main__':
    # cartesian_run(po_x1 = 0.321488854735       ,
    #                 po_y1 = 0.280396099076     ,
    #                 po_z1 = 0.595671305846     ,
    #                 or_x1 = -0.602824221627    ,
    #                 or_y1 = -0.703452981814    ,
    #                 or_z1 = -0.0720126168823   ,
    #                 or_w1 = 0.369555196427     ,
    #                 po_x2 = 0.316162857248     ,
    #                 po_y2 = 0.273323595273     ,
    #                 po_z2 = 0.402604219994     ,
    #                 or_x2 = -0.411284524629    ,
    #                 or_y2 = -0.636113344795    ,
    #                 or_z2 = -0.413691836814    ,
    #                 or_w2 = 0.505038529        ,
    #                 po_x3 = 0.481391181309     ,
    #                 po_y3 = 0.299938340933     ,
    #                 po_z3 = 0.425264935645     ,
    #                 or_x3 = -0.452101201165    ,
    #                 or_y3 = -0.599103797265    ,
    #                 or_z3 = -0.465767253142    ,
    #                 or_w3 = 0.468764343682745  ,
    #                 po_x4 = 0.490153319093     ,
    #                 po_y4 = 0.238661473169     ,
    #                 po_z4 = 0.433159096071     ,
    #                 or_x4 = -0.485169677511    ,
    #                 or_y4 = -0.563505448034    ,
    #                 or_z4 = -0.508248257332    ,
    #                 or_w4 = 0.434460243266     ,
    #                 po_x5 = 0.510016462582     ,
    #                 po_y5 = 0.228544565803     ,
    #                 po_z5 = 0.304167025073     ,
    #                 or_x5 = -0.459968184719    ,
    #                 or_y5 = -0.541787805745    ,
    #                 or_z5 = -0.531137711057    ,
    #                 or_w5 = 0.461289469298      )
    list_point = []
    list_point.append( [0.321494363358, 0.265754890197, 0.260412519144, -0.510705872827, -0.488937072903 , 0.510844450331 , 0.489037828564 ] )
    
    list_point.append( [0.321526739754, 0.267345965249, 0.29743800251, 
                  -0.301246657891, -0.705645448708,-0.0463815355004, 0.639659053708])

    # list_point.append( [0.422782, 0.320909, 0.452966, -0.4967722, -0.5153, -0.484298, 0.503178])
    list_point.append( [0.403138, 0.05639,  0.29816, -0.4967722, -0.5153, -0.484298, 0.503178])
    # list_point.append( [0.406275, 0.0132106, 0.477473, -0.4967722, -0.5153, -0.484298, 0.503178])

    list_point.append( [0.448932, 0.312842, 0.196445, -0.4967722, -0.5153, -0.484298, 0.503178])
    # list_point.append([0.379669,0.292189,0.308982,-0.483682,-0.504274,-0.495697,0.515794])
    
    # list_point.append([0.391264, 0.024747, 0.285496,-0.48351,-0.50426,-0.495888,0.515787])



    list_point.append([ 0.321515872543, 0.257993390497, 0.0840145329647, 
                        -0.510860739162, -0.488974572434, 0.51077366192, 0.488912506472])

    # cartesian_run(po_x1 = 0.379669,
    #                 po_y1 = 0.292189    ,
    #                 po_z1 = 0.308982   ,
    #                 or_x1 = -0.483682      ,
    #                 or_y1 = -0.504274     ,
    #                 or_z1 = -0.495697   ,
    #                 or_w1 = 0.515794      )
    while(True):
        cartesian_run( list_point)
        time.sleep(8)
        except KeyboardInterrupt:
            break
###################################33

                    # 0.422782
                    # 0.320909
                    # 0.452966

                    # -0.4967722
                    # -0.5153
                    # -0.484298
                    # 0.503178




################################33
                    # 0.403138
                    # 0.05639
                    # 0.29816
                    
                    # -0.4967722
                    # -0.5153
                    # -0.484298
                    # 0.503178


####################################

                    # 0.406275
                    # 0.0132106
                    # 0.477473

                    # -0.4967722
                    # -0.5153
                    # -0.484298
                    # 0.503178


###################################

                    # 0.448932
                    # 0.312842
                    # 0.196445

                    # -0.4967722
                    # -0.5153
                    # -0.484298
                    # 0.503178







        
        #   po_x4 = 0.490153319093     ,
        #   po_y4 = 0.238661473169     ,
        #   po_z4 = 0.433159096071     ,
        #   or_x4 = -0.485169677511    ,
        #   or_y4 = -0.563505448034    ,
        #   or_z4 = -0.508248257332    ,
        #   or_w4 = 0.434460243266     ,
        #   po_x5 = 0.510016462582     ,
        #   po_y5 = 0.228544565803     ,
        #   po_z5 = 0.304167025073     ,
        #   or_x5 = -0.459968184719    ,
        #   or_y5 = -0.541787805745    ,
        #   or_z5 = -0.531137711057    ,
        #   or_w5 = 0.461289469298     )




    # cartesian_run(po_x1 = 0.321488854735     ,
    #           po_y1 = 0.280396099076     ,
    #           po_z1 = 0.595671305846     ,
    #           or_x1 = -0.602824221627    ,
    #           or_y1 = -0.703452981814    ,
    #           or_z1 = -0.0720126168823   ,
    #           or_w1 = 0.369555196427     ,
    #           po_x2 = 0.316162857248     ,
    #           po_y2 = 0.273323595273     ,
    #           po_z2 = 0.402604219994     ,
    #           or_x2 = -0.411284524629    ,
    #           or_y2 = -0.636113344795    ,
    #           or_z2 = -0.413691836814    ,
    #           or_w2 = 0.505038529        ,
    #           po_x3 = 0.481391181309     ,
    #           po_y3 = 0.299938340933     ,
    #           po_z3 = 0.425264935645     ,
    #           or_x3 = -0.452101201165    ,
    #           or_y3 = -0.599103797265    ,
    #           or_z3 = -0.465767253142    ,
    #           or_w3 = 0.468764343682745  )
    #         #   po_x4 = 0.490153319093     ,
    #         #   po_y4 = 0.238661473169     ,
    #         #   po_z4 = 0.433159096071     ,
    #         #   or_x4 = -0.485169677511    ,
    #         #   or_y4 = -0.563505448034    ,
    #         #   or_z4 = -0.508248257332    ,
    #         #   or_w4 = 0.434460243266     ,
    #         #   po_x5 = 0.510016462582     ,
    #         #   po_y5 = 0.228544565803     ,
    #         #   po_z5 = 0.304167025073     ,
    #         #   or_x5 = -0.459968184719    ,
    #         #   or_y5 = -0.541787805745    ,
    #         #   or_z5 = -0.531137711057    ,
    #         #   or_w5 = 0.461289469298     )
