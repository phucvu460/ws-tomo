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
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

group.set_goal_tolerance(0.0001)

# # We can get the name of the reference frame for this robot:
# planning_frame = group.get_planning_frame()
# print "============ Reference frame: %s" % planning_frame

# # We can also print the name of the end-effector link for this group:
# eef_link = group.get_end_effector_link()
# print "============ End effector: %s" % eef_link

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# # robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""
joint_goal = group.get_current_joint_values()
print(group.get_current_pose())
print "============ Generating plan 1"
group.get_current_pose().pose
# pose_target = geometry_msgs.msg.Pose()

# pose_target.position.x = 0.2 
# pose_target.position.y = 0.2   
# # pose_target.position.z = 0.595671305846
# # pose_target.orientation.x = -0.602824221627
# # pose_target.orientation.y = -0.703452981814
# # pose_target.orientation.z = -0.0720126168823
# # pose_target.orientation.w = 0.369555196427

# group.set_pose_target(pose_target)

# plan = group.go()
# # Calling `stop()` ensures that there is no residual movement
# group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()
# group.clear_pose_targets()

# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()


# def cartesian_run(po_x, po_y, po_z, or_x, or_y, or_z, or_w):
#     scale = 1
#     group.clear_pose_targets()
#     waypoints = []

#     wpose = group.get_current_pose().pose


#     wpose.position.x =  po_x
#     wpose.position.y =  po_y
#     wpose.position.z =  po_z
#     wpose.orientation.x = or_x
#     wpose.orientation.y = or_y
#     wpose.orientation.z = or_z
#     wpose.orientation.w = or_w
#     waypoints.append(copy.deepcopy(wpose))

#     # wpose.position.x =  0.393419724031-0.1
#     # wpose.position.y =  0.260544494637-0.05
#     # wpose.position.z =  0.170894186415
#     # wpose.orientation.x = -0.16472564503
#     # wpose.orientation.y = -0.250248486538
#     # wpose.orientation.z = -0.533774332684
#     # wpose.orientation.w = 0.790775643671
#     # waypoints.append(copy.deepcopy(wpose))

#     # wpose.position.x =  0.393419724031
#     # wpose.position.y =  0.260544494637-0.05
#     # wpose.position.z =  0.170894186415
#     # wpose.orientation.x = -0.16472564503
#     # wpose.orientation.y = -0.250248486538
#     # wpose.orientation.z = -0.533774332684
#     # wpose.orientation.w = 0.790775643671
#     # waypoints.append(copy.deepcopy(wpose))

#     # wpose.position.x =  0.393419724031-0.1
#     # wpose.position.y =  0.260544494637
#     # wpose.position.z =  0.170894186415
#     # wpose.orientation.x = -0.16472564503
#     # wpose.orientation.y = -0.250248486538
#     # wpose.orientation.z = -0.533774332684
#     # wpose.orientation.w = 0.790775643671
#     # waypoints.append(copy.deepcopy(wpose))

#     # We want the Cartesian path to be interpolated at a resolution of 1 cm
#     # which is why we will specify 0.01 as the eef_step in Cartesian
#     # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
#     group.set_start_state_to_current_state()
#     (plan, fraction) = group.compute_cartesian_path (
#                                     waypoints,   # waypoint poses
#                                     0.2,        # eef_step
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
#         pass
#     return plan, fraction

# if __name__ == '__main__':
#     try:
#         cartesian_run( po_x = 0.353244601948,
#                        po_y = 0.241779765583,
#                        po_z = 0.298871758847,
#                        or_x = 0.116096952893,
#                        or_y = -0.697451565253,
#                        or_z = -0.115544252034,
#                        or_w = 0.697662051052)

#         time.sleep(2)

#         cartesian_run( po_x = 0.393419724031-0.1,
#                        po_y = 0.260544494637-0.05,
#                        po_z = 0.170894186415,
#                        or_x = -0.16472564503,
#                        or_y = -0.250248486538,
#                        or_z = -0.533774332684,
#                        or_w = 0.790775643671)

#         time.sleep(2)

#         cartesian_run( po_x = 0.353244601948,
#                        po_y = 0.241779765583,
#                        po_z = 0.298871758847,
#                        or_x = 0.116096952893,
#                        or_y = -0.697451565253,
#                        or_z = -0.115544252034,
#                        or_w = 0.697662051052)

#         time.sleep(2)

#         cartesian_run( po_x = 0.393419724031,
#                        po_y = 0.260544494637-0.05,
#                        po_z = 0.170894186415,
#                        or_x = -0.16472564503,
#                        or_y = -0.250248486538,
#                        or_z = -0.533774332684,
#                        or_w = 0.790775643671)

#         time.sleep(2)
#     except KeyboardInterrupt:
        
#         pass
