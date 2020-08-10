#!/usr/bin/env python2.7
import rospy
from moveit_python import *
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time
from TOMO_final.msg import position_blisters  

import sys
sys.path.append("..")
from moveit_msgs.msg import PlanningScene
import collision as add_ground
import transformation as tf

class Add_ring(object): 

    def __init__(self,_po_x,_po_y,_po_z,_ori_x,_ori_y,_ori_z,_ori_w):
        
        self._scene = PlanningSceneInterface()

        # clear the scene and init 
        self._scene.remove_world_object()
        self.ground = add_ground.CollisionSceneExample()
        self.ground.add_one_box()
        self.Hole = []
        self.robot = RobotCommander()
        self.po_x = _po_x 
        self.po_y = _po_y  
        self.po_z = 0  
        self.ori_x = _ori_x  
        self.ori_y = _ori_y 
        self.ori_z = _ori_z 
        self.ori_w = _ori_w
        
        # pause to wait for rviz to load
        print ("============ Waiting while RVIZ displays the scene with obstacles...")
        # TODO: need to replace this sleep by explicitly waiting for the scene to be updated.
        rospy.sleep(1)

    def add_rings(self):

        #Get PoseStamped from ROS
        for i in range (len(self.po_x)):
            self.Hole.append(geometry_msgs.msg.PoseStamped())
            
        #Set information of blisters seen by camera for ROS
        for i in range (len(self.po_x)):
            self.Hole[i].header.frame_id = self.robot.get_planning_frame()
            self.Hole[i].header.stamp = rospy.Time.now()
            self.Hole[i].pose.position.x = self.po_x[i]
            self.Hole[i].pose.position.y = self.po_y[i]
            self.Hole[i].pose.position.z = 0          
            self.Hole[i].pose.orientation.x =  self.ori_x[i]
            self.Hole[i].pose.orientation.y =  self.ori_y[i]
            self.Hole[i].pose.orientation.z =  self.ori_z[i]
            self.Hole[i].pose.orientation.w =  self.ori_w[i]
            angles = tf.euler_from_quaternion([self.Hole[i].pose.orientation.w, self.Hole[i].pose.orientation.x, self.Hole[i].pose.orientation.y, self.Hole[i].pose.orientation.z]) #w,x,y,z

        self.Hole_id = []

        for i in range (len(self.po_x)):
            self.Hole_id.append("Hole_" + str(i))
        for i in range (len(self.po_x)):
            self._scene.remove_world_object(self.Hole_id[i])
        for i in range (len(self.po_x)):
            self._scene.add_mesh(self.Hole_id[i],self.Hole[i],"/home/tomo/Hole_3.STL")

def callback(data):

    global _po_x, _po_y, _po_z, _ori_x, _ori_y, _ori_z, _ori_w
    _po_x = data.po_x
    _po_y = data.po_y
    _po_z = data.po_z

    _ori_x = data.ori_x
    _ori_y = data.ori_y
    _ori_z = data.ori_z
    _ori_w = data.ori_w

if __name__ == "__main__":
    rospy.init_node("Add_ring")
    time.sleep(1)
    # while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
    #     time.sleep(0.5)
    # rospy.Subscriber('/position_orientation_blisters',position_blisters, callback) 
    # time.sleep(1)
    # load_scene = Add_ring(_po_x,_po_y,_po_z,_ori_x,_ori_y,_ori_z,_ori_w)
    # load_scene.add_rings()
    pass
