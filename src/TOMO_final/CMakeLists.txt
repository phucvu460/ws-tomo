cmake_minimum_required(VERSION 3.1.3)
project(TOMO_final)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   warehouse_ros
)

add_message_files(
  FILES
  Num.msg
  position_blisters.msg
  cam_to_object.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime 
  LIBRARIES warehouse_ros_mongo
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
