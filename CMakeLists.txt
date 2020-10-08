cmake_minimum_required(VERSION 3.0.2)
project(cga_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  urdf
  kdl_parser
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(cga_fk
    src/cga_fk/main.cpp
)
target_link_libraries(cga_fk
    ${catkin_LIBRARIES}
)
target_include_directories(cga_fk PRIVATE
    include/cga_fk
)
