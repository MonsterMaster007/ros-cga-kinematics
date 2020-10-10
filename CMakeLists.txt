cmake_minimum_required(VERSION 3.0.2)
project(cga_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  urdf
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# In future, put contraint code in library target
add_executable(loop_fk
    src/loop_fk/main.cpp
    src/constraint/constraint.cpp
    src/constraint/delta.cpp
    src/constraint/parallel.cpp
)
target_link_libraries(loop_fk
    ${catkin_LIBRARIES}
)
target_include_directories(loop_fk PRIVATE
    include
)
