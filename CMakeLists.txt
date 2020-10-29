cmake_minimum_required(VERSION 3.0.2)
project(cga_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  urdf
  gazebo_ros
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        tf2_ros
        urdf
        gazebo_ros
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(cpp_cga STATIC IMPORTED)
set_target_properties(cpp_cga PROPERTIES IMPORTED_LOCATION /usr/local/lib/libcpp_cga.a)
set(cpp_cga_INCLUDE_DIRS /usr/local/include/cga)

# In future, put contraint code in library target
add_executable(delta_fk
    src/delta_fk/main.cpp
    src/constraint/delta.cpp
    src/constraint/parallel.cpp
)
target_link_libraries(delta_fk
    ${catkin_LIBRARIES}
    cpp_cga
)
target_include_directories(delta_fk PRIVATE
    include
    ${cpp_cga_INCLUDE_DIRS}
)

add_executable(delta_ik
    src/delta_ik/main.cpp
    src/constraint/delta.cpp
    src/constraint/parallel.cpp
)
target_link_libraries(delta_ik
    ${catkin_LIBRARIES}
    cpp_cga
)
target_include_directories(delta_ik PRIVATE
    include
    ${cpp_cga_INCLUDE_DIRS}
)
