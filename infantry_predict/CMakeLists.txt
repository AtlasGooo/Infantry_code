cmake_minimum_required(VERSION 3.0.2)
project(infantry_predict)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2
  std_msgs
  infantry_msgs
)
catkin_package()

FILE(GLOB STREAM src/** )

SET(TARGET_NAME ${PROJECT_NAME}_node)


add_executable(${TARGET_NAME} main.cpp)

add_dependencies(${TARGET_NAME} infantry_msgs_gencpp)

include_directories( "/usr/include/eigen3" )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
    ${TARGET_NAME}  
    ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS noise.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)