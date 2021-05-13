cmake_minimum_required(VERSION 3.0.2)
project(infantry_serial)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  infantry_msgs
)
catkin_package()

FILE(GLOB STREAM src/** )

SET(TARGET_NAME ${PROJECT_NAME}_node)

add_executable(${TARGET_NAME} main.cpp)

add_dependencies(${TARGET_NAME} infantry_msgs_gencpp)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
    ${TARGET_NAME}  
    ${catkin_LIBRARIES}
)

add_library(stream_lib ${STREAM})
target_link_libraries(stream_lib "${CMAKE_CURRENT_SOURCE_DIR}/lib/libserial.so")

target_link_libraries(${TARGET_NAME} stream_lib)

catkin_install_python(PROGRAMS scripts/angleToJointState.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)