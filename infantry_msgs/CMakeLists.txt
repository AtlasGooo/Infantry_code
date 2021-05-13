cmake_minimum_required(VERSION 3.0.2)
project(infantry_msgs)

add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
)

add_message_files(
    FILES
    EnemyColor.msg
    GimbalRate.msg
    GimbalSetAngle.msg
    GimbalFdbAngle.msg
    TargetPacket.msg
    ArmorPose.msg
    ArmorPosePacket.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)
