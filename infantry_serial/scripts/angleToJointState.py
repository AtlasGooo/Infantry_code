#!/usr/local/bin/python3.8

import random
import numpy
import rospy
from  sensor_msgs.msg import JointState
from infantry_msgs.msg import GimbalFdbAngle


def callback(msg):
    global yaw
    global pitch
    yaw = msg.yaw_angle
    pitch = msg.pitch_angle
    print("yaw, pitch: <{0},{1}>".format(yaw,pitch))
    return

 
if __name__ == '__main__':
    global yaw
    global pitch
    yaw = 0.0
    pitch = 0.0

    rospy.init_node('conveter', anonymous=True)

    rospy.Subscriber("/infantry_serial/infantry_gimbal_angle_fdb",GimbalFdbAngle,callback)
    state_pub = rospy.Publisher("/infantry_serial/gimbal_joint_state", JointState,queue_size=10)
    rate = rospy.Rate(100) # 10hz
    


    state_msg = JointState()
    state_msg.header.seq = 0
    state_msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
    state_msg.position = [yaw, pitch]
    
    
    while not rospy.is_shutdown():
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.seq += 1 
        state_msg.position[0] = yaw
        state_msg.position[1] = pitch

        state_pub.publish(state_msg)
        rate.sleep()