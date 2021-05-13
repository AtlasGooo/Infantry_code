#!/usr/local/bin/python3.8

import random
import numpy
import rospy
from  std_msgs.msg import Float32
 
class pub_gauss():
    def __init__(self,means,sigma):
        rospy.init_node('pub_gauss', anonymous=False)
        pub = rospy.Publisher('gauss_noise', Float32, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
 
        while not rospy.is_shutdown():
 
            noise = random.gauss(means, sigma)
            rospy.loginfo(noise)
            pub.publish(noise)
            rate.sleep()
 
if __name__ == '__main__':
    try:
        pub_gauss(0,0.8)
    except rospy.ROSInterruptException:
        pass