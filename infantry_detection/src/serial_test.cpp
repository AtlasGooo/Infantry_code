
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "infantry_msgs/GimbalFdbAngle.h"
#include "infantry_msgs/GimbalSetAngle.h"
#include "infantry_msgs/GimbalRate.h"
#include "infantry_msgs/EnemyColor.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
    init(argc, argv, "serial_test_publisher_node");
    NodeHandle n;
    Publisher pub = n.advertise<infantry_msgs::GimbalSetAngle>("infantry_serial/infantry_gimbal_angle_set", 1000);
    Rate loop_rate(2);
    int pub_i = 0;
    infantry_msgs::GimbalSetAngle set_angle_;

    while (ros::ok())
    {
        pub_i++;
        set_angle_.pitch_angle = pub_i;
        set_angle_.yaw_angle = pub_i;
        set_angle_.pitch_mode = 0;
        set_angle_.yaw_mode = 0;
        pub.publish(set_angle_);
        loop_rate.sleep();
        if (pub_i>100)
        {
            pub_i=0;
        }
        ROS_INFO("pub: %d",pub_i);
        
    }
    return 0;
}