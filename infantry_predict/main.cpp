//
// Created by ethan on 18-5-15.
//
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>

#include "std_msgs/Header.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "infantry_msgs/ArmorPosePacket.h"
#include "infantry_msgs/ArmorPose.h"

std::mutex lock_;
std::vector<infantry_msgs::ArmorPose> targetPoses;
std_msgs::Header header;
bool poseUpdated = false;

void targetsCallback(const infantry_msgs::ArmorPosePacketConstPtr armorPoses){
    lock_.lock();
    targetPoses.assign(armorPoses->armorPoses.begin(),armorPoses->armorPoses.end());
    header = armorPoses->header;
    poseUpdated = true;
    lock_.unlock();
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"infantry_predict");
    ros::NodeHandle nh("infantry_predict_node");
    ros::Rate loop_rate(100);

    std::string camera_name = "MER139";
    
    ros::Subscriber targetsSub = nh.subscribe("/" + camera_name + "_detection/targets", 2, &targetsCallback);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::vector<infantry_msgs::ArmorPose> tempTargetPoses;
    infantry_msgs::ArmorPose tempTaget;
    infantry_msgs::ArmorPose target;


    transformStamped.header.frame_id = "MER139";
    transformStamped.child_frame_id = "target";

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        if (!poseUpdated)
        {
            continue;
        }

        lock_.lock();
        transformStamped.header.stamp = header.stamp;
        poseUpdated=false;
        tempTargetPoses.assign(targetPoses.begin(),targetPoses.end());
        lock_.unlock();


        transformStamped.header.seq ++;

        int target_num = tempTargetPoses.size();

        if (target_num==0)
        {
            continue;
        }

        
        for (size_t i = 0; i < target_num; i++)
        {
            tempTaget = tempTargetPoses.at(i);
            if (tempTaget.number==3)
            {
                target = tempTaget;
                break;
            }
        }

        transformStamped.transform.translation.x = target.pose.position.x/1000;
        transformStamped.transform.translation.y = target.pose.position.y/1000;
        transformStamped.transform.translation.z = target.pose.position.z/1000;

        transformStamped.transform.rotation.w = 1;
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;

        // ROS_INFO("message: (x:%f, x:%f, y:%f)",transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z);
        br.sendTransform(transformStamped);
        
    }
    
    ros::spin();
    return 0;
}