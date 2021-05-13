/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: Lucian <liuxin.lucain@foxmal.com>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/03/04 20:08:31 by Lucian            #+#    #+#             */
/*   Updated: 2021/03/26 01:17:46 by Lucian           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>
#include <ros/ros.h>
#include "std_msgs/Header.h"
#include "infantry_msgs/GimbalFdbAngle.h"
#include "infantry_msgs/GimbalSetAngle.h"
#include "infantry_msgs/GimbalRate.h"
#include "infantry_msgs/EnemyColor.h"

#include "LogStream.hpp"
#include "LogPacket.hpp"
#include "SerialStream.hpp"
#include "SerialPacket.hpp"
#include "RefereeStream.hpp"
#include "RefereePacket.hpp"
#include "RostopicStream.hpp"
#include "RostopicPacket.hpp"

using namespace stream;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "infantry_serial");
    ros::NodeHandle n("infantry_serial");

    ros::Rate loop_rate(10);
    infantry_msgs::GimbalRate   gimbal_rate_msg;
    infantry_msgs::EnemyColor   enemy_color_msg;
    std_msgs::Header header;

    // initialize the serial port
    serial::Serial sp_serial;
    std::string portPath = "/dev/ttyUSB0";

    sp_serial.setPort(portPath);
    sp_serial.setBaudrate(230400);
    sp_serial.setParity(serial::parity_none);
    serial::Timeout to = serial::Timeout::simpleTimeout(1);
    sp_serial.setTimeout(to);
    
    // wait until the serial port opened successfully
    while (ros::ok())
    {
        try
        {   
            sp_serial.open();
            ROS_INFO("Serial port %s opened successfully.",portPath.c_str());
            break;
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("Fail to open the serial port %s.",portPath.c_str());
            sleep(1);
        }
    }

    // define the serial stream
    SerialOutputStream serial_output_stream;
    SerialInputStream serial_input_stream;
    
    serial_input_stream.SetSerial(&sp_serial);
    serial_output_stream.SetSerial(&sp_serial); 
    
    // define the ros topic stream
    RostopicOutputStream ros_topic_output_stream(&n);
    RostopicInputStream ros_topic_input_stream(&n);
    
    // initialize gimbal_angle_set_packet
    SerialTemplatePacket<GimbalAngleSetPacket> serial_gimble_angle_set_packet;
    RostopicTemplatePacket<infantry_msgs::GimbalSetAngle,GimbalAngleSetPacket> ros_gimbal_angle_set_packet("infantry_gimbal_angle_set");
    serial_output_stream.RegisterPacket(&serial_gimble_angle_set_packet);
    ros_topic_input_stream.RegisterPacket(&ros_gimbal_angle_set_packet);

    // initialize gimbal_angle_fdb_packet
    SerialTemplatePacket<GimbalAngleFdbPacket> serial_gimbal_angle_fdb_packet;
    RostopicTemplatePacket<infantry_msgs::GimbalFdbAngle,GimbalAngleFdbPacket> ros_gimbal_angle_fdb_packet("infantry_gimbal_angle_fdb");
    serial_input_stream.RegisterPacket(&serial_gimbal_angle_fdb_packet);
    ros_topic_output_stream.RegisterPacket(&ros_gimbal_angle_fdb_packet);

    // initialize enemy_color_packet
    SerialTemplatePacket<EnemyColorPacket> serial_enemy_color_packet;
    RostopicTemplatePacket<infantry_msgs::EnemyColor,EnemyColorPacket> ros_enemy_color_packet("infantry_enemy_color");
    serial_input_stream.RegisterPacket(&serial_enemy_color_packet);
    ros_topic_output_stream.RegisterPacket(&ros_enemy_color_packet);


    
    ros_topic_input_stream.Init();
    ros_topic_output_stream.Init();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {

        serial_input_stream.Receive();
        ros_topic_output_stream << &serial_input_stream;
        
        serial_output_stream << &ros_topic_input_stream;

        if(ros_gimbal_angle_set_packet.GetUpdateFlag())
        {
            std::cout<<"rym:"<<(int)ros_gimbal_angle_set_packet.yaw_mode.data<<
                "\tpm:"<<(int)ros_gimbal_angle_set_packet.pitch_mode.data<<
                "\ty:"<<ros_gimbal_angle_set_packet.yaw_angle.data<<
                "\tp:"<<ros_gimbal_angle_set_packet.pitch_angle.data<<std::endl;
        }

        if(serial_gimble_angle_set_packet.GetUpdateFlag())
        {
            std::cout<<"sym:"<<(int)serial_gimble_angle_set_packet.yaw_mode.data<<
                "\tpm:"<<(int)serial_gimble_angle_set_packet.pitch_mode.data<<
                "\ty:"<<serial_gimble_angle_set_packet.yaw_angle.data<<
                "\tp:"<<serial_gimble_angle_set_packet.pitch_angle.data<<std::endl;
        }

        if(serial_gimbal_angle_fdb_packet.GetUpdateFlag())
        {
            // std::cout<<
            //     "y:"<<serial_gimbal_angle_fdb_packet.yaw_angle.data<<
            //     "p:"<<serial_gimbal_angle_fdb_packet.pitch_angle.data<<std::endl;
        }

        serial_output_stream.Send();
        ros_topic_output_stream.Send();

        serial_input_stream.CleanUsedPacket();
        ros_topic_input_stream.CleanUsedPacket();

        loop_rate.sleep();
    }

    sp_serial.close();
    return 0;
}