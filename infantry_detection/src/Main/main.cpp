// ros header
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

// message header
#include "infantry_msgs/ArmorPose.h"
#include "infantry_msgs/ArmorPosePacket.h"
#include "infantry_msgs/TargetPacket.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Header.h"
#include "../Armor/Armor.h"
#include "../General/General.h"
#include "../AngleSolver/AngleSolver.h"
#include <time.h>
#include <mutex>

Mat mRot2Quat(const Mat &tvec,const Mat &rmat);

ArmorDetector detector;
AngleSolver angleSolver;
std_msgs::Header matHeader;
infantry_msgs::ArmorPosePacket posePacket;
geometry_msgs::PointStamped pointPacket;
std::mutex lock_;
Mat srcImage;
bool image_received = false;

void ImageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ROS_INFO("Received \n");
    try
    {
        lock_.lock();
        matHeader.stamp = ros::Time::now();
        matHeader.seq++;
        cv_bridge::toCvShare(img_msg, "bgr8")->image.copyTo(srcImage);
        image_received = true;
        lock_.unlock();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "~");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);

    ROS_INFO("start");

    /////////////get param//////////////

    std::string camera_name;
    std::string svm_model_path;
    Mat tempMat;
    Mat camera_intrinsics;
    Mat camera_distortion;
    bool enemy_color_isred;
    Color enemy_color = RED;
    std::vector<double> camera_intrinsics_vec;
    std::vector<double> camera_distortion_vec;

    nh.getParam("camera_name", camera_name);
    nh.getParam("svm_model_path", svm_model_path);
    nh.getParam("/" + camera_name + "/gxcam_intrinsics", camera_intrinsics_vec);
    nh.getParam("/" + camera_name + "/gxcam_distortion", camera_distortion_vec);
    /////////////init pub and sub//////////////
    ros::Publisher targetPub = nh.advertise<infantry_msgs::ArmorPosePacket>("targets", 1000);
    ros::Publisher amorPointPub = nh.advertise<geometry_msgs::PointStamped>( "amorPoint", 1000);
    ros::Subscriber targetSub = nh.subscribe("/" + camera_name + "_camera/image_raw", 2, &ImageCallback);

    /////////////detection parmaeters//////////////
    camera_intrinsics = Mat(3, 3, CV_64F, &camera_intrinsics_vec[0]);
    camera_distortion = Mat(5, 1, CV_64F, &camera_distortion_vec[0]);

    std::cout << "/////////////detection parmaeters//////////////" << std::endl
              << std::endl;
    std::cout << "camera_intrinsics:" << std::endl;
    std::cout << camera_intrinsics << std::endl;

    std::cout << "camera_distortion:" << std::endl;
    std::cout << camera_distortion << std::endl;

    std::cout << "svm_model_path:" << std::endl;
    std::cout << svm_model_path << std::endl;

    detector.loadSVM(svm_model_path.c_str());
    detector.setEnemyColor(enemy_color);
    angleSolver.setCameraParam(camera_intrinsics, camera_distortion);
    angleSolver.setArmorSize(SMALL_ARMOR, 135, 125);
    angleSolver.setArmorSize(BIG_ARMOR, 230, 127);
    angleSolver.setBulletSpeed(15000);

    posePacket.header.frame_id = camera_name;
    posePacket.header.seq = 0;
    posePacket.header.stamp = ros::Time::now();
    pointPacket.header = posePacket.header;
    namedWindow("test", WINDOW_NORMAL);
    namedWindow("armor", WINDOW_NORMAL);
    while (ros::ok())
    {
        ///////////// set color ///////////////
        nh.getParam("/enemy_color_bool", enemy_color_isred);
        if (enemy_color_isred)
        {
            if (enemy_color == BLUE)
            {
                printf("enemy color changed: RED\n");
            }
            enemy_color = RED;
        }
        else
        {
            if (enemy_color == RED)
            {
                printf("enemy color changed: BLUE\n");
            }
            enemy_color = BLUE;
        }
        detector.setEnemyColor(enemy_color);

        ///////////// detect armors //////////////////////
        if (image_received)
        {
            image_received = false;
            srcImage.copyTo(tempMat);
            armorDetectingThread(tempMat);
            imshow("test", tempMat);
            if (detector.isFoundArmor())
            {
                targetPub.publish(posePacket);
                if(posePacket.armorPoses.at(0).number==3){
                    pointPacket.point = posePacket.armorPoses.at(0).pose.position;
                    pointPacket.point.x /= 1000;
                    pointPacket.point.y /= 1000;
                    pointPacket.point.z /= 1000;
                    amorPointPub.publish(pointPacket);
                }
            }
        }

        char key = waitKey(1);
        switch (key)
        {
        case 'q':
            return 0;
            break;
        case 'p':
            waitKey(0);
            break;

        default:
            break;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

void armorDetectingThread(Mat src)
{
    posePacket.header.stamp = matHeader.stamp;
    detector.run(src);

    if (detector.isFoundArmor())
    {
        printf("target found.\n");
    }
    else
    {
        return;
    }
    int armorAmount = detector.armors.size();
    posePacket.armorPoses.resize(armorAmount);

    Point2f centerPoint;
    vector<Point2f> contourPoints;
    ArmorType type;
    double x;
    double y;
    double z;
    Mat tvec;
    Mat rvec;
    Mat rmat;
    Mat quatVec;
    for (size_t i = 0; i < armorAmount; i++)
    {
        ArmorBox armor = detector.armors[i];

        line(src, armor.armorVertices[0], armor.armorVertices[2], Scalar(255, 0, 0), 5);
        line(src, armor.armorVertices[3], armor.armorVertices[1], Scalar(255, 0, 0), 5);

        angleSolver.getPos(armor.armorVertices, armor.center, armor.type, tvec, rvec);

        x = tvec.at<double>(0, 0);
        y = tvec.at<double>(1, 0);
        z = tvec.at<double>(2, 0);

        printf("x:\t%f\n", x);
        printf("y:\t%f\n", y);
        printf("z:\t%f\n", z);
        printf("q1:\t%f\n", rvec.at<double>(0, 0));
        printf("q2:\t%f\n", rvec.at<double>(1, 0));
        printf("q3:\t%f\n", rvec.at<double>(2, 0));
        printf("number:\t%d\n", int(armor.armorNum));

        posePacket.armorPoses.at(i).pose.position.x = x;
        posePacket.armorPoses.at(i).pose.position.y = y;
        posePacket.armorPoses.at(i).pose.position.z = z;

        cv::Rodrigues(rvec, rmat);
        quatVec = mRot2Quat(tvec, rmat);

        posePacket.armorPoses.at(i).pose.orientation.w = quatVec.at<double>(0, 0);
        posePacket.armorPoses.at(i).pose.orientation.x = quatVec.at<double>(1, 0);
        posePacket.armorPoses.at(i).pose.orientation.y = quatVec.at<double>(2, 0);
        posePacket.armorPoses.at(i).pose.orientation.z = quatVec.at<double>(3, 0);

        posePacket.armorPoses.at(i).number = armor.armorNum;
        ROS_INFO("ARMOR TYPE:%d", armor.type);
        imshow("armor",armor.armorImg);

        // if(armor.armorNum!=3){
        //     cv::waitKey(0);
        // }
    }
    pointPacket.header = posePacket.header;
}

inline float SIGN(float x)
{
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d)
{
    return sqrt(a * a + b * b + c * c + d * d);
}

// quaternion = [w, x, y, z]'
Mat mRot2Quat(const Mat &tvec,const Mat &rmat)
{
    
    // Mat M2 = (Mat_<double>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);
    
    // tempM = M2*tempM;

    float r11 = rmat.at<double>(0, 0);
    float r12 = rmat.at<double>(0, 1);
    float r13 = rmat.at<double>(0, 2);
    float r21 = rmat.at<double>(1, 0);
    float r22 = rmat.at<double>(1, 1);
    float r23 = rmat.at<double>(1, 2);
    float r31 = rmat.at<double>(2, 0);
    float r32 = rmat.at<double>(2, 1);
    float r33 = rmat.at<double>(2, 2);
    float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    if (q0 < 0.0f)
    {
        q0 = 0.0f;
    }
    if (q1 < 0.0f)
    {
        q1 = 0.0f;
    }
    if (q2 < 0.0f)
    {
        q2 = 0.0f;
    }
    if (q3 < 0.0f)
    {
        q3 = 0.0f;
    }
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    if (q0 >= q1 && q0 >= q2 && q0 >= q3)
    {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    }
    else if (q1 >= q0 && q1 >= q2 && q1 >= q3)
    {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    }
    else if (q2 >= q0 && q2 >= q1 && q2 >= q3)
    {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    }
    else if (q3 >= q0 && q3 >= q1 && q3 >= q2)
    {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    }
    else
    {
        printf("coding error\n");
    }
    float r = NORM(q0, q1, q2, q3);
    q0 /= r;
    q1 /= r;
    q2 /= r;
    q3 /= r;

    Mat res = (Mat_<double>(4, 1) << q0, q1, q2, q3);
    return res;
}