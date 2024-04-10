#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
using namespace std;

std::ofstream ofs;

double pre_posi_x = 0.0;
double pre_posi_y = 0.0;
double pre_posi_z = 0.0;
double pre_yaw;
double ori_x=0.0;
double ori_y=0.0;
double ori_w=0.0;
double ori_z=0.0;

double sensorOffsetX = 0;
double sensorOffsetY = 0;

double odomTime=0.0;

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    odomTime = odom->header.stamp.toSec();
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    ofs.open("/home/zyshao/ZYSHAO/projects/Inspection/src/local_planner/src/path_record.txt", ios::app);

    pre_yaw= yaw;
    pre_posi_x = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    pre_posi_y = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    pre_posi_z = odom->pose.pose.position.z;
    ofs<<pre_posi_x<<"  "<<pre_posi_y<<"    "<<pre_yaw<<endl;
    ofs.close();
    // ori_w=odom->pose.pose.orientation.w;
    // ori_x=odom->pose.pose.orientation.x;
    // ori_y=odom->pose.pose.orientation.y;
    // ori_z=odom->pose.pose.orientation.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_recorder");

    ros::NodeHandle nh;
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);


    // ros::Rate loop_rate(10);

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::spin();

    return 0;
}