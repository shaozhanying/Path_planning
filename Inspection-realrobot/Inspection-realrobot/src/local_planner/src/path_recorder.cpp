#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
using namespace std;

std::ofstream ofs;

double pre_posi_x = 0.0;
double pre_posi_y = 0.0;
double pre_posi_z = 0.0;
double pre_yaw;
double ori_x = 0.0;
double ori_y = 0.0;
double ori_w = 0.0;
double ori_z = 0.0;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
double odomTime = 0.0;

bool spaceKeyPressed = false;

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    odomTime = odom->header.stamp.toSec();
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    std::string file_src = ros::package::getPath("inspection_waypoints");
    ofs.open(file_src + "/src/path_record.txt", ios::app);

    pre_yaw = yaw;
    pre_posi_x = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    pre_posi_y = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    pre_posi_z = odom->pose.pose.position.z;
    ofs << pre_posi_x << "  " << pre_posi_y << "    " << pre_yaw << std::endl;
    ofs.close();
    // ori_w=odom->pose.pose.orientation.w;
    // ori_x=odom->pose.pose.orientation.x;
    // ori_y=odom->pose.pose.orientation.y;
    // ori_z=odom->pose.pose.orientation.z;
}

void setNonBlockingTerminal()
{
    int stdin_fileno = fileno(stdin);
    struct termios term;
    struct termios term_orig;
    tcgetattr(stdin_fileno, &term);
    term_orig = term;
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(stdin_fileno, TCSANOW, &term);
}

void terminalInputCallback()
{
    // 使用select函数检查终端输入是否有数据
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fileno(stdin), &read_fds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    int ready = select(fileno(stdin) + 1, &read_fds, NULL, NULL, &timeout);

    if (ready > 0)
    {
        char c;
        int nn = read(fileno(stdin), &c, 1);

        if (c == ' ')
        {
            spaceKeyPressed = true;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_recorder");

    ros::NodeHandle nh;
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);
    ros::Timer terminalInputTimer = nh.createTimer(ros::Duration(0.01), [](const ros::TimerEvent &)
                                                   { terminalInputCallback(); });
    setNonBlockingTerminal();
    while (ros::ok())
    {
        if (spaceKeyPressed)
        {
            std::string file_src = ros::package::getPath("inspection_waypoints");
            ofs.open(file_src + "/src/path_record.txt", ios::app);
            ofs << "-10010 -10010 -10010" << std::endl;
            ofs.close();
            spaceKeyPressed = false; // 重置空格键标志位
        }

        ros::spinOnce();
    }

    return 0;
}