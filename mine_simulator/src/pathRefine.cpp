#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

using namespace std;


double cur_posi_x = 0.0;
double cur_posi_y = 0.0;
double cur_posi_z = 0.0;
double pathScale = 1.25;


double odomTime = 0.0;
double goalX = 0.0;
double goalY = 0.0;
double angle = 0.0;
double vehicleRoll = 0.0;
double vehiclePitch = 0.0;
double vehicleYaw = 0.0;

const double PI = 3.1415926;

nav_msgs::Path path;
int cnt = 0;

void goalHandler(const geometry_msgs::PointStamped::ConstPtr &goal)
{
    goalX = goal->point.x;
    goalY = goal->point.y;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    odomTime = odom->header.stamp.toSec();

    // double roll, pitch, yaw;
    // geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    // tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    // vehicleRoll = roll;
    // vehiclePitch = pitch;
    // vehicleYaw = yaw;
    cur_posi_x = odom->pose.pose.position.x;
    cur_posi_y = odom->pose.pose.position.y;
    cur_posi_z = odom->pose.pose.position.z;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "way_point");

    ros::NodeHandle nh;
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

    ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, goalHandler);

    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 5);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        float x = goalX - cur_posi_x;
        float y = goalY - cur_posi_y;

        int pathSize = 100;
        path.poses.resize(pathSize);

        // angle = atan2(y, x) * 180.0 / PI - vehicleYaw;

        // double step = pathScale / pathSize;
        // double rotDir = angle / pathSize;
        // if (angle > 5.0)
        // {
        //     for (int i = 0; i < pathSize; i++)
        //     {
        //         float rotAng = (i + 1) * rotDir;
        //         float step_l = (i + 1) * step;
        //         path.poses[i].pose.position.x = cos(rotAng) * step_l;
        //         path.poses[i].pose.position.y = sin(rotAng) * step_l;
        //         path.poses[i].pose.position.z = 0.0;
        //     }
        // }
        // else {
        for (int i = 0; i < pathSize; i++)
        {
            path.poses[i].pose.position.x = x/pathSize*(i+1);
            path.poses[i].pose.position.y = y/pathSize*(i+1);
            path.poses[i].pose.position.z = 0.0;
        }
        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";

        pubPath.publish(path);

        loop_rate.sleep();
    }
    return 0;
}