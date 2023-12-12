#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>

using namespace std;

const int N = 110;
int up_bound=11;

int goal_;

double wp[12][2] = {
    {0.0,0.0},
    {0.5099904537200928,13.247758865356445},
    {9.878669738769531,13.026167869567871},
    {10.127654075622559,16.738157272338867},
    {39.65837860107422,16.818004608154297},
    {41.65726089477539,13.974727630615234},
    {45.020076751708984,15.820451736450195},
    {45.360347747802734,29.08767318725586},
    {40.69820785522461,31.10810089111328},
    {40.44618606567383,29.557926177978516},
    {39.650203704833984,23.33303451538086},
    {0.046563148498535156,23.351301193237305},
    // {1.493622064590454,4.445984840393066},
    // {1.2386059761047363,6.8757405281066895},
};

double pre_posi_x = 0.0;
double pre_posi_y = 0.0;
double pre_posi_z = 0.0;
int cnt = 0;

void update_state()
{
    float dis=sqrt(pow(pre_posi_x-wp[goal_][1],2)+pow(pre_posi_y-wp[goal_][0],2));
    if (cnt == 0)
    {
        if (dis<1.0)
            goal_++;
        if (goal_ > up_bound)
        {
            cnt = 1;
            goal_ = up_bound - 1;
        }
    }
    else if (cnt == 1)
    {
        if (dis<1.0)
            goal_--;
        if (goal_ < 0)
            goal_ = 1, cnt = 0;
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    pre_posi_x = odom->pose.pose.position.x;
    pre_posi_y = odom->pose.pose.position.y;
    pre_posi_z = odom->pose.pose.position.z;

    update_state();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "way_point");

    ros::NodeHandle nh;
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

    ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);

    ros::Rate loop_rate(10);
    geometry_msgs::PointStamped waypoint;


    while (ros::ok())
    {
        ros::spinOnce();

        waypoint.point.z = pre_posi_z;
        waypoint.header.frame_id="map";
        waypoint.point.x = wp[goal_][1];
        waypoint.point.y = wp[goal_][0];

        pubWaypoint.publish(waypoint);
        ROS_INFO("%d    %d",cnt,goal_);
        loop_rate.sleep();
    }
    return 0;
}