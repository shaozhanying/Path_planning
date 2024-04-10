#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <math.h>
#include <nav_msgs/Path.h>

#define WAYPOINT_NUM 15
using namespace std;

ifstream ifs;
int goal_ = 0;
// int if_goal_changed = 0;
// int pre_goal = -1;
std::vector<std::vector<std::array<double, 3>>> total_path; // WAYPOINT_NUM
double wp[WAYPOINT_NUM][5] = { // x, y, yaw, navigation mode(0 obs avoid; 1 follow path; 2 narrow path ), waypoint check mode (0 relaxed; 1 strict)
    {4.0,   0.1,    0,   1, 1},
    {10.0,  0.1,    0,   2, 0},
    {11.0,  0.15,   0,   1, 1},
    {16.0,  0.15,   0,   2, 0},
    {23.5,  0,      90,  1, 0},
    {23.9,  22.35,  45,  1, 1},
    {24.4,  22.95,  135, 2, 1},
    {23.8,  23.65,  90,  2, 0},
    {23.5,  40,     180, 1, 0},
    {16.5,  40,     -90, 1, 0},
    {16.5,  10.5,   180, 1, 0},
    {12,    10.5,   -90, 1, 1},
    {11,    0.1,    180, 0, 1},
    {4.0,   0.1,    180, 2, 0},
    {0.0,   0.0,    0,   1, 0}};

double pre_posi_x = 0.0;
double pre_posi_y = 0.0;
double pre_posi_z = 0.0;
double pre_yaw = 0.0;

void update_state()
{
    float dis = sqrt(pow(pre_posi_x - wp[goal_][0], 2) + pow(pre_posi_y - wp[goal_][1], 2));
    float yawdif = abs(pre_yaw - (M_PI * wp[goal_][2] / 180));
    if (yawdif > M_PI)
        yawdif -= 2 * M_PI;
    else if (yawdif < -M_PI)
        yawdif += 2 * M_PI;
    if (yawdif > M_PI)
        yawdif -= 2 * M_PI;
    else if (yawdif < -M_PI)
        yawdif += 2 * M_PI;
    // std::cout<<dis<<" "<< yawdif/M_PI*180<<"  wp[goal_][2] "<< wp[goal_][2]<<std::endl;
    // if(narrow[goal_]==true)
    if (wp[goal_][4] == 1 )
    {
        if (dis < 0.1 && abs(yawdif) < M_PI * 2 / 180)
            goal_ = (goal_ + 1) % WAYPOINT_NUM;
    }
    else
    {
        if (dis < 0.4)
            goal_ = (goal_ + 1) % WAYPOINT_NUM;
    }
    // std::cout << goal_ << " is next goal !\n";

    // if (pre_goal != goal_)
    // {
    //     pre_goal = goal_;
    //     if_goal_changed = 1;
    // std::cout<<"if_goal_changed "<< if_goal_changed<<std::endl;
    // }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    pre_posi_x = odom->pose.pose.position.x;
    pre_posi_y = odom->pose.pose.position.y;
    pre_posi_z = odom->pose.pose.position.z;
    // atan2(2(q0q3+q1q2),1−2(q22+q23))
    pre_yaw = atan2(2 * (odom->pose.pose.orientation.w * odom->pose.pose.orientation.z), 1 - 2 * odom->pose.pose.orientation.z * odom->pose.pose.orientation.z); // atan2(2*(odom->pose.pose.orientation.w*odom->pose.pose.orientation.z,0),1);
    update_state();
}

void goalAccessibleHandler(const std_msgs::Bool unableToGoal)
{
    if (unableToGoal.data)
    {
        goal_ = (goal_ + 1) % WAYPOINT_NUM;
        std::cout << goal_ << " is next goal !\n";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspection_waypoints");

    ros::NodeHandle nh;
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

    // ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
    ros::Publisher pubgoal = nh.advertise<geometry_msgs::PoseStamped>("/goal", 5);
    // ros::Publisher pubNarrowStatus = nh.advertise<std_msgs::Bool>("/narrowStatus", 5);
    ros::Publisher pubWayPointStatus = nh.advertise<std_msgs::Int8>("/wayPointStatus", 5);
    ros::Publisher pubFixPath = nh.advertise<nav_msgs::Path>("/fixed_path", 5);

    ros::Subscriber subGoalAccessible = nh.subscribe<std_msgs::Bool>("/GoalAccessible", 5, goalAccessibleHandler);

    ros::Rate loop_rate(10);
    geometry_msgs::PointStamped waypoint;
    geometry_msgs::PoseStamped goalpose;
    nav_msgs::Path path;
    goal_ = 0;
    std::string file_src = ros::package::getPath("inspection_waypoints");
    ifs.open(file_src + "/src/path_record.txt");
    if (!ifs)
    {
        ROS_ERROR("无法打开路径文件");
        return -1;
    }
    double path_x, path_y, predict_yaw;
    std::vector<std::array<double, 3>> separate_path;

    while (ifs >> path_x >> path_y >> predict_yaw)
    {
        if (path_x <= -10000)
        {
            total_path.emplace_back(separate_path);
            separate_path.clear();
            continue;
        }
        separate_path.emplace_back(std::array<double, 3>{path_x, path_y, predict_yaw});
    }

    while (ros::ok())
    {
        ros::spinOnce();

        goalpose.header.frame_id = "map";
        goalpose.pose.position.x = wp[goal_][0];
        goalpose.pose.position.y = wp[goal_][1];
        goalpose.pose.position.z = pre_posi_z;
        double yaw = M_PI * wp[goal_][2] / 180;
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(0 * 0.5);
        double sp = sin(0 * 0.5);
        double cr = cos(0 * 0.5);
        double sr = sin(0 * 0.5);
        goalpose.pose.orientation.w = cy * cp * cr + sy * sp * sr;
        goalpose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
        goalpose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
        goalpose.pose.orientation.z = sy * cp * cr - cy * sp * sr;
        pubgoal.publish(goalpose);

        {
            int path_size = total_path[goal_].size();

            path.poses.resize(path_size + 1);
            for (int i = 0; i < path_size; i++)
            {
                path.poses[i].pose.position.x = total_path[goal_][i][0];
                path.poses[i].pose.position.y = total_path[goal_][i][1];
                path.poses[i].pose.position.z = total_path[goal_][i][2];
            }
            path.poses[path_size].pose.position.x = wp[goal_][0];
            path.poses[path_size].pose.position.y = wp[goal_][1];
            path.poses[path_size].pose.position.z = pre_posi_z;
            pubFixPath.publish(path);
        }
        std_msgs::Int8 wayPointStatus_;
        wayPointStatus_.data = wp[goal_][3];
        pubWayPointStatus.publish(wayPointStatus_);
        loop_rate.sleep();
    }
    return 0;
}