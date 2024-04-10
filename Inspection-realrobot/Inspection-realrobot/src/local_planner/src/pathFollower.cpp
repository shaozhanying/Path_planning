#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/duration.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "kmd_can_msgs/front_angle_fb.h"
#include "kmd_can_msgs/rear_angle_fb.h"

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.1;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.01;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

bool turnning_mode = false;

bool closeObstacle = false;
bool disObstacle = false;
double closeObstacleLength = 0.5;
int closeObstacleNum = 10;
float shortestDisThreshold = 0.4;

nav_msgs::Path path;

float front_angle_fb_l = 0, front_angle_fb_r = 0;
float rear_angle_fb_l = 0, rear_angle_fb_r = 0;

int wayPointStatus = 3;

// 将车辆的里程计的信息转换到车辆坐标系下(坐标系的转换)
void front_angle_fb_Handler(kmd_can_msgs::front_angle_fb front_angle_fb_msgs)
{
  front_angle_fb_l = front_angle_fb_msgs.front_angle_fb_l;
  front_angle_fb_r = front_angle_fb_msgs.front_angle_fb_r;
  // std::cout<<"front_angle_fb_l "<<front_angle_fb_l-front_angle_fb_r<<std::endl;
  // if(abs(front_angle_fb_l-front_angle_fb_r)>60||abs(rear_angle_fb_l-rear_angle_fb_r)>60)
  //     std::cout<<"turnning mode"<<std::endl;
  // else
  //     std::cout<<"normal mode"<<std::endl;
}

void rear_angle_fb_Handler(kmd_can_msgs::rear_angle_fb rear_angle_fb_msgs)
{
  rear_angle_fb_l = rear_angle_fb_msgs.rear_angle_fb_l;
  rear_angle_fb_r = rear_angle_fb_msgs.rear_angle_fb_r;
  // std::cout<<"rear_angle_fb_l "<<rear_angle_fb_l<<" rear_angle_fb_r "<<rear_angle_fb_r<<std::endl;
}
void wayPointStatusHandler(const std_msgs::Int8 wayPointStatus_)
{
  wayPointStatus = wayPointStatus_.data;
  // std::cout<<"wayPointStatus_ "<<wayPointStatus_<<std::endl;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop)
  {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow)
  {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr &pathIn)
{
  int pathSize = pathIn->poses.size();
  // std::cout<<"pathSize"<<pathSize<<std::endl;
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++)
  {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    path.poses[i].pose.orientation.w = pathIn->poses[i].pose.orientation.w;
    path.poses[i].pose.orientation.x = pathIn->poses[i].pose.orientation.x;
    path.poses[i].pose.orientation.y = pathIn->poses[i].pose.orientation.y;
    path.poses[i].pose.orientation.z = pathIn->poses[i].pose.orientation.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

// 操纵杆处理
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0)
    joySpeed = 1.0;
  if (joy->axes[4] == 0)
    joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop)
    joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive)
  {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1)
  {
    autonomyMode = false;
  }
  else
  {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0)
  {
    joySpeed = speed->data / maxSpeed;
    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }
}

void aveLengthHandler(const std_msgs::Float32::ConstPtr &aveLength)
{
  // std::cout<<"aveLength:"<<aveLength->data<<std::endl;
  // if(aveLength->data < closeObstacleLength) {
  //   closeObstacle  = true;
  //   std::cout<<"aveLength:"<<aveLength->data<<" "<<closeObstacleLength<<std::endl;
  // } else closeObstacle  = false;
}

void shortDisNumHandler(const std_msgs::Float32::ConstPtr &shortDisNum)
{
  if (shortDisNum->data > closeObstacleNum)
  {
    closeObstacle = true;
  }
  else
    closeObstacle = false;
}
void shortestDisHandler(const std_msgs::Float32::ConstPtr &shortestDis)
{
  if (shortestDis->data <= shortestDisThreshold)
  {
    disObstacle = true;
  }
  else
    disObstacle = false;
}

void stopHandler(const std_msgs::Int8::ConstPtr &stop)
{
  safetyStop = stop->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  ros::Subscriber subnWayPointStatus = nh.subscribe<std_msgs::Int8>("/wayPointStatus", 5, wayPointStatusHandler);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/path", 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

  ros::Subscriber subAveLength = nh.subscribe<std_msgs::Float32>("/aveLength", 5, aveLengthHandler);

  ros::Subscriber subShortDisNum = nh.subscribe<std_msgs::Float32>("/shortDisNum", 5, shortDisNumHandler);

  ros::Subscriber subShortestDis = nh.subscribe<std_msgs::Float32>("/shortestDis", 5, shortestDisHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);

  // ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 5);
  // geometry_msgs::TwistStamped cmd_vel;
  // cmd_vel.header.frame_id = "vehicle";
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

  ros::Subscriber subfront_angle_fb = nh.subscribe<kmd_can_msgs::front_angle_fb>("/front_angle_fb", 5, front_angle_fb_Handler);
  ros::Subscriber subrear_angle_fb = nh.subscribe<kmd_can_msgs::rear_angle_fb>("/rear_angle_fb", 5, rear_angle_fb_Handler);

  ros::Publisher pubGoalAccessible = nh.advertise<std_msgs::Bool>("/GoalAccessible", 1);

  geometry_msgs::Twist cmd_vel;
  bool debugout = true;
  if (autonomyMode)
  {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }
  bool stoprotation = false;

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    if (pathInit)
    {
      // vehicleXRec = vehicleX ？
      // vehicleYawRec = vehicleYaw
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      // 0点到目标点的相对距离
      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;

      float endDisWYaw = path.poses[pathSize - 1].pose.orientation.w;
      float endDisXYaw = path.poses[pathSize - 1].pose.orientation.x;
      float endDisYYaw = path.poses[pathSize - 1].pose.orientation.y;
      float endDisZYaw = path.poses[pathSize - 1].pose.orientation.z;
      float endDisYaw = atan2(2 * (endDisWYaw * endDisZYaw), 1 - 2 * endDisZYaw * endDisZYaw);
      float yawSlide = 0.01;

      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);
      if (path.poses[pathSize - 1].pose.position.x == 0 && path.poses[pathSize - 1].pose.position.y == 0)
      {
        // if (vehicleSpeed > 0)
        //   vehicleSpeed -= maxAccel / 100.0;
        // else if (vehicleSpeed < 0)
        //   vehicleSpeed += maxAccel / 100.0;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        if (debugout)
          std::cout << "stop" << std::endl;
        ros::Duration(1).sleep();
        pubSpeed.publish(cmd_vel);
      }
      else
      {
        if (endDisX < 0)
        {
          endDis = -endDis;
        }

        // 找到目标点
        float disX, disY, dis;
        while (pathPointID < pathSize - 1)
        {
          disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
          disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
          dis = sqrt(disX * disX + disY * disY);

          if (dis < lookAheadDis)
          {
            pathPointID++;
          }
          else
          {
            break;
          }
        }

        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        float pathDir = atan2(disY, disX);
        float dirDiff, dirDiffend;
        dirDiffend = vehicleYaw - vehicleYawRec - endDisYaw;
        dirDiff = vehicleYaw - vehicleYawRec - pathDir;
        if (dirDiff > PI)
          dirDiff -= 2 * PI;
        else if (dirDiff < -PI)
          dirDiff += 2 * PI;
        if (dirDiff > PI)
          dirDiff -= 2 * PI;
        else if (dirDiff < -PI)
          dirDiff += 2 * PI;
        // 根据轮子方向，判断是不是原地旋转的模式
        if (!turnning_mode && (front_angle_fb_l * front_angle_fb_r < 0 || rear_angle_fb_l * rear_angle_fb_r < 0))
        {
          turnning_mode = true;
        }

        if (turnning_mode && abs(front_angle_fb_l - front_angle_fb_r) < 1 && abs(rear_angle_fb_l - rear_angle_fb_r) < 1)
        {
          turnning_mode = false;
        }
        // 如果离目标点比较近，直接靠近目标点，并调整朝向
        if ((abs(endDis) <= 0.4) && endDisXYaw == 0)
        {
          // 离目标点不够进，先靠近目标点
          if ((!stoprotation && abs(endDis) > 0.1)) //|| (stoprotation && abs(endDis) > 0.2)
          {

            float joySpeed3;
            if (true) // 只能朝前走
            {
              double time = ros::Time::now().toSec();
              if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre)
              {
                navFwd = false;
                switchTime = time;
              }
              else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre)
              {
                navFwd = true;
                switchTime = time;
              }
            }

            if (!navFwd)
            {
              dirDiff += PI;
              if (dirDiff > PI)
                dirDiff -= 2 * PI;
            }
            joySpeed3 = endDis / slowDwnDisThre;
            vehicleYawRate = -stopYawRateGain * dirDiff;

            if (fabs(dirDiff) < dirDiffThre) // 小车朝向目标点，靠近目标点
            {
              if (!turnning_mode && abs(front_angle_fb_l - front_angle_fb_r) < 1 && abs(rear_angle_fb_l - rear_angle_fb_r) < 1)
              {
                if (vehicleSpeed < joySpeed3)
                  vehicleSpeed += maxAccel / 100.0;
                else if (vehicleSpeed > joySpeed3)
                  vehicleSpeed -= maxAccel / 100.0;
                if (vehicleYawRate > maxYawRate * PI / 180.0)
                  vehicleYawRate = maxYawRate * PI / 180.0;
                else if (vehicleYawRate < -maxYawRate * PI / 180.0)
                  vehicleYawRate = -maxYawRate * PI / 180.0;
                if (debugout)
                  std::cout << "waypoint mode 1 fabs(dirDiff) < dirDiffThre";
              }
              else
              {
                if (debugout)
                  std::cout << "waypoint mode 1 fabs(dirDiff) < dirDiffThre but turning ";

                vehicleYawRate = 0;
              }
            }
            else // 小车未朝向目标点，优先对准目标点
            {
              if (vehicleSpeed > 0)
                vehicleSpeed -= maxAccel / 100.0;
              else if (vehicleSpeed < 0)
                vehicleSpeed += maxAccel / 100.0;
              if (vehicleYawRate > maxYawRate * PI / 180.0)
                vehicleYawRate = maxYawRate * PI / 180.0;
              else if (vehicleYawRate < -maxYawRate * PI / 180.0)
                vehicleYawRate = -maxYawRate * PI / 180.0;

              if (debugout)
                std::cout << "waypoint mode 3 fabs(dirDiff)>1 ";
            }
            stoprotation = false;
          }
          else // 到达目标点后，旋转调整朝向
          {
            vehicleYawRate = -stopYawRateGain * dirDiffend;
            if (vehicleYawRate > maxYawRate * PI / 180.0)
              vehicleYawRate = maxYawRate * PI / 180.0;
            else if (vehicleYawRate < -maxYawRate * PI / 180.0)
              vehicleYawRate = -maxYawRate * PI / 180.0;

            vehicleSpeed = 0;
            stoprotation = true;
            if (debugout)
            {
              std::cout << "waypoint mode 4 abs(endDis) < 0.1 \n";
            }
            // if (abs(dirDiffend) < M_PI * 5 / 180)
            // {
            //   std_msgs::Int8 unableToGoal_;
            //   unableToGoal_.data = true;
            //   pubGoalAccessible.publish(unableToGoal_);
            //   if (debugout)
            //   {
            //     std::cout << "skep waypoint for mode 4 !!!\n";
            //   }
            // }
          }
          if (debugout)
            std::cout << "dis " << dis << " endDis " << endDis << "dirDiff " << dirDiff / M_PI * 180 << " pathDir " << pathDir / M_PI * 180 << " dirDiffend " << dirDiffend << " endDisYaw " << endDisYaw << " stoprotation " << stoprotation << " vehicleYawRate " << vehicleYawRate << " vehicleSpeed " << vehicleSpeed << std::endl;
        }
        else // 未靠近目标点的一般情况下
        {
          stoprotation = false;
          float joySpeed2 = maxSpeed * joySpeed;
          if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
            vehicleYawRate = -stopYawRateGain * dirDiff;
          else
            vehicleYawRate = -yawRateGain * dirDiff;

          if (vehicleYawRate > maxYawRate * PI / 180.0)
            vehicleYawRate = maxYawRate * PI / 180.0;
          else if (vehicleYawRate < -maxYawRate * PI / 180.0)
            vehicleYawRate = -maxYawRate * PI / 180.0;
          if (joySpeed2 == 0 && !autonomyMode)
          {
            vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
          }
          else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal))
          {
            vehicleYawRate = 0;
          }

          if (pathSize <= 1)
          {
            joySpeed2 = 0;
          }
          else if (abs(endDis) / slowDwnDisThre < joySpeed)
          {
            joySpeed2 *= abs(endDis) / slowDwnDisThre;
          }

          float joySpeed3 = joySpeed2;
          // std::cout<<"joySpeed3 "<<joySpeed3<<std::endl;
          if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0)
            joySpeed3 *= slowRate1;
          else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0)
            joySpeed3 *= slowRate2;

          if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) // 小车朝向目标点
          {
            if (!turnning_mode && abs(front_angle_fb_l - front_angle_fb_r) < 1 && abs(rear_angle_fb_l - rear_angle_fb_r) < 1)
            {
              if (vehicleSpeed < joySpeed3)
                vehicleSpeed += maxAccel / 100.0;
              else if (vehicleSpeed > joySpeed3)
                vehicleSpeed -= maxAccel / 100.0;
              if (debugout)
                std::cout << "normal mode 1 "
                          << dirDiff << " dis " << dis <<" speed "<<vehicleSpeed<< std::endl;
            }
            else
            {
              vehicleYawRate = 0;
              if (debugout)
                std::cout << "normal mode 1 0 "
                          << dirDiff << " dis " << dis <<" speed "<<vehicleSpeed<< std::endl;
            }
          }
          else if (fabs(dirDiff) < 1 && dis > stopDisThre) // 小车朝向与目标点有一点角度差，线速度减慢
          {
            if (!turnning_mode && abs(front_angle_fb_l - front_angle_fb_r) < 10 && abs(rear_angle_fb_l - rear_angle_fb_r) < 10)
            {
              if (vehicleSpeed > joySpeed3 / 2)
                vehicleSpeed -= maxAccel / 100.0;
              else if (vehicleSpeed < joySpeed3 / 2)
                vehicleSpeed += maxAccel / 100.0;
              if (debugout)
                std::cout << "normal mode 2 " << dirDiff << " dis " << dis <<" speed "<<vehicleSpeed<< std::endl;
            }
            else
            {
              vehicleYawRate = 0;
              if (debugout)
                std::cout << "normal mode 2 0 " << dirDiff << " dis " << dis <<" speed "<<vehicleSpeed<< std::endl;
            }
          }
          else ////小车朝向与目标点角度差大，优先转向。
          {
            if (vehicleSpeed > 0)
              vehicleSpeed -= maxAccel / 100.0;
            else if (vehicleSpeed < 0)
              vehicleSpeed += maxAccel / 100.0;
            if (debugout)
              std::cout << "normal mode 3 " << dirDiff << " dis " << dis << std::endl;
          }

          if (odomTime < stopInitTime + stopTime && stopInitTime > 0)
          {
            if (debugout)
              std::cout << "stopInitTime" << std::endl;
            vehicleSpeed = 0;

            vehicleYawRate = 0;
          }
        }

        if (safetyStop >= 1)
          vehicleSpeed = 0;
        if (safetyStop >= 2)
          vehicleYawRate = 0;

        pubSkipCount--;
        if (pubSkipCount < 0)
        {
          if (fabs(vehicleSpeed) <= maxAccel / 100.0)
            cmd_vel.linear.x = 0;
          else
            cmd_vel.linear.x = vehicleSpeed;
          cmd_vel.angular.z = vehicleYawRate;
          if (closeObstacle || disObstacle || wayPointStatus == 2) // 如果近距离障碍物多，速度减慢
          {
            cmd_vel.linear.x = cmd_vel.linear.x / 3;
          }
          pubSpeed.publish(cmd_vel);

          pubSkipCount = pubSkipNum;
        }
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}