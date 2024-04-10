#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
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

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1

struct Point_
{
  float x, y;
};
double path_x = 0.0;
double path_y = 0.0;
double PPath[10010][3];
int cnt = 0; // 用来记录上一次离自己最近的点是哪个，方便找到路径中当前离自己最近的
int sum = 0;
int temp_goalI = 0;
Point_ A{0.3, 0.3};
Point_ B{0.3, -0.3};
Point_ C{-0.3, -0.3};
Point_ D{-0.3, 0.3};
bool if_fixed_path = true;
bool change_avoid_obs = false;
std::vector<Eigen::Vector3d> cur_path;
bool change_temp_goal = true;

string pathFolder;
double vehicleLength = 0.7;
double vehicleWidth = 0.7;
double realvehicleLength = 0.6;
double realvehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalClearRange = 0.5;
double lastgoalX = 0;
double lastgoalY = 0;
double goalX = 0;
double goalY = 0;
double goalYaw = 0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

// bool narrowStatus=false;
int wayPointStatus = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

float stuckX = 0;
float stuckY = 0;
double timeInterval5 = 0;
double timeInterval10 = -1;

// void narrowStatusHandler(const std_msgs::Bool narrowStatus_)
// {
//   narrowStatus = narrowStatus_.data;
//   // std::cout<<"narrowStatus "<<narrowStatus<<std::endl;
// }
void wayPointStatusHandler(const std_msgs::Int8 wayPointStatus_)
{
  wayPointStatus = wayPointStatus_.data;
  // std::cout<<"wayPointStatus_ "<<wayPointStatus_<<std::endl;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
  if (!useTerrainAnalysis)
  {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++)
    {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange)
      {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloud2)
{
  if (useTerrainAnalysis)
  {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++)
    {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost))
      {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0)
    joySpeed = 1.0;
  if (joy->axes[4] == 0)
    joySpeed = 0;

  if (joySpeed > 0)
  {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0)
      joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !twoWayDrive)
    joySpeed = 0;

  if (joy->axes[2] > -0.1)
  {
    autonomyMode = false;
  }
  else
  {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1)
  {
    checkObstacle = true;
  }
  else
  {
    checkObstacle = false;
  }
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr &goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalYaw = -4;
  if (goalX != lastgoalX)
  {
    std::cout << "new waypoint" << std::endl;
    lastgoalX = goalX;
    change_avoid_obs = false;
    if_fixed_path = true;
    change_temp_goal = true;
  }
}
void goalposeHandler(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  goalYaw = atan2(2 * (goal->pose.orientation.w * goal->pose.orientation.z), 1 - 2 * goal->pose.orientation.z * goal->pose.orientation.z);
  // std::cout<<"goalYaw "<<goalYaw<<std::endl;
  if (goalX != lastgoalX || goalY != lastgoalY)
  {
    std::cout << "new waypoint" << std::endl;
    timeInterval10 = -1;
    lastgoalX = goalX;
    lastgoalY = goalY;
    change_avoid_obs = false;
    if_fixed_path = true;
    change_temp_goal = true;
    temp_goalI = 0;
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

void pathHandler(const nav_msgs::Path::ConstPtr &pathIn)
{
  int pathSize = pathIn->poses.size();

  for (int i = 0; i < pathSize; i++)
  {
    PPath[i][0] = pathIn->poses[i].pose.position.x;
    PPath[i][1] = pathIn->poses[i].pose.position.y;
    PPath[i][2] = pathIn->poses[i].pose.position.z;
  }
  sum = pathSize;
  // std::cout << "pathSize " << pathSize << std::endl;
}

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr &boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1)
  {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++)
  {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z)
    {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++)
      {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++)
        {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr &addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++)
  {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr &checkObs)
{
  double checkObsTime = ros::Time::now().toSec();

  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay)
  {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header")
  {
    val = fscanf(filePtr, "%s", str);
    if (val != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element")
    {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1)
      {
        printf("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL)
  {
    printf("\nCannot read startPaths files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++)
  {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    if (groupID >= 0 && groupID < groupNum)
    {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL)
  {
    printf("\nCannot read paths files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++)
  {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    if (pathID >= 0 && pathID < pathNum)
    {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum)
      {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL)
  {
    printf("\nCannot read pathList files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr))
  {
    printf("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++)
  {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum)
    {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL)
  {
    printf("\nCannot read correspondences files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++)
  {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    while (1)
    {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1)
      {
        printf("\nError reading input files, exit.\n\n");
        exit(1);
      }

      if (pathID != -1)
      {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum)
        {
          correspondences[gridVoxelID].push_back(pathID);
        }
      }
      else
      {
        break;
      }
    }
  }

  fclose(filePtr);
}

double dot(const Point_ &a, const Point_ &b)
{
  return a.x * b.x + a.y * b.y;
}

Point_ vectorBetween(const Point_ &from, const Point_ &to)
{
  return Point_{to.x - from.x, to.y - from.y};
}

bool isPointInRectangle(const Point_ &A, const Point_ &B, const Point_ &C, const Point_ &D, const Point_ &P)
{
  Point_ AB = vectorBetween(A, B);
  Point_ BC = vectorBetween(B, C);
  Point_ CD = vectorBetween(C, D);
  Point_ DA = vectorBetween(D, A);
  Point_ AP = vectorBetween(A, P);
  Point_ BP = vectorBetween(B, P);
  Point_ CP = vectorBetween(C, P);
  Point_ DP = vectorBetween(D, P);

  if (dot(AP, AB) < 0)
    return 0;
  if (dot(BP, BC) < 0)
    return 0;
  if (dot(CP, CD) < 0)
    return 0;
  if (dot(DP, DA) < 0)
    return 0;

  return 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", realvehicleLength);
  nhPrivate.getParam("vehicleWidth", realvehicleWidth);
  vehicleLength = realvehicleLength + 0.1;
  vehicleWidth = realvehicleWidth + 0.1;
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("costHeightThre", costHeightThre);
  nhPrivate.getParam("costScore", costScore);
  nhPrivate.getParam("useCost", useCost);
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("dirWeight", dirWeight);
  nhPrivate.getParam("dirThre", dirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("minPathScale", minPathScale);
  nhPrivate.getParam("pathScaleStep", pathScaleStep);
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
  nhPrivate.getParam("minPathRange", minPathRange);
  nhPrivate.getParam("pathRangeStep", pathRangeStep);
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nhPrivate.getParam("goalClearRange", goalClearRange);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);

  // ros::Subscriber subnarrowStatus = nh.subscribe<std_msgs::Bool>("/narrowStatus", 5, narrowStatusHandler);
  ros::Subscriber subnWayPointStatus = nh.subscribe<std_msgs::Int8>("/wayPointStatus", 5, wayPointStatusHandler);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, terrainCloudHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, goalHandler);
  ros::Subscriber subGoalpoint = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 5, goalposeHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped>("/navigation_boundary", 5, boundaryHandler);

  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2>("/added_obstacles", 5, addedObstaclesHandler);

  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool>("/check_obstacle", 5, checkObstacleHandler);

  ros::Subscriber subFixedPath = nh.subscribe<nav_msgs::Path>("fixed_path", 5, pathHandler);

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 5);

  ros::Publisher pubGoalAccessible = nh.advertise<std_msgs::Bool>("/GoalAccessible", 5);

  ros::Publisher pubAveLength = nh.advertise<std_msgs::Float32>("/aveLength", 5);

  ros::Publisher pubShortDisNum = nh.advertise<std_msgs::Float32>("/shortDisNum", 5);

  ros::Publisher pubShortestDis = nh.advertise<std_msgs::Float32>("/shortestDis", 5);

  nav_msgs::Path path;

#if PLOTPATHSET == 1
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2>("/free_paths", 2);
#endif

  // ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stacked_scans", 2);

  printf("\nReading path files.\n");

  if (autonomyMode)
  {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++)
  {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++)
  {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
#if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++)
  {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
#endif
  for (int i = 0; i < gridVoxelNum; i++)
  {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  readStartPaths();
#if PLOTPATHSET == 1
  readPaths();
#endif
  readPathList();
  readCorrespondences();

  printf("\nInitialization complete.\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    if (newLaserCloud || newTerrainCloud)
    {
      if (newLaserCloud)
      {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++)
        {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud)
      {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++)
      {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis))
        {
          plannerCloudCrop->push_back(point);
        }
      }

      int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++)
      {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange)
        {
          plannerCloudCrop->push_back(point);
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++)
      {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange)
        {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;
      if (pathRangeBySpeed)
        pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange)
        pathRange = minPathRange;
      float relativeGoalDis = adjacentRange;
      float relativeGoalX;
      float relativeGoalY;
      if (autonomyMode)
      {
        relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        if (!twoWayDrive)
        {
          if (joyDir > 90.0)
            joyDir = 90.0;
          else if (joyDir < -90.0)
            joyDir = -90.0;
        }
      }

      bool pathFound = false;
      float defPathScale = pathScale;
      if (pathScaleBySpeed)
        pathScale = defPathScale * joySpeed;
      if (pathScale < minPathScale)
        pathScale = minPathScale;
      bool useDWA = true;
      freePaths->clear();

      float aveLen = 2.0;
      float shortestDis = 10.0;
      float sumD = 0;
      int sumBoundaryNum = 0;
      int shortDisNum = 0;
      for (int i = 0; i < plannerCloudCrop->points.size(); i++)
      {
        float x = plannerCloudCrop->points[i].x;
        float y = plannerCloudCrop->points[i].y;
        float obsDis = sqrt(x * x + y * y);
        sumD += obsDis;
        sumBoundaryNum++;
        if (obsDis < 0.5)
          shortDisNum++;
        if (obsDis < shortestDis)
          shortestDis = obsDis;
      }
      aveLen = sumD / sumBoundaryNum;
      std_msgs::Float32 aveLenMsg;
      std_msgs::Float32 shortDisNumMsg;
      std_msgs::Float32 shortestDisMsg;
      aveLenMsg.data = aveLen;
      shortDisNumMsg.data = shortDisNum;
      shortestDisMsg.data = shortestDis;
      pubShortDisNum.publish(shortDisNumMsg);
      pubAveLength.publish(aveLenMsg);
      pubShortestDis.publish(shortestDisMsg);

      // if (narrowStatus == true)
      if (wayPointStatus == 2)
      {
        float minDis = 1.5;
        // printf("relativeGoalDis:%f\n", relativeGoalDis);
        float targetX = 0;
        float targetY = 0;
        float slideVehicleLen = 0.05;

        float slide = 0.03;

        float kGoal = relativeGoalY / relativeGoalX;
        float disGoal = sqrt(relativeGoalY * relativeGoalY + relativeGoalX * relativeGoalX);
        float cosGoal = relativeGoalX / disGoal;
        float sinGoal = relativeGoalY / disGoal;
        float searchRange = sqrt(vehicleLength * vehicleLength + vehicleWidth * vehicleWidth) / 2;
        std::set<std::pair<float, float>> leftboundarySet;
        std::set<std::pair<float, float>> rightboundarySet;
        std::set<std::pair<float, float>> fitBoundarySet;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++)
        {
          float x = plannerCloudCrop->points[i].x;
          float y = plannerCloudCrop->points[i].y;
          float z = plannerCloudCrop->points[i].z;
          float h = plannerCloudCrop->points[i].intensity;
          pair<float, float> curBoundary{float(((int)((x) * 100)) / 100.0), float(((int)((y) * 100)) / 100.0)};

          float x_to_goal = x * cosGoal + y * sinGoal;
          float y_to_goal = -x * sinGoal + y * cosGoal;

          if (h > obstacleHeightThre && z > -0.65 && abs(y_to_goal) < searchRange)
          {
            if ((relativeGoalX > 0 && x_to_goal > -searchRange && x_to_goal < disGoal + searchRange))
            {
              if (y_to_goal <= 0)
                leftboundarySet.insert(curBoundary);
              else
                rightboundarySet.insert(curBoundary);
            }
            else if ((relativeGoalX < 0 && x_to_goal < searchRange && x_to_goal > -disGoal - searchRange))
            {
              if (y_to_goal <= 0)
                rightboundarySet.insert(curBoundary);
              else
                leftboundarySet.insert(curBoundary);
            }
          }
        }

        int pathSize = int(disGoal * 100);

        // vector<Eigen::Vector3d> ps;
        // for (int i = 0; i < pathSize; i++)
        // {
        //   Eigen::Vector3d p;
        //   p.x = targetX / pathSize * (i + 1);
        //   p.y = targetY / pathSize * (i + 1);
        //   p.z = 0.0;
        //   ps.push_back(p);
        // }

        for (pair<float, float> boundary1 : leftboundarySet)
        {
          for (pair<float, float> boundary2 : rightboundarySet)
          {
            if (boundary1.first != boundary2.first)
            {
              float kBoundary = (boundary1.second - boundary2.second) / (boundary1.first - boundary2.first);

              float delta1 = boundary1.second - kGoal * boundary1.first;
              float delta2 = boundary2.second - kGoal * boundary2.first;

              if (delta1 * delta2 < 0 && kBoundary * kGoal >= -1 - slide && kBoundary * kGoal <= -1 + slide && sqrt((boundary1.first - boundary2.first) * (boundary1.first - boundary2.first) + (boundary1.second - boundary2.second) * (boundary1.second - boundary2.second)) < vehicleWidth + 0.4)
              {
                pair<float, float> fixPoint{float(((int)((boundary1.first + boundary2.first) * 50)) / 100.0), float(((int)((boundary1.second + boundary2.second) * 50)) / 100.0)};
                fitBoundarySet.insert(fixPoint);
              }
            }
            else if (boundary1.first == boundary2.first && abs(kGoal) < 0.03)
            {
              float delta1 = boundary1.second - kGoal * boundary1.first;
              float delta2 = boundary2.second - kGoal * boundary2.first;
              if (delta1 * delta2 < 0 && sqrt((boundary1.first - boundary2.first) * (boundary1.first - boundary2.first) + (boundary1.second - boundary2.second) * (boundary1.second - boundary2.second)) < vehicleWidth + 0.2)
              {
                pair<float, float> fixPoint{float(((int)((boundary1.first + boundary2.first) * 50)) / 100.0), float(((int)((boundary1.second + boundary2.second) * 50)) / 100.0)};
                fitBoundarySet.insert(fixPoint);
              }
            }
          }
        }
        // for (pair<float, float> boundary1 : leftboundarySet)
        // {
        //       point.x=boundary1.first;
        //       point.y=boundary1.second;
        //       freePaths->push_back(point);
        // }
        // for (pair<float, float> boundary1 : rightboundarySet)
        // {
        //       point.x=boundary1.first;
        //       point.y=boundary1.second;
        //       freePaths->push_back(point);
        // }

        //   sensor_msgs::PointCloud2 freePaths2;
        //   pcl::toROSMsg(*freePaths, freePaths2);
        //   freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        //   freePaths2.header.frame_id = "vehicle";
        //   pubFreePaths.publish(freePaths2);
        // printf("plannerCloudCropSize:%d boundarySet:%d fitBoundarySet:%d\n", plannerCloudCropSize, boundarySet.size(), fitBoundarySet.size());
        float advanceLen = 0.8;
        float advanceK = 0;
        float advanceB;
        if (!fitBoundarySet.empty())
        {
          float minDis = 1000000;

          for (pair<float, float> fixPoint : fitBoundarySet)
          {

            float curDistance = sqrt(fixPoint.first * fixPoint.first + fixPoint.second * fixPoint.second);
            // point.x=fixPoint.first;
            // point.y=fixPoint.second;
            // freePaths->push_back(point);
            if (minDis > curDistance)
            {
              minDis = curDistance;
              targetX = fixPoint.first;
              targetY = fixPoint.second;
            }
          }
          // sensor_msgs::PointCloud2 freePaths2;
          // pcl::toROSMsg(*freePaths, freePaths2);
          // freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          // freePaths2.header.frame_id = "vehicle";
          // pubFreePaths.publish(freePaths2);
          advanceK = (relativeGoalY - targetY) / (relativeGoalX - targetX);
          advanceB = relativeGoalY - advanceK * relativeGoalX;
          if (disGoal > advanceLen)
          {
            if (relativeGoalX >= 0)
              targetX = advanceLen;
            else
              targetX = -advanceLen;

            targetY = advanceK * targetX + advanceB;
          }
          else
          {
            targetX = relativeGoalX;
            targetY = relativeGoalY;
          }
        }
        else
        {
          targetX = relativeGoalX;
          targetY = relativeGoalY;
          // printf("\n???\n");
        }
        point.x = targetX;
        point.y = targetY;
        freePaths->push_back(point);
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths.publish(freePaths2);
        // printf("targetX:%f tagertY:%f relativeGoalX:%f relativaGoalY:%f\n", targetX, targetY, relativeGoalX, relativeGoalY);
        path.poses.resize(pathSize);

        for (int i = 0; i < pathSize; i++)
        {
          path.poses[i].pose.position.x = targetX / pathSize * (i + 1);
          path.poses[i].pose.position.y = targetY / pathSize * (i + 1);
          path.poses[i].pose.position.z = 0.0;
        }
        double pathyaw;
        for (int i = 0; i < path.poses.size(); i++)
        {
          if (i == path.poses.size() - 1 && sqrt((path.poses[i].pose.position.x - relativeGoalX) * (path.poses[i].pose.position.x - relativeGoalX) + (path.poses[i].pose.position.y - relativeGoalY) * (path.poses[i].pose.position.y - relativeGoalY)) < 0.1 && goalYaw > -4)
          {
            pathyaw = goalYaw - vehicleYaw;
            path.poses[i].pose.orientation.w = cos(pathyaw / 2);
            path.poses[i].pose.orientation.x = 0;
            path.poses[i].pose.orientation.y = 0;
            path.poses[i].pose.orientation.z = sin(pathyaw / 2);
          }
          else
          {
            path.poses[i].pose.orientation.w = 0.707;
            path.poses[i].pose.orientation.x = 0.707;
            path.poses[i].pose.orientation.y = 0;
            path.poses[i].pose.orientation.z = 0;
          }
        }

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";

        pubPath.publish(path);

        // 判断是否可达
        if (sqrt(targetX * targetX + targetY * targetY) > 0.1)
        {
          bool isReach = true;
          float dismax = realvehicleLength / 2 + sqrt(targetX * targetX + targetY * targetY);
          float dismin = realvehicleLength / 2;
          for (std::pair<float, float> possibleObs : leftboundarySet)
          {

            if (advanceK != 0)
            {

              float curDis = (possibleObs.first - advanceK * possibleObs.second) / sqrt(1 + advanceK * advanceK);
              float centraiDis = abs(possibleObs.second - advanceK * possibleObs.first - advanceB) / sqrt(1 + advanceK * advanceK);
              if (curDis < dismax && curDis > dismin && centraiDis < 0.5 * realvehicleWidth)
              {
                point.x = possibleObs.first;
                point.y = possibleObs.second;
                freePaths->push_back(point);
                isReach = false;

                // break;
                // std::cout << 0.5 * realvehicleWidth << " " << centraiDis << std::endl;
              }
            }
          }
          for (std::pair<float, float> possibleObs : rightboundarySet)
          {
            if (advanceK != 0)
            {
              float curDis = (possibleObs.first - advanceK * possibleObs.second) / sqrt(1 + advanceK * advanceK);
              float centraiDis = abs(possibleObs.second - advanceK * possibleObs.first - advanceB) / sqrt(1 + advanceK * advanceK);
              if (curDis < dismax && curDis > dismin && centraiDis < 0.5 * realvehicleWidth)
              {
                point.x = possibleObs.first;
                point.y = possibleObs.second;
                freePaths->push_back(point);
                isReach = false;
                // break;
                // std::cout << 0.5 * realvehicleWidth << " " << centraiDis << std::endl;
              }
            }
          }

          sensor_msgs::PointCloud2 freePaths3;
          pcl::toROSMsg(*freePaths, freePaths3);
          freePaths3.header.stamp = ros::Time().fromSec(odomTime);
          freePaths3.header.frame_id = "vehicle";
          pubFreePaths.publish(freePaths3);

          if (!isReach)
          {
            std::cout << "unreachable!" << std::endl;
            path.poses.resize(1);
            path.poses[0].pose.position.x = 0;
            path.poses[0].pose.position.y = 0;
            path.poses[0].pose.position.z = 0;

            path.header.stamp = ros::Time().fromSec(odomTime);
            path.header.frame_id = "vehicle";
            pubPath.publish(path);
            ros::Rate rateac(0.5);
            rateac.sleep();
          }
        }
      }
      else
      {

        if (ros::Time::now().toSec() - timeInterval5 > 5)
        {
          timeInterval5 = ros::Time::now().toSec();
          std::cout << "5s pass " << timeInterval5 << " " << stuckX << " " << stuckY << " " << relativeGoalDis << "\n";
          if ((vehicleX - stuckX) * (vehicleX - stuckX) + (vehicleY - stuckY) * (vehicleY - stuckY) <= vehicleLength * vehicleLength && relativeGoalDis > vehicleLength)
          {
            std::cout << "zzzzzzzzz\n";
            path.poses.resize(1);
            path.poses[0].pose.position.x = 0;
            path.poses[0].pose.position.y = 0;
            path.poses[0].pose.position.z = 0;

            path.header.stamp = ros::Time().fromSec(odomTime);
            path.header.frame_id = "vehicle";
            pubPath.publish(path);
            ros::Duration(5).sleep();
            timeInterval5 += 10;
            // ros::Rate rateac(0.2);
            // rateac.sleep();
          }
          stuckX = vehicleX;
          stuckY = vehicleY;
        }
        bool realWaypoint = true;
        // std::cout << "if_fixed_path " << if_fixed_path << std::endl;
        if (if_fixed_path && wayPointStatus != 0)
        {
          path.poses.clear();
          double dis = 100;
          for (int i = 0; i < sum; i++)
          {
            double pre = sqrt((PPath[i][0] - vehicleX) * (PPath[i][0] - vehicleX) + (PPath[i][1] - vehicleY) * ((PPath[i][1] - vehicleY)));
            if (dis > pre)
            {
              dis = pre;
              cnt = i;
            }
          }
          while (cnt < sum - 1)
          {
            dis = sqrt((PPath[cnt][0] - vehicleX) * (PPath[cnt][0] - vehicleX) + (PPath[cnt][1] - vehicleY) * ((PPath[cnt][1] - vehicleY)));
            if (dis < 0.2)
            {
              cnt++;
            }
            else
            {
              break;
            }
          }
          float sinVehicleYaw = sin(vehicleYaw);
          float cosVehicleYaw = cos(vehicleYaw);
          // std::cout << "sum " << sum << " cnt " << cnt << std::endl;
          point.x = PPath[cnt][0];
          point.y = PPath[cnt][1];
          freePaths->push_back(point);
          sensor_msgs::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          freePaths2.header.frame_id = "map";
          pubFreePaths.publish(freePaths2);
          // ros::Rate rateac(0.5);
          // rateac.sleep();
          double predict_yaw;
          float pointX1 = 0, pointY1 = 0, pointZ1 = 0;
          int j = 0;
          cur_path.clear();
          if (dis > 0.3)
          {
            int pathSize = int(dis * 100);
            pointX1 = PPath[cnt][0] - vehicleX;
            pointY1 = PPath[cnt][1] - vehicleY;
            pointZ1 = vehicleZ;
            double relx = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
            double rely = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
            for (int i = 0; i < pathSize; i++)
            {

              Eigen::Vector3d choose_point;
              choose_point.x() = relx / pathSize * (i + 1);
              choose_point.y() = rely / pathSize * (i + 1);
              choose_point.z() = PPath[i][2];
              cur_path.push_back(choose_point);
            }
          }

          for (int i = cnt; i < sum; i++)
          {
            if (abs(pointX1 * pointX1 + pointY1 * pointY1) > 3)
            {
              // std::cout << "end " << i << std::endl;

              realWaypoint = false;
              break;
            }
            if (abs((pointX1 - (PPath[i][0] - vehicleX)) * (pointX1 - (PPath[i][0] - vehicleX)) + (pointY1 - (PPath[i][1] - vehicleY)) * (pointY1 - (PPath[i][1] - vehicleY))) > 0.01)
            {
              pointX1 = PPath[i][0] - vehicleX;
              pointY1 = PPath[i][1] - vehicleY;
              pointZ1 = vehicleZ;
              Eigen::Vector3d choose_point;
              choose_point.x() = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
              choose_point.y() = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
              choose_point.z() = PPath[i][2];
              cur_path.push_back(choose_point);
            }
            else
            {
              if (i == sum - 1)
              {
                pointX1 = PPath[i][0] - vehicleX;
                pointY1 = PPath[i][1] - vehicleY;
                pointZ1 = vehicleZ;
                Eigen::Vector3d choose_point;
                choose_point.x() = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                choose_point.y() = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                choose_point.z() = PPath[i][2];
                cur_path.push_back(choose_point);
                realWaypoint = true;
              }
            }
          }
          // predict if_obstacle in the path
          for (Eigen::Vector3d pathpoint : cur_path)
          {
            predict_yaw = pathpoint.z() - vehicleYaw;
            A.x = pathpoint.x() + 0.5 * vehicleLength * cos(predict_yaw) + 0.5 * vehicleWidth * sin(predict_yaw);
            A.y = pathpoint.y() - 0.5 * vehicleLength * sin(predict_yaw) + 0.5 * vehicleWidth * cos(predict_yaw);
            B.x = pathpoint.x() + 0.5 * vehicleLength * cos(predict_yaw) - 0.5 * vehicleWidth * sin(predict_yaw);
            B.y = pathpoint.y() - 0.5 * vehicleLength * sin(predict_yaw) - 0.5 * vehicleWidth * cos(predict_yaw);
            C.x = pathpoint.x() - 0.5 * vehicleLength * cos(predict_yaw) - 0.5 * vehicleWidth * sin(predict_yaw);
            C.y = pathpoint.y() + 0.5 * vehicleLength * sin(predict_yaw) - 0.5 * vehicleWidth * cos(predict_yaw);
            D.x = pathpoint.x() - 0.5 * vehicleLength * cos(predict_yaw) + 0.5 * vehicleWidth * sin(predict_yaw);
            D.y = pathpoint.y() + 0.5 * vehicleLength * sin(predict_yaw) + 0.5 * vehicleWidth * cos(predict_yaw);

            int plannerCloudCropSize = plannerCloudCrop->points.size();
            for (int i = 0; i < plannerCloudCropSize; i++)
            {
              float x = plannerCloudCrop->points[i].x;
              float y = plannerCloudCrop->points[i].y;
              float z = plannerCloudCrop->points[i].z;
              float h = plannerCloudCrop->points[i].intensity;
              if (h > obstacleHeightThre && z > -0.65 && isPointInRectangle(A, B, C, D, Point_({x, y})))
              {
                std::cout << "change_avoid_obs " << change_avoid_obs << " " << x << " " << y << std::endl;
                change_avoid_obs = true;
                if_fixed_path = false;
                // point.x = x;
                // point.y = y;
                // point.z = -1;
                // freePaths->push_back(point);
                // sensor_msgs::PointCloud2 freePaths2;
                // pcl::toROSMsg(*freePaths, freePaths2);
                // freePaths2.header.stamp = ros::Time().fromSec(odomTime);
                // freePaths2.header.frame_id = "vehicle";
                // pubFreePaths.publish(freePaths2);
                // ros::Rate rateac(0.5);
                // rateac.sleep();
                break;
              }
            }
            if (change_avoid_obs)
            {
              break;
            }
            // else
            // {
            //   // std::cout << "change_avoid_obs " << change_avoid_obs << std::endl;
            // }
          }
          if (!change_avoid_obs)
          {
            path.poses.resize(cur_path.size());
            for (int j = 0; j < cur_path.size(); j++)
            {
              path.poses[j].pose.position.x = cur_path[j][0];
              path.poses[j].pose.position.y = cur_path[j][1];
              path.poses[j].pose.position.z = vehicleZ;
            }
            // // add yaw
            double pathyaw;
            for (int i = 0; i < path.poses.size(); i++)
            {
              if (i == path.poses.size() - 1 && sqrt((path.poses[i].pose.position.x - relativeGoalX) * (path.poses[i].pose.position.x - relativeGoalX) + (path.poses[i].pose.position.y - relativeGoalY) * (path.poses[i].pose.position.y - relativeGoalY)) < 0.1 && goalYaw > -4 && realWaypoint)
              {
                pathyaw = goalYaw - vehicleYaw;
                path.poses[i].pose.orientation.w = cos(pathyaw / 2);
                path.poses[i].pose.orientation.x = 0;
                path.poses[i].pose.orientation.y = 0;
                path.poses[i].pose.orientation.z = sin(pathyaw / 2);
              }
              else
              {
                path.poses[i].pose.orientation.w = 0.707;
                path.poses[i].pose.orientation.x = 0.707;
                path.poses[i].pose.orientation.y = 0;
                path.poses[i].pose.orientation.z = 0;
              }
            }
            path.header.stamp = ros::Time().fromSec(odomTime);
            path.header.frame_id = "vehicle";
            // std::cout << "path.poses.size " << path.poses.size() << std::endl;

            pubPath.publish(path);
          }
        }
        if (change_avoid_obs || wayPointStatus == 0)
        {

          if (!(sqrt((goalX - vehicleX) * (goalX - vehicleX) + (goalY - vehicleY) * (goalY - vehicleY) < 2.0)) && wayPointStatus != 0)
          {

            if (change_temp_goal) // 改变 far
            {
              //  double temp_goalX,temp_goalY;
              double dis = 100;
              for (int i = 0; i < sum; i++)
              {
                double pre = sqrt((PPath[i][0] - vehicleX) * (PPath[i][0] - vehicleX) + (PPath[i][1] - vehicleY) * ((PPath[i][1] - vehicleY)));
                if (dis > pre)
                {
                  dis = pre;
                  cnt = i;
                }
              }
              for (int i = cnt; i < sum; i++)
              {
                // relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
                // relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);
                double dis = sqrt((PPath[i][0] - vehicleX) * (PPath[i][0] - vehicleX) + (PPath[i][1] - vehicleY) * (PPath[i][1] - vehicleY));
                if (dis * pathScale > 2.0)
                {
                  relativeGoalX = ((PPath[i][0] - vehicleX) * cosVehicleYaw + (PPath[i][1] - vehicleY) * sinVehicleYaw);
                  relativeGoalY = (-(PPath[i][0] - vehicleX) * sinVehicleYaw + (PPath[i][1] - vehicleY) * cosVehicleYaw);
                  relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
                  joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

                  if (!twoWayDrive)
                  {
                    if (joyDir > 90.0)
                      joyDir = 90.0;
                    else if (joyDir < -90.0)
                      joyDir = -90.0;
                  }
                  temp_goalI = i;
                  // std::cout << "0 relativeGoalX " << PPath[i][0] << " relativeGoalY " << PPath[i][1] << std::endl;

                  break;
                }
              }
            }
            else /// 不改变
            {

              relativeGoalX = ((PPath[temp_goalI][0] - vehicleX) * cosVehicleYaw + (PPath[temp_goalI][1] - vehicleY) * sinVehicleYaw);
              relativeGoalY = (-(PPath[temp_goalI][0] - vehicleX) * sinVehicleYaw + (PPath[temp_goalI][1] - vehicleY) * cosVehicleYaw);
              relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
              joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

              if (!twoWayDrive)
              {
                if (joyDir > 90.0)
                  joyDir = 90.0;
                else if (joyDir < -90.0)
                  joyDir = -90.0;
              }
              // std::cout << "1 relativeGoalX " << PPath[temp_goalI][0] << " relativeGoalY " << PPath[temp_goalI][1] << std::endl;
            }
            realWaypoint = false;
          }
          else
          {
            realWaypoint = true;
            // std::cout << "2 relativeGoalX " << goalX << " relativeGoalY " << goalY << std::endl;
          }

          // std::cout << "relativeGoalX " << relativeGoalX << " relativeGoalY " << relativeGoalY << std::endl; /// 不改变
          point.x = relativeGoalX;
          point.y = relativeGoalY;
          point.z = 0.1;
          point.intensity = 100;
          freePaths->push_back(point);
          // sensor_msgs::PointCloud2 freePaths2;
          // pcl::toROSMsg(*freePaths, freePaths2);
          // freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          // freePaths2.header.frame_id = "vehicle";
          // pubFreePaths.publish(freePaths2);
          while (pathScale >= minPathScale && pathRange >= minPathRange)
          {
            for (int i = 0; i < 36 * pathNum; i++)
            {
              clearPathList[i] = 0;
              pathPenaltyList[i] = 0;
            }
            for (int i = 0; i < 36 * groupNum; i++)
            {
              clearPathPerGroupScore[i] = 0;
            }
            float minObsAngCW = -180.0;
            float minObsAngCCW = 180.0;
            float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
            float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
            int plannerCloudCropSize = plannerCloudCrop->points.size();
            for (int i = 0; i < plannerCloudCropSize; i++)
            {
              float x = plannerCloudCrop->points[i].x / pathScale;
              float y = plannerCloudCrop->points[i].y / pathScale;
              float z = plannerCloudCrop->points[i].z;
              float h = plannerCloudCrop->points[i].intensity;
              float dis = sqrt(x * x + y * y);
              if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle)
              {
                for (int rotDir = 0; rotDir < 36; rotDir++)
                {
                  float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                  float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                  if (angDiff > 180.0)
                  {
                    angDiff = 360.0 - angDiff;
                  }
                  if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                      ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle))
                  {
                    continue;
                  }
                  float x2 = cos(rotAng) * x + sin(rotAng) * y;
                  float y2 = -sin(rotAng) * x + cos(rotAng) * y;
                  float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;
                  int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
                  int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
                  if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY)
                  {
                    int ind = gridVoxelNumY * indX + indY;
                    int blockedPathByVoxelNum = correspondences[ind].size();
                    for (int j = 0; j < blockedPathByVoxelNum; j++)
                    {
                      if (h > obstacleHeightThre && z > -0.65 || !useTerrainAnalysis)
                      {
                        clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                      }
                      else
                      {
                        if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre)
                        {
                          pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                        }
                      }
                    }
                  }
                }
              }
              if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) &&
                  (h > obstacleHeightThre && z > -0.65 || !useTerrainAnalysis) && checkRotObstacle)
              {
                float angObs = atan2(y, x) * 180.0 / PI;
                if (angObs > 0)
                {
                  if (minObsAngCCW > angObs - angOffset)
                    minObsAngCCW = angObs - angOffset;
                  if (minObsAngCW < angObs + angOffset - 180.0)
                    minObsAngCW = angObs + angOffset - 180.0;
                }
                else
                {
                  if (minObsAngCW < angObs + angOffset)
                    minObsAngCW = angObs + angOffset;
                  if (minObsAngCCW > 180.0 + angObs - angOffset)
                    minObsAngCCW = 180.0 + angObs - angOffset;
                }
              }
            }
            if (minObsAngCW > 0)
              minObsAngCW = 0;
            if (minObsAngCCW < 0)
              minObsAngCCW = 0;
            for (int i = 0; i < 36 * pathNum; i++)
            {
              int rotDir = int(i / pathNum);
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0)
              {
                angDiff = 360.0 - angDiff;
              }
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle))
              {
                continue;
              }
              if (clearPathList[i] < pointPerPathThre)
              {
                float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
                if (penaltyScore < costScore)
                  penaltyScore = costScore;
                float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
                if (dirDiff > 360.0)
                {
                  dirDiff -= 360.0;
                }
                if (dirDiff > 180.0)
                {
                  dirDiff = 360.0 - dirDiff;
                }
                float rotDirW;
                if (rotDir < 18)
                  rotDirW = fabs(fabs(rotDir - 9) + 1);
                else
                  rotDirW = fabs(fabs(rotDir - 27) + 1);
                float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
                if (score > 0)
                {
                  clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
                }
              }
            }
            float maxScore = 0;
            int selectedGroupID = -1;
            for (int i = 0; i < 36 * groupNum; i++)
            {
              int rotDir = int(i / groupNum);
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float rotDeg = 10.0 * rotDir;
              if (rotDeg > 180.0)
                rotDeg -= 360.0;
              if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
                                                           (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle))
              {
                maxScore = clearPathPerGroupScore[i];
                selectedGroupID = i;
              }
            }
            if (selectedGroupID >= 0)
            {
              int rotDir = int(selectedGroupID / groupNum);
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              selectedGroupID = selectedGroupID % groupNum;
              int selectedPathLength = startPaths[selectedGroupID]->points.size();
              path.poses.resize(selectedPathLength);
              for (int i = 0; i < selectedPathLength; i++)
              {
                float x = startPaths[selectedGroupID]->points[i].x;
                float y = startPaths[selectedGroupID]->points[i].y;
                float z = startPaths[selectedGroupID]->points[i].z;
                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale)
                {
                  path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  path.poses[i].pose.position.z = pathScale * z;
                }
                else
                {
                  path.poses.resize(i);
                  break;
                }
              }
              // add yaw
              double pathyaw;
              for (int i = 0; i < path.poses.size(); i++)
              {
                if (i == path.poses.size() - 1 && sqrt((path.poses[i].pose.position.x - relativeGoalX) * (path.poses[i].pose.position.x - relativeGoalX) + (path.poses[i].pose.position.y - relativeGoalY) * (path.poses[i].pose.position.y - relativeGoalY)) < 0.1 && goalYaw > -4 && realWaypoint)
                {
                  pathyaw = goalYaw - vehicleYaw;
                  path.poses[i].pose.orientation.w = cos(pathyaw / 2);
                  path.poses[i].pose.orientation.x = 0;
                  path.poses[i].pose.orientation.y = 0;
                  path.poses[i].pose.orientation.z = sin(pathyaw / 2);
                }
                else
                {
                  path.poses[i].pose.orientation.w = 0.707;
                  path.poses[i].pose.orientation.x = 0.707;
                  path.poses[i].pose.orientation.y = 0;
                  path.poses[i].pose.orientation.z = 0;
                }
              }
              path.header.stamp = ros::Time().fromSec(odomTime);
              path.header.frame_id = "vehicle";
              pubPath.publish(path);
#if PLOTPATHSET == 1
              // freePaths->clear();
              for (int i = 0; i < 36 * pathNum; i++)
              {
                int rotDir = int(i / pathNum);
                float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                float rotDeg = 10.0 * rotDir;
                if (rotDeg > 180.0)
                  rotDeg -= 360.0;
                float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                if (angDiff > 180.0)
                {
                  angDiff = 360.0 - angDiff;
                }
                if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                    ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) ||
                    !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
                      (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle))
                {
                  continue;
                }
                if (clearPathList[i] < pointPerPathThre)
                {
                  int freePathLength = paths[i % pathNum]->points.size();
                  // std::cout << "pathScale " << pathScale << "pathRange" << pathRange << std::endl;

                  for (int j = 0; j < freePathLength; j++)
                  {
                    point = paths[i % pathNum]->points[j];
                    float x = point.x;
                    float y = point.y;
                    float z = point.z;
                    float dis = sqrt(x * x + y * y);
                    if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal))
                    {

                      point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                      point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                      point.z = pathScale * z;
                      point.intensity = 1.0;

                      {
                        double dis = sqrt((point.x - relativeGoalX) * (point.x - relativeGoalX) + (point.y - relativeGoalY) * (point.y - relativeGoalY));
                        if (dis < 0.05)
                        {
                          change_temp_goal = false;
                        }
                      }
                      freePaths->push_back(point);
                    }
                  }
                }
              }
              sensor_msgs::PointCloud2 freePaths2;
              pcl::toROSMsg(*freePaths, freePaths2);
              freePaths2.header.stamp = ros::Time().fromSec(odomTime);
              freePaths2.header.frame_id = "vehicle";
              pubFreePaths.publish(freePaths2);
#endif
            }

            if (sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY) < realvehicleLength/2)
            {
              change_avoid_obs = false;
              if_fixed_path = true;
              change_temp_goal = true;
            }
            if (selectedGroupID < 0)
            {
              if (pathScale >= minPathScale + pathScaleStep)
              {
                pathScale -= pathScaleStep;
                pathRange = adjacentRange * pathScale / defPathScale;
              }
              else
              {
                pathRange -= pathRangeStep;
              }
            }
            else
            {
              pathFound = true;
              break;
            }
          }

          pathScale = defPathScale;

          if (!pathFound)
          {
            path.poses.resize(1);
            path.poses[0].pose.position.x = 0;
            path.poses[0].pose.position.y = 0;
            path.poses[0].pose.position.z = 0;

            path.header.stamp = ros::Time().fromSec(odomTime);
            path.header.frame_id = "vehicle";
            pubPath.publish(path);

#if PLOTPATHSET == 1
            freePaths->clear();
            sensor_msgs::PointCloud2 freePaths2;
            pcl::toROSMsg(*freePaths, freePaths2);
            freePaths2.header.stamp = ros::Time().fromSec(odomTime);
            freePaths2.header.frame_id = "vehicle";
            pubFreePaths.publish(freePaths2);
#endif
          }
        }
      }
      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
      int plannerCloudCropSize = plannerCloudCrop->points.size();
      bool stopflag = false;
      freePaths->clear();
      bool needYawConstraint = (path.poses[path.poses.size() - 1].pose.orientation.x == 0);
      float collisionRadius = sqrt(vehicleLength * vehicleLength + vehicleWidth * vehicleWidth) / 2;
      float runYaw = atan2(goalY - vehicleY, goalX - vehicleX);
      float yawdif1 = goalYaw - runYaw;
      float yawdif2 = runYaw - vehicleYaw;
      float yawdif3 = goalYaw - vehicleYaw;
      if (yawdif1 > PI)
        yawdif1 -= 2 * PI;
      else if (yawdif1 < -PI)
        yawdif1 += 2 * PI;
      if (yawdif1 > PI)
        yawdif1 -= 2 * PI;
      else if (yawdif1 < -PI)
        yawdif1 += 2 * PI;
      if (yawdif2 > PI)
        yawdif2 -= 2 * PI;
      else if (yawdif2 < -PI)
        yawdif2 += 2 * PI;
      if (yawdif2 > PI)
        yawdif2 -= 2 * PI;
      else if (yawdif2 < -PI)
        yawdif2 += 2 * PI;
      if (yawdif3 > PI)
        yawdif3 -= 2 * PI;
      else if (yawdif3 < -PI)
        yawdif3 += 2 * PI;
      if (yawdif3 > PI)
        yawdif3 -= 2 * PI;
      else if (yawdif3 < -PI)
        yawdif3 += 2 * PI;
      float x2;
      float y2;
      float x3;
      float y3;
      float yaw;
      for (int i = 0; i < plannerCloudCropSize; i++)
      {
        float x = plannerCloudCrop->points[i].x;
        float y = plannerCloudCrop->points[i].y;
        float z = plannerCloudCrop->points[i].z;
        float h = plannerCloudCrop->points[i].intensity;
        float obs2goalDis = sqrt((x - relativeGoalX) * (x - relativeGoalX) + (y - relativeGoalY) * (y - relativeGoalY));
        double usedvehicleWidth = wayPointStatus == 2 ? realvehicleWidth : vehicleWidth;
        double usedvehicleLength = wayPointStatus == 2 ? realvehicleLength : vehicleLength;
        // if (needYawConstraint)
        {

          // if (relativeGoalDis < 0.1)
          if ((wayPointStatus == 2&&relativeGoalDis < 0.1)||(wayPointStatus != 2&&relativeGoalDis < 1))
          {
            if (yawdif3 < 0)
            {
              for (yaw = vehicleYaw; yaw > goalYaw; yaw = yaw - 10 * PI / 180)
              {
                x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
                y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
                if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
                {
                  stopflag = true;
                  break;
                }
              }
              yaw = goalYaw;
              x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
              y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
              x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
              y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
              if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
              {
                stopflag = true;
              }
              if (stopflag)
              {
                ROS_INFO("warnning 3<!!!!!! %f %f %f %f %f %f %f\n", x, y, x2, y2, x3, y3, (yaw - vehicleYaw) / PI * 180);

                point.x = x;
                point.y = y;
                point.z = z;
                freePaths->push_back(point);
                break;
              }
            }
            else
            {
              for (yaw = vehicleYaw; yaw < goalYaw; yaw = yaw + 10 * PI / 180)
              {
                x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
                y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
                if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
                {
                  stopflag = true;
                  break;
                }
              }
              yaw = goalYaw;
              x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
              y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
              x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
              y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
              if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
              {
                stopflag = true;
              }
              if (stopflag)
              {
                ROS_INFO("warnning 3>!!!!!! %f %f %f %f %f %f %f\n", x, y, x2, y2, x3, y3, (yaw - vehicleYaw) / PI * 180);

                point.x = x;
                point.y = y;
                point.z = z;
                freePaths->push_back(point);
                break;
              }
            }
          }
          else if (relativeGoalDis < 0.4)
          {
            if (yawdif1 < 0)
            {
              for (yaw = runYaw; yaw > goalYaw; yaw = yaw - 10 * PI / 180)
              {
                x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
                y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
                if (h > obstacleHeightThre && z > -0.65 && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
                {
                  stopflag = true;
                  break;
                }
              }
              yaw = goalYaw;
              x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
              y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
              if (h > obstacleHeightThre && z > -0.65 && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
              {
                stopflag = true;
              }
              if (stopflag)
              {
                ROS_INFO("warnning 1<!!!!!! %f %f %f %f %f\n", x, y, x3, y3, (yaw - vehicleYaw) / PI * 180);

                point.x = x;
                point.y = y;
                point.z = z;
                freePaths->push_back(point);
                break;
              }
            }
            else
            {
              for (yaw = runYaw; yaw < goalYaw; yaw = yaw + 10 * PI / 180)
              {
                x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
                y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
                if (h > obstacleHeightThre && z > -0.65 && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
                {
                  stopflag = true;
                  break;
                }
              }
              yaw = goalYaw;
              x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
              y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
              if (h > obstacleHeightThre && z > -0.65 && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
              {
                stopflag = true;
              }
              if (stopflag)
              {
                ROS_INFO("warnning 1>!!!!!! %f %f %f %f %f \n", x, y, x3, y3, (yaw - vehicleYaw) / PI * 180);

                point.x = x;
                point.y = y;
                point.z = z;
                freePaths->push_back(point);
                break;
              }
            }

            if (yawdif2 < 0)
            {
              for (yaw = vehicleYaw; yaw > runYaw; yaw = yaw - 10 * PI / 180)
              {
                x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
                y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
                if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
                {
                  stopflag = true;
                  break;
                }
              }
              yaw = runYaw;
              x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
              y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
              if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
              {
                stopflag = true;
              }
              if (stopflag)
              {
                ROS_INFO("warnning 2<!!!!!! %f %f %f %f %f %f %f\n", x, y, x2, y2, (yaw - vehicleYaw) / PI * 180, vehicleYaw / PI * 180, runYaw / PI * 180);

                point.x = x;
                point.y = y;
                point.z = z;
                freePaths->push_back(point);
                break;
              }
            }
            else
            {
              for (yaw = vehicleYaw; yaw < runYaw; yaw = yaw + 10 * PI / 180)
              {
                x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
                y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
                if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
                {
                  stopflag = true;
                  break;
                }
              }
              yaw = runYaw;
              x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
              y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
              if (h > obstacleHeightThre && z > -0.65 && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
              {
                stopflag = true;
              }
              if (stopflag)
              {
                ROS_INFO("warnning 2>!!!!!! %f %f %f %f %f \n", x, y, x2, y2, (yaw - vehicleYaw) / PI * 180);

                point.x = x;
                point.y = y;
                point.z = z;
                freePaths->push_back(point);
                break;
              }
            }
          }
        }

        if (h > obstacleHeightThre && z > -0.65 && (fabs(x) < realvehicleLength / 2.0 && fabs(y) < realvehicleWidth / 2.0 - 0.5))
        {
          ROS_INFO("accident!!!!!! %f %f %f %f \n", x, y, z, h);
          stopflag = true;

          point.x = x;
          point.y = y;
          point.z = z;
          // freePaths->push_back(point);

          break;
        }
      }
      if (stopflag == true)
      {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);

        // 10s内不可达 并且在goal附近 跳过这个wayPoint
        if (timeInterval10 == -1)
        {
          timeInterval10 = ros::Time::now().toSec();
        }
        std::cout << "now: " << ros::Time::now().toSec() << " begin: " << timeInterval10 << " relativeGoalDis: " << relativeGoalDis << "\n";
        if (ros::Time::now().toSec() - timeInterval10 > 10)
        {
          timeInterval5=ros::Time::now().toSec();
          std_msgs::Int8 unableToGoal_;
          unableToGoal_.data = true;
          pubGoalAccessible.publish(unableToGoal_);
        }

        // sensor_msgs::PointCloud2 freePaths2;
        // pcl::toROSMsg(*freePaths, freePaths2);
        // freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        // freePaths2.header.frame_id = "vehicle";
        // pubFreePaths.publish(freePaths2);
        ros::Rate rateac(0.5);
        rateac.sleep();
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
