#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/duration.h>
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

bool debugmode = true;

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
double fusionVoxelSize = 0.2;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr fusionCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr fusionCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr fusionCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

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
std::vector<int> correspondences[gridVoxelNum]; // 记录不同位置的障碍物会影响到的轨迹

bool newLaserCloud = false;
bool newTerrainCloud = false;
bool newFusionCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter, fusionDwzFilter;

float stuckX = 0;
float stuckY = 0;
double timeInterval20 = -1;
double timeInterval10 = -1;

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

void fusionCloudHandler(const sensor_msgs::PointCloud2ConstPtr &fusionCloud2)
{
  std::cout << "fusionCloudHandler  " << useTerrainAnalysis << std::endl;
  if (useTerrainAnalysis)
  {

    fusionCloud->clear();
    pcl::fromROSMsg(*fusionCloud2, *fusionCloud);

    fusionCloudCrop->clear();
    int fusionCloudSize = fusionCloud->points.size();
    for (int i = 0; i < fusionCloudSize; i++)
    {
      pcl::PointXYZI point;
      float pointX = fusionCloud->points[i].x;
      float pointY = fusionCloud->points[i].y;
      float pointZ = 0;

      // std::cout << "订阅到的点 X:" << pointX << " Y:" << pointY << " Z:" << pointZ << "\n";
      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (abs(pointY) > 0.18 || pointX > 0 || pointX < -0.4)
      {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = 100;
        fusionCloudCrop->push_back(point);
      }
    }

    fusionCloudDwz->clear();
    fusionDwzFilter.setInputCloud(fusionCloudCrop);
    fusionDwzFilter.filter(*fusionCloudDwz);

    newFusionCloud = true;
  }
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
    timeInterval20 = -1;
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

// 靠近障碍物处理
void approachObstaculeHandler(ros::Publisher &pubShortDisNum, ros::Publisher &pubAveLength, ros::Publisher &pubShortestDis)
{
  // 算出障碍物到小车的最短距离（shortestDis），并算出距离小于0.5的障碍物的数量（shortDisNum）障碍物平均距离：（aveLen），发给pathfollower.cpp, 判断是否低速行驶
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
}
// 找到附近的固定路径
bool refresh_Cnt_and_dis_changeobs()
{
  bool realWaypoint = true;
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
    if (dis < 0.3)
      cnt++;
    else
      break;
  }

  double predict_yaw; // 局部变量
  float sinVehicleYaw = sin(vehicleYaw);
  float cosVehicleYaw = cos(vehicleYaw);
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

  // 检查修改change_avoid_obs的值（也就是路径上有无障碍物） change_avoid_obs是全局，因此不用传参
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
      if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && isPointInRectangle(A, B, C, D, Point_({x, y})))
      {
        std::cout << "change_avoid_obs " << change_avoid_obs << " " << x << " " << y << std::endl;
        change_avoid_obs = true;
        if_fixed_path = false;
        break;
      }
    }
    if (change_avoid_obs)
      break;
  }
  return realWaypoint;
}
// 获取两侧的障碍物点
bool getSideBoundarySet(std::set<std::pair<float, float>> &leftboundarySet,
                        std::set<std::pair<float, float>> &rightboundarySet,
                        float relativeGoalX,
                        float relativeGoalY)
{
  pcl::PointXYZI point;
  float disGoal = sqrt(relativeGoalY * relativeGoalY + relativeGoalX * relativeGoalX);
  float cosGoal = relativeGoalX / disGoal;
  float sinGoal = relativeGoalY / disGoal;
  float dismax = realvehicleLength / 2 + disGoal;   // X方向的搜索范围
  float dismin = -realvehicleLength / 2;            // X方向的搜索范围
  float searchRange = (realvehicleWidth + 0.2) / 2; // Y方向的搜索范围
  float safeRange = (realvehicleWidth) / 2;         // Y方向的安全范围，判断是否可通行
  int plannerCloudCropSize = plannerCloudCrop->points.size();

  bool isReach = true;
  for (int i = 0; i < plannerCloudCropSize; i++)
  {
    float x = plannerCloudCrop->points[i].x;
    float y = plannerCloudCrop->points[i].y;
    float z = plannerCloudCrop->points[i].z;
    float h = plannerCloudCrop->points[i].intensity;
    // 小车与目标点连线的方向的坐标系下的障碍物坐标
    float x_to_goal = x * cosGoal + y * sinGoal;
    float y_to_goal = -x * sinGoal + y * cosGoal;
    // pair<float, float> curBoundary{float(((int)((x_to_goal) * 100)) / 100.0), float(((int)((y_to_goal) * 100)) / 100.0)}; // 以厘米为分辨率的栅格
    pair<float, float> curBoundary{x_to_goal, y_to_goal}; // 以厘米为分辨率的栅格

    if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && abs(y_to_goal) < searchRange)
    {
      if ((relativeGoalX > 0 && x_to_goal > dismin && x_to_goal < dismax)) // 目标点在正方
      {
        if (abs(y_to_goal) > safeRange)
        {
          if (y_to_goal >= 0) // 判断放入左/右集合
          {
            point.x = x;
            point.y = y;
            point.intensity = 20;
            freePaths->push_back(point);
            leftboundarySet.insert(curBoundary);
          }
          else
          {
            point.x = x;
            point.y = y;
            point.intensity = 40;
            freePaths->push_back(point);
            rightboundarySet.insert(curBoundary);
          }
        }
        else // 不可通行
        {
          isReach = false;
        }
      }
      else if ((relativeGoalX < 0 && x_to_goal < -dismin && x_to_goal > -dismax)) // 目标点在后方
      {
        if (abs(y_to_goal) > safeRange)
        {
          if (y_to_goal >= 0) // 判断放入左/右集合
          {
            point.x = x;
            point.y = y;
            point.intensity = 40;
            freePaths->push_back(point);
            rightboundarySet.insert(curBoundary);
          }
          else
          {
            point.x = x;
            point.y = y;
            point.intensity = 20;
            freePaths->push_back(point);
            leftboundarySet.insert(curBoundary);
          }
        }
        else // 不可通行
        {
          isReach = false;
        }
      }
    }
  }
  return isReach;
}
// 获取与行进路线垂直的障碍物点对的中点
std::pair<float, float> getFitBoundarySet(std::set<std::pair<float, float>> leftboundarySet,
                                          std::set<std::pair<float, float>> rightboundarySet,
                                          // std::set<std::pair<float, float>> &fitBoundarySet,
                                          float relativeGoalX,
                                          float relativeGoalY)
{
  float slide = 0.1; // 阈值：当两个点y值差小于此值时，存储中点信息。
  float kGoal = relativeGoalY / relativeGoalX;

  // 遍历找出相对平行的左右点对，记录对应中点。
  float minDis = 1000000;
  float targetX = 0;
  float targetY = 0;
  float advanceLen = 0.8;
  // float advanceK = 0;
  // float advanceB;

  float disGoal = sqrt(relativeGoalY * relativeGoalY + relativeGoalX * relativeGoalX);
  float cosGoal = relativeGoalX / disGoal;
  float sinGoal = relativeGoalY / disGoal;

  for (pair<float, float> boundary1 : leftboundarySet)
  {
    for (pair<float, float> boundary2 : rightboundarySet)
    {
      double narrowWidth = sqrt((boundary1.first - boundary2.first) * (boundary1.first - boundary2.first) + (boundary1.second - boundary2.second) * (boundary1.second - boundary2.second));
      if (abs(boundary1.first - boundary2.first) < slide)
      {
        pair<float, float> fixPoint{(boundary1.first + boundary2.first) / 2, (boundary1.second + boundary2.second) / 2};
        {
          pcl::PointXYZI point;
          point.y = (fixPoint.first / cosGoal + fixPoint.second / sinGoal) / (cosGoal / sinGoal + sinGoal / cosGoal);
          point.x = fixPoint.first / cosGoal - point.y * sinGoal / cosGoal;
          point.intensity = 70;
          // freePaths->push_back(point);
        }
        if (minDis > narrowWidth)
        {
          minDis = narrowWidth;
          targetX = cosGoal * fixPoint.first - sinGoal * fixPoint.second;
          targetY = sinGoal * fixPoint.first + cosGoal * fixPoint.second;
          std::cout << "target " << targetX << " " << targetY;
          // TODO 把 targetX targetY 转回小车坐标系

          double targetY_ = (fixPoint.first / cosGoal + fixPoint.second / sinGoal) / (cosGoal / sinGoal + sinGoal / cosGoal);
          double targetX_ = fixPoint.first / cosGoal - targetY_ * sinGoal / cosGoal;
          std::cout << "target_ " << targetX_ << " " << targetY_ << std::endl;
        }
      }
    }
  }
  if (minDis < 10)
  {
    float disGoal = sqrt(relativeGoalY * relativeGoalY + relativeGoalX * relativeGoalX);
    if (disGoal > advanceLen)
    {
      targetY = targetY / abs(targetX) * advanceLen;
      if (relativeGoalX >= 0)
        targetX = advanceLen;
      else
        targetX = -advanceLen;
      // targetY = advanceK * targetX + advanceB;
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
  }
  return std::pair<float, float>(targetX, targetY);
}

void checkSafety(float relativeGoalX, float relativeGoalY,
                 ros::Publisher &pubPath, nav_msgs::Path path)
{
  pcl::PointXYZI point;
  int plannerCloudCropSize = plannerCloudCrop->points.size();
  bool stopflag = false;
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
    // 检查靠近路标点时，到路标点调整朝向时会不会有障碍物
    {
      double goaldis = sqrt((goalX - vehicleX) * (goalX - vehicleX) + (goalY - vehicleY) * (goalY - vehicleY));
      if (goaldis < 0.1)
      {
        if (yawdif3 < 0)
        {
          for (yaw = vehicleYaw; yaw > goalYaw; yaw = yaw - 10 * PI / 180)
          {
            x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
            y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
            if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
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
          if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
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
            if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
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
          if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
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
      else if (wayPointStatus == 2 && goaldis < 0.4)
      {
        if (yawdif1 < 0)
        {
          for (yaw = runYaw; yaw > goalYaw; yaw = yaw - 10 * PI / 180)
          {
            x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
            y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
            if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
            {
              stopflag = true;
              break;
            }
          }
          yaw = goalYaw;
          x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
          y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
          if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
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
            if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
            {
              stopflag = true;
              break;
            }
          }
          yaw = goalYaw;
          x3 = cos(yaw - vehicleYaw) * (x - relativeGoalX) + sin(yaw - vehicleYaw) * (y - relativeGoalY);
          y3 = -sin(yaw - vehicleYaw) * (x - relativeGoalX) + cos(yaw - vehicleYaw) * (y - relativeGoalY);
          if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x3) < usedvehicleLength / 2.0 && fabs(y3) < usedvehicleWidth / 2.0))
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
            if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
            {
              stopflag = true;
              break;
            }
          }
          yaw = runYaw;
          x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
          y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
          if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
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
            if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
            {
              stopflag = true;
              break;
            }
          }
          yaw = runYaw;
          x2 = cos(yaw - vehicleYaw) * x + sin(yaw - vehicleYaw) * y;
          y2 = -sin(yaw - vehicleYaw) * x + cos(yaw - vehicleYaw) * y;
          if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x2) < usedvehicleLength / 2.0 && fabs(y2) < usedvehicleWidth / 2.0))
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
    // 检查是否发生碰撞
    if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) && (fabs(x) < realvehicleLength / 2.0 && fabs(y) < realvehicleWidth / 2.0))
    {
      ROS_INFO("accident!!!!!! %f %f %f %f \n", x, y, z, h);
      stopflag = true;

      point.x = x;
      point.y = y;
      point.z = z;
      freePaths->push_back(point);

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
    if (debugmode)
    {
      std::cout << "0 pubpath for stopflag " << path.poses.size() << std::endl;
    }
    pubPath.publish(path);

    ros::Rate rateac(0.5);
    rateac.sleep();
  }
}

// 生成避障模式下的临时路标点
void get_temp_waypoint(float &relativeGoalX, float &relativeGoalY, bool &realWaypoint)
{
  float sinVehicleYaw = sin(vehicleYaw);
  float cosVehicleYaw = cos(vehicleYaw);
  if (!(sqrt((goalX - vehicleX) * (goalX - vehicleX) + (goalY - vehicleY) * (goalY - vehicleY) < 2.0)) && wayPointStatus != 0)
  {
    if (change_temp_goal) // 改变临时路边点
    {
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
      if (debugmode)
      {
        std::cout << "the most near point is " << PPath[cnt][0] << " " << PPath[cnt][1] << "\n";
      }
      for (int i = cnt; i < sum; i++)
      {
        double dis = sqrt((PPath[i][0] - vehicleX) * (PPath[i][0] - vehicleX) + (PPath[i][1] - vehicleY) * (PPath[i][1] - vehicleY));
        if (dis * pathScale > 3.5)
        {
          relativeGoalX = ((PPath[i][0] - vehicleX) * cosVehicleYaw + (PPath[i][1] - vehicleY) * sinVehicleYaw);
          relativeGoalY = (-(PPath[i][0] - vehicleX) * sinVehicleYaw + (PPath[i][1] - vehicleY) * cosVehicleYaw);

          if (debugmode)
          {
            std::cout << "the temp way point is " << PPath[i][0] << " " << PPath[i][1] << "\n";
            std::cout << "the car is " << vehicleX << " " << vehicleY << "\n";
          }

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
  }
}
// 检查是否长时间徘徊
void checkStuck(float relativeGoalDis, ros::Publisher &pubPath, nav_msgs::Path path)
{
  if (timeInterval20 == -1)
  {
    timeInterval20 = ros::Time::now().toSec();
  }
  else if (ros::Time::now().toSec() - timeInterval20 > 20)
  {
    double StuckDis = sqrt((vehicleX - stuckX) * (vehicleX - stuckX) + (vehicleY - stuckY) * (vehicleY - stuckY));
    std::cout << "20S Pass\n now time " << ros::Time::now().toSec() << " timeInterval20: " << timeInterval20 << " " << int(ros::Time::now().toSec() - timeInterval20) << " \n"
              << vehicleX << " " << vehicleY << " " << stuckX << " " << stuckY << " \nStuckDis： "
              << StuckDis << " relativeGoalDis： " << relativeGoalDis << "\n";
    if ((vehicleX - stuckX) * (vehicleX - stuckX) + (vehicleY - stuckY) * (vehicleY - stuckY) <= vehicleLength * vehicleLength && relativeGoalDis > 1)
    {
      path.poses.resize(1);
      path.poses[0].pose.position.x = 0;
      path.poses[0].pose.position.y = 0;
      path.poses[0].pose.position.z = 0;
      path.poses[0].pose.orientation.w = 0.707;
      path.poses[0].pose.orientation.x = 0.707;
      path.poses[0].pose.orientation.y = 0;
      path.poses[0].pose.orientation.z = 0;
      path.header.stamp = ros::Time().fromSec(odomTime);
      path.header.frame_id = "vehicle";
      pubPath.publish(path);
      if (debugmode)
      {
        std::cout << "!!!!!!!!sleep!!!!!!!\n";
        std::cout << "1 pubpath for sleep " << path.poses.size() << std::endl;
      }
    }
    timeInterval20 = ros::Time::now().toSec();
    stuckX = vehicleX;
    stuckY = vehicleY;
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", realvehicleLength);
  nhPrivate.getParam("vehicleWidth", realvehicleWidth);
  vehicleLength = realvehicleLength;
  vehicleWidth = realvehicleWidth;
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("fusionVoxelSize", fusionVoxelSize);
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

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);
  fusionDwzFilter.setLeafSize(fusionVoxelSize, fusionVoxelSize, fusionVoxelSize);

  ros::Subscriber subnWayPointStatus = nh.subscribe<std_msgs::Int8>("/wayPointStatus", 5, wayPointStatusHandler);
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, laserCloudHandler);
  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, terrainCloudHandler);
  ros::Subscriber subFusionCloud = nh.subscribe<sensor_msgs::PointCloud2>("/fusion_cloud", 5, fusionCloudHandler);
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);
  ros::Subscriber subGoalpoint = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 5, goalposeHandler);
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);
  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped>("/navigation_boundary", 5, boundaryHandler);
  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2>("/added_obstacles", 5, addedObstaclesHandler);
  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool>("/check_obstacle", 5, checkObstacleHandler);
  ros::Subscriber subFixedPath = nh.subscribe<nav_msgs::Path>("fixed_path", 5, pathHandler);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 5);
  ros::Publisher pubGoalAccessible = nh.advertise<std_msgs::Bool>("/GoalAccessible", 1);
  ros::Publisher pubAveLength = nh.advertise<std_msgs::Float32>("/aveLength", 5);
  ros::Publisher pubShortDisNum = nh.advertise<std_msgs::Float32>("/shortDisNum", 5);
  ros::Publisher pubShortestDis = nh.advertise<std_msgs::Float32>("/shortestDis", 5);
  nav_msgs::Path path;
#if PLOTPATHSET == 1
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2>("/free_paths", 2);
#endif
  // 读取DWA需要的轨迹数据
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

    if (newLaserCloud || newTerrainCloud || newFusionCloud) // 收到新的雷达输入
    {
      // 处理障碍物数据，全部障碍物相对小车的坐标存在plannerCloudCrop中
      if (newLaserCloud) // 雷达原始数据（实际不使用）
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

      if (newTerrainCloud) // 地形分析后的雷达数据
      {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      if (newFusionCloud) // 2D激光雷达点云
      {
        newFusionCloud = false;
        std::cout << "newFusionCloud" << std::endl;

        plannerCloud->clear();
        *plannerCloud = *fusionCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear(); // 存储各形式障碍物点云(转换成小车坐标系的)
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
      // 判断近处的障碍物情况，判断是否减速
      approachObstaculeHandler(pubShortDisNum, pubAveLength, pubShortestDis);
      // 处理目标点和DWA相关数据
      float pathRange = adjacentRange; // DWA搜索范围
      if (pathRangeBySpeed)
        pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange)
        pathRange = minPathRange;
      float relativeGoalDis = adjacentRange; // 目标点到机器人的距离
      float relativeGoalX;                   // 相对机器人的目标点
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

      // 计算DWA路径缩放尺度
      float defPathScale = pathScale;
      if (pathScaleBySpeed)
        pathScale = defPathScale * joySpeed;
      if (pathScale < minPathScale)
        pathScale = minPathScale;
      bool useDWA = true;
      freePaths->clear();

      if (wayPointStatus == 2)
      {

        float targetX = 0;
        float targetY = 0;
        float slideVehicleLen = 0.05;

        std::set<std::pair<float, float>> leftboundarySet;
        std::set<std::pair<float, float>> rightboundarySet;
        // std::set<std::pair<float, float>> fitBoundarySet;
        float disGoal = sqrt(relativeGoalY * relativeGoalY + relativeGoalX * relativeGoalX);
        // 获取两侧的障碍物点
        bool isReach = getSideBoundarySet(leftboundarySet,
                                          rightboundarySet,
                                          relativeGoalX,
                                          relativeGoalY);
        if (isReach)
        {
          // 获取与行进路线垂直的障碍物点对的中点集合,和最窄处的中点
          std::pair<float, float> target = getFitBoundarySet(leftboundarySet,
                                                             rightboundarySet,
                                                             //  fitBoundarySet,
                                                             relativeGoalX,
                                                             relativeGoalY);
          targetX = target.first;
          targetY = target.second;
          // 可视化
          // for (pair<float, float> boundary : leftboundarySet)
          // {
          //   point.x = boundary.first;
          //   point.y = boundary.second;
          //   point.intensity = 20;
          //   freePaths->push_back(point);
          // }
          // for (pair<float, float> boundary : rightboundarySet)
          // {
          //   point.x = boundary.first;
          //   point.y = boundary.second;
          //   point.intensity = 40;
          //   freePaths->push_back(point);
          // }
          // for (pair<float, float> boundary : fitBoundarySet)
          // {
          //   point.x = boundary.first;
          //   point.y = boundary.second;
          //   point.intensity = 70;
          //   freePaths->push_back(point);
          // }
          point.x = targetX;
          point.y = targetY;
          freePaths->push_back(point);
          // printf("targetX:%f tagertY:%f relativeGoalX:%f relativaGoalY:%f\n", targetX, targetY, relativeGoalX, relativeGoalY);
          // 输出path
          int pathSize = int(relativeGoalDis * 100);
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
            if (i == path.poses.size() - 1 && sqrt((path.poses[i].pose.position.x - relativeGoalX) * (path.poses[i].pose.position.x - relativeGoalX) + (path.poses[i].pose.position.y - relativeGoalY) * (path.poses[i].pose.position.y - relativeGoalY)) < 0.4 && goalYaw > -4)
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
          if (debugmode)
          {
            std::cout << "2 pubpath for narrow way target " << path.poses.size() << std::endl;
          }
          pubPath.publish(path);
        }
        else // 不可达 停止等待
        {
          std::cout << "unreachable!" << std::endl;
          path.poses.resize(1);
          path.poses[0].pose.position.x = 0;
          path.poses[0].pose.position.y = 0;
          path.poses[0].pose.position.z = 0;
          path.poses[0].pose.orientation.w = 0.707;
          path.poses[0].pose.orientation.x = 0.707;
          path.poses[0].pose.orientation.y = 0;
          path.poses[0].pose.orientation.z = 0;
          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          if (debugmode)
          {
            std::cout << "3 pubpath for unreachable " << path.poses.size() << std::endl;
          }
          pubPath.publish(path);
        }
      }
      else
      {

        bool realWaypoint = true;
        // std::cout << "if_fixed_path " << if_fixed_path << std::endl;
        if (if_fixed_path && wayPointStatus != 0)
        {
          path.poses.clear();
          // 找到附近的固定路径
          realWaypoint = refresh_Cnt_and_dis_changeobs();
          // 如果没有障碍物,发布固定路径为path
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
              if (i == path.poses.size() - 1 && sqrt((path.poses[i].pose.position.x - relativeGoalX) * (path.poses[i].pose.position.x - relativeGoalX) + (path.poses[i].pose.position.y - relativeGoalY) * (path.poses[i].pose.position.y - relativeGoalY)) < 0.4 && goalYaw > -4 && realWaypoint)
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
            if (debugmode)
            {
              std::cout << "4 pubpath for fix way " << path.poses.size() << std::endl;
            }
            pubPath.publish(path);
          }
        }
        if (change_avoid_obs || wayPointStatus == 0)
        {
          // 检查是否长时间徘徊
          checkStuck(relativeGoalDis, pubPath, path);
          // 生成临时路标点
          get_temp_waypoint(relativeGoalX, relativeGoalY, realWaypoint); // 作用就是改变relativeGoalX  和 relativeGoalY

          // 可视化路标点
          point.x = relativeGoalX;
          point.y = relativeGoalY;
          point.z = 0.1;
          point.intensity = 100;
          freePaths->push_back(point);
          // DWA算法
          bool pathFound = false;
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
            // 遍历障碍物，计算DWA每条轨迹的碰撞惩罚
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
                      if ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) || !useTerrainAnalysis)
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
                  ((h > obstacleHeightThre || z > -0.3 || sqrt(x * x + y * y + z * z) < 1) || !useTerrainAnalysis) && checkRotObstacle)
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
            // 计算每条轨迹的分数
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
            // 发布最优路径
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
                if (i == path.poses.size() - 1 && sqrt((path.poses[i].pose.position.x - relativeGoalX) * (path.poses[i].pose.position.x - relativeGoalX) + (path.poses[i].pose.position.y - relativeGoalY) * (path.poses[i].pose.position.y - relativeGoalY)) < 0.4 && goalYaw > -4 && realWaypoint)
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
              if (debugmode)
              {
                std::cout << "5 pubpath for avoid " << path.poses.size() << std::endl;
              }
              pubPath.publish(path);
#if PLOTPATHSET == 1
              bool isGoalUnaccessible = true;

              double relativeGoalX_ = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
              double relativeGoalY_ = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

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
                // 可视化候选路径，并检测目标点与临时目标点是否可达
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
                        if (dis < 0.05) // 能否到达临时目标点
                        {
                          change_temp_goal = false;
                        }
                        dis = sqrt((point.x - relativeGoalX_) * (point.x - relativeGoalX_) + (point.y - relativeGoalY_) * (point.y - relativeGoalY_));
                        if (dis < 0.05) // 能否到达目标点
                        {
                          isGoalUnaccessible = false;
                        }
                      }
                      freePaths->push_back(point);
                    }
                  }
                }
              }

              // 目标点不可达且，10s内不可达 并且在goal附近 跳过这个wayPoint
              if (relativeGoalX_ * relativeGoalX_ + relativeGoalY_ * relativeGoalY_ < 1)
              {
                if (isGoalUnaccessible)
                {
                  std::cout << "near wayPoint\nisGoalUnaccessible = " << isGoalUnaccessible << " timeInterval10 " << timeInterval10 << " " << int(ros::Time::now().toSec() - timeInterval10) << "\n";
                  if (timeInterval10 == -1)
                  {
                    timeInterval10 = ros::Time::now().toSec();
                  }
                  if (ros::Time::now().toSec() - timeInterval10 > 10)
                  {
                    std_msgs::Int8 unableToGoal_;
                    unableToGoal_.data = true;
                    pubGoalAccessible.publish(unableToGoal_);
                  }
                }
              }
#endif
            }

            if (sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY) < realvehicleLength)
            {
              change_avoid_obs = false;
              timeInterval20 = -1;
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
          // DWA算法结束
          pathScale = defPathScale;

          if (!pathFound)
          {
            path.poses.resize(1);
            path.poses[0].pose.position.x = 0;
            path.poses[0].pose.position.y = 0;
            path.poses[0].pose.position.z = 0;
            path.poses[0].pose.orientation.w = 0.707;
            path.poses[0].pose.orientation.x = 0.707;
            path.poses[0].pose.orientation.y = 0;
            path.poses[0].pose.orientation.z = 0;
            path.header.stamp = ros::Time().fromSec(odomTime);
            path.header.frame_id = "vehicle";
            if (debugmode)
            {
              std::cout << "6 pubpath for pathnotFound " << path.poses.size() << std::endl;
            }
            pubPath.publish(path);
          }
        }
      }
      // 检查安全性
      checkSafety(relativeGoalX, relativeGoalY, pubPath, path);
      sensor_msgs::PointCloud2 freePaths2;
      pcl::toROSMsg(*freePaths, freePaths2);
      freePaths2.header.stamp = ros::Time().fromSec(odomTime);
      freePaths2.header.frame_id = "vehicle";
      pubFreePaths.publish(freePaths2);
      status = ros::ok();
      rate.sleep();
    }
  }
  return 0;
}
