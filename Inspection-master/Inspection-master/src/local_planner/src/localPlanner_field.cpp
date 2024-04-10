#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
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

string pathFolder;
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
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
double goalX = 0;
double goalY = 0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

// const int pathNum = 343;
// const int groupNum = 7;
// float gridVoxelSize = 0.01;
// float searchRadius = 0.5;
// float gridVoxelOffsetX = 1.2;
// float gridVoxelOffsetY = 2;
// const int gridVoxelNumX = 121;
// const int gridVoxelNumY = 401;

// const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

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
// pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
// #if PLOTPATHSET == 1
// pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
// pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
// #endif

// int pathList[pathNum] = {0};
// float endDirPathList[pathNum] = {0};
// int clearPathList[36 * pathNum] = {0};
// float pathPenaltyList[36 * pathNum] = {0};
// float clearPathPerGroupScore[36 * groupNum] = {0};
// std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
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

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!useTerrainAnalysis) {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
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

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (useTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
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

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
  double checkObsTime = ros::Time::now().toSec();

  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

class pointVector
{
public:
    pointVector(pcl::PointXYZI p1, pcl::PointXYZI p2) :pBegin(p1), pEnd(p2)//, length(len)//初始化列表
    {
        length = sqrt(pow((pEnd.x - pBegin.x), 2) + pow(pEnd.y - pBegin.y, 2));//计算长度
        //drawVector();
    }

    pointVector addVector(pointVector v1, pointVector v2)//向量求和
    {
        pcl::PointXYZI pEndtmp;
        pEndtmp.x=v2.pEnd.x + v1.pEnd.x - v1.pBegin.x;
        pEndtmp.y=v2.pEnd.y + v1.pEnd.y - v1.pBegin.y;
        pointVector vt(v1.pBegin, pEndtmp);
        return vt;
    }
    pcl::PointXYZI pBegin;
    pcl::PointXYZI pEnd;
    double length;
};
class ArtificialPotential//人工势场法
{
public:
	double disCal(pcl::PointXYZI p1, pcl::PointXYZI p2)//两点距离计算
  {
	  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }
	pointVector Fatt()//计算引力
  {
    pcl::PointXYZI pTarget,pCar;
    pTarget.x=goalX;
    pTarget.y=goalY;
    pCar.x=vehicleX;
    pCar.y=vehicleY;
  Rho_att = disCal(pTarget, pCar);//与目标点距离
	double F_att = eta * Rho_att;//引力大小
	pcl::PointXYZI pFatt;
  pFatt.x=F_att / Rho_att * (pTarget.x - pCar.x) + pCar.x;
  pFatt.y=F_att / Rho_att * (pTarget.y - pCar.y) + pCar.y;
	pointVector vFatt(pCar, pFatt);

	// vFatt.printVector("vFatt");
	//vFatt.drawVector();

	return vFatt;
}

	pointVector Fobs()//计算障碍物斥力
{
  int plannerCloudCropSize = plannerCloud->size();
 pcl::PointXYZI pTarget,pCar;
    pTarget.x=goalX;
    pTarget.y=goalY;
    pCar.x=vehicleX;
    pCar.y=vehicleY;
  for(int i=0;i<plannerCloudCropSize;i++)
	{
    pcl::PointXYZI point_;
    point_=plannerCloud->at(i);
		double Rho_obs = disCal(point_, pCar);//与这个障碍物中心点的距离
		double Freo1 = 0.0;
		double Freo2 = 0.0;//这个障碍物产生的两个引力的大小

		if (Rho_obs >= Rho0)
		{
			Freo1 = 0.0;
			Freo2 = 0.0;
		}
		else
		{
			Freo1 = k * (1 / Rho_obs - 1 / Rho0) * pow(Rho_att, n) / pow(Rho_obs, 2);
			Freo2 = n * k / 2 * pow(1 / Rho_obs - 1 / Rho0, 2) * pow(Rho_att, n - 1);
		}

		pcl::PointXYZI pFreo1;
    pFreo1.x=(pCar.x - point_.x) / Rho_obs * (Freo1 + Rho_obs) + point_.x;
    pFreo1.y=(pCar.y - point_.y) / Rho_obs * (Freo1 + Rho_obs) + point_.y;//由这个障碍物指向车辆
		pcl::PointXYZI pFreo2;
    pFreo2.x=Freo2 / Rho_att * (pTarget.x - pCar.x) + pCar.x;
    pFreo2.y=Freo2 / Rho_att * (pTarget.y - pCar.y) + pCar.y;//由车辆指向目标点
		pointVector vFreo1(pCar, pFreo1);
		pointVector vFreo2(pCar, pFreo2);
		pointVector vFobs = vFreo1.addVector(vFreo1, vFreo2);//计算这个障碍物的合斥力
		vFreo.push_back(vFobs);//存入容器

	}

	//计算所有障碍物产生的合斥力
	pointVector vFobsTotal = vFreo[0];
	for (int i = 1; i < vFreo.size(); i++)
	{
		vFobsTotal = vFobsTotal.addVector(vFobsTotal, vFreo[i]);
	}
	//vFobsTotal.drawVector();
	return vFobsTotal;
}
 nav_msgs::Path  process()
{
  nav_msgs::Path path;
  path.poses.resize(2);
  path.header.stamp = ros::Time().fromSec(odomTime);
  path.header.frame_id = "vehicle";
    pointVector vFatt = Fatt();//引力
		pointVector vFobsTotal = Fobs();//总障碍物斥力
		pointVector vFtotal = vFatt.addVector(vFatt, vFobsTotal);//引力与斥力的合力

		double dis = v0 * delta_t + vFtotal.length * pow(delta_t, 2) / 2;//每个时间间隔内走过的距离
		v0 += vFtotal.length * delta_t;//速度
    path.poses[0].pose.position.x = 0;
    path.poses[0].pose.position.y = 0;
    path.poses[0].pose.position.z = 0;
    path.poses[0].pose.orientation.w=1;
    path.poses[0].pose.orientation.x=0;
    path.poses[0].pose.orientation.y=0;
    path.poses[0].pose.orientation.z=0;
    float sinVehicleYaw = sin(vehicleYaw);
    float cosVehicleYaw = cos(vehicleYaw);
    double nextp_x= dis / vFtotal.length * (vFtotal.pEnd.x - vFtotal.pBegin.x) + vFtotal.pBegin.x;
    double nextp_y= dis / vFtotal.length * (vFtotal.pEnd.y - vFtotal.pBegin.y) + vFtotal.pBegin.y;
		path.poses[1].pose.position.x = ((nextp_x - vehicleX) * cosVehicleYaw 
                + (nextp_y - vehicleY) * sinVehicleYaw);
		path.poses[1].pose.position.y = (-(nextp_x - vehicleX) * sinVehicleYaw 
                + (nextp_y - vehicleY) * cosVehicleYaw);//更新车辆坐标位置
    path.poses[1].pose.position.z = 0;
		path.poses[1].pose.orientation.w=1;
    path.poses[1].pose.orientation.x=0;
    path.poses[1].pose.orientation.y=0;
    path.poses[1].pose.orientation.z=0;
		// cout << "CarLoc: (" << road.car0->pc.x << ", " << road.car0->pc.y << "), dis to goal: " << Rho_att << ", dis: " << dis << ", v: " << v0 << endl << endl;
		//road.car0->drawCar(road.car0->pc);
		vFreo.clear();
    
    return path;
}
public:
	double delta_t = 0.02;//时间步长
	double Rho_att;//与目标点的距离
	double eta = 0.3;//引力增益系数//已考虑车辆质量，算出的力直接当作加速度用
	double k = 0.3;//障碍物斥力增益系数//已考虑车辆质量，算出的力直接当作加速度用
	int n = 3;
	double Rho0 = 500.0;//斥力起作用的范围
	vector<pointVector> vFreo;//存储每个障碍物产生的合斥力
	double v0 = 0.0;//起始速度为0
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
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

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/terrain_map", 5, terrainCloudHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5, boundaryHandler);

  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2> ("/added_obstacles", 5, addedObstaclesHandler);

  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool> ("/check_obstacle", 5, checkObstacleHandler);

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);
  nav_msgs::Path path;

  #if PLOTPATHSET == 1
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2> ("/free_paths", 2);
  #endif


  printf ("\nReading path files.\n");

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  printf ("\nInitialization complete.\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloud || newTerrainCloud) {
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
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
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange) pathRange = minPathRange;
      float relativeGoalDis = adjacentRange;

      if (autonomyMode) {
        float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        if (!twoWayDrive) {
          if (joyDir > 90.0) joyDir = 90.0;
          else if (joyDir < -90.0) joyDir = -90.0;
        }
      }

      // bool pathFound = false;
      if((vehicleX-goalX)*(vehicleX-goalX)+(vehicleY-goalY)*(vehicleY-goalY)>0.04)
      {
          ArtificialPotential artificialPotential;
          path=artificialPotential.process();
          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);
      }
    //accident check
      int plannerCloudCropSize = plannerCloudCrop->points.size();
      for(int i = 0; i < plannerCloudCropSize; i++)
      {
          float x = plannerCloudCrop->points[i].x;
          float y = plannerCloudCrop->points[i].y;
          if((fabs(x) < vehicleLength  / 2.0 && fabs(y) < vehicleWidth/ 2.0) )
          {
            ROS_INFO("accident!!!!!!\n");
            break;
          }
      }

    }
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
