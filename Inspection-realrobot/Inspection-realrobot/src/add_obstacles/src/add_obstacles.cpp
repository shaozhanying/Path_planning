#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std;

//读取position.txt中的信息，生产多边形障碍。
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "add_obstacles");

    ros::NodeHandle nh;
    std::fstream file;
    sensor_msgs::PointCloud2 pub_obstacles;
    std::string file_src=ros::package::getPath("add_obstacles");
    file.open(file_src+"/position.txt");
    int obstacle_num;
    file >> obstacle_num;
    pcl::PointCloud<pcl::PointXYZI> all_add_obstacle;
    for(int j = 0;j < obstacle_num;j++){
      int point_num;
      vector<vector<double>> position;
      file >> point_num;
      for(int i = 0;i < point_num;i++){
        double x,y;
        file >> x >> y;
        vector<double> xy;
        xy.push_back(x);
        xy.push_back(y);
        position.push_back(xy);
      }

      for(int i = 0;i < point_num;i++){
        pcl::PointCloud<pcl::PointXYZI> add_obstacle;
        double x1 = position[i][0],y1 = position[i][1];
        double x2 = position[(i + 1)%point_num][0],y2 = position[(i + 1)%point_num][1];
        double length = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        double add_x = x1, add_y = y1;
        add_obstacle.width = 10 * length;
        add_obstacle.height = 30;
        add_obstacle.is_dense=false;
        add_obstacle.sensor_orientation_=Eigen::Quaternionf::Identity();
        add_obstacle.sensor_origin_=Eigen::Vector4f::Identity();
        add_obstacle.points.resize(add_obstacle.width * add_obstacle.height);
        for(int j = 0;j < (int)(10 * length);j++){
          add_x += (x2 - x1) / (10 * length);
          add_y += (y2 - y1) / (10 * length);
          for(double m = 0;m < add_obstacle.height;m++){
            add_obstacle.points[j * add_obstacle.height + m].x = add_x;
            add_obstacle.points[j * add_obstacle.height + m].y = add_y;
            add_obstacle.points[j * add_obstacle.height + m].z = m / add_obstacle.height * 2 - 0.5;
            add_obstacle.points[j * add_obstacle.height + m].intensity = 200;
      
          }
        }
      all_add_obstacle += add_obstacle;
      }
    }
      file.close();
    
    ros::Publisher pubAddedObstacles = nh.advertise<sensor_msgs::PointCloud2> ("/added_obstacles", 5);
    pcl::toROSMsg(all_add_obstacle,pub_obstacles);
    pub_obstacles.header.frame_id = "map";
    pub_obstacles.header.stamp=ros::Time::now();
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        ros::spinOnce();
        pubAddedObstacles.publish(pub_obstacles);
        loop_rate.sleep();
    }
    return 0;
}