#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

typedef struct Param
{
    double x_size;
    double y_size;
    double z_size;

    int obs_num;
    int cir_num;
    double resolution;

    double w_l;
    double w_h;
    double h_l;
    double h_h;

    double w_c_l;
    double w_c_h;

    double x_l;
    double x_h;

    double y_l;
    double y_h;
}Param;

class randomMap
{
public:
    randomMap();
    randomMap(Param map);
    void generate();
    int has_map = 0;
    sensor_msgs::PointCloud2 getMap(){return globalMap_pcd_;}
private:
    Param map_;
    std::uniform_real_distribution<double> rand_x;
    std::uniform_real_distribution<double> rand_y;
    std::uniform_real_distribution<double> rand_w;
    std::uniform_real_distribution<double> rand_h;
    sensor_msgs::PointCloud2 globalMap_pcd_;
    pcl::PointCloud<pcl::PointXYZ> cloudMap_;
    pcl::PointCloud<pcl::PointXYZ> RoundCloudMap_;
    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
};
