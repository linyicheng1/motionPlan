#include "A_star.h"
#include "JPS.h"
#include "sample_base.cpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include "path_finding.h"

void rcvMap_CB(const sensor_msgs::PointCloud2 &map);
void rcvTarget_CB(const nav_msgs::Path &target);
void mapParamConfig(mapParam &param,ros::NodeHandle n);
a_star aStar_G;
int main(int argc,char **argv)
{
    //订阅地图信息、目标点信息，设置当前位置为（0,0,0）
    //发布路径、发布调试信息
    ros::init(argc,argv,"path_finding");
    ros::NodeHandle n("~");
    ros::Subscriber map_sub,target_sub;
    ros::Publisher path_pub;
    map_sub = n.subscribe("/random_map_node/global_map",1,rcvMap_CB);
    target_sub = n.subscribe("/waypoint_generator/waypoints",1,rcvTarget_CB);
    path_pub = n.advertise<sensor_msgs::PointCloud2>("path_vis",1);

    mapParam param;
    mapParamConfig(param,n);
    aStar_G.setMapParam(param);

    ros::Rate rate(100);
    bool status = ros::ok();
    ROS_INFO("initial finished ,wait for map");
    while(status)
    {
        aStar_G.findPath();
        if(!aStar_G.getPath().empty())
        {
            ROS_INFO("need to publish path msg !!!");
        }
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
/**
 * @biref 地图信息回调函数，接收地图信息并给算法类赋值
 * @param map 地图数据
 */
void rcvMap_CB(const sensor_msgs::PointCloud2 &map)
{
    static bool has_map = false;
    if(has_map)
    {//只在初始化的时候更新一次地图
        return ;
    }
    has_map = true;
    ROS_INFO("have received the map message!!!");
    //转换成pcl点云的数据格式
    pcl::PointCloud<pcl::PointXYZ> map_pcl;
    pcl::fromROSMsg(map,map_pcl);
    //给算法类设定地图信息
    aStar_G.setMap(map_pcl);

}

/**
 * @brief 目标点的回调函数,接收数据并赋值
 * @param target
 */
void rcvTarget_CB(const nav_msgs::Path &target)
{
    if(target.poses[0].pose.position.z<0.0)
    {
        ROS_WARN("target pose is out of the map !");
        return;
    }
    Eigen::Vector3d target3d;
    target3d << target.poses[0].pose.position.x,
                target.poses[0].pose.position.y,
                target.poses[0].pose.position.z;
    ROS_INFO("get target point success ! x:%f y:%f z:%f",target3d[0],target3d[1],target3d[2]);
    aStar_G.setTarget(target3d);
}

void mapParamConfig(mapParam &param,ros::NodeHandle n)
{
    n.param("map/cloud_margin",param.cloud_margin,0.0);
    n.param("map/resolution",param.resolution,0.2);

    n.param("map/x_size",param.x_size, 10.0);
    n.param("map/y_size",param.y_size, 10.0);
    n.param("map/z_size",param.z_size, 2.0 );

    param.map_lower << -param.x_size/2,-param.y_size/2,0.0;
    param.map_upper << +param.x_size/2,+param.y_size/2,param.z_size;
    param.inv_resolution = 1.0/param.resolution;

    param.max_x_id = (int)(param.x_size*param.inv_resolution);
    param.max_y_id = (int)(param.y_size*param.inv_resolution);
    param.max_z_id = (int)(param.z_size*param.inv_resolution);
}


