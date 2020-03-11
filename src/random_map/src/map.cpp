#include <ros/ros.h>
#include "random_map_generator.h"


Param getParam(ros::NodeHandle n);

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "random_map_node");
   ros::NodeHandle n( "~" );
   ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);

   //传入需要随机生成的地图的参数
   randomMap random_map(getParam(n));
   //生成随机地图
   random_map.generate();
   ros::Rate loop_rate(100);
   while (ros::ok())
   {
      if( !random_map.has_map) continue;
      //将地图发布到ros上
      map_pub.publish(random_map.getMap());
      ros::spinOnce();
      loop_rate.sleep();
   }
}

Param getParam(ros::NodeHandle n)
{
   Param map;
   //地图的长宽高
   n.param("map/x_size", map.x_size, 50.0);
   n.param("map/y_size", map.y_size, 50.0);
   n.param("map/z_size", map.z_size, 5.0);

   //生成的障碍物数目、随机圆环数目
   n.param("map/obs_num", map.obs_num, 30);
   n.param("map/circle_num", map.cir_num, 30);

   //地图的精度
   n.param("map/resolution", map.resolution, 0.2);

   //柱子的宽度范围
   n.param("ObstacleShape/lower_rad", map.w_l, 0.1);
   n.param("ObstacleShape/upper_rad", map.w_h, 0.4);

   //柱子的高度范围
   n.param("ObstacleShape/lower_hei", map.h_l, 3.0);
   n.param("ObstacleShape/upper_hei", map.h_h, 7.0);

   //随机生成的圆半径范围
   n.param("CircleShape/lower_circle_rad", map.w_c_l, 0.3);
   n.param("CircleShape/upper_circle_rad", map.w_c_h, 0.8);

   //半长宽高，使生成的地图在视野中央
   map.x_l = -map.x_size / 2.0;
   map.x_h = +map.x_size / 2.0;

   map.y_l = -map.y_size / 2.0;
   map.y_h = +map.y_size / 2.0;
   return map;
}
