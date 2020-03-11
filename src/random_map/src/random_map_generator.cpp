#include "random_map_generator.h"

using namespace std;
using namespace Eigen;

randomMap::randomMap(Param map)
{
   map_ = map;
}

void randomMap::generate()
{
   pcl::PointXYZ pt_random;
   random_device rd;
   default_random_engine eng(rd());

   //位置x，y
   rand_x = uniform_real_distribution<double>(map_.x_l, map_.x_h );
   rand_y = uniform_real_distribution<double>(map_.y_l, map_.y_h );
   //柱子宽度w
   rand_w = uniform_real_distribution<double>(map_.w_l, map_.w_h);
   //柱子高度h
   rand_h = uniform_real_distribution<double>(map_.h_l, map_.h_h);
   //随机生成圆心和半径
   uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(map_.x_l + 1.0, map_.x_h - 1.0);
   uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(map_.y_l + 1.0, map_.y_h - 1.0);
   uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(map_.w_c_l    , map_.w_c_h    );
   //长短半轴的缩放系数、使圆变成随机椭圆
   uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
   //圆的旋转随机角度
   uniform_real_distribution<double> rand_roll      = uniform_real_distribution<double>(- M_PI,     + M_PI);
   uniform_real_distribution<double> rand_pitch     = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
   uniform_real_distribution<double> rand_yaw       = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
   //首先生成一些圆
   for(int i=0;i<map_.cir_num;i++)
   {
      std::vector<Vector3d> circle_set;
      double x0, y0, z0, R;
      x0   = rand_x_circle(eng);
      y0   = rand_y_circle(eng);
      z0   = rand_h(eng) / 2.0;
      R    = rand_r_circle(eng);
      //避免在起点有障碍物
      if(sqrt( pow(x0, 2) + pow(y0, 2) ) < 2.0 )
         continue;
      double a, b;
      a = rand_ellipse_c(eng);
      b = rand_ellipse_c(eng);
      //根据参数生成一圈的点云
      double x, y, z;
      Vector3d pt3, pt3_rot;
      for(double theta = -M_PI; theta < M_PI; theta += 0.025)
      {
         x = a * cos(theta) * R;
         y = b * sin(theta) * R;
         z = 0;
         pt3 << x, y, z;
         circle_set.push_back(pt3);
      }
      //旋转圆
      Matrix3d Rot;
      double roll,  pitch, yaw;
      roll  = rand_roll(eng);
      pitch = rand_pitch(eng);
      yaw   = rand_yaw(eng);
      //旋转矩阵
      Rot << cos(roll) * cos(yaw)  - cos(pitch) * sin(roll) * sin(yaw), - cos(pitch) * cos(yaw) * sin(roll) - cos(roll) * sin(yaw),   sin(roll) * sin(pitch),
              cos(yaw)  * sin(roll) + cos(roll) * cos(pitch) * sin(yaw),   cos(roll) * cos(pitch) * cos(yaw) - sin(roll) * sin(yaw), - cos(roll) * sin(pitch),
              sin(pitch)  * sin(yaw),                                         cos(yaw) * sin(pitch),                                         cos(pitch);
      for(auto pt: circle_set)
      {
         pt3_rot = Rot * pt;
         pt_random.x = (float)(pt3_rot(0) + x0 + 0.001);
         pt_random.y = (float)(pt3_rot(1) + y0 + 0.001);
         pt_random.z = (float)(pt3_rot(2) + z0 + 0.001);

         if(pt_random.z >= 0.0)
            cloudMap_.points.push_back( pt_random );
      }
   }

   //然后生成一些柱子
   for(int i=0;i<map_.obs_num;i++)
   {
      double x, y, w, h;
      x    = rand_x(eng);
      y    = rand_y(eng);
      w    = rand_w(eng);

      //在原点的一定范围内不产生障碍物
      if(sqrt( pow(x, 2) + pow(y, 2) ) < 0.8 )
         continue;

      //对位置、宽度取整数便于构成点云
      x = floor(x/map_.resolution) * map_.resolution + map_.resolution / 2.0;
      y = floor(y/map_.resolution) * map_.resolution + map_.resolution / 2.0;
      int widNum = (int)ceil(w/map_.resolution);
      //生成一个点云柱
      for(int r = (int)(-widNum/2.0); r < widNum/2.0; r ++ )
      {
         for(int s = (int)(-widNum/2.0); s < widNum/2.0; s ++ )
         {
            h    = rand_h(eng);
            int heiNum = (int)(2.0 * ceil(h/map_.resolution));
            for(int t = 0; t < heiNum; t ++ ){
               pt_random.x = (float)(x + (r+0.0) * map_.resolution + 0.001);
               pt_random.y = (float)(y + (s+0.0) * map_.resolution + 0.001);
               pt_random.z = (float)(    (t+0.0) * map_.resolution * 0.5 + 0.001);
               cloudMap_.points.push_back( pt_random );
            }
         }
      }
   }
   cloudMap_.is_dense = true;
   has_map = 1;
   //坐标限定
   pcl::PointXYZ pt;
   for(int idx=0;idx<(int)cloudMap_.points.size();idx++)
   {

      pt = cloudMap_.points[idx];
      Vector3d cor_round = coordRounding(Vector3d(pt.x,pt.y,pt.z));
      pt.x = (float)cor_round(0);
      pt.y = (float)cor_round(1);
      pt.z = (float)cor_round(2);
      RoundCloudMap_.push_back(pt);
   }
   //转成ros格式
   pcl::toROSMsg(RoundCloudMap_, globalMap_pcd_);
   globalMap_pcd_.header.frame_id = "world";

}

Eigen::Vector3d randomMap::coordRounding(const Eigen::Vector3d & coord)
{
   return gridIndex2coord(coord2gridIndex(coord));
}

Eigen::Vector3i randomMap::coord2gridIndex(const Eigen::Vector3d &pt)
{
   auto _max_x_id = (int)(map_.x_size / map_.resolution);
   auto _max_y_id = (int)(map_.y_size / map_.resolution);
   auto _max_z_id = (int)(map_.z_size / map_.resolution);

   Vector3i idx;
   idx <<  min( max( int( (pt(0) - map_.x_l) / map_.resolution), 0), _max_x_id - 1),
           min( max( int( (pt(1) - map_.y_l) / map_.resolution), 0), _max_y_id - 1),
           min( max( int( (pt(2) - 0) / map_.resolution), 0), _max_z_id - 1);

   return idx;
}

Eigen::Vector3d randomMap::gridIndex2coord(const Eigen::Vector3i &index)
{
   Vector3d pt;
   //pt*精度 + 偏移量
   pt(0) = ((double)index(0) + 0.5) * map_.resolution + map_.x_l;
   pt(1) = ((double)index(1) + 0.5) * map_.resolution + map_.y_l;
   pt(2) = ((double)index(2) + 0.5) * map_.resolution + 0;

   return pt;
}
