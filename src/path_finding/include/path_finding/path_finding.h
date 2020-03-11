#ifndef __PATH_FINDINF__
#define __PATH_FINDINF__ 
#include <vector>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef struct mapParam
{
    double cloud_margin;
    double resolution;
    double inv_resolution;

    double x_size;
    double y_size;
    double z_size;

    Eigen::Vector3d map_lower;
    Eigen::Vector3d map_upper;

    int max_x_id;
    int max_y_id;
    int max_z_id;
}mapParam;


//路径规划类的基类
//各路径规划算法需要继承该类，从而统一接口
class path_finding
{
public:
    path_finding(){}
    ~path_finding(){}
    std::vector<Eigen::Vector3d> getPath(){return path_;}//返回规划好的路径
    void setMap(pcl::PointCloud<pcl::PointXYZ> cloud){map_ = cloud;has_map_ = true;}//设置地图
    void setStart(Eigen::Vector3d start){startPoint_ = start;}//设置初始位置
    void setTarget(Eigen::Vector3d target){targetPoint_ = target;has_target_ = true;}//设置终点位置
    virtual void findPath(){}//开始搜索路径

    Eigen::Vector3d startPoint_;
    Eigen::Vector3d targetPoint_;
    pcl::PointCloud<pcl::PointXYZ> map_;
    std::vector<Eigen::Vector3d> path_;
    bool has_map_;
    bool has_target_;
};


#endif //__PATH_FINDINF__ 
