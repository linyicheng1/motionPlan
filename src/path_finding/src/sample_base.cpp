#include "sample_base.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ompl::base::State *state);

sample_base::sample_base(mapParam param,int type)
{
    plannerType_ = type;
    param_ = param;
    setStart(Eigen::Vector3d(0,0,0));
}

void sample_base::setMap(pcl::PointCloud<pcl::PointXYZ> cloud)//三维转一维
{
    map_ = cloud;
    has_map_ = true;

    obj = new uint8_t[param_.max_x_id*param_.max_y_id*param_.max_z_id];
    memset(obj,0,param_.max_x_id*param_.max_y_id*param_.max_z_id* sizeof(uint8_t));
    for(int i=0;i<(int)cloud.points.size();i++)
    {
        if(cloud.points[i].x<param_.map_lower[0]||
           cloud.points[i].y<param_.map_lower[1]||
           cloud.points[i].z<param_.map_lower[2]||
           cloud.points[i].x>=param_.map_upper[0]||
           cloud.points[i].y>=param_.map_upper[1]||
           cloud.points[i].z>=param_.map_upper[2])
        {
            return ;
        }
        auto id_x = static_cast<int>((cloud.points[i].x-param_.map_lower[0])*param_.inv_resolution);
        auto id_y = static_cast<int>((cloud.points[i].y-param_.map_lower[1])*param_.inv_resolution);
        auto id_z = static_cast<int>((cloud.points[i].z-param_.map_lower[2])*param_.inv_resolution);

        int get = id_x*param_.max_y_id*param_.max_z_id + id_y*param_.max_z_id + id_z;
        obj[get] = uint8_t(1);
    }
}

void sample_base::findPath()
{
    if(has_map_&&has_target_) {
        has_target_ = false;
        path_.clear();
        //搜索空间为3d
        ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
        //设置搜索空间范围
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, param_.map_lower[0]);
        bounds.setLow(1, param_.map_lower[1]);
        bounds.setLow(2, param_.map_lower[2]);

        bounds.setHigh(0, param_.map_upper[0]);
        bounds.setHigh(1, param_.map_upper[1]);
        bounds.setHigh(2, param_.map_upper[2]);
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        //信息空间
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        //障碍物检测
        si->setStateValidityChecker(isStateValid);
        si->setup();//BUG
        //设置初始位置和目标位置
        ob::ScopedState<> start(space);
        ob::ScopedState<> target(space);
        for (int i = 0; i < 3; i++) {
            start[i] = startPoint_[i];
            target[i] = targetPoint_[i];
        }
        //构造求解问题
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, target);
        pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

        //构造求解方法
        ob::PlannerPtr optimizingPlaner(std::make_shared<og::RRTstar>(si));
        optimizingPlaner->setProblemDefinition(pdef);
        optimizingPlaner->setup();
        //求解规划问题
        double runTime = 2.0;
        ompl::base::PlannerStatus solved = optimizingPlaner->solve(runTime);
        if(solved)
        {
            std::cout<<"RTT has find the target point!"<<std::endl;
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            std::vector<Eigen::Vector3d> path_points;
            for(int pathId = 0;pathId<path->getStateCount();pathId++)
            {
                const ob::RealVectorStateSpace::StateType *state = path->getState(pathId)->as<ob::RealVectorStateSpace::StateType>();
                Eigen::Vector3d point;

                point[0] = state->values[0];
                point[1] = state->values[1];
                point[2] = state->values[2];

                path_.push_back(point);
                setStart(targetPoint_);
            }
        }
    }
    else
    {
        return;
    }
}

extern sample_base *RRT_Star_Ptr;
bool isStateValid(const ompl::base::State *state)//检测是否有障碍物
{
    const ob::RealVectorStateSpace::StateType* state3D =
            state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector3d coord;
    Eigen::Vector3i id;
    coord << state3D->values[0],
             state3D->values[1],
             state3D->values[2];
    id = RRT_Star_Ptr->coord2gridIndex(coord);
    //返回是否越界、
    return (id[0] >= 0 && id[0] < RRT_Star_Ptr->param_.max_x_id && id[1] >= 0 && id[1] < RRT_Star_Ptr->param_.max_y_id && id[2] >= 0 && id[2] < RRT_Star_Ptr->param_.max_z_id &&
            (RRT_Star_Ptr->obj[id[0]*RRT_Star_Ptr->param_.max_y_id*RRT_Star_Ptr->param_.max_z_id + id[1]*RRT_Star_Ptr->param_.max_z_id + id[2]] < 0.5));
}
Eigen::Vector3i sample_base::coord2gridIndex(const Eigen::Vector3d &pt)//将三维坐标点转换成三维地图上一点
{
    Eigen::Vector3i idx;
    idx <<  std::min( std::max( int( (pt[0] - param_.map_lower[0]) * param_.inv_resolution), 0), param_.max_x_id - 1),
            std::min( std::max( int( (pt[1] - param_.map_lower[1]) * param_.inv_resolution), 0), param_.max_y_id - 1),
            std::min( std::max( int( (pt[2] - param_.map_lower[2]) * param_.inv_resolution), 0), param_.max_z_id - 1);
    return idx;
}

Eigen::Vector3d sample_base::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index[0] + 0.5) * param_.resolution + param_.map_lower[0];
    pt(1) = ((double)index[1] + 0.5) * param_.resolution + param_.map_lower[1];
    pt(2) = ((double)index[2] + 0.5) * param_.resolution + param_.map_lower[2];

    return pt;
}

void sample_base::setTarget(Eigen::Vector3d target)
{
    targetPoint_ = gridIndex2coord(coord2gridIndex(target));
    has_target_ = true;
}

