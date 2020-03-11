#ifndef __SAMPLE_BASE_
#define __SAMPLE_BASE_

#include "path_finding.h"
#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
enum optimalPlanner
{
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};
class sample_base:public path_finding
{
public:
    mapParam param_;
    uint8_t * obj;
    sample_base(mapParam param,int type);
    ~sample_base(){}
    void setMap(pcl::PointCloud<pcl::PointXYZ> cloud);//设置地图
    void setTarget(Eigen::Vector3d target);
    void findPath();//开始搜索路径
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
private:
    int plannerType_;


   // ompl::base::PlannerStatus solved_;

};

#endif //__SAMPLE_BASE_
