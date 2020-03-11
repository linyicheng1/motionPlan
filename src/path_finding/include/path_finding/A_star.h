#ifndef __A_STAR_H
#define __A_STAR_H

#include "path_finding.h"
struct  GridNode;
typedef GridNode* GridNodePtr;
#define inf 1>>20
struct GridNode
{
    int id;
    Eigen::Vector3d coord;//word
    Eigen::Vector3i dir;
    Eigen::Vector3i index;

    double gScore,fScore;
    GridNodePtr cameFrom;
    std::multimap<double ,GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index,Eigen::Vector3d _coord)
    {
        id = 0;
        index = _index;
        coord = _coord;
        dir = Eigen::Vector3i::Zero();

        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
    }
    GridNode(){}
    ~GridNode(){}
};



class a_star:public path_finding
{
public:
    a_star(mapParam param);
    void findPath(void);
    void setMap(pcl::PointCloud<pcl::PointXYZ> cloud);
    void setStart(Eigen::Vector3d start);//设置初始位置
    void setTarget(Eigen::Vector3d target);//设置终点位置

    void setMapParam(mapParam param){param_ = param;}
private:
    Eigen::Vector3i start_id_;
    Eigen::Vector3i target_id_;
    GridNodePtr startNodePtr;
    GridNodePtr targetNodePtr;
    mapParam param_;
    std::multimap<double,GridNodePtr> openSet;
    GridNodePtr *** GridNodeMap;
    uint8_t * obj;
    double getHeu(GridNodePtr node1,GridNodePtr node2);
    void getNeighborPt(GridNodePtr currentPt,std::vector<GridNodePtr> &neighborPtr,std::vector<double> &cost);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    std::vector<Eigen::Vector3d> calPath();
    void resetMap(void);
};

#endif //__A_STAR_H
