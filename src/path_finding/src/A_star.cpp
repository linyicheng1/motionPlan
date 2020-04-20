#include "A_star.h"

/**
 * @biref 初始化地图及地图参数，设置起点为(0,0,0)
 * @param param
 * */
a_star::a_star(mapParam param)
{
    has_map_ = false;
    has_target_ = false;

    setMapParam(param);

    setStart(Eigen::Vector3d(0,0,0));
    resetMap();
}

void a_star::findPath()
{
    if(has_map_&&has_target_)//先看懂这些，再迁移
    {//得到地图信息并且目标点位置更新了，开始搜索路径
        has_target_ = false;
        std::cout<<"/**************A Star path finding begin ***********/"<<std::endl;
        openSet.clear();
        path_.clear();
        resetMap();
        GridNodePtr currentPtr = NULL;
        GridNodePtr neighborPtr = NULL;

        startNodePtr->gScore = 0;
        startNodePtr->fScore = getHeu(startNodePtr,targetNodePtr);
        startNodePtr->nodeMapIt = openSet.insert(std::make_pair(startNodePtr->fScore,startNodePtr));

        std::vector<GridNodePtr> neighborsPtr;
        std::vector<double> edgeCost;
        while(!openSet.empty())
        {
            currentPtr = openSet.begin()->second;
            openSet.erase(currentPtr->nodeMapIt);
            currentPtr->id = 1;
            if(currentPtr->index == target_id_)
            {
                delete targetNodePtr;
                targetNodePtr = currentPtr;
                calPath();
                setStart(currentPtr->coord);
                std::cout<<"A* has find the target point!"<<std::endl;
                return;
            }
            getNeighborPt(currentPtr,neighborsPtr,edgeCost);
            for(int i=0;i<neighborsPtr.size();i++)
            {
                neighborPtr = neighborsPtr[i];
                if(neighborPtr->id == 0)
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCost[i];
                    neighborPtr->fScore = getHeu(neighborPtr,targetNodePtr);
                    neighborPtr->dir = currentPtr->index;
                    neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->fScore+neighborPtr->gScore,neighborPtr));
                    neighborPtr->id = -1;
                }
                else if(neighborPtr->id == -1)
                {
                    double tmp_gScore = currentPtr->gScore + edgeCost[i];
                    if(tmp_gScore<neighborPtr->gScore)
                    {
                        openSet.erase(neighborPtr->nodeMapIt);
                        neighborPtr->gScore = tmp_gScore;
                        neighborPtr->dir = currentPtr->index;
                        neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->fScore+neighborPtr->gScore,neighborPtr));
                    }
                }
                else
                {
                    continue;
                }
            }
        }

    }
    else
    {
        return;
    }
}
void a_star::setMap(pcl::PointCloud<pcl::PointXYZ> cloud)
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
        int id_x = static_cast<int>((cloud.points[i].x-param_.map_lower[0])*param_.inv_resolution);
        int id_y = static_cast<int>((cloud.points[i].y-param_.map_lower[1])*param_.inv_resolution);
        int id_z = static_cast<int>((cloud.points[i].z-param_.map_lower[2])*param_.inv_resolution);

        int get = id_x*param_.max_y_id*param_.max_z_id + id_y*param_.max_z_id + id_z;
        obj[get] = uint8_t(1);
    }
}
void a_star::setStart(Eigen::Vector3d start)
{
    start_id_ = coord2gridIndex(start);
    startPoint_ = gridIndex2coord(start_id_);
    startNodePtr = new GridNode(start_id_,startPoint_);
}
void a_star::setTarget(Eigen::Vector3d target)
{
    target_id_ = coord2gridIndex(target);
    targetPoint_ = gridIndex2coord(target_id_);
    targetNodePtr = new GridNode(target_id_,targetPoint_);
    has_target_ = true;
}


Eigen::Vector3i a_star::coord2gridIndex(const Eigen::Vector3d &pt)
{
    Eigen::Vector3i idx;
    idx <<  std::min( std::max( int( (pt[0] - param_.map_lower[0]) * param_.inv_resolution), 0), param_.max_x_id - 1),
            std::min( std::max( int( (pt[1] - param_.map_lower[1]) * param_.inv_resolution), 0), param_.max_y_id - 1),
            std::min( std::max( int( (pt[2] - param_.map_lower[2]) * param_.inv_resolution), 0), param_.max_z_id - 1);
    return idx;
}

Eigen::Vector3d a_star::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index[0] + 0.5) * param_.resolution + param_.map_lower[0];
    pt(1) = ((double)index[1] + 0.5) * param_.resolution + param_.map_lower[1];
    pt(2) = ((double)index[2] + 0.5) * param_.resolution + param_.map_lower[2];

    return pt;
}

void a_star::getNeighborPt(GridNodePtr currentPt, std::vector<GridNodePtr> &neighborPtr, std::vector<double> &cost)
{
    neighborPtr.clear();
    cost.clear();
    Eigen::Vector3i neighborId[26] = {
            Eigen::Vector3i(0,0,1),Eigen::Vector3i(0,0,-1),
            Eigen::Vector3i(0,1,0),Eigen::Vector3i(0,1,1),Eigen::Vector3i(0,1,-1),
            Eigen::Vector3i(0,-1,0),Eigen::Vector3i(0,-1,1),Eigen::Vector3i(0,-1,-1),

            Eigen::Vector3i(1,0,0),Eigen::Vector3i(1,0,1),Eigen::Vector3i(1,0,-1),
            Eigen::Vector3i(1,1,0),Eigen::Vector3i(1,1,1),Eigen::Vector3i(1,1,-1),
            Eigen::Vector3i(1,-1,0),Eigen::Vector3i(1,-1,1),Eigen::Vector3i(1,-1,-1),

            Eigen::Vector3i(-1,0,0),Eigen::Vector3i(-1,0,1),Eigen::Vector3i(-1,0,-1),
            Eigen::Vector3i(-1,1,0),Eigen::Vector3i(-1,1,1),Eigen::Vector3i(-1,1,-1),
            Eigen::Vector3i(-1,-1,0),Eigen::Vector3i(-1,-1,1),Eigen::Vector3i(-1,-1,-1)
    };
    for(int i=0;i<26;i++)
    {
        Eigen::Vector3i pt = currentPt->index ;
        pt += neighborId[i];
        //避免搜索到地图范围外
        if(pt[0]<0||pt[1]<0||pt[2]<0||
           pt[0]>=param_.max_x_id||pt[1]>=param_.max_y_id||pt[2]>=param_.max_z_id)
        {
            continue;
        }
        if(obj[pt[0]*param_.max_y_id*param_.max_z_id + pt[1]*param_.max_z_id+pt[2]]==1)
        {
            continue;
        }
        neighborPtr.push_back(GridNodeMap[pt[0]][pt[1]][pt[2]]);
        double tmp = sqrt(neighborId[i][0]*neighborId[i][0] + neighborId[i][1]*neighborId[i][1] + neighborId[i][2]*neighborId[i][2])*param_.resolution;
        cost.push_back(tmp);
    }
}

std::vector<Eigen::Vector3d> a_star::calPath()
{
    std::vector<GridNodePtr> gridPath;
    gridPath.push_back(targetNodePtr);
    GridNodePtr nowPtr(targetNodePtr);
    while(true)
    {
        nowPtr = GridNodeMap[nowPtr->dir[0]][nowPtr->dir[1]][nowPtr->dir[2]];
        if(nowPtr->index == start_id_)
        {
            gridPath.push_back(nowPtr);
            break;
        }
        else
        {
            gridPath.push_back(nowPtr);
        }
    }
    for(auto ptr:gridPath)
    {
        path_.push_back(ptr->coord);
    }
    std::reverse(path_.begin(),path_.end());
    return path_;
}

double a_star::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    double tempDistance[3];
    double length = 0;
    tempDistance[0] = fabsf(node1->coord[0] - node2->coord[0]);
    tempDistance[1] = fabsf(node1->coord[1] - node2->coord[1]);
    tempDistance[2] = fabsf(node1->coord[2] - node2->coord[2]);
    double min = tempDistance[0];
    int Index[2];
    for(int i=1;i<3;i++)
    {
        if(min>tempDistance[i])
        {
            min = tempDistance[i];
            Index[i-1] = 0;
        }
        else
        {
            Index[i-1] = i;
        }
    }
    length += sqrt(3)*min;
    double min2 = tempDistance[Index[0]];
    if(min2>tempDistance[Index[1]])
    {
        min2 = tempDistance[Index[1]];
    }
    length += sqrt(2)*(min2-min);
    length += fabsf(tempDistance[Index[1]]-tempDistance[Index[0]]);
    return length;
}

void a_star::resetMap()
{
    GridNodeMap = new GridNodePtr **[param_.max_x_id];
    for(int i=0;i<param_.max_x_id;i++)
    {
        GridNodeMap[i] = new GridNodePtr *[param_.max_y_id];
        for(int j=0;j<param_.max_y_id;j++)
        {
            GridNodeMap[i][j] = new GridNodePtr [param_.max_z_id];
            for(int k = 0;k<param_.max_z_id;k++)
            {
                Eigen::Vector3i tmpId(i,j,k);
                Eigen::Vector3d pos = gridIndex2coord(tmpId);
                GridNodeMap[i][j][k] = new GridNode(tmpId,pos);
            }
        }
    }
}

