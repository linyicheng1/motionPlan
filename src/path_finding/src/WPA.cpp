#include "WPA.h"


//�Ȱ�����ܹ�����
/**
 * @biref ��ʼ����ͼ����ͼ�������������Ϊ(0,0,0)
 * @param param
 * */
WPA::WPA(mapParam param)
{
    has_map_ = false;
    has_target_ = false;

    setMapParam(param);
    punish_Val=param_.map_upper[0]-param_.map_lower[0];
    setStart(Eigen::Vector3d(0,0,0));

    //resetMap();
}

WPA::~WPA()
{

}

void WPA::findPath()
{
    if(has_map_&&has_target_)
    {
        has_target_=false;
        path_.clear();
        std::cout<<"/**************WPA path finding begin ***********/"<<std::endl;
        vector<MatrixXd>().swap(wolves);
        vector<int>().swap(head_Wolf);
        vector<double>().swap(wolf_Fit);
        head_Wolf.clear();
        wolves.clear();
        wolf_Fit.clear();
        srand((int)time(NULL));
        rand();
        wolf_Num = 50;//50匹狼
        wolf_Dim = 10;
        max_Val_x = param_.map_upper[0];
        max_Val_y=param_.map_upper[1];
        max_Val_z=param_.map_upper[2];
        min_Val_x=param_.map_lower[0];
        min_Val_y=param_.map_lower[1];
        min_Val_z=param_.map_lower[2];
        prob_Ind = 13;
        scout_Times = 30;//最大游走次数
        scout_Step = 0.01*max_Val_x;//游走的步长,这里不同维度下步长因子相同  0.05
        rush_Step = 0.05*max_Val_x;//奔袭时的步长    0.1
        atk_Step = 0.025*max_Val_x;//围攻时的步长   0.025
        d_Near = 0.08*max_Val_x;//由奔袭转入围攻的判定阈值，这里参考matlab代码用定值，而没有按论文来计算  0.08
        int dir_Factor = 10;//方向因子  10
        double beita = 5;//群体更新比例因子 5
        srand((int)time(NULL));
        rand();
        param_H = dir_Factor + rand() % dir_Factor;//������Ϊ[dir_Factor,2*dir_Factor)֮�������
        param_R = (1.0 + rand() / (double)RAND_MAX) * wolf_Num * 0.5 / beita;
        init_Wolf();//初始化种群
        pick_Head();//选择头狼
        int gen_Num=20;
        for(int i=0;i<gen_Num;i++)
        {
            ac_Wolfscout();//ac=action 游走行为
            ac_Summon();//召唤和围攻行为
            ac_Quarryment();//群体更新，强者生存
        }
        std::cout<<"WPA finish searching\n";
        std::cout<<"WPA value:"<<lead_Val<<"\n";
        //还要加一环，由头狼转换成路径表达
        wolf2path();
        setStart(targetPoint_);
    }
}

void WPA::setMap(pcl::PointCloud<pcl::PointXYZ> cloud)//��άתһά
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


void WPA::setStart(Eigen::Vector3d start)
{
    start_id_ = coord2gridIndex(start);
    startPoint_ = gridIndex2coord(start_id_);
}

void WPA::setTarget(Eigen::Vector3d target)
{
    target_id_ = coord2gridIndex(target);
    targetPoint_ = gridIndex2coord(target_id_);
    has_target_ = true;
}

void WPA::wolf2path()
{
    for(int i=0;i<wolf_Dim;i++)
    {
        path_.push_back(Eigen::Vector3d(wolves[head_Wolf[0]](0,i),wolves[head_Wolf[0]](1,i),wolves[head_Wolf[0]](2,i)));
    }
}

double WPA::Road_assess(MatrixXd wolf)
{//由于迭代的原因，实际上每个节点也不一定是整数，所以评估的时候，首先将两个相邻节点转换成离散的整数点，再用bresenham构造离散路径
    double road_Val=0;
    for(int i=0;i<(wolf_Dim-1);i++)
    {
        vector<Vector3i> pts;
        Eigen::Vector3i st_Voxel,end_Voxel;
        st_Voxel=coord2gridIndex(Eigen::Vector3d(wolf(0,i),wolf(1,i),wolf(2,i)));
        end_Voxel=coord2gridIndex(Eigen::Vector3d(wolf(0,i+1),wolf(1,i+1),wolf(2,i+1)));
        if(st_Voxel[0]==end_Voxel[0] && st_Voxel[1]==end_Voxel[1] && st_Voxel[2]==end_Voxel[2])
            continue;
        line2voxel(st_Voxel,end_Voxel,pts);
        for(int j=0;j<pts.size();j++)
        {
            Eigen::Vector3i id=pts[j];
            if(id[0] >= 0 && id[0] < param_.max_x_id && id[1] >= 0 && id[1] < param_.max_y_id && id[2] >= 0 && id[2] < param_.max_z_id &&
               (obj[id[0]*param_.max_y_id*param_.max_z_id + id[1]*param_.max_z_id + id[2]] < 0.5))
                road_Val+=1;
            else
                road_Val+=punish_Val;
        }
    }
}

void line2voxel(Vector3i A,Vector3i B,vector<Vector3i> &pts)//������������ɵ�ֱ��ת����һ�����ϵ�ͼ�������С����
{//untested
    if(B[2]==A[2])//��άbresenham
    {
        int dx=abs(B[0]-B[0]);
        int dy=abs(B[1]-A[1]);
        int x=A[0];
        int y=B[0];
        int sign_X=B[0]>A[0]?1:-1;
        int sign_Y=B[1]>A[1]?1:-1;
        if(dx>dy)
        {
            int e=-dx;
            for(int i=0;i<dx;i++)
            {
                x+=sign_X;
                e+=2*dy;
                if(e>=0)
                {
                    y+=sign_Y;
                    e-=2*dx;
                }
                pts.push_back(Vector3i(x,y,A[2]));
            }
        }
        else
        {
            int e=-dy;
            for(int i=0;i<dy;i++)
            {
                y+=sign_Y;
                e+=2*dx;
                if(e>=0)
                {
                    x+=sign_X;
                    e-=2*dy;
                }
                pts.push_back(Vector3i(x,y,A[2]));
            }
        }
    }
    else//��άbresenham
    {
        int x=A[0];
        int y=A[1];
        int z=A[2];
        int dx=abs(B[0]-A[0]);
        int dy=abs(B[1]-A[1]);
        int dz=abs(B[2]-A[2]);
        int sign_X=B[0]>A[0]?1:-1;
        int sign_Y=B[1]>A[1]?1:-1;
        int sign_Z=B[2]>A[2]?1:-1;
        if(dx>=dy && dx>=dz)//dx >= dy&dz
        {
            int e1=2*dy-dx;
            int e2=2*dz-dx;
            for(int i=0;i<dx;i++)
            {
                x+=sign_X;
                if(e1<0)
                    e1+=2*dy;
                else
                {
                    y+=sign_Y;
                    e1=e1+2*(dy-dx);
                }
                if(e2<0)
                    e2+=2*dz;
                else
                {
                    z+=sign_Z;
                    e2=e2+2*(dz-dx);
                }
                pts.push_back(Vector3i(x,y,z));
            }
        }
        else if(dy >=dx && dy>=dz)
        {
            int e1=2*dz-dy;
            int e2=2*dx-dy;
            for(int i=0;i<dy;i++)
            {
                y+=sign_Y;
                if(e1<0)
                    e1+=2*dz;
                else
                {
                    z+=sign_Z;
                    e1=e1+2*(dz-dy);
                }
                if(e2<0)
                    e2+=2*dx;
                else
                {
                    x+=sign_X;
                    e2=e2+2*(dx-dy);
                }
                pts.push_back(Vector3i(x,y,z));
            }
        }
        else    //dz >= dx&dy
        {
            int e1=2*dx-dz;
            int e2=2*dy-dz;
            for(int i=0;i<dy;i++)
            {
                z+=sign_Z;
                if(e1<0)
                    e1+=2*dx;
                else
                {
                    x+=sign_X;
                    e1=e1+2*(dx-dz);
                }
                if(e2<0)
                    e2+=2*dy;
                else
                {
                    y+=sign_Y;
                    e2=e2+2*(dy-dz);
                }
                pts.push_back(Vector3i(x,y,z));
            }
        }
    }
}

Eigen::Vector3i WPA::coord2gridIndex(const Eigen::Vector3d &pt)
{
    Eigen::Vector3i idx;
    idx <<  std::min( std::max( int( (pt[0] - param_.map_lower[0]) * param_.inv_resolution), 0), param_.max_x_id - 1),
            std::min( std::max( int( (pt[1] - param_.map_lower[1]) * param_.inv_resolution), 0), param_.max_y_id - 1),
            std::min( std::max( int( (pt[2] - param_.map_lower[2]) * param_.inv_resolution), 0), param_.max_z_id - 1);
    return idx;
}

Eigen::Vector3d WPA::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index[0] + 0.5) * param_.resolution + param_.map_lower[0];
    pt(1) = ((double)index[1] + 0.5) * param_.resolution + param_.map_lower[1];
    pt(2) = ((double)index[2] + 0.5) * param_.resolution + param_.map_lower[2];

    return pt;
}


//params: num:�ǵ����� dim:�ǵ�ά�� max_Bound/min_Bound:ÿһά��Ӧ�����Сֵ������Ĭ�ϲ�ͬά��֮�����һ��
//problem_Type:��ǰ��������Ӧ������ gen_Num:����������
void WPA::process(int num, int dim, double max_Bound, double min_Bound,int problem_Type,int gen_Num,int scout_Num,double &final_Res)
{
    vector<MatrixXd>().swap(wolves);
    vector<int>().swap(head_Wolf);
    vector<double>().swap(wolf_Fit);
    head_Wolf.clear();
	wolves.clear();
	wolf_Fit.clear();
	srand((int)time(NULL));
	rand();
	wolf_Num = num;
	wolf_Dim = dim;
	max_Val = max_Bound;
	min_Val = min_Bound;
	prob_Ind = problem_Type;
	scout_Times = scout_Num;//游走次数
	scout_Step = 0.01*max_Bound;//游走的步长,这里不同维度下步长因子相同  0.05
	rush_Step = 0.05*max_Bound;//奔袭时的步长    0.1
	atk_Step = 0.025*max_Bound;//围攻时的步长   0.025
	d_Near = 0.08*max_Bound;//由奔袭转入围攻的判定阈值，这里参考matlab代码用定值，而没有按论文来计算  0.08
	int dir_Factor = 10;//方向因子  10
	double beita = 5;//群体更新比例因子 5
    srand((int)time(NULL));
    rand();
	param_H = dir_Factor + rand() % dir_Factor;//方向数为[dir_Factor,2*dir_Factor)之间的整数
	param_R = (1.0 + rand() / (double)RAND_MAX) * wolf_Num * 0.5 / beita;
	init_Wolf();//初始化种群
	pick_Head();//选择头狼
	bool flag = false;
	//debug
	/*
	for (int i = 0; i < wolf_Num; i++)
	{
		if(fabs(wolf_Fit[i]-cal_Fitness(wolves[i]))>1e-5)
		    flag=true;
	}
	if (flag)
		cout << "at the beginning,error\n";
	else
		cout << "at the beginning,no error!\n";
	 */
	//vector<double> gen_Result;//�洢ÿһ�ε���������Ž��
	for (int i = 0; i < gen_Num; i++)
	{	
		/*
		for (int ind = 0; ind < wolf_Num; ind++)//������Ⱥ��Ӧ��
			wolf_Fit[ind] = cal_Fitness(wolves[ind]);
		*/
		ac_Wolfscout();//ac=action 游走行为
        std::cout<<"prob ID:"<<prob_Ind<<"\n";
		ac_Summon();//召唤和围攻行为
		ac_Quarryment();//群体更新，强者生存
		//gen_Result.push_back(lead_Val);
		//std::cout << "current generation,the best result:" << lead_Val << endl;
	}
	//std::cout<<wolves[head_Wolf[0]]<<endl;
	//cout<<"head_val:"<<cal_Fitness(wolves[head_Wolf[0]])<<endl;
	final_Res = -1*lead_Val;
}


//这里直接用最简单的随机生成方案
//且默认每一维度的上下限一致
void WPA::init_Wolf()
{
	for(int i=0;i<wolf_Num;i++)
	    wolf_Fit.push_back(0.0);
	MatrixXd wolf_Pack(3*wolf_Num, wolf_Dim);
    //后期如果要修改，直接在这里加
	srand((int)time(NULL));
	rand();
	for (int i = 0; i < wolf_Num; i++)//由于一匹狼代表一条路径，所以狼的第一维和最后一维代表起点和终点，计算过程中始终不改变
	{
	    wolf_Pack(3*i,0)=startPoint_[0];
	    wolf_Pack(3*i+1,0)=startPoint_[1];
	    wolf_Pack(3*i+2,0)=startPoint_[2];
		for (int j = 1; j < (wolf_Dim-1); j++)
        {
		    wolf_Pack(3*i,j)=min_Val_x+(max_Val_x-min_Val_x)*rand()/(double)(RAND_MAX);
		    wolf_Pack(3*i+1,j)=min_Val_y+(max_Val_y-min_Val_y)*rand()/(double)(RAND_MAX);
		    wolf_Pack(3*i+2,j)=min_Val_z+(max_Val_z-min_Val_z)*rand()/(double)(RAND_MAX);
        }
        wolf_Pack(3*i,wolf_Dim-1)=targetPoint_[0];
        wolf_Pack(3*i+1,wolf_Dim-1)=targetPoint_[1];
        wolf_Pack(3*i+2,wolf_Dim-1)=targetPoint_[2];
	}
    //填入狼群
	for (int i = 0; i < wolf_Num; i++)
		wolves.push_back(wolf_Pack.block(3*i, 0, 3, wolf_Dim));
}


void WPA::pick_Head()
{
	head_Wolf.clear();
    //计算狼群适应度并选出头狼
    srand((int)time(NULL));
    rand();
	vector<int> wolf_Candidate;
	double cur_Max = cal_Fitness(wolves[0]);
	for (int i = 0; i < wolf_Num; i++)
	{
		wolf_Fit[i] = cal_Fitness(wolves[i]);
		if (wolf_Fit[i] > cur_Max)
			cur_Max = wolf_Fit[i];
	}
	lead_Val = cur_Max;
	for (int i = 0; i < wolf_Num; i++)
		if (fabs(wolf_Fit[i] - cur_Max) < 1e-6)
			wolf_Candidate.push_back(i);
	int candi_Num = wolf_Candidate.size();
	if (candi_Num == 1)
		head_Wolf.push_back(wolf_Candidate[0]);
	else
	{
        //srand((int)time(NULL));
        //rand();
		int pick_Ind = rand() % candi_Num;
		head_Wolf.push_back(wolf_Candidate[pick_Ind]);
	}
	lead_Val = wolf_Fit[head_Wolf[0]];
}

//由于整体是参考对应的matlab代码编写，这里和原始论文所述机制有一些出入
void WPA::ac_Wolfscout()
{
    bool over_Flag=false;
	int Cnt = 0;
	MatrixXd step_Stan = MatrixXd::Ones(3, wolf_Dim-2);//��β���㲻��
	while (Cnt < scout_Times && !over_Flag)
	{
		for (int i = 0; i < wolf_Num; i++)//这里把所有狼都视作人工探狼,而没有像原始论文一样有一个S_num
		{
			if (wolf_Fit[i] > lead_Val)
			{
				head_Wolf[0] = i;
				lead_Val = wolf_Fit[i];
                over_Flag=true;
                break;
                //感觉可以在这里根据是否有多匹狼大于头狼，转换成多头狼的模式
			}
			else
			{//探狼向h个方向进行搜索
				MatrixXd scout_Try(3*param_H, wolf_Dim);
				double* scout_Res = new double[param_H];
				double cur_Max = wolf_Fit[i]-1;
				for (int j = 0; j < param_H; j++)//朝各方向做出尝试
				{
				    scout_Try.block(3*j,0,3,wolf_Dim)=wolves[i];
					scout_Try.block(3*j, 1, 3, wolf_Dim-2) += scout_Step* sin((double)2.0 * SELF_PI * j / param_H) * step_Stan;
					for(int k=1;k<(wolf_Dim-1);k++)
                    {
					    if(scout_Try(3*j,k)>max_Val_x)
                            scout_Try(3*j,k)=max_Val_x;
					    if(scout_Try(3*j,k)<min_Val_x)
                            scout_Try(3*j,k)=min_Val_x;
					    if(scout_Try(3*j+1,k)>max_Val_y)
                            scout_Try(3*j+1,k)=max_Val_y;
					    if(scout_Try(3*j+1,k)<min_Val_y)
                            scout_Try(3*j+1,k)=min_Val_y;
                        if(scout_Try(3*j+2,k)>max_Val_z)
                            scout_Try(3*j+2,k)=max_Val_z;
                        if(scout_Try(3*j+2,k)<min_Val_z)
                            scout_Try(3*j+2,k)=min_Val_z;
                    }
					scout_Res[j] = cal_Fitness(scout_Try.block(3*j, 0, 3, wolf_Dim));
					if (scout_Res[j] > cur_Max)
						cur_Max = scout_Res[j];
				}
				if (cur_Max <= wolf_Fit[i])//如果各方向均不大于此前的值，则不更新
                {
                    delete[] scout_Res;
                    scout_Res=NULL;
                    continue;
                }
				vector<int> candi_Dir;
				for (int j = 0; j < param_H; j++)
				{
					if (fabs(scout_Res[j] - cur_Max) < 1e-6)
						candi_Dir.push_back(j);
				}
				int candi_Num = candi_Dir.size();
				/*
				if (candi_Num == 0)
					continue;
				*/
				if (candi_Num > 1)//如果有多个最大值，随机选一个
				{
					int pick_Ind = rand() % candi_Num;
					wolves[i] = scout_Try.block(3*candi_Dir[pick_Ind], 0, 3, wolf_Dim);
				}
				else
					wolves[i] = scout_Try.block(3*candi_Dir[0], 0, 3, wolf_Dim);
				wolf_Fit[i] = cur_Max;//更新当前狼对应的适应度
				delete[] scout_Res;
				scout_Res=NULL;
			}
			if (wolf_Fit[i] > lead_Val)
			{
				head_Wolf[0] = i;
				lead_Val = wolf_Fit[i];
                //感觉可以在这里根据是否有多匹狼大于头狼，转换成多头狼的模式
			}
		}
		Cnt++;
	}
}

void WPA::ac_Summon()
{
	for (int i = 0; i < wolf_Num; i++)
	{
		if (i == head_Wolf[0])
			continue;
		double cur_Dist = cal_Dist(wolves[i], wolves[head_Wolf[0]],0);
		if (cur_Dist > d_Near)//奔袭
		{
			for (int j = 1; j < (wolf_Dim-1); j++)
			{
                //论文对应部分等效成这样，即每一维的奔袭步长的绝对值是确定的，而不受与头狼的绝对距离的影响
				if (wolves[i](0, j) > wolves[head_Wolf[0]](0, j))
					wolves[i](0, j) -= rush_Step;
				else
					wolves[i](0, j) += rush_Step;

                if (wolves[i](1, j) > wolves[head_Wolf[0]](1, j))
                    wolves[i](1, j) -= rush_Step;
                else
                    wolves[i](1, j) += rush_Step;

                if (wolves[i](2, j) > wolves[head_Wolf[0]](2, j))
                    wolves[i](2, j) -= rush_Step;
                else
                    wolves[i](2, j) += rush_Step;

				if(wolves[i](0, j)>max_Val_x)
                    wolves[i](0, j)=max_Val_x;
				if(wolves[i](0, j)<min_Val_x)
                    wolves[i](0, j)=min_Val_x;

                if(wolves[i](1, j)>max_Val_y)
                    wolves[i](1, j)=max_Val_y;
                if(wolves[i](1, j)<min_Val_y)
                    wolves[i](1, j)=min_Val_y;

                if(wolves[i](2, j)>max_Val_z)
                    wolves[i](2, j)=max_Val_z;
                if(wolves[i](2, j)<min_Val_z)
                    wolves[i](2, j)=min_Val_z;
			}
			wolf_Fit[i] = cal_Fitness(wolves[i]);
			if (wolf_Fit[i] > lead_Val)//大于头狼浓度则成为新的头狼
			{
				head_Wolf[0] = i;
				lead_Val = wolf_Fit[i];
			}
		}
		else//围攻
		{
			ac_Beleaguer(i);
		}
	}
}


//感觉很多步长都可以由静态变成动态，由迭代次数和距离来决定
void WPA::ac_Beleaguer(int wolf_Ind)//Χ��
{   //untested
    //srand((int)time(NULL));
    //rand();
	double r = 1 - 2.0 * rand() / (double)RAND_MAX;//-1~1֮��������
	for (int i = 1; i < (wolf_Dim-1); i++)
	{//根据当前维度下的距离和之前的攻击距离系数来共同确定步长
        //这里的随机数r可以放外边也可以放里边
		wolves[wolf_Ind](0, i) = wolves[wolf_Ind](0, i) + r * atk_Step * fabs(wolves[head_Wolf[0]](0, i) - wolves[wolf_Ind](0, i));
        wolves[wolf_Ind](1, i) = wolves[wolf_Ind](1, i) + r * atk_Step * fabs(wolves[head_Wolf[0]](1, i) - wolves[wolf_Ind](1, i));
        wolves[wolf_Ind](2, i) = wolves[wolf_Ind](2, i) + r * atk_Step * fabs(wolves[head_Wolf[0]](2, i) - wolves[wolf_Ind](2, i));
	    if(wolves[wolf_Ind](0, i)>max_Val_x)
            wolves[wolf_Ind](0, i)=max_Val_x;
	    if(wolves[wolf_Ind](0, i)<min_Val_x)
            wolves[wolf_Ind](0, i)=min_Val_x;

        if(wolves[wolf_Ind](1, i)>max_Val_y)
            wolves[wolf_Ind](1, i)=max_Val_y;
        if(wolves[wolf_Ind](1, i)<min_Val_y)
            wolves[wolf_Ind](1, i)=min_Val_y;

        if(wolves[wolf_Ind](2, i)>max_Val_z)
            wolves[wolf_Ind](2, i)=max_Val_z;
        if(wolves[wolf_Ind](2, i)<min_Val_z)
            wolves[wolf_Ind](2, i)=min_Val_z;
	}
	wolf_Fit[wolf_Ind] = cal_Fitness(wolves[wolf_Ind]);
	if (wolf_Fit[wolf_Ind] > lead_Val)//如果围攻后的适应度大于头狼，则更新
	{
		lead_Val = wolf_Fit[wolf_Ind];
		head_Wolf[0] = wolf_Ind;
	}
}

void WPA::ac_Quarryment()//wolf update
{
	int* index_Set = new int[wolf_Num];//用于获取排序前后索引的对应关系
	double* new_Fit = new double[wolf_Num];//存入当前的各狼的适应度
	for (int i = 0; i < wolf_Num; i++)
	{
		index_Set[i] = i;
		new_Fit[i] = wolf_Fit[i];
	}
	for (int i = 0; i < (wolf_Num-1); i++)//从小到大排序
	{
		for (int j = i+1; j < wolf_Num; j++)
		{
			if (new_Fit[i] > new_Fit[j])
			{
				double tmp_Fit = new_Fit[i];
				new_Fit[i] = new_Fit[j];
				new_Fit[j] = tmp_Fit;

				int tmp = index_Set[i];
				index_Set[i] = index_Set[j];
				index_Set[j] = tmp;
			}
		}
	}
	vector<int> candi_Index;
	for (int i = (wolf_Num - 1); i >= 0; i--)
	{
		if (fabs(new_Fit[i] - lead_Val) < 1e-6)
			candi_Index.push_back(i);
	}
	int head_Ind;
	int candi_Num = candi_Index.size();
	if (candi_Num > 1)
	{
		int tmp = rand() % candi_Num;
		head_Ind = candi_Index[tmp];
	}
	else
		head_Ind = candi_Index[0];
	head_Wolf[0] = index_Set[head_Ind];
    vector<int>().swap(candi_Index);
	for (int i = 0; i < param_R; i++)
	{
		if (i == head_Ind)
			continue;
		for (int j = 1; j < (wolf_Dim-1); j++)
		{
		    //x
			double new_Val = wolves[head_Wolf[0]](0, j) * (1 + 0.5 * (1 - 2 * rand() / (double)RAND_MAX));
			if (new_Val > max_Val_x)
				new_Val = max_Val_x;
			if (new_Val < min_Val_x)
				new_Val = min_Val_x;
			wolves[index_Set[i]](0, j) = new_Val;
			//y
			new_Val=wolves[head_Wolf[0]](1, j) * (1 + 0.5 * (1 - 2 * rand() / (double)RAND_MAX));
            if (new_Val > max_Val_y)
                new_Val = max_Val_y;
            if (new_Val < min_Val_y)
                new_Val = min_Val_y;
            wolves[index_Set[i]](1, j) = new_Val;
            //z
            new_Val=wolves[head_Wolf[0]](2, j) * (1 + 0.5 * (1 - 2 * rand() / (double)RAND_MAX));
            if (new_Val > max_Val_z)
                new_Val = max_Val_z;
            if (new_Val < min_Val_z)
                new_Val = min_Val_z;
            wolves[index_Set[i]](2, j) = new_Val;
		}
		wolf_Fit[index_Set[i]] = cal_Fitness(wolves[index_Set[i]]);
	}
	delete[] index_Set;
	delete[] new_Fit;
	index_Set=NULL;
	new_Fit=NULL;
}


double WPA::cal_Fitness(MatrixXd wolf)//�����Ӧ�����ǵ���Ӧ��
{
	double fitness = 0;
	float cur,x1,x2;
	MatrixXd standard, tmp, tmp1, tmp2;
	switch (prob_Ind)
	{
	    case 0://Ackley
	    standard=MatrixXd::Ones(1,wolf_Dim);
	    tmp1=wolf.cwiseAbs2();
	    tmp1=tmp1*standard.transpose();
	    for(int i=0;i<wolf_Dim;i++)
        {
	        fitness=fitness+cos(2*SELF_PI*wolf(0,i));
        }
	    fitness/=wolf_Dim;
	    fitness=-20*exp(-0.2*sqrt(tmp1(0,0)/wolf_Dim))-exp(fitness)+20+exp(1);
	    break;

	    case 1://Bukin
        fitness=100*sqrt(fabs(wolf(0,1)-0.01*pow(wolf(0,0),2)))+0.01*fabs(wolf(0,0)+10);
        break;

	    case 2://Cross-in-Tray
        fitness=fabs(sin(wolf(0,0))*sin(wolf(0,1))*exp(fabs(100-sqrt(pow(wolf(0,0),2)+pow(wolf(0,1),2))/SELF_PI)))+1;
        fitness=-0.0001*pow(fitness,0.1);
        break;

	    case 3://Drop-Wave
	    fitness=1+cos(12*sqrt(pow(wolf(0,0),2)+pow(wolf(0,1),2)));
	    fitness=-1*fitness/(0.5*(pow(wolf(0,0),2)+pow(wolf(0,1),2))+2);
        break;

	    case 4://Eggholder
	    fitness=-1*(wolf(0,1)+47)*sin(sqrt(fabs(0.5*wolf(0,0)+wolf(0,1)+47)))-wolf(0,0)*sin(sqrt(fabs(wolf(0,0)-47-wolf(0,1))));
	    break;

        case 5://Griewank
        standard=MatrixXd::Ones(1,wolf_Dim);
        tmp1=wolf.cwiseAbs2();
        tmp1=tmp1*standard.transpose();
        fitness=1;
        for(int i=0;i<wolf_Dim;i++)
        {
            fitness=fitness*cos(wolf(0,i)/sqrt(i+1));
        }
        fitness=tmp1(0,0)/4000-fitness+1;
        break;

	    case 6://Holder Table
	    fitness=-1*fabs(sin(wolf(0,0))*cos(wolf(0,1))*exp(fabs(1-sqrt(pow(wolf(0,0),2)+pow(wolf(0,1),2))/SELF_PI)));
	    break;

	    case 7://Levy
	    cur=1+(wolf(0,wolf_Dim-1)-1)/4;
	    fitness=pow(sin(SELF_PI*(1+(wolf(0,0)-1)/4)),2)+pow(cur-1,2)*(1+pow(sin(2*SELF_PI*cur),2));
        for(int i=0;i<(wolf_Dim-1);i++)
        {
            cur=1+(wolf(0,i)-1)/4;
            fitness=fitness+pow(cur-1,2)*(1+10*pow(sin(SELF_PI*cur+1),2));
        }
	    break;

	    case 8://Levy Function N. 13
	    fitness=pow(sin(3*SELF_PI*wolf(0,0)),2)+pow(wolf(0,0)-1,2)*(1+pow(sin(3*SELF_PI*wolf(0,1)),2))+pow(wolf(0,1)-1,2)*(1+pow(sin(2*SELF_PI*wolf(0,1)),2));
	    break;

	    case 9://Rastrigin
	    fitness=10*wolf_Dim;
	    for(int i=0;i<wolf_Dim;i++)
        {
	        fitness=fitness+(pow(wolf(0,i),2)-10*cos(2*SELF_PI*wolf(0,i)));
        }
	    break;

	    case 10://Schaffer Function N. 2
	    x1=wolf(0,0);
	    x2=wolf(0,1);
	    fitness=0.5+(pow(sin(x1*x1-x2*x2),2)-0.5)/(pow(1+0.001*(x1*x1+x2*x2),2));
	    break;

	    case 11://Schwefel
	    fitness=418.9829*wolf_Dim;
	    for(int i=0;i<wolf_Dim;i++)
	        fitness=fitness-wolf(0,i)*sin(sqrt(fabs(wolf(0,i))));
	    break;

	    case 12://Shubert
        x1=wolf(0,0);
        x2=wolf(0,1);
        cur=0;
        fitness=0;
        for(int i=0;i<5;i++)
        {
            cur=cur+(i+1)*cos((i+2)*x1+i+1);
            fitness=fitness+(i+1)*cos((i+2)*x2+i+1);
        }
        fitness=fitness*cur;
        break;

	    case 13:
        fitness=Road_assess(wolf);
        break;
	    /*
	case 0://Easom
		fitness = -1 * cos(wolf(0, 0)) * cos(wolf(0, 1)) * exp(-1 * pow(wolf(0, 0) - SELF_PI, 2) - pow(wolf(0, 1) - SELF_PI, 2));
		break;
	case 1://Matyas
		fitness = 0.26 * (wolf(0, 0) * wolf(0, 0) + wolf(0, 1) * wolf(0, 1)) - 0.48 * wolf(0, 0) * wolf(0, 1);
		break;
	case 2://Rosenbrock
		standard = MatrixXd::Ones(1, wolf_Dim - 1);
		tmp1 = wolf.block(0, 1, 1, wolf_Dim - 1) - wolf.block(0, 0, 1, wolf_Dim - 1).cwiseAbs2();
		tmp1 = 100 * tmp1.cwiseAbs2();
		tmp2 = (wolf.block(0, 0, 1, wolf_Dim - 1) - standard).cwiseAbs2();
		tmp = (tmp1 + tmp2) * standard.transpose();
		fitness = tmp(0, 0);
		break;
	case 3://Colville
		fitness = 100 * pow(wolf(0, 0) * wolf(0, 0) - wolf(0, 1), 2) + pow(wolf(0, 0) - 1, 2) + pow(wolf(0, 2) - 1, 2) + 90 * pow(wolf(0, 2) * wolf(0, 2) - wolf(0, 3), 2) + 10.1 * (pow(wolf(0, 1) - 1, 2) + pow(wolf(0, 3) - 1, 2)) + 19.8 * (wolf(0, 1) - 1) * (wolf(0, 3) - 1);
		break;
	case 4://Trid6
		standard = MatrixXd::Ones(1, wolf_Dim);
		tmp1 = (wolf - standard).cwiseAbs2();
		tmp2 = wolf.block(0, 1, 1, wolf_Dim - 1)*(wolf.block(0, 0, 1, wolf_Dim - 1)).transpose();
		tmp = tmp1 * standard.transpose() - tmp2;
		fitness = tmp(0, 0);
		break;
	case 5://Stepint
		for (int x = 0; x < wolf_Dim; x++)
			fitness =fitness+ int(wolf(0, x));
		fitness += 30.0;
		break;
	case 6://Step
		for (int x = 0; x < wolf_Dim; x++)
			fitness = fitness + pow(int(wolf(0, x) + 0.5), 2);
		break;
	case 7://Sumsquares
		tmp = wolf.cwiseAbs2();
		for (int x = 0; x < wolf_Dim; x++)
			fitness = fitness + (x + 1) * tmp(0, x);
		break;
	case 8://Sphere
		standard = MatrixXd::Ones(1, wolf_Dim);
		tmp = wolf.cwiseAbs2();
		tmp = tmp * standard.transpose();
		fitness = tmp(0, 0);
		break;
	case 9://Booth
		fitness = pow(wolf(0, 0) + 2 * wolf(0, 1) - 7, 2) + pow(2 * wolf(0, 0) + wolf(0, 1) - 5, 2);
		break;
	case 10://Bohachevskyl
		fitness = pow(wolf(0, 0), 2) + 2 * pow(wolf(0, 1), 2) - 0.3 * cos(3 * SELF_PI * wolf(0, 0)) - 0.4 * cos(4 * SELF_PI * wolf(0, 1)) + 0.7;
		break;
	case 11://Rastrigin
		break;
	     */
	Default:
		break;
	}
	fitness *= -1;//Ҫ����Ӧ��Խ��Խ�ã�������������С�����Գ���-1
	return fitness;
}

double WPA::cal_Dist(MatrixXd wolf_A, MatrixXd wolf_B,int cal_Mode)
{
	double cur_Dist = 0;
	MatrixXd tmp = wolf_A - wolf_B;
	switch (cal_Mode)
	{
	case 0://ŷʽ����
		cur_Dist = tmp.squaredNorm();
		break;
	case 1://BUG
		for (int i = 0; i < wolf_Dim; i++)
			cur_Dist = cur_Dist + fabs(tmp(0, i));
		break;
	default:
		cout << "error distance mode\n";
		break;
	}
	return cur_Dist;
}