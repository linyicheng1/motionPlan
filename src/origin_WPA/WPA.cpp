#include "WPA.h"

//先把整体架构搭建完成

WPA::WPA()
{

}

WPA::~WPA()
{

}

//params: num:狼的数量 dim:狼的维数 max_Bound/min_Bound:每一维对应最大最小值，这里默认不同维度之间界限一样
//problem_Type:当前解决问题对应的索引 gen_Num:最大迭代次数
void WPA::process(int num, int dim, double max_Bound, double min_Bound,int problem_Type,int gen_Num,int scout_Num,double &final_Res)
{
	head_Wolf.clear();
	wolves.clear();
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
	flag = false;
	vector<double> gen_Result;//存储每一次迭代后的最优结果
	for (int i = 0; i < gen_Num; i++)
	{	
		/*
		for (int ind = 0; ind < wolf_Num; ind++)//计算种群适应度
			wolf_Fit[ind] = cal_Fitness(wolves[ind]);
		*/
		ac_Wolfscout();//ac=action 游走行为
		ac_Summon();//召唤和围攻行为
		ac_Quarryment();//群体更新，强者生存
		gen_Result.push_back(lead_Val);
		//std::cout << "current generation,the best result:" << lead_Val << endl;
	}
	final_Res = -1*lead_Val;
}


//这里直接用最简单的随机生成方案
//且默认每一维度的上下限一致
void WPA::init_Wolf()
{
	for(int i=0;i<wolf_Num;i++)
	    wolf_Fit.push_back(0.0);
	MatrixXd wolf_Pack(wolf_Num, wolf_Dim);
	//后期如果要修改，直接在这里加
	srand((int)time(NULL));
	rand();
	for (int i = 0; i < wolf_Num; i++)
	{
		for (int j = 0; j < wolf_Dim; j++)
			wolf_Pack(i, j) =min_Val +(max_Val-min_Val)*rand() / (double)(RAND_MAX);
	}
	//填入狼群
	for (int i = 0; i < wolf_Num; i++)
		wolves.push_back(wolf_Pack.block(i, 0, 1, wolf_Dim));
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
        srand((int)time(NULL));
        rand();
		int pick_Ind = rand() % candi_Num;
		head_Wolf.push_back(wolf_Candidate[pick_Ind]);
	}
	lead_Val = wolf_Fit[head_Wolf[0]];
}

//由于整体是参考对应的matlab代码编写，这里和原始论文所述机制有一些出入
void WPA::ac_Wolfscout()
{
	int Cnt = 0;
	MatrixXd step_Stan = MatrixXd::Ones(1, wolf_Dim);
	while (Cnt < scout_Times)
	{
	    /*  //debug
	    cout<<"each time:"<<lead_Val<<endl;
	    cout<<"wolf:"<<wolves[head_Wolf[0]]<<endl;
	    cout<<"fitness:"<<cal_Fitness(wolves[head_Wolf[0]])<<endl;
	     */
		for (int i = 0; i < wolf_Num; i++)//这里把所有狼都视作人工探狼,而没有像原始论文一样有一个S_num
		{
			if (wolf_Fit[i] > lead_Val)
			{
				head_Wolf[0] = i;
				lead_Val = wolf_Fit[i];
				//感觉可以在这里根据是否有多匹狼大于头狼，转换成多头狼的模式
			}
			else
			{//探狼向h个方向进行搜索
				MatrixXd scout_Try(param_H, wolf_Dim);
				double* scout_Res = new double[param_H];
				double cur_Max = wolf_Fit[i]-1;
				for (int j = 0; j < param_H; j++)//朝各方向做出尝试
				{
					scout_Try.block(j, 0, 1, wolf_Dim) = wolves[i] + scout_Step* sin((double)2.0 * SELF_PI * j / param_H) * step_Stan;
					scout_Res[j] = cal_Fitness(scout_Try.block(j, 0, 1, wolf_Dim));
					if (scout_Res[j] > cur_Max)
						cur_Max = scout_Res[j];
				}
				if (cur_Max <= wolf_Fit[i])//如果各方向均不大于此前的值，则不更新
					continue;
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
					wolves[i] = scout_Try.block(candi_Dir[pick_Ind], 0, 1, wolf_Dim);
				}
				else
					wolves[i] = scout_Try.block(candi_Dir[0], 0, 1, wolf_Dim);
				wolf_Fit[i] = cur_Max;//更新当前狼对应的适应度
				delete[] scout_Res;
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
			for (int j = 0; j < wolf_Dim; j++)
			{
				//论文对应部分等效成这样，即每一维的奔袭步长的绝对值是确定的，而不受与头狼的绝对距离的影响
				if (wolves[i](0, j) > wolves[head_Wolf[0]](0, j))
					wolves[i](0, j) -= rush_Step;
				else
					wolves[i](0, j) += rush_Step;
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
void WPA::ac_Beleaguer(int wolf_Ind)//围攻
{   //untested
    srand((int)time(NULL));
    rand();
	double r = 1 - 2.0 * rand() / (double)RAND_MAX;//-1~1之间的随机数
	for (int i = 0; i < wolf_Dim; i++)
	{//根据当前维度下的距离和之前的攻击距离系数来共同确定步长
		//这里的随机数r可以放外边也可以放里边
		wolves[wolf_Ind](0, i) = wolves[wolf_Ind](0, i) + r * atk_Step * fabs(wolves[head_Wolf[0]](0, i) - wolves[wolf_Ind](0, i));
	}
	wolf_Fit[wolf_Ind] = cal_Fitness(wolves[wolf_Ind]);
	if (wolf_Fit[wolf_Ind] > lead_Val)//如果围攻后的适应度大于头狼，则更新
	{
		lead_Val = wolf_Fit[wolf_Ind];
		head_Wolf[0] = wolf_Ind;
	}
}

void WPA::ac_Quarryment()
{//seems no use
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
	for (int i = 0; i < param_R; i++)
	{
		if (i == head_Ind)
			continue;
		for (int j = 0; j < wolf_Dim; j++)
		{
			double new_Val = wolves[head_Wolf[0]](0, j) * (1 + 0.5 * (1 - 2 * rand() / (double)RAND_MAX));
			if (new_Val > max_Val)
				new_Val = max_Val;
			if (new_Val < min_Val)
				new_Val = min_Val;
			wolves[index_Set[i]](0, j) = new_Val;
		}
		wolf_Fit[index_Set[i]] = cal_Fitness(wolves[index_Set[i]]);
	}
	delete[] index_Set;
	delete[] new_Fit;
}


double WPA::cal_Fitness(MatrixXd wolf)//计算对应索引狼的适应度
{
	double fitness = 0;
	MatrixXd standard, tmp, tmp1, tmp2;
	switch (prob_Ind)
	{
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
	Default:
		break;
	}
	fitness *= -1;//要求适应度越大越好，而这里是求最小，所以乘上-1
	return fitness;
}

double WPA::cal_Dist(MatrixXd wolf_A, MatrixXd wolf_B,int cal_Mode)
{
	double cur_Dist = 0;
	MatrixXd tmp = wolf_A - wolf_B;
	switch (cal_Mode)
	{
	case 0://欧式距离
		cur_Dist = tmp.squaredNorm();
		break;
	case 1:
		for (int i = 0; i < wolf_Dim; i++)
			cur_Dist = cur_Dist + fabs(tmp(0, i));
		break;
	default:
		cout << "error distance mode\n";
		break;
	}
	return cur_Dist;
}