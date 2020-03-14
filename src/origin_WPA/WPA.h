#pragma once
#ifndef WOLF_PACK_ALGORITHM
#define WOLF_PACK_ALGORITHM
#endif
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include <ctime>

#define SELF_PI 3.1415926

using namespace std;
using namespace Eigen;

class WPA
{
public:
	WPA();
	~WPA();
	void process(int num, int dim, double max_Bound, double min_Bound, int problem_Type, int gen_Num, int scout_Num,double &final_Res);

private:
	void init_Wolf();//狼群初始化
	void pick_Head();//选择头狼
	void ac_Wolfscout();//游走行为
	void ac_Summon();//召唤行为
	void ac_Beleaguer(int wolf_Ind);//围攻行为
	void ac_Quarryment();//群体更新
	double cal_Fitness(MatrixXd wolf);//计算适应度
	double cal_Dist(MatrixXd wolf_A, MatrixXd wolf_B, int cal_Mode);//计算两匹狼的距离

	vector<MatrixXd> wolves;
	vector<int>head_Wolf;//头狼对应的索引，由于可能有多个，所以用容器,目前为单头狼
	int wolf_Num, wolf_Dim;//狼群数量和维数
	int scout_Times;//游走的最大次数
	int prob_Ind;//当前问题对应索引
	int param_H;//探狼要搜寻的方向数,这里在整体迭代过程中保持一个定值，后期可搞成动态的
	int param_R;//每次迭代后需要淘汰的狼的数量
	double max_Val, min_Val;
	//double* wolf_Fit;//狼群适应度的动态数组指针
	vector<double> wolf_Fit;
	double lead_Val;//头狼的适应度
	double scout_Step,rush_Step,atk_Step;//游走、奔袭、攻击时的步长
	double d_Near;//由奔袭转入围攻的距离阈值
};
