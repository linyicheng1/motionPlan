// WolfPackAlgorithm.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "WPA.h"
#include <iostream>

void get_Func_detail(int prob_Ind, double& max_Val, double& min_Val, int& dimention, double& ans);

//存在的问题：不够random

int main()
{
	//test.process(4, 5, 10, -10);
	double min_Val, max_Val, ans;
	int num, dim;
	int j = 10;
	while (j--)
	{
		cout << endl;
		for (int i =7; i < 8; i++)
		{
			WPA test;
			double result;
			get_Func_detail(i, max_Val, min_Val, dim, ans);
			test.process(200, dim, max_Val, min_Val, i, 100, 100, result);//scout_Num 20
			std::cout << "standard result:" << ans << "		cal_result:" << result << "\n";
		}
	}
	cout << "Done\n";
	return 1;
}

void get_Func_detail(int prob_Ind,double &max_Val,double &min_Val,int &dimention,double &ans)
{
	switch (prob_Ind)
	{
	case 0://Easom函数
		max_Val = 10.0;
		min_Val = -10.0;
		dimention = 2;
		ans = -1;
		break;
	case 1://Matyas
		max_Val = 10.0;
		min_Val = -10.0;
		dimention = 2;
		ans = 0;
		break;
	case 2://Rosenbrock
		max_Val = 2.048;
		min_Val = -2.048;
		dimention = 2;
		ans = 0;
		break;
	case 3://Colville
		max_Val = 10.0;
		min_Val = -10.0;
		dimention = 4;
		ans = 0;
		break;
	case 4://Trid6
		max_Val = 36.0;
		min_Val = -36.0;
		dimention = 6;
		ans = -50;
		break;
	case 5://Stepint    seems error
		max_Val = 5.12;
		min_Val = -5.12;
		dimention = 5;
		ans = 0;
		break;
	case 6://Step       seems error
		max_Val = 100.0;
		min_Val = -100.0;
		dimention = 30;
		ans = 0;
		break;
	case 7://Sumsquares     seems error
		max_Val = 10.0;
		min_Val = -10.0;
		dimention = 150;
		ans = 0;
		break;
	case 8://Sphere
		max_Val = 1.5;
		min_Val = -1.5;
		dimention = 200;
		ans = 0;
		break;
	case 9://Booth
		max_Val = 10.0;
		min_Val = -10.0;
		dimention = 2;
		ans = 0;
		break;
	case 10://Bohachevskyl
		max_Val = 100.0;
		min_Val = -100.0;
		dimention = 2;
		ans = 0;
		break;
	case 11://Rastrigin
		max_Val = 5.12;
		min_Val = -5.12;
		dimention = 30;
		ans = 0;
		break;
	case 12://Quadric
		max_Val = 30.0;
		min_Val = -30.0;
		dimention = 100;
		ans = 0;
		break;
	case 13://Eggcrate
		max_Val = 2*SELF_PI;
		min_Val = -2 * SELF_PI;
		dimention = 2;
		ans = 0;
		break;
	case 14://Schaffer
		max_Val = 100.0;
		min_Val = -100.0;
		dimention = 2;
		ans = 0;
		break;
	case 15://Six Hump Camel Back
		max_Val = 5.0;
		min_Val = -5.0;
		dimention = 2;
		ans = -1.0316;
		break;
	default:
		break;
	}
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
