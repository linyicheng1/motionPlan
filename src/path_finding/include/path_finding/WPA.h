#ifndef WOLF_PACK_ALGORITHM
#define WOLF_PACK_ALGORITHM
#include "path_finding.h"
#include "sample_base.h"
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include <Eigen/Eigen>
#include <ctime>

#define SELF_PI 3.141592654

using namespace std;
using namespace Eigen;

void line2voxel(Vector3i A,Vector3i B,vector<Vector3i> &pts);

class WPA:public path_finding
{
public:
	WPA(mapParam param);
	~WPA();
	void process(int num, int dim, double max_Bound, double min_Bound, int problem_Type, int gen_Num, int scout_Num,double &final_Res);

	void findPath(void);
    void setMap(pcl::PointCloud<pcl::PointXYZ> cloud);
	void setStart(Eigen::Vector3d start);
    void setTarget(Eigen::Vector3d target);//�����յ�λ��
    void setMapParam(mapParam param){param_ = param;}
    std::vector<Eigen::Vector3d> getPath(){return path_;}//���ع滮�õ�·��

private:
	void init_Wolf();//��Ⱥ��ʼ��
	void pick_Head();//ѡ��ͷ��
	void ac_Wolfscout();//������Ϊ
	void ac_Summon();//�ٻ���Ϊ
	void ac_Beleaguer(int wolf_Ind);//Χ����Ϊ
	void ac_Quarryment();//Ⱥ�����
	double cal_Fitness(MatrixXd wolf);//������Ӧ��
	double cal_Dist(MatrixXd wolf_A, MatrixXd wolf_B, int cal_Mode);//������ƥ�ǵľ���

	vector<MatrixXd> wolves;
	vector<int>head_Wolf;//ͷ�Ƕ�Ӧ�����������ڿ����ж��������������,ĿǰΪ��ͷ��
	int wolf_Num, wolf_Dim;//��Ⱥ������ά��
	int scout_Times;//���ߵ�������
	int prob_Ind;//��ǰ�����Ӧ����
	int param_H;//̽��Ҫ��Ѱ�ķ�����,������������������б���һ����ֵ�����ڿɸ�ɶ�̬��
	int param_R;//ÿ�ε�������Ҫ��̭���ǵ�����
	double max_Val_x,max_Val_y,max_Val_z,min_Val_x,min_Val_y,min_Val_z;
	double max_Val,min_Val;
	//double* wolf_Fit;//��Ⱥ��Ӧ�ȵĶ�̬����ָ��
	vector<double> wolf_Fit;
	double lead_Val;//ͷ�ǵ���Ӧ��
	double scout_Step,rush_Step,atk_Step;//���ߡ���Ϯ������ʱ�Ĳ���
	double d_Near;//�ɱ�Ϯת��Χ���ľ�����ֵ


	double Road_assess(MatrixXd wolf);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    void wolf2path();//将头狼转换成路径以供显示

    //void resetMap(void);//for what?
    mapParam param_;
    std::vector<Eigen::Vector3d> path_;
    uint8_t * obj;
    double punish_Val;//碰撞到障碍物后的惩罚
    Eigen::Vector3i start_id_;
    Eigen::Vector3i target_id_;
};
#endif
