# motionPlan
实现A*、JPS、RRT*等路径搜索算法，并在ROS下完成可视化操作

将path_finding random_map rviz_plugins 三个包放到ROS工作空间中编译即可
path_finding 中为路径搜索算法主函数，所有的路径算法均继承来在path_finding基类，从而统一接口

random_map 用于自动生成随机3d地图
rviz_plugins 为构建的3d目标位置。
3d nav goal 用法在于鼠标左键点击地图确定二维位置，然后按住右键，确定z轴位置。

运行：roslaunch random_map map.launch 打开rviz，并生成地图
运行：path_finding 即可，选定目标点后自动采用RRT*和A*两种算法自动规划路径。
（其中， path_finding包默认参数和launch文件默认参数一致，不修改地图参数的情况下可在IDE中Debug调试）

编译依赖：ROS
	Eigen3
	PCL
	OMPL
![avatar](https://github.com/linyicheng1/motionPlan/tree/master/src/random_map/path.png)
