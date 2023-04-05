/********************************************
 * Class: REGIONGROWINGWITHJAYA
 * Author: Dayu
 * Site: WHU
 * Date: 2020504
 * email: dayuyu@whu.edu.cn
********************************************/
#ifndef REGIONGROWINGWITHJAYA_H
#define REGIONGROWINGWITHJAYA_H

#include "regionGrowing4Points.h"
#include "jaya/ObjectedFunc.h"

/**
* 该类有可能有较大内存占用，可以通过指针初始化，使用后delete
* 或者实例初始化，在一个代码块中，代码块结束自动释放
* usage: 1. 设置相关参数
*        2. regionGrowing
*        3. 析构
* Domo：
*     {
		ResionGrowing4Points regionG(triangle_path);
		regionG.regionGrowing();
		regionG.savePointsOfSupCenterByRG(triangle_path);
		regionG.saveSuperVoxelByRG(triangle_path);
	   }
*/
class ResionGrowingWithJay : public ObjectedFuncBase, public ResionGrowing4Points
{

public:
	ResionGrowingWithJay(
		std::vector<Triangle>* vTriangle, //const 
		std::vector<std::vector<Vertex>>* vvVertex,//const 
		std::vector<int>* vSupervoxelIndex //const 
	);
	~ResionGrowingWithJay();

	double getValue(std::vector<double> solution);

	/**
	* 计算平面检测后每个平面的oriented bounding box
	* 方式：使用CGAL
	* 时间：394个平面，耗时17.3秒
	*/
	void calOBB4Plane_byCGAL();

	/**
	* 计算平面检测后每个平面的oriented bounding box
	* 仿照PCL中的OBB计算
	* 时间：394个平面，耗时0.88秒
	*/
	void calOBB4Plane_byPCL(std::vector<std::vector<size_t>>& outSortedPlan_tris,
							std::vector<float>& vMaxArea, std::vector<float>& vTriArea);

	void calPlaneCofficient(std::vector<std::vector<size_t>>& outSortedPlan_tris, 
							std::vector<float>& vPlaneCoffcients) const;

private:

};

#endif //REGIONGROWINGWITHJAYA

