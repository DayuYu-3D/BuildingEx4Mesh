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
* �����п����нϴ��ڴ�ռ�ã�����ͨ��ָ���ʼ����ʹ�ú�delete
* ����ʵ����ʼ������һ��������У����������Զ��ͷ�
* usage: 1. ������ز���
*        2. regionGrowing
*        3. ����
* Domo��
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
	* ����ƽ�����ÿ��ƽ���oriented bounding box
	* ��ʽ��ʹ��CGAL
	* ʱ�䣺394��ƽ�棬��ʱ17.3��
	*/
	void calOBB4Plane_byCGAL();

	/**
	* ����ƽ�����ÿ��ƽ���oriented bounding box
	* ����PCL�е�OBB����
	* ʱ�䣺394��ƽ�棬��ʱ0.88��
	*/
	void calOBB4Plane_byPCL(std::vector<std::vector<size_t>>& outSortedPlan_tris,
							std::vector<float>& vMaxArea, std::vector<float>& vTriArea);

	void calPlaneCofficient(std::vector<std::vector<size_t>>& outSortedPlan_tris, 
							std::vector<float>& vPlaneCoffcients) const;

private:

};

#endif //REGIONGROWINGWITHJAYA

