/********************************************
 * Class: BuildingExtract
 * Author: Dayu
 * Site: WHU
 * Date: 2020528
 * email: dayuyu@whu.edu.cn
********************************************/
#pragma once

#include <vector>

#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/vertex.h"

/**
* 该类用户提取建筑物
* usage: 1. 初始化:输入tirangle和vertes
*        2. setPlanerTriangles 或者 setPlanerSupervoxels
*        3. do something, such as filteringSmallePlaneByNums, etc.
*        4. 析构
* notice: 可以输入Supervoxel平面或者Triangle平面，输入的什么平面，返回的结果就是什么平面
*/
class BuildingExtract
{
public:
	BuildingExtract(const std::vector<Triangle>& vTris, const std::vector<std::vector<Vertex>>& vvVertexs);
	~BuildingExtract();

	void setPlanerTriangles(const std::vector<std::vector<size_t>>* pvTris);
	void setPlanerSupervoxels(const std::vector<std::vector<size_t>>* pvSuperv, std::vector<int>* vSupervoxelIndex);
	//设置triangle的texture，可选项：使用颜色特征过滤是需要设定
	void setTexture(std::vector<std::string> vTexture);
	/**
	* 根据三角形/超体素数量，简单筛选掉小于等于nums的平面。
	* vvIxs_svORtri_out: 筛选后剩余的超体素/三角形的索引
	* minNums: 例如30对于超体素，5000对于三角形
	*/
	void filteringSmallePlaneByNums(std::vector<std::vector<size_t>>& vvIxs_svORtri_out, int minNums) const;
	/**
	* 根据三角形/超体素数量，简单筛选掉小于等于minArea的平面。
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	*/
	void filteringSmallePlaneByArea(std::vector<std::vector<size_t>>& vvIxs_tri_out, float minArea) const;

	/**
	* 根据三角形/超体素数量，简单筛选掉小于等于minArea的平面。
	* vvIxs_tri_in: 平面三角形的索引
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	*/
	void filteringSmallePlaneByArea(
		const std::vector<std::vector<size_t>>& vvIxs_tri_in, 
		std::vector<std::vector<size_t>>& vvIxs_tri_out, float minArea) const;

	/**
	* 根据一个平面的面积与投影的面积的比值，简单筛选平面。
	* vvIxs_sv_in: const 输入的平面超体素的索引
	* vIxs_tri_out: 筛选后剩余的三角形的索引
	* minRatio: 最小的比值，大于该值则剔除
	* maxRatio: 最大的比值，小于该值则剔除
	*/
	void filteringPlaneByAreaRatio_superV(const std::vector<std::vector<size_t>>& vvIxs_sv_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float minRatio, float maxRatio) const;
	/**
	* 根据一个平面的面积与投影的面积的比值，简单筛选平面。
	* vvIxs_tri_in: const 输入的平面三角形的索引
	* vIxs_tri_out: 筛选后剩余的三角形的索引
	* minRatio: 最小的比值，大于该值则剔除
	* maxRatio: 最大的比值，小于该值则剔除
	*/
	void filteringPlaneByAreaRatio_tris(const std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float minRatio, float maxRatio) const;

	/**
	* 使用PCA对平面进行二次拟合，拟合会得到一个拟合系数，取值为0-1，根据这个系数进行过滤。
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	* 
	*/
	void filteringPlaneByPCA(std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float minCoeffi) const;

	/**
	* 使用颜色刷选平面，主要目的是去除绿色树木平面：使用颜色距离差异方法
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	* vvIxs_tri_in: 输入三角形索引
	* maxGreenDistance: 与绿色的颜色距离差异程度
	*/
	void filteringPlaneByColor(std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float maxGreenDistance) const;
	/**
	* 使用颜色刷选平面，主要目的是去除绿色树木平面：使用 ExG － ExR指数
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	* vvIxs_tri_in: 输入三角形索引
	* maxGreenDistance: 与绿色的颜色距离差异程度
	*/
	void filteringPlaneByExG_EXR(std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float maxGreenDistance) const;

	/**
	* 使用相对地面距离刷选平面，主要目的是低矮植被、车辆等
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	* vvIxs_tri_in: 输入三角形索引
	* thresholdHeight: 阈值，相对地面距离   
	* AGL: Above ground level
	* 详述：此版本为第一版本，使用地面的顶点搜索knn，速度慢
	*       使用的最大高度AGLH作为阈值，可能有飞点，存在小部分低矮地物未被滤除干净
	*/
	void filteringPlaneByAGL_Height_1(
		const std::vector<Triangle>& vTriangle_ground,
		std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float thresholdHeight) const;

	/**
	* 使用相对地面距离刷选平面，主要目的是低矮植被、车辆等
	* vvIxs_tri_out: 筛选后剩余的三角形的索引
	* vvIxs_tri_in: 输入三角形索引
	* thresholdHeight: 阈值，相对地面距离
	* AGL: Above ground leve2
	* 详述：此版本为第二版本，使用地面的上小心的中心点搜索knn，速度快
	*       使用的使用面积加权AGLH作为阈值
	*/
	void filteringPlaneByAGL_Height_2(
		const std::vector<Triangle>& vTriangle_ground,
		const std::vector<std::vector<Vertex>>& vvVertexs_ground,
		std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float thresholdHeight) const;


protected:
	bool paramsValidate() const;

	/**
	* 将Supervoxel平面转为triangle平面
	* vvIxs_Sv_in 输入的平Supervoxel平面
	* vvIxs_tri_out: 输出的triangle平面
	*/
	void convertSvPlanes2TriPlanes(const std::vector<std::vector<size_t>>& vvIxs_Sv_in, 
		std::vector<std::vector<size_t>>& vvIxs_tri_out) const;

	/**
	* 计算所有三角形的三维面积之和
	*/
	void calTotalArea_3D_triangles(double& totalArea) const;

	/**
	* 计算三角形平面的三维面积
	* vvIxs_tri_in: 筛选后剩余的三角形的索引
	* vArea_out: 计算的每个平面的三维面积
	*/
	void calArea2Planes_3D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vArea_out) const;
	/**
	* 计算三角形平面的2维面积。
	* vvIxs_tri_in: 筛选后剩余的三角形的索引
	* vArea_out: 计算的每个平面的2维面积
	*/
	void calArea2Planes_2D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vArea_out) const;
	/**
	* 计算三角形平面的三维面积权重
	* vvIxs_tri_in: 筛选后剩余的三角形的索引
	* vArea_out: 计算的每个平面的面积占所有平面面积的权重
	*/
	void calAreaRatio2Planes_3D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vAreaRatio_out) const;

private:
	const std::vector<Triangle>& _vTris;
	const std::vector<std::vector<Vertex>>& _vvVertexs;
	std::vector<std::string> _vTextures;

	const std::vector<std::vector<size_t>>* _pvvPlane_Tris;
	const std::vector<std::vector<size_t>>* _pvvPlane_SuperVoxel;
	std::vector<std::vector<int>> _vvSupervoxelIndex;   //每个超体素含有的面的索引
};
