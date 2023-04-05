/********************************************
 * Class: ResionGrowing4Points
 * Author: Dayu
 * Site: WHU
 * Date: 2020504
 * email: dayuyu@whu.edu.cn
********************************************/
#ifndef REGIONGROWING4POINTS_H
#define REGIONGROWING4POINTS_H

#include <vector>
#include <string>

#include "PCH.h" //预编译头

#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/vertex.h"

// CGAL includes.
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>

#include "codelibrary/geometry/io/xyz_io.h"
#include "codelibrary/visualization/color/rgb32_color.h"

#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/trianglereader.h"
#include "meshpaser/vertex.h"

#include "topologySearch4Supervoxel.h" 

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
class ResionGrowing4Points
{

	// Type declarations.
	using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
	using FT = typename Kernel::FT;
	using Point_3 = typename Kernel::Point_3;
	using Vector_3 = typename Kernel::Vector_3;

	using Input_range = CGAL::Point_set_3<Point_3>;
	using Point_map = typename Input_range::Point_map;
	using Normal_map = typename Input_range::Vector_map;

	//生命邻居查询方式，only二选一
	///case1：用于点云的KNN邻居查询
	//using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Input_range, Point_map>;
	///case2：根据拓扑关系的邻居查询
	using Neighbor_query = CGAL::Shape_detection::Point_set::topology_neighbor_query_4supervoxels;

	using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, Input_range, Point_map, Normal_map>;
	using Region_growing = CGAL::Shape_detection::Region_growing<Input_range, Neighbor_query, Region_type>;

	using Indices = std::vector<std::size_t>;
	using Output_range = CGAL::Point_set_3<Point_3>;
	using Points_3 = std::vector<Point_3>;


public:
	ResionGrowing4Points(std::string strPath);
	ResionGrowing4Points(
		std::vector<Triangle>* vTriangle, //const 
		std::vector<std::vector<Vertex>>* vvVertex,//const 
		std::vector<int>* vSupervoxelIndex //const 
	);
	~ResionGrowing4Points();

	void setParams(size_t k=12, float maxDistanceToPlane = 2,float maxAcceptAngle = 20, size_t minRegion_size=6);

	/**
	* 区域生长执行函数
	*/
	void regionGrowing();

	/**
	* 保存区域增长后超体素的中心点，有颜色，相同颜色代表一个区域
	* savePath建议为trianglePath,将根据自路径自动重名为_regions_points_color.ply
	*/
	void savePointsOfSupCenterByRG(std::string savePath) const;

	/**
	* 保存区域增长后的超体素，有颜色，相同颜色代表一个区域
	* savePath建议为trianglePath,将根据自路径自动重名为_regions_points_color.ply
	*/
	void saveSuperVoxelByRG(std::string savePath) const;

	/**
	* 仅保存区域增长后的表面，有颜色，相同颜色代表一个区域
	* savePath建议为trianglePath,将根据自路径自动重名为_regions_points_color.ply
	*/
	void saveOnlyPlaneSuperVoxelByRG(std::string savePath) const;

	/**
	* 仅保存区域增长后的非表面，有颜色，相同颜色代表一个区域
	* savePath建议为trianglePath,将根据自路径自动重名为_regions_points_color.ply
	*/
	void saveOnlyNotPlaneSuperVoxelByRG(std::string savePath) const;

	/**
	* 将区域生长的平面保存，返回的是一个个平面，每个平面保存的是超体素的id,
	* 不排序
	* bsave 为是否保存平面到文件，文件路径与trianglePath同目录
	*/
	void getPlanarSupervoxel(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane = true);

	/**
	* 将区域生长的平面保存，返回的是一个个平面，每个平面保存的是超体素的id,并按照数目从大到小排序
	* bsave 为是否保存平面到文件，文件路径与trianglePath同目录
	*/
	void getSortedPlanarSupervoxel(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane = true);

	/**
	* 将区域生长的平面保存，返回的是一个个平面，每个平面保存的是内部所有triangle的id,并按照数目从大到小排序
	* bsave 为是否保存平面到文件，文件路径与trianglePath同目录
	*/
	void getSortedPlanarTriangle(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane = false);
	/**
	* 保存不同颜色平面的三角形索引到文件中
	*/
	void writePlanerTriangles(const std::vector<std::vector<size_t>>& vvPlanerTris, std::string savePath);
	/**
	* 保存不同颜色平面的超体素索引到文件中
	*/
	void writePlanerSupervoxels(const std::vector<std::vector<size_t>>& vvPlanerSupervoxels, std::string savePath);
protected:
	/**
	* 根据文件解析
	*/
	void perseTrianglePath();

	/**
	* 
	*/
	void convertMesh2PointsWithNormal();

protected:
	std::vector<Triangle>* _vTriangle;
	std::vector<std::vector<Vertex>>* _vvVertex;
	std::vector<int>* _vSupervoxelIndex;               //三角形对应的超体素索引
	std::vector<std::vector<int>> _vvSupervoxelIndex;   //每个超体素含有的面的索引
	
private:
	std::string _triPath;  //triangle文件名称

	//region growing params
	std::size_t _k; //knn，搜索的邻近点
	float _maxDistanceToPlane;
	float _maxAcceptAngle;
	std::size_t _minRegion_size;
	size_t _numPlaneFound; //检测到的平面个数

	CGAL::Point_set_3<Kernel::Point_3> _inputPointSet;  //区域生长的输入点云，附带法向量，即xyzn1n2n3
	CGAL::Point_set_3<Kernel::Point_3> _outputPointSet; //区域生长的输出点云，附带法向量，即xyzn1n2n3
	std::vector<std::size_t>           _unassignedItems;


	Neighbor_query* _neighbor_query; //区域生长类
};

#endif

