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

#include "PCH.h" //Ԥ����ͷ

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

	//�����ھӲ�ѯ��ʽ��only��ѡһ
	///case1�����ڵ��Ƶ�KNN�ھӲ�ѯ
	//using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Input_range, Point_map>;
	///case2���������˹�ϵ���ھӲ�ѯ
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
	* ��������ִ�к���
	*/
	void regionGrowing();

	/**
	* �����������������ص����ĵ㣬����ɫ����ͬ��ɫ����һ������
	* savePath����ΪtrianglePath,��������·���Զ�����Ϊ_regions_points_color.ply
	*/
	void savePointsOfSupCenterByRG(std::string savePath) const;

	/**
	* ��������������ĳ����أ�����ɫ����ͬ��ɫ����һ������
	* savePath����ΪtrianglePath,��������·���Զ�����Ϊ_regions_points_color.ply
	*/
	void saveSuperVoxelByRG(std::string savePath) const;

	/**
	* ����������������ı��棬����ɫ����ͬ��ɫ����һ������
	* savePath����ΪtrianglePath,��������·���Զ�����Ϊ_regions_points_color.ply
	*/
	void saveOnlyPlaneSuperVoxelByRG(std::string savePath) const;

	/**
	* ����������������ķǱ��棬����ɫ����ͬ��ɫ����һ������
	* savePath����ΪtrianglePath,��������·���Զ�����Ϊ_regions_points_color.ply
	*/
	void saveOnlyNotPlaneSuperVoxelByRG(std::string savePath) const;

	/**
	* ������������ƽ�汣�棬���ص���һ����ƽ�棬ÿ��ƽ�汣����ǳ����ص�id,
	* ������
	* bsave Ϊ�Ƿ񱣴�ƽ�浽�ļ����ļ�·����trianglePathͬĿ¼
	*/
	void getPlanarSupervoxel(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane = true);

	/**
	* ������������ƽ�汣�棬���ص���һ����ƽ�棬ÿ��ƽ�汣����ǳ����ص�id,��������Ŀ�Ӵ�С����
	* bsave Ϊ�Ƿ񱣴�ƽ�浽�ļ����ļ�·����trianglePathͬĿ¼
	*/
	void getSortedPlanarSupervoxel(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane = true);

	/**
	* ������������ƽ�汣�棬���ص���һ����ƽ�棬ÿ��ƽ�汣������ڲ�����triangle��id,��������Ŀ�Ӵ�С����
	* bsave Ϊ�Ƿ񱣴�ƽ�浽�ļ����ļ�·����trianglePathͬĿ¼
	*/
	void getSortedPlanarTriangle(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane = false);
	/**
	* ���治ͬ��ɫƽ����������������ļ���
	*/
	void writePlanerTriangles(const std::vector<std::vector<size_t>>& vvPlanerTris, std::string savePath);
	/**
	* ���治ͬ��ɫƽ��ĳ������������ļ���
	*/
	void writePlanerSupervoxels(const std::vector<std::vector<size_t>>& vvPlanerSupervoxels, std::string savePath);
protected:
	/**
	* �����ļ�����
	*/
	void perseTrianglePath();

	/**
	* 
	*/
	void convertMesh2PointsWithNormal();

protected:
	std::vector<Triangle>* _vTriangle;
	std::vector<std::vector<Vertex>>* _vvVertex;
	std::vector<int>* _vSupervoxelIndex;               //�����ζ�Ӧ�ĳ���������
	std::vector<std::vector<int>> _vvSupervoxelIndex;   //ÿ�������غ��е��������
	
private:
	std::string _triPath;  //triangle�ļ�����

	//region growing params
	std::size_t _k; //knn���������ڽ���
	float _maxDistanceToPlane;
	float _maxAcceptAngle;
	std::size_t _minRegion_size;
	size_t _numPlaneFound; //��⵽��ƽ�����

	CGAL::Point_set_3<Kernel::Point_3> _inputPointSet;  //����������������ƣ���������������xyzn1n2n3
	CGAL::Point_set_3<Kernel::Point_3> _outputPointSet; //����������������ƣ���������������xyzn1n2n3
	std::vector<std::size_t>           _unassignedItems;


	Neighbor_query* _neighbor_query; //����������
};

#endif

