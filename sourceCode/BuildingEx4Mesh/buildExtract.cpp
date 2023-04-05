#include "buildExtract.h"

#include <iostream>
#include <chrono>
#include <numeric>
#include <exception>

#include <omp.h>

#include "codelibrary/geometry/kernel/point_3d.h"
#include "codelibrary/util/tree/kd_tree.h"

#include "yutils/utils.h"
#include "colordistance.h"
#include "meshpaser/caltriangleuvcoords.h"
#include "meshpaser/calTrianglsCenter.h"

namespace {

	float otsuAlgThreshold(std::vector<float> vExG_ExR) {
		if (!vExG_ExR.size()) {
			std::cout << "return 0, due to vExg_ExR is null\n";
			return 0;
		}

		float maxExp = *std::max_element(vExG_ExR.begin(), vExG_ExR.end());
		float minExp = *std::min_element(vExG_ExR.begin(), vExG_ExR.end());
		float rangeExp = maxExp - minExp;
		float interval = rangeExp / 255;
		///
		int threshod = 0; 

		std::vector<float> histogram[255]; 

		double totalNum = vExG_ExR.size(); 

		for (int i = 0; i < totalNum; i++)   
		{
			int ix = int ((vExG_ExR[i] - minExp)/interval); //向下取整
			if (ix > 254) {
				ix = 254;
			}
			histogram[ix].emplace_back(vExG_ExR[i]);

		}

		//double varValue = 0; //类间方差中间值保存
		//double w1 = 0; //背景像素点数所占比例
		//double u1 = 0; //背景平均灰度
		//double w2 = 0; //前景像素点数所占比例
		//double u2 = 0; //前景平均灰度
		//for (int i = 0; i < 255; ++i) {
		//	w1 = 0; u1 = 0; w2 = 0; u2 = 0;
		//	//***********背景各分量值计算**************************
		//	for (int j = 0; j <= i; j++) //背景部分各值计算
		//	{
		//		w1 += histogram[j].size();  //背景部分像素点总数
		//		u1 += std::accumulate(histogram[j].begin(), histogram[j].end(),0); //背景部分像素总灰度和
		//	}
		//	if (w1 == 0) //背景部分像素点数为0时退出
		//		continue;
		//	u1 = u1 / w1; //背景像素平均灰度
		//	w1 = w1 / totalNum; // 背景部分像素点数所占比例


		//	//***********前景各分量值计算**************************
		//	for (int k = i + 1; k < 255; k++)
		//	{
		//		w2 += histogram[k].size();  //前景部分像素点总数
		//		u2 += std::accumulate(histogram[k].begin(), histogram[k].end(), 0);  //前景部分像素总灰度和
		//	}
		//	if (w2 == 0) //前景部分像素点数为0时退出
		//		break;

		//	u2 = u2 / w2; //前景像素平均灰度
		//	w2 = w2 / totalNum; // 前景部分像素点数所占比例

		//	//***********类间方差计算******************************
		//	double varValueI = w2 * w1 * (u1 - u2) * (u1 - u2); //当前类间方差计算
		//	if (varValue < varValueI)
		//	{
		//		varValue = varValueI;
		//		if (histogram[i].size()) {
		//			threshod = std::accumulate(histogram[i].begin(), histogram[i].end(), 0) / histogram[i].size();					
		//		}
		//		else
		//		{
		//			threshod = minExp + i * interval + 0.5 * interval;
		//		}

		//	}
		//}
		
		// Compute threshold
		// Init variables
		float sum = 0;
		float sumB = 0;
		int q1 = 0;
		int q2 = 0;
		float varMax = 0;

		// Auxiliary value for computing m2
		for (int i = 0; i < 255; i++) {
			sum += i * ((int)histogram[i].size());
		}

		for (int i = 0; i < 255; i++) {
			// Update q1
			q1 += histogram[i].size();
			if (q1 == 0)
				continue;
			// Update q2
			q2 = totalNum - q1;

			if (q2 == 0)
				break;
			// Update m1 and m2
			sumB += (float)(i * ((int)histogram[i].size()));
			float m1 = sumB / q1;
			float m2 = (sum - sumB) / q2;


			float varBetween = (float)q1 * (float)q2 * (m1 - m2) * (m1 - m2);


			if (varBetween > varMax) {
				varMax = varBetween;
				threshod = i;
			}
		}
		threshod = minExp + threshod * interval+ 0.5 * interval;
		return threshod;
	}
}

BuildingExtract::BuildingExtract(const std::vector<Triangle>& vTris, const std::vector<std::vector<Vertex>>& vvVertexs)
	:_vTris(vTris),
	_vvVertexs(vvVertexs),
	_pvvPlane_Tris(nullptr),
	_pvvPlane_SuperVoxel(nullptr)
{
}

BuildingExtract::~BuildingExtract()
{
}

void BuildingExtract::setPlanerTriangles(const std::vector<std::vector<size_t>>* pvTris)
{
	_pvvPlane_Tris = pvTris;
}

void BuildingExtract::setPlanerSupervoxels(const std::vector<std::vector<size_t>>* pvSuperv, std::vector<int>* vSupervoxelIndex)
{
	_pvvPlane_SuperVoxel = pvSuperv;

	int numSupervoxel = *std::max_element(vSupervoxelIndex->begin(), vSupervoxelIndex->end());
	_vvSupervoxelIndex.resize(numSupervoxel + 1);
	std::cout << "*　Num of supervoxel is: " << numSupervoxel + 1 << std::endl;
	for (int i = 0; i < vSupervoxelIndex->size(); ++i)
	{
		int ix = vSupervoxelIndex->at(i);
		_vvSupervoxelIndex[ix].emplace_back(i);
	}
}

void BuildingExtract::setTexture(std::vector<std::string> vTexture)
{
	_vTextures = vTexture;
}

void BuildingExtract::filteringSmallePlaneByNums(std::vector<std::vector<size_t>>& vvIxs_svORtri_out, int minNums) const
{
	if (!this->paramsValidate())
		return;
	const auto& vplane = _pvvPlane_SuperVoxel != nullptr ? *_pvvPlane_SuperVoxel : *_pvvPlane_Tris;

	vvIxs_svORtri_out.clear();
	for (auto& plane : vplane)
	{
		if (plane.size() > minNums) {
			vvIxs_svORtri_out.emplace_back(plane);
		}
	}
}
void BuildingExtract::filteringSmallePlaneByArea(std::vector<std::vector<size_t>>& vvIxs_tri_out, float minArea) const
{
	if (!this->paramsValidate())
		return;	
	std::vector < std::vector<size_t>> vvplan_tris; 
	if (_vvSupervoxelIndex.size() != 0 && _pvvPlane_SuperVoxel != nullptr) { 
		this->convertSvPlanes2TriPlanes(*_pvvPlane_SuperVoxel, vvplan_tris);
	}
	else
	{ 
		vvplan_tris = *_pvvPlane_Tris;
	}

	this->filteringSmallePlaneByArea(vvplan_tris, vvIxs_tri_out, minArea);
}

void BuildingExtract::filteringSmallePlaneByArea(
	const std::vector<std::vector<size_t>>& vvIxs_tri_in, 
	std::vector<std::vector<size_t>>& vvIxs_tri_out, float minArea) const
{
	std::vector<double> vPlaneArea; 
	this->calArea2Planes_3D(vvIxs_tri_in, vPlaneArea);


	for (size_t i = 0; i < vPlaneArea.size(); ++i)
	{
		float area = vPlaneArea[i];
		if (area > minArea) {
			vvIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		}
	}

	//Optional： 保存面积到文件
	if (1)
	{
		std::sort(vPlaneArea.begin(), vPlaneArea.end());
		yutil::CalTriangArea::write1DVector(vPlaneArea, "plane_area.txt");
	}

}

void BuildingExtract::filteringPlaneByAreaRatio_superV(
	const std::vector<std::vector<size_t>>& vvIxs_sv_in, std::vector<std::vector<size_t>>& vvIxs_tri_out
	, float minRatio, float maxRatio) const
{
	if (!vvIxs_sv_in.size())
	{
		std::cerr << "!vIxs_in.size()" << std::endl;
		return;
	}
	std::vector < std::vector<size_t>> vvplan_tris; 
	this->convertSvPlanes2TriPlanes(vvIxs_sv_in, vvplan_tris);

	this->filteringPlaneByAreaRatio_tris(vvplan_tris, vvIxs_tri_out, minRatio,maxRatio);
}

void BuildingExtract::filteringPlaneByAreaRatio_tris(
	const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<std::vector<size_t>>& vvIxs_tri_out,
	float minRatio, float maxRatio) const
{
	std::vector<double> vPlaneArea_3D;
	std::vector<double> vPlaneArea_2D; 
	this->calArea2Planes_3D(vvIxs_tri_in,vPlaneArea_3D);
	this->calArea2Planes_2D(vvIxs_tri_in, vPlaneArea_2D);
	std::vector<float> vAreaRatio; 
	for (size_t i = 0; i < vPlaneArea_3D.size(); ++i) {
		float ratio = vPlaneArea_2D[i] / vPlaneArea_3D[i];
		vAreaRatio.emplace_back(ratio);
	}

	for (size_t i = 0; i < vvIxs_tri_in.size(); ++i)
	{
		float ratio = vAreaRatio[i];
		if (ratio <= minRatio || ratio >= maxRatio) {
			vvIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		}
	}


	if (1)
	{
		std::sort(vAreaRatio.begin(), vAreaRatio.end());
		yutil::CalTriangArea::write1DVector(vAreaRatio, "plane_areaRatio.txt");
	}
}

void BuildingExtract::filteringPlaneByPCA(std::vector<std::vector<size_t>>& vvIxs_tri_in, 
	std::vector<std::vector<size_t>>& vIxs_tri_out, float minCoeffi) const
{
	using Triangle_3 = Kernel::Triangle_3;
	using Point_3 = Kernel::Point_3;
	using Plane_3 = Kernel::Plane_3;
	using FT = float;

	 //-------------------------------------------------------------------------  
     std::cout << "* PCA for calculating plane cofficient" << std::endl;	 
	 int nPlanes = vvIxs_tri_in.size();
	 std::vector<double> vCoffient(nPlanes);
 #pragma omp parallel for
     for (int i = 0; i < nPlanes; ++i)
     {
		 auto plane = vvIxs_tri_in[i];
		 std::vector<Triangle_3> plane_cgal;
		 for (auto triIx : plane) {

			 Triangle tri = _vTris[triIx];
			 auto geomIx = tri._geomIndex;
			 auto v1 = _vvVertexs[geomIx][tri._vertexIndexs[0]];
			 auto v2 = _vvVertexs[geomIx][tri._vertexIndexs[1]];
			 auto v3 = _vvVertexs[geomIx][tri._vertexIndexs[2]];
			 Point_3 p1(v1._coor[0], v1._coor[1], v1._coor[2]);
			 Point_3 p2(v2._coor[0], v2._coor[1], v2._coor[2]);
			 Point_3 p3(v3._coor[0], v3._coor[1], v3._coor[2]);
			 Triangle_3 tri_cgal(p1, p2, p3);
			 plane_cgal.emplace_back(tri_cgal);
		 }
		Plane_3 plane3;
		//tag：0表示使用三角形顶点拟合；1表示使用三角形边拟合；2表示使用三角形来拟合
		//试验表明：2的效果最好，2021/6/8
		FT coffient = CGAL::linear_least_squares_fitting_3(
			plane_cgal.begin(), plane_cgal.end(), plane3, CGAL::Dimension_tag<2>());
		vCoffient[i] = coffient;
     }


	 for (size_t i = 0; i < vvIxs_tri_in.size(); ++i)
	 {
		 float ratio = vCoffient[i];
		 if (ratio >= minCoeffi) {
			 vIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		 }
	 }


	 if (1)
	 {
		 std::vector<double> vAreaRation_planes;
		 this->calAreaRatio2Planes_3D(vvIxs_tri_in,vAreaRation_planes);

		 for (size_t i = 0; i < vCoffient.size(); ++i)
		 {
			 vCoffient[i] = vCoffient[i] * (vAreaRation_planes[i]);
		 }
		 std::sort(vCoffient.begin(), vCoffient.end());   
		 yutil::CalTriangArea::write1DVector(vCoffient, "plane_conffient.txt");

		 //TODO: delete
		 double totalArea;
		 this->calTotalArea_3D_triangles(totalArea);
		 std::vector<double> plane_area;
		 this->calArea2Planes_3D(vvIxs_tri_in, plane_area);
		 double totalarea_plane = 0;
		 for (double& area : plane_area)
			 totalarea_plane += area;
		 std::cout << "Total triangle area: " << totalArea << " total plane area: " << totalarea_plane << std::endl;
	 }
}

void BuildingExtract::filteringPlaneByColor(std::vector<std::vector<size_t>>& vvIxs_tri_in, 
	std::vector<std::vector<size_t>>& vIxs_tri_out, float maxGreenDistance) const
{
	if (!_vTextures.size()) {
		std::cout << " ***!!! _vTextures is null, please setTextures" << std::endl;
		return;
	}

	//-------------------------------------------------------------------------

	auto startT = std::chrono::system_clock::now();//获取开始时间
	CalTriangleUVCoords calUV(_vTris, _vvVertexs, _vTextures);
	std::vector<cl::RGB32Color> vMeanTriRGB; 
	calUV.getMeanRGB4Triangle(vMeanTriRGB);
	auto entT = std::chrono::system_clock::now();//获取结束时间
	auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
	std::cout << " ** use time in cal mean color of trirangle: " << (double)ut / 1000 << "s" << std::endl;

	std::vector< cl::RGB32Color> vMeanColor_plane;
	for (auto planeIx : vvIxs_tri_in)
	{
		unsigned int r = 0, g = 0, b = 0;
		for (size_t triIx : planeIx)
		{
			cl::RGB32Color& triMeanColor = vMeanTriRGB[triIx];
			r += triMeanColor.red();
			g += triMeanColor.green();
			b += triMeanColor.blue();
		}
		r /= planeIx.size();
		g /= planeIx.size();
		b /= planeIx.size();
		cl::RGB32Color mc(r,g,b);
		vMeanColor_plane.emplace_back(mc);
	}

	std::vector<float> vcolorDsitanceToGreen;
	cl::RGB32Color green(0,112,0);
	for (auto mc : vMeanColor_plane)
	{

		float colorDist = cd::getDeltaE_CIE76(green,mc);
		vcolorDsitanceToGreen.push_back(colorDist);
	}


	for (size_t i = 0; i < vvIxs_tri_in.size(); ++i)
	{
		float cd = vcolorDsitanceToGreen[i];
		if (cd >= maxGreenDistance) {
			vIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		}
	}


	if (1)
	{
		std::sort(vcolorDsitanceToGreen.begin(), vcolorDsitanceToGreen.end());
		yutil::CalTriangArea::write1DVector(vcolorDsitanceToGreen, "plane_colorDistance2Green.txt");
	}
}

void BuildingExtract::filteringPlaneByExG_EXR(std::vector<std::vector<size_t>>& vvIxs_tri_in,
	std::vector<std::vector<size_t>>& vIxs_tri_out, float maxGreenDistance) const
{
	if (!_vTextures.size()) {
		std::cout << " ***!!! _vTextures is null, please setTextures" << std::endl;
		return;
	}

	//-------------------------------------------------------------------------

	auto startT = std::chrono::system_clock::now();//获取开始时间
	CalTriangleUVCoords calUV(_vTris, _vvVertexs, _vTextures);
	std::vector<cl::RGB32Color> vMeanTriRGB;
	calUV.getMeanRGB4Triangle(vMeanTriRGB);
	auto entT = std::chrono::system_clock::now();//获取结束时间
	auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
	std::cout << " ** use time in cal mean color of trirangle: " << (double)ut / 1000 << "s" << std::endl;

	std::vector< cl::RGB32Color> vMeanColor_plane;
	for (auto planeIx : vvIxs_tri_in)
	{
		unsigned int r = 0, g = 0, b = 0;
		for (size_t triIx : planeIx)
		{
			cl::RGB32Color& triMeanColor = vMeanTriRGB[triIx];
			r += triMeanColor.red();
			g += triMeanColor.green();
			b += triMeanColor.blue();
		}
		r /= planeIx.size();
		g /= planeIx.size();
		b /= planeIx.size();
		cl::RGB32Color mc(r, g, b);
		vMeanColor_plane.emplace_back(mc);
	}


	std::vector<float> vExG_ExR;

	for (auto mc : vMeanColor_plane)
	{
		float exg_exr = 3 * mc.green() - 2.4 * mc.red() - mc.blue();
		vExG_ExR.push_back(exg_exr);
	}


	//for (size_t i = 0; i < vvIxs_tri_in.size(); ++i)
	//{
	//	float cd = vExG_ExR[i];
	//	if (cd < maxGreenDistance) {
	//		vIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
	//	}
	//}


	float threshold = otsuAlgThreshold(vExG_ExR);
	std::cout << "The threshold by otsu: " << threshold << std::endl;
	for (size_t i = 0; i < vvIxs_tri_in.size(); ++i)
	{
		float cd = vExG_ExR[i];
		if (cd < threshold) {
			vIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		}
	}

	//Optional： 保存拟合系数到文件
	if (1)
	{
		std::sort(vExG_ExR.begin(), vExG_ExR.end());
		yutil::CalTriangArea::write1DVector(vExG_ExR, "plane_colorDistance2Green.txt");
	}
}

void BuildingExtract::filteringPlaneByAGL_Height_1(
	const std::vector<Triangle>& vTriangle_ground,
	std::vector<std::vector<size_t>>& vvIxs_tri_in,
	std::vector<std::vector<size_t>>& vIxs_tri_out, float thresholdHeight) const
{

	cl::Array<cl::RPoint3D> vRPoint(vTriangle_ground.size()*3);
	for (int i = 0; i < vTriangle_ground.size();++i) 
	{
		auto triGround = vTriangle_ground[i];
		auto geomIx = triGround._geomIndex;
		auto p1Ix = _vvVertexs[geomIx][triGround._vertexIndexs[0]];
		auto p2Ix = _vvVertexs[geomIx][triGround._vertexIndexs[1]];
		auto p3Ix = _vvVertexs[geomIx][triGround._vertexIndexs[2]];

		cl::RPoint3D rpoint1(p1Ix._coor.x(), p1Ix._coor.y(), p1Ix._coor.z());
		vRPoint[3 * i + 0] = rpoint1;
		cl::RPoint3D rpoint2(p2Ix._coor.x(), p2Ix._coor.y(), p2Ix._coor.z());
		vRPoint[3 * i + 1] = rpoint2;
		cl::RPoint3D rpoint3(p3Ix._coor.x(), p3Ix._coor.y(), p3Ix._coor.z());
		vRPoint[3 * i + 2] = rpoint3;
	}
	//vRPoint.shrink_to_fit();


	std::cout << "* Building KD tree...\n";
	cl::KDTree<cl::RPoint3D> kdtree;
	kdtree.SwapPoints(&vRPoint);
	std::cout << "* KD tree done\n";
	auto startT = std::chrono::system_clock::now();//获取结束时间


	std::vector<float> vPlane_AGLH(vvIxs_tri_in.size());

	CalTrianglesCenter calCenter(_vTris, _vvVertexs);
	int nPlane = vvIxs_tri_in.size();
#pragma omp parallel for
	for (int nP =0; nP<nPlane;++nP)
	{
		auto vPlane = vvIxs_tri_in[nP];		
		float plane_GH = 0;   
		cl::Array<cl::RPoint3D> vTriCenters; 
		cl::Array<cl::RPoint3D> neighbors; 
		calCenter.getMassCenter(vTriCenters,vPlane);

		if (!vTriCenters.size())
			continue;
		float plane_maxH = vTriCenters[0].z; 
		const int k_neighbors = 30;
		for (auto massCenter : vTriCenters) 
		{
			kdtree.FindKNearestNeighbors(massCenter, k_neighbors, &neighbors);
			if (plane_maxH < massCenter.z) {
				plane_maxH = massCenter.z;
			}
			for (auto vertex : neighbors)
			{
				float z = vertex.z;
				plane_GH += z;
			}
		}
		plane_GH /= vPlane.size()* k_neighbors;
		
		float AGLH = plane_maxH - plane_GH;

		vPlane_AGLH[nP] = AGLH;

	}
	auto endT = std::chrono::system_clock::now();//获取结束时间
	auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(endT - startT).count();
	std::cout << " ** use time in cal AGLH: " << (double)ut / 1000 << "s\n";

	///阈值几种考虑：1.平均值：缺点，三角形不均衡问题
    ///            2.面积加权平均值
	///            3.最高减去最低，然后除以2
	///            4.最高值
	for (size_t i = 0; i < vPlane_AGLH.size(); ++i)
	{
		float aglh = vPlane_AGLH[i];
		if (aglh > thresholdHeight) {
			vIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		}
	}

	//Optional： 保存拟合系数到文件
	if (1)
	{
		std::sort(vPlane_AGLH.begin(), vPlane_AGLH.end());
		yutil::CalTriangArea::write1DVector(vPlane_AGLH, "plane_AGLH.txt");
	}
}

void BuildingExtract::filteringPlaneByAGL_Height_2(
	const std::vector<Triangle>& vTriangle_ground,
	const std::vector<std::vector<Vertex>>& vvVertexs_ground,
	std::vector<std::vector<size_t>>& vvIxs_tri_in, 
	std::vector<std::vector<size_t>>& vIxs_tri_out, float thresholdHeight) const
{

	cl::Array<cl::RPoint3D> vRPoint(vTriangle_ground.size());
	CalTrianglesCenter calCenter1(vTriangle_ground, vvVertexs_ground);
	calCenter1.getMassCenter(vRPoint);

	std::cout << "* Building KD tree...\n";
	cl::KDTree<cl::RPoint3D> kdtree;
	kdtree.SwapPoints(&vRPoint);
	std::cout << "* KD tree done\n";
	auto startT = std::chrono::system_clock::now();//获取结束时间

	///阈值几种考虑：1.平均值：缺点，三角形不均衡问题
	///            2.面积加权平均值
	///            3.最高减去最低，然后除以2
	///            4.最高值
	/////计算每个平面的面积加权
	/////////首先计算每个面的总面积
	std::vector<double> vArea;
	this->calArea2Planes_3D(vvIxs_tri_in, vArea);
	vArea.shrink_to_fit();
	std::vector<std::vector<float>> vvAreaRatio(vvIxs_tri_in.size());
	for (int i = 0; i < vvIxs_tri_in.size(); ++i)
	{
		for (size_t triIx : vvIxs_tri_in[i])
		{
			const Triangle& tri = _vTris[triIx];
			Vertex vertex1 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[0]];
			Vertex vertex2 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[1]];
			Vertex vertex3 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[2]];
			yutil::CalTriangArea calArea;
			float area2 = calArea.calAreaOFtriangle_square(vertex1, vertex2, vertex3);
			float area = std::sqrt(area2);
			vvAreaRatio[i].emplace_back(area / vArea[i]);
		}
	}

	std::vector<float> vPlane_AGLH(vvIxs_tri_in.size());

	CalTrianglesCenter calCenter(_vTris, _vvVertexs);
	int nPlane = vvIxs_tri_in.size();
#pragma omp parallel for
	for (int nP = 0; nP < nPlane; ++nP)
	{
		auto vPlane = vvIxs_tri_in[nP];
		cl::Array<cl::RPoint3D> vTriCenters; //质心		
		calCenter.getMassCenter(vTriCenters, vPlane);

		if (!vTriCenters.size())
			continue;

		float plane_aglh = 0;
		const int k_neighbors = 10;
		for(int nTri =0;nTri<vPlane.size();++nTri)
		{
			auto massCenter = vTriCenters[nTri];
			cl::Array<cl::RPoint3D> neighbors;
			kdtree.FindKNearestNeighbors(massCenter, k_neighbors, &neighbors);

			float tri_GH = 0; 
			for (auto vertex : neighbors)
			{
				float z = vertex.z;
				tri_GH += z;
			}
			tri_GH /=  k_neighbors;
			float tri_aglh = massCenter.z - tri_GH;

			float tri_aglh_withArea = tri_aglh * vvAreaRatio[nP][nTri];
			plane_aglh += tri_aglh_withArea;
		}

		vPlane_AGLH[nP] = plane_aglh;

	}
	auto endT = std::chrono::system_clock::now();//获取结束时间
	auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(endT - startT).count();
	std::cout << " ** use time in cal AGLH: " << (double)ut / 1000 << "s\n";


	for (size_t i = 0; i < vPlane_AGLH.size(); ++i)
	{
		float aglh = vPlane_AGLH[i];
		if (aglh > thresholdHeight) {
			vIxs_tri_out.emplace_back(vvIxs_tri_in[i]);
		}
	}

	//Optional： 保存拟合系数到文件
	if (1)
	{
		std::sort(vPlane_AGLH.begin(), vPlane_AGLH.end());
		yutil::CalTriangArea::write1DVector(vPlane_AGLH, "plane_AGLH.txt");
	}
}


bool BuildingExtract::paramsValidate() const 
{
	if (_pvvPlane_SuperVoxel == nullptr && _pvvPlane_Tris == nullptr)
	{
		std::cerr << "error dur to：_pvSuperVoxel == nullptr && _pvTris == nullptr" << std::endl;
		return false;
	}
	if (_pvvPlane_SuperVoxel != nullptr && _pvvPlane_Tris != nullptr)
	{
		std::cerr << "error dur to：_pvSuperVoxel != nullptr && _pvTris != nullptr" << std::endl;
		return false;
	}
	return true;
}

void BuildingExtract::calTotalArea_3D_triangles(double& totalArea) const
{
	totalArea = 0.0;
	for (size_t i = 0; i < _vTris.size(); ++i)
	{
		const Triangle& tri = _vTris[i];
		Vertex vertex1 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[0]];
		Vertex vertex2 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[1]];
		Vertex vertex3 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[2]];
		yutil::CalTriangArea calArea;
		float area2 = calArea.calAreaOFtriangle_square(vertex1, vertex2, vertex3);
		totalArea += std::sqrt(area2);
	}
}

void BuildingExtract::convertSvPlanes2TriPlanes(
	const std::vector<std::vector<size_t>>& vvIxs_Sv_in, std::vector<std::vector<size_t>>& vvIxs_tri_out) const 
{
	for (auto plane : vvIxs_Sv_in) {
		std::vector<size_t> vTrisIx;
		for (size_t superIx : plane) {
			const auto& vSuperV = _vvSupervoxelIndex[superIx];
			for (size_t triIX : vSuperV)
			{
				vTrisIx.emplace_back(triIX);
			}
		}
		vvIxs_tri_out.emplace_back(vTrisIx);
	}
}

void BuildingExtract::calArea2Planes_3D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vArea_out) const
{
	for (auto& plane : vvIxs_tri_in) {
		double areaPlane = 0.f;
		for (size_t triIx : plane)
		{
			const Triangle& tri = _vTris[triIx];
			Vertex vertex1 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[0]];
			Vertex vertex2 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[1]];
			Vertex vertex3 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[2]];
			yutil::CalTriangArea calArea;
			float area2 = calArea.calAreaOFtriangle_square(vertex1, vertex2, vertex3);
			areaPlane += std::sqrt(area2);
		}
		vArea_out.emplace_back(areaPlane);
	}
}

void BuildingExtract::calArea2Planes_2D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vArea_out) const
{
	for (auto& plane : vvIxs_tri_in) {
		double areaPlane = 0.f;
		for (size_t triIx : plane)
		{
			const Triangle& tri = _vTris[triIx];
			Vertex vertex1 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[0]];
			Vertex vertex2 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[1]];
			Vertex vertex3 = _vvVertexs[tri._geomIndex][tri._vertexIndexs[2]];
			vertex1._coor.z() = 0;
			vertex2._coor.z() = 0;
			vertex3._coor.z() = 0;
			yutil::CalTriangArea calArea;
			float area2 = calArea.calAreaOFtriangle_square(vertex1, vertex2, vertex3);
			areaPlane += std::sqrt(area2);
		}
		vArea_out.emplace_back(areaPlane);
	}
}

void BuildingExtract::calAreaRatio2Planes_3D(
	const std::vector<std::vector<size_t>>& vvIxs_tri_in,
	std::vector<double>& vAreaRatio_out) const
{
	this->calArea2Planes_3D(vvIxs_tri_in, vAreaRatio_out);

	double totalArea_planes = std::accumulate(vAreaRatio_out.begin(),vAreaRatio_out.end(),0.0);
	for (double& areaRatio : vAreaRatio_out)
	{
		areaRatio = areaRatio / totalArea_planes;
	}
}
