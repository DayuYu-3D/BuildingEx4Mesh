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
* �����û���ȡ������
* usage: 1. ��ʼ��:����tirangle��vertes
*        2. setPlanerTriangles ���� setPlanerSupervoxels
*        3. do something, such as filteringSmallePlaneByNums, etc.
*        4. ����
* notice: ��������Supervoxelƽ�����Triangleƽ�棬�����ʲôƽ�棬���صĽ������ʲôƽ��
*/
class BuildingExtract
{
public:
	BuildingExtract(const std::vector<Triangle>& vTris, const std::vector<std::vector<Vertex>>& vvVertexs);
	~BuildingExtract();

	void setPlanerTriangles(const std::vector<std::vector<size_t>>* pvTris);
	void setPlanerSupervoxels(const std::vector<std::vector<size_t>>* pvSuperv, std::vector<int>* vSupervoxelIndex);
	//����triangle��texture����ѡ�ʹ����ɫ������������Ҫ�趨
	void setTexture(std::vector<std::string> vTexture);
	/**
	* ����������/��������������ɸѡ��С�ڵ���nums��ƽ�档
	* vvIxs_svORtri_out: ɸѡ��ʣ��ĳ�����/�����ε�����
	* minNums: ����30���ڳ����أ�5000����������
	*/
	void filteringSmallePlaneByNums(std::vector<std::vector<size_t>>& vvIxs_svORtri_out, int minNums) const;
	/**
	* ����������/��������������ɸѡ��С�ڵ���minArea��ƽ�档
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	*/
	void filteringSmallePlaneByArea(std::vector<std::vector<size_t>>& vvIxs_tri_out, float minArea) const;

	/**
	* ����������/��������������ɸѡ��С�ڵ���minArea��ƽ�档
	* vvIxs_tri_in: ƽ�������ε�����
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	*/
	void filteringSmallePlaneByArea(
		const std::vector<std::vector<size_t>>& vvIxs_tri_in, 
		std::vector<std::vector<size_t>>& vvIxs_tri_out, float minArea) const;

	/**
	* ����һ��ƽ��������ͶӰ������ı�ֵ����ɸѡƽ�档
	* vvIxs_sv_in: const �����ƽ�泬���ص�����
	* vIxs_tri_out: ɸѡ��ʣ��������ε�����
	* minRatio: ��С�ı�ֵ�����ڸ�ֵ���޳�
	* maxRatio: ���ı�ֵ��С�ڸ�ֵ���޳�
	*/
	void filteringPlaneByAreaRatio_superV(const std::vector<std::vector<size_t>>& vvIxs_sv_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float minRatio, float maxRatio) const;
	/**
	* ����һ��ƽ��������ͶӰ������ı�ֵ����ɸѡƽ�档
	* vvIxs_tri_in: const �����ƽ�������ε�����
	* vIxs_tri_out: ɸѡ��ʣ��������ε�����
	* minRatio: ��С�ı�ֵ�����ڸ�ֵ���޳�
	* maxRatio: ���ı�ֵ��С�ڸ�ֵ���޳�
	*/
	void filteringPlaneByAreaRatio_tris(const std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float minRatio, float maxRatio) const;

	/**
	* ʹ��PCA��ƽ����ж�����ϣ���ϻ�õ�һ�����ϵ����ȡֵΪ0-1���������ϵ�����й��ˡ�
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	* 
	*/
	void filteringPlaneByPCA(std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float minCoeffi) const;

	/**
	* ʹ����ɫˢѡƽ�棬��ҪĿ����ȥ����ɫ��ľƽ�棺ʹ����ɫ������췽��
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	* vvIxs_tri_in: ��������������
	* maxGreenDistance: ����ɫ����ɫ�������̶�
	*/
	void filteringPlaneByColor(std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float maxGreenDistance) const;
	/**
	* ʹ����ɫˢѡƽ�棬��ҪĿ����ȥ����ɫ��ľƽ�棺ʹ�� ExG �� ExRָ��
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	* vvIxs_tri_in: ��������������
	* maxGreenDistance: ����ɫ����ɫ�������̶�
	*/
	void filteringPlaneByExG_EXR(std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float maxGreenDistance) const;

	/**
	* ʹ����Ե������ˢѡƽ�棬��ҪĿ���ǵͰ�ֲ����������
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	* vvIxs_tri_in: ��������������
	* thresholdHeight: ��ֵ����Ե������   
	* AGL: Above ground level
	* �������˰汾Ϊ��һ�汾��ʹ�õ���Ķ�������knn���ٶ���
	*       ʹ�õ����߶�AGLH��Ϊ��ֵ�������зɵ㣬����С���ֵͰ�����δ���˳��ɾ�
	*/
	void filteringPlaneByAGL_Height_1(
		const std::vector<Triangle>& vTriangle_ground,
		std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float thresholdHeight) const;

	/**
	* ʹ����Ե������ˢѡƽ�棬��ҪĿ���ǵͰ�ֲ����������
	* vvIxs_tri_out: ɸѡ��ʣ��������ε�����
	* vvIxs_tri_in: ��������������
	* thresholdHeight: ��ֵ����Ե������
	* AGL: Above ground leve2
	* �������˰汾Ϊ�ڶ��汾��ʹ�õ������С�ĵ����ĵ�����knn���ٶȿ�
	*       ʹ�õ�ʹ�������ȨAGLH��Ϊ��ֵ
	*/
	void filteringPlaneByAGL_Height_2(
		const std::vector<Triangle>& vTriangle_ground,
		const std::vector<std::vector<Vertex>>& vvVertexs_ground,
		std::vector<std::vector<size_t>>& vvIxs_tri_in,
		std::vector<std::vector<size_t>>& vIxs_tri_out, float thresholdHeight) const;


protected:
	bool paramsValidate() const;

	/**
	* ��Supervoxelƽ��תΪtriangleƽ��
	* vvIxs_Sv_in �����ƽSupervoxelƽ��
	* vvIxs_tri_out: �����triangleƽ��
	*/
	void convertSvPlanes2TriPlanes(const std::vector<std::vector<size_t>>& vvIxs_Sv_in, 
		std::vector<std::vector<size_t>>& vvIxs_tri_out) const;

	/**
	* �������������ε���ά���֮��
	*/
	void calTotalArea_3D_triangles(double& totalArea) const;

	/**
	* ����������ƽ�����ά���
	* vvIxs_tri_in: ɸѡ��ʣ��������ε�����
	* vArea_out: �����ÿ��ƽ�����ά���
	*/
	void calArea2Planes_3D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vArea_out) const;
	/**
	* ����������ƽ���2ά�����
	* vvIxs_tri_in: ɸѡ��ʣ��������ε�����
	* vArea_out: �����ÿ��ƽ���2ά���
	*/
	void calArea2Planes_2D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vArea_out) const;
	/**
	* ����������ƽ�����ά���Ȩ��
	* vvIxs_tri_in: ɸѡ��ʣ��������ε�����
	* vArea_out: �����ÿ��ƽ������ռ����ƽ�������Ȩ��
	*/
	void calAreaRatio2Planes_3D(const std::vector<std::vector<size_t>>& vvIxs_tri_in, std::vector<double>& vAreaRatio_out) const;

private:
	const std::vector<Triangle>& _vTris;
	const std::vector<std::vector<Vertex>>& _vvVertexs;
	std::vector<std::string> _vTextures;

	const std::vector<std::vector<size_t>>* _pvvPlane_Tris;
	const std::vector<std::vector<size_t>>* _pvvPlane_SuperVoxel;
	std::vector<std::vector<int>> _vvSupervoxelIndex;   //ÿ�������غ��е��������
};
