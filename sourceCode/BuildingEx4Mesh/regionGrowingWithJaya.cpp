#include "regionGrowingWithJaya.h"

#include <iostream>
#include <random>
#include <fstream>
#include <numeric>
#include <array>
#include <set>

#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Real_timer.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "yutils/utils.h"
#include "CalOBB_byPCA.h"

ResionGrowingWithJay::ResionGrowingWithJay(
    std::vector<Triangle>* vTriangle,
    std::vector<std::vector<Vertex>>* vvVertex,
    std::vector<int>* vSupervoxelIndex)
    : ResionGrowing4Points(vTriangle,vvVertex,vSupervoxelIndex),
    ObjectedFuncBase()
{
}

ResionGrowingWithJay::~ResionGrowingWithJay()
{
}

double ResionGrowingWithJay::getValue(std::vector<double> solution)
{
    //k, 最大距离，最大角度，最小超体素个数
    this->setParams(16, solution[0], solution[1], solution[2]);
    if(solution.size()!=3)
        this->setParams(16, solution[0], solution[1], 3);
    std::cout << "maxDis: " << solution[0] <<" maxAngle: " << solution[1]
        <<  std::endl;

    CGAL::Real_timer timer;
    timer.start();

    //设置参数后，调用区域生长
    //this->setParams(k, maxDistanceToPlane, maxAcceptAngle, minRegion_size);
    this->regionGrowing();
    std::cout << "Elapsed time in Region Growing: " << timer.time() << std::endl;

    std::vector<std::vector<size_t>> outSortedPlan_tris;
    this->getSortedPlanarTriangle(outSortedPlan_tris, false);
    std::cout << "Elapsed time in overall getSorted: " << timer.time() << std::endl;

    //计算每个平面的OBB的最大面积，计算每个平面的面积
    //this->calOBB4Plane_byCGAL();
    std::vector<float> vMaxAreas, vTriArea;
    this->calOBB4Plane_byPCL(outSortedPlan_tris, vMaxAreas, vTriArea);
    std::cout << "Elapsed time in OBB PCL: " << timer.time() << std::endl;

    //计算每个平面的平面拟合系数
    std::vector<float> vPlaneCofficients;
    this->calPlaneCofficient(outSortedPlan_tris, vPlaneCofficients);
    std::cout << "Elapsed time in plane cofficient: " << timer.time() << std::endl;

    //计算每个平面的面积与所有平面面积之和的比值
    std::vector<float> areaRatio(vTriArea.size());
    float area_totalPlane = std::accumulate(vTriArea.begin(), vTriArea.end(), 0.f);
    for (size_t i = 0; i < vTriArea.size(); ++i)
        areaRatio[i] = vTriArea[i] / area_totalPlane;

    //计算每个平面的完整度比重
    //计算公式：每个平面的面积/OBB最大面的面积
    std::vector<float> interGtriy(vTriArea.size());
    for (size_t i = 0; i < vTriArea.size(); ++i)
    {
        if (vMaxAreas[i] < 0) vMaxAreas[i] = 0;
        interGtriy[i] = vTriArea[i] / vMaxAreas[i];
        if (interGtriy[i] > 1) interGtriy[i] = 1;        
    }        

    //计算优化目标函数
    //计算公式：面积比重(areaRatio)*(拟合系数(Cofficients)*完整度(interGtriy))
    float value = 0.f;
    for (size_t i = 0; i < vTriArea.size(); ++i)
    {
        value += areaRatio[i] * (0.5 * vPlaneCofficients[i] + 0.5 * interGtriy[i]);
    }

    std::cout << "Elapsed time in calValue: " << timer.time() << std::endl;
    std::cout << "vaule: " << 1-value << std::endl;
    return 1-value;
}

void ResionGrowingWithJay::calOBB4Plane_byCGAL()
{
    std::vector<std::vector<size_t>> outSortedPlan_tris;
    this->getSortedPlanarTriangle(outSortedPlan_tris, false);

    //平面点转为CGAL点
    std::vector< std::vector < Kernel::Point_3>> vvPlanes(outSortedPlan_tris.size());
    for (size_t i = 0; i < outSortedPlan_tris.size(); ++i)
    {
        auto& plane = outSortedPlan_tris[i];
        for (size_t& triIx : plane)
        {
            int& geomIx = this->_vTriangle->at(triIx)._geomIndex;
            int& v0Ix = this->_vTriangle->at(triIx)._vertexIndexs[0];
            int& v1Ix = this->_vTriangle->at(triIx)._vertexIndexs[1];
            int& v2Ix = this->_vTriangle->at(triIx)._vertexIndexs[2];

            Vertex& v0 = this->_vvVertex->at(geomIx)[v0Ix];
            Vertex& v1 = this->_vvVertex->at(geomIx)[v1Ix];
            Vertex& v2 = this->_vvVertex->at(geomIx)[v2Ix];
            Kernel::Point_3 p0(v0._coor[0], v0._coor[1], v0._coor[2]);
            Kernel::Point_3 p1(v1._coor[0], v1._coor[1], v1._coor[2]);
            Kernel::Point_3 p2(v2._coor[0], v2._coor[1], v2._coor[2]);

            vvPlanes[i].push_back(p0);
            vvPlanes[i].push_back(p1);
            vvPlanes[i].push_back(p2);
        }
    }
    
    CGAL::Real_timer timer;
    timer.start();
    
    //cgal oriented bounding box function
    for (size_t i = 0; i < vvPlanes.size(); ++i)
    {
        std::vector< Kernel::Point_3>& plane = vvPlanes[i];
        std::array<Kernel::Point_3, 8> obb_points;
        CGAL::oriented_bounding_box(plane, obb_points, CGAL::parameters::use_convex_hull(true));
        
        //if(0){ //开发阶段阶段验证
        //std::ofstream fout;
        //fout.open("testPlane0.txt");
        //for (auto& point : plane)
        //    fout << point[0] << " " << point[1] << " " << point[2] << std::endl;
        //fout.close();
        //std::array<Kernel::Point_3, 8> obb_points;
        //CGAL::oriented_bounding_box(plane, obb_points, CGAL::parameters::use_convex_hull(true));
        //std::cout << obb_points[0][0] << " " << obb_points[0][1] << " " << obb_points[0][2] << std::endl;
        //std::cout << obb_points[1][0] << " " << obb_points[1][1] << " " << obb_points[1][2] << std::endl;
        //std::cout << obb_points[2][0] << " " << obb_points[2][1] << " " << obb_points[2][2] << std::endl;
        //std::cout << obb_points[3][0] << " " << obb_points[3][1] << " " << obb_points[3][2] << std::endl;
        //std::cout << obb_points[4][0] << " " << obb_points[4][1] << " " << obb_points[4][2] << std::endl;
        //std::cout << obb_points[5][0] << " " << obb_points[5][1] << " " << obb_points[5][2] << std::endl;
        //std::cout << obb_points[6][0] << " " << obb_points[6][1] << " " << obb_points[6][2] << std::endl;
        //std::cout << obb_points[7][0] << " " << obb_points[7][1] << " " << obb_points[7][2] << std::endl;
        //}
    }

    std::cout << "Elapsed time in OBB by CGAL: " << timer.time() << std::endl;
}

void ResionGrowingWithJay::calOBB4Plane_byPCL(std::vector<std::vector<size_t>>& outSortedPlan_tris,
                                              std::vector<float>& vMaxArea,
                                              std::vector<float>& vTriArea)
{ 
    vMaxArea.clear();
    vTriArea.clear();

    //首先利用set来顶点去重
    using namespace std;
    yutil::CalTriangArea calArea;
    size_t numPlanes = outSortedPlan_tris.size();
    vector<set<array<int, 2>>> vertexIx(numPlanes);
    for (size_t i = 0; i < numPlanes; ++i)
    {
        float totalTriArea = 0.f;
        auto& plane = outSortedPlan_tris[i];
        for (size_t& triIx : plane)
        {
            auto& tri = this->_vTriangle->at(triIx);
            int& geomIx = tri._geomIndex;
            int& v0Ix = tri._vertexIndexs[0];
            int& v1Ix = tri._vertexIndexs[1];
            int& v2Ix = tri._vertexIndexs[2];

            vertexIx[i].insert({ geomIx,v0Ix });
            vertexIx[i].insert({ geomIx,v1Ix });
            vertexIx[i].insert({ geomIx,v2Ix });

            Vertex& v0 = this->_vvVertex->at(geomIx)[v0Ix];
            Vertex& v1 = this->_vvVertex->at(geomIx)[v1Ix];
            Vertex& v2 = this->_vvVertex->at(geomIx)[v2Ix];
            
            totalTriArea += std::sqrt(calArea.calCGALAreaOFtriangle_square(v0._coor, v1._coor, v2._coor));
        }
        vTriArea.emplace_back(totalTriArea);
    }
    //平面点索引集合转为平面点集合
    std::vector< std::vector <Vec3d>> vvPlanes(numPlanes);
    for (size_t i = 0; i < numPlanes; ++i)
    {
        for (auto& it = vertexIx[i].begin(); it != vertexIx[i].end(); it++)
        {
            const int& geomIx = (*it)[0];
            const int& vIx = (*it)[1];
            Vertex& v = this->_vvVertex->at(geomIx)[vIx];
            vvPlanes[i].emplace_back(v._coor);
        }
    }

    //PCL oriented bounding box function
    CalOBB_byPCA<Vec3d> calObb;
    for (size_t i = 0; i < vvPlanes.size(); ++i)
    {
        std::vector<Vec3d>& plane = vvPlanes[i];        
        calObb.setInputPoints(&vvPlanes[0]);
        calObb.compute();
        Vec3d min_point_OBB, max_point_OBB, position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        bool is_valid = calObb.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        if (!is_valid)
            std::cout << "getOBB is not valid" << std::endl;
        float xSpace = max_point_OBB[0] - min_point_OBB[0];
        float ySpace = max_point_OBB[1] - min_point_OBB[1];
        float zSpace = max_point_OBB[2] - min_point_OBB[2];
        float area1 = xSpace * ySpace;
        float area2 = xSpace * zSpace;
        float area3 = ySpace * zSpace;
        float maxArea = area1 > area2 ? area1 : area2;
        maxArea = maxArea > area3 ? maxArea : area3;
        vMaxArea.emplace_back(maxArea);
        //calObb.ConvertOBB2Array(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB));
    }

    //开发阶段，结果验证
    //std::cout << "Points: " << vvPlanes[0].size() << " ~~ " << position_OBB[0] << 
    //    " " << position_OBB[1] << " " << position_OBB[2] << std::endl;
    //std::cout << obb_points[0][0] << " " << obb_points[0][1] << " " << obb_points[0][2] << std::endl;
    //std::cout << obb_points[1][0] << " " << obb_points[1][1] << " " << obb_points[1][2] << std::endl;
    //std::cout << obb_points[2][0] << " " << obb_points[2][1] << " " << obb_points[2][2] << std::endl;
    //std::cout  << obb_points[3][0] << " " << obb_points[3][1] << " " << obb_points[3][2] << std::endl;
    //std::cout << obb_points[4][0] << " " << obb_points[4][1] << " " << obb_points[4][2] << std::endl;
    //std::cout  << obb_points[5][0] << " " << obb_points[5][1] << " " << obb_points[5][2] << std::endl;
    //std::cout << obb_points[6][0] << " " << obb_points[6][1] << " " << obb_points[6][2] << std::endl;
    //std::cout << obb_points[7][0] << " " << obb_points[7][1] << " " << obb_points[7][2] << std::endl;
}

void ResionGrowingWithJay::calPlaneCofficient(
    std::vector<std::vector<size_t>>& outSortedPlan_tris, 
    std::vector<float>& vPlaneCoffcients) const
{
    vPlaneCoffcients.clear();
    vPlaneCoffcients.resize(outSortedPlan_tris.size());

    ///-. 转换CGAL triangle    
    int nPlanes = outSortedPlan_tris.size();
//#pragma omp parallel for
    for (int i = 0; i < nPlanes; ++i)
    {
        auto& plane = outSortedPlan_tris[i];
        std::vector<Kernel::Triangle_3> plane_cgal;
        for (size_t& triIx : plane) {
            ///转为cgal triangle
            Triangle tri = _vTriangle->at(triIx);
            auto geomIx = tri._geomIndex;
            auto v1 = _vvVertex->at(geomIx)[tri._vertexIndexs[0]];
            auto v2 = _vvVertex->at(geomIx)[tri._vertexIndexs[1]];
            auto v3 = _vvVertex->at(geomIx)[tri._vertexIndexs[2]];
            Kernel::Point_3 p1(v1._coor[0], v1._coor[1], v1._coor[2]);
            Kernel::Point_3 p2(v2._coor[0], v2._coor[1], v2._coor[2]);
            Kernel::Point_3 p3(v3._coor[0], v3._coor[1], v3._coor[2]);
            Kernel::Triangle_3 tri_cgal(p1, p2, p3);
            plane_cgal.emplace_back(tri_cgal);
        }
        Kernel::Plane_3 plane3;
        //tag：0表示使用三角形顶点拟合；1表示使用三角形边拟合；2表示使用三角形来拟合
        //试验表明：2的效果最好，2021/6/8
        float coffient = CGAL::linear_least_squares_fitting_3(
            plane_cgal.begin(), plane_cgal.end(), plane3, CGAL::Dimension_tag<2>());
        vPlaneCoffcients[i] = coffient;
    }
}