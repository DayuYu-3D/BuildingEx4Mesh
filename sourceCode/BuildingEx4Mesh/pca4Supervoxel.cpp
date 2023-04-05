#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>

#include "codelibrary/base/log.h"
#include "codelibrary/visualization/color/rgb32_color.h"

#include "PCH.h"

#include "yutils/Cfg.h"
#include "yutils/FileNameUtils.h"
#include "yutils/cmdline.h"
#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/trianglereader.h"
#include "meshpaser/vertex.h"
#include "meshpaser/calTrianglsCenter.h"
#include "meshpaser/caltriangleuvcoords.h"

// typedef float FT;
// typedef CGAL::Cartesian<FT> Kernel;
// typedef Kernel::Line_3 Line_3;
// typedef Kernel::Plane_3 Plane_3;
// typedef Kernel::Point_3 Point_3;
// typedef Kernel::Triangle_3 Triangle_3;

// int pca4SuperVoxel(int argc, char *argv[])
// {
//     LOG_ON(INFO);

//     cmdline::parser a;
//     a.add<std::string>("cfgPath", 'f', "The path of params.cfg. Default: ../params.cfg",
//                        false, "../../params.cfg");
//     a.parse_check(argc, argv);
//     std::string fp_cfg = a.get<std::string>("cfgPath"); // 获取输入的参数值

//     auto startT = std::chrono::system_clock::now(); //获取开始时间
//     //-------------------------------------------------------------------------
//     //-----------------------Step0: 参数解析------------------------------------
//     Cfg cfg;
//     std::string triangle_path;
//     cfg.readConfigFile(fp_cfg.c_str(), "triangle_path", triangle_path);

//     //-------------------------------------------------------------------------
//     //-----------------------Step1: 网格文件解析---------------------------------
//     std::vector<Triangle> vTriangle;           ///大量内存占用
//     std::vector<std::vector<Vertex>> vvVertex; ///大量内存占用
//     std::vector<int> vSupervoxelIndex;         ///较大内存占用
//     std::vector<std::string> vTexture;
//     LOG(INFO) << "Reading points from " << triangle_path.c_str() << "...";
//     TriangleReader triReader(triangle_path);
//     if (!triReader.readSupervoxel(vTriangle, vvVertex, vTexture, vSupervoxelIndex))
//     {
//         std::cerr << "Please check if " << triangle_path.c_str() << " is exist." << std::endl;
//         return -1;
//     }
//     if (vSupervoxelIndex.size() != vTriangle.size())
//     {
//         std::cerr << "vSupervoxelIndex.size()!=vTriangle.size()" << std::endl;
//         return -1;
//     }

//     //-------------------------------------------------------------------------
//     //-----------------------Step2: 转换CGAL和超体素----------------------------
//     int nSupervoxels = *(std::max_element(vSupervoxelIndex.begin(), vSupervoxelIndex.end())) + 1;
//     std::cout << "convert triangle to cgal triangle" << std::endl;
//     std::vector<std::vector<Triangle_3>> vSuperVoxel(nSupervoxels); ///大量内存占用
//     std::vector<std::vector<int>> vTri3Index(nSupervoxels);         ///大量内存占用
//     int nTris = vTriangle.size();
// #pragma omp parallel for
//     for (int i = 0; i < nTris; ++i)
//     {
//         ///转为cgal triangle
//         auto tri = vTriangle[i];
//         auto geomIx = tri._geomIndex;
//         auto v1 = vvVertex[geomIx][tri._vertexIndexs[0]];
//         auto v2 = vvVertex[geomIx][tri._vertexIndexs[1]];
//         auto v3 = vvVertex[geomIx][tri._vertexIndexs[2]];
//         Point_3 p1(v1._coor[0], v1._coor[1], v1._coor[2]);
//         Point_3 p2(v2._coor[0], v2._coor[1], v2._coor[2]);
//         Point_3 p3(v3._coor[0], v3._coor[1], v3._coor[2]);
//         Triangle_3 tri3(p1, p2, p3);

//         int ix = vSupervoxelIndex[i];
// #pragma omp critical
//         {
//             vSuperVoxel[ix].push_back(tri3);
//             vTri3Index[ix].push_back(i);
//         }
//     }

//     //-------------------------------------------------------------------------
//     //-----------------------Step3: PCA分析----------------------------
//     std::cout << "pca ..." << std::endl;
//     std::vector<FT> vCoffient(nSupervoxels);
//     std::vector<Plane_3> vPlane;
// #pragma omp parallel for
//     for (int i = 0; i < nSupervoxels; ++i)
//     {
//         Plane_3 plane;
//         auto supervoxel = vSuperVoxel[i];
//         FT coffient = CGAL::linear_least_squares_fitting_3(
//             supervoxel.begin(), supervoxel.end(), plane, CGAL::Dimension_tag<2>());
//         vCoffient[i] = coffient;
//     }
//     std::cout << "pca done" << std::endl;
//     std::ofstream outfile;
//     outfile.open("xxxx.dat");
//     for (auto cof : vCoffient)
//     {
//         outfile << cof << std::endl;
//     }
//     outfile.close();

//     //-------------------------------------------------------------------------
//     //-----------------------Step4: 过滤---------------------------------------
//     std::vector<unsigned int> vSaveIx;
//     for (int i = 0; i < nSupervoxels; ++i)
//     {
//         if (vCoffient[i] > 0.8)
//         {
//             for (auto tri3Ix : vTri3Index[i])
//             {
//                 vSaveIx.push_back(tri3Ix);
//             }
//         }
//     }

//     //-------------------------------------------------------------------------
//     //-----------------------Step5: 保存过滤后的超体素----------------------------
//     std::string strFP = yUtils::getFilePath(triangle_path);
//     std::string strFp_PCA = strFP + "/triangles_pca.txt";
//     triReader.writeTrianglesData(vTriangle, vSaveIx, strFp_PCA);
//     return 0;
// }
