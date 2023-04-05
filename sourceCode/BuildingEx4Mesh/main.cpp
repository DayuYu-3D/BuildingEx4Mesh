#include <string>
#include <fstream>
#include <random>
#include <limits.h>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <array>

#include "PCH.h"

#include "yutils/Cfg.h"
#include "yutils/FileNameUtils.h"
#include "yutils/cmdline.h"

#include "meshpaser/trianglereader.h"
#include "meshpaser/calTrianglsCenter.h"
#include "meshpaser/caltriangleuvcoords.h"

#include "advancingFronReconstruct.h"
#include "regionGrowing4Points.h"
#include "buildExtract.h"
#include "topologyConnect.h"

#include "jaya/JayaClassic.h"
#include "jaya/JayaSelfAdaptive.h"
#include "regionGrowingWithJaya.h"

int main(int argc, char *argv[])
{
    std::wcout.imbue(std::locale(""));   

    cmdline::parser a;
    a.add<std::string>("cfgPath", 'f', "The path of params.cfg. Default: ../params.cfg",
                       false, "../../../../params.cfg");
    a.parse_check(argc, argv);
    std::string fp_cfg = a.get<std::string>("cfgPath"); // 获取输入的参数值

    //-------------------------------------------------------------------------
    Cfg cfg;
    std::string triangle_path;
    cfg.readConfigFile(fp_cfg.c_str(), "triangle_path", triangle_path);
    std::string groundTriangle_path;
    cfg.readConfigFile(fp_cfg.c_str(), "groundTriangle_path", groundTriangle_path);
    std::string smaxDistanceToPlane;
    cfg.readConfigFile(fp_cfg.c_str(), "maxDistanceToPlane", smaxDistanceToPlane);
    std::string smaxAcceptAngle;
    cfg.readConfigFile(fp_cfg.c_str(), "maxAcceptAngle", smaxAcceptAngle);
    std::string sminRegionSize;
    cfg.readConfigFile(fp_cfg.c_str(), "minRegionSize", sminRegionSize);
    std::string sk;
    cfg.readConfigFile(fp_cfg.c_str(), "k", sk);
    float maxDistanceToPlane = atof(smaxDistanceToPlane.c_str());
    float maxAcceptAngle = atof(smaxAcceptAngle.c_str());
    size_t minRegion_size = atoi(sminRegionSize.c_str());
    size_t k = atoi(sk.c_str());  
    std::cout << "maxDistanceToPlane: " << maxDistanceToPlane << std::endl;
    std::cout << "maxAcceptAngle: " << maxAcceptAngle << std::endl;
    std::cout << "minRegion_size: " << minRegion_size << std::endl;
    std::cout << "k: " << k << std::endl;

    //-------------------------------------------------------------------------
    if (triangle_path == "")
    {
        std::cerr << "! _triPath = null " << std::endl;
    }
    std::vector<Triangle> _vTriangle;           ///大量内存占用
    std::vector<std::vector<Vertex>> _vvVertex; ///大量内存占用
    std::vector<int> _vSupervoxelIndex;         
    std::vector<std::string> vTexture;
    std::cout << "* Reading points from " << triangle_path.c_str() << "...\n";
    TriangleReader triReader(triangle_path);
    if (!triReader.readSupervoxel(_vTriangle, _vvVertex, vTexture, _vSupervoxelIndex))
    {
        std::cerr << "! Please check if " << triangle_path.c_str() << " is exist.\n" ;
        return -1;
    }
    if (_vSupervoxelIndex.size() != _vTriangle.size())
    {
        std::cerr << "! vSupervoxelIndex.size()!=vTriangle.size()" << std::endl;
        return -1;
    }
    triReader.getAbsoutePath(triangle_path, vTexture);

    TopologyConnect topoCct;
    topoCct.createConnectByVertex(&_vTriangle, &_vvVertex);

    //-------------------------------------------------------------------------
    //-----------------------test: 区域增长----------------------------
    //ResionGrowingWithJay regionGwithJAYA(&_vTriangle, &_vvVertex, &_vSupervoxelIndex);
    //regionGwithJAYA.setParams(k, maxDistanceToPlane, maxAcceptAngle, minRegion_size);
    //std::vector<double> solution = { maxDistanceToPlane,maxAcceptAngle,(double)minRegion_size};
    //regionGwithJAYA.getValue(solution);
    //return -2;

    //-------------------------------------------------------------------------
    //-----------------------test: 区域增长withjaya----------------------------
    if (0)
    {
        auto startT11 = std::chrono::system_clock::now(); //获取开始时间
        std::cout << "!!!!!!!!: Jaya_start ... ...: " << std::endl;

        srand(time(NULL));
        Variable maxDis(2, 10); 
        Variable maxAngle(10, 45);
        ///Variable minNums(2, 15); 
        ///minNums.setType(2);
        std::vector<Variable> variables = { maxDis,maxAngle };
        ResionGrowingWithJay regionGwithJAYA(&_vTriangle, &_vvVertex, &_vSupervoxelIndex);
        //HimmelblauFunc hmfunc;
        JayaSelfAdaptive jaya(variables, false, &regionGwithJAYA);
        BestAndWorst beworst = jaya.run(20).getBestAndWorstValue();
        std::cout << "best: " << beworst.bestV << std::endl;
        std::cout << "worst: " << beworst.worstV << std::endl;
        std::cout << "bestSolution: " << beworst.bestSolution._solution[0] << " " << beworst.bestSolution._solution[1] << std::endl;
        std::cout << "=========\n";
        
        auto entT = std::chrono::system_clock::now();
        auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT11).count();
        std::cout << "!!!!!!!!: Jaya_end ... ...: " << (double)ut / 1000 << "s" << std::endl;

        return -2;
    }

    bool bSaveTempResult = false;

    auto startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "!!!!!!!!: Region Growing_start ... ...: " << std::endl;
    //-------------------------------------------------------------------------
    std::vector<std::vector<size_t>> outSortedPlan_tris;    
    ResionGrowing4Points regionG(&_vTriangle,&_vvVertex,&_vSupervoxelIndex);
    regionG.setParams(k,maxDistanceToPlane,maxAcceptAngle,minRegion_size);
    regionG.regionGrowing();
    //regionG.savePointsOfSupCenterByRG(triangle_path);
    // regionG.saveSuperVoxelByRG(triangle_path);
    regionG.saveOnlyPlaneSuperVoxelByRG(triangle_path);
    //regionG.saveOnlyNotPlaneSuperVoxelByRG(triangle_path);        
    regionG.getSortedPlanarTriangle(outSortedPlan_tris,false);
    auto entT = std::chrono::system_clock::now();
    auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "!!!!!!!!: Region Growing_end ... ...: " << (double)ut / 1000 << "s" << std::endl;
    if (bSaveTempResult)
    {
        std::cout << "* Writing _onlyPlaneSorted_color_ByRG.txt ...\n";
        std::string saveP1 = yUtils::getNameLessExtension(triangle_path)
            + "_onlyPlaneSorted_color_ByRG.txt";
        regionG.writePlanerTriangles(outSortedPlan_tris, saveP1);
    }
    //-------------------------------------------------------------------------
    BuildingExtract buildEt(_vTriangle, _vvVertex);
    buildEt.setTexture(vTexture);
    //buildEt.setPlanerTriangles(&outSortedPlan_tris     );

    //std::vector<std::vector<size_t>> outFilteredPlan_tris_2;
    //buildEt.filteringPlaneByPCA(outSortedPlan_tris, outFilteredPlan_tris_2,0.92);
    ////std::cout << "* Writing plane superVoxels.txt ...\n";
    ////std::string saveP12 = yUtils::getNameLessExtension(triangle_path) + "_onlyPlaneSorted_color_ByRG_pca.txt";
    ////regionG.writePlanerTriangles(outFilteredPlan_tris_2, saveP12);
    //return -1;

    startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "!!!!!!!!: Cull small planes_start ... ...: " << std::endl;
    std::vector<std::vector<size_t>> outFilteredPlan_tris4;
    buildEt.filteringSmallePlaneByArea(outSortedPlan_tris, outFilteredPlan_tris4, 10);
    
    if (bSaveTempResult)
    {
        std::cout << "* Writing _onlyPlaneSorted_color_ByRG_4.txt ...\n";
        std::string saveP1 = yUtils::getNameLessExtension(triangle_path)
            + "_onlyPlaneSorted_color_ByRG_4.txt";
        regionG.writePlanerTriangles(outFilteredPlan_tris4, saveP1);
    }
    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "!!!!!!!!: Cull small planes_end ... ...: " << (double)ut / 1000 << "s" << std::endl;

    if (groundTriangle_path == "")
    {
        std::cerr << "! groundTriangle_path = null\n";
    }
    
    std::cout << "* Reading ground mesh from " << groundTriangle_path.c_str() << "...\n";
    TriangleReader triReader11(groundTriangle_path);
    //_vvVertex.clear(); vTexture.clear(); 
    std::vector<Triangle> _vTriangle_ground;  ///较大内存占用
    std::vector<std::vector<Vertex>> vvVertex_ground; ///大量内存占用
    std::vector<std::string> vTexture_ground;
    if (!triReader11.readTriangleData(_vTriangle_ground, vvVertex_ground, vTexture_ground))
    {
        std::cerr << "! Please check if " << groundTriangle_path.c_str() << " is exist.\n";
        return -1;
    }
    startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "!!!!!!!!: Cull low planes_start ... ...: " << std::endl;
    std::vector<std::vector<size_t>> outFilteredPlan_tris_5;
    buildEt.filteringPlaneByAGL_Height_2(_vTriangle_ground, vvVertex_ground,
        outFilteredPlan_tris4, outFilteredPlan_tris_5, 2); 
    
    if (bSaveTempResult)
    {
        std::cout << "* Writing _onlyPlaneSorted_color_ByRG_5 ...\n";
        std::string saveP1 = yUtils::getNameLessExtension(triangle_path)
            + "_onlyPlaneSorted_color_ByRG_5.txt";
        regionG.writePlanerTriangles(outFilteredPlan_tris_5, saveP1);
    }
    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "!!!!!!!!: Cull low planes_end ... ...: " << (double)ut / 1000 << "s" << std::endl;

    startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "!!!!!!!!: Cull green planes_start ... ...: " << std::endl;
    std::vector<std::vector<size_t>> outFilteredPlan_tris_6;
    buildEt.filteringPlaneByExG_EXR(outFilteredPlan_tris_5, outFilteredPlan_tris_6, 0);
    std::cout << "* Writing _onlyPlaneSorted_color_ByRG_6 ...\n";
    if (bSaveTempResult) {
        std::string saveP1 = yUtils::getNameLessExtension(triangle_path)
            + "_onlyPlaneSorted_color_ByRG_6.txt";
        regionG.writePlanerTriangles(outFilteredPlan_tris_6, saveP1);
    }

    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "!!!!!!!!: Cull green planes_end ... ...: " << (double)ut / 1000 << "s" << std::endl;

    //std::vector<std::vector<size_t>> outFilteredPlan_tris_2;
    //buildEt.filteringPlaneByColor(outFilteredPlan_tris, outFilteredPlan_tris_2, 60);
    //std::cout << "* Writing plane superVoxels.txt ...\n";
    //saveP1 = yUtils::getNameLessExtension(triangle_path) + "_onlyPlaneSorted_color_ByRG_4.txt";
    //regionG.writePlanerTriangles(outFilteredPlan_tris_2, saveP1);

    //std::vector<std::vector<size_t>> outFilteredPlan_tris_2;
    //buildEt.filteringPlaneByPCA(outFilteredPlan_tris, outFilteredPlan_tris_2,0.92);
    //std::cout << "* Writing plane superVoxels.txt ...\n";
    //saveP1 = yUtils::getNameLessExtension(triangle_path) + "_onlyPlaneSorted_color_ByRG_3.txt";
    //regionG.writePlanerTriangles(outFilteredPlan_tris_2, saveP1);

    //std::vector<std::vector<size_t>> outFilteredPlan_tris_2;
    //buildEt.filteringPlaneByAreaRatio_tris(outFilteredPlan_tris, outFilteredPlan_tris_2,0.2,0.8);
    //std::cout << "* Writing plane superVoxels.txt ...\n";
    //saveP1 = yUtils::getNameLessExtension(triangle_path) + "_onlyPlaneSorted_color_ByRG_2.txt";
    //regionG.writePlanerTriangles(outFilteredPlan_tris_2, saveP1);

    //-------------------------------------------------------------------------
    startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "!!!!!!!!: Greedy recovery_start ... ...: " << std::endl;
    std::vector<size_t> vtriangle_final;
    topoCct.bruteSearchByDFS(&outFilteredPlan_tris_6, &vtriangle_final, &_vTriangle, &_vvVertex);
    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "!!!!!!!!: Greedy recovery_start ... ...: " << (double)ut / 1000 << "s" << std::endl;

    std::string saveFinal = yUtils::getNameLessExtension(triangle_path) 
        + "_onlyPlaneSorted_color_ByRG_final.txt";
    triReader.writeTrianglesData(_vTriangle,vtriangle_final, saveFinal);

    std::vector<size_t> vtriangle_ix_nonBuilding;
    std::vector<size_t> vtriangle_ix(_vTriangle.size());
    std::iota(vtriangle_ix.begin(), vtriangle_ix.end(), 0);
    std::set_difference(vtriangle_ix.begin(), vtriangle_ix.end(),
        vtriangle_final.begin(), vtriangle_final.end(),
        std::inserter(vtriangle_ix_nonBuilding, vtriangle_ix_nonBuilding.begin()));
    std::string saveNonBuilding = yUtils::getNameLessExtension(triangle_path)
        + "_onlyPlaneSorted_color_ByRG_nonbuilding.txt";
    triReader.writeTrianglesData(_vTriangle, vtriangle_ix_nonBuilding, saveNonBuilding);

    std::wcout << (L"执行完毕\n");
    return 0;
}
