#include "regionGrowing4Points.h"

#include <iostream>
#include <random>
#include <fstream>

#include "codelibrary/geometry/io/xyz_io.h"
#include "codelibrary/visualization/color/rgb32_color.h"

#include "meshpaser/calTrianglsCenter.h"
#include "meshpaser/caltriangleuvcoords.h"

#include "yutils/FileNameUtils.h"

namespace {
    // Type declarations.
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    Kernel::Compute_squared_distance_3; Kernel::Rep_tag;
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

    // Define an insert iterator.
    struct Insert_point_colored_by_region_index
    {
        using Color_map =
            typename Output_range::template Property_map<unsigned char>;
        using Index_map =
            typename Output_range::template Property_map<size_t>;

        using argument_type = Indices;
        using result_type = void;

        const Input_range& m_input_range;
        const Point_map m_point_map;
        Output_range& m_output_range;
        std::size_t& m_number_of_regions;

        Color_map m_red, m_green, m_blue;
        Index_map m_index;
        Index_map m_index_supervoxel;
        Index_map m_index_plane;

        Insert_point_colored_by_region_index(
            const Input_range& input_range,
            const Point_map point_map,
            Output_range& output_range,
            std::size_t& number_of_regions) : m_input_range(input_range),
            m_point_map(point_map),
            m_output_range(output_range),
            m_number_of_regions(number_of_regions),
            m_index_plane(0)
        {
            m_red =
                m_output_range.template add_property_map<unsigned char>("red", 0).first;
            m_green =
                m_output_range.template add_property_map<unsigned char>("green", 0).first;
            m_blue =
                m_output_range.template add_property_map<unsigned char>("blue", 0).first;
            m_index_supervoxel =
                m_output_range.template add_property_map<size_t>("index_supervoxel", 0).first;
            m_index_plane =
                m_output_range.template add_property_map<size_t>("index_plane", 0).first;
        }

        result_type operator()(const argument_type& region)
        {

            CGAL::Random rand(static_cast<unsigned int>(m_number_of_regions));
            const unsigned char r =
                static_cast<unsigned char>(64 + rand.get_int(0, 192));
            const unsigned char g =
                static_cast<unsigned char>(64 + rand.get_int(0, 192));
            const unsigned char b =
                static_cast<unsigned char>(64 + rand.get_int(0, 192));

            for (const std::size_t index : region)
            {
                const auto& key = *(m_input_range.begin() + index);

                const Point_3& point = get(m_point_map, key);
                const auto it = m_output_range.insert(point);

                m_red[*it] = r;
                m_green[*it] = g;
                m_blue[*it] = b;
                m_index_supervoxel[*it] = index;
                m_index_plane[*it] = m_number_of_regions;
            }
            ++m_number_of_regions;
        }
    }; // Insert_point_colored_by_region_index
}

ResionGrowing4Points::ResionGrowing4Points(std::string strPath)
    :  _triPath(strPath),
       _vTriangle(nullptr),
       _vvVertex(nullptr),
       _vSupervoxelIndex(nullptr),
       _numPlaneFound(0),
       _inputPointSet(true), //true��ʾ����������
    _neighbor_query(nullptr)
{
    this->perseTrianglePath();
    this->setParams();
}

ResionGrowing4Points::ResionGrowing4Points(
    std::vector<Triangle>* vTriangle,
    std::vector<std::vector<Vertex>>* vvVertex,
    std::vector<int>* vSupervoxelIndex)
    : _vTriangle(vTriangle),
    _vvVertex(vvVertex),
    _vSupervoxelIndex(vSupervoxelIndex),
    _triPath(""),
    _numPlaneFound(0),
    _inputPointSet(true), //true��ʾ����������
    _neighbor_query(nullptr)
{
    this->setParams();
}

ResionGrowing4Points::~ResionGrowing4Points()
{
    if (_triPath != "") //��ֹ�ڶ������캯�����������ⲿָ�뱻ɾ��
    {
        if (!_vTriangle)
            delete _vTriangle;
        if (!_vvVertex)
            delete _vvVertex;
        if (!_vSupervoxelIndex)
            delete _vSupervoxelIndex;
    }
    if (_neighbor_query != nullptr)
    {
        delete _neighbor_query;
        _neighbor_query = nullptr;
    }
}

void ResionGrowing4Points::setParams(
    size_t k, float maxDistanceToPlane, float maxAcceptAngle, size_t minRegion_size)
{
    _k = k;
    _maxDistanceToPlane = maxDistanceToPlane;
    _maxAcceptAngle = maxAcceptAngle;
    _minRegion_size = minRegion_size;
}

void ResionGrowing4Points::savePointsOfSupCenterByRG(std::string savePath) const
{
    if (_outputPointSet.size())
    {
        std::string saveP1 = yUtils::getNameLessExtension(savePath) + "_regions_points_color.ply";
        std::ofstream out(saveP1);
        out << _outputPointSet;
        std::cout << "* found regions are saved in " << saveP1 << std::endl;
        out.close();
    }
    else {
        std::cerr << "! save error due to _outputPointSet is null\n";
    }
}

void ResionGrowing4Points::saveSuperVoxelByRG(std::string savePath) const
{
    if (!_outputPointSet.size()) {
        std::cerr << "! save error due to _outputPointSet is null\n";
        return;
    }
    //-------------------------------------------------------------------------
    //-----------------------Step4: ����ָ���-=------------------------------
    size_t nTris = _vTriangle->size();
    std::vector<std::vector<uint8_t>> vTriColor(nTris);
    ///Ϊ�������������򱣴���ɫ
    auto it = _outputPointSet.begin();
    while (it != _outputPointSet.end())
    {
        size_t index = _outputPointSet.property_map<size_t>("index_supervoxel").first[*it];
        unsigned char red = _outputPointSet.property_map<unsigned char>("red").first[*it];
        unsigned char green = _outputPointSet.property_map<unsigned char>("green").first[*it];
        unsigned char blue = _outputPointSet.property_map<unsigned char>("blue").first[*it];
        std::vector<uint8_t> rgb(3);
        rgb[0] = red, rgb[1] = green;
        rgb[2] = blue;
        for (size_t k = 0; k < _vvSupervoxelIndex[index].size(); k++)
        {
            size_t triIX = _vvSupervoxelIndex[index][k];
            vTriColor[triIX] = rgb;
        }
        ++it;
    }

    ///��ɫֵ������룬Ϊunassigned_points
    { //������
        std::mt19937 random;
        // size_t nC = _unassignedItems.size();
        for (auto unassigend_inedx : _unassignedItems)
        {
            auto color = cl::RGB32Color(random());
            std::vector<uint8_t> rgb(3);
            rgb[0] = color.red(), rgb[1] = color.green();
            rgb[2] = color.blue();
            for (size_t k = 0; k < _vvSupervoxelIndex[unassigend_inedx].size(); k++)
            {
                size_t triIX = _vvSupervoxelIndex[unassigend_inedx][k];
                vTriColor[triIX] = rgb;
            }
        }
    }
    ///д����������
    std::cout << "* Writing superVoxels.txt ...\n";
    std::string saveP1 = yUtils::getNameLessExtension(savePath) + "_color_ByRG.txt";
    TriangleReader triReader("");
    triReader.writeTrianglesData(*_vTriangle, vTriColor, saveP1);
    std::cout  << "* The file has been saved to" << saveP1 <<std::endl;
}

void ResionGrowing4Points::saveOnlyPlaneSuperVoxelByRG(std::string savePath) const
{
    if (!_outputPointSet.size()) {
        std::cerr << "! save error due to _outputPointSet is null\n";
        return;
    }
    //-------------------------------------------------------------------------
    //-----------------------Step4: ����ָ���-=------------------------------
    size_t nTris = _vTriangle->size();
    std::vector<std::vector<uint8_t>> vTriColor;//reseave
    std::vector<Triangle> vTriangle_temp;
    vTriColor.reserve(nTris);
    vTriangle_temp.reserve(nTris);
    ///Ϊ�������������򱣴���ɫ
    auto it = _outputPointSet.begin();
    while (it != _outputPointSet.end())
    {
        size_t index = _outputPointSet.property_map<size_t>("index_supervoxel").first[*it];
        unsigned char red = _outputPointSet.property_map<unsigned char>("red").first[*it];
        unsigned char green = _outputPointSet.property_map<unsigned char>("green").first[*it];
        unsigned char blue = _outputPointSet.property_map<unsigned char>("blue").first[*it];
        std::vector<uint8_t> rgb(3);
        rgb[0] = red, rgb[1] = green;
        rgb[2] = blue;
        for (size_t k = 0; k < _vvSupervoxelIndex[index].size(); k++)
        {
            size_t triIX = _vvSupervoxelIndex[index][k];
            //vTriColor[triIX] = rgb;
            vTriColor.emplace_back(rgb);
            Triangle tirangle= _vTriangle->at(triIX);
            vTriangle_temp.emplace_back(tirangle);
        }
        ++it;
    }
    vTriColor.shrink_to_fit();
    vTriangle_temp.shrink_to_fit();
    return;
    ///д����������
    std::cout << "* Writing OnlyPlane superVoxels.txt ...\n";
    std::string saveP1 = yUtils::getNameLessExtension(savePath) + "_onlyPlane_color_ByRG.txt";
    TriangleReader triReader("");
    triReader.writeTrianglesData(vTriangle_temp, vTriColor, saveP1);
    std::cout << "* The file has been saved to" << saveP1 << std::endl;
}

void ResionGrowing4Points::saveOnlyNotPlaneSuperVoxelByRG(std::string savePath) const
{
    if (!_outputPointSet.size()) {
        std::cerr << "! save error due to _outputPointSet is null\n";
        return;
    }
    //-------------------------------------------------------------------------
    //-----------------------Step4: ����ָ���-=------------------------------
    size_t nTris = _vTriangle->size();
    std::vector<std::vector<uint8_t>> vTriColor;//reseave
    std::vector<Triangle> vTriangle_temp;
    vTriColor.reserve(nTris);
    vTriangle_temp.reserve(nTris);
    ///��ɫֵ������룬Ϊunassigned_points
    { //������
        std::mt19937 random;
        // size_t nC = _unassignedItems.size();
        for (auto unassigend_inedx : _unassignedItems)
        {
            auto color = cl::RGB32Color(random());
            std::vector<uint8_t> rgb(3);
            rgb[0] = color.red(), rgb[1] = color.green();
            rgb[2] = color.blue();
            for (size_t k = 0; k < _vvSupervoxelIndex[unassigend_inedx].size(); k++)
            {
                size_t triIX = _vvSupervoxelIndex[unassigend_inedx][k];
                vTriColor.emplace_back(rgb);
                Triangle tirangle = _vTriangle->at(triIX);
                vTriangle_temp.emplace_back(tirangle);
            }
        }
    }
    vTriColor.shrink_to_fit();
    vTriangle_temp.shrink_to_fit();
    ///д����������
    std::cout << "* Writing OnlyNotPlane superVoxels.txt ...\n";
    std::string saveP1 = yUtils::getNameLessExtension(savePath) + "_onlyNonePlane_color_ByRG.txt";
    TriangleReader triReader("");
    triReader.writeTrianglesData(vTriangle_temp, vTriColor, saveP1);
    std::cout << "* The file has been saved to" << saveP1 << std::endl;
}

void ResionGrowing4Points::perseTrianglePath()
{
    if (_triPath == "")
    {
        std::cerr << "! _triPath = null " << std::endl;
    }

    //-------------------------------------------------------------------------
    //-----------------------Step1: �����ļ�����---------------------------------
    _vTriangle =new std::vector<Triangle> ;           ///�����ڴ�ռ��
    _vvVertex = new std::vector<std::vector<Vertex>> ; ///�����ڴ�ռ��
    _vSupervoxelIndex = new std::vector<int> ; ///�ϴ��ڴ�ռ��
    std::vector<std::string> vTexture;
    std::cout << "* Reading points from " << _triPath.c_str() << "...\n";
    TriangleReader triReader(_triPath);
    if (!triReader.readSupervoxel(*_vTriangle, *_vvVertex, vTexture, *_vSupervoxelIndex))
    {
        std::cerr << "! Please check if " << _triPath.c_str() << " is exist." << std::endl;
        return;
    }
    if (_vSupervoxelIndex->size() != _vTriangle->size())
    {
        std::cerr << "! vSupervoxelIndex.size()!=vTriangle.size()" << std::endl;
        return;
    }
}

void ResionGrowing4Points::convertMesh2PointsWithNormal() 
{
    //_inputPointSet.clear();��������

    /// supervoxelת���ɶ�ά   
    int numSupervoxel = *std::max_element(_vSupervoxelIndex->begin(), _vSupervoxelIndex->end());
    _vvSupervoxelIndex.clear();
    size_t ntemp = (size_t)numSupervoxel + 1;
    _vvSupervoxelIndex.resize(ntemp);
    std::cout << "*��Num of supervoxel is: " << numSupervoxel + 1 << std::endl;
    for (int i = 0; i < _vSupervoxelIndex->size(); ++i)
    {
        int ix = _vSupervoxelIndex->at(i);
        _vvSupervoxelIndex[ix].emplace_back(i);
    }

    ///���������ĵ�
    std::vector<Vec3d> vCenterPointsofSuperv;
    for (auto superV : _vvSupervoxelIndex)
    {
        Vec3d vec3T(0, 0, 0);
        for (int triIx : superV)
        {
            auto tri = _vTriangle->at(triIx);
            auto p1 = _vvVertex->at(tri._geomIndex)[tri._vertexIndexs[0]]._coor;
            auto p2 = _vvVertex->at(tri._geomIndex)[tri._vertexIndexs[1]]._coor;
            auto p3 = _vvVertex->at(tri._geomIndex)[tri._vertexIndexs[2]]._coor;
            vec3T += (p1 + p2 + p3);
        }
        vec3T = vec3T / (superV.size() * 3);
        vCenterPointsofSuperv.push_back(vec3T);
    }

    //if (0)
    //{
    //    ///���泬�������ĵ㵽�ļ�
    //    std::cout << "* Writing center of supervoxel...\n";
    //    std::string saveP = yUtils::getFilePath(_triPath) + "/points_superVCenter.txt";
    //    TriangleReader triReader("");
    //    triReader.writePointClouds(vCenterPointsofSuperv, saveP);
    //    std::cout << "* The file has been saved to\n" << saveP;
    //}

    ///���������ĵ�תΪcgal::point_3
    ///�����ص�ƽ��������    
    for (size_t i = 0; i < _vvSupervoxelIndex.size(); ++i)
    {
        float nomalX = 0.f, nomalY = 0.f, nomalZ = 0.f;
        auto superV = _vvSupervoxelIndex[i];
        for (int triIx : superV)
        {
            auto tri = _vTriangle->at(triIx);
            auto normal = tri._normal;
            nomalX += normal[0], nomalY += normal[1], nomalZ += normal[2];
        }
        nomalX /= superV.size(), nomalY /= superV.size(), nomalZ /= superV.size();
        Vector_3 n1(nomalX, nomalY, nomalZ);
        auto pt = vCenterPointsofSuperv[i];
        Point_3 p1(pt[0], pt[1], pt[2]);
        _inputPointSet.insert(p1, n1);
    }

    if (0)  //��������ߵĵ�
    {
        std::string saveP1 = yUtils::getNameLessExtension(_triPath) + "_regions_points_normals.ply";
        std::ofstream out(saveP1);
        out << _inputPointSet;
        std::cout << "* points with normal saved in " << saveP1 << std::endl;
        out.close();
    }
}

void ResionGrowing4Points::regionGrowing()
{

    if (!_inputPointSet.size())
    {
        //-------------------------------------------------------------------------
        //-----------------------Step2: ��ʽת��---------------------------------
        this->convertMesh2PointsWithNormal();        
        std::cout << "* loaded " << _inputPointSet.size() << " points with normals \n";
    }


    //-------------------------------------------------------------------------
    //-----------------------Step3: һ���Գ�ʼ������---------------------------------
    if (_neighbor_query == nullptr)//��ʱ17������
    {
        // ����Neighbor_queryʵ����only��ѡһ
        ///case1�����ڵ��Ƶ�KNN�ھӲ�ѯ
        //Neighbor_query neighbor_query(_inputPointSet, _k, _inputPointSet.point_map());
        ///case2���������˹�ϵ���ھӲ�ѯ
        _neighbor_query = new Neighbor_query(_vTriangle, _vvVertex, _vSupervoxelIndex, &_vvSupervoxelIndex); //��ʱ����
    }
    CGAL::Timer timer;
    timer.start();
    // ����Region_type���Region_growing���ʵ��
    Region_type region_type(
        _inputPointSet,
        _maxDistanceToPlane, _maxAcceptAngle, _minRegion_size,
        _inputPointSet.point_map(), _inputPointSet.normal_map());
    Region_growing regionGrowing(_inputPointSet, *_neighbor_query, region_type);

    // Run the algorithm.
    _numPlaneFound = 0;
    _unassignedItems.clear();
    _outputPointSet.clear();

    Insert_point_colored_by_region_index inserter(
        _inputPointSet, _inputPointSet.point_map(), _outputPointSet, _numPlaneFound);

    regionGrowing.detect(boost::make_function_output_iterator(inserter));

    //// Print the number of found regions.
    //std::cout << "Elapsed time in regiong growing: " << timer.time() << std::endl;
    //std::cout << "* found planes regions: " << _numPlaneFound << std::endl;

    //// Get all unassigned items.
    //regionGrowing.unassigned_items(std::back_inserter(_unassignedItems));
    //// Print the number of unassigned items.
    //std::cout << "* " << _unassignedItems.size() << " points do not belong to any region \n";
    
    //-------------------------------------------------------------------------
    //-----------------------Step3: region growing-----------------------------    


    if (0) //����δ����ƽ��ĵ�
    {
        // Store all unassigned points.
        Points_3 unassigned_points;
        unassigned_points.reserve(_unassignedItems.size());
        for (const auto index : _unassignedItems)
        {
            const auto& key = *(_inputPointSet.begin() + index);
            const Point_3& point = get(_inputPointSet.point_map(), key);
            unassigned_points.push_back(point);
        }
        std::cout << "* " << unassigned_points.size() << " unassigned points are stored \n";
        std::cout << std::endl << "* region growing finished" << std::endl;
    }
}

void ResionGrowing4Points::getSortedPlanarTriangle(
    std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane)
{
    this->getPlanarSupervoxel(vOutSortedPlan, false);

    //supervoxel����ת��Ϊtriangle����
    int numPlane = vOutSortedPlan.size();
    std::vector<size_t> vTrisIx;
    for (int i = 0; i < numPlane; ++i)
    {  
        vTrisIx.clear();
        for (auto& voxelIx : vOutSortedPlan[i]) {
            size_t nums = _vvSupervoxelIndex[voxelIx].size();
            for (size_t k = 0; k < nums; ++k)
            {
                int& triIX = _vvSupervoxelIndex[voxelIx][k];
                vTrisIx.emplace_back(triIX);
            }
        }
        vOutSortedPlan[i].clear();
        vOutSortedPlan[i] = vTrisIx;
    }

    //����ÿ��ƽ���ڵ�������Ԫ��������
    std::sort(vOutSortedPlan.begin(), vOutSortedPlan.end(), [=](std::vector<size_t>& a, std::vector<size_t>& b)
        {
            return a.size() > b.size();
        }
    );

    //Optional������ƽ�漰��������
    if (bSavePlane) {
        std::ofstream outFile;
        //���ļ�
        std::cout << "* Writing plane.txt ...\n";
        std::string saveP1 = yUtils::getNameLessExtension(_triPath) + "_planes_trianglesNum_sorted.txt";
        outFile.open(saveP1);
        for (auto iter = vOutSortedPlan.begin(); iter != vOutSortedPlan.end(); ++iter) {
            outFile << iter->size() << std::endl;
        }
        outFile.close();
    }
}

void ResionGrowing4Points::getPlanarSupervoxel(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane)
{
    std::map<size_t, std::vector<size_t>> planeMap; //ƽ�������, ��ƽ���ڵ�supervoxel����
    using pairTemp = std::pair<int, std::vector<size_t>>;
    auto it = _outputPointSet.begin();
    while (it != _outputPointSet.end())
    {
        size_t& index_plane = _outputPointSet.property_map<size_t>("index_plane").first[*it];
        size_t& index_supervoxel = _outputPointSet.property_map<size_t>("index_supervoxel").first[*it];

        auto& l_it = planeMap.find(index_plane);
        if (l_it == planeMap.end())
        {
            std::vector<size_t> vSupervoxel;
            vSupervoxel.emplace_back(index_supervoxel);
            planeMap.insert(pairTemp(index_plane, vSupervoxel));
        }
        else
        {
            std::vector<size_t>& vSupervoxel = l_it->second;
            vSupervoxel.emplace_back(index_supervoxel);
        }
        ++it;
    }

    for (auto& pla : planeMap) {
        vOutSortedPlan.emplace_back(pla.second);
    }
}

void ResionGrowing4Points::getSortedPlanarSupervoxel(std::vector<std::vector<size_t>>& vOutSortedPlan, bool bSavePlane)
{
    this->getPlanarSupervoxel(vOutSortedPlan, bSavePlane);
    //����ÿ��ƽ��ĳ�������������
    std::sort(vOutSortedPlan.begin(), vOutSortedPlan.end(), [=](std::vector<size_t>& a, std::vector<size_t>& b)
        {
            return a.size() > b.size();
        }
    );

    //Optional�� ����ƽ�漰��������
    if (bSavePlane) {
        std::ofstream outFile;
        //���ļ�
        std::cout << "* Writing plane.txt ...\n";
        std::string saveP1 = yUtils::getNameLessExtension(_triPath) + "_planes_supervoxelNums_sorted.txt";
        outFile.open(saveP1);
        for (auto iter = vOutSortedPlan.begin(); iter != vOutSortedPlan.end(); ++iter) {
            outFile << iter->size() << std::endl;
        }
        outFile.close();
    }

}

void ResionGrowing4Points::writePlanerTriangles(const std::vector<std::vector<size_t>>& vvPlanerTris, std::string savePath)
{
    ///��ɫֵ������룬Ϊunassigned_points
    std::vector<std::vector<uint8_t>> vTriColor;
    std::vector<Triangle> vTris_temp;
    std::mt19937 random;
    for (auto plane : vvPlanerTris)
    {
        auto color = cl::RGB32Color(random());
        std::vector<uint8_t> rgb(3);
        rgb[0] = color.red(), rgb[1] = color.green();
        rgb[2] = color.blue();
        for (auto k : plane)
        {
            vTriColor.emplace_back(rgb);
            Triangle& tirangle = _vTriangle->at(k);
            vTris_temp.emplace_back(tirangle);
        }
    }
    vTriColor.shrink_to_fit();
    vTris_temp.shrink_to_fit();
    ///д����������
    TriangleReader triReader("");
    triReader.writeTrianglesData(vTris_temp, vTriColor, savePath);
    std::cout << "* The plane file has been saved to" << savePath << std::endl;
}

void ResionGrowing4Points::writePlanerSupervoxels(const std::vector<std::vector<size_t>>& vvPlanerSupervoxels, std::string savePath)
{
    std::vector<std::vector<size_t>> vvPlanerTris;
    //supervoxel����ת��Ϊtriangle����
    for (int i = 0; i < vvPlanerSupervoxels.size(); ++i)
    {
        std::vector<size_t> vTrisIx;
        for (auto& voxelIx : vvPlanerSupervoxels[i]) {
            for (size_t k = 0; k < _vvSupervoxelIndex[voxelIx].size(); k++)
            {
                size_t triIX = _vvSupervoxelIndex[voxelIx][k];
                vTrisIx.emplace_back(triIX);
            }
        }
        vvPlanerTris.push_back(vTrisIx);
    }

    this->writePlanerTriangles(vvPlanerTris, savePath);
}