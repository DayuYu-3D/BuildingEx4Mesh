// Author(s)     : Dayu Yu
// Email         : Dayuyu@whu.edu.cn
// Adress        : Wuhan University

#ifndef TOPOLOGYSEARCH4SUPERVOXEL
#define TOPOLOGYSEARCH4SUPERVOXEL

#include <CGAL/license/Shape_detection.h>

// STL includes.
#include <typeinfo>

#include <map>
#include <unordered_map>
#include <set>
#include <chrono>

#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/vertex.h"

namespace CGAL {
    namespace Shape_detection {
        namespace Point_set {

            /*!
              这个类返回一个查询的超体素中心点的与之拓扑相连的超体素中心点

              \cgalModels `NeighborQuery`
            */
            class topology_neighbor_query_4supervoxels 
            {
                public:
                    topology_neighbor_query_4supervoxels(
                        const std::vector<Triangle>* vTriangle,
                        const std::vector<std::vector<Vertex>>* vvVertex,
                        const std::vector<int>* vSupervoxelIndex,  
                        const std::vector<std::vector<int>>* vvSupervoxelIndex )
                        :
                        _vTriangle(vTriangle),
                        _vvVertex(vvVertex),
                        _vSupervoxelIndex(vSupervoxelIndex),
                        _vvSupervoxelIndex(vvSupervoxelIndex)
                    {
                        //do something
                        this->createConnectByVertex(_vTriangle, _vvVertex);
                        this->createConnectBySupervoxels();
                    }


                    void createConnectByVertex(const std::vector<Triangle>* vTriangle,
                        const std::vector<std::vector<Vertex>>* vvVertex)
                    {
                        auto startT = std::chrono::system_clock::now(); //获取开始时间

                        _vmPTop.clear();
                        _vmPTop.resize(vvVertex->size());

                        size_t nTris = vTriangle->size();
                        for (size_t i = 0; i < nTris; ++i)
                        {
                            const auto& triangle = vTriangle->at(i);
                            const int& geomIX = triangle._geomIndex;
                            const int& v1Ix = vvVertex->at(geomIX).at(triangle._vertexIndexs[0])._index;
                            const int& v2Ix = vvVertex->at(geomIX).at(triangle._vertexIndexs[1])._index;
                            const int& v3Ix = vvVertex->at(geomIX).at(triangle._vertexIndexs[2])._index;


                            std::map<size_t, std::set<size_t>>& mPTop = _vmPTop[geomIX];

                            auto it1 = mPTop.find(v1Ix);
                            if (it1 == mPTop.end())
                            { 
                                std::set<size_t> sTriIX;
                                sTriIX.insert(i);
                                mPTop.insert(std::make_pair(v1Ix, sTriIX));
                            }
                            else {
                                it1->second.insert(i);
                            }

                            auto it2 = mPTop.find(v2Ix);
                            if (it2 == mPTop.end())
                            { 
                                std::set<size_t> sTriIX;
                                sTriIX.insert(i);
                                mPTop.insert(std::make_pair(v2Ix, sTriIX));
                            }
                            else {
                                it2->second.insert(i);
                            }

                            auto it3 = mPTop.find(v3Ix);
                            if (it3 == mPTop.end())
                            { 
                                std::set<size_t> sTriIX;
                                sTriIX.insert(i);
                                mPTop.insert(std::make_pair(v3Ix, sTriIX));
                            }
                            else {
                                it3->second.insert(i);
                            }
                        }

                        auto entT = std::chrono::system_clock::now();//获取结束时间
                        auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
                        std::cout << " **** Time elipsed in createConnectByVertex func: " << (double)ut / 1000 << "s" << std::endl;
                    }

                    void createConnectBySupervoxels()
                    {

                        std::vector<std::set<size_t>> vsTriIx;                        
                        for (size_t i = 0; i < _vvSupervoxelIndex->size(); ++i)
                        {

                            std::set<size_t> sNeiborTriangles;
                            for (const int& triIx : _vvSupervoxelIndex->at(i))
                            {

                                const Triangle& tri = _vTriangle->at(triIx);
                                const size_t& geomIx = tri._geomIndex;
                                const size_t& v1Ix = _vvVertex->at(geomIx).at(tri._vertexIndexs[0])._index;
                                const size_t& v2Ix = _vvVertex->at(geomIx).at(tri._vertexIndexs[1])._index;
                                const size_t& v3Ix = _vvVertex->at(geomIx).at(tri._vertexIndexs[2])._index;
                                const std::map<size_t, std::set<size_t>>& mPTop = _vmPTop[geomIx];

                                auto it1 = mPTop.find(v1Ix);
                                if (it1 == mPTop.end())
                                {
                                    std::cout << "当前key不存在!!!" << std::endl;   
                                    std::cout << v1Ix << std::endl;
                                }
                                else {
                                    for (size_t triIx_n : it1->second)
                                        sNeiborTriangles.insert(triIx_n);
                                }
                                auto it2 = mPTop.find(v2Ix);
                                if (it2 == mPTop.end())
                                {
                                    std::cout << "当前key不存在!!!" << std::endl;
                                }
                                else {
                                    for (size_t triIx_n : it2->second)
                                        sNeiborTriangles.insert(triIx_n);
                                }
                                auto it3 = mPTop.find(v3Ix);
                                if (it3 == mPTop.end())
                                {
                                    std::cout << "当前key不存在!!!" << std::endl;
                                }
                                else {
                                    for (size_t triIx_n : it3->second)
                                        sNeiborTriangles.insert(triIx_n);
                                }
                            }
                            vsTriIx.emplace_back(sNeiborTriangles);
                        }


                        for (size_t i = 0; i < vsTriIx.size(); ++i)
                        {
                            std::set<size_t> sSuperveoxelIx;
                            for (size_t triIx : vsTriIx[i]) //vstriix[i]为set<size_t>
                            {
                                int superIx = _vSupervoxelIndex->at(triIx);
                                sSuperveoxelIx.insert(superIx);
                            }
                            _vsSupervoxelIx.emplace_back(sSuperveoxelIx);
                        }

                    }

                    void operator()(
                        const std::size_t query_index,
                        std::vector<std::size_t>& neighbors) const {

                        CGAL_precondition(query_index < m_input_range.size());

                        neighbors.clear();
                        std::set<size_t> sSupervoxelIx = _vsSupervoxelIx[query_index];
                        for (auto it = sSupervoxelIx.begin(); it != sSupervoxelIx.end(); ++it)
                            neighbors.push_back(*it);
                    }

                private:

                    const std::vector<Triangle>* _vTriangle;
                    const std::vector<std::vector<Vertex>>* _vvVertex;
                    const std::vector<int>* _vSupervoxelIndex;           
                    const std::vector<std::vector<int>>* _vvSupervoxelIndex;   


                    std::vector< std::map<size_t, std::set<size_t>>> _vmPTop;


                    std::vector<std::set<size_t>> _vsSupervoxelIx;
            };

        } // namespace Point_set
    } // namespace Shape_detection
} // namespace CGAL

#endif // TOPOLOGYSEARCH4SUPERVOXEL
