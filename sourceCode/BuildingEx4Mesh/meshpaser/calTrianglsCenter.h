/********************************************
 * Class: CalTriangleUVCoords
 * Author: Dayu
 * Site: WHU
 * Date: 2020317
********************************************/
#ifndef CALTRIANGLSCENTER_H
#define CALTRIANGLSCENTER_H

#include <vector>

#include "codelibrary/geometry/kernel/point_3d.h"

#include "triangle.h"
#include "vertex.h"

class CalTrianglesCenter {
public:
    CalTrianglesCenter(const std::vector<Triangle> &triangles,
                       const std::vector<std::vector<Vertex>> &vvVertex)
        : _tris(triangles),
          _vvVertex(vvVertex)
    { }

    /**
     *计算形心，出入T应该是一个vector，支持push_back操作
     */
    template <typename T>
    void getMassCenter(T &centers) const
    {
        int nTris = _tris.size();
        for(int i = 0; i < nTris; ++i)
        {
            auto tri = _tris.at(i);
            auto v1 = _vvVertex.at(tri._geomIndex).at(tri._vertexIndexs[0]);
            auto v2 = _vvVertex.at(tri._geomIndex).at(tri._vertexIndexs[1]);
            auto v3 = _vvVertex.at(tri._geomIndex).at(tri._vertexIndexs[2]);

            Vec3d mv = (v1._coor + v2._coor + v3._coor) / 3;
            cl::RPoint3D rpt(mv.x(), mv.y(), mv.z());
            centers.push_back(rpt);
        }
    }

    /**
     *计算形心，出入T应该是一个vector，支持push_back操作
     */
    template <typename T>
    void getMassCenter(T& centers, const std::vector<size_t>& vtriIx) const
    {
        int nTris = vtriIx.size();
        for (int i = 0; i < nTris; ++i)
        {
            size_t triIx = vtriIx[i];
            auto tri = _tris.at(triIx);
            auto v1 = _vvVertex.at(tri._geomIndex).at(tri._vertexIndexs[0]);
            auto v2 = _vvVertex.at(tri._geomIndex).at(tri._vertexIndexs[1]);
            auto v3 = _vvVertex.at(tri._geomIndex).at(tri._vertexIndexs[2]);

            Vec3d mv = (v1._coor + v2._coor + v3._coor) / 3;
            cl::RPoint3D rpt(mv.x(), mv.y(), mv.z());
            centers.push_back(rpt);
        }
    }

private:
    const std::vector<Triangle> &_tris;
    const std::vector<std::vector<Vertex>> &_vvVertex;
};

#endif // CALTRIANGLSCENTER_H
