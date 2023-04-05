/********************************************
 * Class: CalTriangleUVCoords
 * Author: Dayu
 * Site: WHU
 * Date: 2020317
********************************************/
#ifndef CALTRIANGLEUVCOORDS_H
#define CALTRIANGLEUVCOORDS_H

#include <vector>
#include <string>

#include "codelibrary/visualization/color/rgb32_color.h"

#include "triangle.h"
#include "vertex.h"

/**
 * @brief The UV struct: UV即纹理坐标，仅用于纹理坐标插值计算
 */
struct UV
{
public:
    UV(float x, float y) {
        _v[0] = x;
        _v[1] = y;
    }
    UV() {
        _v[0] = 0.f;
        _v[1] = 0.f;
    }
    inline float &operator [] (int i) {
        return _v[i];
    }
    inline float operator [] (int i) const {
        return _v[i];
    }
public:
    float _v[2];
};

/**
 * @brief 为每个三角形首先对uv坐标进行插值，根据插值的uv坐标取像素值并求平均RGB值
 */
class CalTriangleUVCoords
{
public:
    CalTriangleUVCoords(const std::vector<Triangle> &triangles,
                        const std::vector<std::vector<Vertex>> &vvVertex,
                        const std::vector<std::string> &vTexturePath);

    /**
     * @brief getMeanRGB4Triangle
     * @param vMeanRGB4triangle：返回的每个三角形的平均RGB值
     * @notice 该函数使用了omp加速，440w面片用时2.5s
     */
    void getMeanRGB4Triangle(std::vector<cl::RGB32Color> &vMeanRGB4triangle) const;

protected:
    /**
     * @brief triRasterizationBySweepLine：简易版本光栅化算法
     * @param pVecs: 三角形的三个顶点的uv坐标
     * @param UVinTri: 三角型内部插值得到的uv坐标
     * @param pWidthHeight：纹理图像的width和height
     */
    void triRasterizationBySweepLine(UV pVecs[3],
                                     std::vector<UV> &UVinTri, int pWidthHeight[2])const;
    /**
     * @brief fillBottomFlatTriangle:底平三角形
     * @param pVecs: 三角形的三个顶点的uv坐标
     * @param UVinTri: 三角型内部插值得到的uv坐标
     * @param pWidthHeight：纹理图像的width和height
     */
    void fillBottomFlatTriangle(const UV pVecs[3],
                                std::vector<UV> &UVinTri, int pWidthHeight[2]) const;
    /**
     * @brief fillTopFlatTriangle:顶平三角形
     * @param pVecs: 三角形的三个顶点的uv坐标
     * @param UVinTri: 三角型内部插值得到的uv坐标
     * @param pWidthHeight：纹理图像的width和height
     */
    void fillTopFlatTriangle(const UV pVecs[3],
                             std::vector<UV> &UVinTri, int pWidthHeight[2]) const;

private:
    const std::vector<Triangle> &_vTris;
    const std::vector<std::vector<Vertex>> &_vvVertex;
    const std::vector<std::string> &_vTexturePath;

    const int _pixelSampleSpace; //UV坐标采样像素间隔
};

#endif // CALTRIANGLEUVCOORDS_H
