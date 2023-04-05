/********************************************
 * Class: TriangleRead
 * Author: Dayu
 * Site: WHU
 * Date: 20210316
********************************************/
#ifndef TRIANGLEREADER_H
#define TRIANGLEREADER_H

#include <string>
#include <vector>

#include "triangle.h"
#include "vertex.h"

class TriangleReader
{
public:
    TriangleReader(std::string triFP);

    /**
     * @brief 快速解析顶点和三角面片文件夹
     * @param vTri: 需要分配的三角面片矢量
     * @param vvVertex: 需要分配的顶点二维矢量
     * @param vTexture: 每个geom对应的纹理图像路径
     * @return 解析成功,返回true
     */
    bool readTriangleData(std::vector<Triangle> &vTri,
                          std::vector<std::vector<Vertex>> &vvVertex,
                          std::vector<std::string> &vTexture) const;

    /**
     * @brief 快速解析超体素数据
     * @param vTri: 需要分配的三角面片矢量
     * @param vvVertex: 需要分配的顶点二维矢量
     * @param vTexture: 每个geom对应的纹理图像路径
     * @param vSupervoxelIndex: 每个面片对应的超体素索引
     * @return
     */
    bool readSupervoxel(std::vector<Triangle> &vTri,
                        std::vector<std::vector<Vertex>> &vvVertex,
                        std::vector<std::string> &vTexture,
                        std::vector<int> &vSupervoxelIndex) const;

    /**
     * @brief readTrianglesData: 写triangle文件
     * 只保存triangle信息，不保存verte文本，vertex还是用之前的文本
     * @param vTri: vector of triangle,
     */
    void writeTrianglesData(const std::vector<Triangle> &vTri,
                            const std::vector<unsigned int> &vTriIndex,
                            std::string triP) const;

    /**
     * @brief readTrianglesData: 写triangle文件
     * 只保存triangle信息，不保存verte文本，vertex还是用之前的文本
     * @param vTri: vector of triangle,
     */
    void writeTrianglesData(const std::vector<Triangle>& vTri,
        const std::vector<size_t>& vTriIndex,
        std::string triP) const;

    /**
     * @brief 写triangle文件,有RGB颜色值，255，255,255
     * 只保存triangle信息，不保存verte文本，vertex还是用之前的文本
     * @param vTri: vector of triangle,
     */
    void writeTrianglesData(const std::vector<Triangle> &vTri,
                            const std::vector<std::vector<uint8_t>> &vColor,
                            std::string triP) const;

     /**
     * @brief writeSuperVoxels 写超体素到文件
     * @param vTri：面片
     * @param vSupervoxelIndex：索引，数量等于面片
     * @param triP:路径
     */
    void writeSuperVoxels(const std::vector<Triangle> &vTri,
                          const std::vector<int> &vSupervoxelIndex,
                          std::string triP) const;

    /**
     * @brief writeSeedsOfSupervoxel 写点云到文件
     * @param vPoints 点云的数组
     * @param triP 需要保存的路径
     */
    void writePointClouds(const std::vector<Vec3d>& vPoints,
        const std::string triP) const;

    void getAbsoutePath(std::string triPath, std::vector<std::string>& vTexture_inAndOut) const;
private:
    /**
     * @brief 检查文件完整性
     * @return 完整返回true,否则else
     */
    bool fileValidate() const;

    /**
     * @brief 预先为顶点和triangle矢量分配内存,防止内存指数型增长
     * @param vTri: 需要分配的三角面片矢量
     * @param vvVertex: 需要分配的顶点二维矢量
     */
    void allocateMemory(std::vector<Triangle> &vTri,
                        std::vector<std::vector<Vertex>> &vvVertex) const;

    /**
     * @brief 阅读面片数据的顶点
     * @param vvVertex: 保存的顶点数组
     */
    void readVertexData(std::vector<std::vector<Vertex>> &vvVertex) const;
private:
    std::string _triFP;
};

#endif // TRIANGLEREADER_H
