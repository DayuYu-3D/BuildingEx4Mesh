#include "caltriangleuvcoords.h"

#include <fstream>
#include <omp.h>

#include <QtGui/qimage.h>
#include <QtGui/qimagereader.h>
#include <qlibrary.h>
#include <qdebug.h>

#include "codelibrary/base/log.h"

namespace  {

/**
 * @brief compareVec3:根据v升序
 */
bool compareVec3(UV a, UV b)
{
    return a[1] < b[1];
}

template<typename T>
inline T maxNum(T a, T b)
{
    return a > b ? a : b;
}

template<typename T>
inline T minNum(T a, T b)
{
    return a < b ? a : b;
}
}

CalTriangleUVCoords::CalTriangleUVCoords(
    const std::vector<Triangle> &triangles,
    const std::vector<std::vector<Vertex> > &vvVertex,
    const std::vector<std::string> &vTexturePath)
    : _vTris(triangles),
      _vvVertex(vvVertex),
      _vTexturePath(vTexturePath),
      _pixelSampleSpace(1)
{
    //do nothing
    //QString qjpeg("./imageformats/qjpeg.dll");
    //QLibrary qjpeglib(qjpeg.toUtf8().data());
    //if (qjpeglib.load())
    //{
    //    std::cout << " ** Load qjpeg.dll success" << std::endl;
    //}
    //else {
    //    std::wcout << " !! Load qjpeg.dll failed" << std::endl;
    //}
}

void CalTriangleUVCoords::getMeanRGB4Triangle(
    std::vector<cl::RGB32Color> &vMeanRGB4triangle) const
{
    int nTris = _vTris.size();
    size_t nGeom = _vvVertex.size();
    if(nGeom != _vTexturePath.size()) {
        std::wcout<< L"Geom与纹理图像数量不相等，退出"<<std::endl;
        return;
    }
    vMeanRGB4triangle.reserve(nTris + 1);
    vMeanRGB4triangle.resize(nTris);

    //预先读取纹理图片到内存中
    std::vector<QImage> vQimg(nGeom);
#pragma omp parallel for
    for(int i = 0; i < (int)nGeom; ++i)
    {
        std::string textPath = _vTexturePath.at(i);
        QImage img1;
        if (!img1.load(textPath.c_str())) {
            std::cerr << "!!! Qimamg.load() error: " << textPath <<std::endl;
            std::cerr << "The plug-in detected: ";
            qInfo() << QImageReader::supportedImageFormats();
        }
        vQimg[i] = img1;
    }
    //std::cout << "vqimg size:" << vQimg.size()<<std::endl;

#pragma omp parallel for
    for(int i = 0; i < nTris; ++i)
    {
        Triangle tri = _vTris[i];
        int ixGeom = tri._geomIndex;
        auto uv1 = _vvVertex[ixGeom][tri._vertexIndexs[0]]._texCoor;
        auto uv2 = _vvVertex[ixGeom][tri._vertexIndexs[1]]._texCoor;
        auto uv3 = _vvVertex[ixGeom][tri._vertexIndexs[2]]._texCoor;

        //获取纹理图像的长和宽
        QImage &img = vQimg[ixGeom];
        int nPixelWidth = img.width();
        int nPixelHeight = img.height();
        UV vecs[3] = {
            UV(uv1[0], uv1[1]),
            UV(uv2[0], uv2[1]),
            UV(uv3[0], uv3[1])
        };
        //扫描线算法
        std::vector<UV> uvCoords;
        int pwidthHgt[2] = {nPixelWidth, nPixelWidth};
        this->triRasterizationBySweepLine(vecs, uvCoords, pwidthHgt);
        if(uvCoords.size() == 0)
        {
            uvCoords.emplace_back(UV(uv1[0], uv1[1]));
            uvCoords.emplace_back(UV(uv2[0], uv2[1]));
            uvCoords.emplace_back(UV(uv3[0], uv3[1]));
        }

        //根据uv查询像素值并取平均
        int mR = 0, mG = 0, mB = 0;
        for(auto uvCoord : uvCoords)
        {
            int xPixel = std::ceil(uvCoord[0] * nPixelWidth);
            //翻转y，由于纹理坐标原点位于图像左下方，qimage原点位于图像左上方
            int yPixel = nPixelHeight - 1 - std::ceil(uvCoord[1] * nPixelHeight);
            if(yPixel >= nPixelHeight) {//向上取整边界溢出
                yPixel = nPixelHeight - 1;
            }
            if(xPixel >= nPixelWidth) {
                xPixel = nPixelWidth - 1;
            }
            QRgb rgb = img.pixel(xPixel, yPixel);
            mR += qRed(rgb);
            mG += qGreen(rgb);
            mB += qBlue(rgb);
        }
        mR = int(mR / uvCoords.size());
        mG = int(mG / uvCoords.size());
        mB = int(mB / uvCoords.size());
        cl::RGB32Color rgb3c(mR, mG, mB);
        vMeanRGB4triangle[i] = rgb3c;

//        //测试打印输出点
//        std::vector<std::vector<int>> vvrgb;
//        std::vector<int> ss;
//        ss.push_back(qRed(rgb));
//        ss.push_back(qGreen(rgb));
//        ss.push_back(qBlue(rgb));
//        vvrgb.push_back(ss);
//        std::ofstream outfile;
//        outfile.open("afile.xyz");
//        for(size_t x = 0; x < uvCoords.size(); ++x)
//        {
//            outfile << uvCoords[x][0] << " " << uvCoords[x][1] << " " << 0
//                    << " " << vvrgb[x][0] << " " << vvrgb[x][1] << " " <<
//                    vvrgb[x][2] << std::endl;
//        }
//        outfile.close();
    }
}

void CalTriangleUVCoords::triRasterizationBySweepLine(
    UV pVecs[], std::vector<UV> &UVinTri,
    int pWidthHeight[]) const
{
    std::sort(pVecs, pVecs + 3, compareVec3); //按y升序
    /* here we know that v1.y <= v2.y <= v3.y */
    //初始为Bottom triangle.
    if (pVecs[1][1] == pVecs[2][1])
    {
        fillTopFlatTriangle(pVecs, UVinTri, pWidthHeight);
    }
    //初始为top triangle.
    else if (pVecs[0][1] == pVecs[1][1])
    {
        fillBottomFlatTriangle(pVecs, UVinTri, pWidthHeight);
    }
    else
    {
        //将一个三角形分割成一个底平和顶平三角形
        //确定分割点位置，两点确定一直线，将y坐标带入直线方程求出x
        UV mid((pVecs[0][0] + (( pVecs[1][1] -  pVecs[0][1]) / (pVecs[2][1] - pVecs[0][1])) * (pVecs[2][0] - pVecs[0][0])),
               pVecs[1][1]);

        UVinTri.emplace_back(mid);//先把中点加进去
        float deltaX = (float)_pixelSampleSpace / (float)pWidthHeight[0];
        auto minCurX = minNum(mid[0], pVecs[1][0]);
        auto maxCurX = maxNum(mid[0], pVecs[1][0]);
        for(auto scanLineX = minCurX; scanLineX < maxCurX; scanLineX += deltaX)
        {
            UV uv(scanLineX, mid[1]);
            UVinTri.emplace_back(uv);
        }

        UV ss[3] = {pVecs[0], pVecs[1], mid};
        fillTopFlatTriangle(ss, UVinTri, pWidthHeight);
        UV zz[3] = {pVecs[1], mid, pVecs[2]};
        fillBottomFlatTriangle(zz, UVinTri, pWidthHeight);
    }
}

void CalTriangleUVCoords::fillTopFlatTriangle(
    const UV pVecs[], std::vector<UV> &UVinTri, int pWidthHeight[]) const
{
    auto curX1 = pVecs[0][0];
    auto curX2 = pVecs[0][0];

    float deltaY = (float)_pixelSampleSpace / (float)pWidthHeight[1];
    float deltaX = (float)_pixelSampleSpace / (float)pWidthHeight[0];

    auto invslope1 = deltaY * (pVecs[1][0] - pVecs[0][0]) / (pVecs[1][1] - pVecs[0][1]);
    auto invslope2 = deltaY * (pVecs[2][0] - pVecs[0][0]) / (pVecs[2][1] - pVecs[0][1]);

    for (auto scanlineY = pVecs[0][1]; scanlineY <= pVecs[1][1]; scanlineY += deltaY)
    {
        if(curX1 == curX2)
        {
            //do nothing
        }
        else
        {
            auto minCurX = minNum(curX1, curX2);
            auto maxCurX = maxNum(curX1, curX2);
            for(auto scanLineX = minCurX; scanLineX < maxCurX; scanLineX += deltaX)
            {
                UV uv(scanLineX, scanlineY);
                UVinTri.emplace_back(uv);
            }
        }
        curX1 += invslope1;
        curX2 += invslope2;
    }
}

void CalTriangleUVCoords::fillBottomFlatTriangle(
    const UV pVecs[], std::vector<UV> &UVinTri, int pWidthHeight[]) const
{
    auto curX1 = pVecs[2][0];
    auto curX2 = pVecs[2][0];

    float deltaY = (float)_pixelSampleSpace / (float)pWidthHeight[1];
    float deltaX = (float)_pixelSampleSpace / (float)pWidthHeight[0];

    auto invslope1 = deltaY * (pVecs[0][0] - pVecs[2][0]) / (pVecs[0][1] - pVecs[2][1]);
    auto invslope2 = deltaY * (pVecs[1][0] - pVecs[2][0]) / (pVecs[1][1] - pVecs[2][1]);

    for (auto scanlineY = pVecs[2][1]; scanlineY > pVecs[0][1]; scanlineY -= deltaY)
    {
        if(curX1 == curX2)
        {
            //do nothing
        }
        else
        {
            auto minCurX = minNum(curX1, curX2);
            auto maxCurX = maxNum(curX1, curX2);
            for(auto scanLineX = minCurX; scanLineX < maxCurX; scanLineX += deltaX)
            {
                UV uv(scanLineX, scanlineY);
                UVinTri.emplace_back(uv);
            }
        }
        curX1 -= invslope1;
        curX2 -= invslope2;
    }
}

