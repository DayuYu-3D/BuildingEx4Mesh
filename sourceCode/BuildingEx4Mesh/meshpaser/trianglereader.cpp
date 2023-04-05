#include "trianglereader.h"

#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>

#include <qfile.h>
#include <qtextstream.h>

#include "yutils/FileNameUtils.h"
#include "yutils/FileUtil.h"

namespace {
    inline char* strtok_rs(char* x, char const* xx, char** xxx) {
#ifdef WIN32
        return strtok_s(x, xx, xxx);// WIN32
#elif
        return strtok_r(x, xx, xxx); //linux
#endif 
    }
}


TriangleReader::TriangleReader(std::string triFP)
    : _triFP(triFP)
{
}

bool TriangleReader::readTriangleData(std::vector<Triangle> &vTri,
                                      std::vector<std::vector<Vertex> > &vvVertex,
                                      std::vector<std::string> &vTexture) const
{
    if(!this->fileValidate())
    {
        std::cout << "Check file integrity failed" << std::endl;
        return false;
    }

    //读取纹理图像的路径
    std::string dir = yUtils::getFilePath(_triFP);
    QFile textureF((dir + "/textures.txt").c_str());
    if (textureF.open(QIODevice::ReadOnly | QIODevice::Text)) {
        while (!textureF.atEnd()) {
            QByteArray line = textureF.readLine();
            QString strLine(line);
            strLine.remove("\n");
            if(strLine.size() != 0) {
                vTexture.push_back(strLine.toStdString());
            }
        }
        textureF.close();
    } else {
        std::cout << "Read texture file error" << std::endl;
    }

    //计算共有多少个点和triangle,便于开辟内存
    std::cout << "calculate the numbers of triangles and vertex..." << std::endl;
    time_t tStart = std::time(0);
    this->allocateMemory(vTri, vvVertex);
    std::cout << "Use time in allocate memeory:" << std::time(0) - tStart << "s" << std::endl;

    std::cout << "Read vertexs and triangls..." << std::endl;
    tStart = time(0);
    //读取三角面片
    QFile trianglesF(_triFP.c_str());
    Triangle tri;
    if (trianglesF.open(QIODevice::ReadOnly | QIODevice::Text )) {
        uchar *fpr = trianglesF.map(0, trianglesF.size()); //加快读写速度,速度提升一倍,目前windows最好方案
        char *s = strdup((char *)fpr);
        char *substr;
        char *token = NULL;
        char *next = NULL;
        while ((substr = strtok_s(s, "\n", &next)))
        {
            s = next;
            for (int i = 0; i < 7; i++)
            {
                char *lineSubStr = strtok_rs(substr, " ", &token);
                substr = token;
                switch (i)
                {
                case 0:
                    tri._geomIndex = std::atoi(lineSubStr);
                    break;
                case 1:
                    tri._vertexIndexs[0] = std::atoi(lineSubStr);
                    break;
                case 2:
                    tri._vertexIndexs[1] = std::atoi(lineSubStr);
                    break;
                case 3:
                    tri._vertexIndexs[2] = std::atoi(lineSubStr);
                    break;
                case 4:
                    tri._normal[0] = std::atof(lineSubStr);
                    break;
                case 5:
                    tri._normal[1] = std::atof(lineSubStr);
                    break;
                case 6:
                    tri._normal[2] = std::atof(lineSubStr);
                    break;
                }
            }
            vTri.emplace_back(tri);
        }
        trianglesF.close();
    } else {
        std::cout << "Read triangle file error" << std::endl;
        return false;
    }

    //read顶点数据
    this->readVertexData(vvVertex);

    std::cout << "Use time in parse mesh files:" << time(0) - tStart << "s" << std::endl;
    std::cout << "Read number of trianlges:" << vTri.size() << std::endl;
    return true;
}

bool TriangleReader::readSupervoxel(
    std::vector<Triangle> &vTri,
    std::vector<std::vector<Vertex> > &vvVertex,
    std::vector<std::string> &vTexture,
    std::vector<int> &vSupervoxelIndex) const
{
    if(!this->fileValidate())
    {
        std::cerr << "Check file integrity failed" << std::endl;
        return false;
    }

    //读取纹理图像的路径
    std::string dir = yUtils::getFilePath(_triFP);
    QFile textureF((dir + "/textures.txt").c_str());
    if (textureF.open(QIODevice::ReadOnly | QIODevice::Text)) {
        while (!textureF.atEnd()) {
            QByteArray line = textureF.readLine();
            QString strLine(line);
            strLine.remove("\n");
            if(strLine.size() != 0) {
                vTexture.push_back(strLine.toStdString());
            }
        }
        textureF.close();
    } else {
        std::cerr << "Read texture file error" << std::endl;
        return false;
    }

    //计算共有多少个点和triangle,便于开辟内存
    std::cout << "calculate the numbers of triangles and vertex..." << std::endl;
    time_t tStart = std::time(0);
    this->allocateMemory(vTri, vvVertex);
    std::cout << "Use time in allocate memeory:" << std::time(0) - tStart << "s" << std::endl;

    std::cout << "Read vertexs and triangls..." << std::endl;
    tStart = time(0);

    //读取三角面片
    QFile trianglesF(_triFP.c_str());
    Triangle tri; //三角形的临时变量，避免重复申请内存
    if (trianglesF.open(QIODevice::ReadOnly | QIODevice::Text )) {
        uchar *fpr = trianglesF.map(0, trianglesF.size()); //加快读写速度,速度提升一倍,目前windows最好方案
        char *s = strdup((char *)fpr);
        char *substr;
        char *token = NULL;
        char *next = NULL;
        while ((substr = strtok_rs(s, "\n", &next)))
        {            
            s = next;
            for (int i = 0; i < 8; i++)
            {
                char* lineSubStr = strtok_rs(substr, " ", &token);
                substr = token;
                switch (i)
                {
                case 0:
                    tri._geomIndex = std::atoi(lineSubStr);
                    break;
                case 1:
                    tri._vertexIndexs[0] = std::atoi(lineSubStr);
                    break;
                case 2:
                    tri._vertexIndexs[1] = std::atoi(lineSubStr);
                    break;
                case 3:
                    tri._vertexIndexs[2] = std::atoi(lineSubStr);
                    break;
                case 4:
                    tri._normal[0] = std::atof(lineSubStr);
                    break;
                case 5:
                    tri._normal[1] = std::atof(lineSubStr);
                    break;
                case 6:
                    tri._normal[2] = std::atof(lineSubStr);
                    break;
                case 7:
                    vSupervoxelIndex.emplace_back(std::atoi(lineSubStr));
                    break;
                }
            }
            vTri.emplace_back(tri);
        }
        trianglesF.close();
    } else {
        std::cerr << "Read triangle file error" << std::endl;
        return false;
    }

    //read顶点数据
    this->readVertexData(vvVertex);

    vSupervoxelIndex.shrink_to_fit();
    std::cout << "Use time in parse mesh files:" << time(0) - tStart << "s" << std::endl;
    std::cout << "Read number of trianlges:" << vTri.size() << std::endl;
    return true;
}

void TriangleReader::writeTrianglesData(const std::vector<Triangle> &vTri,
                                        const std::vector<unsigned int> &vTriIndex,
                                        std::string triP) const
{
    QFile triF(triP.c_str());
    triF.open(QIODevice::WriteOnly | QIODevice::Text); //不存在会自动创建
    QTextStream smTri(&triF);

    for(auto triIx : vTriIndex)
    {
        auto triangle = vTri[triIx];
        int index_geom = triangle._geomIndex;
        //int index = triangle._index; //在三角网中的索引
        int index_vertex1 = triangle._vertexIndexs[0];
        int index_vertex2 = triangle._vertexIndexs[1];
        int index_vertex3 = triangle._vertexIndexs[2];
        double normalx = triangle._normal[0];
        double normaly = triangle._normal[1];
        double normalz = triangle._normal[2];
        smTri << index_geom << " " << index_vertex1 << " "
              << index_vertex2 << " " << index_vertex3 << " "
              << normalx << " " << normaly << " " << normalz << "\n";
    }
    triF.close();
}

void TriangleReader::writeTrianglesData(const std::vector<Triangle>& vTri,
    const std::vector<size_t>& vTriIndex,
    std::string triP) const
{
    std::vector<unsigned int> vTemp;
    for (auto& temp : vTriIndex)
        vTemp.emplace_back(temp);
    this->writeTrianglesData(vTri,vTemp,triP);
}

void TriangleReader::writeTrianglesData(const std::vector<Triangle> &vTri,
                                        const std::vector<std::vector<uint8_t>> &vColor,
                                        std::string triP) const
{
    QFile triF(triP.c_str());
    triF.open(QIODevice::WriteOnly | QIODevice::Text); //不存在会自动创建
    QTextStream smTri(&triF);

    size_t nTri = vTri.size();
    for(size_t i = 0; i < nTri; ++i)
    {
        auto tri = vTri[i];
        int index_geom = tri._geomIndex;
        //int index = triangle._index; //在三角网中的索引
        int index_vertex1 = tri._vertexIndexs[0];
        int index_vertex2 = tri._vertexIndexs[1];
        int index_vertex3 = tri._vertexIndexs[2];
        double normalx = tri._normal[0];
        double normaly = tri._normal[1];
        double normalz = tri._normal[2];
        auto color = vColor[i];
        smTri << index_geom << " " << index_vertex1 << " "
              << index_vertex2 << " " << index_vertex3 << " "
              << normalx << " " << normaly << " " << normalz << " "
              << color[0] << " " << color[1] << " " << color[2] << "\n";
    }
    triF.close();
}

void TriangleReader::writeSuperVoxels(
    const std::vector<Triangle> &vTri,
    const std::vector<int> &vSupervoxelIndex, std::string triP) const
{
    if(vTri.size() != vSupervoxelIndex.size())
    {
        std::wcerr << L"错误，超体素索引数量与三角面片数量不符合" << std::endl;
        return;
    }
    QFile triF(triP.c_str());
    triF.open(QIODevice::WriteOnly | QIODevice::Text); //不存在会自动创建
    QTextStream smTri(&triF);

    size_t nTri = vTri.size();
    for(size_t i = 0; i < nTri; ++i)
    {
        auto tri = vTri[i];
        int index_geom = tri._geomIndex;
        //int index = triangle._index; //在三角网中的索引
        int index_vertex1 = tri._vertexIndexs[0];
        int index_vertex2 = tri._vertexIndexs[1];
        int index_vertex3 = tri._vertexIndexs[2];
        double normalx = tri._normal[0];
        double normaly = tri._normal[1];
        double normalz = tri._normal[2];
        int idSuperVocel = vSupervoxelIndex[i];
        smTri << index_geom << " " << index_vertex1 << " "
              << index_vertex2 << " " << index_vertex3 << " "
              << normalx << " " << normaly << " " << normalz << " "
              << idSuperVocel << "\n";
    }
    triF.close();
}

void TriangleReader::writePointClouds(const std::vector<Vec3d>& vPoints, const std::string triP) const
{
    if (!vPoints.size())
    {
        std::wcerr << L"点云数量为0，do nothing" << std::endl;
        return;
    }
    QFile triF(triP.c_str());
    triF.open(QIODevice::WriteOnly | QIODevice::Text); //不存在会自动创建
    QTextStream smTri(&triF);

    size_t nP = vPoints.size();
    for (size_t i = 0; i < nP; ++i)
    {
        auto point = vPoints[i];
        auto px = point.x();
        auto py = point.y();
        auto pz = point.z();
        smTri << px << " " << py << " "
            << pz << "\n";
    }
    triF.close();
}

void TriangleReader::getAbsoutePath(std::string triPath, std::vector<std::string>& vTexture_inAndOut) const
{
    for (auto& str : vTexture_inAndOut)
    {
       str = yUtils::getFilePath(triPath) +"/textures/"+ yUtils::getSimpleFileName(str);
    }
}

bool TriangleReader::fileValidate() const
{
    std::cout << "Check file integrity " << _triFP << std::endl;
    bool isVal = false;

    std::ifstream fin(_triFP);
    if(!fin) {
        std::cout << "triangles file not exist" << std::endl;
        return isVal;
    }
    std::string dir = yUtils::getFilePath(_triFP);
    std::ifstream fin2(dir + "/textures.txt");
    if(!fin2)
    {
        std::cout << "textures file not exist" << std::endl;
        return isVal;
    }
    auto dirPs = dir + "/points";
    if(!std::filesystem::exists(dirPs))
    {
        std::cout << "points dir not exist" << std::endl;
        return isVal;
    }

    return !isVal;
}

void TriangleReader::allocateMemory(std::vector<Triangle> &vTri,
                                    std::vector<std::vector<Vertex> > &vvVertex) const
{
    std::string fp_dir = yUtils::getFilePath(_triFP);

    size_t numTriangle = 0, numPoint = 0, numPointFile = 0;
    numTriangle = yUtils::getFileLinesNum(_triFP);
    numPointFile = yUtils::getNumofFilesInDirectory(fp_dir + "/points");
    auto geomfileL = yUtils::getFileListInDirectory(fp_dir + "/points");
    yUtils::sortFileList(geomfileL);
    //开辟内存
    ///所有三角面片
    vTri.reserve(numTriangle + 1);
    ///所有顶点,按geom1,2,3...组织
    vvVertex.reserve(numPointFile + 1);
    //开辟每个geom中顶点的内存
    for(auto geomF : geomfileL) {
        if(std::filesystem::is_directory(geomF)) {
            continue;
        }
        size_t numPointofGeom = yUtils::getFileLinesNum(geomF);
        std::vector<Vertex> pointsOfGeom;
        pointsOfGeom.reserve(numPointofGeom + 1);
        vvVertex.push_back(pointsOfGeom);
        numPoint += numPointofGeom;
    }
    std::cout << "Allocate memory of vertexs:" << numPoint << std::endl;
    std::cout << "Allocate memory of triangles:" << numTriangle << std::endl;
}

void TriangleReader::readVertexData(std::vector<std::vector<Vertex> > &vvVertex) const
{
    int numP = 0;
    //读取顶点
    std::string fp_dir = yUtils::getFilePath(_triFP);
    auto geomfileL = yUtils::getFileListInDirectory(fp_dir + "/points");
    yUtils::sortFileList(geomfileL);

    Vertex vex; //顶点的临时变量，避免重复申请内存
    for(uint i = 0; i < geomfileL.size(); ++i)
    {
        if(std::filesystem::is_directory(geomfileL[i])) {
            continue;
        }

        QFile pointF(geomfileL[i].c_str());
        if (pointF.open(QIODevice::ReadOnly | QIODevice::Text)) {
            uchar *fpr = pointF.map(0, pointF.size()); //加快读写速度,速度提升一倍
            char *s = strdup((char *)fpr);
            char *substr;
            char *token = NULL;
            char *next = NULL;
            while ((substr = strtok_rs(s, "\n", &next)))
            {
                s = next;
                for (int i = 0; i < 6; i++)
                {
                    char *lineSubStr = strtok_rs(substr, " ", &token);
                    substr = token;
                    switch (i)
                    {
                    case 0:
                        vex._index = std::atoi(lineSubStr);
                        break;
                    case 1:
                        vex._coor[0] = std::atof(lineSubStr);
                        break;
                    case 2:
                        vex._coor[1] = std::atof(lineSubStr);
                        break;
                    case 3:
                        vex._coor[2] = std::atof(lineSubStr);//注意:这是正的,即没有将z轴翻转
                        break;
                    case 4:
                        vex._texCoor[0] = std::atof(lineSubStr);
                        break;
                    case 5:
                        vex._texCoor[1] = std::atof(lineSubStr);
                        break;
                    }
                }
                vvVertex[i].emplace_back(vex); //添加顶点
                numP += 1;//纯粹计数
            }
            pointF.close();
        }
    }
    std::cout << "Read number of vertexs:" << numP << std::endl;
}
