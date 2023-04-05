#include "topologyConnect.h"

#include <chrono>
#include <fstream>
#include <stack>

TopologyConnect::TopologyConnect()
{
}

TopologyConnect::~TopologyConnect()
{
}

void TopologyConnect::createConnectByVertex(
	const std::vector<Triangle>* vTriangle, const std::vector<std::vector<Vertex>>* vvVertex)
{
	auto startT = std::chrono::system_clock::now(); //获取开始时间

	_vmPTop.resize(vvVertex->size());

	size_t nTris = vTriangle->size();
	for (size_t i = 0; i < nTris; ++i)
	{
		const auto& triangle = vTriangle->at(i);
		const int& geomIX = triangle._geomIndex;
		const int& v1Ix = vvVertex->at(geomIX).at(triangle._vertexIndexs[0])._index;
		const int& v2Ix = vvVertex->at(geomIX).at(triangle._vertexIndexs[1])._index;
		const int& v3Ix = vvVertex->at(geomIX).at(triangle._vertexIndexs[2])._index;
		//const int& v1Ix = triangle._vertexIndexs[0];
		//const int& v2Ix = triangle._vertexIndexs[1];
		//const int& v3Ix = triangle._vertexIndexs[2];

		std::map<size_t, std::set<size_t>>& mPTop = _vmPTop[geomIX];


		auto it1 = mPTop.find(v1Ix);
		if (it1 == mPTop.end())
		{ //当前key不存在
			std::set<size_t> sTriIX;
			sTriIX.insert(i);
			mPTop.insert(std::make_pair(v1Ix, sTriIX));
		}
		else {
			it1->second.insert(i);
		}

		auto it2 = mPTop.find(v2Ix);
		if (it2 == mPTop.end())
		{ //当前key不存在
			std::set<size_t> sTriIX;
			sTriIX.insert(i);
			mPTop.insert(std::make_pair(v2Ix, sTriIX));
		}
		else {
			it2->second.insert(i);
		}

		auto it3 = mPTop.find(v3Ix);
		if (it3 == mPTop.end())
		{ //当前key不存在
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

	//TODO：delete 保存到文件看看正确
	if (0) {
		std::ofstream out("createConnectByVertex_text.txt");
		for (const auto& m : _vmPTop[1]) {
			out << m.first << ": ";
			for (const auto& s : m.second) {
				out << s<<" ";
			}
			out << std::endl;
		}
	}
}

void TopologyConnect::bruteSearchByDFS(
	const std::vector<size_t>* vTriangleIndex, 
	std::vector<size_t>* vTriangleIndex_out,
	const std::vector<Triangle>* vTriangle, 
	const std::vector<std::vector<Vertex>>* vvVertex) const
{
	auto startT = std::chrono::system_clock::now(); //获取开始时间


	std::vector<std::vector<size_t>> vvTriIx(vvVertex->size());
	size_t nTris = vTriangleIndex->size();
	for (size_t i = 0; i < nTris; ++i)
	{
		const size_t& triIx = vTriangleIndex->at(i);
		const int& geomIx = vTriangle->at(triIx)._geomIndex;
		vvTriIx[geomIx].emplace_back(triIx);
	}

	for (auto& vtriIx : vvTriIx) 
	{
		vtriIx.shrink_to_fit();
	}

	std::set<size_t> vistedTriIx;

	//step1: 
	for (size_t i = 0; i < vvTriIx.size(); ++i) 
	{
		std::vector<size_t>& vTriIx = vvTriIx[i]; //第i个geom

		if (!vTriIx.size()) 
		{
			continue;
		}


		std::stack<size_t> unvisitedTriangleIx;

		std::set<size_t> visitedVertexIx;

		for (auto& triIx : vTriIx) 
		{
			unvisitedTriangleIx.push(triIx);
		}

		size_t* currentTriIx;
		while (!unvisitedTriangleIx.empty())
		{
			currentTriIx = &unvisitedTriangleIx.top();

			unvisitedTriangleIx.pop();

			if (vistedTriIx.find(*currentTriIx) == vistedTriIx.end())
			{
				vistedTriIx.insert(*currentTriIx);

				const Triangle& triangle = vTriangle->at(*currentTriIx);
				size_t vertex3[3];
				vertex3[0] = vvVertex->at(i).at(triangle._vertexIndexs[0])._index;
				vertex3[1] = vvVertex->at(i).at(triangle._vertexIndexs[1])._index;
				vertex3[2] = vvVertex->at(i).at(triangle._vertexIndexs[2])._index;

				for (int k = 0; k < 3; ++k)
				{

					if (visitedVertexIx.find(vertex3[k]) == visitedVertexIx.end())
					{//没有访问过
						auto& it = _vmPTop[i].find(vertex3[k]); 
						if (it == _vmPTop[i].end())
						{
							std::cerr << "error in TopologyConnect: "<< vertex3[k] << std::endl;
							std::cerr << i << " "<<k<<" " << triangle._vertexIndexs[k] << std::endl;
							return;
						}
						else {
							auto& triset = it->second;
							for (const size_t& triIX : triset)
							{
								unvisitedTriangleIx.push(triIX);
							}
						}
						visitedVertexIx.insert(vertex3[k]);
					}
				}
			}
		}
	}

	//set 转为 vector
	vTriangleIndex_out->assign(vistedTriIx.begin(), vistedTriIx.end());

	auto entT = std::chrono::system_clock::now();//获取结束时间
	auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
	std::cout << " **** Time elipsed in bruteSearchByDFS func: " << (double)ut / 1000 << "s" << std::endl;}

void TopologyConnect::bruteSearchByDFS(
	const std::vector<std::vector<size_t>>* vvTriIx, 
	std::vector<size_t>* vTriangleIndex_out,
	const std::vector<Triangle>* vTriangle, 
	const std::vector<std::vector<Vertex>>* vvVertex) const
{
	std::vector<size_t> vTriangleIndex; 
	for (const auto& plane : *vvTriIx) {
		for (const size_t& triIx : plane) {
			vTriangleIndex.emplace_back(triIx);
		}
	}

	this->bruteSearchByDFS(&vTriangleIndex, vTriangleIndex_out, vTriangle,vvVertex);
}

//*****************************deprecated*****************
void TopologyConnect::bruteSearchByDFS_deprecated(
	const std::vector<size_t>* vTriangleIndex,
	std::vector<size_t>* vTriangleIndex_out,
	const std::vector<Triangle>* vTriangle,
	const std::vector<std::vector<Vertex>>* vvVertex) const
{
	auto startT = std::chrono::system_clock::now(); //获取开始时间


	std::vector<std::vector<size_t>> vvTriIx(vvVertex->size());
	size_t nTris = vTriangleIndex->size();
	for (size_t i = 0; i < nTris; ++i)
	{
		const size_t& triIx = vTriangleIndex->at(i);
		const int& geomIx = vTriangle->at(triIx)._geomIndex;
		vvTriIx[geomIx].emplace_back(triIx);
	}


	for (auto& vtriIx : vvTriIx)
	{
		vtriIx.shrink_to_fit();
	}

	std::set<size_t> vistedTriIx;

	//step1: 
	for (size_t i = 0; i < vvTriIx.size(); ++i)
	{
		std::vector<size_t>& vTriIx = vvTriIx[i]; //第i个geom


		if (!vTriIx.size())
		{
			continue;
		}


		std::stack<size_t> unvisitedVertex;

		std::set<size_t> visitedVertexIx;

		for (auto& triIx : vTriIx)
		{
			auto& v1Ix = vTriangle->at(triIx)._vertexIndexs[0];
			auto& v2Ix = vTriangle->at(triIx)._vertexIndexs[1];
			auto& v3Ix = vTriangle->at(triIx)._vertexIndexs[2];

			if (visitedVertexIx.find(v1Ix) == visitedVertexIx.end()) {
				unvisitedVertex.push(v1Ix);
				visitedVertexIx.insert(v1Ix);
			}
			if (visitedVertexIx.find(v2Ix) == visitedVertexIx.end()) {
				unvisitedVertex.push(v2Ix);
				visitedVertexIx.insert(v2Ix);
			}
			if (visitedVertexIx.find(v3Ix) == visitedVertexIx.end()) {
				unvisitedVertex.push(v3Ix);
				visitedVertexIx.insert(v3Ix);
			}

			vistedTriIx.insert(triIx);
		}

		visitedVertexIx.clear();
		size_t* currentVertexIx;

		while (!unvisitedVertex.empty())
		{
			currentVertexIx = &unvisitedVertex.top();

			unvisitedVertex.pop();
			visitedVertexIx.insert(*currentVertexIx);

			auto it = _vmPTop[i].find(*currentVertexIx);
			if (it != _vmPTop[i].end())
			{
				auto& triset = it->second;
				for (const size_t& triIx : triset)
				{

					if (vistedTriIx.find(triIx) == vistedTriIx.end())
					{

						const Triangle& triangle = vTriangle->at(triIx);
						const size_t& v1Ix = triangle._vertexIndexs[0];
						const size_t& v2Ix = triangle._vertexIndexs[1];
						const size_t& v3Ix = triangle._vertexIndexs[2];

						if (visitedVertexIx.find(v1Ix) == visitedVertexIx.end()) {
							unvisitedVertex.push(v1Ix);
						}
						if (visitedVertexIx.find(v2Ix) == visitedVertexIx.end()) {
							unvisitedVertex.push(v2Ix);
						}
						if (visitedVertexIx.find(v3Ix) == visitedVertexIx.end()) {
							unvisitedVertex.push(v3Ix);
						}

						vistedTriIx.insert(triIx);
					}
				}
			}
			else
			{
				std::cerr << "error in TopologyConnect" << std::endl;
				return;
			}
		}
	}

	//set 转为 vector
	vTriangleIndex_out->assign(vistedTriIx.begin(), vistedTriIx.end());

	auto entT = std::chrono::system_clock::now();//获取结束时间
	auto ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
	std::cout << " **** Time elipsed in bruteSearchByDFS func: " << (double)ut / 1000 << "s" << std::endl;
}