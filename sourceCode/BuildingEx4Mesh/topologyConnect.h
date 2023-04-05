#include <map>
#include <unordered_map>
#include <set>

#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/vertex.h"

class TopologyConnect
{

public:
	TopologyConnect();
	~TopologyConnect();
	

	void createConnectByVertex(const std::vector<Triangle>* vTriangle, 
								const std::vector<std::vector<Vertex>>* vvVertex);


	void bruteSearchByDFS(const std::vector<size_t>* vTriangleIndex,
		                  std::vector<size_t>* vTriangleIndex_out,
						  const std::vector<Triangle>* vTriangle, 
						  const std::vector<std::vector<Vertex>>* vvVertex) const;


	void bruteSearchByDFS_deprecated(const std::vector<size_t>* vTriangleIndex,
		std::vector<size_t>* vTriangleIndex_out,
		const std::vector<Triangle>* vTriangle,
		const std::vector<std::vector<Vertex>>* vvVertex) const;


	void bruteSearchByDFS(const std::vector<std::vector<size_t>>*  vvTriIx,
						  std::vector<size_t>* vTriangleIndex_out,
						  const std::vector<Triangle>* vTriangle,
						  const std::vector<std::vector<Vertex>>* vvVertex) const;

private:

	std::vector< std::map<size_t, std::set<size_t>>> _vmPTop;
};