#pragma once

namespace CRWCompGeo
{

class VertexMeta
{
public:
std::vector<int64_t> ConnectedVertices;
std::vector<int64_t> ConnectedEdges;
std::vector<int64_t> ConnectedTriangles;
Vector3d Normal;
};

}
