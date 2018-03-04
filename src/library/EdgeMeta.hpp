#pragma once

#include<vector>
#include<array>

namespace CRWCompGeo
{

class EdgeMeta
{
public:
std::array<int64_t, 2> Vertices;
std::vector<int64_t> ConnectedTriangles;
};

}
