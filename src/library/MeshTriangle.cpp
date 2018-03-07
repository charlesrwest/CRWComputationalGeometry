#include "MeshTriangle.hpp"

using namespace CRWCompGeo;

Triangle::Triangle(int64_t vertex1, int64_t vertex2, int64_t vertex3) : Vertices{-1, -1, -1}
{
}

Triangle::Triangle(int64_t vertex1, int64_t vertex2, int64_t vertex3) : Vertices{vertex1, vertex2, vertex3}
{
}

