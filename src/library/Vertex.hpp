#pragma once

#include<vector>
#include "Vector3d.hpp"

namespace CRWCompGeo
{

class Vertex
{
public:
Vertex() = default;
Vertex(const Vector3d& location);

Vector3d Location;
};















}
