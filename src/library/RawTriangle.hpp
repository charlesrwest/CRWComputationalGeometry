#pragma once

#include "Vector3d.hpp"
#include<array>

namespace CRWCompGeo
{


class RawTriangle
{
public:
RawTriangle(const Vector3d& corner1, const Vector3d& corner2, const Vector3d& corner3);

std::array<Vector3d, 3> Corners;
};



























}
