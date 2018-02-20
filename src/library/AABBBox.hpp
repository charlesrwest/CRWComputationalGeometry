#pragma once
#include "Vector3d.hpp"

namespace CRWCompGeo
{

class AABBBox
{
public:
AABBBox(const Vector3d& center, const Vector3d& halfExtents);

Vector3d Center;
Vector3d HalfExtents;
};


}
