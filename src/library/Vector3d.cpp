#include "Vector3d.hpp"

using namespace CRWCompGeo;

Vector3d::Vector3d() : std::array<double, 3>{{0.0,0.0,0.0}}
{
}

Vector3d::Vector3d(double x, double y, double z) : X(x), Y(y), Z(z)
{
}
