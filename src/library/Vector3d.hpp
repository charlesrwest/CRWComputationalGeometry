#pragma once
#include<array>

namespace CRWCompGeo
{

class Vector3d : public union{ std::array<double, 3>; struct {double X, double Y, double Z}};
{
public:
Vector3d();

Vector3d(double x, double y, double z);
}; 



















}
