#pragma once
#include<array>


namespace CRWCompGeo
{


class Triangle
{
public:
Triangle(int64_t vertex1, int64_t vertex2, int64_t vertex3);

std::array<int64_t, 3> Vertices;
};

























}
