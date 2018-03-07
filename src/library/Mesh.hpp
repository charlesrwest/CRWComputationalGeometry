#pragma once

#include "Vertex.hpp"
#include "MeshTriangle.hpp"
#include "Vector2d.hpp"


namespace CRWCompGeo
{

class Mesh
{
public:
std::vector<Vertex> Vertices;
std::vector<std::vector<Vector2d>> UVChannelCoordinates; //[channel][vertexIndex] 
std::vector<Triangle> Triangles;
};

























}
