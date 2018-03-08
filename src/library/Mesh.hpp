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
std::vector<Triangle> Triangles;

std::vector<Vector3d> VertexNormals;
std::vector<std::vector<Vector2d>> UVChannelCoordinates; //[channel][vertexIndex] 
std::vector<int64_t> MaterialSectionBounds; //Each entry holds one past the last triangle index of the given section (10 triangles starting at 0 would have 10 in position [0])
};

























}
