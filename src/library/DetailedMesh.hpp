#pragma once

#include "Mesh.hpp"
#include "VertexMeta.hpp"
#include "TriangleMeta.hpp"
#include "EdgeMeta.hpp"

namespace CRWCompGeo
{

/**
This type of mesh mantains more information concerning connections betweens it parts (not necessary for all applications).
*/
class DetailedMesh : public Mesh
{
public:
DetailedMesh() = default;
DetailedMesh(const Mesh& sourceMesh);

std::vector<VertexMeta> VertexMetas;
std::vector<EdgeMeta> EdgeMetas;
std::vector<TriangleMeta> TriangleMetas;
};


























}
