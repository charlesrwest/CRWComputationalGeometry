#pragma once

#include "Mesh.hpp"

namespace CRWCompGeo
{

/**
This type of mesh mantains more information concerning connections betweens it parts (not necessary for all applications).
*/
class DetailedMesh : public Mesh
{
public:
std::vector<VertexMeta> VertexMetas;
};


























}
