#include "DetailedMesh.hpp"

#include<algorithm>
#include "Utility.hpp"

using namespace CRWCompGeo;

//Returns -1 if nothing found
int64_t FindConnectingEdge(int64_t vertex0, int64_t vertex1, const DetailedMesh& mesh)
{
const VertexMeta& vertex_0_meta = mesh.VertexMetas[vertex0];

for(int64_t edge_index : vertex_0_meta.ConnectedEdges)
{
const EdgeMeta& edge_meta = mesh.EdgeMetas[edge_index];

if(
    ((edge_meta.Vertices[0] == vertex0) && (edge_meta.Vertices[1] == vertex1)) ||
    ((edge_meta.Vertices[1] == vertex0) && (edge_meta.Vertices[0] == vertex1))
    )
{
return edge_index;
}
}

return -1;
}

DetailedMesh::DetailedMesh(const Mesh& sourceMesh) : Mesh(sourceMesh)
{
VertexMetas.resize(Vertices.size());
TriangleMetas.resize(Triangles.size());

//Determine connected triangles and vertices
for(int64_t triangle_index = 0; triangle_index < Triangles.size(); triangle_index++)
{
CRWCompGeo::Triangle& triangle = Triangles[triangle_index];

for(int64_t offset_index = 0; offset_index < triangle.Vertices.size(); offset_index++)
{
int64_t vertex_index = triangle.Vertices[offset_index];

//Link vertex to triangle
VertexMetas[vertex_index].ConnectedTriangles.emplace_back(triangle_index);

std::array<int64_t, 2> other_vertex_indices{triangle.Vertices[(offset_index+1)%triangle.Vertices.size()], triangle.Vertices[(offset_index+2)%triangle.Vertices.size()]};

//Link each vertex in the triangle to each of the others
for(int64_t other_vertex_index : other_vertex_indices)
{
VertexMetas[vertex_index].ConnectedVertices.emplace_back(other_vertex_index);
}

}
}

//Some strange geometries can have vertices linked in more than one triangle, so make sure each of the vectors doesn't have duplicates
for(VertexMeta& vertexMeta : VertexMetas)
{
SortAndRemoveDuplicates(vertexMeta.ConnectedVertices);
}

//Determine triangle normals
for(int64_t triangle_index = 0; triangle_index < Triangles.size(); triangle_index++)
{
const Triangle& triangle = Triangles[triangle_index];
TriangleMeta& triangle_meta = TriangleMetas[triangle_index];

triangle_meta.Normal = ComputeTriangleNormal(triangle, *this);
}

//Determine vertex normals
for(VertexMeta& vertexMeta : VertexMetas)
{
vertexMeta.Normal = ComputeVertexNormal(vertexMeta, *this);
}

//Populate edges
for(int64_t triangle_index = 0; triangle_index < Triangles.size(); triangle_index++)
{
const Triangle& triangle = Triangles[triangle_index];

for(int64_t offset_index = 0; offset_index < triangle.Vertices.size(); offset_index++)
{
int64_t vertex_0_index = triangle.Vertices[offset_index];
int64_t vertex_1_index = triangle.Vertices[(offset_index+1)%triangle.Vertices.size()];

//Check if we have this edge already
int64_t connecting_edge_index = FindConnectingEdge(vertex_0_index, vertex_1_index, *this);
if(connecting_edge_index >= 0)
{
//Just add the triangle to the edge
EdgeMetas[connecting_edge_index].ConnectedTriangles.emplace_back(triangle_index);
}
else
{
//Add the edge
connecting_edge_index = EdgeMetas.size();
EdgeMetas.emplace_back();
EdgeMeta& edge_meta = EdgeMetas.back();

edge_meta.Vertices = {vertex_0_index, vertex_1_index};
edge_meta.ConnectedTriangles.emplace_back(triangle_index);

VertexMetas[vertex_0_index].ConnectedEdges.emplace_back(connecting_edge_index);
VertexMetas[vertex_1_index].ConnectedEdges.emplace_back(connecting_edge_index);
}

}
}


}
