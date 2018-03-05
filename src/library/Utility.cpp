#include "Utility.hpp"
#include<set>
#include<queue>
#include<tuple>

using namespace CRWCompGeo;

double DetermineMaxNeighborDisplacementOnNormal(int64_t vertexIndex, const DetailedMesh& mesh)
{
const Vertex& vertex = mesh.Vertices[vertexIndex];
const VertexMeta& vertex_meta = mesh.VertexMetas[vertexIndex];

double max_displacement = std::numeric_limits<double>::lowest();

for(int64_t connected_vertex_index : vertex_meta.ConnectedVertices)
{
const Vertex& connected_vertex = mesh.Vertices[connected_vertex_index];
Vector3d position_difference = connected_vertex.Location - vertex.Location;

max_displacement = std::max(max_displacement, std::fabs(linalg::dot(position_difference, vertex_meta.Normal)));
}

return max_displacement;
}

int64_t GetMinNumberOfVerticesConnectedToEdgeVertices(int64_t edgeIndex, const DetailedMesh& mesh)
{
const EdgeMeta& edge_meta = mesh.EdgeMetas[edgeIndex];

return std::min(mesh.VertexMetas[edge_meta.Vertices[0]].ConnectedVertices.size(), mesh.VertexMetas[edge_meta.Vertices[1]].ConnectedVertices.size());
}

double GetMaxDisplacementForEdge(int64_t edgeIndex, const DetailedMesh& mesh)
{
const EdgeMeta& edge_meta = mesh.EdgeMetas[edgeIndex];

double max_displacement = std::numeric_limits<double>::lowest();
for(int64_t vertex_index : edge_meta.Vertices)
{
max_displacement = std::max(max_displacement, DetermineMaxNeighborDisplacementOnNormal(vertex_index, mesh));
}

return max_displacement;
}




/**
This function uses edge contraction to attempt to simplify the mesh.
@param maxAllowedDisplacement: How far a neighbor is allowed to be displaced from the plane of the vertex before the vertex should not be considered for simplification
@param numConnectedFacesMin: How many connected triangles a vertex must have to be considered for removal (negative if not needed)
@param mesh: The mesh to simplify
*/
void CRWCompGeo::SimplifyMesh(double maxAllowedDisplacement, int64_t numConnectedMin, DetailedMesh& mesh)
{
std::set<int64_t> blacklisted_vertices;
std::set<int64_t> blacklisted_edges;
std::set<int64_t> blacklisted_triangles;
std::priority_queue<std::pair<double, int64_t>> negative_display_mag_and_edge_index_queue; 

auto CheckAndPotentiallyAddEdge = [&](int64_t edgeIndex)
{
if(blacklisted_edges.count(edgeIndex) != 0)
{
//Skip blacklisted edge
return;
}

int64_t min_connected_vertices = GetMinNumberOfVerticesConnectedToEdgeVertices(edgeIndex, mesh);
if(min_connected_vertices < numConnectedMin)
{
return;
}

double max_displacement = GetMaxDisplacementForEdge(edgeIndex, mesh);

if(max_displacement > maxAllowedDisplacement)
{
return;
}

negative_display_mag_and_edge_index_queue.push({-max_displacement, edgeIndex});
};

//Consider all edges for removal
for(int64_t edge_index = 0; edge_index < mesh.EdgeMetas.size(); edge_index++)
{
CheckAndPotentiallyAddEdge(edge_index);
}

while(negative_display_mag_and_edge_index_queue.size() > 0)
{
double negative_stored_edge_max_displacement = std::numeric_limits<double>::lowest();
int64_t edge_index = -1;

//Get edge with the smallest displacement
std::tie(negative_stored_edge_max_displacement, edge_index) = negative_display_mag_and_edge_index_queue.top();
negative_display_mag_and_edge_index_queue.pop();

bool displacement_outdated = negative_stored_edge_max_displacement != (-GetMaxDisplacementForEdge(edge_index, mesh));
bool edge_blacklisted = blacklisted_edges.count(edge_index) > 0;

if(edge_blacklisted || displacement_outdated)
{
continue; //Skip this edge
}

const EdgeMeta& edge_meta = mesh.EdgeMetas[edge_index];

//Black list the edge/vertices, make a new vertex, proprogate change, add effected edges to queue
blacklisted_edges.emplace(edge_index);
for(int64_t edge_triangle : edge_meta.ConnectedTriangles)
{
blacklisted_triangles.emplace(edge_triangle);
}
blacklisted_vertices.emplace(edge_meta.Vertices[0]);
blacklisted_vertices.emplace(edge_meta.Vertices[1]);

//Make new vertex at midpoint of edge and merge in the connections from the two vertices it replaces
Vector3d midpoint = .5*(mesh.Vertices[edge_meta.Vertices[0]].Location + mesh.Vertices[edge_meta.Vertices[1]].Location);
int64_t new_vertex_index = mesh.Vertices.size();
mesh.Vertices.emplace_back(midpoint);
mesh.VertexMetas.emplace_back();
VertexMeta& new_vertex_meta = mesh.VertexMetas.back();

const VertexMeta& original_vertex_0_meta = mesh.VertexMetas[edge_meta.Vertices[0]];
const VertexMeta& original_vertex_1_meta = mesh.VertexMetas[edge_meta.Vertices[1]];

new_vertex_meta.ConnectedVertices.insert(new_vertex_meta.ConnectedVertices.end(),
                                         original_vertex_0_meta.ConnectedVertices.begin(), original_vertex_0_meta.ConnectedVertices.end());

new_vertex_meta.ConnectedVertices.insert(new_vertex_meta.ConnectedVertices.end(),
                                         original_vertex_1_meta.ConnectedVertices.begin(), original_vertex_1_meta.ConnectedVertices.end());
EraseFromVectorWithoutPreservingOrder(edge_meta.Vertices, new_vertex_meta.ConnectedVertices);
SortAndRemoveDuplicates(new_vertex_meta.ConnectedVertices);

new_vertex_meta.ConnectedEdges.insert(new_vertex_meta.ConnectedEdges.end(),
                                      original_vertex_0_meta.ConnectedEdges.begin(), original_vertex_0_meta.ConnectedEdges.end());
new_vertex_meta.ConnectedEdges.insert(new_vertex_meta.ConnectedEdges.end(),
                                      original_vertex_1_meta.ConnectedEdges.begin(), original_vertex_1_meta.ConnectedEdges.end());
EraseFromVectorWithoutPreservingOrder(std::initializer_list<int64_t>{edge_index}, new_vertex_meta.ConnectedEdges);
SortAndRemoveDuplicates(new_vertex_meta.ConnectedEdges);

new_vertex_meta.ConnectedTriangles.insert(new_vertex_meta.ConnectedTriangles.end(),
                                          original_vertex_0_meta.ConnectedTriangles.begin(), original_vertex_0_meta.ConnectedTriangles.end());
new_vertex_meta.ConnectedTriangles.insert(new_vertex_meta.ConnectedTriangles.end(),
                                          original_vertex_1_meta.ConnectedTriangles.begin(), original_vertex_1_meta.ConnectedTriangles.end());
EraseFromVectorWithoutPreservingOrder(edge_meta.ConnectedTriangles, new_vertex_meta.ConnectedTriangles);
SortAndRemoveDuplicates(new_vertex_meta.ConnectedTriangles);

//Update connected triangles
for(int64_t triangle_index : new_vertex_meta.ConnectedTriangles)
{
Triangle& triangle = mesh.Triangles[triangle_index];

for(int64_t& vertex_index : triangle.Vertices)
{
    if((vertex_index == edge_meta.Vertices[0]) || (vertex_index == edge_meta.Vertices[1]))
    {
        vertex_index = new_vertex_index;
    }
}

TriangleMeta& triangle_meta = mesh.TriangleMetas[triangle_index];
triangle_meta.Normal = ComputeTriangleNormal(triangle, mesh);
}

//Update connected edges
for(int64_t connected_edge_index : new_vertex_meta.ConnectedEdges)
{
    EdgeMeta& connected_edge_meta = mesh.EdgeMetas[connected_edge_index];
    EraseFromVectorWithoutPreservingOrder(edge_meta.ConnectedTriangles, connected_edge_meta.ConnectedTriangles);
    SortAndRemoveDuplicates(connected_edge_meta.ConnectedTriangles);
}

//Update connected vertices
for(int64_t connected_vertex_index : new_vertex_meta.ConnectedVertices)
{
    VertexMeta& connected_vertex_meta = mesh.VertexMetas[connected_vertex_index];

    EraseFromVectorWithoutPreservingOrder(edge_meta.Vertices, connected_vertex_meta.ConnectedVertices);
    connected_vertex_meta.ConnectedVertices.emplace_back(new_vertex_index);
    SortAndRemoveDuplicates(connected_vertex_meta.ConnectedVertices);

    EraseFromVectorWithoutPreservingOrder(edge_meta.ConnectedTriangles, connected_vertex_meta.ConnectedTriangles);
    SortAndRemoveDuplicates(connected_vertex_meta.ConnectedTriangles);

    connected_vertex_meta.Normal = ComputeVertexNormal(connected_vertex_meta, mesh);
}

//Update vertex normal
new_vertex_meta.Normal = ComputeVertexNormal(new_vertex_meta, mesh);

//Recalculate displacements for all connected edges
for(int64_t connected_vertex_index : new_vertex_meta.ConnectedVertices)
{
    const VertexMeta& connected_vertex_meta = mesh.VertexMetas[connected_vertex_index];

    for(int64_t connected_edge_index : connected_vertex_meta.ConnectedEdges)
    {
        CheckAndPotentiallyAddEdge(connected_edge_index);
    }
}

}

}

Vector3d CRWCompGeo::ComputeTriangleNormal(const Triangle& triangle, Mesh& mesh)
{
    const Vertex& vertex0 = mesh.Vertices[triangle.Vertices[0]];
    const Vertex& vertex1 = mesh.Vertices[triangle.Vertices[1]];
    const Vertex& vertex2 = mesh.Vertices[triangle.Vertices[2]];

    std::array<Vector3d, 2> edge_directions{vertex1.Location - vertex0.Location, vertex2.Location - vertex0.Location};

    Vector3d normal_direction = linalg::cross(edge_directions[0], edge_directions[1]);
    return linalg::normalize(normal_direction);
}

Vector3d CRWCompGeo::ComputeVertexNormal(const VertexMeta& vertexMeta, DetailedMesh& mesh)
{
    Vector3d normal(0.0, 0.0, 0.0);

    for(int64_t triangle_index : vertexMeta.ConnectedTriangles)
    {
    normal += mesh.TriangleMetas[triangle_index].Normal;
    }

    normal = linalg::normalize(normal);

    return normal;
}
