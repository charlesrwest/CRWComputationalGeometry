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

int64_t GetMaxNumberOfVerticesConnectedToEdgeVertices(int64_t edgeIndex, const DetailedMesh& mesh)
{
const EdgeMeta& edge_meta = mesh.EdgeMetas[edgeIndex];

return std::max(mesh.VertexMetas[edge_meta.Vertices[0]].ConnectedVertices.size(), mesh.VertexMetas[edge_meta.Vertices[1]].ConnectedVertices.size());
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

template<class ContainerToUpdateType>
void UpdateConnections(const std::vector<int64_t>& newToOldDictionary, ContainerToUpdateType& vectorToUpdate)
{
for(int64_t& value : vectorToUpdate)
{
    value = newToOldDictionary[value];
}
}

bool VertexEdgeConnectionsAreConsistent(int64_t vertexIndex, const DetailedMesh& mesh)
{
    const VertexMeta& vertex_meta = mesh.VertexMetas[vertexIndex];

    std::vector<int64_t> connected_edge_vertices;

    for(int64_t connected_edge_index : vertex_meta.ConnectedEdges)
    {
        const EdgeMeta& edge_meta = mesh.EdgeMetas[connected_edge_index];

        for(int64_t edge_vertex : edge_meta.Vertices)
        {
            if(edge_vertex == vertexIndex)
            {
                continue;
            }

            if(std::find(vertex_meta.ConnectedVertices.begin(), vertex_meta.ConnectedVertices.end(), edge_vertex) != vertex_meta.ConnectedVertices.end())
            {
                connected_edge_vertices.emplace_back(edge_vertex);
            }
            else
            {
                return false; //Found edge connected to vertex not in the connected vertex list
            }
        }
    }

    for(int64_t connected_vertex_index : vertex_meta.ConnectedVertices)
    {
        if(std::find(connected_edge_vertices.begin(), connected_edge_vertices.end(), connected_vertex_index) == connected_edge_vertices.end())
        {
            return false; //Found a connected vertex that no edge points to
        }
    }

    return true;
}

bool CheckGlobalVertexEdgeConsistency(const DetailedMesh& mesh)
{
    for(int64_t vertex_index = 0; vertex_index < mesh.Vertices.size(); vertex_index++)
    {
        if(!VertexEdgeConnectionsAreConsistent(vertex_index, mesh))
        {
            return false;
        }
    }

    return true;
}

void CRWCompGeo::SimplifyMesh(double maxAllowedDisplacement, int64_t numConnectedMin, int64_t numConnectedFacesMax, DetailedMesh& mesh)
{
std::set<int64_t> blacklisted_vertices;
std::set<int64_t> blacklisted_edges;
std::set<int64_t> blacklisted_triangles;

std::set<int64_t> border_edges;
std::priority_queue<std::pair<double, int64_t>> negative_display_mag_and_edge_index_queue; 

auto CheckAndPotentiallyAddEdge = [&](int64_t edgeIndex)
{
if(blacklisted_edges.count(edgeIndex) != 0)
{
//Skip blacklisted edge
return;
}

if(border_edges.count(edgeIndex) > 0)
{
    return;
}

int64_t max_connected_vertices = GetMaxNumberOfVerticesConnectedToEdgeVertices(edgeIndex, mesh);
if(max_connected_vertices > numConnectedFacesMax)
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

//Consider all edges for inclusion in border edge list or removal
for(int64_t edge_index = 0; edge_index < mesh.EdgeMetas.size(); edge_index++)
{
    int64_t min_connected_vertices = GetMinNumberOfVerticesConnectedToEdgeVertices(edge_index, mesh);
    if(min_connected_vertices < numConnectedMin)
    {
        border_edges.emplace(edge_index);
        continue;
    }

    CheckAndPotentiallyAddEdge(edge_index);
}

int64_t edge_removal_iteration = 0;
while(negative_display_mag_and_edge_index_queue.size() > 0)
{
double negative_stored_edge_max_displacement = std::numeric_limits<double>::lowest();
int64_t edge_index = -1;

//Get edge with the smallest displacement
std::tie(negative_stored_edge_max_displacement, edge_index) = negative_display_mag_and_edge_index_queue.top();
negative_display_mag_and_edge_index_queue.pop();

bool displacement_outdated = negative_stored_edge_max_displacement != (-GetMaxDisplacementForEdge(edge_index, mesh));
bool edge_blacklisted = blacklisted_edges.count(edge_index) > 0;

int64_t max_connected_vertices = GetMaxNumberOfVerticesConnectedToEdgeVertices(edge_index, mesh);
bool too_many_connected_vertices = max_connected_vertices > numConnectedFacesMax;

if(edge_blacklisted || displacement_outdated || too_many_connected_vertices)
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
const Vector3d& original_vertex_0_location = mesh.Vertices[edge_meta.Vertices[0]].Location;
const Vector3d& original_vertex_1_location = mesh.Vertices[edge_meta.Vertices[1]].Location;
Vector3d midpoint = .5*(original_vertex_0_location + original_vertex_1_location);
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

//Update connected edges
for(int64_t connected_edge_index : new_vertex_meta.ConnectedEdges)
{
    EdgeMeta& connected_edge_meta = mesh.EdgeMetas[connected_edge_index];
    EraseFromVectorWithoutPreservingOrder(edge_meta.ConnectedTriangles, connected_edge_meta.ConnectedTriangles);
    SortAndRemoveDuplicates(connected_edge_meta.ConnectedTriangles);

    for(int64_t& vertex_index : connected_edge_meta.Vertices)
    {
        if(std::find(edge_meta.Vertices.begin(), edge_meta.Vertices.end(), vertex_index) != edge_meta.Vertices.end())
        {
            vertex_index = new_vertex_index;
        }
    }
}

//Find any edges which all connect the new vertex to the same vertex
for(int64_t connected_vertex_index : new_vertex_meta.ConnectedVertices)
{
    std::vector<int64_t> duplicate_edges;

    for(int64_t connected_edge_index : new_vertex_meta.ConnectedEdges)
    {
        const EdgeMeta& edge_meta = mesh.EdgeMetas[connected_edge_index];

        if(std::find(edge_meta.Vertices.begin(), edge_meta.Vertices.end(), connected_vertex_index) != edge_meta.Vertices.end())
        {
            duplicate_edges.emplace_back(connected_edge_index);
        }
    }

    if(duplicate_edges.size() == 1)
    {
        continue; //No duplicates
    }

    //Keep the last edge as the survivor and merge all of the triangles all of the edges touch into it
    EdgeMeta& survivor_edge = mesh.EdgeMetas[duplicate_edges.back()];
    for(int64_t duplicate_edge_index = 0; (duplicate_edge_index+1) < duplicate_edges.size(); duplicate_edge_index++)
    {
        EdgeMeta& duplicate_edge = mesh.EdgeMetas[duplicate_edges[duplicate_edge_index]];

        survivor_edge.ConnectedTriangles.insert(survivor_edge.ConnectedTriangles.end(), duplicate_edge.ConnectedTriangles.begin(), duplicate_edge.ConnectedTriangles.end());
    }
    SortAndRemoveDuplicates(survivor_edge.ConnectedTriangles);

    //Update both vertices to remove all references to any duplicate edges besides the survivor
    duplicate_edges.resize(duplicate_edges.size() - 1);

    EraseFromVectorWithoutPreservingOrder(duplicate_edges, new_vertex_meta.ConnectedEdges);
    SortAndRemoveDuplicates(new_vertex_meta.ConnectedEdges);

    VertexMeta& connected_vertex_meta = mesh.VertexMetas[connected_vertex_index];
    EraseFromVectorWithoutPreservingOrder(duplicate_edges, connected_vertex_meta.ConnectedEdges);
    SortAndRemoveDuplicates(connected_vertex_meta.ConnectedEdges);

    //Black list all duplicates except for the survivor
    blacklisted_edges.insert(duplicate_edges.begin(), duplicate_edges.end());
}



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

//Remove all of the blacklisted elements
std::vector<int64_t> old_vertex_index_to_new_index(mesh.Vertices.size(), -1);
std::vector<int64_t> old_edge_index_to_new_index(mesh.EdgeMetas.size(), -1);
std::vector<int64_t> old_triangle_index_to_new_index(mesh.EdgeMetas.size(), -1);

std::vector<Vertex> new_vertices;
std::vector<VertexMeta> new_vertex_metas;

for(int64_t old_vertex_index = 0; old_vertex_index < mesh.Vertices.size(); old_vertex_index++)
{
    if(blacklisted_vertices.count(old_vertex_index) != 0)
    {
        continue;  //Leave index -1
    }

    old_vertex_index_to_new_index[old_vertex_index] = new_vertices.size();
    new_vertices.emplace_back(mesh.Vertices[old_vertex_index]);
    new_vertex_metas.emplace_back(mesh.VertexMetas[old_vertex_index]);
}

std::vector<EdgeMeta> new_edge_metas;

for(int64_t old_edge_index = 0; old_edge_index < mesh.EdgeMetas.size(); old_edge_index++)
{
    if(blacklisted_edges.count(old_edge_index) != 0)
    {
        continue;  //Leave index -1
    }

    old_edge_index_to_new_index[old_edge_index] = new_edge_metas.size();
    new_edge_metas.emplace_back(mesh.EdgeMetas[old_edge_index]);
}

std::vector<Triangle> new_triangles;
std::vector<TriangleMeta> new_triangle_metas;

for(int64_t old_triangle_index = 0; old_triangle_index < mesh.Triangles.size(); old_triangle_index++)
{
    if(blacklisted_triangles.count(old_triangle_index) != 0)
    {
        continue;  //Leave index -1
    }

    old_triangle_index_to_new_index[old_triangle_index] = new_triangles.size();
    new_triangle_metas.emplace_back(mesh.TriangleMetas[old_triangle_index]);
    new_triangles.emplace_back(mesh.Triangles[old_triangle_index]);
}

//Update mesh have the new data members
mesh.Vertices = std::move(new_vertices);
mesh.VertexMetas = std::move(new_vertex_metas);
mesh.EdgeMetas = std::move(new_edge_metas);
mesh.Triangles = std::move(new_triangles);
mesh.TriangleMetas = std::move(new_triangle_metas);

for(VertexMeta& vertex_meta : mesh.VertexMetas)
{
    UpdateConnections(old_vertex_index_to_new_index, vertex_meta.ConnectedVertices);
    UpdateConnections(old_edge_index_to_new_index, vertex_meta.ConnectedEdges);
    UpdateConnections(old_triangle_index_to_new_index, vertex_meta.ConnectedTriangles);
}

for(EdgeMeta& edge_meta : mesh.EdgeMetas)
{
    UpdateConnections(old_vertex_index_to_new_index, edge_meta.Vertices);
    UpdateConnections(old_triangle_index_to_new_index, edge_meta.ConnectedTriangles);
}

for(Triangle& triangle : mesh.Triangles)
{
    UpdateConnections(old_vertex_index_to_new_index, triangle.Vertices);
}
}

Vector3d CRWCompGeo::ComputeTriangleNormal(const Triangle& triangle, const Mesh& mesh)
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

std::vector<Vector3d> CRWCompGeo::ComputeVertexNormals(const Mesh& mesh)
{
std::vector<Vector3d> results;
results.resize(mesh.Vertices.size());

//Compute triangle normals and use them to update vertex normals
for(const Triangle& triangle : mesh.Triangles)
{
Vector3d triangle_normal = ComputeTriangleNormal(triangle, mesh);

for(int64_t vertex_index : triangle.Vertices)
{
results[vertex_index] += triangle_normal;
}
}

for(Vector3d& vertex_normal : results)
{
vertex_normal = linalg::normalize(vertex_normal);
}

return results;
}
