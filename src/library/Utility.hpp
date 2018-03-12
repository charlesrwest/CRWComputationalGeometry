#pragma once

#include "DetailedMesh.hpp"
#include<algorithm>

namespace CRWCompGeo
{

/**
This function uses edge contraction to attempt to simplify the mesh.
@param maxAllowedDisplacement: How far a neighbor is allowed to be displaced from the plane of the vertex before the vertex should not be considered for simplification
@param numConnectedFacesMin: How many connected triangles a vertex must have to be considered for removal (negative if not needed)
@param numConnectedFacesMax: How many connected triangles a vertex can have before it should no longer be considered for edge collapse
@param mesh: The mesh to simplify
*/
void SimplifyMesh(double maxAllowedDisplacement, int64_t numConnectedMin, int64_t numConnectedFacesMax, DetailedMesh& mesh);





Vector3d ComputeTriangleNormal(const Triangle& triangle, const Mesh& mesh);
Vector3d ComputeVertexNormal(const VertexMeta& vertexMeta, DetailedMesh& mesh);

std::vector<Vector3d> ComputeVertexNormals(const Mesh& mesh);

template <class Type, class Container>
void EraseFromVectorWithoutPreservingOrder(const Container& elementsToDelete, std::vector<Type>& vector) 
{
int64_t number_of_elements_to_remove = 0;

auto ShouldDelete = [&](const Type& value)
{
    return std::find(elementsToDelete.begin(), elementsToDelete.end(), value) != elementsToDelete.end();
};

for(int64_t element_offset = 0; (element_offset + number_of_elements_to_remove) < vector.size(); element_offset++)
{
if(ShouldDelete(vector[element_offset]))
{
//Find an element at the end that should not be swapped
for(; (number_of_elements_to_remove + element_offset + 1) < vector.size(); number_of_elements_to_remove++)
{
    if(!ShouldDelete(vector[vector.size() - (number_of_elements_to_remove+1)]))
    {
            break;
    }
}
//Swap with the last element that should not be removed
std::swap(vector[element_offset], vector[vector.size() - (number_of_elements_to_remove+1)]);
number_of_elements_to_remove++;
}

} 

vector.resize(vector.size() - number_of_elements_to_remove);
}

template <class Container>
void SortAndRemoveDuplicates(Container& container)
{
    std::sort(container.begin(), container.end());
    auto last = std::unique(container.begin(), container.end());

    //Erase duplicates
    container.erase(last, container.end());
}




}
