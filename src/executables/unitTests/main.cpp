#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "Vector3d.hpp"
#include "Utility.hpp"
#include<iostream>
#include "DetailedMesh.hpp"

TEST_CASE( "Point initialization", "Point" )
{
    CRWCompGeo::Vector3d point(1.0, 2.0, 3.0);

    REQUIRE(point.x == Approx(1.0));
    REQUIRE(point.y == Approx(2.0));
    REQUIRE(point.z == Approx(3.0));
}

TEST_CASE( "EraseFromVectorWithoutPreservingOrder", "Utility" )
{
    std::vector<int64_t> vector = {0, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<int64_t> elements_to_remove = {0, 2, 4, 7, 9};

    CRWCompGeo::EraseFromVectorWithoutPreservingOrder(elements_to_remove, vector);

    REQUIRE(vector.size() == 10);
    for(int64_t required_value : {1, 3, 5, 6, 8})
    {
        REQUIRE(std::find(vector.begin(), vector.end(), required_value) != vector.end());
    }

    for(int64_t removed_value : elements_to_remove)
    {
        REQUIRE(std::find(vector.begin(), vector.end(), removed_value) == vector.end());
    }
}

TEST_CASE( "Detailed Mesh Triangle Normals", "DetailedMesh" )
{
    CRWCompGeo::Mesh basic_mesh;
    basic_mesh.Vertices.emplace_back(CRWCompGeo::Vector3d(0.0, 0.0, 0.0));
    basic_mesh.Vertices.emplace_back(CRWCompGeo::Vector3d(1.0, 0.0, 0.0));
    basic_mesh.Vertices.emplace_back(CRWCompGeo::Vector3d(1.0, 1.0, 0.0));

    basic_mesh.Triangles.emplace_back(0, 1, 2);

    CRWCompGeo::DetailedMesh detailed_mesh(basic_mesh);

    for(const CRWCompGeo::TriangleMeta& triangle_meta : detailed_mesh.TriangleMetas)
    {
        REQUIRE(length(triangle_meta.Normal) == Approx(1.0));
    }

    for(const CRWCompGeo::VertexMeta& vertex_meta : detailed_mesh.VertexMetas)
    {
        REQUIRE(length(vertex_meta.Normal) == Approx(1.0));
        REQUIRE(vertex_meta.Normal.x == 0.0);
        REQUIRE(vertex_meta.Normal.y == 0.0);
        REQUIRE(vertex_meta.Normal.z == 1.0);
    }
}
