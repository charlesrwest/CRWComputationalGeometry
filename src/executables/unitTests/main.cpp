#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "Vector3d.hpp"
#include "Utility.hpp"
#include<iostream>

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
