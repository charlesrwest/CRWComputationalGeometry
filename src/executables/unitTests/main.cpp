#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "Vector3d.hpp"

TEST_CASE( "Point initialization", "Point" )
{
    CRWCompGeo::Vector3d point(1.0, 2.0, 3.0);

    REQUIRE(point.x == Approx(1.0));
    REQUIRE(point.y == Approx(2.0));
    REQUIRE(point.z == Approx(3.0));
}
