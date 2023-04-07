#include "pcl_cloud_span/pcl_cloud_span.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Name is pcl_cloud_span", "[library]")
{
  REQUIRE(name() == "pcl_cloud_span");
}
