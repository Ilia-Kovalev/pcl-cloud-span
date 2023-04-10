cmake_minimum_required(VERSION 3.14)

project(pcl_cloud_spanTests LANGUAGES CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)

# ---- Dependencies ----

if(PROJECT_IS_TOP_LEVEL)
  find_package(pcl_cloud_span REQUIRED)
  enable_testing()
endif()

find_package(GTest REQUIRED)
include(GoogleTest)

find_package(PCL REQUIRED COMPONENTS filters)

# ---- Tests ----

add_executable(pcl_cloud_span_test "source/filters_test.cpp")
target_link_libraries(
    pcl_cloud_span_test PRIVATE
    pcl_cloud_span::pcl_cloud_span
    GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main
    ${PCL_LIBRARIES}
)
target_compile_features(pcl_cloud_span_test PRIVATE cxx_std_14)
if (MSVC)
  target_compile_options(pcl_cloud_span_test PRIVATE /bigobj)
endif()

gtest_discover_tests(pcl_cloud_span_test)

# ---- End-of-file commands ----

add_folders(Test)
