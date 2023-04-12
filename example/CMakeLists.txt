cmake_minimum_required(VERSION 3.14)

project(pcl_cloud_spanExamples CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)

if(PROJECT_IS_TOP_LEVEL)
  find_package(pcl_cloud_span REQUIRED)
endif()

add_custom_target(run-examples)

function(add_example NAME)
  add_executable("${NAME}" "${NAME}.cpp")
  target_link_libraries("${NAME}" PRIVATE pcl_cloud_span::pcl_cloud_span)
  target_compile_features("${NAME}" PRIVATE cxx_std_14)
  add_custom_target("run_${NAME}" COMMAND "${NAME}" VERBATIM)
  add_dependencies("run_${NAME}" "${NAME}")
  add_dependencies(run-examples "run_${NAME}")
endfunction()

add_example(voxel_grid_benchmark)
find_package(PCL REQUIRED COMPONENTS filters io)
target_link_directories(voxel_grid_benchmark PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(voxel_grid_benchmark PRIVATE ${PCL_LIBRARIES} pcl_io_ply)

add_folders(Example)
