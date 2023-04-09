cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    pcl_cloud_span
    VERSION 0.0.0
    DESCRIPTION "A small header-only library to use point clouds of arbitrary formats in PCL algorithms without unnecessary copying."
    HOMEPAGE_URL "https://github.com/Tristis116/pcl-cloud-span"
    LANGUAGES CXX
)

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

# ---- Declare library ----

add_library(pcl_cloud_span_pcl_cloud_span INTERFACE)
add_library(pcl_cloud_span::pcl_cloud_span ALIAS pcl_cloud_span_pcl_cloud_span)

set_property(
    TARGET pcl_cloud_span_pcl_cloud_span PROPERTY
    EXPORT_NAME pcl_cloud_span
)

target_include_directories(
    pcl_cloud_span_pcl_cloud_span ${warning_guard}
    INTERFACE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)

target_compile_features(pcl_cloud_span_pcl_cloud_span INTERFACE cxx_std_14)

find_package(PCL REQUIRED COMPONENTS common)
target_link_libraries(pcl_cloud_span_pcl_cloud_span INTERFACE ${PCL_INCLUDE_LIBS})

# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Examples ----

if(PROJECT_IS_TOP_LEVEL)
  option(BUILD_EXAMPLES "Build examples tree." "${pcl_cloud_span_DEVELOPER_MODE}")
  if(BUILD_EXAMPLES)
    add_subdirectory(example)
  endif()
endif()

# ---- Developer mode ----

if(NOT pcl_cloud_span_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of pcl_cloud_span"
  )
endif()

include(cmake/dev-mode.cmake)
