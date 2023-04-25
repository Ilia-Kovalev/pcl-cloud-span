# PCL Cloud Span
[![Build](https://github.com/Tristis116/pcl-cloud-span/actions/workflows/ci.yml/badge.svg)](https://github.com/Tristis116/pcl-cloud-span/actions/workflows/ci.yml) 

Header-only cross-platform library to pass existing point cloud data in arbitrary format to PCL (Point Cloud Library)
algorithms without unnecessary copy.


# Rationale

Point cloud is a data structure that can have enormous memory footprint. Because of that it may be very
expensive to copy, especially on embedded platforms.

PCL stores point cloud data as `std::vector` of one of PCL point types. So if you have a point cloud in
another format (for example, ROS `PointCloud2`), you have to copy the whole point cloud to use it as input
to PCL algorithms. 

Eliminating redundant copying allows to improve point cloud processing speed in pipelines that are not
pure PCL from end to end.

# How it works

PCL heavily uses templates across the whole library to generalize point cloud processing algorithms for
all types of points including custom point types. A container for points is a templated class
`pcl::PointCloud<PointT>`.

This library utilizes this template nature of PCL to specialize `pcl::PointCloud` for special point type.
This specialization could use `std::span`-like wrapper to access to points data via pointer to it, but some of
PCL algorithms (for example, filters) has the same type for input and output point cloud. So it
is impossible to use just `std::span`-like wrapper because PCL algorithms need to create a new point cloud
inside.

To solve this problem this library uses a special container that has the same interface and behavior as
`std::vector`, but can be initialized as a span over a pointer to non-PCL data. See
[span_or_vector](https://github.com/Tristis116/span-or-vector) for more details about the container.

# Usage

Here is a minimal example how to use this library with `pcl::VoxelGrid` and point cloud data in ROS format: 

```cpp

// Include the main header of this library
#include <pcl_cloud_span/pcl_cloud_span.h>

// Include necessary PCL headers
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>


// Wrap a point type you use with `Spannable` template class and register it in the
// GLOBAL scope. This should be done only once per point type.
//
// The wrapped point type has the same fields as a point type it wraps, so
// you can just copy registration code from pcl/impl/point_types.hpp file and wrap the
// point type name with `Spannable` template.
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl_cloud_span::Spannable<pcl::PointXYZI>,
                                  (float, x, x)(float, y, y)(float, z, z)(float,
                                                                          intensity,
                                                                          intensity))

// Function that downsamples point cloud of ROS format and returns result as
// pcl::PointCloud
pcl::PointCloud<pcl::PointXYZI>
downsample(pcl::PCLPointCloud2& input)
{
  // Create filter
  pcl::VoxelGrid<pcl_cloud_span::Spannable<pcl::PointXYZI>> filter;
  filter.setLeafSize(.1, .1, .1);

  // Create point cloud span
  const auto cloud_span = pcl_cloud_span::makeCloudSpanPtr(
      reinterpret_cast<pcl::PointXYZI*>(input.data.data()), input.width, input.height);

  // Set the span to filter as input
  filter.setInputCloud(cloud_span);

  // Get filtered point cloud
  pcl::PointCloud<pcl_cloud_span::Spannable<pcl::PointXYZI>> result;
  filter.filter(result);

  // Return the output as pcl::PointCloud<pcl::PointXYZI> without copying point data
  return pcl_cloud_span::convertToPCL(std::move(result));
}

```

## Important note for using pcl::PointXYZ

`pcl::PointXYZ` represents 3D points but in fact it is a structure of 4 floats. So it you use
it with a cloud span with 3D point data then you'll go out of range. To avoid it you should create
a custom 3D point type. You can use [this example](example/voxel_grid_benchmark.cpp) to see how it
can be implemented.

# Performance test

[The test](example/voxel_grid_benchmark.cpp) imitates ROS environment with PointCloud2 point cloud as an input. Then pcl::VoxelGrid is applied.
The result of the whole pipeline is `pcl::PointCloud` that can be used later in usual way. Only downsampling
process with necessary type conversions is measured.

Input data is Lucy sample from [Stanford dataset](http://graphics.stanford.edu/data/3Dscanrep/).
It's a point cloud of 58,241,932 points with only XYZ coordinates.

Downsampling parameters:

- leaf size = 5
- min points per voxel = 20

Platform:

- compiler: MSVC 143
- CPU: AMD Ryzen 5 3600X

Test cases:

1) Using `pcl::PCLPointCloud2` as is and then converting it to `pcl::PointCloud`
2) Convert to `pcl::PointCloud` before filter and leave output without changes
3) Use a cloud span over input data and convert result to regular `pcl::PointCloud` type.

Average processing time for 10 runs:

| Test case | Average time |
| - | - |
| 1. native PointCloud2 | 0.7855245 |
| 2. native PointCloud | 0.5327120 |
| 3. cloud span | 0.4956996 |

In this conditions cloud span usage provides 7.5% speed-up because of unnecessary copying elimination.

# Building and installing

See the [BUILDING](BUILDING.md) document.

# Contributing

See the [CONTRIBUTING](CONTRIBUTING.md) document.

# Licensing

See the [LICENSE](LICENSE.txt) document
