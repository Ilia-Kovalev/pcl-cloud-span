#include <pcl_cloud_span/pcl_cloud_span.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <iostream>

using pcl_cloud_span::convertToPCL;
using pcl_cloud_span::makeCloudSpanPtr;
using pcl_cloud_span::Spannable;
using pcl_cloud_span::SpanType;

struct Point {
  union {
    float data[3];
    struct {
      float x;
      float y;
      float z;
    };
  };
  PCL_ADD_EIGEN_MAPS_POINT4D
};

std::ostream&
operator<<(std::ostream& o, const Point& p)
{
  return o << '(' << p.x << ", " << p.y << ", " << p.z << ')';
}

using SpannablePoint = Spannable<Point, SpanType::ReadOnly>;
using CloudSpan = pcl::PointCloud<SpannablePoint>;
using Cloud = pcl::PointCloud<Point>;

POINT_CLOUD_REGISTER_POINT_STRUCT(Point, (float, x, x)(float, y, y)(float, z, z))
POINT_CLOUD_REGISTER_POINT_STRUCT(SpannablePoint,
                                  (float, x, x)(float, y, y)(float, z, z))

using Seconds = double;

template <typename F>
Seconds
measureTime(F&& f)
{
  auto const start = std::chrono::high_resolution_clock::now();
  f();
  auto const end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
}

int
main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cout << "Usage:\nvoxel_grid_benchmark input_ply_file leaf_size "
                 "min_points_per_voxel [output_dir]\nInput ply file should have "
                 "only XYZ fields";
    return 1;
  }

  const pcl::PCLPointCloud2ConstPtr in_cloud = [&]() {
    const auto cloud = std::make_shared<pcl::PCLPointCloud2>();
    if (pcl::io::loadPLYFile(argv[1], *cloud) == -1) {
      PCL_ERROR("Unable to read file");
      throw std::runtime_error("");
    }
    return cloud;
  }();

  const float leaf_size = std::stof(argv[2]);
  const int min_points_per_voxel = std::stoi(argv[3]);
  if (min_points_per_voxel <= 1) {
    PCL_ERROR("min_points_per_voxel should be > 0");
    return 1;
  }

  auto const setupFilter = [&](auto& filter) {
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
  };

  using CaseResult = std::pair<Seconds, pcl::PointCloud<Point>>;
  using CaseOperation = std::function<CaseResult()>;
  using Case = std::pair<std::string, CaseOperation>;

  std::vector<Case> cases = {
      {"native_PointCloud2",
       [&]() {
         pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
         setupFilter(filter);
         pcl::PointCloud<Point> out;
         const auto duration = measureTime([&]() {
           filter.setInputCloud(in_cloud);
           pcl::PCLPointCloud2 out_pc2;
           filter.filter(out_pc2);
           pcl::fromPCLPointCloud2(out_pc2, out);
         });
         return std::make_pair(duration, out);
       }},
      {"native_PointXYZ",
       [&]() {
         pcl::VoxelGrid<Point> filter;
         setupFilter(filter);
         pcl::PointCloud<Point> out;
         const auto duration = measureTime([&]() {
           auto in = std::make_shared<pcl::PointCloud<Point>>();
           pcl::fromPCLPointCloud2(*in_cloud, *in);
           filter.setInputCloud(in);
           filter.filter(out);
         });
         return std::make_pair(duration, out);
       }},
      {"CloudSpan",
       [&]() {
         pcl::VoxelGrid<SpannablePoint> filter;
         setupFilter(filter);
         pcl::PointCloud<Point> out;
         const auto duration = measureTime([&]() {
           const auto in =
               makeCloudSpanPtr(reinterpret_cast<const Point*>(in_cloud->data.data()),
                                in_cloud->width,
                                in_cloud->height);
           filter.setInputCloud(in);
           pcl::PointCloud<SpannablePoint> out_spannable;
           filter.filter(out_spannable);
           out = convertToPCL(std::move(out_spannable));
         });
         return std::make_pair(duration, out);
       }},

  };

  for (const auto c : cases) {
    std::string name;
    CaseOperation op;
    std::tie(name, op) = c;

    Seconds duration;
    pcl::PointCloud<Point> out;
    std::tie(duration, out) = op();
    std::cout << "Duration of " << name << " case: " << duration << "s\n";
    if (argc == 5) {
      const auto file_name = std::string(argv[4]) + "/" + name + ".ply";
      pcl::io::savePLYFile(file_name, out);
    }
  }

  return 0;
}
