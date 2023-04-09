/*
 * MIT License
 *
 * Copyright (c) 2023 Ilia Kovalev
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifdef _MSC_VER
typedef unsigned long long pop_t;
#endif // _MSC_VER

#include <pcl_cloud_span/pcl_cloud_span.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <gmock/gmock.h>

#include <random>

using pcl_cloud_span::Spannable;
using pcl_cloud_span::SpanType;

using Point = pcl::PointXYZI;
using SpannablePoint = Spannable<Point, SpanType::ReadOnly>;
using CloudSpan = pcl::PointCloud<SpannablePoint>;
using Cloud = pcl::PointCloud<Point>;

POINT_CLOUD_REGISTER_POINT_STRUCT(SpannablePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float,
                                                                          intensity,
                                                                          intensity))

namespace pcl {
bool
operator==(const Point& a, const Point& b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity;
}
} // namespace pcl

TEST(FilterTest, ApproximateVoxelGridTest)
{
  const auto in_cloud = std::make_shared<Cloud>(100, 1, Point{});
  std::default_random_engine eng(0);
  std::uniform_real_distribution<> dis(-1, 1);

  std::generate(in_cloud->begin(), in_cloud->end(), [&]() -> Point {
    return Point(dis(eng), dis(eng), dis(eng));
  });

  const auto in_cloud_span = std::make_shared<const CloudSpan>(
      reinterpret_cast<const SpannablePoint*>(in_cloud->data()), in_cloud->size());

  pcl::ApproximateVoxelGrid<SpannablePoint> filter_span;
  filter_span.setInputCloud(in_cloud_span);
  filter_span.setLeafSize(.4, .4, .4);
  pcl::PointCloud<SpannablePoint> out;
  filter_span.filter(out);

  pcl::ApproximateVoxelGrid<Point> filter;
  filter.setInputCloud(in_cloud);
  filter.setLeafSize(.4, .4, .4);
  pcl::PointCloud<Point> expected;
  filter.filter(expected);

  EXPECT_THAT(pcl_cloud_span::convertToPCL(out).points,
              ::testing::ContainerEq(expected.points));
  EXPECT_EQ(&in_cloud->points[0], &in_cloud_span->points[0]);
}

TEST(FilterTest, BilateralFilterTest)
{
  const auto in_cloud = std::make_shared<Cloud>(100, 1, Point{});
  std::default_random_engine eng(0);
  std::uniform_real_distribution<> coord_dis(-1, 1);
  std::uniform_real_distribution<> intencity_dis(0, 1);

  std::generate(in_cloud->begin(), in_cloud->end(), [&]() -> Point {
    return Point(coord_dis(eng), coord_dis(eng), coord_dis(eng), intencity_dis(eng));
  });

  const auto in_cloud_span = std::make_shared<const CloudSpan>(
      reinterpret_cast<const SpannablePoint*>(in_cloud->data()), in_cloud->size());

  pcl::BilateralFilter<SpannablePoint> filter_span;
  filter_span.setInputCloud(in_cloud_span);
  filter_span.setHalfSize(.3);
  filter_span.setStdDev(.1);
  pcl::PointCloud<SpannablePoint> out;
  filter_span.filter(out);

  pcl::BilateralFilter<Point> filter;
  filter.setInputCloud(in_cloud);
  filter.setHalfSize(.3);
  filter.setStdDev(.1);
  pcl::PointCloud<Point> expected;
  filter.filter(expected);

  EXPECT_THAT(pcl_cloud_span::convertToPCL(out).points,
              ::testing::ContainerEq(expected.points));
  EXPECT_EQ(&in_cloud->points[0], &in_cloud_span->points[0]);
}

TEST(FilterTest, BoxClipper3DTest)
{
  auto in_cloud = Cloud(100, 1, Point{});
  std::default_random_engine eng(0);
  std::uniform_real_distribution<> coord_dis(-1, 1);
  std::uniform_real_distribution<> intencity_dis(0, 1);

  std::generate(in_cloud.begin(), in_cloud.end(), [&]() -> Point {
    return Point(coord_dis(eng), coord_dis(eng), coord_dis(eng), intencity_dis(eng));
  });

  const auto in_cloud_span = CloudSpan(
      reinterpret_cast<const SpannablePoint*>(in_cloud.data()), in_cloud.size());

  pcl::BoxClipper3D<SpannablePoint> filter_span(
      {.1, -.1, 0}, {.1, -.1, .01}, {.3, .3, .3});
  pcl::Indices out;
  filter_span.clipPointCloud3D(in_cloud_span, out);

  pcl::BoxClipper3D<Point> filter({.1, -.1, 0}, {.1, -.1, .01}, {.3, .3, .3});
  pcl::Indices expected;
  filter.clipPointCloud3D(in_cloud, expected);

  EXPECT_THAT(out, ::testing::ContainerEq(expected));
  EXPECT_EQ(&in_cloud.points[0], &in_cloud_span.points[0]);
}

TEST(FilterTest, ConditionalRemovalTest)
{
  const auto in_cloud = std::make_shared<Cloud>(100, 1, Point{});
  std::default_random_engine eng(0);
  std::uniform_real_distribution<> coord_dis(-1, 1);
  std::uniform_real_distribution<> intencity_dis(0, 1);

  std::generate(in_cloud->begin(), in_cloud->end(), [&]() -> Point {
    return Point(coord_dis(eng), coord_dis(eng), coord_dis(eng), intencity_dis(eng));
  });

  const auto in_cloud_span = std::make_shared<const CloudSpan>(
      reinterpret_cast<const SpannablePoint*>(in_cloud->data()), in_cloud->size());

  pcl::ConditionalRemoval<SpannablePoint> filter_span;
  filter_span.setInputCloud(in_cloud_span);
  {
    auto const condition = std::make_shared<pcl::ConditionAnd<SpannablePoint>>();
    auto const comparation = std::make_shared<pcl::FieldComparison<SpannablePoint>>(
        "x", pcl::ComparisonOps::GT, 0.0);
    condition->addComparison(comparation);
    filter_span.setCondition(condition);
  }
  pcl::PointCloud<SpannablePoint> out;
  filter_span.filter(out);

  pcl::ConditionalRemoval<Point> filter;
  filter.setInputCloud(in_cloud);
  {
    auto const condition = std::make_shared<pcl::ConditionAnd<Point>>();
    auto const comparation =
        std::make_shared<pcl::FieldComparison<Point>>(
        "x", pcl::ComparisonOps::GT, 0.0);
    condition->addComparison(comparation);
    filter.setCondition(condition);
  }
  pcl::PointCloud<Point> expected;
  filter.filter(expected);

  EXPECT_THAT(pcl_cloud_span::convertToPCL(out).points,
              ::testing::ContainerEq(expected.points));
  EXPECT_EQ(&in_cloud->points[0], &in_cloud_span->points[0]);
}

TEST(FilterTest, ExtractIndicesTest)
{
  const auto in_cloud = std::make_shared<Cloud>();
  in_cloud->push_back({0, 0, 0});
  in_cloud->push_back({1, 0, 0});
  in_cloud->push_back({2, 0, 0});
  in_cloud->push_back({3, 0, 0});

  const auto in_cloud_span = std::make_shared<const CloudSpan>(
      reinterpret_cast<const SpannablePoint*>(in_cloud->points.data()),
      in_cloud->size());

  const auto in_indices = std::make_shared<pcl::Indices>(pcl::Indices{1, 2});

  pcl::ExtractIndices<SpannablePoint> filter_span;
  filter_span.setInputCloud(in_cloud_span);
  filter_span.setIndices(in_indices);
  pcl::PointCloud<SpannablePoint> out;
  filter_span.filter(out);

  pcl::ExtractIndices<Point> filter;
  filter.setInputCloud(in_cloud);
  filter.setIndices(in_indices);
  pcl::PointCloud<Point> expected;
  filter.filter(expected);

  EXPECT_THAT(pcl_cloud_span::convertToPCL(out).points,
              ::testing::ContainerEq(expected.points));
  EXPECT_EQ(&in_cloud->points[0], &in_cloud_span->points[0]);
}

TEST(FilterTest, VoxelGridTest)
{
  const auto in_cloud = std::make_shared<Cloud>(100, 1, Point{});
  std::default_random_engine eng(0);
  std::uniform_real_distribution<> dis(-1, 1);

  std::generate(in_cloud->begin(), in_cloud->end(), [&]() -> Point {
    return Point(dis(eng), dis(eng), dis(eng));
  });

  const auto in_cloud_span = std::make_shared<const CloudSpan>(
      reinterpret_cast<const SpannablePoint*>(in_cloud->data()), in_cloud->size());

  pcl::VoxelGrid<SpannablePoint> filter_span;
  filter_span.setInputCloud(in_cloud_span);
  filter_span.setLeafSize(.4, .4, .4);
  filter_span.setMinimumPointsNumberPerVoxel(2);
  pcl::PointCloud<SpannablePoint> out;
  filter_span.filter(out);

  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(in_cloud);
  filter.setLeafSize(.4, .4, .4);
  filter.setMinimumPointsNumberPerVoxel(2);
  pcl::PointCloud<Point> expected;
  filter.filter(expected);

  EXPECT_THAT(pcl_cloud_span::convertToPCL(out).points,
              ::testing::ContainerEq(expected.points));
  EXPECT_EQ(&in_cloud->points[0], &in_cloud_span->points[0]);
}
