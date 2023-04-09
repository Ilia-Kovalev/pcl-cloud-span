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

#include <pcl_cloud_span/pcl_cloud_span.h>

#include <pcl/filters/extract_indices.h>

#include <gmock/gmock.h>

using pcl_cloud_span::Spannable;
using pcl_cloud_span::SpanType;

using SpannablePoint = Spannable<pcl::PointXYZ, SpanType::ReadOnly>;

POINT_CLOUD_REGISTER_POINT_STRUCT(SpannablePoint, (float, x, x)(float, y,  y)(float, z, z))

namespace pcl {
bool
operator==(const PointXYZ& a, const PointXYZ& b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
} // namespace pcl

TEST(FilterTest, ExtractIndicesTest)
{
  const auto in_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  in_cloud->push_back({0, 0, 0});
  in_cloud->push_back({1, 0, 0});
  in_cloud->push_back({2, 0, 0});
  in_cloud->push_back({3, 0, 0});

  const auto in_cloud_span = std::make_shared<pcl::PointCloud<SpannablePoint>>(
      reinterpret_cast<const SpannablePoint*>(in_cloud->points.data()), in_cloud->size());

  const auto in_indices = std::make_shared<pcl::Indices>(pcl::Indices{1, 2});

  pcl::ExtractIndices<SpannablePoint> extract_span;
  extract_span.setInputCloud(in_cloud_span);
  extract_span.setIndices(in_indices);
  pcl::PointCloud<SpannablePoint> out;
  extract_span.filter(out);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(in_indices);
  pcl::PointCloud<pcl::PointXYZ> expected;
  extract.filter(expected);

  EXPECT_THAT(pcl_cloud_span::convertToPCL(out).points, ::testing::ContainerEq(expected.points));
  const auto& in_span = *in_cloud_span;
  EXPECT_EQ(&in_cloud->points[0], &in_span.points[0]);
}
