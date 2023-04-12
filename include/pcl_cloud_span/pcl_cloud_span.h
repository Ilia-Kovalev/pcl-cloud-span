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

#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl_cloud_span/point_wrapper.h>

#include <pcl_cloud_span/impl/point_cloud.h>

namespace pcl_cloud_span {
template <typename PointT, SpanType span_type>
pcl::PointCloud<PointT>
convertToPCL(const pcl::PointCloud<Spannable<PointT, span_type>>& in)
{
  pcl::PointCloud<PointT> out;
  out.header = in.header;
  out.points = {in.points.begin(), in.points.end()};
  out.width = in.width;
  out.height = in.height;
  out.is_dense = in.is_dense;
  out.sensor_origin_ = in.sensor_origin_;
  out.sensor_orientation_ = in.sensor_orientation_;
  return out;
}

template <typename PointT>
pcl::PointCloud<PointT>
convertToPCL(pcl::PointCloud<Spannable<PointT, SpanType::ReadOnly>>&& in)
{
  pcl::PointCloud<PointT> out;
  out.header = in.header;

  out.points = static_cast<typename pcl::PointCloud<PointT>::VectorType>(
      *reinterpret_cast<impl::utils::span_or_vector<
          PointT,
          impl::utils::span_type::read_only,
          typename pcl::PointCloud<PointT>::VectorType::allocator_type>*>(&in.points));

  out.width = in.width;
  out.height = in.height;
  out.is_dense = in.is_dense;
  out.sensor_origin_ = in.sensor_origin_;
  out.sensor_orientation_ = in.sensor_orientation_;
  return out;
}

template <typename PointT>
pcl::PointCloud<Spannable<PointT, SpanType::ReadOnly>>
makeCloudSpan(const PointT* data, std::uint32_t width, std::uint32_t height = 1)
{
  return {reinterpret_cast<const Spannable<PointT, SpanType::ReadOnly>*>(data),
          width,
          height};
}

template <typename PointT>
typename pcl::PointCloud<Spannable<PointT, SpanType::ReadOnly>>::Ptr
makeCloudSpanPtr(const PointT* data, std::uint32_t width, std::uint32_t height = 1)
{
  return std::make_shared<pcl::PointCloud<Spannable<PointT, SpanType::ReadOnly>>>(
      makeCloudSpan(data, width, height));
}

} // namespace pcl_cloud_span
