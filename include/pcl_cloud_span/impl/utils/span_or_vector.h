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

#include <boost/core/span.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pcl_cloud_span {
namespace impl {
namespace utils {

enum class span_type { read_only, read_write };

namespace detail {

template <typename T, typename Allocator = std::allocator<T>>
class span_or_vector_base {
  using vector_type = std::vector<T, Allocator>;
  using span_type = boost::span<T>;

public:
  using value_type = typename span_type::value_type;
  using allocator_type = typename vector_type::allocator_type;
  using size_type = typename span_type::size_type;
  using difference_type = typename span_type::difference_type;
  using reference = typename span_type::reference;
  using const_reference = typename span_type::const_reference;
  using pointer = typename span_type::pointer;
  using const_pointer = typename span_type::const_pointer;
  using iterator = typename span_type::iterator;
  using const_iterator = typename span_type::const_iterator;
  using reverse_iterator = typename span_type::reverse_iterator;
  using const_reverse_iterator = typename span_type::const_reverse_iterator;

  template <typename... Args>
  span_or_vector_base(Args&&... args) : vector_(std::forward<Args>(args)...)
  {
    update_span();
  }

  span_or_vector_base(T* data, size_type count, const Allocator allocator = Allocator())
  : vector_(allocator), span_(data, count), current_span_(data, count), is_span_(true)
  {}

  std::vector<T>
  move_to_vector()
  {
    vector_type result;

    if (is_vector())
      result = std::move(vector_);
    else
      result = {current_span_.begin(), current_span_.end()};

    span_ = {};
    current_span_ = {};

    return result;
  }

  const T*
  data() const noexcept
  {
    return current_span_.data();
  }

  const_iterator
  begin() const noexcept
  {
    return current_span_.begin();
  }

  const_iterator
  end() const noexcept
  {
    return current_span_.cend();
  }

  const_reverse_iterator
  rbegin() const noexcept
  {
    return const_reverse_iterator(begin());
  }

  const_reverse_iterator
  rend() const noexcept
  {
    return const_reverse_iterator(end());
  }

  const_iterator
  cbegin() const noexcept
  {
    return begin();
  }

  const_iterator
  cend() const noexcept
  {
    return end();
  }

  const_reverse_iterator
  crbegin() const noexcept
  {
    return rbegin();
  }

  const_reverse_iterator
  crend() const noexcept
  {
    return rend();
  }

  bool
  empty() const noexcept
  {
    return current_span_.empty();
  }

  size_type
  size() const noexcept
  {
    return current_span_.size();
  }

  size_type
  max_size() const noexcept
  {
    return vector_type{}.max_size();
  }

  size_type
  capacity() const noexcept
  {
    return span_.size();
  }

  const T&
  operator[](const size_type pos) const noexcept
  {
    return current_span_[pos];
  }

  const T&
  at(const size_type pos) const
  {
    if (pos >= size())
      throw std::out_of_range("pos (which is " + std::to_string(pos) +
                              ") >= this->size() (which is " + std::to_string(size()));
    return current_span_[pos];
  }

  const T&
  front() const noexcept
  {
    return current_span_.front();
  }

  const T&
  back() const noexcept
  {
    return current_span_.back();
  }

  allocator_type
  get_allocator() const noexcept
  {
    return vector_.get_allocator();
  }

  void
  shrink_to_fit()
  {
    if (is_vector())
      vector_.shrink_to_fit();
    else
      span_ = current_span_;
  }

  void
  pop_back() noexcept
  {
    if (is_vector()) {
      vector_.pop_back();
      update_span();
    }
    else
      current_span_.first(span_.size() - 1);
  }

  void
  clear() noexcept
  {
    if (is_vector())
      vector_.clear();
    else
      current_span_ = {};
  }

  void
  swap(span_or_vector_base& other) noexcept
  {
    if (this == &other)
      return;

    std::swap(vector_, other.vector_);
    std::swap(span_, other.span_);
    std::swap(current_span_, other.current_span_);
    std::swap(is_span_, other.is_span_);
  }

protected:
  void
  switch_to_vector(size_type new_size = 0, const T& val = {})
  {
    assert(is_span());

    vector_.reserve(std::max({span_.size(), new_size}));
    vector_.resize(std::min({new_size, size()}));
    std::copy_n(current_span_.begin(), std::min({new_size, size()}), vector_.begin());
    vector_.resize(new_size, val);

    is_span_ = false;
    update_span();
  }

  void
  update_span()
  {
    assert(is_vector());
    current_span_ = {vector_.data(), vector_.size()};
  }

  vector_type&
  vector() noexcept
  {
    assert(is_vector());
    return vector_;
  }

  template <
      typename F,
      typename = std::enable_if_t<std::is_void_v<std::result_of_t<F(vector_type&)>>>>
  void
  modify_vector(F&& operation)
  {
    assert(is_vector());

    std::invoke(std::forward<F>(operation), vector_);
    update_span();
  }

  template <
      typename F,
      typename = std::enable_if_t<!std::is_void_v<std::result_of_t<F(vector_type&)>>>>
  std::result_of_t<F(vector_type&)>
  modify_vector(F&& operation)
  {
    assert(is_vector());

    const auto result = std::invoke(std::forward<F>(operation), vector_);
    update_span();
    return result;
  }

  bool
  is_span() const noexcept
  {
    return is_span_;
  }

  bool
  is_vector() const noexcept
  {
    return !is_span_;
  }

  void
  shrink_span(size_type new_size) noexcept
  {
    assert(new_size <= current_span_.size());
    current_span_ = current_span_.first(new_size);
  }

  void
  cut_head_of_span(size_type count) noexcept
  {
    assert(is_span());
    assert(count <= size());
    current_span_ = current_span_.last(size() - count);
  }

  void
  cut_tail_of_span(size_type count) noexcept
  {
    assert(is_span());
    assert(count <= size());
    current_span_ = current_span_.first(size() - count);
  }

private:
  vector_type vector_;
  span_type span_;
  span_type current_span_;
  bool is_span_ = false;
};

} // namespace detail

template <typename T,
          span_type SpanType = span_type::read_only,
          typename Allocator = std::allocator<T>>
class span_or_vector;

template <typename T, typename Allocator>
class span_or_vector<T, span_type::read_only, Allocator>
: public detail::span_or_vector_base<T, Allocator> {

  using base_type = detail::span_or_vector_base<T, Allocator>;

public:
  using base_type::allocator_type;
  using base_type::const_iterator;
  using base_type::const_pointer;
  using base_type::const_reference;
  using base_type::const_reverse_iterator;
  using base_type::difference_type;
  using base_type::iterator;
  using base_type::pointer;
  using base_type::reference;
  using base_type::reverse_iterator;
  using base_type::size_type;
  using base_type::value_type;

  using base_type::operator[];
  using base_type::begin;
  using base_type::cbegin;
  using base_type::cend;
  using base_type::crbegin;
  using base_type::crend;
  using base_type::data;
  using base_type::end;
  using base_type::rbegin;
  using base_type::rend;

  template <typename... Args>
  span_or_vector(Args&&... args) : base_type(std::forward<Args>(args)...)
  {}

  span_or_vector(const T* data,
                 size_type count,
                 const Allocator allocator = Allocator())
  : base_type(const_cast<T*>(data), count, allocator)
  {}

  template <typename... Args>
  auto
  emplace_back(Args&&... args)
  {
    if (this->is_span())
      switch_to_vector(this->size());

    return modify_vector(
        [&](auto const& v) { return v.emplace_back(std::forward<Args>(args)...); });
  }

  template <typename Arg>
  void
  push_back(Arg&& arg)
  {
    if (this->is_span())
      this->switch_to_vector(this->size());

    this->modify_vector([&](auto& v) { v.push_back(std::forward<Arg>(arg)); });
  }

  template <typename... Args>
  iterator
  emplace(const_iterator pos, Args&&... args)
  {
    if (this->is_span())
      this->switch_to_vector(this->size());

    return this->modify_vector(
        [&](auto& v) { return v.emplace(pos, std::forward<Args>(args)...); });
  }

  template <typename InputIt>
  iterator
  insert(const_iterator pos, InputIt first, InputIt last)
  {
    if (this->is_span())
      this->switch_to_vector(this->size());

    return this->modify_vector([&](auto& v) { return v.insert(pos, first, last); });
  }

  iterator
  insert(const_iterator pos, std::initializer_list<T> list)
  {
    if (this->is_span())
      this->switch_to_vector(this->size());

    return this->modify_vector([&](auto& v) { return v.insert(pos, list); });
  }

  template <typename... Args>
  void
  assign(Args&&... args)
  {
    if (this->is_span())
      this->switch_to_vector();

    this->modify_vector([&](auto& v) { v.assign(std::forward<Args>(args)...); });
  }

  span_or_vector&
  operator=(std::initializer_list<T> list)
  {
    assign(list);
    return *this;
  }

  void
  resize(size_type count)
  {
    if (this->is_span()) {
      if (count <= this->size()) {
        this->shrink_span(count);
        return;
      }
      this->switch_to_vector(count);
      return;
    }

    this->modify_vector([&](auto& v) { v.resize(count); });
  }

  void
  resize(size_type count, const T& value)
  {
    if (this->is_span()) {
      if (count <= this->size()) {
        this->shrink_span(count);
        return;
      }
      this->switch_to_vector(count, value);
      return;
    }

    this->modify_vector([&](auto& v) { v.resize(count, value); });
  }

  void
  reserve(size_type new_cap)
  {
    if (this->is_vector()) {
      this->modify_vector([&](auto& v) { v.reserve(new_cap); });
      return;
    }

    if (new_cap <= this->capacity())
      return;

    const auto sz = this->size();
    this->switch_to_vector(new_cap);
    resize(sz);
  }

  iterator
  erase(const_iterator first, const_iterator last)
  {
    if (this->is_span()) {
      if (first == begin()) {
        this->cut_head_of_span(std::distance(first, last));
        return begin();
      }
      else if (last == end()) {
        this->cut_tail_of_span(std::distance(first, last));
        return end();
      }
      this->switch_to_vector(this->size());
    }

    return this->modify_vector([&](auto& v) { return v.erase(first, last); });
  }

  iterator
  erase(const_iterator pos)
  {
    return erase(pos, pos + 1);
  }

  T*
  data()
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<T*>(base_type::data());
  }

  iterator
  begin()
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<iterator>(base_type::begin());
  }

  iterator
  end()
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<iterator>(base_type::end());
  }

  reverse_iterator
  rbegin()
  {
    return reverse_iterator(begin());
  }

  reverse_iterator
  rend()
  {
    return reverse_iterator(end());
  }

  T&
  operator[](const size_type pos)
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<T&>(base_type::operator[](pos));
  }

  T&
  at(const size_type pos)
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<T&>(base_type::at(pos));
  }

  T&
  front() noexcept
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<T&>(base_type::front());
  }

  T&
  back()
  {
    if (this->is_span())
      this->switch_to_vector(this->size());
    return const_cast<T&>(base_type::back());
  }

private:
};

} // namespace utils
} // namespace impl
} // namespace pcl_cloud_span
