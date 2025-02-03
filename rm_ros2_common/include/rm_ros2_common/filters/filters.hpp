/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <cstring>

#include <rm_ros2_common/math_tools.hpp>

template <typename T>
class Filter
{
public:
  Filter() = default;
  virtual ~Filter() = default;
  virtual void input(T input_value) = 0;
  virtual T output() = 0;
  virtual void clear() = 0;
};

template <typename T>
class MovingAverageFilter : public Filter<T>
{
public:
  explicit MovingAverageFilter(int num_data): num_data_(num_data), idx_(0), sum_(0.0){
    buffer_ = new T[num_data_];
    memset((void*)buffer_, 0.0, sizeof(T) * num_data_);
  }
  ~MovingAverageFilter(){
    delete[] buffer_;
  }
  void input(T input_value){
    sum_ -= buffer_[idx_];
    sum_ += input_value;
    buffer_[idx_] = input_value;
    ++idx_;
    idx_ %= num_data_;
  }
  T output(){
    return sum_ / num_data_;
  }
  void clear(){
    sum_ = 0.0;
    memset((void*)buffer_, 0.0, sizeof(T) * num_data_);
  }

private:
  T* buffer_;
  int num_data_;
  int idx_;
  T sum_;
};
template class MovingAverageFilter<double>;
template class MovingAverageFilter<float>;

template <typename T>
class Vector3WithFilter
{
public:
  Vector3WithFilter(int num_data)
  {
    for (int i = 0; i < 3; i++)
      filter_vector_.push_back(std::make_shared<MovingAverageFilter<T>>(num_data));
  }
  void input(T vector[3])
  {
    for (int i = 0; i < 3; i++)
      filter_vector_[i]->input(vector[i]);
  }
  void clear()
  {
    for (int i = 0; i < 3; i++)
      filter_vector_[i]->clear();
  }
  T x()
  {
    return filter_vector_[0]->output();
  }
  T y()
  {
    return filter_vector_[1]->output();
  }
  T z()
  {
    return filter_vector_[2]->output();
  }

private:
  std::vector<std::shared_ptr<MovingAverageFilter<T>>> filter_vector_;
};

template <typename T>
class RampFilter : public Filter<T>
{
public:
  RampFilter(T acc, T dt){
    acc_ = acc;
    dt_ = dt;
    RampFilter::clear();
  }
  ~RampFilter() = default;
  void input(T input_value){
    last_value_ += minAbs(input_value - last_value_, acc_ * dt_);
  }
  void clear(){
    last_value_ = 0.;
  }
  void clear(T last_value){
    last_value_ = last_value;
  }
  void setAcc(T acc){
    acc_ = acc;
  }  // without clear.
  T output(){
    return last_value_;
  }

private:
  T last_value_;
  T acc_;
  T dt_;
};
template class RampFilter<float>;
template class RampFilter<double>;
