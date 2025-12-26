
// This filter comes from
// https://github.com/google-research/tiny-differentiable-simulator/blob/master/examples/whole_body_control/com_velocity_estimator.hpp
// hence we must include Apache License 2.0 too
#pragma once

#include <cassert>
#include <iostream>
#include <utility>
#include <deque>
// A stable O(1) moving filter for incoming data streams. Implements the
// Neumaier's algorithm to calculate the moving window average,
// which is numerically stable.
// Kahan 求和算法:由于浮点数运算损失导致舍入误差，解决a+b=b+a,但a+(b+c) != (a+b)+c,
//双端队列的操作：
// push_back(elem); 在容器尾部添加一个数据
// push_front(elem); 在容器头部插入一个数据
// pop_back(); 删除容器最后一个数据
// pop_front(); 删除容器第一个数据
//双端队列的数据存取
// at(int idx); 返回索引idx所指的数据
// operator[]; 返回索引idx所指的数据
// front(); 返回容器中第一个数据元素
// back(); 返回容器中最后一个数据元素
class MovingWindowFilter {
 public:

  MovingWindowFilter() {}

  MovingWindowFilter(int window_size) : window_size_(window_size) {
    assert(window_size_ > 0);
    sum_ = 0.0;
    correction_ = 0.0;
  }

  // Computes the moving window average.
  double CalculateAverage(double new_value) {
    if (value_deque_.size() < window_size_) 
    {
      // pass 双端队列长度不够直接不管
    } else 
    {
      // The left most value needs to be subtracted from the moving sum first.
      // 数据数量满了，需要移除最旧的数据点
      UpdateNeumaierSum(-value_deque_.front());
      // 删除容器第一个数据
      value_deque_.pop_front();
    }
    // Add the new value.
    UpdateNeumaierSum(new_value);
    //在容器尾部添加一个数据
    value_deque_.push_back(new_value);

    return (sum_ + correction_) / double(window_size_);
  }

  std::deque<double> GetValueQueue() {
    return value_deque_;
  }
 private:
  //window_size 表示移动窗口的大小，即窗口内包含的数据点数量。
  int window_size_;
  //存储当前窗口内所有数据点的总和。
  double sum_;
  //Neumaier 算法中的修正项，用于处理浮点数精度损失
  double correction_;
  //双端队列，用于存储窗口内的所有数据点
  std::deque<double> value_deque_;

  //将sum的第一个数据移除，数据移除
  void UpdateNeumaierSum(double value) {
    //将当前总和 sum_ 与传入的值 value 相加，得到新的总和 new_sum
    double new_sum = sum_ + value;
    //判定哪一个数在加法运算中可能会导致精度损失。
    if (std::abs(sum_) >= std::abs(value)) {
      // If previous sum is bigger, low-order digits of value are lost.
      // (sum_ - new_sum) + value 能够计算出丢失的部分，并将其累加到修正项 correction_ 中。
      // else相同道理
      correction_ += (sum_ - new_sum) + value;
    } else {
      correction_ += (value - new_sum) + sum_;
    }
    sum_ = new_sum;
  }
};