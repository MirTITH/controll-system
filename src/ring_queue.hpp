/**
 * @file ring_queue.hpp
 * @author X. Y.
 * @brief 环形队列
 * @version 0.1
 * @date 2023-06-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <vector>

template <typename T>
class RingQueue
{
private:
    std::vector<T> data_;
    size_t size_;
    size_t pos_; // 指向最近一次入队的元素

public:
    RingQueue(){};

    RingQueue(size_t size, T fill_value = 0)
    {
        set_size(size, fill_value);
    }

    void set_size(size_t size, T fill_value = 0)
    {
        data_.resize(size);
        size_ = size;
        pos_  = 0;
        fill(fill_value);
    }

    void fill(T value)
    {
        std::fill(data_.begin(), data_.end(), value);
    }

    void push_front(T value)
    {
        if (pos_ != 0) {
            pos_--;
        } else {
            pos_ = size_ - 1;
        }

        data_.at(pos_) = value;
    }

    T &operator[](size_t index)
    {
        return data_[(pos_ + index) % size_];
    }

    const T &operator[](size_t index) const
    {
        return data_[(pos_ + index) % size_];
    }

    T &at(size_t index)
    {
        return data_.at((pos_ + index) % size_);
    }

    const T &at(size_t index) const
    {
        return data_.at((pos_ + index) % size_);
    }

    size_t size() const
    {
        return size_;
    }
};
