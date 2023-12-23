#pragma once

#include <algorithm>
#include <queue>
#include <vector>

namespace msquare {

namespace hybrid_a_star_2 {

// CostFunc: class with operator(), take KeyType as input and generate
// some value as priority output. lower value of priority will be pop first
template <typename KeyType, typename CostFunc> class BucketPriorityQueue {
public:
  using CostType = typename CostFunc::ValueType;

private:
  using ElementType = std::pair<CostType, KeyType>;

  struct Compare {
    bool operator()(const ElementType &lhs, const ElementType &rhs) {
      return lhs.first > rhs.first;
    }
  };

  class Queue : public std::priority_queue<ElementType,
                                           std::vector<ElementType>, Compare> {
  public:
    std::vector<ElementType> &container() { return this->c; }
  };

public:
  BucketPriorityQueue(const CostFunc &cost_func, CostType bucket_size,
                      std::size_t buckets_num, int reserved_bucket_num = 0,
                      int reserved_per_bucket = 0)
      : cost_func_(cost_func), bucket_size_inv_(CostType(1) / bucket_size),
        reserved_per_bucket_(reserved_per_bucket) {
    bucket_ids_.resize(buckets_num);
    bucket_pool_.resize(reserved_bucket_num);
    for (auto &bucket : bucket_pool_) {
      bucket.container().reserve(reserved_per_bucket);
    }
    clear();
  }

  void clear() {
    for (auto &bucket : bucket_pool_) {
      bucket.container().clear();
    }
    std::fill(bucket_ids_.begin(), bucket_ids_.end(), -1);
    available_bucket_ids_.clear();
    available_bucket_ids_.reserve(bucket_pool_.size());
    int bucket_pool_size = bucket_pool_.size();
    for (int i = 0; i < bucket_pool_size; i++) {
      available_bucket_ids_.push_back(bucket_pool_size - 1 - i);
    }
    start_ = 0;
    end_ = 0;
    size_ = 0;
  }

  void push(const KeyType &data) {
    std::size_t bin = getBin(data);
    if (bucket_ids_[bin] == -1) {
      bucket_ids_[bin] = allocateBucket();
    }
    auto &bucket = bucket_pool_[bucket_ids_[bin]];
    bucket.push(std::make_pair(cost_func_(data), data));
    start_ = std::min(start_, bin);
    end_ = std::max(end_, bin + 1);
    size_++;
  }

  KeyType pop() {
    findStart();
    auto &bucket = bucket_pool_[bucket_ids_[start_]];
    KeyType res = bucket.top().second;
    bucket.pop();
    if (bucket.empty()) {
      available_bucket_ids_.push_back(bucket_ids_[start_]);
      bucket_ids_[start_] = -1;
    }
    size_--;
    return res;
  }

  bool empty() { return size_ == 0; }

  bool erase(const KeyType &data) {
    std::size_t bin = getBin(data);
    if (bucket_ids_[bin] == -1) {
      return false;
    }
    auto &heap = bucket_pool_[bucket_ids_[bin]].container();
    int index = -1;
    for (std::size_t i = 0; i < heap.size(); i++) {
      if (heap[i].second == data) {
        index = i;
        break;
      }
    }
    if (index == -1) {
      return false;
    }
    erase(bin, index);
    return true;
  }

  std::size_t size() { return size_; }

  KeyType pop_back() {
    findEnd();
    auto &heap = bucket_pool_[bucket_ids_[end_ - 1]].container();
    CostType max_cost = heap[0].first;
    std::size_t index = 0;
    for (std::size_t i = 1; i < heap.size(); i++) {
      if (heap[i].first > max_cost) {
        max_cost = heap[i].first;
        index = i;
      }
    }
    KeyType key = heap[index].second;
    erase(end_ - 1, index);
    return key;
  }

private:
  bool invalidBucket(std::size_t id) {
    return bucket_ids_[id] == -1 || bucket_pool_[bucket_ids_[id]].empty();
  }

  void findStart() {
    while (start_ != end_ && invalidBucket(start_)) {
      start_++;
    }
  }

  void findEnd() {
    while (start_ != end_ && invalidBucket(end_ - 1)) {
      end_--;
    }
  }

  void erase(std::vector<ElementType> &heap, int index) {
    if (index + 1 == heap.size()) {
      heap.pop_back();
    } else {
      std::swap(heap[index], heap.back());
      heap.pop_back();
      std::make_heap(heap.begin(), heap.end(), compare_);
    }
    size_--;
  }

  void erase(int bin, int index) {
    auto &heap = bucket_pool_[bucket_ids_[bin]].container();
    erase(heap, index);
    if (heap.empty()) {
      available_bucket_ids_.push_back(bucket_ids_[bin]);
      bucket_ids_[bin] = -1;
    }
  }

  std::size_t getBin(const KeyType &data) {
    std::size_t bin =
        std::floor(std::max(cost_func_(data) * bucket_size_inv_, CostType(0)));
    return std::min(bin, bucket_ids_.size() - 1);
  }

  int allocateBucket() {
    if (available_bucket_ids_.empty()) {
      bucket_pool_.emplace_back();
      bucket_pool_.back().container().reserve(reserved_per_bucket_);
      return int(bucket_pool_.size()) - 1;
    } else {
      int res = available_bucket_ids_.back();
      available_bucket_ids_.pop_back();
      return res;
    }
  }

  std::vector<Queue> bucket_pool_;
  std::vector<int> bucket_ids_;
  std::vector<int> available_bucket_ids_;
  std::size_t start_;
  std::size_t end_;
  CostType bucket_size_inv_;
  const CostFunc &cost_func_;
  Compare compare_;
  std::size_t size_;
  std::size_t reserved_per_bucket_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
