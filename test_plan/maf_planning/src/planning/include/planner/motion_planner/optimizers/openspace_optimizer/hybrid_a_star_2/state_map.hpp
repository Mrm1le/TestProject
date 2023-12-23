#pragma once

#include <algorithm>
#include <array>
#include <vector>

namespace msquare {

namespace hybrid_a_star_2 {

// CostFunc: class with operator(), take KeyType as input and generate
// some value as priority output. lower value of priority will be accept when
// insert
template <int Dim, typename KeyType, typename CostFunc> class StateMap {
private:
  using CostType = typename CostFunc::ValueType;

public:
  StateMap(const CostFunc &cost_func, KeyType invalid_key,
           const std::array<int, Dim> &bucket_size_bit,
           std::size_t reserved_bucket_num)
      : cost_func_(cost_func), bucket_size_bit_(bucket_size_bit),
        invalid_key_(invalid_key) {
    bucket_element_size_ = 1;
    for (int i = 0; i < Dim; i++) {
      bucket_element_size_ <<= bucket_size_bit[i];
      bucket_size_mask_[i] = (1 << bucket_size_bit[i]) - 1;
    }
    for (std::size_t i = 0; i < reserved_bucket_num; i++) {
      bucket_pool_.emplace_back(bucket_element_size_);
    }
    clear();
  }

  const std::array<int, Dim> &size() { return size_; }
  const std::array<float, Dim> &step() { return step_; }
  const std::array<float, Dim> &origin() { return origin_; }

  void setSize(const std::array<int, Dim> &size) {
    size_ = size;
    for (int i = 0; i < Dim; i++) {
      map_size_[i] = bitDivUp(size_[i], bucket_size_bit_[i]);
    }
    bucket_ids_.resize(numElement(map_size_));
    std::fill(bucket_ids_.begin(), bucket_ids_.end(), -1);
  }

  void setStep(const std::array<float, Dim> &step) {
    step_ = step;
    for (int i = 0; i < Dim; i++) {
      inv_step_[i] = 1.0 / step[i];
    }
  }

  void setOrigin(const std::array<float, Dim> &origin) { origin_ = origin; }

  struct TryInsertResult {
    bool belong = false;
    int bucket_id = -1;
    int offset = -1;
    bool success = false;
    KeyType key;
  };

  template <typename OtherKey>
  TryInsertResult tryInsert(const std::array<float, Dim> &pos,
                            const OtherKey &key) {
    TryInsertResult result;
    int bucket_index = 0;
    int offset = 0;
    for (int i = 0; i < Dim; i++) {
      int quantized_pos = std::round((pos[i] - origin_[i]) * inv_step_[i]);
      if (quantized_pos < 0 || quantized_pos >= size_[i]) {
        return result;
      }
      bucket_index *= map_size_[i];
      bucket_index += quantized_pos >> bucket_size_bit_[i];
      offset <<= bucket_size_bit_[i];
      offset += quantized_pos & bucket_size_mask_[i];
    }

    if (bucket_ids_[bucket_index] == -1) {
      bucket_ids_[bucket_index] = allocateBucket();
    }
    auto &bucket = bucket_pool_[bucket_ids_[bucket_index]];

    result.belong = true;
    result.bucket_id = bucket_ids_[bucket_index];
    result.offset = offset;
    result.success = cost_func_(key) < bucket[offset].first;
    result.key = bucket[offset].second;
    return result;
  }

  KeyType insert(int bucket_id, int offset, const KeyType &key) {
    auto &bucket = bucket_pool_[bucket_id];
    KeyType replaced = bucket[offset].second;
    bucket[offset] = std::make_pair(cost_func_(key), key);
    return replaced;
  }

  void clear() {
    std::fill(bucket_ids_.begin(), bucket_ids_.end(), -1);
    available_bucket_ids_.clear();
    available_bucket_ids_.reserve(bucket_pool_.size());
    int bucket_pool_size = bucket_pool_.size();
    for (int i = 0; i < bucket_pool_size; i++) {
      available_bucket_ids_.push_back(bucket_pool_size - 1 - i);
      std::fill(
          bucket_pool_[i].begin(), bucket_pool_[i].end(),
          std::make_pair(std::numeric_limits<CostType>::max(), invalid_key_));
    }
  }

private:
  int allocateBucket() {
    if (available_bucket_ids_.empty()) {
      bucket_pool_.emplace_back(
          bucket_element_size_,
          std::make_pair(std::numeric_limits<CostType>::max(), invalid_key_));
      return int(bucket_pool_.size()) - 1;
    } else {
      int res = available_bucket_ids_.back();
      available_bucket_ids_.pop_back();
      return res;
    }
  }

  int bitDivUp(int data, int divider_bit) {
    return (data + (1 << divider_bit) - 1) >> divider_bit;
  }

  int numElement(const std::array<int, Dim> &size) {
    int num = 1;
    for (const auto &item : size) {
      num *= item;
    }
    return num;
  }

  std::array<int, Dim> size_;
  std::array<float, Dim> origin_;
  std::array<float, Dim> step_;
  std::array<float, Dim> inv_step_;
  std::array<int, Dim> bucket_size_bit_;
  std::array<int, Dim> bucket_size_mask_;
  std::array<int, Dim> map_size_;
  int bucket_element_size_;
  const CostFunc &cost_func_;
  KeyType invalid_key_;

  std::vector<std::vector<std::pair<CostType, KeyType>>> bucket_pool_;
  std::vector<int> bucket_ids_;
  std::vector<int> available_bucket_ids_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
