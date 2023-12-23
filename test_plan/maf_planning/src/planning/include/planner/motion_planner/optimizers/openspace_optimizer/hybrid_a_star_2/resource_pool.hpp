#pragma once

#include <stack>
#include <utility>
#include <vector>

namespace msquare {

namespace hybrid_a_star_2 {

template <typename ResourceType> class ResourcePool {
public:
  struct AllocateResult {
    bool success;
    int key;
  };

  struct BadAlloc {};

  ResourcePool(std::size_t size = 0) { expand(size); }

  void expand(std::size_t size) {
    if (size > storage_.size()) {
      std::size_t old_size = storage_.size();
      storage_.resize(size);
      available_.reserve(size);
      for (size_t i = 0; i < size - old_size; i++) {
        available_.push_back(size - i - 1);
      }
    }
  }

  void releaseAll() {
    if (available_.size() == storage_.size()) {
      return;
    }
    std::size_t size = storage_.size();
    available_.reserve(size);
    available_.clear();
    for (size_t i = 0; i < size; i++) {
      available_.push_back(size - i - 1);
    }
  }

  AllocateResult allocate() {
    if (available_.empty()) {
      return {false, 0};
    } else {
      int key = available_.back();
      available_.pop_back();
      return {true, key};
    }
  }

  ResourceType &allocateForce() {
    auto res = allocate();
    if (res.success) {
      return storage_[res.key];
    } else {
      throw BadAlloc{};
    }
  }

  template <typename... Args> ResourceType &allocateAndInit(Args... args) {
    ResourceType &res = allocateForce();
    res.init(args...);
    return res;
  }

  void release(int key) { available_.push_back(key); }
  const ResourceType &operator[](int key) const { return storage_[key]; }
  ResourceType &operator[](int key) { return storage_[key]; }
  std::size_t availableSize() { return available_.size(); }
  std::size_t totalSize() { return storage_.size(); }
  int invalidKey() { return -1; }

private:
  std::vector<ResourceType> storage_;
  std::vector<int> available_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
