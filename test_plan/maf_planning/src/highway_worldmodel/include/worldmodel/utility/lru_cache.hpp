#pragma once

#include <unordered_map>
/***
namespace std {
// Add std::pair hashing function
template <typename T1, typename T2> class hash<std::pair<T1, T2>> {
public:
  std::size_t operator()(const std::pair<T1, T2> &x) const {
    return std::hash<T1>()(x.first) ^ std::hash<T2>()(x.second);
  }
};
}
***/
// namespace std

namespace msd_worldmodel {
namespace worldmodel_v1 {

template <typename key_t, typename value_t> class LRUCache {
public:
  typedef typename std::pair<key_t, value_t> key_value_pair_t;
  typedef typename std::list<key_value_pair_t>::iterator list_iterator_t;

  explicit LRUCache(size_t max_size) : max_size_(max_size) {}

  void put(const key_t &key, const value_t &value) {
    auto it = cache_items_map_.find(key);
    cache_items_list_.emplace_front(key, value);
    if (it != cache_items_map_.end()) {
      cache_items_list_.erase(it->second);
      cache_items_map_.erase(it);
    }
    cache_items_map_[key] = cache_items_list_.begin();
    if (cache_items_map_.size() > max_size_) {
      cache_items_map_.erase((--cache_items_list_.end())->first);
      cache_items_list_.pop_back();
    }
  }

  const value_t &get(const key_t &key) {
    auto it = cache_items_map_.find(key);
    if (it == cache_items_map_.end()) {
      // throw std::range_error("LRUCache: Invalid key");
    } else {
      cache_items_list_.splice(cache_items_list_.begin(), cache_items_list_,
                               it->second);
      return it->second->second;
    }
  }

  bool exists(const key_t &key) {
    return cache_items_map_.find(key) != cache_items_map_.end();
  }

  const std::list<key_value_pair_t> &getItemList() const {
    return cache_items_list_;
  }

  void clear() {
    cache_items_list_.clear();
    cache_items_map_.clear();
  }

private:
  const size_t max_size_;
  std::list<key_value_pair_t> cache_items_list_;
  std::unordered_map<key_t, list_iterator_t> cache_items_map_;
};

} // namespace worldmodel_v1
} // namespace msd_worldmodel
