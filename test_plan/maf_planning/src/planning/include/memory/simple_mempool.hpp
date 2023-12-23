#pragma once

#include "planning/common/logging.h"
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace msquare {
namespace memory {

class MemoryPool {
public:
  virtual ~MemoryPool() = default;
  static const uint32_t MAX_ITEM_SIZE = 32;

  virtual void *Alloc(const void *config) = 0;

  virtual void Free(void *raw_pitem) = 0;
};

using MemoryPoolPtr = std::shared_ptr<MemoryPool>;

template <typename M, typename E> class SimpleMemoryPool : public MemoryPool {
private:
public:
  explicit SimpleMemoryPool(uint32_t _size = 6) : is_inited_(false) {
    if (_size > MAX_ITEM_SIZE) {
      MSD_LOG(ERROR, "Setting-size[%u] over max size, resize to %u\n", _size,
              MAX_ITEM_SIZE);
      item_size_ = MAX_ITEM_SIZE;
    } else {
      item_size_ = _size;
    }

    allocated_item_cnt_.store(0);

    (void)Init();
  }

  ~SimpleMemoryPool() { Fini(); }

  int Init() {
    if (is_inited_) {
      return 0;
    }

    std::lock_guard<std::mutex> lg(config_lock_);

    // double check
    if (is_inited_) {
      return 0;
    }

    // 1. pre-create items
    for (uint32_t i = 0; i < item_size_; i++) {
      // input free E
      auto item_ptr = new M(E{});
      if (!item_ptr) {
        return -1;
      }

      viraddr2index_mapping_.insert(std::make_pair(item_ptr, i));

      item_pool_[i] = item_ptr;
    }

    item_mask_.store(~((0x01 << item_size_) - 1));
    is_inited_ = true;

    return 0;
  }

  void Fini() {
    if (!is_inited_) {
      return;
    }

    std::lock_guard<std::mutex> lg(config_lock_);

    // double check
    if (!is_inited_) {
      return;
    }

    for (uint32_t i = 0; i < item_size_; i++) {
      delete (item_pool_[i]);
    }
  }

  void *Alloc(const void *config) {
    E *pconfig = (E *)config;

    return (void *)Alloc(*pconfig);
  }

  void Free(void *raw_pitem) {
    M *pitem = static_cast<M *>(raw_pitem);
    Free(pitem);
  }

  M *Alloc(const E &config) {
    uint32_t cur_mask;
    uint32_t target_mask;
    uint32_t new_index;

    do {
      cur_mask = item_mask_.load();
      if (cur_mask == (uint32_t)-1) {
        break;
      }

      new_index = __builtin_ffs(~cur_mask) - 1;
      target_mask = cur_mask | (0x01UL << new_index);
    } while (!item_mask_.compare_exchange_weak(cur_mask, target_mask,
                                               std::memory_order_acq_rel));

    if (cur_mask == (uint32_t)-1) {
      MSD_LOG(ERROR, "Memory pool is empty, alloc failed!\n");
      return nullptr;
    }

    M *pres = item_pool_[new_index];
    pres->reset(config);
    (void)allocated_item_cnt_++;

    // found new item, index is new_index
    return pres;
  }

  void Free(M *pitem) {
    pitem->unset();

    uint32_t free_index = viraddr2index_mapping_[pitem];

    uint32_t cur_mask;
    uint32_t target_mask;

    do {
      cur_mask = item_mask_.load();
      target_mask = cur_mask & (~(0x01UL << free_index));
    } while (!item_mask_.compare_exchange_weak(cur_mask, target_mask,
                                               std::memory_order_acq_rel));

    (void)allocated_item_cnt_--;
  }

  void ShowCurrentPool() {
    uint32_t cur_mask = item_mask_.load();

    MSD_LOG(INFO, "Current use mask 0x%x, allocated item [%u/%u]\n", cur_mask,
            allocated_item_cnt_.load(), item_size_);
  }

private:
private:
  // 0: free; 1: used
  std::atomic<uint32_t> item_mask_;
  std::atomic<uint32_t> allocated_item_cnt_;
  uint32_t item_size_;
  M *item_pool_[MAX_ITEM_SIZE];
  std::mutex config_lock_;
  bool is_inited_;
  std::map<M *, uint32_t> viraddr2index_mapping_;
};

} // namespace memory
} // namespace msquare
