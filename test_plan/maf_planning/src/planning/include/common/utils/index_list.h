#ifndef MODULES_PLANNING_OPTIMIZERS_INDEX_LIST_H_
#define MODULES_PLANNING_OPTIMIZERS_INDEX_LIST_H_

#include <unordered_map>
#include <vector>

// #include "boost/thread/shared_mutex.hpp"
// #include "google/protobuf/stubs/map_util.h"
#include "map_util.h"
#include <iostream>

namespace msquare {

// Expose some useful utils from protobuf.
// Find*()
using google::protobuf::FindCopy;
using google::protobuf::FindLinkedPtrOrDie;
using google::protobuf::FindLinkedPtrOrNull;
using google::protobuf::FindOrDie;
using google::protobuf::FindOrDieNoPrint;
using google::protobuf::FindOrNull;
using google::protobuf::FindPtrOrNull;
using google::protobuf::FindWithDefault;

// Contains*()
using google::protobuf::ContainsKey;
using google::protobuf::ContainsKeyValuePair;

// Insert*()
using google::protobuf::InsertAndDeleteExisting;
using google::protobuf::InsertIfNotPresent;
using google::protobuf::InsertKeyOrDie;
using google::protobuf::InsertOrDie;
using google::protobuf::InsertOrDieNoPrint;
using google::protobuf::InsertOrUpdate;
using google::protobuf::InsertOrUpdateMany;

// Lookup*()
using google::protobuf::AddTokenCounts;
using google::protobuf::LookupOrInsert;
using google::protobuf::LookupOrInsertNew;
using google::protobuf::LookupOrInsertNewLinkedPtr;
using google::protobuf::LookupOrInsertNewSharedPtr;

// Misc Utility Functions
using google::protobuf::AppendKeysFromMap;
using google::protobuf::AppendValuesFromMap;
using google::protobuf::EraseKeyReturnValuePtr;
using google::protobuf::InsertKeysFromMap;
using google::protobuf::InsertOrReturnExisting;
using google::protobuf::UpdateReturnCopy;

static constexpr uint32_t origin_capacity = 100;

template <typename I, typename T> class IndexedList {
public:
  /**
   * @brief copy object into the container. If the id is already exist,
   * overwrite the object in the container.
   * @param id the id of the object
   * @param object the const reference of the objected to be copied to the
   * container.
   * @return The pointer to the object in the container.
   */
  T *Add(const I id, const T &object) {
    object_list_.reserve(origin_capacity);
    auto obs = Find(id);
    if (obs) {
      // std::cout << "object " << id << " is already in container" <<
      // std::endl;
      obs->update(object);
      return obs;
    } else {
      T *ptr = nullptr;
      if (object_list_size_ < object_list_.size()) {
        object_list_[object_list_size_].update(object);
        ptr = &object_list_[object_list_size_];
      } else {
        object_list_.push_back(object);
        ptr = &(object_list_.back());
      }
      object_list_size_ += 1;
      object_list_ptr_.push_back(ptr);
      object_dict_.insert({id, object_list_size_ - 1});

      // update object_list_ptr_ when capacity changed for safety
      if (capacity_ != object_list_.capacity()) {
        object_list_ptr_.clear();
        for (int i = 0; i < object_list_size_; i++) {
          object_list_ptr_.push_back(&object_list_[i]);
        }

        capacity_ = object_list_.capacity();
      }
      return ptr;
    }
  }

  /**
   * @brief Find object by id in the container
   * @param id the id of the object
   * @return the raw pointer to the object if found.
   * @return nullptr if the object is not found.
   */
  T *Find(const I id) {
    auto ptr = FindOrNull(object_dict_, id);
    if (ptr) {
      return &object_list_[*ptr];
    } else {
      return nullptr;
    }
  }

  /**
   * @brief Find object by id in the container
   * @param id the id of the object
   * @return the raw pointer to the object if found.
   * @return nullptr if the object is not found.
   */
  const T *Find(const I id) const {
    auto ptr = FindOrNull(object_dict_, id);
    if (ptr) {
      return &object_list_[*ptr];
    } else {
      return nullptr;
    }
  }

  /**
   * @brief List all the items in the container.
   * @return the list of const raw pointers of the objects in the container.
   */
  const std::vector<const T *> &Items() const { return object_list_ptr_; }

  /**
   * @brief List all the items in the container.
   * @return the unordered_map of ids and objects in the container.
   */
  const std::unordered_map<I, uint32_t> &Dict() const { return object_dict_; }

  /**
   * @brief Copy the container with objects.
   */
  IndexedList &operator=(const IndexedList &other) {
    if (this == &other) {
      return *this;
    }
    this->Clear();
    for (const auto &item : other.Dict()) {
      Add(item.first, *(other.Items()[item.second]));
    }
    return *this;
  }

  void Clear() {
    object_list_.reserve(origin_capacity);
    object_list_size_ = 0;
    object_list_ptr_.reserve(origin_capacity);
    object_list_ptr_.clear();
    object_dict_.clear();
  }

private:
  std::vector<T> object_list_{};
  std::vector<const T *> object_list_ptr_{};
  std::unordered_map<I, uint32_t> object_dict_{};
  uint32_t capacity_ = origin_capacity;
  uint32_t object_list_size_ = 0;
};

// TODO(shike): replace boost with std
// template <typename I, typename T>
// class ThreadSafeIndexedList : public IndexedList<I, T> {
// public:
//  T *Add(const I id, const T &object) {
//    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
//    return IndexedList<I, T>::Add(id, object);
//  }
//
//  T *Find(const I id) {
//    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
//    return IndexedList<I, T>::Find(id);
//  }
//
//  std::vector<const T *> Items() const {
//    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
//    return IndexedList<I, T>::Items();
//  }
//
// private:
//  mutable boost::shared_mutex mutex_;
//};

} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_INDEX_LIST_H_ */
