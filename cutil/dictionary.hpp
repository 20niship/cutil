#pragma once

#include "hash_map.hpp"
#include <initializer_list>
#include <string>
#include <vector>

namespace cutil {

// dictionary - High-level key-value container with string keys
template <typename Value> class dictionary {
public:
  using key_type        = std::string;
  using mapped_type     = Value;
  using value_type      = std::pair<const std::string, Value>;
  using size_type       = std::size_t;
  using reference       = value_type&;
  using const_reference = const value_type&;

private:
  using HashMap = hash_map<std::string, Value>;

public:
  // Iterators
  class iterator {
    friend class dictionary;

  public:
    using difference_type   = std::ptrdiff_t;
    using value_type        = dictionary::value_type;
    using pointer           = value_type*;
    using reference         = value_type&;
    using iterator_category = std::forward_iterator_tag;

    iterator() : it_() {}

    iterator(typename HashMap::iterator it) : it_(it) {}

    iterator& operator++() {
      ++it_;
      return *this;
    }

    iterator operator++(int) {
      iterator tmp = *this;
      ++it_;
      return tmp;
    }

    reference operator*() const {
      auto& pair = *it_;
      return const_cast<reference>(reinterpret_cast<const_reference>(*it_));
    }

    pointer operator->() const {
      auto& pair = *it_;
      return const_cast<pointer>(reinterpret_cast<const value_type*>(&(*it_)));
    }

    bool operator==(const iterator& other) const { return it_ == other.it_; }

    bool operator!=(const iterator& other) const { return it_ != other.it_; }

  private:
    typename HashMap::iterator it_;
  };

  class const_iterator {
    friend class dictionary;

  public:
    using difference_type   = std::ptrdiff_t;
    using value_type        = dictionary::value_type;
    using pointer           = const value_type*;
    using reference         = const value_type&;
    using iterator_category = std::forward_iterator_tag;

    const_iterator() : it_() {}

    const_iterator(typename HashMap::const_iterator it) : it_(it) {}

    const_iterator(const iterator& it) : it_(it.it_) {}

    const_iterator& operator++() {
      ++it_;
      return *this;
    }

    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++it_;
      return tmp;
    }

    reference operator*() const { return reinterpret_cast<const_reference>(*it_); }

    pointer operator->() const { return reinterpret_cast<const pointer>(&(*it_)); }

    bool operator==(const const_iterator& other) const { return it_ == other.it_; }

    bool operator!=(const const_iterator& other) const { return it_ != other.it_; }

  private:
    typename HashMap::const_iterator it_;
  };

  // Constructors
  dictionary() = default;

  dictionary(const dictionary& other) : map_(other.map_) {}

  dictionary(dictionary&& other) noexcept : map_(std::move(other.map_)) {}

  dictionary(std::initializer_list<std::pair<std::string, Value>> init) {
    for(const auto& [key, value] : init) {
      insert(key, value);
    }
  }

  ~dictionary() = default;

  // Assignment operators
  dictionary& operator=(const dictionary& other) {
    if(this != &other) {
      map_ = other.map_;
    }
    return *this;
  }

  dictionary& operator=(dictionary&& other) noexcept {
    if(this != &other) {
      map_ = std::move(other.map_);
    }
    return *this;
  }

  // Insert
  std::pair<iterator, bool> insert(const std::string& key, const Value& value) {
    auto [it, inserted] = map_.insert(key, value);
    return {iterator(it), inserted};
  }

  std::pair<iterator, bool> insert(std::string&& key, Value&& value) {
    auto [it, inserted] = map_.insert(std::move(key), std::move(value));
    return {iterator(it), inserted};
  }

  // Erase
  bool erase(const std::string& key) { return map_.erase(key); }

  size_t erase_if(std::function<bool(const std::string&, const Value&)> predicate) {
    size_t count = 0;
    auto it      = begin();
    while(it != end()) {
      if(predicate(it->first, it->second)) {
        it = erase(it);
        ++count;
      } else {
        ++it;
      }
    }
    return count;
  }

  iterator erase(iterator it) {
    auto key = it->first;
    ++it;
    map_.erase(key);
    return it;
  }

  // Find
  iterator find(const std::string& key) { return iterator(map_.find(key)); }

  const_iterator find(const std::string& key) const { return const_iterator(map_.find(key)); }

  // At (with bounds checking)
  Value& at(const std::string& key) { return map_.at(key); }

  const Value& at(const std::string& key) const { return map_.at(key); }

  // Operator[]
  Value& operator[](const std::string& key) { return map_[key]; }

  // Clear
  void clear() { map_.clear(); }

  // Size and capacity
  size_t size() const noexcept { return map_.size(); }

  bool empty() const noexcept { return map_.empty(); }

  // Contains
  bool contains(const std::string& key) const { return map_.contains(key); }

  // Count
  size_t count(const std::string& key) const { return map_.contains(key) ? 1 : 0; }

  // Iterators
  iterator begin() { return iterator(map_.begin()); }

  iterator end() { return iterator(map_.end()); }

  const_iterator begin() const { return const_iterator(map_.begin()); }

  const_iterator end() const { return const_iterator(map_.end()); }

  const_iterator cbegin() const { return const_iterator(map_.cbegin()); }

  const_iterator cend() const { return const_iterator(map_.cend()); }

  // Keys
  std::vector<std::string> keys() const {
    std::vector<std::string> result;
    for(const auto& item : *this) {
      result.push_back(item.first);
    }
    return result;
  }

  // Values
  std::vector<Value> values() const {
    std::vector<Value> result;
    for(const auto& item : *this) {
      result.push_back(item.second);
    }
    return result;
  }

  // Merge
  void merge(const dictionary& other) {
    for(const auto& [key, value] : other) {
      insert(key, value);
    }
  }

  // Get with default value
  Value get(const std::string& key, const Value& default_value) const {
    auto it = find(key);
    if(it != end()) {
      return it->second;
    }
    return default_value;
  }

  // Update existing or insert
  Value& put(const std::string& key, const Value& value) {
    auto [it, _] = insert(key, value);
    return map_[key];
  }

private:
  HashMap map_;
};

} // namespace cutil
