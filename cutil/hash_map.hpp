#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cutil {

// Forward declaration
template <typename Key, typename Value, typename Hash = std::hash<Key>, typename KeyEqual = std::equal_to<Key>> class hash_map;

namespace detail {

// Hash node
template <typename Key, typename Value> struct hash_node {
  Key key;
  Value value;
  size_t hash_code;
  bool occupied = false;

  hash_node() = default;
  hash_node(const Key& k, const Value& v, size_t h) : key(k), value(v), hash_code(h), occupied(true) {}
};

} // namespace detail

// hash_map - Hash table implementation
template <typename Key, typename Value, typename Hash, typename KeyEqual> class hash_map {
public:
  using key_type        = Key;
  using mapped_type     = Value;
  using value_type      = std::pair<const Key, Value>;
  using size_type       = std::size_t;
  using difference_type = std::ptrdiff_t;
  using hasher          = Hash;
  using key_equal       = KeyEqual;
  using reference       = value_type&;
  using const_reference = const value_type&;
  using pointer         = value_type*;
  using const_pointer   = const value_type*;

private:
  using Node = detail::hash_node<Key, Value>;

  static constexpr float LOAD_FACTOR_MAX   = 0.75f;
  static constexpr size_t INITIAL_CAPACITY = 16;
  static constexpr size_t MIN_CAPACITY     = 8;

public:
  // Iterator
  class iterator {
    friend class hash_map;

  public:
    using difference_type   = std::ptrdiff_t;
    using value_type        = hash_map::value_type;
    using pointer           = hash_map::pointer;
    using reference         = hash_map::reference;
    using iterator_category = std::forward_iterator_tag;

    iterator() : map_(nullptr), index_(0) {}

    iterator& operator++() {
      if(map_) {
        ++index_;
        while(index_ < map_->buckets_.size() && !map_->buckets_[index_].occupied) {
          ++index_;
        }
      }
      return *this;
    }

    iterator operator++(int) {
      iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    reference operator*() const {
      return reinterpret_cast<reference>(map_->buckets_[index_]);
    }

    pointer operator->() const {
      return reinterpret_cast<pointer>(std::addressof(map_->buckets_[index_]));
    }

    bool operator==(const iterator& other) const { return map_ == other.map_ && index_ == other.index_; }

    bool operator!=(const iterator& other) const { return !(*this == other); }

  private:
    iterator(hash_map* map, size_t index) : map_(map), index_(index) {}

    hash_map* map_;
    size_t index_;
  };

  class const_iterator {
    friend class hash_map;

  public:
    using difference_type   = std::ptrdiff_t;
    using value_type        = hash_map::value_type;
    using pointer           = hash_map::const_pointer;
    using reference         = hash_map::const_reference;
    using iterator_category = std::forward_iterator_tag;

    const_iterator() : map_(nullptr), index_(0) {}

    const_iterator(const iterator& it) : map_(it.map_), index_(it.index_) {}

    const_iterator& operator++() {
      if(map_) {
        ++index_;
        while(index_ < map_->buckets_.size() && !map_->buckets_[index_].occupied) {
          ++index_;
        }
      }
      return *this;
    }

    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    reference operator*() const {
      return reinterpret_cast<reference>(const_cast<Node&>(map_->buckets_[index_]));
    }

    pointer operator->() const {
      return reinterpret_cast<pointer>(const_cast<Node*>(std::addressof(map_->buckets_[index_])));
    }

    bool operator==(const const_iterator& other) const { return map_ == other.map_ && index_ == other.index_; }

    bool operator!=(const const_iterator& other) const { return !(*this == other); }

  private:
    const_iterator(const hash_map* map, size_t index) : map_(map), index_(index) {}

    const hash_map* map_;
    size_t index_;
  };

  // Constructors
  hash_map() : capacity_(INITIAL_CAPACITY), size_(0) { buckets_.resize(capacity_); }

  hash_map(const hash_map& other) : capacity_(other.capacity_), size_(0) {
    buckets_.resize(capacity_);
    for(const auto& node : other.buckets_) {
      if(node.occupied) {
        insert(node.key, node.value);
      }
    }
  }

  hash_map(hash_map&& other) noexcept : buckets_(std::move(other.buckets_)), capacity_(other.capacity_), size_(other.size_) {
    other.capacity_ = INITIAL_CAPACITY;
    other.size_     = 0;
    other.buckets_.resize(INITIAL_CAPACITY);
  }

  ~hash_map() = default;

  hash_map& operator=(const hash_map& other) {
    if(this != &other) {
      clear();
      capacity_ = other.capacity_;
      buckets_.resize(capacity_);
      for(const auto& node : other.buckets_) {
        if(node.occupied) {
          insert(node.key, node.value);
        }
      }
    }
    return *this;
  }

  hash_map& operator=(hash_map&& other) noexcept {
    if(this != &other) {
      buckets_  = std::move(other.buckets_);
      capacity_ = other.capacity_;
      size_     = other.size_;

      other.capacity_ = INITIAL_CAPACITY;
      other.size_     = 0;
      other.buckets_.resize(INITIAL_CAPACITY);
    }
    return *this;
  }

  // Insert
  std::pair<iterator, bool> insert(const Key& key, const Value& value) {
    if(should_rehash()) {
      rehash(capacity_ * 2);
    }

    size_t idx    = find_or_insert_index(key);
    bool inserted = !buckets_[idx].occupied;

    if(inserted) {
      buckets_[idx].key = key;
      buckets_[idx].value = value;
      buckets_[idx].occupied = true;
      buckets_[idx].hash_code = Hash{}(key);
      ++size_;
    } else {
      buckets_[idx].value = value;
    }

    return {iterator(this, idx), inserted};
  }

  // Erase
  bool erase(const Key& key) {
    size_t idx = find_index(key);
    if(idx == std::numeric_limits<size_t>::max()) {
      return false;
    }

    buckets_[idx].key.~Key();
    buckets_[idx].value.~Value();
    buckets_[idx].occupied = false;
    --size_;
    return true;
  }

  // Find
  iterator find(const Key& key) {
    size_t idx = find_index(key);
    if(idx == std::numeric_limits<size_t>::max()) {
      return end();
    }
    return iterator(this, idx);
  }

  const_iterator find(const Key& key) const {
    size_t idx = find_index(key);
    if(idx == std::numeric_limits<size_t>::max()) {
      return end();
    }
    return const_iterator(this, idx);
  }

  // At (with bounds checking)
  Value& at(const Key& key) {
    size_t idx = find_index(key);
    if(idx == std::numeric_limits<size_t>::max()) {
      throw std::out_of_range("key not found");
    }
    return buckets_[idx].value;
  }

  const Value& at(const Key& key) const {
    size_t idx = find_index(key);
    if(idx == std::numeric_limits<size_t>::max()) {
      throw std::out_of_range("key not found");
    }
    return buckets_[idx].value;
  }

  // Operator[]
  Value& operator[](const Key& key) {
    auto it = find(key);
    if(it != end()) {
      return it->second;
    }
    // Insert new entry with default value
    auto [new_it, _] = insert(key, Value());
    return new_it->second;
  }

  // Clear
  void clear() {
    for(auto& node : buckets_) {
      if(node.occupied) {
        node.key.~Key();
        node.value.~Value();
        node.occupied = false;
      }
    }
    size_ = 0;
  }

  // Size and capacity
  size_t size() const noexcept { return size_; }

  bool empty() const noexcept { return size_ == 0; }

  size_t capacity() const noexcept { return capacity_; }

  // Iterators
  iterator begin() {
    for(size_t i = 0; i < buckets_.size(); ++i) {
      if(buckets_[i].occupied) {
        return iterator(this, i);
      }
    }
    return end();
  }

  iterator end() { return iterator(this, buckets_.size()); }

  const_iterator begin() const {
    for(size_t i = 0; i < buckets_.size(); ++i) {
      if(buckets_[i].occupied) {
        return const_iterator(this, i);
      }
    }
    return end();
  }

  const_iterator end() const { return const_iterator(this, buckets_.size()); }

  const_iterator cbegin() const { return begin(); }

  const_iterator cend() const { return end(); }

  // Contains
  bool contains(const Key& key) const { return find_index(key) != std::numeric_limits<size_t>::max(); }

private:
  std::vector<Node> buckets_;
  size_t capacity_;
  size_t size_;

  size_t find_index(const Key& key) const {
    if(buckets_.empty()) return std::numeric_limits<size_t>::max();

    size_t hash_code = Hash{}(key);
    size_t idx       = hash_code % capacity_;

    for(size_t i = 0; i < capacity_; ++i) {
      size_t probe_idx = (idx + i) % capacity_;
      if(!buckets_[probe_idx].occupied) {
        return std::numeric_limits<size_t>::max();
      }
      if(buckets_[probe_idx].hash_code == hash_code && KeyEqual{}(buckets_[probe_idx].key, key)) {
        return probe_idx;
      }
    }
    return std::numeric_limits<size_t>::max();
  }

  size_t find_or_insert_index(const Key& key) {
    size_t hash_code = Hash{}(key);
    size_t idx       = hash_code % capacity_;

    for(size_t i = 0; i < capacity_; ++i) {
      size_t probe_idx = (idx + i) % capacity_;
      if(!buckets_[probe_idx].occupied || KeyEqual{}(buckets_[probe_idx].key, key)) {
        return probe_idx;
      }
    }
    return std::numeric_limits<size_t>::max();
  }

  bool should_rehash() const { return static_cast<float>(size_) / static_cast<float>(capacity_) >= LOAD_FACTOR_MAX; }

  void rehash(size_t new_capacity) {
    auto old_buckets  = buckets_;
    auto old_capacity = capacity_;

    capacity_ = new_capacity;
    size_     = 0;
    buckets_.clear();
    buckets_.resize(capacity_);

    for(const auto& node : old_buckets) {
      if(node.occupied) {
        insert(node.key, node.value);
      }
    }
  }
};

} // namespace cutil
