#pragma once

#include <cstring>
#include <functional>
#include <memory>
#include <typeinfo>
#include <utility>

namespace cutil {

// Forward declarations
template <typename... Types> class variant;

namespace detail {

// Helper: Index of a type in a type list
template <typename T, typename... Types> struct type_index;

template <typename T, typename First, typename... Rest> struct type_index<T, First, Rest...> {
  static constexpr size_t value = std::is_same_v<T, First> ? 0 : 1 + type_index<T, Rest...>::value;
};

// Helper: Check if type is in list
template <typename T, typename... Types> constexpr bool has_type = (std::is_same_v<T, Types> || ...);

// Helper: Max size among types
template <typename... Types> struct max_size;

template <> struct max_size<> {
  static constexpr size_t value = 0;
};

template <typename First, typename... Rest> struct max_size<First, Rest...> {
  static constexpr size_t value = std::max(sizeof(First), max_size<Rest...>::value);
};

// Helper: Max alignment among types
template <typename... Types> struct max_align;

template <> struct max_align<> {
  static constexpr size_t value = 1;
};

template <typename First, typename... Rest> struct max_align<First, Rest...> {
  static constexpr size_t value = std::max(alignof(First), max_align<Rest...>::value);
};

} // namespace detail

// variant - Type-safe union for multiple types
template <typename... Types> class variant {
  static constexpr size_t storage_size  = detail::max_size<Types...>::value;
  static constexpr size_t storage_align = detail::max_align<Types...>::value;

  using Destructor = void (*)(void*);
  using CopyConstructor = void (*)(const void*, void*);
  using MoveConstructor = void (*)(void*, void*);

public:
  // Default constructor - constructs first type
  variant() : index_(0) {
    new(storage_) std::tuple_element_t<0, std::tuple<Types...>>();
  }

  // Constructor from value
  template <typename T, typename = std::enable_if_t<detail::has_type<T, Types...>>> explicit variant(const T& value) : index_(detail::type_index<T, Types...>::value) {
    new(storage_) T(value);
  }

  // Constructor from rvalue
  template <typename T, typename = std::enable_if_t<detail::has_type<T, Types...>>> explicit variant(T&& value) : index_(detail::type_index<T, Types...>::value) {
    new(storage_) T(std::move(value));
  }

  // Copy constructor
  variant(const variant& other) : index_(other.index_) { other.copy_to(storage_, index_); }

  // Move constructor
  variant(variant&& other) noexcept : index_(other.index_) { other.move_to(storage_, index_); }

  // Destructor
  ~variant() { destroy(index_); }

  // Copy assignment
  variant& operator=(const variant& other) {
    if(this != &other) {
      destroy(index_);
      index_ = other.index_;
      other.copy_to(storage_, index_);
    }
    return *this;
  }

  // Move assignment
  variant& operator=(variant&& other) noexcept {
    if(this != &other) {
      destroy(index_);
      index_ = other.index_;
      other.move_to(storage_, index_);
    }
    return *this;
  }

  // Assignment from value
  template <typename T, typename = std::enable_if_t<detail::has_type<T, Types...>>> variant& operator=(const T& value) {
    destroy(index_);
    index_ = detail::type_index<T, Types...>::value;
    new(storage_) T(value);
    return *this;
  }

  // Assignment from rvalue
  template <typename T, typename = std::enable_if_t<detail::has_type<T, Types...>>> variant& operator=(T&& value) {
    destroy(index_);
    index_ = detail::type_index<T, Types...>::value;
    new(storage_) T(std::move(value));
    return *this;
  }

  // Get index
  size_t index() const noexcept { return index_; }

  // Get value by type
  template <typename T> T& get() {
    if(index() != detail::type_index<T, Types...>::value) {
      throw std::bad_typeid();
    }
    return *reinterpret_cast<T*>(storage_);
  }

  template <typename T> const T& get() const {
    if(index() != detail::type_index<T, Types...>::value) {
      throw std::bad_typeid();
    }
    return *reinterpret_cast<const T*>(storage_);
  }

  // Get value by index
  template <size_t I> std::tuple_element_t<I, std::tuple<Types...>>& get() {
    if(index() != I) {
      throw std::bad_typeid();
    }
    return *reinterpret_cast<std::tuple_element_t<I, std::tuple<Types...>>*>(storage_);
  }

  template <size_t I> const std::tuple_element_t<I, std::tuple<Types...>>& get() const {
    if(index() != I) {
      throw std::bad_typeid();
    }
    return *reinterpret_cast<const std::tuple_element_t<I, std::tuple<Types...>>*>(storage_);
  }

  // Try get
  template <typename T> T* try_get() {
    if(index() == detail::type_index<T, Types...>::value) {
      return reinterpret_cast<T*>(storage_);
    }
    return nullptr;
  }

  template <typename T> const T* try_get() const {
    if(index() == detail::type_index<T, Types...>::value) {
      return reinterpret_cast<const T*>(storage_);
    }
    return nullptr;
  }

  // Equality comparison
  bool operator==(const variant& other) const {
    if(index() != other.index()) return false;
    return equals(other, index_);
  }

  bool operator!=(const variant& other) const { return !(*this == other); }

private:
  alignas(storage_align) unsigned char storage_[storage_size];
  size_t index_;

  void destroy(size_t idx) {
    switch(idx) {
      case 0:
        if constexpr(sizeof...(Types) > 0) {
          reinterpret_cast<std::tuple_element_t<0, std::tuple<Types...>>*>(storage_)->~std::tuple_element_t<0, std::tuple<Types...>>();
        }
        break;
      default:
        destroy_impl<1>(idx);
        break;
    }
  }

  template <size_t I> void destroy_impl(size_t idx) {
    if constexpr(I < sizeof...(Types)) {
      if(idx == I) {
        reinterpret_cast<std::tuple_element_t<I, std::tuple<Types...>>*>(storage_)->~std::tuple_element_t<I, std::tuple<Types...>>();
      } else {
        destroy_impl<I + 1>(idx);
      }
    }
  }

  void copy_to(void* dest, size_t idx) const {
    switch(idx) {
      case 0:
        if constexpr(sizeof...(Types) > 0) {
          new(dest) std::tuple_element_t<0, std::tuple<Types...>>(*reinterpret_cast<const std::tuple_element_t<0, std::tuple<Types...>>*>(storage_));
        }
        break;
      default:
        copy_impl<1>(dest, idx);
        break;
    }
  }

  template <size_t I> void copy_impl(void* dest, size_t idx) const {
    if constexpr(I < sizeof...(Types)) {
      if(idx == I) {
        new(dest) std::tuple_element_t<I, std::tuple<Types...>>(*reinterpret_cast<const std::tuple_element_t<I, std::tuple<Types...>>*>(storage_));
      } else {
        copy_impl<I + 1>(dest, idx);
      }
    }
  }

  void move_to(void* dest, size_t idx) noexcept {
    switch(idx) {
      case 0:
        if constexpr(sizeof...(Types) > 0) {
          new(dest) std::tuple_element_t<0, std::tuple<Types...>>(std::move(*reinterpret_cast<std::tuple_element_t<0, std::tuple<Types...>>*>(storage_)));
        }
        break;
      default:
        move_impl<1>(dest, idx);
        break;
    }
  }

  template <size_t I> void move_impl(void* dest, size_t idx) noexcept {
    if constexpr(I < sizeof...(Types)) {
      if(idx == I) {
        new(dest) std::tuple_element_t<I, std::tuple<Types...>>(std::move(*reinterpret_cast<std::tuple_element_t<I, std::tuple<Types...>>*>(storage_)));
      } else {
        move_impl<I + 1>(dest, idx);
      }
    }
  }

  bool equals(const variant& other, size_t idx) const {
    switch(idx) {
      case 0:
        if constexpr(sizeof...(Types) > 0) {
          return *reinterpret_cast<const std::tuple_element_t<0, std::tuple<Types...>>*>(storage_) == *reinterpret_cast<const std::tuple_element_t<0, std::tuple<Types...>>*>(other.storage_);
        }
        return true;
      default:
        return equals_impl<1>(other, idx);
    }
  }

  template <size_t I> bool equals_impl(const variant& other, size_t idx) const {
    if constexpr(I < sizeof...(Types)) {
      if(idx == I) {
        return *reinterpret_cast<const std::tuple_element_t<I, std::tuple<Types...>>*>(storage_) == *reinterpret_cast<const std::tuple_element_t<I, std::tuple<Types...>>*>(other.storage_);
      }
      return equals_impl<I + 1>(other, idx);
    }
    return true;
  }
};

} // namespace cutil
