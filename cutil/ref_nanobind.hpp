#pragma once

#include "ref.hpp"
#include <nanobind/nanobind.h>

namespace nanobind {
namespace detail {

// Type caster for Ref<T>
template <typename T> struct type_caster<cutil::Ref<T>> {
  using RefType = cutil::Ref<T>;

  NB_TYPE_CASTER(RefType, const_name("Ref[") + make_caster<T>::name + const_name("]"));

  bool from_python(handle src, uint8_t flags, cleanup_list*) noexcept {
    // Convert from Python to Ref<T>
    type_caster<T*> caster;
    if(!caster.from_python(src, flags, nullptr)) {
      return false;
    }
    value = cutil::Ref<T>(caster.operator T*());
    return true;
  }

  static handle cast(const RefType& src, return_value_policy policy, handle parent) {
    if(!src) {
      return none().release();
    }
    return type_caster<T*>::cast(src.get(), policy, parent);
  }
};

// Type caster for WeakPtr<T>
template <typename T> struct type_caster<cutil::WeakPtr<T>> {
  using WeakPtrType = cutil::WeakPtr<T>;

  NB_TYPE_CASTER(WeakPtrType, const_name("WeakPtr[") + make_caster<T>::name + const_name("]"));

  bool from_python(handle src, uint8_t flags, cleanup_list*) noexcept {
    // Convert from Python to WeakPtr<T>
    type_caster<T*> caster;
    if(!caster.from_python(src, flags, nullptr)) {
      return false;
    }
    // Create a temporary Ref and convert to WeakPtr
    cutil::Ref<T> temp(caster.operator T*());
    value = cutil::WeakPtr<T>(temp);
    return true;
  }

  static handle cast(const WeakPtrType& src, return_value_policy policy, handle parent) {
    if(src.expired()) {
      return none().release();
    }
    auto locked = src.lock();
    if(!locked) {
      return none().release();
    }
    return type_caster<T*>::cast(locked.get(), policy, parent);
  }
};

} // namespace detail

// Enable Ref<T> as holder type for nanobind classes
template <typename T> struct is_holder<cutil::Ref<T>> : std::true_type {};

template <typename T> struct holder_type<T, std::enable_if_t<std::is_same_v<cutil::Ref<T>, cutil::Ref<T>>>> {
  static_assert(std::is_base_of_v<cutil::enable_ref_from_this<T>, T>, "Holder type Ref<T> requires T to inherit from enable_ref_from_this<T>");

  using type = cutil::Ref<T>;

  static constexpr auto init = [](cutil::Ref<T>& h, T* ptr) { h = cutil::make_ref<T>(*ptr); };

  static T* get(cutil::Ref<T>& h) noexcept { return h.get(); }
};

} // namespace nanobind
