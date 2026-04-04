#pragma once

#include <atomic>
#include <cstring>
#include <memory>
#include <utility>

namespace cutil {

// Forward declarations
template <typename T> class Ref;

template <typename T> class WeakPtr;

template <typename T> class enable_ref_from_this;

namespace detail {
struct PrivateConstructorTag {};
} // namespace detail

// Control block for reference counting
namespace detail {

struct ControlBlockBase {
  std::atomic<long> strong_count{0};
  std::atomic<long> weak_count{0};
  bool managed = false; // true if memory is allocated by Ref

  virtual ~ControlBlockBase()                                  = default;
  virtual void destroy_object()                                = 0;
  virtual void* get_deleter(const std::type_info& ti) noexcept = 0;
};

template <typename T, typename Deleter = std::default_delete<T>> struct ControlBlock : public ControlBlockBase {
  T* ptr = nullptr;
  Deleter deleter;

  ControlBlock() = default;

  ControlBlock(T* p) : ptr(p) {}

  ControlBlock(T* p, const Deleter& d) : ptr(p), deleter(d) {}

  ControlBlock(T* p, Deleter&& d) : ptr(p), deleter(std::move(d)) {}

  void destroy_object() override {
    if(ptr) {
      deleter(ptr);
      ptr = nullptr;
    }
  }

  void* get_deleter(const std::type_info& ti) noexcept override {
    if(ti == typeid(Deleter)) {
      return &deleter;
    }
    return nullptr;
  }

  static ControlBlock* create_empty() { return new ControlBlock(nullptr); }
};

template <typename T> struct ControlBlockInPlace : public ControlBlockBase {
  alignas(T) unsigned char storage[sizeof(T)];

  ControlBlockInPlace() = default;

  template <typename... Args> void construct(Args&&... args) { new(storage) T(std::forward<Args>(args)...); }

  T* get_ptr() { return reinterpret_cast<T*>(storage); }

  void destroy_object() override { reinterpret_cast<T*>(storage)->~T(); }

  void* get_deleter(const std::type_info&) noexcept override { return nullptr; }
};

} // namespace detail

// Ref<T> - Smart pointer with reference counting
template <typename T> class Ref {
  template <typename U> friend class Ref;

  template <typename U> friend class WeakPtr;

  template <typename U> friend class enable_ref_from_this;

public:
  using element_type = T;

  // Default constructor
  constexpr Ref() noexcept : ptr(nullptr), cb(nullptr) {}

  // Null pointer constructor
  constexpr Ref(std::nullptr_t) noexcept : ptr(nullptr), cb(nullptr) {}

  // Constructor from raw pointer
  explicit Ref(T* p) : ptr(p) {
    if(p) {
      cb = new detail::ControlBlock<T>(p);
      cb->strong_count.store(1, std::memory_order_relaxed);
      cb->weak_count.store(1, std::memory_order_relaxed);
      cb->managed = true;
    }
  }

  // Constructor with custom deleter
  template <typename Deleter> Ref(T* p, Deleter d) : ptr(p) {
    if(p) {
      cb = new detail::ControlBlock<T, Deleter>(p, std::move(d));
      cb->strong_count.store(1, std::memory_order_relaxed);
      cb->weak_count.store(1, std::memory_order_relaxed);
      cb->managed = true;
    }
  }

  // Copy constructor
  Ref(const Ref& other) noexcept : ptr(other.ptr), cb(other.cb) {
    if(cb) {
      cb->strong_count.fetch_add(1, std::memory_order_acq_rel);
    }
  }

  // Copy constructor from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> Ref(const Ref<U>& other) noexcept : ptr(other.ptr), cb(other.cb) {
    if(cb) {
      cb->strong_count.fetch_add(1, std::memory_order_acq_rel);
    }
  }

  // Move constructor
  Ref(Ref&& other) noexcept : ptr(other.ptr), cb(other.cb) {
    other.ptr = nullptr;
    other.cb  = nullptr;
  }

  // Move constructor from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> Ref(Ref<U>&& other) noexcept : ptr(other.ptr), cb(other.cb) {
    other.ptr = nullptr;
    other.cb  = nullptr;
  }

  // Constructor from WeakPtr
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> explicit Ref(const WeakPtr<U>& weak) {
    if(weak.cb && weak.cb->strong_count.load(std::memory_order_acquire) > 0) {
      ptr = weak.ptr;
      cb  = weak.cb;
      cb->strong_count.fetch_add(1, std::memory_order_acq_rel);
    } else {
      ptr = nullptr;
      cb  = nullptr;
    }
  }

  // Destructor
  ~Ref() { reset(); }

  // Copy assignment
  Ref& operator=(const Ref& other) noexcept {
    reset();
    ptr = other.ptr;
    cb  = other.cb;
    if(cb) {
      cb->strong_count.fetch_add(1, std::memory_order_acq_rel);
    }
    return *this;
  }

  // Copy assignment from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> Ref& operator=(const Ref<U>& other) noexcept {
    reset();
    ptr = other.ptr;
    cb  = other.cb;
    if(cb) {
      cb->strong_count.fetch_add(1, std::memory_order_acq_rel);
    }
    return *this;
  }

  // Move assignment
  Ref& operator=(Ref&& other) noexcept {
    reset();
    ptr       = other.ptr;
    cb        = other.cb;
    other.ptr = nullptr;
    other.cb  = nullptr;
    return *this;
  }

  // Move assignment from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> Ref& operator=(Ref<U>&& other) noexcept {
    reset();
    ptr       = other.ptr;
    cb        = other.cb;
    other.ptr = nullptr;
    other.cb  = nullptr;
    return *this;
  }

  // Null assignment
  Ref& operator=(std::nullptr_t) noexcept {
    reset();
    return *this;
  }

  // Dereference operator
  T& operator*() const noexcept { return *ptr; }

  // Member access operator
  T* operator->() const noexcept { return ptr; }

  // Subscript operator for array support
  T& operator[](std::ptrdiff_t idx) const noexcept { return ptr[idx]; }

  // Call operator (if T is callable)
  template <typename... Args> auto operator()(Args&&... args) const { return (*ptr)(std::forward<Args>(args)...); }

  // Get raw pointer
  T* get() const noexcept { return ptr; }

  // Explicit bool conversion
  explicit operator bool() const noexcept { return ptr != nullptr; }

  // Get use count
  long use_count() const noexcept { return cb ? cb->strong_count.load(std::memory_order_acquire) : 0; }

  // Check if unique (use_count == 1)
  bool unique() const noexcept { return use_count() == 1; }

  // Reset the pointer
  void reset() noexcept {
    if(cb) {
      if(cb->strong_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        cb->destroy_object();
        if(cb->weak_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {
          delete cb;
        }
      }
    }
    ptr = nullptr;
    cb  = nullptr;
  }

  // Reset with new pointer
  void reset(T* p) noexcept {
    if(p != ptr) {
      reset();
      if(p) {
        cb = new detail::ControlBlock<T>(p);
        cb->strong_count.store(1, std::memory_order_relaxed);
        cb->weak_count.store(1, std::memory_order_relaxed);
        cb->managed = true;
        ptr         = p;
      }
    }
  }

  // Reset with new pointer and deleter
  template <typename Deleter> void reset(T* p, Deleter d) noexcept {
    if(p != ptr) {
      reset();
      if(p) {
        cb = new detail::ControlBlock<T, Deleter>(p, std::move(d));
        cb->strong_count.store(1, std::memory_order_relaxed);
        cb->weak_count.store(1, std::memory_order_relaxed);
        cb->managed = true;
        ptr         = p;
      }
    }
  }

  // Swap pointers
  void swap(Ref& other) noexcept {
    std::swap(ptr, other.ptr);
    std::swap(cb, other.cb);
  }

  // Get deleter
  template <typename Deleter> Deleter* get_deleter() const noexcept {
    if(cb) {
      return static_cast<Deleter*>(cb->get_deleter(typeid(Deleter)));
    }
    return nullptr;
  }

  // Owner_before for ordering
  template <typename U> bool owner_before(const Ref<U>& other) const noexcept { return cb < other.cb; }

  // Owner_before with WeakPtr
  template <typename U> bool owner_before(const WeakPtr<U>& other) const noexcept { return cb < other.cb; }

private:
  T* ptr                       = nullptr;
  detail::ControlBlockBase* cb = nullptr;

  template <typename U> Ref(U* p, detail::ControlBlockBase* control_block, detail::PrivateConstructorTag) noexcept : ptr(p), cb(control_block) {}
};

// WeakPtr<T> - Weak reference
template <typename T> class WeakPtr {
  template <typename U> friend class Ref;

  template <typename U> friend class WeakPtr;

  template <typename U> friend class enable_ref_from_this;

public:
  using element_type = T;

  // Default constructor
  constexpr WeakPtr() noexcept : ptr(nullptr), cb(nullptr) {}

  // Copy constructor
  WeakPtr(const WeakPtr& other) noexcept : ptr(other.ptr), cb(other.cb) {
    if(cb) {
      cb->weak_count.fetch_add(1, std::memory_order_acq_rel);
    }
  }

  // Copy constructor from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> WeakPtr(const WeakPtr<U>& other) noexcept : ptr(other.ptr), cb(other.cb) {
    if(cb) {
      cb->weak_count.fetch_add(1, std::memory_order_acq_rel);
    }
  }

  // Constructor from Ref
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> WeakPtr(const Ref<U>& ref) noexcept : ptr(ref.ptr), cb(ref.cb) {
    if(cb) {
      cb->weak_count.fetch_add(1, std::memory_order_acq_rel);
    }
  }

  // Move constructor
  WeakPtr(WeakPtr&& other) noexcept : ptr(other.ptr), cb(other.cb) {
    other.ptr = nullptr;
    other.cb  = nullptr;
  }

  // Move constructor from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> WeakPtr(WeakPtr<U>&& other) noexcept : ptr(other.ptr), cb(other.cb) {
    other.ptr = nullptr;
    other.cb  = nullptr;
  }

  // Destructor
  ~WeakPtr() { reset(); }

  // Copy assignment
  WeakPtr& operator=(const WeakPtr& other) noexcept {
    reset();
    ptr = other.ptr;
    cb  = other.cb;
    if(cb) {
      cb->weak_count.fetch_add(1, std::memory_order_acq_rel);
    }
    return *this;
  }

  // Copy assignment from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> WeakPtr& operator=(const WeakPtr<U>& other) noexcept {
    reset();
    ptr = other.ptr;
    cb  = other.cb;
    if(cb) {
      cb->weak_count.fetch_add(1, std::memory_order_acq_rel);
    }
    return *this;
  }

  // Assignment from Ref
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> WeakPtr& operator=(const Ref<U>& ref) noexcept {
    reset();
    ptr = ref.ptr;
    cb  = ref.cb;
    if(cb) {
      cb->weak_count.fetch_add(1, std::memory_order_acq_rel);
    }
    return *this;
  }

  // Move assignment
  WeakPtr& operator=(WeakPtr&& other) noexcept {
    reset();
    ptr       = other.ptr;
    cb        = other.cb;
    other.ptr = nullptr;
    other.cb  = nullptr;
    return *this;
  }

  // Move assignment from convertible type
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> WeakPtr& operator=(WeakPtr<U>&& other) noexcept {
    reset();
    ptr       = other.ptr;
    cb        = other.cb;
    other.ptr = nullptr;
    other.cb  = nullptr;
    return *this;
  }

  // Lock to create Ref
  Ref<T> lock() const noexcept {
    if(cb && cb->strong_count.load(std::memory_order_acquire) > 0) {
      cb->strong_count.fetch_add(1, std::memory_order_acq_rel);
      return Ref<T>(ptr, cb, detail::PrivateConstructorTag{});
    }
    return Ref<T>();
  }

  // Check if expired
  bool expired() const noexcept { return !cb || cb->strong_count.load(std::memory_order_acquire) == 0; }

  // Get use count
  long use_count() const noexcept { return cb ? cb->strong_count.load(std::memory_order_acquire) : 0; }

  // Reset
  void reset() noexcept {
    if(cb) {
      if(cb->weak_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        delete cb;
      }
    }
    ptr = nullptr;
    cb  = nullptr;
  }

  // Swap
  void swap(WeakPtr& other) noexcept {
    std::swap(ptr, other.ptr);
    std::swap(cb, other.cb);
  }

  // Owner_before
  template <typename U> bool owner_before(const Ref<U>& other) const noexcept { return cb < other.cb; }

  template <typename U> bool owner_before(const WeakPtr<U>& other) const noexcept { return cb < other.cb; }

private:
  T* ptr                       = nullptr;
  detail::ControlBlockBase* cb = nullptr;

  template <typename U> WeakPtr(U* p, detail::ControlBlockBase* control_block) noexcept : ptr(p), cb(control_block) {}
};

// enable_ref_from_this<T> - Mixin class for enabling Ref from *this
template <typename T> class enable_ref_from_this {
  template <typename U, typename... Args> friend Ref<U> make_ref(Args&&...);

public:
  // Get Ref from *this
  Ref<T> ref_from_this() {
    if(!weak_this.expired()) {
      return weak_this.lock();
    }
    return Ref<T>();
  }

  Ref<const T> ref_from_this() const {
    if(!weak_this.expired()) {
      return weak_this.lock();
    }
    return Ref<const T>();
  }

  // Get WeakPtr from *this
  WeakPtr<T> weak_from_this() noexcept { return weak_this; }

  WeakPtr<const T> weak_from_this() const noexcept { return weak_this; }

protected:
  enable_ref_from_this() noexcept = default;
  enable_ref_from_this(const enable_ref_from_this&) noexcept {}
  enable_ref_from_this& operator=(const enable_ref_from_this&) noexcept { return *this; }
  ~enable_ref_from_this() = default;

private:
  template <typename U> friend class Ref;

  mutable WeakPtr<T> weak_this;
};

// Helper: make_ref - Create a Ref with allocated memory
template <typename T, typename... Args> Ref<T> make_ref(Args&&... args) {
  auto* ptr = new T(std::forward<Args>(args)...);
  auto ref  = Ref<T>(ptr);

  // Set up enable_ref_from_this if applicable
  if constexpr(std::is_base_of_v<enable_ref_from_this<T>, T>) {
    auto& base     = static_cast<enable_ref_from_this<T>&>(*ptr);
    base.weak_this = WeakPtr<T>(ref);
  }

  return ref;
}

// Comparison operators
template <typename T, typename U> bool operator==(const Ref<T>& a, const Ref<U>& b) noexcept { return a.get() == b.get(); }

template <typename T, typename U> bool operator!=(const Ref<T>& a, const Ref<U>& b) noexcept { return a.get() != b.get(); }

template <typename T, typename U> bool operator<(const Ref<T>& a, const Ref<U>& b) noexcept { return a.get() < b.get(); }

template <typename T, typename U> bool operator<=(const Ref<T>& a, const Ref<U>& b) noexcept { return a.get() <= b.get(); }

template <typename T, typename U> bool operator>(const Ref<T>& a, const Ref<U>& b) noexcept { return a.get() > b.get(); }

template <typename T, typename U> bool operator>=(const Ref<T>& a, const Ref<U>& b) noexcept { return a.get() >= b.get(); }

// Null pointer comparisons
template <typename T> bool operator==(const Ref<T>& a, std::nullptr_t) noexcept { return a.get() == nullptr; }

template <typename T> bool operator==(std::nullptr_t, const Ref<T>& b) noexcept { return nullptr == b.get(); }

template <typename T> bool operator!=(const Ref<T>& a, std::nullptr_t) noexcept { return a.get() != nullptr; }

template <typename T> bool operator!=(std::nullptr_t, const Ref<T>& b) noexcept { return nullptr != b.get(); }

} // namespace cutil
