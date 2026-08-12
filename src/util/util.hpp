#pragma once

#include <cstddef>

#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <unordered_map>

namespace util {

#define UNUSED(x) (void)(x)
#define MY_ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

template<typename T, bool is_shared>
class LockWrapper final {
  std::unique_lock<T> lk_;
public:
  explicit LockWrapper(T& mtx) : lk_{mtx} {}
};

template<typename T>
class LockWrapper<T, true> final {
  std::shared_lock<T> lk_;
public:
  explicit LockWrapper(T& mtx) : lk_{mtx} {}
};

#define MY_MAKE_TEST_ATTR_SHARED_IMPL(x) shared_##x

#define MY_MAKE_TEST_ATTR_SHARED(x) MY_MAKE_TEST_ATTR_SHARED_IMPL(x)

#define MY_MAKE_NAME_UNIQUE_IMPL(x, ln) x##ln

#define MY_MAKE_NAME_UNIQUE(x, ln) MY_MAKE_NAME_UNIQUE_IMPL(x, ln)

#define MY_ATTR_SHARED(x) static constexpr bool MY_MAKE_TEST_ATTR_SHARED(x)() noexcept { return true; }

#define MY_ATTR_UNIQUE(x) static constexpr bool MY_MAKE_TEST_ATTR_SHARED(x)() noexcept { return false; }

#define MY_SHARED_ATTR_LOCK(base, x, mtx) ::util::LockWrapper<std::decay_t<decltype(mtx)>, base::MY_MAKE_TEST_ATTR_SHARED(x)()> MY_MAKE_NAME_UNIQUE(lock, __LINE__)(mtx)

#define MY_MUTEX_WRAPPER_FUNC_BODY(obj, obj_type, func, mtx, ...) MY_SHARED_ATTR_LOCK(obj_type, func, mtx); return obj.func(__VA_ARGS__);

template<typename T>
struct array_data_t {
  T* ptr;
  std::size_t size;
};

using const_str_data_t = array_data_t<const char*>;

struct enum_class_hash_t {
  template <typename T>
  size_t operator()(T t) const {
    return static_cast<size_t>(t);
  }
};

template<typename T>
class OpaquePointer final {
  T* ptr_ = nullptr;
public:
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;

  OpaquePointer() = default;

  explicit OpaquePointer(T* ptr) : ptr_{ptr} {}

  pointer get() noexcept {
    return ptr_;
  }

  const_pointer get() const noexcept {
    return ptr_;
  }

  pointer operator->() noexcept {
    return ptr_;
  }

  const_pointer operator->() const noexcept {
    return ptr_;
  }

  reference operator*() noexcept {
    return *ptr_;
  }

  const_reference operator*() const noexcept {
    return *ptr_;
  }

  reference operator[](std::size_t n) noexcept {
    return *(ptr_ + n);
  }

  const_reference operator[](std::size_t n) const noexcept {
    return *(ptr_ + n);
  }
};
};  // namespace util
