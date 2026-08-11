#pragma once

#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <unordered_map>

namespace util {

#define UNUSED(x) (void)(x)

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

struct enum_class_hash_t {
  template <typename T>
  size_t operator()(T t) const {
    return static_cast<size_t>(t);
  }
};
};  // namespace util
