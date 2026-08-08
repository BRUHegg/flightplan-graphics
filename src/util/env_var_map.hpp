#pragma once

#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <variant>
#include <vector>

namespace fms_environment {

template<class Key, typename ... VarTypes>
class EnvVarMap {
protected:
  std::unordered_map<Key, std::variant<VarTypes...>> values_;
  mutable std::shared_mutex mtx_;

public:
  using value_type = std::variant<VarTypes...>;
  using define_t = std::pair<Key, std::variant<VarTypes...>>;

  template<std::size_t N>
  EnvVarMap(const define_t (&initial)[N]) {
    for(std::size_t i = 0; i < N; ++i) {
      values_[initial[i].first] = initial[i].second;
    }
  }

  template<typename T>
  std::optional<T> Get(const Key& key) const noexcept {
    std::shared_lock cr_lock(mtx_);
    auto it = values_.find(key);
    if(it == values_.end()) {
      return std::nullopt;
    }
    if(std::holds_alternative<T>(it->second)) {
      return std::get<T>(it->second);
    }
    return std::nullopt;
  }

  template<typename T>
  bool Set(const Key& key, const T& value) noexcept {
    std::unique_lock cr_lock(mtx_);
    auto it = values_.find(key);
    if(it == values_.end()) {
      return false;
    }
    if(std::holds_alternative<T>(it->second)) {
      it->second = value;
      return true;
    }
    return false;
  }

  bool HasKey(const Key& key) {
    std::unique_lock cr_lock(mtx_);
    auto it = values_.find(key);
    if(it == values_.end()) {
      return false;
    }
    return true;
  }
};
} // namespace fms_environment
