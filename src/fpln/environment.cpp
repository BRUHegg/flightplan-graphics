#include "environment.hpp"

#include <cassert>
#include <charconv>
#include <cstdint>

#include <optional>
#include <mutex>
#include <shared_mutex>
#include <string>

namespace {

constexpr std::size_t kConvertBuffLength = 100;
constexpr std::size_t kConvertPrecision = 10;

using value_type = typename fms_environment::EnvDataRefMap::value_type;

template <typename T>
  requires(std::integral<T> || std::floating_point<T>)
std::optional<T> StrToNumber(const std::string& str) noexcept {
  T res_num;
  auto [ptr, ec] =
      std::from_chars(str.data(), str.data() + str.size(), res_num);
  if (ec == std::errc()) {
    return res_num;
  }
  return std::nullopt;
}

template <typename T>
bool SetNumeric(value_type& dst, const std::string& val) noexcept {
  assert(std::holds_alternative<T>(dst));
  auto res = StrToNumber<T>(val);
  if (res) {
    dst = *res;
    return true;
  }
  return false;
}

template <typename T>
std::optional<std::string> GetNumeric(const value_type& val) noexcept {
  assert(std::holds_alternative<T>(val));
  char out_buff[kConvertBuffLength];
  T num = std::get<T>(val);
  if constexpr (std::is_floating_point_v<T>) {
    auto [ptr, ec] =
        std::to_chars(out_buff, out_buff + kConvertBuffLength, num,
                      std::chars_format::scientific, kConvertPrecision);
    if (ec == std::errc()) {
      return std::string{out_buff, ptr};
    }
  } else {
    auto [ptr, ec] =
        std::to_chars(out_buff, out_buff + kConvertBuffLength, num);
    if (ec == std::errc()) {
      return std::string{out_buff, ptr};
    }
  }
  return std::nullopt;
}
}  // namespace

namespace fms_environment {

bool EnvDataRefMap::SetFromString(const str_type& key,
                                  const std::string& val) noexcept {
  std::unique_lock cr_lock(mtx_);
  auto it = values_.find(key);
  if (it == values_.end()) {
    return false;
  }
  if (std::holds_alternative<std::string>(it->second)) {
    it->second = val;
    return true;
  } else if (std::holds_alternative<std::int64_t>(it->second)) {
    return SetNumeric<std::int64_t>(it->second, val);
  }
  return SetNumeric<double>(it->second, val);
}

std::optional<std::string> EnvDataRefMap::GetString(
    const str_type& key) noexcept {
  std::shared_lock cr_lock(mtx_);
  auto it = values_.find(key);
  if (it == values_.end()) {
    return std::nullopt;
  }
  if (std::holds_alternative<std::string>(it->second)) {
    return std::get<std::string>(it->second);
  } else if (std::holds_alternative<std::int64_t>(it->second)) {
    return GetNumeric<std::int64_t>(it->second);
  }
  return GetNumeric<double>(it->second);
}
}  // namespace fms_environment