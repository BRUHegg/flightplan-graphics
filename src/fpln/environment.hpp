#pragma once

#include <cassert>
#include <charconv>
#include <cstdint>

#include <string>

#include <util/env_var_map.hpp>

namespace fms_environment {

const char AC_LAT_DEG_VAR[] = "ac_lat_deg";
const char AC_LON_DEG_VAR[] = "ac_lon_deg";
const char AC_BRNG_TRU_DEG_VAR[] = "ac_brng_tru_deg";
const char AC_SLIP_DEG_VAR[] = "ac_slip_deg";
const char AC_MAGVAR_DEG_VAR[] = "ac_magvar_deg";
const char AC_GS_KTS_VAR[] = "ac_gs_kts";
const char AC_TAS_KTS_VAR[] = "ac_tas_kts";
const char FPL_SEL[] = "fpl_sel";

constexpr double AC_LAT_DEF = 45.588670483;
constexpr double AC_LON_DEF = -122.598150383;
constexpr double AC_BRNG_TRU_DEF = 175.0;
constexpr double AC_SLIP_DEF = 0;
constexpr double AC_MAGVAR_DEF = 0;
constexpr double AC_GS_KTS_DEF = 0;
constexpr double AC_TAS_KTS_DEF = 0;

using val_ref_t = std::string;
using env_base_t = fms_environment::EnvVarMap<val_ref_t, 
  double, std::int64_t, std::string>;

class EnvDataRefMap : public env_base_t {
public:
  static constexpr std::size_t kConvertBuffLength = 100;
  static constexpr std::size_t kConvertPrecision = 10;
private:
  template<typename T>
  requires(std::integral<T> || std::floating_point<T>)
  std::optional<T> StrToNumber(const std::string& str) noexcept {
    T res_num;
    auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), res_num);
    if(ec == std::errc()) {
      return res_num;
    }
    return std::nullopt;
  }

  template<typename T>
  bool SetNumeric(value_type& dst, const std::string& val) noexcept {
    assert(std::holds_alternative<T>(dst));
    auto res = StrToNumber<T>(val);
    if(res) {
      dst = *res;
      return true;
    }
    return false;
  }

  template<typename T>
  std::optional<std::string> GetNumeric(const value_type& val) const noexcept {
    assert(std::holds_alternative<T>(val));
    char out_buff[kConvertBuffLength];
    T num = std::get<T>(val);
    if constexpr (std::is_floating_point_v<T>) {
      auto [ptr, ec] = std::to_chars(out_buff, out_buff + kConvertBuffLength, num,
        std::chars_format::scientific, kConvertPrecision);
      if(ec == std::errc()) {
        return std::string{out_buff, ptr};
      }
    } else {
      auto [ptr, ec] = std::to_chars(out_buff, out_buff + kConvertBuffLength, num);
      if(ec == std::errc()) {
        return std::string{out_buff, ptr};
      }
    }
    return std::nullopt;
  }
public:

  using str_type = val_ref_t;

  template<std::size_t N>
  EnvDataRefMap(const define_t (&initial)[N]) : env_base_t{initial} {}

  bool SetFromString(const str_type& key, const std::string& val) noexcept {
    std::unique_lock cr_lock(mtx_);
    auto it = values_.find(key);
    if(it == values_.end()) {
      return false;
    }
    if(std::holds_alternative<std::string>(it->second)) {
      it->second = val;
      return true;
    } else if(std::holds_alternative<std::int64_t>(it->second)) {
      return SetNumeric<std::int64_t>(it->second, val);
    }
    return SetNumeric<double>(it->second, val);
  }

  std::optional<std::string> GetString(const str_type& key) noexcept {
    std::shared_lock cr_lock(mtx_);
    auto it = values_.find(key);
    if(it == values_.end()) {
      return std::nullopt;
    }
    if(std::holds_alternative<std::string>(it->second)) {
      return std::get<std::string>(it->second);
    } else if(std::holds_alternative<std::int64_t>(it->second)) {
      return GetNumeric<std::int64_t>(it->second);
    }
    return GetNumeric<double>(it->second);
  }
};

using reference_desc_t = typename EnvDataRefMap::define_t;

const fms_environment::reference_desc_t kBaseVariables[] = {
     {AC_LAT_DEG_VAR, AC_LAT_DEF},
     {AC_LON_DEG_VAR, AC_LON_DEF},
     {AC_BRNG_TRU_DEG_VAR, AC_BRNG_TRU_DEF},
     {AC_SLIP_DEG_VAR, AC_SLIP_DEF},
     {AC_MAGVAR_DEG_VAR, AC_MAGVAR_DEF},
     {AC_GS_KTS_VAR, AC_GS_KTS_DEF},
     {AC_TAS_KTS_VAR, AC_TAS_KTS_DEF},
     {FPL_SEL, std::int64_t{1}}
};
} // fms_environment
