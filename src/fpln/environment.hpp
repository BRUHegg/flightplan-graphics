#pragma once

#include <cassert>
#include <charconv>
#include <cstdint>

#include <optional>
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
const char ND_IS_TRACK_UP_VAR[] = "nd_is_track_up";
const char ND_HDG_IS_TRUE_VAR[] = "nd_hdg_is_true";
const char ND_EFIS_AIRPORT_ON_VAR[] = "nd_efis_airport_on";
const char ND_EFIS_STATION_ON_VAR[] = "nd_efis_station_on";
const char ND_EFIS_WAYPOINT_ON_VAR[] = "nd_efis_waypoint_on";
const char AUTOPILOT_HDG_SEL_DEG_VAR[] = "ap_hdg_sel_deg";
const char AUTOPILOT_HDG_IS_TRACK_VAR[] = "ap_hdg_is_track";
const char ND_MODE_VAR[] = "nd_mode";
const char ND_RANGE_IDX_VAR[] = "nd_range_idx";
const char FPL_SEL_VAR[] = "fpl_sel";

constexpr double AC_LAT_DEF = 45.588670483;
constexpr double AC_LON_DEF = -122.598150383;
constexpr double AC_BRNG_TRU_DEF = 175.0;
constexpr double AC_SLIP_DEF = 0;
constexpr double AC_MAGVAR_DEF = 0;
constexpr double AC_GS_KTS_DEF = 0;
constexpr double AC_TAS_KTS_DEF = 0;

using val_ref_t = std::string;
using env_base_t = fms_environment::EnvVarMap<val_ref_t, 
  bool, double, std::int64_t, std::string>;

class EnvDataRefMap final: public env_base_t {
public:
  using str_type = val_ref_t;

  template<std::size_t N>
  EnvDataRefMap(const define_t (&initial)[N]) : env_base_t{initial} {}

  bool SetFromString(const str_type& key, const std::string& val) noexcept;

  std::optional<std::string> GetString(const str_type& key) noexcept;
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
     {AUTOPILOT_HDG_SEL_DEG_VAR, std::int64_t{340}},
     {AUTOPILOT_HDG_IS_TRACK_VAR, false},
     {ND_IS_TRACK_UP_VAR, true},
     {ND_HDG_IS_TRUE_VAR, false},
     {std::string{ND_EFIS_AIRPORT_ON_VAR} + "_0", false},
     {std::string{ND_EFIS_AIRPORT_ON_VAR} + "_1", false},
     {std::string{ND_EFIS_STATION_ON_VAR} + "_0", false},
     {std::string{ND_EFIS_STATION_ON_VAR} + "_1", false},
     {std::string{ND_EFIS_WAYPOINT_ON_VAR} + "_0", false},
     {std::string{ND_EFIS_WAYPOINT_ON_VAR} + "_1", false},
     {std::string{ND_MODE_VAR} + "_0", std::int64_t{2}},
     {std::string{ND_MODE_VAR} + "_1", std::int64_t{2}},
     {std::string{ND_RANGE_IDX_VAR} + "_0", std::int64_t{0}},
     {std::string{ND_RANGE_IDX_VAR} + "_1", std::int64_t{0}},
     {FPL_SEL_VAR, std::int64_t{1}}
};
} // fms_environment
