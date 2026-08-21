#pragma once

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <libnav/arpt_db.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace cdu_pages {

class PosInit final : public PageBase {
  geo::point last_pos_;
  geo::point gps_pos_;
  std::optional<libnav::airport_t> ref_airport_;
  std::optional<geo::point> inertial_pos_;

  util::OpaquePointer<libnav::ArptDB> airport_db_;
  util::OpaquePointer<fms_core::FPLSys> fpl_sys_;

  std::string get_pos_init_airport_str() const noexcept;

  std::string get_pos_init_gps_pos_str() const noexcept;

  std::string get_pos_init_inertial_pos_str() const noexcept;
public:
  PosInit(util::OpaquePointer<fms_core::FPLSys> fpl_sys);

  fms_displays::CDUPage get_page_number() const noexcept override;

  void update() noexcept override;

  cdu_event_res_t on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept override;

  cdu_scr_data_t get_screen_data() const noexcept override;
};
} // namespace cdu_pages
