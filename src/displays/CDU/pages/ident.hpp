#pragma once

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace cdu_pages {

class Ident final : public PageBase {
  unsigned airac_cycle_;
  fms_core::aircraft_info_t ac_info_;
  int drag_ = 0;
  int fuel_flow_ = 0;
  bool is_armed_ = false;

  static std::string month_date_to_str(std::chrono::month_day md) noexcept;

  static std::string get_ident_date_airac_string(
    unsigned airac) noexcept;

  static std::string get_ident_aircraft_string(
    const fms_core::aircraft_info_t& ac_inf) noexcept;

  static std::string get_ident_airac_string(
    unsigned airac) noexcept;

  static void fill_drag_ff_num(int num, char out_buff[5]) noexcept;

  static std::optional<int> get_ident_entry_number(
    const std::string& scratchpad) noexcept;

  std::string get_ident_drag_ff() const noexcept;

public:
  explicit Ident(util::OpaquePointer<fms_core::FPLSys> fpl_sys);

  fms_displays::CDUPage get_page_number() const noexcept override;

  cdu_event_res_t on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept override;

  cdu_scr_data_t get_screen_data() const noexcept override;
};
} // namespace cdu_pages
