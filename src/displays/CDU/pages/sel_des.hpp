#pragma once

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <libnav/navaid_db.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace cdu_pages {

class SelectDesired final : public PageBase {
public:
  enum class State {
    WAIT,
    FAIL,
    SUCCESS
  };

  enum class Error {
    INVALID_ENTRY,
    NOT_IN_DATA_BASE,
    NONE
  };

  struct search_result_t {
    libnav::waypoint_t waypoint;
    double seg_id_ = -1.0;
    double leg_id_ = -1.0;
    Error err = Error::INVALID_ENTRY;
  };
private:
  util::OpaquePointer<fms_core::FPLSys> fpl_sys_;
  util::OpaquePointer<libnav::NavaidDB> navaid_db_;

  // this is a nested page. When on_event is called it'll pretend to
  // be page_
  fms_displays::CDUPage page_ = fms_displays::CDUPage::SELECT_DESIRED; 
  State state_ = State::FAIL;

  search_result_t res_;

  unsigned curr_subpg_ = 1;
  unsigned cnt_subpg_ = 0;

  std::vector<libnav::waypoint_entry_t> sel_des_data_;
  std::string sel_des_nm_ = "";
public:
  explicit SelectDesired(util::OpaquePointer<fms_core::FPLSys> fpl_sys);

  void set_state(fms_displays::CDUPage outer_page, 
    const std::string& wpt_name, double leg_id, double seg_id);

  State get_state() const noexcept;

  std::optional<search_result_t> get_result() noexcept;

  fms_displays::CDUPage get_page_number() const noexcept override;

  cdu_event_res_t on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept override;

  void on_page_change(fms_displays::CDUPage page) noexcept override;

  cdu_scr_data_t get_screen_data() const noexcept override;
};
} // namespace cdu_pages
