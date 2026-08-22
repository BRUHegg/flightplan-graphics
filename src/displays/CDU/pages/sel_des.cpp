#include "sel_des.hpp"

#include <cstddef>

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <libnav/navaid_db.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace {

const char SEL_DES_WPT_HDG[] = " SELECT DESIRED WPT";
const std::size_t MAX_WPT_NAME_SZ = 5;
} // namespace

namespace cdu_pages {

SelectDesired::SelectDesired(util::OpaquePointer<fms_core::FPLSys> fpl_sys) :
  fpl_sys_{fpl_sys}, navaid_db_{fpl_sys->get_navaid_db_ptr()} {}

void SelectDesired::set_state(fms_displays::CDUPage outer_page,
  const std::string& wpt_name, double leg_id, double seg_id) {
  if(state_ != State::FAIL) {
    return;
  }
  page_ = outer_page;
  sel_des_nm_ = wpt_name;
  res_.leg_id_ = leg_id;
  res_.seg_id_ = seg_id;
  res_.err = Error::INVALID_ENTRY;
  curr_subpg_ = 1;
  if(wpt_name == "" || wpt_name.size() > MAX_WPT_NAME_SZ) {
    state_ = State::FAIL;
    return;
  }

  std::vector<libnav::waypoint_entry_t> tmp;
  std::size_t n_found = navaid_db_->get_wpt_data(wpt_name, &tmp);
  sel_des_data_ = tmp;
  cnt_subpg_ = n_found / fms_displays::N_CDU_DATA_LINES + 
    bool(n_found % fms_displays::N_CDU_DATA_LINES);
  if(n_found == 0) {
    state_ = State::FAIL;
    res_.err = Error::NOT_IN_DATA_BASE;
  } else if(n_found == 1) {
    state_ = State::SUCCESS;
    res_.err = Error::NONE;
    res_.waypoint.id = wpt_name;
    res_.waypoint.data = sel_des_data_[0];
  } else {
    state_ = State::WAIT;
  }
}

SelectDesired::State SelectDesired::get_state() const noexcept {
  return state_;
}

std::optional<SelectDesired::search_result_t> 
  SelectDesired::get_result() noexcept {
  if(state_ == State::WAIT) {
    return std::nullopt;
  }
  state_ = State::FAIL;
  return res_;
}

fms_displays::CDUPage SelectDesired::get_page_number() const noexcept {
  return fms_displays::CDUPage::SELECT_DESIRED;
}

cdu_event_res_t SelectDesired::on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept {
  UNUSED(s_out);
  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=page_};
  if(scratchpad_has_delete(scratchpad)) {
    res.err = fms_displays::CDUError::INVALID_DELETE;
    return res;
  } else if(!scratchpad.empty()) {
    res.err = fms_displays::CDUError::INVALID_ENTRY;
    return res;
  }
  unsigned i_start = (curr_subpg_ - 1U) * fms_displays::N_CDU_DATA_LINES;
  unsigned i_end = std::min(unsigned(sel_des_data_.size()), i_start + 
    fms_displays::N_CDU_DATA_LINES) - 1U;
  unsigned curr_idx = i_start + event - 1U;
  if (curr_idx <= i_end) {
    res_.waypoint.id = sel_des_nm_;
    res_.waypoint.data = sel_des_data_[curr_idx];
    res_.err = Error::NONE;
    state_ = State::SUCCESS;
  }

  return res;
}

void SelectDesired::on_page_change(fms_displays::CDUPage page) noexcept {
  if(page != page_ && page != fms_displays::CDUPage::NEXT_PAGE && page
    != fms_displays::CDUPage::PREV_PAGE) {
    state_ = State::FAIL;
    return;
  }
  if(state_ == State::WAIT) {
    if(page == fms_displays::CDUPage::NEXT_PAGE) {
      curr_subpg_++;
    } else if(page == fms_displays::CDUPage::PREV_PAGE) {
      curr_subpg_--;
    }
    if(curr_subpg_ == 0U) {
      curr_subpg_ = cnt_subpg_;
    } else if(curr_subpg_ > cnt_subpg_) {
      curr_subpg_ = 1U;
    }
  }
}

cdu_scr_data_t SelectDesired::get_screen_data() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};

  out.heading_small = get_small_heading(curr_subpg_, cnt_subpg_);
  out.heading_big = SEL_DES_WPT_HDG;
  out.heading_color = fms_displays::CDUColor::WHITE;

  std::size_t start_idx = std::size_t((curr_subpg_ - 1) * 6);
  std::size_t end_idx = std::min(sel_des_data_.size(), start_idx + 6);

  for (std::size_t i = start_idx; i < end_idx; i++) {
    std::string wpt_tp =
        sel_des_nm_ + " " + libnav::navaid_to_str(sel_des_data_[i].type);
    std::string lat_str =
        strutils::lat_to_str(sel_des_data_[i].pos.lat_rad * geo::RAD_TO_DEG);
    std::string lon_str =
        strutils::lon_to_str(sel_des_data_[i].pos.lon_rad * geo::RAD_TO_DEG);
    std::string main_str = lat_str + lon_str;
    if (sel_des_data_[i].navaid) {
      main_str =
          strutils::freq_to_str(sel_des_data_[i].navaid->freq) + " " + main_str;
    }
    out.data_lines.push_back(wpt_tp);
    out.data_lines.push_back(main_str);
  }

  return out;
}
} // namespace cdu_pages
