#include "pos_init.hpp"

#include <cassert>
#include <cstddef>

#include <format>
#include <optional>
#include <string>
#include <vector>

#include <libnav/arpt_db.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace {

// POS INIT page:
const char POS_INIT_HEADING[] = "      POS INIT";
const char POS_INIT_LINE_1[] = "                LAST POS";
const char POS_INIT_LINE_3[] = " REF AIRPORT";
const char POS_INIT_LINE_5[] = " GATE";
const char POS_INIT_LINE_7[] = " UTC             GPS POS";
const char POS_INIT_LINE_9[] = "        SET INERTIAL POS";
const char POS_INIT_LINE_12[] = "<INDEX            ROUTE>";
const char POS_INIT_INERTIAL_POS_BLANK_STR[] = {
  '@', '@', '@', strutils::DEGREE_SYMBOL, '@', '@', '.', '@', ' ',
  '@', '@', '@', '@', strutils::DEGREE_SYMBOL, '@', '@', '.', '@', '\0'
};
} // namespace

namespace cdu_pages {

std::string PosInit::get_pos_init_airport_str() const noexcept {
  if(!ref_airport_) {
    return "----";
  }
  std::string res_icao = ref_airport_->icao;
  std::string res_pos = get_displayed_pos(ref_airport_->data.pos);
  assert(res_icao.size() + res_pos.size() <= fms_displays::N_CDU_DATA_COLS);
  std::size_t diff = fms_displays::N_CDU_DATA_COLS - res_icao.size() - 
    res_pos.size();
  std::string spaces(diff, ' ');
  return res_icao + spaces + res_pos;
}

std::string PosInit::get_pos_init_gps_pos_str() const noexcept {
  auto now_utc = std::chrono::utc_clock::now();
  std::string utc_str = std::format("{:%H%M}", now_utc);
  std::string pos_str = get_displayed_pos(fpl_sys_->get_ac_pos());
  assert(fms_displays::N_CDU_DATA_COLS >= utc_str.size() + pos_str.size());
  std::size_t cnt_spaces = fms_displays::N_CDU_DATA_COLS - utc_str.size() - 
    pos_str.size();
  return utc_str + std::string(cnt_spaces, ' ') + pos_str;
}

std::string PosInit::get_pos_init_inertial_pos_str() const noexcept {
  return str_align_right(std::string{POS_INIT_INERTIAL_POS_BLANK_STR});
}

PosInit::PosInit(util::OpaquePointer<fms_core::FPLSys> fpl_sys) : 
  airport_db_{fpl_sys->get_arpt_db_ptr()}, fpl_sys_{fpl_sys} {
  last_pos_ = fpl_sys_->get_ac_pos();
  gps_pos_ = last_pos_;
}

fms_displays::CDUPage PosInit::get_page_number() const noexcept {
  return fms_displays::CDUPage::POS_INIT;
}

void PosInit::update() noexcept {
  gps_pos_ = fpl_sys_->get_ac_pos();
}

cdu_event_res_t PosInit::on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept {
  cdu_event_res_t res{.err = fms_displays::CDUError::NONE, 
    .page = fms_displays::CDUPage::POS_INIT};
  if(scratchpad_has_delete(scratchpad)) {
    res.err = fms_displays::CDUError::INVALID_DELETE;
    return res;
  }
  if(event == fms_displays::CDU_KEY_LSK_TOP + 5) {
    res.page = fms_displays::CDUPage::INIT_REF_INDEX;
    s_out = scratchpad;
    return res;
  } else if(event == fms_displays::CDU_KEY_RSK_TOP + 5) {
    res.page = fms_displays::CDUPage::RTE;
    s_out = scratchpad;
    return res;
  } else if(event == fms_displays::CDU_KEY_LSK_TOP + 1) {
    libnav::airport_t tmp;
    tmp.icao = scratchpad;
    bool is_airport = airport_db_->get_airport_data(tmp.icao, &tmp.data);
    
    if(!is_airport) {
      res.err = fms_displays::CDUError::NOT_IN_DATABASE;
      return res;
    }
    ref_airport_ = tmp;
    return res;
  } else if(event == fms_displays::CDU_KEY_RSK_TOP) {
    if(!scratchpad.empty()) {
      res.err = fms_displays::CDUError::INVALID_ENTRY;
      return res;
    }
    s_out = get_scratchpad_pos(last_pos_);
    return res;
  } else if(event == fms_displays::CDU_KEY_RSK_TOP + 1) {
    if(!scratchpad.empty()) {
      res.err = fms_displays::CDUError::INVALID_ENTRY;
      return res;
    }
    if(ref_airport_) {
      s_out = get_scratchpad_pos(ref_airport_->data.pos);
      return res;
    }
  } else if(event == fms_displays::CDU_KEY_RSK_TOP + 3) {
    if(!scratchpad.empty()) {
      res.err = fms_displays::CDUError::INVALID_ENTRY;
      return res;
    }
    s_out = get_scratchpad_pos(fpl_sys_->get_ac_pos());
    return res;
  }
  s_out = scratchpad;
  return res;
}

cdu_scr_data_t PosInit::get_screen_data() const noexcept {
  cdu_scr_data_t out = {};
  out.heading_big = POS_INIT_HEADING;
  out.heading_color = fms_displays::CDUColor::WHITE;
  out.data_lines.push_back(POS_INIT_LINE_1);
  out.data_lines.push_back(str_align_right(
    get_displayed_pos(last_pos_)));
  out.data_lines.push_back(POS_INIT_LINE_3);
  out.data_lines.push_back(get_pos_init_airport_str());
  out.data_lines.push_back(POS_INIT_LINE_5);
  out.data_lines.push_back("");
  out.data_lines.push_back(POS_INIT_LINE_7);
  out.data_lines.push_back(get_pos_init_gps_pos_str());
  out.data_lines.push_back(POS_INIT_LINE_9);
  out.data_lines.push_back(get_pos_init_inertial_pos_str());
  out.data_lines.push_back(fms_displays::ALL_DASH);
  out.data_lines.push_back(POS_INIT_LINE_12);
  return out;
}
} // namespace cdu_pages