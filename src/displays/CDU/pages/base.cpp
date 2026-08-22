#include "base.hpp"

#include <string>

#include <displays/CDU/common.hpp>
#include <libnav/geo_utils.hpp>
#include <libnav/str_utils.hpp>

namespace {

const std::string CDU_ALL_S_WHITE = std::string(fms_displays::N_CDU_DATA_COLS, 
  fms_displays::CDU_S_WHITE);
const std::string CDU_ALL_B_WHITE = std::string(fms_displays::N_CDU_DATA_COLS, 
  fms_displays::CDU_B_WHITE);
} // namespace

namespace cdu_pages {

std::string string_from_error(fms_displays::CDUError err) noexcept {
  switch (err)
  {
  case(fms_displays::CDUError::INVALID_DELETE):
    return "INVALID_DELETE";
  case(fms_displays::CDUError::INVALID_ENTRY):
    return "INVALID ENTRY";
  case(fms_displays::CDUError::INVALID_ROUTE_UPLINK):
    return "INVALID ROUTE UPLINK";
  case(fms_displays::CDUError::NOT_IN_DATABASE):
    return "NOT IN DATABASE";
  default:
    return "";
  }
}

bool scratchpad_has_delete(const std::string& scratchpad) noexcept {
  if (scratchpad.size() && scratchpad[0] == fms_displays::DELETE_SYMBOL) {
    return true;
  }
  return false;
}

std::string get_scratchpad_pos(geo::point pos) noexcept {
  std::string tmp = strutils::lat_to_str(pos.lat_rad * geo::RAD_TO_DEG) 
    + strutils::lon_to_str(pos.lon_rad * geo::RAD_TO_DEG);
  std::string res;
  for(std::size_t i = 0; i < tmp.size(); ++i) {
    if(tmp[i] != strutils::DEGREE_SYMBOL) {
      res.push_back(tmp[i]);
    }
  }
  return res;
}

std::string str_align_right(const std::string& str) {
  if(str.size() > fms_displays::N_CDU_DATA_COLS) {
    std::size_t diff = str.size() - fms_displays::N_CDU_DATA_COLS;
    return str.substr(diff);
  }
  std::size_t diff = fms_displays::N_CDU_DATA_COLS - str.size();
  std::string out = std::string(diff, ' ') + str;
  return out;
}

std::string get_displayed_pos(geo::point pos) noexcept {
  return strutils::lat_to_str(pos.lat_rad * geo::RAD_TO_DEG) + " " + 
    strutils::lon_to_str(pos.lon_rad * geo::RAD_TO_DEG);
}

std::string get_small_heading(
  unsigned subpage, unsigned cnt_subpages) noexcept {
  std::string curr_spg = std::to_string(subpage);
  std::string n_spg = std::to_string(cnt_subpages);
  std::string out = curr_spg + "/" + n_spg + " ";
  out = std::string(std::size_t(fms_displays::N_CDU_DATA_COLS) - out.size(), 
    ' ') + out;
  return out;
}


cdu_scr_data_t::cdu_scr_data_t() {
  data_lines.reserve(fms_displays::N_CDU_DATA_LINES * 2);
  chr_sts.reserve(fms_displays::N_CDU_DATA_LINES * 2);
  for (unsigned i = 0; i < fms_displays::N_CDU_DATA_LINES; i++) {
    chr_sts.push_back(CDU_ALL_S_WHITE);
    chr_sts.push_back(CDU_ALL_B_WHITE);
  }
}

void PageBase::update() noexcept {}

void PageBase::on_page_change(fms_displays::CDUPage) noexcept {}
} // namespace cdu_pages
