#include "ident.hpp"

#include <cassert>

#include <optional>
#include <string>

#include <fpln/fpln_sys.hpp>
#include <util/date_time.hpp>
#include <util/util.hpp>

namespace {

constexpr unsigned CNT_CDU_IDENT_NUMBER_LETTERS = 4;

const char IDENT_PAGE_HEADING[] = "         IDENT";
const char IDENT_LINE_1[] = " MODEL        ENG RATING";
const char IDENT_LINE_3[] = " NAV DATA         ACTIVE";
const char IDENT_LINE_9[] = "                 DRAG/FF";
const char ARM_IDENT_LINE_9[] = "             ARM DRAG/FF";
const char IDENT_LINE_12[] = "<INDEX         POS INIT>";
const char IDENT_ARM_CHANGE_STR[] = "ARM";

const char* MONTH_NAMES[] = {
  "JAN",
  "FEB",
  "MAR",
  "APR",
  "MAY",
  "JUN",
  "JUL",
  "AUG",
  "SEP",
  "OCT",
  "NOV",
  "DEC"
};
} // namespace

namespace cdu_pages {

std::string Ident::month_date_to_str(std::chrono::month_day md) noexcept {
  if(!md.ok()) {
    return "";
  }
  char day[3];
  day[1] = '\0'; day[2] = '\0';
  unsigned day_num = static_cast<unsigned>(md.day());
  if(day_num > 9) {
    day[0] = '0' + (day_num / 10);
    day[1] = '0' + (day_num % 10);
  } else {
    day[0] = '0' + day_num;
  }
  unsigned month_num = static_cast<unsigned>(md.month());
  return std::string{MONTH_NAMES[month_num - 1]} + std::string{day};
}

std::string Ident::get_ident_date_airac_string(unsigned airac) noexcept {
  util::airac_dates_t airac_dates = util::get_airac_dates(airac);
  char out_year[3];
  out_year[2] = '\0'; out_year[0] = '0' + (airac / 1000); 
  out_year[1] = '0' + ((airac / 100) % 10);
  std::string base_md = month_date_to_str(util::ymd_to_md(airac_dates.base));
  std::string exp_md = month_date_to_str(util::ymd_to_md(airac_dates.expiry));
  return base_md + exp_md + std::string{"/"} + std::string{out_year};
}

std::string Ident::get_ident_aircraft_string(
  const fms_core::aircraft_info_t& ac_inf) noexcept {
  if(ac_inf.model.size() + ac_inf.engine_model.size() >= 
    std::size_t{fms_displays::N_CDU_DATA_COLS}) {
    std::string raw = ac_inf.model + " " + ac_inf.engine_model;
    return raw.substr(0, std::size_t{fms_displays::N_CDU_DATA_COLS});
  }
  std::size_t n_spaces = std::size_t{fms_displays::N_CDU_DATA_COLS} - 
    ac_inf.model.size() - ac_inf.engine_model.size();
  return ac_inf.model + std::string(n_spaces, ' ') + ac_inf.engine_model;
}

std::string Ident::get_ident_airac_string(unsigned airac) noexcept {
  char out_buff[5];
  auto [ptr, ec] = std::to_chars(out_buff, out_buff + MY_ARRAY_SIZE(out_buff), 
    airac);
  if(ec != std::errc{}) {
    return {};
  }
  *ptr = '\0';
  std::string date_str = get_ident_date_airac_string(airac);
  std::string airac_str = std::string{"AIRAC-"} + std::string{out_buff};
  std::size_t cnt_spaces = std::size_t{24} - date_str.size() - airac_str.size();
  return airac_str + std::string(cnt_spaces, ' ') + date_str;
}

void Ident::fill_drag_ff_num(int num, char out_buff[5]) noexcept {
  out_buff[4] = '\0';
  out_buff[2] = '.';
  if(num < 0) {
    out_buff[0] = '-';
  } else {
    out_buff[0] = '+';
  }
  out_buff[3] = '0' + abs(num % 10);
  num /= 10;
  out_buff[1] = '0' + abs(num % 10);
}

std::string Ident::get_ident_drag_ff() const noexcept {
  char drag[5];
  char ff[5];
  fill_drag_ff_num(drag_, drag);
  fill_drag_ff_num(fuel_flow_, ff);
  return std::string{drag} + "/" + std::string{ff};
}

std::optional<int> Ident::get_ident_entry_number(const std::string& scratchpad) noexcept {
  if(scratchpad.size() != CNT_CDU_IDENT_NUMBER_LETTERS &&  
    scratchpad.size() != CNT_CDU_IDENT_NUMBER_LETTERS + 1) {
    return std::nullopt;
  }
  std::string entry;
  if(scratchpad.size() == CNT_CDU_IDENT_NUMBER_LETTERS + 1) {
    if(scratchpad[0] == '/') {
      entry = scratchpad.substr(1);
    } else if(scratchpad[CNT_CDU_IDENT_NUMBER_LETTERS] == '/') {
      entry = scratchpad.substr(0, CNT_CDU_IDENT_NUMBER_LETTERS);
    } else {
      return std::nullopt;
    }
  } else {
    entry = scratchpad;
  }
  
  bool is_positive = true;
  if(entry[0] == '-') {
    is_positive = false;
  } else if(entry[0] != '+') {
    return std::nullopt;
  }
  if(entry[1] > '9' || entry[1] < '0' || 
    entry[3] > '9' || entry[3] < '0' || 
    entry[2] != '.') {
    return std::nullopt;
  }
  int res = (entry[1] - '0') * 10 + (entry[3] - '0');
  if(!is_positive) {
    res *= -1;
  }
  return res;
}

Ident::Ident(util::OpaquePointer<fms_core::FPLSys> fpl_sys) {
  ac_info_ = fpl_sys->get_aircraft_info();
  airac_cycle_ = static_cast<unsigned>(
    fpl_sys->get_awy_db_ptr()->get_airac());
}

fms_displays::CDUPage Ident::get_page_number() const noexcept {
  return fms_displays::CDUPage::IDENT;
}

cdu_event_res_t Ident::on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept {
  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::IDENT};
  if(scratchpad_has_delete(scratchpad)) {
    res.err = fms_displays::CDUError::INVALID_DELETE;
    return res;
  }
  if(event == fms_displays::CDU_KEY_LSK_TOP + 5) {
    res.page = fms_displays::CDUPage::INIT_REF_INDEX;
    s_out = scratchpad;
    return res;
  } else if(event == fms_displays::CDU_KEY_RSK_TOP + 5) {
    res.page = fms_displays::CDUPage::POS_INIT;
    s_out = scratchpad;
    return res;
  } else if(event == fms_displays::CDU_KEY_RSK_TOP + 4 && !scratchpad.empty()) {
    if(scratchpad == std::string{IDENT_ARM_CHANGE_STR}) {
      is_armed_ = !is_armed_;
    } else {
      if(!is_armed_) {
        if(!scratchpad.empty()) {
          res.err = fms_displays::CDUError::INVALID_ENTRY;
        }
        return res;
      }
      auto entry_num = get_ident_entry_number(scratchpad);
      if(!entry_num) {
        res.err = fms_displays::CDUError::INVALID_ENTRY;
        return res;
      }
      int num = *entry_num;
      if(scratchpad.size() == CNT_CDU_IDENT_NUMBER_LETTERS || 
        scratchpad.back() == '/') {
        drag_ = num;
      } else {
        fuel_flow_ = num;
      }
    }
  }
  return res;
}

cdu_scr_data_t Ident::get_screen_data() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};
  out.heading_big = IDENT_PAGE_HEADING;
  out.heading_color = fms_displays::CDUColor::WHITE;
  out.data_lines.push_back(IDENT_LINE_1);
  out.data_lines.push_back(get_ident_aircraft_string(ac_info_));
  out.data_lines.push_back(IDENT_LINE_3);
  out.data_lines.push_back(get_ident_airac_string(airac_cycle_));
  out.data_lines.push_back("");
  out.data_lines.push_back("");
  out.data_lines.push_back("");
  out.data_lines.push_back("");
  if(is_armed_) {
    out.data_lines.push_back(ARM_IDENT_LINE_9);
  } else {
    out.data_lines.push_back(IDENT_LINE_9);
  }
  std::string drag_ff = get_ident_drag_ff();
  assert(drag_ff.size() < fms_displays::N_CDU_DATA_COLS);
  std::string drag_offset = std::string(
    fms_displays::N_CDU_DATA_COLS - drag_ff.size(), ' ');
  out.data_lines.push_back(drag_offset + drag_ff);
  out.data_lines.push_back(fms_displays::ALL_DASH);
  out.data_lines.push_back(IDENT_LINE_12);
  return out;
}
} // namespace cdu_pages
