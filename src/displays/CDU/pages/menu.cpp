#include "menu.hpp"

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace {

const char MENU_PAGE_HEADING[] = "          MENU";
const char MENU_LINE_1[] = "                EFIS CTL";
const char MENU_LINE_2[] = "<FMC";
const char MENU_LINE_4[] = "<DLNK";
const char MENU_LINE_5[] = "                 DSP CTL";
const char MENU_LINE_6[] = "<SAT";
} // namespace

namespace cdu_pages {

Menu::Menu(util::OpaquePointer<fms_core::FPLSys> fpl_sys) { UNUSED(fpl_sys); }

fms_displays::CDUPage Menu::get_page_number() const noexcept {
  return fms_displays::CDUPage::MENU;
}

cdu_event_res_t Menu::on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept {
  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::MENU};
  if(scratchpad_has_delete(scratchpad)) {
    res.err = fms_displays::CDUError::INVALID_DELETE;
    return res;
  }
  if(event == fms_displays::CDU_KEY_LSK_TOP) {
    res.page = fms_displays::CDUPage::IDENT;
    s_out = scratchpad;
    return res;
  }
  if(!scratchpad.empty()) {
    res.err = fms_displays::CDUError::INVALID_ENTRY;
    return res;
  }
  return res;
}

cdu_scr_data_t Menu::get_screen_data() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};
  out.heading_big = std::string{MENU_PAGE_HEADING};
  out.data_lines.push_back(MENU_LINE_1);
  out.data_lines.push_back(MENU_LINE_2);
  out.data_lines.push_back("");
  out.data_lines.push_back(MENU_LINE_4);
  out.data_lines.push_back(MENU_LINE_5);
  out.data_lines.push_back(MENU_LINE_6);
  return out;
}
} // namespace cdu_pages
