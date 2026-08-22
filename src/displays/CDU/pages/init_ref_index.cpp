#include "init_ref_index.hpp"

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace {

const char INIT_REF_INDEX_HEADING[] = "    INIT/REF INDEX";
const char* INIT_REF_INDEX_LINES[] = {
  "",
  "<IDENT         NAV DATA>",
  "",
  "<POS               ALTN>",
  "",
  "<PERF                   ",
  "",
  "<THRUST LIM             ",
  "",
  "<TAKEOFF                ",
  "",
  "<APPROACH         MAINT>"};
} // namespace

namespace cdu_pages {

InitRefIndex::InitRefIndex(util::OpaquePointer<fms_core::FPLSys> fpl_sys) {
  UNUSED(fpl_sys);
}

fms_displays::CDUPage InitRefIndex::get_page_number() const noexcept {
  return fms_displays::CDUPage::INIT_REF_INDEX;
}

cdu_event_res_t InitRefIndex::on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept {
  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::INIT_REF_INDEX};
  if(scratchpad_has_delete(scratchpad)) {
    res.err = fms_displays::CDUError::INVALID_DELETE;
    return res;
  }
  if(event == fms_displays::CDU_KEY_LSK_TOP) {
    res.page = fms_displays::CDUPage::IDENT;
    s_out = scratchpad;
    return res;
  } else if(event == fms_displays::CDU_KEY_LSK_TOP + 1) {
    res.page = fms_displays::CDUPage::POS_INIT;
    s_out = scratchpad;
    return res;
  } else if(event == fms_displays::CDU_KEY_LSK_TOP + 2) {
    res.page = fms_displays::CDUPage::INIT_REF;
    s_out = scratchpad;
    return res;
  }
  return res;
}

cdu_scr_data_t InitRefIndex::get_screen_data() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};
  out.heading_big = INIT_REF_INDEX_HEADING;
  out.heading_color = fms_displays::CDUColor::WHITE;
  for(std::size_t i = 0; i < MY_ARRAY_SIZE(INIT_REF_INDEX_LINES); ++i) {
    out.data_lines.push_back(std::string{INIT_REF_INDEX_LINES[i]});
  }
  return out;
}
} // namespace cdu_pages
