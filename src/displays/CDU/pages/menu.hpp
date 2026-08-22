#pragma once

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <util/util.hpp>

#include "base.hpp"

namespace cdu_pages {

class Menu final : public PageBase {
public:
  explicit Menu(util::OpaquePointer<fms_core::FPLSys> fpl_sys);

  fms_displays::CDUPage get_page_number() const noexcept override;

  cdu_event_res_t on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept override;

  cdu_scr_data_t get_screen_data() const noexcept override;
};
} // namespace cdu_pages
