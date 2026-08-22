#pragma once

#include <cstddef>

#include <fpln/fpln_sys.hpp>
#include <util/util.hpp>

#include "pages/sel_des.hpp"

namespace fms_displays {

struct cdu_context_t {
  cdu_pages::SelectDesired select_desired;

  std::size_t side_index;
  std::size_t sel_fpl_idx;
  std::size_t act_fpl_idx;

  cdu_context_t(std::size_t sd_index, std::size_t s_fpl_idx, 
    std::size_t a_fpl_idx, util::OpaquePointer<fms_core::FPLSys> fpl_sys);
};
} // namespace fms_displays
