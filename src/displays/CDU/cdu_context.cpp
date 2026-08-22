#include "cdu_context.hpp"

namespace fms_displays {

cdu_context_t::cdu_context_t(std::size_t sd_index, std::size_t s_fpl_idx, 
    std::size_t a_fpl_idx, util::OpaquePointer<fms_core::FPLSys> fpl_sys) :
  select_desired{fpl_sys}, side_index{sd_index}, sel_fpl_idx{s_fpl_idx}, 
  act_fpl_idx{a_fpl_idx} {}
} // namespace fms_displays