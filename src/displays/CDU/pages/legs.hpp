#pragma once

#include <cstddef>

#include <optional>
#include <string>
#include <vector>

#include <fpln/fpln_sys.hpp>
#include <libnav/geo_utils.hpp>
#include <util/util.hpp>

#include <displays/CDU/cdu_context.hpp>

#include "base.hpp"
#include "sel_des.hpp"

namespace cdu_pages {

class Legs final : public PageBase {
  using flightplan_type = typename fms_core::FPLSys::flightplan_type;

  util::OpaquePointer<fms_core::FPLSys> fpl_sys_;
  util::OpaquePointer<fms_displays::cdu_context_t> cdu_cntx_;

  unsigned curr_subpg_ = 1;
  unsigned cnt_subpg_ = 0;

  fms_core::NDMode nd_mode_;
  util::OpaquePointer<flightplan_type> fpln_;

  bool leg_sel_pr_ = false;
  size_t n_seg_list_sz_, n_leg_list_sz_;
  std::vector<fms_core::list_node_ref_t<fms_core::fpl_seg_t>> seg_list_;
  std::vector<fms_core::list_node_ref_t<fms_core::leg_list_data_t>> leg_list_;
  fms_core::fpln_info_t fpl_infos_[fms_core::N_FPL_SYS_RTES];
  std::pair<std::size_t, double> leg_sel_[fms_displays::N_CDU_RTES];
  std::size_t pln_ctr_idx_[fms_displays::N_CDU_RTES];
  geo::point pln_ctr_pos_[fms_displays::N_CDU_RTES];

  std::string get_cdu_leg_nm(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) const noexcept;
  
  std::size_t get_leg_start_idx() const noexcept;

  std::size_t get_leg_end_idx() const noexcept;

  void reset_leg_dto_sel(std::size_t fp_idx) noexcept;

  void reset_leg_all_sel() noexcept;

  void handle_legs_map_ctr_advance() noexcept;

  bool handle_legs_dto(size_t usr_idx, std::string scratchpad,
                          std::string& s_out) noexcept;

  cdu_event_res_t handle_legs_delete(std::size_t usr_idx) noexcept;

  cdu_event_res_t handle_legs_insert(std::size_t usr_idx, 
    const std::string& scratchpad) noexcept;

  static fms_core::spd_cstr_t get_spd_cstr(const std::string& str) noexcept;

  static fms_core::alt_cstr_t get_alt_cstr(const std::string& str) noexcept;

  cdu_event_res_t handle_legs_cstr_mod(std::size_t usr_idx, 
    const std::string& scratchpad) noexcept;

  void update_fpl_infos() noexcept;

  static std::string get_cdu_leg_prop(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) noexcept;

  static std::string get_cdu_leg_spdcstr(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) noexcept;

  static std::string get_leg_alt(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src, 
    bool alt2=false, bool fl=false) noexcept;

  static std::string get_cdu_leg_vcstr(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) noexcept;

  std::string get_legs_bottom() const noexcept;
public:
  explicit Legs(util::OpaquePointer<fms_core::FPLSys> fpl_sys, 
    util::OpaquePointer<fms_displays::cdu_context_t> cntx);

  fms_displays::CDUPage get_page_number() const noexcept override;

  void update() noexcept override;

  cdu_event_res_t on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept override;

  void on_page_change(fms_displays::CDUPage page) noexcept override;

  cdu_scr_data_t get_screen_data() const noexcept override;
};
} // namespace cdu_pages
