/*
    This project is licensed under
    Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

    A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

    Author: discord/bruh4096#4512

    This file contains definitions of member functions for FPLSys class.
    This class is used to manage all of the flight plans.
*/

#include "fpln_sys.hpp"

#include <cassert>
#include <cstdint>

#include <algorithm>
#include <mutex>
#include <optional>
#include <shared_mutex>

#include "environment.hpp"
#include <util/util.hpp>

namespace {

constexpr std::size_t DEFAULT_MAP_CTR_IDX = 2;
} // namespace

namespace fms_core {


fpln_info_t::fpln_info_t() {
  for(std::size_t i = 0; i < MY_ARRAY_SIZE(map_ctr_idx); ++i) {
    map_ctr_idx[i] = DEFAULT_MAP_CTR_IDX;
  }
}
// FPLSys member function definitions:

// Public member functions:

FPLSys::FPLSys(util::OpaquePointer<libnav::ArptDB> arpt_db,
               util::OpaquePointer<libnav::NavaidDB> navaid_db,
               util::OpaquePointer<libnav::AwyDB> awy_db, 
               util::OpaquePointer<fms_environment::EnvDataRefMap> env_map,
               path_type cifp_path,
               path_type fpl_path) : arpt_db_ptr_{arpt_db}, 
               navaid_db_ptr_{navaid_db}, awy_db_ptr_{awy_db},
               env_map_ptr_{env_map}, cifp_dir_path_{cifp_path},
               fpl_dir_{fpl_path} {

  cifp_dir_path_ = cifp_path;
  fpl_dir_ = fpl_path;

  for (size_t i = 0; i < N_FPL_SYS_RTES; i++) {
    fpl_vec_[i] = new flightplan_type{
      arpt_db_ptr_, navaid_db_ptr_, awy_db_ptr_, cifp_dir_path_};
  }

  leg_sel_cdu_l_ = {0, 0};
  leg_sel_cdu_r_ = {0, 0};

  fpl_datas_ = std::vector<fpln_data_t>(N_FPL_SYS_RTES);
  std::fill(rte_ids_, rte_ids_ + MY_ARRAY_SIZE(rte_ids_), -1.0);

  cdu_rte_idx_ = std::vector<size_t>(2);
  act_rte_idx_ = N_FPL_SYS_RTES;
  act_rte_id_ = -1;
  copy_ids_ = {-1, -1};

  flight_ident_ = "";
  fnb_dep_icao_ = {"", ""};

  nd_modes_ = std::vector<NDMode>(N_INTFCS, fms_core::NDMode::MAX);
  cdu_sel_fpl_ = std::vector<size_t>(N_INTFCS);

  double_values_ = position_.get_val_pointers();

  update_hot_env_vars();
}

void FPLSys::set_aircraft_info(const aircraft_info_t& a_inf) noexcept {
  std::unique_lock lk(main_mutex_);
  aircraft_info_ = a_inf;
}

aircraft_info_t FPLSys::get_aircraft_info() const noexcept {
  std::shared_lock lk(main_mutex_);
  return aircraft_info_;
}

std::size_t FPLSys::get_cnt_flplns() const noexcept { 
  return MY_ARRAY_SIZE(fpl_vec_); 
}

util::OpaquePointer<FPLSys::flightplan_type> FPLSys::get_fpln_ptr(
    std::size_t fpln_idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(fpln_idx < MY_ARRAY_SIZE(fpl_vec_));
  return util::OpaquePointer{fpl_vec_[fpln_idx]};
}

util::OpaquePointer<libnav::AwyDB> FPLSys::get_awy_db_ptr() const noexcept {
  return awy_db_ptr_;
}

util::OpaquePointer<libnav::ArptDB> FPLSys::get_arpt_db_ptr() const noexcept {
  return arpt_db_ptr_;
}

util::OpaquePointer<libnav::NavaidDB> FPLSys::get_navaid_db_ptr() const noexcept {
  return navaid_db_ptr_;
}

FPLSys::path_type FPLSys::get_fpln_dir() const noexcept { return fpl_dir_; }

std::pair<std::size_t, double> FPLSys::get_sel_leg(bool rt) const noexcept {
  std::shared_lock lk(main_mutex_);
  if (rt) {
    return leg_sel_cdu_r_;
  }
  return leg_sel_cdu_l_;
}

void FPLSys::set_sel_leg(std::pair<std::size_t, double> val, bool rt) noexcept {
  std::unique_lock lk(main_mutex_);
  if (rt) {
    leg_sel_cdu_r_ = val;
  }
  leg_sel_cdu_l_ = val;
}

bool FPLSys::get_exec() const noexcept { 
  std::shared_lock lk(main_mutex_);
  return execute_status_; 
}

size_t FPLSys::get_act_idx() const noexcept { 
  std::shared_lock lk(main_mutex_);
  return act_rte_idx_; 
}

std::vector<list_node_ref_t<fpl_seg_t>> FPLSys::get_seg_list(
  std::size_t* sz, std::size_t idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(idx < fpl_datas_.size());

  *sz = fpl_datas_[idx].seg_list.size();
  return fpl_datas_[idx].seg_list;
}

std::vector<list_node_ref_t<leg_list_data_t>> FPLSys::get_leg_list(
  std::size_t* sz, std::size_t idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(idx < N_FPL_SYS_RTES);

  *sz = fpl_datas_[idx].leg_list.size();
  return fpl_datas_[idx].leg_list;
}

std::size_t FPLSys::get_nd_seg(nd_leg_data_t* out, std::size_t n_max, 
  std::size_t idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(idx < N_FPL_SYS_RTES);

  if (fpl_datas_[idx].leg_list.size() == 0) return 0;
  std::size_t n_written = 0;
  std::size_t i_start = 1;

  if (fpl_datas_[idx].act_leg_idx != -1 && fpl_datas_[idx].act_leg_idx)
    i_start = std::size_t(fpl_datas_[idx].act_leg_idx) - 1;

  for (std::size_t i = i_start; i < fpl_datas_[idx].leg_list.size() - 1; i++) {
    if (!n_max) return n_written;

    if (fpl_datas_[idx].leg_list[i].data.is_discon) continue;

    nd_leg_data_t tmp;
    tmp.leg_data = fpl_datas_[idx].leg_list[i].data.misc_data;
    tmp.arc_ctr = fpl_datas_[idx].leg_list[i].data.leg.center_fix.data.pos;
    out[n_written] = tmp;

    n_max--;
    n_written++;
  }

  return n_written;
}

int FPLSys::get_act_leg_idx(std::size_t idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(idx < N_FPL_SYS_RTES);

  if (fpl_datas_[idx].act_leg_idx == -1) return -1;
  return 1;
}

void FPLSys::set_cdu_sel_fpl_idx(
  std::size_t src, std::size_t sd_idx) {
  assert(sd_idx < cdu_sel_fpl_.size());
  std::unique_lock lk(main_mutex_);
  cdu_sel_fpl_[sd_idx] = src;
}

std::size_t FPLSys::get_cdu_sel_fpl_idx(std::size_t sd_idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(sd_idx < cdu_sel_fpl_.size());
  return cdu_sel_fpl_[sd_idx];
}

void FPLSys::set_nd_mode(NDMode src, std::size_t sd_idx) {
  std::unique_lock lk(main_mutex_);
  assert(sd_idx < nd_modes_.size());
  nd_modes_[sd_idx] = src;
}

NDMode FPLSys::get_nd_mode(std::size_t sd_idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(sd_idx < nd_modes_.size());
  return nd_modes_[sd_idx];
}

bool FPLSys::get_ctr(geo::point* out, std::size_t sd_idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  std::size_t idx = cdu_sel_fpl_[sd_idx];
  std::size_t curr_idx = fpl_datas_[idx].map_ctr_idx[sd_idx];

  if (curr_idx + 1 < fpl_datas_[idx].leg_list.size()) {
    bool has_pos =
        fpl_datas_[idx].leg_list[curr_idx].data.misc_data.has_calc_wpt;
    if (has_pos) {
      *out = fpl_datas_[idx].leg_list[curr_idx].data.misc_data.calc_wpt.data.pos;
      return true;
    }
  }

  return false;
}

double FPLSys::get_rte_id(std::size_t sd_idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  return rte_ids_[sd_idx];
}

geo::point FPLSys::get_ac_pos() const noexcept {
  std::shared_lock lk(main_mutex_);
  return {position_.ac_lat_deg * geo::DEG_TO_RAD, 
    position_.ac_lon_deg * geo::DEG_TO_RAD};
}

hdg_info_t FPLSys::get_hdg_info() const noexcept {
  std::shared_lock lk(main_mutex_);
  hdg_info_t out = {};
  out.brng_tru_rad = position_.ac_brng_deg * geo::DEG_TO_RAD;
  out.slip_rad = position_.ac_slip_deg * geo::DEG_TO_RAD;
  out.magvar_rad = position_.ac_magvar_deg * geo::DEG_TO_RAD;
  return out;
}

spd_info_t FPLSys::get_spd_info() const noexcept {
  std::shared_lock lk(main_mutex_);
  spd_info_t out = {};
  out.gs_kts = position_.ac_gs_kts;
  out.tas_kts = position_.ac_tas_kts;

  return out;
}

act_leg_info_t FPLSys::get_act_leg_info(std::size_t idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  assert(idx < N_FPL_SYS_RTES);

  act_leg_info_t out = {};
  out.dist_nm = "----";
  out.dist_sz = DIST_FONT_SZ_DD;

  if (fpl_datas_[idx].act_leg_idx != -1) {
    leg_seg_t act_seg =
        fpl_datas_[idx].leg_list[fpl_datas_[idx].act_leg_idx].data.misc_data;
    geo::point curr_pos = {position_.ac_lat_deg * geo::DEG_TO_RAD,
                           position_.ac_lon_deg * geo::DEG_TO_RAD};

    out.name = act_seg.calc_wpt.id;
    out.dist_sz = DIST_FONT_SZ_DD;
    double dist_nm = -1;
    if (act_seg.has_calc_wpt) dist_nm = curr_pos.get_gc_dist_nm(act_seg.end);

    if (dist_nm != -1) {
      std::uint8_t out_prec = 0;
      if (dist_nm < 10) out_prec = 1;

      if (dist_nm >= 100) out.dist_sz = DIST_FONT_SZ_TD;

      out.dist_nm = strutils::double_to_str(dist_nm, out_prec);
    }
  }

  return out;
}

fpln_info_t FPLSys::get_fpl_info(size_t idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  fpln_info_t out;
  out.leg_list_id = fpl_datas_[idx].leg_list_id;
  out.seg_list_id = fpl_datas_[idx].seg_list_id;

  for(std::size_t i = 0; i < N_INTFCS; ++i)  {
    out.map_ctr_idx[i] = fpl_datas_[idx].map_ctr_idx[i];
  }
  out.fpl_id_last = fpl_datas_[idx].fpl_id_last;
  out.act_leg_idx = fpl_datas_[idx].act_leg_idx;

  return out;
}

void FPLSys::step_ctr(bool bwd, std::size_t sd_idx) {
  std::unique_lock lk(main_mutex_);
  std::size_t idx = cdu_sel_fpl_[sd_idx];
  if (!fpl_datas_[idx].leg_list.size()) return;

  std::size_t* curr_idx = &fpl_datas_[idx].map_ctr_idx[sd_idx];

  if (bwd) {
    if ((*curr_idx) - 1)
      *curr_idx = *curr_idx - 1;
    else
      *curr_idx = fpl_datas_[idx].leg_list.size() - 1;
    std::size_t curr_v = *curr_idx;

    while (fpl_datas_[idx].leg_list[*curr_idx].data.is_discon ||
           !fpl_datas_[idx].leg_list[*curr_idx].data.misc_data.has_calc_wpt) {
      if (*curr_idx)
        *curr_idx = *curr_idx - 1;
      else
        *curr_idx = fpl_datas_[idx].leg_list.size() - 1;
      if (*curr_idx == curr_v) break;
    }
  } else {
    if (*curr_idx < fpl_datas_[idx].leg_list.size() - 1)
      *curr_idx = *curr_idx + 1;
    else
      *curr_idx = 1;
    std::size_t curr_v = *curr_idx;

    while (fpl_datas_[idx].leg_list[*curr_idx].data.is_discon ||
           !fpl_datas_[idx].leg_list[*curr_idx].data.misc_data.has_calc_wpt) {
      if (*curr_idx < fpl_datas_[idx].leg_list.size() - 1)
        *curr_idx = *curr_idx + 1;
      else
        *curr_idx = 1;
      if (*curr_idx == curr_v) break;
    }
  }
}

void FPLSys::reset_ctr(std::size_t sd_idx) {
  std::unique_lock lk(main_mutex_);
  std::size_t idx = cdu_sel_fpl_[sd_idx];
  if (!fpl_datas_[idx].leg_list.size()) return;

  std::size_t* curr_idx = &fpl_datas_[idx].map_ctr_idx[sd_idx];

  if(fpl_datas_[idx].leg_list.size() <= 2) {
    *curr_idx = 1;
  } else {
    *curr_idx = 2;
  }
}

void FPLSys::rte_activate(size_t idx) {
  assert(idx && idx < N_FPL_SYS_RTES);
  std::unique_lock lk(main_mutex_);
  if (!fpl_vec_[idx]->can_activate()) return;
  act_rte_idx_ = idx;
}

void FPLSys::set_flt_nbr(std::string str) { flight_ident_ = str; }

std::string FPLSys::get_flt_nbr() const noexcept { 
  std::shared_lock lk(main_mutex_);
  return flight_ident_; 
}

RTECopySts FPLSys::act_can_copy() const noexcept {
  std::shared_lock lk(main_mutex_);
  if (act_rte_idx_ != N_FPL_SYS_RTES && !execute_status_) {
    double id1 = fpl_vec_[RTE1_IDX]->get_id();
    double id2 = fpl_vec_[RTE2_IDX]->get_id();
    if (id1 != copy_ids_[0] || id2 != copy_ids_[1]) return RTECopySts::READY;
    return RTECopySts::COMPLETE;
  }
  return RTECopySts::UNAVAIL;
}

void FPLSys::copy_act() {
  std::unique_lock lk(main_mutex_);
  if (act_rte_idx_ != N_FPL_SYS_RTES && !execute_status_) {
    size_t tgt_idx = RTE2_IDX;
    if (act_rte_idx_ == RTE2_IDX) tgt_idx = RTE1_IDX;
    fpl_vec_[tgt_idx]->copy_from_other(*fpl_vec_[act_rte_idx_]);

    double id1 = fpl_vec_[RTE1_IDX]->get_id();
    double id2 = fpl_vec_[RTE2_IDX]->get_id();
    copy_ids_[0] = id1;
    copy_ids_[1] = id2;
  }
}

void FPLSys::execute() {
  std::unique_lock lk(main_mutex_);
  if (execute_status_) {
    fpl_vec_[0]->copy_from_other(*fpl_vec_[act_rte_idx_]);
    execute_status_ = false;
    act_rte_id_ = fpl_vec_[act_rte_idx_]->get_id();
  }
}

void FPLSys::erase() {
  std::unique_lock lk(main_mutex_);
  if (execute_status_) {
    execute_status_ = false;
    if (act_rte_id_ == -1) {
      act_rte_idx_ = N_FPL_SYS_RTES;
      return;
    }
    fpl_vec_[act_rte_idx_]->copy_from_other(*fpl_vec_[0]);
    act_rte_id_ = fpl_vec_[act_rte_idx_]->get_id();
  }
}

void FPLSys::update() {
  std::unique_lock lk(main_mutex_);
  update_hot_env_vars();
  update_flight_plans();
}

FPLSys::~FPLSys() {
  for(std::size_t i = 0; i < MY_ARRAY_SIZE(fpl_vec_); ++i) {
    delete fpl_vec_[i];
  }
}

// Private member functions:

std::unordered_map<FPLSys::pos_data_t::str_type, double*> 
  FPLSys::pos_data_t::get_val_pointers() {
  std::unordered_map<FPLSys::pos_data_t::str_type, double*> res;
  res[fms_environment::AC_LAT_DEG_VAR] = &ac_lat_deg;
  res[fms_environment::AC_LON_DEG_VAR] = &ac_lon_deg;
  res[fms_environment::AC_BRNG_TRU_DEG_VAR] = &ac_brng_deg;
  res[fms_environment::AC_SLIP_DEG_VAR] = &ac_slip_deg;
  res[fms_environment::AC_MAGVAR_DEG_VAR] = &ac_magvar_deg;
  res[fms_environment::AC_GS_KTS_VAR] = &ac_gs_kts;
  res[fms_environment::AC_TAS_KTS_VAR] = &ac_tas_kts;
  return res;
}

void FPLSys::update_flight_plans() noexcept {
  for (size_t i = 0; i < N_FPL_SYS_RTES; i++) {
    bool cr_is_act = fpl_vec_[i]->is_active();
    if (!execute_status_) {
      if ((i == act_rte_idx_ || i == 0) && !cr_is_act)
        fpl_vec_[i]->activate();
      else if ((i != act_rte_idx_ && i) && cr_is_act)
        fpl_vec_[i]->deactivate();
    }
    fpl_vec_[i]->update(position_.ac_slip_deg * geo::DEG_TO_RAD);
    update_lists(i);

    update_flt_nbr(i);
    rte_ids_[i] = fpl_vec_[i]->get_id();
  }

  if (act_rte_idx_ < N_FPL_SYS_RTES &&
      act_rte_id_ != fpl_vec_[act_rte_idx_]->get_id()) {
    execute_status_ = true;
  }

  if (act_rte_idx_ < N_FPL_SYS_RTES && !fpl_vec_[act_rte_idx_]->can_activate() &&
      !execute_status_) {
    act_rte_idx_ = N_FPL_SYS_RTES;
    act_rte_id_ = -1;
  }
}

void FPLSys::update_seg_list(std::size_t idx) {
  assert(idx < N_FPL_SYS_RTES);

  std::size_t sz = fpl_vec_[idx]->get_seg_list_sz();
  fpl_datas_[idx].seg_list_id =
      fpl_vec_[idx]->get_sl_seg(0, sz, &fpl_datas_[idx].seg_list);
}

void FPLSys::update_leg_list(std::size_t idx) {
  assert(idx < N_FPL_SYS_RTES);

  size_t sz = fpl_vec_[idx]->get_leg_list_sz();
  fpl_datas_[idx].leg_list_id = fpl_vec_[idx]->get_ll_seg(
      0, sz, &fpl_datas_[idx].leg_list, &fpl_datas_[idx].act_leg_idx);

  for(std::size_t i = 0; i < N_INTFCS; ++i) {
    if (fpl_datas_[idx].map_ctr_idx[i] >= fpl_datas_[idx].leg_list.size() &&
        fpl_datas_[idx].leg_list.size() != 0) {
      fpl_datas_[idx].map_ctr_idx[i] = fpl_datas_[idx].leg_list.size() - 1;
    }
  }
}

void FPLSys::update_lists(std::size_t idx) {
  assert(idx < N_FPL_SYS_RTES);

  double fpl_id_curr = fpl_vec_[idx]->get_id();
  if (fpl_id_curr != fpl_datas_[idx].fpl_id_last) {
    update_seg_list(idx);
    update_leg_list(idx);
  }

  fpl_datas_[idx].fpl_id_last = fpl_id_curr;
}

void FPLSys::update_flt_nbr(size_t idx) {
  if (idx == RTE1_IDX || idx == RTE2_IDX) {
    size_t fnb_idx = idx == RTE2_IDX;
    std::string c_icao = fpl_vec_[idx]->get_dep_icao();
    if (c_icao != fnb_dep_icao_[fnb_idx]) {
      fnb_dep_icao_[fnb_idx] = c_icao;
      flight_ident_ = "";
    }
  }
}

void FPLSys::update_hot_env_vars() {
  for(auto [name, ptr]: double_values_) {
    auto resp = env_map_ptr_->Get<double>(name);
    if(resp) {
      *ptr = *resp;
    }
  }
}
}  // namespace test
