/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        Author: discord/bruh4096#4512

        This file contains declarations of member functions for FPLSys class.
    This class is used to manage all of the flight plans.
*/

#pragma once

#include <iostream>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "environment.hpp"
#include "flightpln_int.hpp"

#define UNUSED(x) (void)(x)

namespace test {
enum class RTECopySts { READY, COMPLETE, UNAVAIL };

enum class NDMode { APP, VOR, MAP, PLAN };

constexpr NDMode DFLT_ND_MODE = NDMode::MAP;

// FplSys stores 3 routes in fpl_vec_
// Index 0 is for active route, 1 for RTE1 and 2 for RTE2
constexpr size_t N_FPL_SYS_RTES = 3;
constexpr size_t ACT_RTE_IDX = 0;
constexpr size_t RTE1_IDX = 1;
constexpr size_t RTE2_IDX = 2;
constexpr size_t N_INTFCS = 2;  // Number of interfaces(interface is a CDU+ND)

constexpr double DIST_FONT_SZ_DD = 21;  // Double digits;
constexpr double DIST_FONT_SZ_TD = 19;

struct hdg_info_t {
  double brng_tru_rad, slip_rad, magvar_rad;
};

struct spd_info_t {
  double gs_kts, tas_kts;
};

struct act_leg_info_t {
  std::string name, time_z, dist_nm;
  double dist_sz;
};

struct fpln_info_t {
  double leg_list_id;
  double seg_list_id;

  size_t cap_ctr_idx, fo_ctr_idx;
  double fpl_id_last;
  int act_leg_idx;
};

struct fpln_data_t : fpln_info_t {
  std::vector<list_node_ref_t<fpl_seg_t>> seg_list;
  std::vector<list_node_ref_t<leg_list_data_t>> leg_list;
};

class FPLSys {
 public:
  FPLSys(std::shared_ptr<libnav::ArptDB> arpt_db,
         std::shared_ptr<libnav::NavaidDB> navaid_db,
         std::shared_ptr<libnav::AwyDB> awy_db, 
         std::shared_ptr<fms_environment::EnvDataRefMap> env_map,
         std::string cifp_path,
         std::string fpl_path);

  std::size_t get_cnt_flplns() const noexcept;

  std::shared_ptr<FplnInt> get_fpln_ptr(std::size_t fpln_idx) const noexcept;

  std::shared_ptr<libnav::AwyDB> get_awy_db_ptr() const noexcept;

  std::shared_ptr<libnav::ArptDB> get_arpt_db_ptr() const noexcept;

  std::shared_ptr<libnav::NavaidDB> get_navaid_db_ptr() const noexcept;

  std::string get_fpln_dir() const noexcept;

  std::pair<std::size_t, double> get_sel_leg(bool rt) const noexcept;

  void set_sel_leg(std::pair<std::size_t, double> val, bool rt) noexcept;

  bool get_exec() const noexcept;

  std::size_t get_act_idx() const noexcept;

  std::vector<list_node_ref_t<fpl_seg_t>> get_seg_list(
    std::size_t* sz, std::size_t idx = 0) const noexcept;

  std::vector<list_node_ref_t<leg_list_data_t>> get_leg_list(
    std::size_t* sz, std::size_t idx = 0) const noexcept;

  std::size_t get_nd_seg(nd_leg_data_t* out, std::size_t n_max, 
    std::size_t idx = 0) const noexcept;

  int get_act_leg_idx(std::size_t idx = 0) const noexcept;

  void set_cdu_sel_fpl_idx(std::size_t src, std::size_t sd_idx);

  std::size_t get_cdu_sel_fpl_idx(std::size_t sd_idx) const noexcept;

  void set_nd_mode(NDMode src, std::size_t sd_idx);

  NDMode get_nd_mode(std::size_t sd_idx) const noexcept;

  bool get_ctr(geo::point* out, std::size_t sd_idx) const noexcept;

  geo::point get_ac_pos() const noexcept;

  hdg_info_t get_hdg_info() const noexcept;

  spd_info_t get_spd_info() const noexcept;

  fpln_info_t get_fpl_info(size_t idx = 0) const noexcept;

  act_leg_info_t get_act_leg_info(size_t idx = 0) const noexcept;

  void step_ctr(bool bwd, size_t sd_idx);

  void rte_activate(size_t idx);

  void set_flt_nbr(std::string str);

  std::string get_flt_nbr() const noexcept;

  RTECopySts act_can_copy() const noexcept;

  void copy_act();

  void execute();

  void erase();

  void update();

 private:
  // These are used by commands
  struct pos_data_t {
    // Position:
    double ac_lat_deg;
    double ac_lon_deg;
    double ac_brng_deg;
    double ac_slip_deg;
    double ac_magvar_deg;
    // Speed
    double ac_gs_kts;
    double ac_tas_kts;

    using str_type = typename fms_environment::val_ref_t;

    std::unordered_map<str_type, double*> get_val_pointers();
  };

  std::unordered_map<fms_environment::val_ref_t, double*> double_values_;

  std::shared_ptr<libnav::ArptDB> arpt_db_ptr_;
  std::shared_ptr<libnav::NavaidDB> navaid_db_ptr_;
  std::shared_ptr<libnav::AwyDB> awy_db_ptr_;
  std::shared_ptr<fms_environment::EnvDataRefMap> env_map_ptr_;

  std::vector<std::shared_ptr<FplnInt>> fpl_vec_;

  std::pair<std::size_t, double> leg_sel_cdu_l_;
  std::pair<std::size_t, double> leg_sel_cdu_r_;

  std::string cifp_dir_path_;
  std::string fpl_dir_;

  mutable std::shared_mutex main_mutex_;

  pos_data_t position_;
  std::vector<fpln_data_t> fpl_datas_;

  std::size_t act_rte_idx_;
  double act_rte_id_;
  std::vector<double> copy_ids_;
  std::vector<std::size_t> cdu_rte_idx_;
  std::string flight_ident_;
  std::vector<std::string> fnb_dep_icao_;  // Departure icaos used to reset flight number
  std::vector<NDMode> nd_modes_;
  std::vector<std::size_t> cdu_sel_fpl_;

  bool execute_status_ = false;

  void update_seg_list(std::size_t idx = 0);

  void update_leg_list(std::size_t idx = 0);

  void update_lists(std::size_t idx = 0);

  void update_flt_nbr(std::size_t idx = 0);

  void update_hot_env_vars();
};
}  // namespace test
