#pragma once

#include <cstddef>

#include <memory>
#include <shared_mutex>

#include <libnav/arpt_db.hpp>
#include <libnav/navaid_db.hpp>
#include <libnav/awy_db.hpp>

#include "fpln_base.hpp"
#include "flightpln_int.hpp"
#include <util/pathlib.hpp>
#include <util/util.hpp>

namespace fms_core {

class FlightPlan final {
  mutable std::shared_mutex main_mutex_;
  FplnInt fpln_;
public:
  FlightPlan(util::OpaquePointer<libnav::ArptDB> apt_db,
          util::OpaquePointer<libnav::NavaidDB> nav_db,
          util::OpaquePointer<libnav::AwyDB> aw_db, pathlib::Path cifp_path);

  double get_id() const noexcept;

  std::size_t get_leg_list_sz() const noexcept;

  std::size_t get_seg_list_sz() const noexcept;

  double get_ll_seg(std::size_t start, std::size_t l,
                    std::vector<list_node_ref_t<leg_list_data_t>>* out,
                    int* act_idx_out) noexcept;
  
  double get_sl_seg(std::size_t start, std::size_t l,
                    std::vector<list_node_ref_t<fpl_seg_t>>* out) noexcept;

  bool is_active() const noexcept;

  bool can_activate() const noexcept;

  void activate();

  void deactivate();

  void print_refs() const noexcept;

  // Functions for copying data from 1 flightplan to another:

  void copy_from_other(FlightPlan& other);

  // Import from .fms file:

  libnav::DbErr load_from_fms(const std::string& file_nm, bool set_arpts = true);

  // Export to .fms file:

  void save_to_fms(const std::string& file_nm, bool save_sid_star = true) const;

  std::string get_co_rte_nm() const noexcept;

  // Airport functions:

  libnav::DbErr set_dep(std::string icao); // needs impl

  std::string get_dep_icao() const noexcept;

  libnav::DbErr set_arr(std::string icao); // needs impl

  std::string get_arr_icao();

  // Runway functions:

  std::vector<std::string> get_dep_rwys(bool filter_rwy = false,
                                        bool filter_sid = false) const noexcept;

  std::vector<std::string> get_arr_rwys(bool filter_rwy = false,
                                        bool filter_star = false,
                                        bool is_arr = true) const noexcept;

  bool set_dep_rwy(const std::string& rwy); // needs impl

  std::string get_dep_rwy();

  bool get_dep_rwy_data(libnav::runway_entry_t* out);

  bool set_arr_rwy(std::string& rwy); // needs impl

  std::string get_arr_rwy() const;

  bool get_arr_rwy_data(libnav::runway_entry_t* out);

  // Airport procedure functions:

  std::string get_curr_proc(ProcType tp, bool trans = false);

  std::vector<std::string> get_arpt_proc(ProcType tp, bool is_arr = false,
                                         bool filter_rwy = false,
                                         bool filter_proc = false);

  std::vector<std::string> get_arpt_proc_trans(ProcType tp, bool is_rwy = false,
                                               bool is_arr = false,
                                               bool incl_none = true);

  bool set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr = false); // needs impl

  bool set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr = false); // needs impl

  // Enroute:

  bool add_enrt_seg(timed_ptr_t<seg_list_node_t> next, std::string name); //Needs impl

  // End MUST be an airway id

  bool awy_insert_str(timed_ptr_t<seg_list_node_t> next, std::string end_id); //Needs impl

  bool awy_insert(timed_ptr_t<seg_list_node_t> next, libnav::waypoint_t end);

  bool delete_via(timed_ptr_t<seg_list_node_t> next);

  bool delete_seg_end(timed_ptr_t<seg_list_node_t> next);

  // Leg list interface functions:

  bool dir_from_to(timed_ptr_t<leg_list_node_t> from,
                   timed_ptr_t<leg_list_node_t> to);

  void add_direct(libnav::waypoint_t wpt, timed_ptr_t<leg_list_node_t> next);

  bool delete_leg(timed_ptr_t<leg_list_node_t> next);

  void set_spd_cstr(timed_ptr_t<leg_list_node_t> node, spd_cstr_t cst);

  // This one only sets alt1 for now.

  void set_alt_cstr(timed_ptr_t<leg_list_node_t> node, alt_cstr_t cst);

  // Calculation function

  void update(double hdg_trk_diff);
};
} // namespace fms_core
