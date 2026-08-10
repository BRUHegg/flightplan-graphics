/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        Author: discord/bruh4096#4512

        This file contains declarations of member functions for flightplan
   interface class. This class acts as a layer ontop of the flightplan class.
   Its job is to fetch data from appropriate navigation data bases and store it
   in the flightplan correctly.
*/

#pragma once

#include <memory>
#include <vector>

#include <libnav/awy_db.hpp>
#include <libnav/cifp_parser.hpp>
#include <libnav/common.hpp>
#include <libnav/str_utils.hpp>
#include <util/geom.hpp>

#include "flightplan.hpp"

namespace fms_core {
enum ProcType { PROC_TYPE_SID = 0, PROC_TYPE_STAR = 1, PROC_TYPE_APPCH = 2 };

struct dfms_arr_data_t {
  std::string star, star_trans, arr_rwy, arr_icao;
};

struct spd_cstr_t {
  int magnitude;
  libnav::SpeedMode mode;
};

struct alt_cstr_t {
  int magnitude;
  libnav::AltMode mode;
};

class FplnInt : public FlightPlan {
 public:
  FplnInt(std::shared_ptr<libnav::ArptDB> apt_db,
          std::shared_ptr<libnav::NavaidDB> nav_db,
          std::shared_ptr<libnav::AwyDB> aw_db, std::string cifp_path);

  // Functions for copying data from 1 flightplan to another:

  void copy_from_other(FplnInt& other);

  // Import from .fms file:

  libnav::DbErr load_from_fms(std::string& file_nm, bool set_arpts = true);

  // Export to .fms file:

  void save_to_fms(std::string& file_nm, bool save_sid_star = true) const;

  std::string get_co_rte_nm();

  // Airport functions:

  libnav::DbErr set_dep(std::string icao);

  std::string get_dep_icao();

  libnav::DbErr set_arr(std::string icao);

  std::string get_arr_icao();

  // Runway functions:

  std::vector<std::string> get_dep_rwys(bool filter_rwy = false,
                                        bool filter_sid = false);

  std::vector<std::string> get_arr_rwys(bool filter_rwy = false,
                                        bool filter_star = false,
                                        bool is_arr = true);

  bool set_dep_rwy(std::string& rwy);

  std::string get_dep_rwy();

  bool get_dep_rwy_data(libnav::runway_entry_t* out);

  bool set_arr_rwy(std::string& rwy);

  std::string get_arr_rwy();

  bool get_arr_rwy_data(libnav::runway_entry_t* out);

  // Airport procedure functions:

  std::string get_curr_proc(ProcType tp, bool trans = false);

  std::vector<std::string> get_arpt_proc(ProcType tp, bool is_arr = false,
                                         bool filter_rwy = false,
                                         bool filter_proc = false);

  std::vector<std::string> get_arpt_proc_trans(ProcType tp, bool is_rwy = false,
                                               bool is_arr = false,
                                               bool incl_none = true);

  bool set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr = false);

  bool set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr = false);

  // Enroute:

  bool add_enrt_seg(timed_ptr_t<seg_list_node_t> next, std::string name);

  // End MUST be an airway id

  bool awy_insert_str(timed_ptr_t<seg_list_node_t> next, std::string end_id);

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

 private:
  mutable std::string co_rte_nm_;
  std::string arr_rwy_;
  bool appr_is_rwy_;

  std::vector<libnav::str_umap_t> proc_db_;
  std::shared_ptr<libnav::AwyDB> awy_db_;
  std::shared_ptr<libnav::NavaidDB> navaid_db_;

  libnav::arinc_rwy_db_t dep_rnw_, arr_rnw_;
  bool has_dep_rnw_data_, has_arr_rnw_data_;
  libnav::runway_entry_t dep_rnw_data_, arr_rnw_data_;

  double fpl_id_calc_;

  bool airac_mismatch_;

  // Static member functions:

  static size_t get_proc_db_idx(ProcType tp, bool is_arr = false);

  static FplSegment get_proc_tp(ProcType tp);

  static FplSegment get_trans_tp(ProcType tp);

  static std::vector<std::string> get_proc(libnav::str_umap_t& db,
                                           std::string rw = "");

  static std::vector<std::string> get_apprs(libnav::str_umap_t& proc_db,
                                            libnav::str_umap_t& appr_db,
                                            std::string proc,
                                            bool filter = false);

  static std::vector<std::string> get_proc_trans(std::string proc,
                                                 libnav::str_umap_t& db,
                                                 libnav::arinc_rwy_db_t& rwy_db,
                                                 bool is_rwy = false,
                                                 bool incl_none = true);

  static std::string get_dfms_enrt_leg(const leg_list_node_t* lg,
                                       bool force_dir = false);

  // Non-static member functions:

  bool is_apt_valid(libnav::Airport* ptr) const;

  void update_act_leg();

  // Auxiliury functions for import from .fms:

  /*
      Function: process_dfms_term_line
      Description:
      Processes a single line of .fms file describing airports/procedures
      @param l_split: reference to the target split line
      @return error code
  */

  libnav::DbErr process_dfms_proc_line(std::vector<std::string>& l_split,
                                       bool set_arpts,
                                       dfms_arr_data_t* arr_data);

  libnav::DbErr set_dfms_arr_data(dfms_arr_data_t* arr_data, bool set_arpt);

  bool get_dfms_wpt(std::vector<std::string>& l_split, libnav::waypoint_t* out);

  // Auxiliury functions for export to .fms:

  std::string get_dfms_arpt_leg(bool is_arr = false) const;

  size_t get_dfms_enrt_legs(std::vector<std::string>* out) const;

  // The main .fms import function:

  libnav::DbErr load_fms_fpln(std::string& file_nm, bool set_arpts = true);

  // Other auxiliury functions:

  /*
      Function: adjust_list_pointers
      Description:
      Adjusts pointers inside leg_data_list and seg_list after copying from
     another flightplan.

      @param other: reference to other flight plan object
  */

  void adjust_list_pointers(FplnInt& other);

  void copy_act_leg(FplnInt& other);

  void update_apt_dbs(bool arr = false);

  libnav::arinc_rwy_data_t get_rwy_data(std::string nm, bool is_arr = false);

  std::string get_curr_proc_imp(ProcType tp, bool trans = false);

  bool add_fpl_seg(libnav::arinc_leg_seq_t& legs, FplSegment seg_tp,
                   std::string ref_nm, std::string seg_nm = "",
                   seg_list_node_t* next = nullptr, bool set_ref = true);

  /*
      Function: get_awy_tf_leg
      Description:
      Makes a TF leg using a waypoint id taken from airway data base.
      @param wpt_id: id of the waypoint taken from airway data base. MUST be a
     valid id
      @return: arinc424 TF leg
  */

  leg_t get_awy_tf_leg(libnav::awy_point_t awy_pt);

  void add_awy_seg(std::string awy, seg_list_node_t* next,
                   std::vector<libnav::awy_point_t>& awy_pts);

  bool set_sid_star(std::string proc_nm, bool is_star = false,
                    bool reset_rwy = true);

  bool set_appch_legs(std::string appch, std::string& arr_rwy,
                      libnav::arinc_leg_seq_t legs, std::string appch_seg = "");

  bool set_appch(std::string appch);

  bool add_trans_legs(ProcType tp, std::string trans,
                      libnav::arinc_leg_seq_t& pr_legs,
                      libnav::arinc_leg_seq_t& tr_legs);

  bool set_proc_trans(ProcType tp, std::string trans, bool is_arr = false);

  // Calculation functions:

  double get_leg_mag_var_deg(leg_list_node_t* leg);

  double get_leg_turn_rad(leg_list_node_t* curr);

  static bool get_df_start(leg_seg_t curr_seg, leg_t next, geo::point* out);

  /*
      Function: get_to_leg_start
      Description:
      Calculates start of a leg that can be offset by a turn(see TURN_OFFS_LEGS)
      @param curr_seg: current segment
      @param next: next arinc424 leg
      @param out: pointer to where the output should be stored
  */

  static void get_to_leg_start(leg_seg_t curr_seg, leg_t next,
                               double mag_var_deg, double hdg_trk_diff,
                               geo::point* out);

  static bool get_cf_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next,
                               double mag_var_deg, geo::point* out,
                               bool* to_inh, double* turn_radius_out);

  /*
      Function: get_leg_start
      Description:
      Calculates start of next leg.
      @param curr_seg: current segment
      @param curr_leg: current arinc424 leg
      @param next: next arinc424 leg
      @param out: pointer to where the output should be stored
      @param to_inh: set to true when turn offset is inhibited
      (90 degree and more turns). Otherwise not set
      @param turn_radius_nm: where to output turn radius if required
  */

  bool get_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next,
                     double mag_var_deg, double hdg_trk_diff, geo::point* out,
                     bool* to_inh, double* turn_radius_nm);

  static void set_xi_leg(leg_list_node_t* leg);

  /*
      Function: set_turn_offset
      Description:
      Offsets the end of previous leg so that a turn without overshoot is
     possible.
      @param leg: pointer to a node of leg list
      @param prev_leg: non-bypassed leg before leg. THIS IS IMPORTANT:
      IT MUST NOT BE BYPASSED.
  */

  static void set_turn_offset(leg_list_node_t* leg, leg_list_node_t* prev_leg);

  // The following functions are used to calculate ends of arinc424 legs.

  /*
      Function: calculate_alt_leg
      Description:
      Calculates end of CA or VA leg
      @param leg: pointer to a node of leg list
      @param hdg_trk_diff: difference between heading and track in radians
  */

  void calculate_alt_leg(leg_list_node_t* leg, double hdg_trk_diff,
                         double curr_alt_ft);

  /*
      Function: calculate_alt_leg
      Description:
      Calculates end of CI or VI leg
      @param leg: pointer to a node of leg list
      @param hdg_trk_diff: difference between heading and track in radians
  */

  void calculate_intc_leg(leg_list_node_t* leg, double hdg_trk_diff);

  void calculate_fc_leg(leg_list_node_t* leg);

  /*
      Function: calculate_dme_leg
      Description:
      Calculates end of CD, FD or VD leg
      @param leg: pointer to a node of leg list
      @param hdg_trk_diff: difference between heading and track in radians
  */

  void calculate_dme_leg(leg_list_node_t* leg, double hdg_trk_diff);

  /*
      Function: calculate_alt_leg
      Description:
      Calculates end of CF/TF/DF leg
      @param leg: pointer to a node of leg list
  */

  void calculate_crs_trk_dir_leg(leg_list_node_t* leg);

  /*
      Function: calculate_leg
      Description:
      Handles all supported leg types.
      @param leg: pointer to a node of leg list
      @param hdg_trk_diff: difference between heading and track in radians
  */

  void calculate_leg(leg_list_node_t* leg, double hdg_trk_diff,
                     double curr_alt_ft);
};
}  // namespace test
