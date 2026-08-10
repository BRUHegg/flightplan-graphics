/*
    This project is licensed under
    Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

    A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

    Author: discord/bruh4096#4512

    This file contains definitions of member functions for flightplan interface
   class. This class acts as a layer ontop of the flightplan class. Its job is
   to fetch data from appropriate navigation data bases and store it in the
   flightplan correctly.
*/

#include "flightpln_int.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <set>
#include <string>
#include <vector>

#include <libnav/arpt_db.hpp>
#include <libnav/cifp_parser.hpp>
#include <libnav/geo_utils.hpp>
#include <libnav/navaid_db.hpp>
#include <libnav/str_utils.hpp>
#include <util/geom.hpp>

namespace {

constexpr size_t N_PROC_DB_SZ = 5;
constexpr size_t N_ARR_DB_OFFSET = 2;
constexpr size_t N_DFMS_ENRT_WORDS = 6;
constexpr double DEFAULT_VS_FPM = 2000;
constexpr double DEFAULT_GS_KTS = 250;
constexpr double CLB_RATE_FT_PER_NM = 500;
constexpr double TURN_RADIUS_NM = 1;  // Untill there is a VNAV
constexpr double ASSUMED_RNP_PROC_NM = 1;
constexpr double ASSUMED_RNP_ENRT_NM = 3;
constexpr double CF_STRAIGHT_DEV_RAD = (5 * geo::DEG_TO_RAD);
const std::string MISSED_APPR_SEG_NM = "MISSED APPRCH";
const std::string INTC_LEG_NM = "(INTC)";
// X-Plane .fms format stuff
constexpr char DFMS_COL_SEP = ' ';
constexpr uint8_t N_DFMS_OUT_PREC = 6;
const std::string DFMS_PADDING = "I\n1100 Version\n";
const std::string DFMS_RWY_PREFIX = "RW";
// .fms row headers:
const std::string DFMS_AIRAC_CYCLE_NM = "CYCLE";
const std::string DFMS_DEP_NM = "ADEP";
const std::string DFMS_DEP_RWY_NM = "DEPRWY";
const std::string DFMS_SID_NM = "SID";
const std::string DFMS_SID_TRANS_NM = "SIDTRANS";
const std::string DFMS_ARR_NM = "ADES";
const std::string DFMS_ARR_RWY_NM = "DESRWY";
const std::string DFMS_STAR_NM = "STAR";
const std::string DFMS_STAR_TRANS_NM = "STARTRANS";
const std::string DFMS_N_ENRT_NM = "NUMENR";

const std::string DFMS_DIR_SEG_NM = "DRCT";

const std::string DFMS_FILE_POSTFIX = ".fms";

const std::set<std::string> NOT_FOLLOWED_BY_DF = {"AF", "CI", "PI", "RF", "VI"};
const std::set<std::string> AFTER_INTC = {"AF", "CF", "FA", "FC",
                                          "FD", "FM", "IF"};
// The following set contains legs that allow to be offset by a turn(onto
/// the current leg)
const std::set<std::string> TURN_OFFS_LEGS = {"DF", "CI", "CA", "CD", "CR",
                                              "VA", "VI", "VR", "VD"};
const std::set<std::string> LEGS_CALC = {"DF", "TF", "CF", "VA", "CA", "FA",
                                         "VI", "CI", "FD", "CD", "VD"};
// const std::map<std::string, std::set<std::string>> ILLEGAL_NEXT_LEG = {
//     {"AF", {"DF", "IF", "PI"}},
//     {"CA", {"AF", "HA", "HF", "HM", "PI", "RF", "TF"}},
//     {"CD", {"HA", "HF", "HM", "PI", "RF", "TF"}},
//     {"CF", {"IF"}},
//     {"CI", {"CA", "CD", "CD", "CR", "DF", "HA", "HF", "HM", "PI", "RF",
//     "TF", "VA", "VD", "VI", "VM", "VR"}},
//     {}
// };

/*
    General info:
    turns:
    positive - right turn
    negative - left turn
*/

bool is_ang_greater(double ang1_rad, double ang2_rad);  // true if ang1 > ang2

double get_turn_rad(double ang1, geo::point p1, double ang2, geo::point p2);

double get_turn_by_dir(double curr_brng_rad, double tgt_brng_rad,
                       libnav::TurnDir t_dir);

double get_cf_big_turn_isect(fms_core::leg_seg_t curr, fms_core::leg_t next, double m_var,
                             geo::point* out);

std::string get_appr_rwy(std::string& appr);

std::string get_dfms_rwy(std::string& rwy_nm);

geo::point get_xa_end_point(geo::point prev, float brng_deg, float va_alt_ft,
                            double clb_ft_nm = CLB_RATE_FT_PER_NM);

geo::point get_dme_end_point(geo::point start, double true_brng_rad,
                             geo::point st, double dist_nm);

libnav::waypoint_t get_ca_va_wpt(geo::point pos, int n_ft);

libnav::waypoint_t get_xd_wpt(geo::point pos, std::string main_nm, int dme_nm);

double get_rnp(fms_core::leg_list_node_t* leg);

bool is_ang_greater(double ang1_rad, double ang2_rad) {
  if (ang1_rad < 0) {
    ang1_rad += 2 * M_PI;
  }
  if (ang2_rad < 0) {
    ang2_rad += 2 * M_PI;
  }

  if (ang1_rad - ang2_rad > M_PI) {
    ang2_rad += 2 * M_PI;
  } else if (ang2_rad - ang1_rad > M_PI) {
    ang1_rad += 2 * M_PI;
  }

  return ang1_rad > ang2_rad;
}

double get_turn_rad(double ang1, geo::point p1, double ang2, geo::point p2) {
  double brng_12 = p1.get_gc_bearing_rad(p2);
  if (ang1 < 0) ang1 += 2 * M_PI;
  if (ang2 < 0) ang2 += 2 * M_PI;
  if (brng_12 < 0) brng_12 += 2 * M_PI;

  bool left_turn;
  if (ang1 == brng_12)
    left_turn = is_ang_greater(ang1, ang2);
  else
    left_turn = is_ang_greater(ang1, brng_12);

  double turn_rad = ang2 - ang1;

  if (left_turn && turn_rad > 0)
    turn_rad -= 2 * M_PI;
  else if (!left_turn && turn_rad < 0)
    turn_rad += 2 * M_PI;

  return turn_rad;
}

double get_turn_by_dir(double curr_brng_rad, double tgt_brng_rad,
                       libnav::TurnDir t_dir) {
  if (curr_brng_rad < 0) {
    curr_brng_rad += 2 * M_PI;
  }
  if (tgt_brng_rad < 0) {
    tgt_brng_rad += 2 * M_PI;
  }
  double turn_rad = tgt_brng_rad - curr_brng_rad;
  if (t_dir == libnav::TurnDir::LEFT && turn_rad > 0) {
    turn_rad -= 2 * M_PI;
  } else if (t_dir == libnav::TurnDir::RIGHT && turn_rad < 0) {
    turn_rad += 2 * M_PI;
  } else if (t_dir == libnav::TurnDir::EITHER) {
    if (turn_rad > M_PI) {
      turn_rad -= 2 * M_PI;
    } else if (turn_rad < -M_PI) {
      turn_rad += 2 * M_PI;
    }
  }

  return turn_rad;
}

double get_cf_big_turn_isect(fms_core::leg_seg_t curr, fms_core::leg_t next, double m_var,
                             geo::point* out) {
  assert(next.has_main_fix);
  geo::point next_main_pos = next.main_fix.data.pos;

  geom::vect2_t mp_proj = {0, 0};
  geom::vect2_t cs_proj = geom::project_point(curr.start, next_main_pos);
  geom::vect2_t ce_proj = geom::project_point(curr.end, next_main_pos);
  double next_inbd = double(next.outbd_crs_deg) * geo::DEG_TO_RAD + M_PI;
  if (!next.outbd_crs_true) next_inbd -= m_var;
  geom::vect2_t next_dir = {sin(next_inbd), cos(next_inbd)};

  geom::vect2_t out_proj;
  double turn_rad_nm =
      geom::get_turn_isect_smpl(cs_proj, ce_proj, mp_proj, next_dir, &out_proj);

  double brng_rad = atan2(out_proj.x, out_proj.y);
  double dist_nm = out_proj.absval();

  *out = geo::get_pos_from_brng_dist(next_main_pos, brng_rad, dist_nm);
  return turn_rad_nm;
}

std::string get_appr_rwy(std::string& appr) {
  std::string rw;

  for (size_t i = 0; i < appr.size(); i++) {
    if (rw.size() < 2 && appr[i] >= '0' && appr[i] <= '9') {
      rw.push_back(appr[i]);
    } else if (rw.size() &&
               (appr[i] == 'L' || appr[i] == 'R' || appr[i] == 'C')) {
      rw.push_back(appr[i]);
      break;
    } else if (rw.size()) {
      break;
    }
  }

  return strutils::normalize_rnw_id(rw);
}

std::string get_dfms_rwy(std::string& rwy_nm) {
  if (rwy_nm[0] == 'R' && rwy_nm[1] == 'W') {
    std::string ret = rwy_nm.substr(2, rwy_nm.size() - 2);

    return ret;
  }
  return "";
}

geo::point get_xa_end_point(geo::point prev, float brng_deg, float va_alt_ft,
                            double clb_ft_nm) {
  double clb_nm = va_alt_ft / clb_ft_nm;

  geo::point curr =
      geo::get_pos_from_brng_dist(prev, brng_deg * geo::DEG_TO_RAD, clb_nm);

  return curr;
}

geo::point get_dme_end_point(geo::point start, double true_brng_rad,
                             geo::point st, double dist_nm) {
  geom::vect2_t start_proj = {0, 0};
  geom::vect2_t st_proj = geom::project_point(st, start);

  geom::vect2_t vec = {sin(true_brng_rad), cos(true_brng_rad)};

  double dist_end =
      geom::get_vect_circ_isect_dist(start_proj, st_proj, vec, dist_nm);

  if (dist_end == -1) {
    return start;
  } else {
    return geo::get_pos_from_brng_dist(start, true_brng_rad, dist_end);
  }
}

libnav::waypoint_t get_ca_va_wpt(geo::point pos, int n_ft) {
  libnav::waypoint_t out = {};
  out.id = "(" + std::to_string(n_ft) + ")";
  out.data.pos = pos;
  out.data.arinc_type = 0;
  out.data.area_code = "";
  out.data.country_code = "";
  out.data.type = libnav::NavaidType::WAYPOINT;
  out.data.navaid = nullptr;

  return out;
}

libnav::waypoint_t get_xd_wpt(geo::point pos, std::string main_nm, int dme_nm) {
  libnav::waypoint_t out = {};
  out.id = "(" + main_nm + "/" + std::to_string(dme_nm) + ")";
  out.data.pos = pos;
  out.data.arinc_type = 0;
  out.data.area_code = "";
  out.data.country_code = "";
  out.data.type = libnav::NavaidType::WAYPOINT;
  out.data.navaid = nullptr;

  return out;
}

double get_rnp(fms_core::leg_list_node_t* leg) {
  if (leg->data.leg.rnp != 0) return double(leg->data.leg.rnp);

  fms_core::FplSegment seg_tp = leg->data.seg->data.seg_type;

  if (seg_tp != fms_core::FplSegment::ENRT) return ASSUMED_RNP_PROC_NM;

  return ASSUMED_RNP_ENRT_NM;
}
} // namespace

namespace fms_core {
// FplnInt member functions:
// Public functions:

FplnInt::FplnInt(std::shared_ptr<libnav::ArptDB> apt_db,
                 std::shared_ptr<libnav::NavaidDB> nav_db,
                 std::shared_ptr<libnav::AwyDB> aw_db, std::string cifp_path)
    : FlightPlan(apt_db, nav_db, cifp_path) {
  awy_db_ = aw_db;
  navaid_db_ = nav_db;
  proc_db_.resize(N_PROC_DB_SZ);

  fpl_id_calc_ = 0;

  has_dep_rnw_data_ = false;
  has_arr_rnw_data_ = false;

  co_rte_nm_ = "";

  arr_rwy_ = "";
  appr_is_rwy_ = false;

  airac_mismatch_ = false;
  if (aw_db->get_airac() != fix_airac_version_) airac_mismatch_ = true;
}

void FplnInt::copy_from_other(FplnInt& other) {
  std::unique_lock lock(fpl_mtx_);
  if (other.departure_ != nullptr) {
    if (departure_ == nullptr ||
        departure_->icao_code != other.departure_->icao_code) {
      delete departure_;
      departure_ = nullptr;
      departure_ = new libnav::Airport(*other.departure_, departure_legs_);
      update_apt_dbs();
    }
  }

  if (other.arrival_ != nullptr) {
    if (arrival_ == nullptr || arrival_->icao_code != other.arrival_->icao_code) {
      delete arrival_;
      arrival_ = nullptr;
      arrival_ = new libnav::Airport(*other.arrival_);
      update_apt_dbs(true);
    }
  }

  for (std::size_t i = 1; i < other.fpl_refs_.size(); i++) {
    fpl_refs_[i] = other.fpl_refs_[i];
    if (fpl_refs_[i].ptr != nullptr) {
      fpl_refs_[i].ptr =
          fpl_refs_[i].ptr - other.seg_stack_.nodes + seg_stack_.nodes;
    }
  }

  struct_util::copy_list(other.seg_stack_, seg_stack_, other.seg_list_, seg_list_);
  struct_util::copy_list(other.leg_data_stack_, leg_data_stack_, other.leg_list_,
                         leg_list_);

  adjust_list_pointers(other);

  copy_act_leg(other);

  co_rte_nm_ = other.co_rte_nm_;

  arr_rwy_ = other.arr_rwy_;
  appr_is_rwy_ = other.appr_is_rwy_;

  has_dep_rnw_data_ = other.has_dep_rnw_data_;
  has_arr_rnw_data_ = other.has_arr_rnw_data_;

  dep_rnw_data_ = other.dep_rnw_data_;
  arr_rnw_data_ = other.arr_rnw_data_;

  update_id();
}

libnav::DbErr FplnInt::load_from_fms(std::string& file_nm, bool set_arpts) {
  libnav::DbErr out = load_fms_fpln(file_nm, set_arpts);
  if (out != libnav::DbErr::SUCCESS) {
    reset_fpln();
  } else {
    co_rte_nm_ = departure_->icao_code + arrival_->icao_code;
  }
  return out;
}

void FplnInt::save_to_fms(std::string& file_nm, bool save_sid_star) const {
  std::unique_lock lock(fpl_mtx_);
  if (!is_apt_valid(departure_) || !is_apt_valid(arrival_)) {
    return;
  }

  co_rte_nm_ = departure_->icao_code + arrival_->icao_code;

  std::ofstream out(file_nm + DFMS_FILE_POSTFIX, std::ofstream::out);

  out << DFMS_PADDING;
  std::string curr_cycle = std::to_string(navaid_db_->get_wpt_cycle());
  out << DFMS_AIRAC_CYCLE_NM + DFMS_COL_SEP + curr_cycle + "\n";

  out << DFMS_DEP_NM << DFMS_COL_SEP << departure_->icao_code << "\n";
  std::size_t dep_rwy_seg_idx = static_cast<std::size_t>(
    FplSegment::DEP_RWY);
  std::string dep_rwy = fpl_refs_[dep_rwy_seg_idx].name;

  if (dep_rwy != "") {
    out << DFMS_DEP_RWY_NM << DFMS_COL_SEP << DFMS_RWY_PREFIX + dep_rwy << "\n";
  }

  if (save_sid_star) {
    std::size_t sid_seg_idx = static_cast<std::size_t>(
        FplSegment::SID);
    std::size_t sid_trans_seg_idx = static_cast<std::size_t>(
        FplSegment::SID_TRANS);
    std::string sid_name = fpl_refs_[sid_seg_idx].name;
    std::string sid_trans_name = fpl_refs_[sid_trans_seg_idx].name;

    if (sid_name != "") {
      out << DFMS_SID_NM << DFMS_COL_SEP << sid_name << "\n";

      if (sid_trans_name != "") {
        out << DFMS_SID_TRANS_NM << DFMS_COL_SEP << sid_trans_name << "\n";
      }
    }
  }

  out << DFMS_ARR_NM << DFMS_COL_SEP << arrival_->icao_code << "\n";

  if (arr_rwy_ != "") {
    out << DFMS_ARR_RWY_NM << DFMS_COL_SEP << DFMS_RWY_PREFIX + arr_rwy_ << "\n";
  }

  if (save_sid_star) {
    std::size_t star_seg_idx = static_cast<std::size_t>(
        FplSegment::STAR);
    std::size_t star_trans_seg_idx = static_cast<std::size_t>(
        FplSegment::STAR_TRANS);
    std::string star_name = fpl_refs_[star_seg_idx].name;
    std::string star_trans_name = fpl_refs_[star_trans_seg_idx].name;

    if (star_name != "") {
      out << DFMS_STAR_NM << DFMS_COL_SEP << star_name << "\n";

      if (star_trans_name != "") {
        out << DFMS_STAR_TRANS_NM << DFMS_COL_SEP << star_trans_name << "\n";
      }
    }
  }

  std::vector<std::string> vec;
  size_t n_legs = get_dfms_enrt_legs(&vec);

  out << DFMS_N_ENRT_NM << DFMS_COL_SEP << std::to_string(int(n_legs)) << "\n";

  for (size_t i = 0; i < n_legs; i++) {
    out << vec[i] << "\n";
  }

  out.close();
}

std::string FplnInt::get_co_rte_nm() {
  std::shared_lock lock(fpl_mtx_);
  return co_rte_nm_;
}

libnav::DbErr FplnInt::set_dep(std::string icao) {
  std::unique_lock lock(fpl_mtx_);
  libnav::DbErr out = set_arpt(icao, &departure_, false, departure_legs_);
  if (departure_ != nullptr && departure_->icao_code == icao &&
      out != libnav::DbErr::ERR_NONE) {
    update_apt_dbs();

    // If there is an arrival and departure was changed, clear arrival data
    if (arrival_ != nullptr) {
      arr_rwy_ = "";
      has_arr_rnw_data_ = false;
      arr_rnw_.clear();
      proc_db_[N_ARR_DB_OFFSET + PROC_TYPE_STAR].clear();
      proc_db_[N_ARR_DB_OFFSET + PROC_TYPE_APPCH].clear();
      delete arrival_;
      arrival_ = nullptr;
    }
  }

  return out;
}

std::string FplnInt::get_dep_icao() {
  std::shared_lock lock(fpl_mtx_);
  if (departure_ != nullptr) return departure_->icao_code;
  return "";
}

libnav::DbErr FplnInt::set_arr(std::string icao) {
  std::shared_lock lock(fpl_mtx_);
  if (departure_ == nullptr) {
    return libnav::DbErr::ERR_NONE;
  }

  libnav::DbErr err = set_arpt(icao, &arrival_, true, arrival_legs_);
  if (err != libnav::DbErr::ERR_NONE) {
    arr_rwy_ = "";
    has_arr_rnw_data_ = false;

    if (arrival_ != nullptr) {
      update_apt_dbs(true);
    }
  }
  return err;
}

std::string FplnInt::get_arr_icao() {
  std::shared_lock lock(fpl_mtx_);
  if (arrival_ != nullptr) return arrival_->icao_code;
  return "";
}

std::vector<std::string> FplnInt::get_dep_rwys(bool filter_rwy,
                                               bool filter_sid) {
  std::shared_lock lock(fpl_mtx_);
  std::vector<std::string> out = {};

  std::string curr_rwy = get_ref_for(FplSegment::DEP_RWY).name;
  if (filter_sid && curr_rwy != "") {
    out.push_back(curr_rwy);
  } else {
    std::string curr_sid = get_ref_for(FplSegment::SID).name;
    size_t db_idx = get_proc_db_idx(PROC_TYPE_SID, false);

    for (auto i : dep_rnw_) {
      if (filter_rwy && curr_sid != "" &&
          proc_db_[db_idx][curr_sid].find(i.first) ==
              proc_db_[db_idx][curr_sid].end()) {
        continue;
      }
      out.push_back(i.first);
    }
  }
  return out;
}

std::vector<std::string> FplnInt::get_arr_rwys(bool filter_rwy,
                                               bool filter_star, bool is_arr) {
  std::shared_lock lock(fpl_mtx_);
  std::vector<std::string> out = {};

  std::string curr_rwy = arr_rwy_;
  if (filter_star && curr_rwy != "") {
    out.push_back(curr_rwy);
  } else {
    std::string curr_star = get_ref_for(FplSegment::STAR).name;
    size_t db_idx = get_proc_db_idx(PROC_TYPE_STAR, is_arr);

    for (auto i : arr_rnw_) {
      if (filter_rwy && curr_star != "" &&
          proc_db_[db_idx][curr_star].find(i.first) ==
              proc_db_[db_idx][curr_star].end()) {
        continue;
      }
      out.push_back(i.first);
    }
  }
  return out;
}

bool FplnInt::set_dep_rwy(std::string& rwy) {
  std::unique_lock lock(fpl_mtx_);

  if (dep_rnw_.find(rwy) != dep_rnw_.end()) {
    std::string curr_rwy = get_ref_for(FplSegment::DEP_RWY).name;
    if (rwy != curr_rwy) {
      int data_found =
          arpt_db_ptr_->get_rnw_data(departure_->icao_code, rwy, &dep_rnw_data_);
      has_dep_rnw_data_ = data_found;
      if (!data_found) {
        has_dep_rnw_data_ = false;
        dep_rnw_data_ = {};
      }

      std::string curr_sid = get_ref_for(FplSegment::SID).name;
      std::string curr_trans = get_ref_for(FplSegment::SID_TRANS).name;
      delete_ref(FplSegment::SID_TRANS);
      delete_ref(FplSegment::SID);

      libnav::arinc_rwy_data_t rwy_data = dep_rnw_[rwy];

      leg_t ins_leg{};
      ins_leg.leg_type = "IF";
      libnav::waypoint_t rnw_wpt = {};
      rnw_wpt.id = rwy;
      rnw_wpt.data.pos = rwy_data.pos;
      rnw_wpt.data.area_code = departure_->icao_code;
      rnw_wpt.data.type = libnav::NavaidType::RWY;
      ins_leg.set_main_fix(rnw_wpt);

      std::vector<leg_t> legs = {};

      add_legs(ins_leg, legs, FplSegment::DEP_RWY, rwy);
      get_ref_for(FplSegment::DEP_RWY).name = rwy;

      set_sid_star(curr_sid);
      set_proc_trans(PROC_TYPE_SID, curr_trans, false);
    }

    return true;
  }

  return false;
}

std::string FplnInt::get_dep_rwy() {
  std::shared_lock lock(fpl_mtx_);
  if (departure_ != nullptr && get_ref_for(
    FplSegment::DEP_RWY).ptr != nullptr)
    return get_ref_for(FplSegment::DEP_RWY).name;
  return "";
}

bool FplnInt::get_dep_rwy_data(libnav::runway_entry_t* out) {
  std::shared_lock lock(fpl_mtx_);
  if (has_dep_rnw_data_) {
    *out = dep_rnw_data_;
    return true;
  }

  return false;
}

bool FplnInt::set_arr_rwy(std::string& rwy) {
  std::unique_lock lock(fpl_mtx_);

  if (arr_rnw_.find(rwy) != arr_rnw_.end()) {
    if (arr_rwy_ != rwy) {
      int data_found =
          arpt_db_ptr_->get_rnw_data(arrival_->icao_code, rwy, &arr_rnw_data_);
      has_arr_rnw_data_ = true;
      if (!data_found) {
        has_arr_rnw_data_ = false;
        arr_rnw_data_ = {};
      }

      arr_rwy_ = rwy;

      libnav::arinc_rwy_data_t rwy_data = arr_rnw_[rwy];
      libnav::waypoint_t rwy_wpt = {arr_rwy_,
                                    {libnav::NavaidType::RWY, 0, rwy_data.pos,
                                     arrival_->icao_code, "", nullptr}};
      leg_t rwy_leg{};
      rwy_leg.leg_type = "TF";
      rwy_leg.set_main_fix(rwy_wpt);

      libnav::arinc_leg_seq_t legs = {};

      delete_ref(FplSegment::APPCH_TRANS);

      set_sid_star(get_ref_for(FplSegment::STAR).name, true, false);

      add_legs(rwy_leg, legs, FplSegment::APPCH, arr_rwy_);
      get_ref_for(FplSegment::APPCH).name = arr_rwy_;
      appr_is_rwy_ = true;
    }

    return true;
  }

  return false;
}

std::string FplnInt::get_arr_rwy() {
  std::shared_lock lock(fpl_mtx_);
  return arr_rwy_;
}

bool FplnInt::get_arr_rwy_data(libnav::runway_entry_t* out) {
  std::shared_lock lock(fpl_mtx_);
  if (has_arr_rnw_data_) {
    *out = arr_rnw_data_;
    return true;
  }

  return false;
}

std::string FplnInt::get_curr_proc(ProcType tp, bool trans) {
  std::shared_lock lock(fpl_mtx_);
  return get_curr_proc_imp(tp, trans);
}

std::vector<std::string> FplnInt::get_arpt_proc(ProcType tp, bool is_arr,
                                                bool filter_rwy,
                                                bool filter_proc) {
  std::shared_lock lock(fpl_mtx_);
  FplSegment s_tp = get_proc_tp(tp);
  size_t tp_idx = size_t(s_tp);

  if (tp != PROC_TYPE_APPCH && filter_rwy && fpl_refs_[tp_idx].name != "") {
    return {fpl_refs_[tp_idx].name};
  } else if (tp == PROC_TYPE_APPCH && filter_proc &&
             fpl_refs_[tp_idx].name != "") {
    if (appr_is_rwy_)
      return {};
    else
      return {fpl_refs_[tp_idx].name};
  }

  size_t db_idx = get_proc_db_idx(tp, is_arr);

  if (db_idx != N_PROC_DB_SZ) {
    std::string rwy = "";

    if (filter_proc) {
      if (is_arr) {
        rwy = arr_rwy_;
      } else {
        rwy = get_ref_for(FplSegment::DEP_RWY).name;
      }
    }

    if (tp == PROC_TYPE_APPCH) {
      std::string star_nm = "";
      if (filter_rwy) star_nm = get_ref_for(FplSegment::STAR).name;
      size_t star_idx = get_proc_db_idx(PROC_TYPE_STAR, is_arr);
      return get_apprs(proc_db_[star_idx], proc_db_[db_idx], star_nm, filter_rwy);
    } else {
      return get_proc(proc_db_[db_idx], rwy);
    }
  }

  return {};
}

std::vector<std::string> FplnInt::get_arpt_proc_trans(ProcType tp, bool is_rwy,
                                                      bool is_arr,
                                                      bool incl_none) {
  std::shared_lock lock(fpl_mtx_);

  if (tp == PROC_TYPE_APPCH && appr_is_rwy_) return {};

  size_t ref_idx = size_t(get_proc_tp(tp));
  std::string proc_name = fpl_refs_[ref_idx].name;
  if (proc_name != "") {
    size_t db_idx = get_proc_db_idx(tp, is_arr);
    if (is_arr) {
      return get_proc_trans(proc_name, proc_db_[db_idx], arr_rnw_, is_rwy,
                            incl_none);
    }
    return get_proc_trans(proc_name, proc_db_[db_idx], dep_rnw_, is_rwy,
                          incl_none);
  }
  return {};
}

bool FplnInt::set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr) {
  std::unique_lock lock(fpl_mtx_);

  size_t db_idx = get_proc_db_idx(tp, is_arr);

  std::string curr_proc = get_curr_proc_imp(tp);
  if (proc_nm == curr_proc) return false;

  switch (db_idx) {
    case PROC_TYPE_SID:
      return set_sid_star(proc_nm);
    case PROC_TYPE_STAR + N_ARR_DB_OFFSET:
      return set_sid_star(proc_nm, true);
    case PROC_TYPE_APPCH + N_ARR_DB_OFFSET:
      return set_appch(proc_nm);
    default:
      return false;
  }

  return false;
}

bool FplnInt::set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr) {
  std::unique_lock lock(fpl_mtx_);

  std::string curr_trans = get_curr_proc_imp(tp, true);
  if (trans == curr_trans) return false;

  return set_proc_trans(tp, trans, is_arr);
}

bool FplnInt::add_enrt_seg(timed_ptr_t<seg_list_node_t> next,
                           std::string name) {
  std::unique_lock lock(fpl_mtx_);

  if (next.id == seg_list_.id && next.ptr != &(seg_list_.head)) {
    if (next.ptr == nullptr) {
      seg_list_node_t* prev = seg_list_.tail.prev;
      leg_list_node_t* end_leg = prev->data.end;

      if (prev->data.seg_type <= FplSegment::ENRT && 
        prev != &(seg_list_.head)) {
        bool add_seg = false;

        if (end_leg != nullptr) {
          libnav::waypoint_t end_fix = end_leg->data.leg.main_fix;
          if (end_fix.data.area_code == "ENRT") {
            std::string end_leg_awy_id = end_fix.get_awy_id();
            add_seg = awy_db_->is_in_awy(name, end_leg_awy_id);
          }
        } else if (prev->prev->data.seg_type > FplSegment::DEP_RWY) {
          leg_list_node_t* base_end_leg = prev->prev->data.end;
          if (base_end_leg != nullptr) {
            std::string base_awy_id =
                base_end_leg->data.leg.main_fix.get_awy_id();
            std::vector<libnav::awy_point_t> awy_pts;
            size_t n_pts = awy_db_->get_aa_path(prev->data.name, base_awy_id,
                                               name, &awy_pts);

            if (n_pts) {
              delete_segment(prev);
              add_awy_seg(prev->data.name, next.ptr, awy_pts);
              add_seg = true;
            }
          }
        }
        if (add_seg) {
          seg_list_node_t* seg_add = seg_stack_.get_new();
          if (seg_add != nullptr) {
            seg_add->data.name = name;
            seg_add->data.seg_type = FplSegment::ENRT;
            seg_add->data.is_direct = false;
            seg_add->data.is_discon = false;
            seg_add->data.end = nullptr;
            seg_list_.insert_before(&(seg_list_.tail), seg_add);
            update_id();

            return true;
          }
        }
      }
    } else if (next.ptr != &(seg_list_.head) &&
               next.ptr->prev != &(seg_list_.head)) {
      seg_list_node_t* prev = next.ptr->prev;
      seg_list_node_t* base_seg = prev->prev;
      leg_list_node_t* prev_end_leg = prev->data.end;
      leg_list_node_t* end_leg = base_seg->data.end;

      if (end_leg != nullptr) {
        libnav::waypoint_t end_fix = end_leg->data.leg.main_fix;
        std::string end_leg_awy_id = end_fix.get_awy_id();

        if (end_fix.data.area_code == "ENRT" &&
            awy_db_->is_in_awy(name, end_leg_awy_id)) {
          if (prev_end_leg != nullptr && !(prev->data.is_discon)) {
            libnav::waypoint_t prev_end_fix = prev_end_leg->data.leg.main_fix;
            std::string prev_end_leg_awy_id = prev_end_fix.get_awy_id();

            if (awy_db_->is_in_awy(name, prev_end_leg_awy_id)) {
              std::vector<libnav::awy_point_t> awy_pts;
              size_t n_pts = awy_db_->get_ww_path(name, end_leg_awy_id,
                                                 prev_end_leg_awy_id, &awy_pts);

              if (n_pts) {
                delete_segment(prev);
                add_awy_seg(name, next.ptr, awy_pts);

                return true;
              }
            }
          }
          FplSegment prev_tp = prev->data.seg_type;
          if (!(prev->data.is_discon)) {
            delete_segment(prev, true, true);
          }

          seg_list_node_t* seg_add = seg_stack_.get_new();
          if (seg_add != nullptr) {
            seg_add->data.name = name;
            seg_add->data.seg_type = prev_tp;
            seg_add->data.is_direct = false;
            seg_add->data.is_discon = false;
            seg_add->data.end = nullptr;
            seg_list_.insert_before(base_seg->next, seg_add);
            update_id();

            return true;
          }
        }
      }
    }
  }

  return false;
}

bool FplnInt::awy_insert_str(timed_ptr_t<seg_list_node_t> next,
                             std::string end_id) {
  std::vector<libnav::waypoint_entry_t> cand;
  std::vector<std::string> id_split =
      strutils::str_split(end_id, libnav::AUX_ID_SEP);
  size_t n_cand = navaid_db_->get_wpt_by_awy_str(end_id, &cand);
  assert(n_cand != 0);
  libnav::waypoint_t wpt = {id_split[0], cand[0]};

  return awy_insert(next, wpt);
}

bool FplnInt::awy_insert(timed_ptr_t<seg_list_node_t> next,
                         libnav::waypoint_t end) {
  std::unique_lock lock(fpl_mtx_);

  if (next.id == seg_list_.id && next.ptr != &(seg_list_.head)) {
    if (next.ptr == nullptr) {
      seg_list_node_t* prev = seg_list_.tail.prev;
      if (prev->data.end != nullptr) {
        leg_t dir_leg = {};
        dir_leg.leg_type = "TF";
        dir_leg.set_main_fix(end);
        add_direct_leg(dir_leg, &(leg_list_.tail));
        return true;
      }
    } else {
      seg_list_node_t* prev = next.ptr->prev;

      if (prev != &(seg_list_.head)) {
        std::string prev_name = prev->data.name;
        seg_list_node_t* prev_full = prev->prev;

        std::string end_id = end.get_awy_id();

        bool in_awy = end.data.area_code == "ENRT" &&
                      awy_db_->is_in_awy(prev_name, end_id);
        if (prev_full->data.end != nullptr && in_awy) {
          leg_list_node_t* prev_leg = prev_full->data.end;
          libnav::waypoint_t start_fix = prev_leg->data.leg.main_fix;
          std::string start_id = start_fix.get_awy_id();

          std::vector<libnav::awy_point_t> awy_pts;
          size_t n_pts = size_t(
              awy_db_->get_ww_path(prev_name, start_id, end_id, &awy_pts));

          if (n_pts) {
            delete_segment(prev, true, true);
            add_awy_seg(prev_name, prev_full->next, awy_pts);

            return true;
          }
        } else if (prev_full->data.end != nullptr && !in_awy) {
          delete_segment(prev, true, true);
          leg_t dir_leg = {};
          dir_leg.leg_type = "TF";
          dir_leg.set_main_fix(end);
          add_direct_leg(dir_leg, prev_full->data.end->next);

          return true;
        }
      }
    }
  }

  return false;
}

bool FplnInt::delete_via(timed_ptr_t<seg_list_node_t> curr) {
  std::unique_lock lock(fpl_mtx_);

  if (curr.id == seg_list_.id && curr.ptr != &(seg_list_.head) &&
      curr.ptr != nullptr && curr.ptr->prev != &(seg_list_.head)) {
    if (act_leg_ != nullptr && curr.ptr == act_leg_->data.seg) return false;
    if (!curr.ptr->data.is_discon && !curr.ptr->data.is_direct) {
      delete_segment(curr.ptr, true, false, true);

      return true;
    }
  }
  return false;
}

bool FplnInt::delete_seg_end(timed_ptr_t<seg_list_node_t> curr) {
  std::unique_lock lock(fpl_mtx_);

  if (curr.id == seg_list_.id && curr.ptr != &(seg_list_.head) &&
      curr.ptr != nullptr && !curr.ptr->data.is_discon &&
      curr.ptr != &(seg_list_.tail)) {
    if (act_leg_ != nullptr && curr.ptr == act_leg_->data.seg) return false;

    seg_list_node_t* next = curr.ptr->next;
    if (next != &(seg_list_.tail) && !next->data.is_direct &&
        !next->data.is_discon && curr.ptr->data.end != nullptr) {
      delete_segment(curr.ptr->next, true, false, true);
    }

    delete_segment(curr.ptr, false, true);
    return true;
  }

  return false;
}

bool FplnInt::dir_from_to(timed_ptr_t<leg_list_node_t> from,
                          timed_ptr_t<leg_list_node_t> to) {
  std::unique_lock lock(fpl_mtx_);

  if (from.id == leg_list_.id && from.id == to.id) {
    if (to.ptr->prev == act_leg_) act_leg_ = from.ptr;
    delete_range(from.ptr, to.ptr);
    return true;
  }
  return false;
}

void FplnInt::add_direct(libnav::waypoint_t wpt,
                         timed_ptr_t<leg_list_node_t> next) {
  std::unique_lock lock(fpl_mtx_);

  if (next.id == leg_list_.id) {
    leg_t dir_leg{};
    dir_leg.leg_type = "DF";
    dir_leg.set_main_fix(wpt);
    dir_leg.turn_dir = libnav::TurnDir::EITHER;

    add_direct_leg(dir_leg, next.ptr);
  }
}

bool FplnInt::delete_leg(timed_ptr_t<leg_list_node_t> next) {
  std::unique_lock lock(fpl_mtx_);

  if (next.id == leg_list_.id) {
    return delete_singl_leg(next.ptr);
  }
  return false;
}

void FplnInt::set_spd_cstr(timed_ptr_t<leg_list_node_t> node, spd_cstr_t cst) {
  std::unique_lock lock(fpl_mtx_);
  if (node.id != leg_list_.id) return;
  node.ptr->data.leg.speed_desc = cst.mode;
  node.ptr->data.leg.spd_lim_kias = cst.magnitude;

  update_id();
}

void FplnInt::set_alt_cstr(timed_ptr_t<leg_list_node_t> node, alt_cstr_t cst) {
  std::unique_lock lock(fpl_mtx_);
  if (node.id != leg_list_.id) return;
  node.ptr->data.leg.alt_desc = cst.mode;
  node.ptr->data.leg.alt1_ft = cst.magnitude;
  node.ptr->data.leg.alt2_ft = 0;

  update_id();
}

void FplnInt::update(double hdg_trk_diff) {
  std::unique_lock lock(fpl_mtx_);
  if (!is_apt_valid(departure_) || !is_apt_valid(arrival_)) {
    co_rte_nm_ = "";
    return;
  }
  if (fpl_id_calc_ != fpl_id_curr_) {
    double curr_alt_ft = 0;
    leg_list_node_t* leg_curr = leg_list_.head.next;

    while (leg_curr != &(leg_list_.tail)) {
      leg_list_node_t* next_leg = leg_curr->next;
      // Delete double discons:
      if (leg_curr->data.is_discon && (leg_curr->prev->data.is_discon ||
                                       leg_curr->prev == &(leg_list_.head))) {
        delete_segment(leg_curr->data.seg, false);
        leg_curr = next_leg;
        continue;
      }

      if (leg_curr->prev->data.is_discon) {
        leg_curr->data.leg.leg_type = "IF";
      } else if (leg_curr->prev != &(leg_list_.head) &&
                 leg_curr->data.leg.leg_type == "IF") {
        std::string prev_type = leg_curr->prev->data.leg.leg_type;
        if (NOT_FOLLOWED_BY_DF.find(prev_type) != NOT_FOLLOWED_BY_DF.end()) {
          leg_curr->data.leg.leg_type = "CF";
          geo::point prev_pos = leg_curr->prev->data.misc_data.start;
          geo::point curr_pos = leg_curr->data.leg.main_fix.data.pos;
          double trk_deg = prev_pos.get_gc_bearing_rad(curr_pos);
          leg_curr->data.leg.outbd_crs_deg = trk_deg;
          leg_curr->data.leg.outbd_crs_true = true;
        } else {
          leg_curr->data.leg.leg_type = "DF";
        }
      }

      if (!leg_curr->data.is_discon) {
        if (leg_curr->data.seg->data.seg_type == FplSegment::DEP_RWY) {
          libnav::arinc_rwy_data_t dep_data =
              get_rwy_data(get_ref_for(FplSegment::DEP_RWY).name);
          curr_alt_ft = dep_data.thresh_elev_msl_ft;
        } else if (leg_curr->data.leg.main_fix.data.type ==
                   libnav::NavaidType::RWY) {
          libnav::arinc_rwy_data_t arr_data = get_rwy_data(arr_rwy_, true);
          curr_alt_ft = arr_data.thresh_elev_msl_ft;
        }
        calculate_leg(leg_curr, hdg_trk_diff, curr_alt_ft);
        if (leg_curr->data.leg.alt1_ft != 0)
          curr_alt_ft = leg_curr->data.leg.alt1_ft;
      }

      leg_curr = next_leg;
    }

    fpl_id_calc_ = fpl_id_curr_;
  }

  update_act_leg();
}

// Private functions:

// Static member functions:

size_t FplnInt::get_proc_db_idx(ProcType tp, bool is_arr) {
  if (tp == PROC_TYPE_SID && is_arr) {
    return N_PROC_DB_SZ;
  }

  return tp + N_ARR_DB_OFFSET * is_arr;
}

FplSegment FplnInt::get_proc_tp(ProcType tp) {
  switch (tp) {
    case PROC_TYPE_SID:
      return FplSegment::SID;
    case PROC_TYPE_STAR:
      return FplSegment::STAR;
    case PROC_TYPE_APPCH:
      return FplSegment::APPCH;
    default:
      return FplSegment::NONE;
  }
}

FplSegment FplnInt::get_trans_tp(ProcType tp) {
  switch (tp) {
    case PROC_TYPE_SID:
      return FplSegment::SID_TRANS;
    case PROC_TYPE_STAR:
      return FplSegment::STAR_TRANS;
    case PROC_TYPE_APPCH:
      return FplSegment::APPCH_TRANS;
    default:
      return FplSegment::NONE;
  }
}

std::vector<std::string> FplnInt::get_proc(libnav::str_umap_t& db,
                                           std::string rw) {
  std::vector<std::string> out;

  for (auto i : db) {
    if (rw != "" && i.second.find(rw) == i.second.end()) {
      continue;
    }
    out.push_back(i.first);
  }

  return out;
}

std::vector<std::string> FplnInt::get_apprs(libnav::str_umap_t& proc_db,
                                            libnav::str_umap_t& appr_db,
                                            std::string proc, bool filter) {
  if (filter) assert(proc_db.find(proc) != proc_db.end());

  std::vector<std::string> out;
  for (auto i : appr_db) {
    std::string curr_nm = i.first;
    if (filter) {
      std::string c_rnw = get_appr_rwy(curr_nm);

      if (proc_db[proc].find(c_rnw) == proc_db[proc].end()) continue;
    }

    out.push_back(curr_nm);
  }

  return out;
}

std::vector<std::string> FplnInt::get_proc_trans(std::string proc,
                                                 libnav::str_umap_t& db,
                                                 libnav::arinc_rwy_db_t& rwy_db,
                                                 bool is_rwy, bool incl_none) {
  std::vector<std::string> out;

  assert(db.find(proc) != db.end());

  for (auto i : db[proc]) {
    if (i == libnav::NONE_TRANS && !incl_none) continue;

    if (is_rwy && rwy_db.find(i) != rwy_db.end()) {
      out.push_back(i);
    } else if (!is_rwy && rwy_db.find(i) == rwy_db.end()) {
      out.push_back(i);
    }
  }

  return out;
}

std::string FplnInt::get_dfms_enrt_leg(const leg_list_node_t* lg, bool force_dir) {
  leg_t leg = lg->data.leg;
  libnav::navaid_type_t xp_type =
      libnav::libnav_to_xp_fix(leg.main_fix.data.type);
  std::string type_str = std::to_string(int(xp_type));
  std::string awy_nm = lg->data.seg->data.name;
  std::string dfms_awy_nm = DFMS_DIR_SEG_NM;

  if (awy_nm != DCT_LEG_NAME && !force_dir) {
    dfms_awy_nm = awy_nm;
  }

  double curr_alt_ft = 0;
  if (leg.alt_desc == libnav::AltMode::AT) {
    curr_alt_ft = double(leg.alt2_ft);
  }

  std::string alt_str = strutils::double_to_str(curr_alt_ft, N_DFMS_OUT_PREC);
  std::string lat_str = strutils::double_to_str(
      leg.main_fix.data.pos.lat_rad * geo::RAD_TO_DEG, N_DFMS_OUT_PREC);
  std::string lon_str = strutils::double_to_str(
      leg.main_fix.data.pos.lon_rad * geo::RAD_TO_DEG, N_DFMS_OUT_PREC);

  return type_str + DFMS_COL_SEP + leg.main_fix.id + DFMS_COL_SEP +
         dfms_awy_nm + DFMS_COL_SEP + alt_str + DFMS_COL_SEP + lat_str +
         DFMS_COL_SEP + lon_str;
}

// Non-static member functions:

bool FplnInt::is_apt_valid(libnav::Airport* ptr) const {
  if (ptr == nullptr) return false;
  return arpt_db_ptr_->is_airport(ptr->icao_code);
}

void FplnInt::update_act_leg() {
  if (!is_active_) {
    act_leg_ = nullptr;
    return;
  }

  if (act_leg_ == nullptr) {
    leg_list_node_t* curr = &(leg_list_.head);
    curr = curr->next;
    if (curr->data.misc_data.is_rwy && curr != &(leg_list_.tail)) {
      curr = curr->next;
    }
    if (curr != &(leg_list_.tail)) act_leg_ = curr;
  }
}

libnav::DbErr FplnInt::process_dfms_proc_line(std::vector<std::string>& l_split,
                                              bool set_arpts,
                                              dfms_arr_data_t* arr_data) {
  bool db_err = false;

  if (set_arpts && l_split[0] == DFMS_DEP_NM) {
    libnav::DbErr err = set_dep(l_split[1]);
    if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
      return err;
    }
  } else if (l_split[0] == DFMS_DEP_RWY_NM) {
    std::string tmp_rwy = get_dfms_rwy(l_split[1]);
    db_err = !set_dep_rwy(tmp_rwy);
  } else if (l_split[0] == DFMS_SID_NM) {
    db_err = !set_arpt_proc(PROC_TYPE_SID, l_split[1]);
  } else if (l_split[0] == DFMS_SID_TRANS_NM) {
    db_err = !set_arpt_proc_trans(PROC_TYPE_SID, l_split[1]);
  } else if (set_arpts && l_split[0] == DFMS_ARR_NM) {
    arr_data->arr_icao = l_split[1];
  } else if (l_split[0] == DFMS_ARR_RWY_NM) {
    arr_data->arr_rwy = l_split[1];
  } else if (l_split[0] == DFMS_STAR_NM) {
    arr_data->star = l_split[1];
  } else if (l_split[0] == DFMS_SID_TRANS_NM) {
    arr_data->star_trans = l_split[1];
  }

  if (db_err) {
    return libnav::DbErr::DATA_BASE_ERROR;
  }

  return libnav::DbErr::SUCCESS;
}

libnav::DbErr FplnInt::set_dfms_arr_data(dfms_arr_data_t* arr_data,
                                         bool set_arpt) {
  if (set_arpt && arr_data->arr_icao != "") {
    libnav::DbErr err = set_arr(arr_data->arr_icao);
    if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
      return err;
    }
  }
  std::string tmp_rwy = get_dfms_rwy(arr_data->arr_rwy);
  if (tmp_rwy != "" && !set_arr_rwy(tmp_rwy))
    return libnav::DbErr::DATA_BASE_ERROR;

  if (arr_data->star != "" &&
      !set_arpt_proc(PROC_TYPE_STAR, arr_data->star, true))
    return libnav::DbErr::DATA_BASE_ERROR;

  if (arr_data->star_trans != "" &&
      !set_arpt_proc_trans(PROC_TYPE_STAR, arr_data->star_trans, true))
    return libnav::DbErr::DATA_BASE_ERROR;

  return libnav::DbErr::SUCCESS;
}

bool FplnInt::get_dfms_wpt(std::vector<std::string>& l_split,
                           libnav::waypoint_t* out) {
  libnav::navaid_type_t tp =
      libnav::navaid_type_t(strutils::stoi_with_strip(l_split[0]));
  libnav::NavaidType nav_tp = libnav::xp_fix_type_to_libnav(tp);

  std::vector<libnav::waypoint_entry_t> wpt_entrs;
  size_t n_ent =
      navaid_db_->get_wpt_data(l_split[1], &wpt_entrs, "", "", nav_tp);

  if (n_ent) {
    double p_lat_rad =
        double(strutils::stof_with_strip(l_split[4])) * geo::DEG_TO_RAD;
    double p_lon_rad =
        double(strutils::stof_with_strip(l_split[5])) * geo::DEG_TO_RAD;

    libnav::sort_wpt_entry_by_dist(&wpt_entrs, {p_lat_rad, p_lon_rad});

    *out = {l_split[1], wpt_entrs[0]};
    return true;
  }

  return false;
}

std::string FplnInt::get_dfms_arpt_leg(bool is_arr) const {
  libnav::Airport* ptr = departure_;
  std::string seg_nm = DFMS_DEP_NM;

  if (is_arr) {
    ptr = arrival_;
    seg_nm = DFMS_ARR_NM;
  }

  std::string icao_cd = ptr->icao_code;

  double alt_restr = 0;
  double arpt_lat_deg = 0;
  double arpt_lon_deg = 0;

  libnav::airport_data_t arpt_data;
  arpt_db_ptr_->get_airport_data(icao_cd, &arpt_data);

  alt_restr = arpt_data.elevation_ft;
  arpt_lat_deg = arpt_data.pos.lat_rad * geo::RAD_TO_DEG;
  arpt_lon_deg = arpt_data.pos.lon_rad * geo::RAD_TO_DEG;

  std::string alt_restr_str =
      strutils::double_to_str(alt_restr, N_DFMS_OUT_PREC);
  std::string arpt_lat_str =
      strutils::double_to_str(arpt_lat_deg, N_DFMS_OUT_PREC);
  std::string arpt_lon_str =
      strutils::double_to_str(arpt_lon_deg, N_DFMS_OUT_PREC);

  std::string seg_tp = "1";

  return seg_tp + DFMS_COL_SEP + icao_cd + DFMS_COL_SEP + seg_nm +
         DFMS_COL_SEP + alt_restr_str + DFMS_COL_SEP + arpt_lat_str +
         DFMS_COL_SEP + arpt_lon_str;
}

size_t FplnInt::get_dfms_enrt_legs(std::vector<std::string>* out) const {
  out->push_back(get_dfms_arpt_leg());

  const leg_list_node_t* start = &(leg_list_.head);

  while (start->next != &(leg_list_.tail) &&
         (start->next->data.seg->data.seg_type < FplSegment::ENRT ||
          start->data.seg->data.seg_type == FplSegment::DEP_RWY)) {
    start = start->next;
  }

  bool first_leg = true;

  while (start != &(leg_list_.tail) &&
         start->data.seg->data.seg_type <= FplSegment::ENRT) {
    if (!start->data.is_discon) {
      std::string tmp = get_dfms_enrt_leg(start, first_leg);
      out->push_back(tmp);
      if (first_leg) {
        first_leg = false;
      }
    }

    start = start->next;
  }

  out->push_back(get_dfms_arpt_leg(true));

  return out->size();
}

libnav::DbErr FplnInt::load_fms_fpln(std::string& file_nm, bool set_arpts) {
  if (libnav::does_file_exist(file_nm + DFMS_FILE_POSTFIX)) {
    std::ifstream file(file_nm + DFMS_FILE_POSTFIX);
    if (file.is_open()) {
      std::string line;
      bool read_enrt = false;
      dfms_arr_data_t arr_data;
      std::string awy_last = "";
      std::string end_last = "";
      while (getline(file, line)) {
        std::vector<std::string> ln_split = strutils::str_split(line);

        if (!read_enrt && ln_split.size() > 1) {
          if (ln_split[0] == DFMS_N_ENRT_NM) {
            read_enrt = true;
          } else if (ln_split[0] == DFMS_AIRAC_CYCLE_NM) {
            int fpl_cycle = strutils::stoi_with_strip(ln_split[1]);
            if (airac_mismatch_ || fix_airac_version_ != fpl_cycle)
              return libnav::DbErr::DATA_BASE_ERROR;
          } else {
            libnav::DbErr err =
                process_dfms_proc_line(ln_split, set_arpts, &arr_data);

            if (err != libnav::DbErr::SUCCESS) {
              return err;
            }
          }
        } else if (read_enrt && ln_split.size() == N_DFMS_ENRT_WORDS) {
          std::string wpt_id = "";
          bool add_awy_seg = false;
          if (ln_split[2] != DFMS_DEP_NM && ln_split[2] != DFMS_ARR_NM) {
            libnav::waypoint_t wpt;
            bool ret = get_dfms_wpt(ln_split, &wpt);
            wpt_id = wpt.get_awy_id();

            if (ret) {
              if (ln_split[2] == DFMS_DIR_SEG_NM ||
                  (ln_split[2] != awy_last && awy_last != "")) {
                add_awy_seg = true;
              } else {
                awy_last = ln_split[2];
                end_last = wpt_id;
              }
            } else {
              return libnav::DbErr::DATA_BASE_ERROR;
            }
          } else {
            add_awy_seg = true;
          }

          if (add_awy_seg) {
            if (awy_last != "" && end_last != "") {
              add_enrt_seg({nullptr, seg_list_.id}, awy_last);
              awy_insert_str({&(seg_list_.tail), seg_list_.id}, end_last);
            }
            awy_last = "";
            end_last = "";

            if (wpt_id != "") awy_insert_str({nullptr, seg_list_.id}, wpt_id);
          }
        }
      }

      file.close();

      return set_dfms_arr_data(&arr_data, set_arpts);
    }

    file.close();
  }
  return libnav::DbErr::FILE_NOT_FOUND;
}

// Other auxiliury functions:

void FplnInt::adjust_list_pointers(FplnInt& other) {
  leg_list_node_t* curr_leg = leg_list_.head.next;

  while (curr_leg != &leg_list_.tail) {
    if (curr_leg->data.seg != nullptr) {
      curr_leg->data.seg =
          curr_leg->data.seg - other.seg_stack_.nodes + seg_stack_.nodes;
    }
    curr_leg = curr_leg->next;
  }

  seg_list_node_t* curr_seg = seg_list_.head.next;
  ;

  while (curr_seg != &seg_list_.tail) {
    if (curr_seg->data.end != nullptr) {
      curr_seg->data.end = curr_seg->data.end - other.leg_data_stack_.nodes +
                           leg_data_stack_.nodes;
    }
    curr_seg = curr_seg->next;
  }
}

void FplnInt::copy_act_leg(FplnInt& other) {
  if (other.act_leg_ == nullptr) {
    act_leg_ = nullptr;
  } else {
    act_leg_ = other.act_leg_ - other.leg_data_stack_.nodes + leg_data_stack_.nodes;
  }
}

void FplnInt::update_apt_dbs(bool arr) {
  if (arr) {
    arr_rnw_ = arrival_->get_rwy_db();
    proc_db_[N_ARR_DB_OFFSET + PROC_TYPE_STAR] = arrival_->get_all_stars();
    proc_db_[N_ARR_DB_OFFSET + PROC_TYPE_APPCH] = arrival_->get_all_appch();
  } else {
    dep_rnw_ = departure_->get_rwy_db();
    proc_db_[PROC_TYPE_SID] = departure_->get_all_sids();
    proc_db_[PROC_TYPE_STAR] = departure_->get_all_stars();
    proc_db_[PROC_TYPE_APPCH] = departure_->get_all_appch();
  }
}

libnav::arinc_rwy_data_t FplnInt::get_rwy_data(std::string nm, bool is_arr) {
  libnav::arinc_rwy_data_t out = {};

  libnav::arinc_rwy_db_t* db = &dep_rnw_;

  if (is_arr) db = &arr_rnw_;

  if (db->find(nm) != db->end()) {
    out = db->at(nm);
  }

  return out;
}

std::string FplnInt::get_curr_proc_imp(ProcType tp, bool trans) {
  FplSegment s_tp = get_proc_tp(tp);

  if (trans) {
    if (s_tp == FplSegment::SID)
      s_tp = FplSegment::SID_TRANS;
    else if (s_tp == FplSegment::STAR)
      s_tp = FplSegment::STAR_TRANS;
    else
      s_tp = FplSegment::APPCH_TRANS;
  }

  if (s_tp == FplSegment::APPCH && appr_is_rwy_) return "";
  return get_ref_for(s_tp).name;
}

bool FplnInt::add_fpl_seg(libnav::arinc_leg_seq_t& legs,
                          FplSegment seg_tp, std::string ref_nm,
                          std::string seg_nm, seg_list_node_t* next,
                          bool set_ref) {
  if (legs.size()) {
    size_t seg_idx = size_t(seg_tp);
    leg_t start = legs[0];
    std::vector<leg_t> legs_ins;

    for (size_t i = 1; i < legs.size(); i++) {
      legs_ins.push_back(legs[i]);
    }

    if (seg_nm == "") seg_nm = ref_nm;
    add_legs(start, legs_ins, seg_tp, seg_nm, next);
    if (set_ref) fpl_refs_[seg_idx].name = ref_nm;

    return true;
  }

  return false;
}

leg_t FplnInt::get_awy_tf_leg(libnav::awy_point_t awy_pt) {
  std::string wpt_uid = awy_pt.get_uid();
  std::vector<std::string> uid_split = strutils::str_split(
      wpt_uid, libnav::AUX_ID_SEP);  // This should be somewhere in libnav
  std::vector<libnav::waypoint_entry_t> cand;
  size_t n_cand = navaid_db_->get_wpt_by_awy_str(wpt_uid, &cand);
  assert(n_cand != 0);
  libnav::waypoint_t wpt = {uid_split[0], cand[0]};
  leg_t out{};
  out.leg_type = "TF";
  out.set_main_fix(wpt);

  return out;
}

void FplnInt::add_awy_seg(std::string awy, seg_list_node_t* next,
                          std::vector<libnav::awy_point_t>& awy_pts) {
  leg_t start_leg = get_awy_tf_leg(awy_pts[0]);
  std::vector<leg_t> legs;

  for (size_t i = 1; i < awy_pts.size(); i++) {
    legs.push_back(get_awy_tf_leg(awy_pts[i]));
  }

  add_legs(start_leg, legs, FplSegment::ENRT, awy, next);
}

bool FplnInt::set_sid_star(std::string proc_nm, bool is_star, bool reset_rwy) {
  size_t db_idx;
  ProcType proc_tp;
  FplSegment proc_seg, trans_seg;
  if (!is_star) {
    proc_tp = PROC_TYPE_SID;
    db_idx = get_proc_db_idx(PROC_TYPE_SID, false);
    proc_seg = FplSegment::SID;
    trans_seg = FplSegment::SID_TRANS;
  } else {
    proc_tp = PROC_TYPE_STAR;
    db_idx = get_proc_db_idx(PROC_TYPE_STAR, true);
    proc_seg = FplSegment::STAR;
    trans_seg = FplSegment::STAR_TRANS;
  }

  if (proc_db_[db_idx].find(proc_nm) != proc_db_[db_idx].end()) {
    std::string rwy;
    if (!is_star)
      rwy = get_ref_for(FplSegment::DEP_RWY).name;
    else
      rwy = arr_rwy_;

    if (rwy == "") {
      delete_ref(trans_seg);
      delete_ref(proc_seg);
      fpl_refs_[size_t(proc_seg)].name = proc_nm;
    } else {
      libnav::arinc_leg_seq_t legs;
      libnav::arinc_leg_seq_t
          legs_add;  // Additional legs. Inserted if there is a blank transition
      std::string none_trans = libnav::NONE_TRANS;

      if (!is_star) {
        legs = departure_->get_sid(proc_nm, rwy);
        legs_add = departure_->get_sid(proc_nm, none_trans);
      } else {
        legs = arrival_->get_star(proc_nm, rwy);
        legs_add = arrival_->get_star(proc_nm, none_trans);
      }

      libnav::arinc_leg_seq_t* l_add = &legs_add;
      libnav::arinc_leg_seq_t* l_tmp = &legs;
      if (!is_star) std::swap(l_add, l_tmp);

      size_t i_beg = l_add->size() > 0;
      for (size_t i = i_beg; i < l_tmp->size(); i++) {
        l_add->push_back(l_tmp->at(i));
      }

      std::string trans_nm = fpl_refs_[size_t(trans_seg)].name;
      delete_ref(trans_seg);
      bool retval = add_fpl_seg(*l_add, proc_seg, proc_nm);
      if (!retval)  // Case: runway doesn't belong to sid
      {
        delete_ref(proc_seg);
        if (is_star) {
          if (reset_rwy) {
            arr_rwy_ = "";
            has_arr_rnw_data_ = false;
            delete_ref(FplSegment::APPCH);
            delete_ref(FplSegment::APPCH_TRANS);
          }

          delete_ref(FplSegment::STAR_TRANS);
        } else {
          if (reset_rwy) delete_ref(FplSegment::DEP_RWY);
          delete_ref(FplSegment::SID_TRANS);
        }

        if (reset_rwy) fpl_refs_[size_t(proc_seg)].name = proc_nm;
      } else {
        set_proc_trans(proc_tp, trans_nm, is_star);
      }

      return retval;
    }
  }

  return false;
}

bool FplnInt::set_appch_legs(std::string appch, std::string& arr_rwy,
                             libnav::arinc_leg_seq_t legs,
                             std::string appch_seg) {
  size_t rwy_idx = 0;
  bool rwy_found = false;
  libnav::arinc_leg_seq_t appch_legs = {};
  libnav::arinc_leg_seq_t ga_legs = {};  // Missed approach legs
  for (size_t i = 0; i < legs.size(); i++) {
    appch_legs.push_back(legs[i]);

    if (legs[i].main_fix.id == arr_rwy) {
      rwy_idx = i;
      rwy_found = true;
      break;
    }
  }

  bool added = add_fpl_seg(appch_legs, FplSegment::APPCH, 
    appch, appch_seg);

  if (added) {
    if (rwy_found) {
      for (size_t i = rwy_idx; i < legs.size(); i++) {
        ga_legs.push_back(legs[i]);
      }

      seg_list_node_t* appr_seg = get_ref_for(
        FplSegment::APPCH).ptr;
      if (appr_seg != nullptr) {
        seg_list_node_t* seg_ins = get_ref_for(
            FplSegment::APPCH).ptr->next;
        return add_fpl_seg(ga_legs, FplSegment::APPCH, "", MISSED_APPR_SEG_NM,
                           seg_ins, false);
      } else {
        return false;
      }
    }

    return true;
  }
  return false;
}

bool FplnInt::set_appch(std::string appch) {
  size_t db_idx = get_proc_db_idx(PROC_TYPE_APPCH, true);
  appr_is_rwy_ = false;

  if (proc_db_[db_idx].find(appch) != proc_db_[db_idx].end()) {
    std::string curr_star = get_ref_for(FplSegment::STAR).name;
    std::string curr_star_trans = get_ref_for(FplSegment::STAR_TRANS).name;
    delete_ref(FplSegment::STAR_TRANS);
    delete_ref(FplSegment::STAR);

    std::string tmp = libnav::NONE_TRANS;
    libnav::arinc_leg_seq_t legs = arrival_->get_appch(appch, tmp);
    std::string curr_tr = get_ref_for(FplSegment::APPCH).name;
    delete_ref(FplSegment::APPCH_TRANS);

    std::string tmp_rwy = get_appr_rwy(appch);
    bool added = set_appch_legs(appch, tmp_rwy, legs);
    if (added) {
      arr_rwy_ = tmp_rwy;
      int data_found =
          arpt_db_ptr_->get_rnw_data(arrival_->icao_code, arr_rwy_, &arr_rnw_data_);
      has_arr_rnw_data_ = true;
      if (!data_found) {
        has_arr_rnw_data_ = false;
        arr_rnw_data_ = {};
      }

      set_sid_star(curr_star, true, false);
      set_proc_trans(PROC_TYPE_STAR, curr_star_trans, true);
      set_proc_trans(PROC_TYPE_APPCH, curr_tr, true);
      return true;
    }
  }

  arr_rwy_ = "";

  delete_ref(FplSegment::STAR_TRANS);
  delete_ref(FplSegment::STAR);
  delete_ref(FplSegment::APPCH_TRANS);
  delete_ref(FplSegment::APPCH);
  return false;
}

bool FplnInt::add_trans_legs(ProcType tp, std::string trans,
                             libnav::arinc_leg_seq_t& pr_legs,
                             libnav::arinc_leg_seq_t& tr_legs) {
  FplSegment seg_tp = get_proc_tp(tp);
  FplSegment t_tp = get_trans_tp(tp);
  size_t t_idx = size_t(t_tp);
  if (tr_legs.size() == 0 && trans != "") {
    delete_ref(t_tp);
    return false;
  }

  libnav::arinc_leg_seq_t all_legs;

  if (tp == PROC_TYPE_SID) {
    all_legs = pr_legs;
    for (size_t i = 1; i < tr_legs.size(); i++) {
      all_legs.push_back(tr_legs[i]);
    }
  } else {
    all_legs = tr_legs;
    for (size_t i = 1; i < pr_legs.size(); i++) {
      all_legs.push_back(pr_legs[i]);
    }
  }

  if (all_legs.size() == 0) {
    delete_ref(t_tp);
    return false;
  }

  std::string proc_name = fpl_refs_[size_t(seg_tp)].name;
  std::string seg_name = proc_name;
  if (trans != "") seg_name += "." + trans;

  bool ret = true;
  if (tp != PROC_TYPE_APPCH)
    add_fpl_seg(all_legs, seg_tp, proc_name, seg_name);
  else
    ret = set_appch_legs(proc_name, arr_rwy_, all_legs, seg_name);
  fpl_refs_[t_idx].name = trans;

  return ret;
}

bool FplnInt::set_proc_trans(ProcType tp, std::string trans, bool is_arr) {
  if (trans == libnav::NONE_TRANS) {
    trans = "";
  }

  FplSegment seg_tp = get_proc_tp(tp);
  FplSegment t_tp = get_trans_tp(tp);

  size_t db_idx = get_proc_db_idx(tp, is_arr);
  size_t seg_idx = size_t(seg_tp);
  size_t t_idx = size_t(t_tp);

  std::string curr_proc = fpl_refs_[seg_idx].name;

  if (curr_proc != "" && fpl_refs_[seg_idx].ptr == nullptr &&
      proc_db_[db_idx][curr_proc].find(trans) !=
          proc_db_[db_idx][curr_proc].end()) {
    delete_ref(t_tp);
    fpl_refs_[t_idx].name = trans;

    return false;
  } else if (curr_proc == "") {
    fpl_refs_[t_idx].name = "";
    delete_ref(t_tp);

    return false;
  }
  libnav::Airport* apt = departure_;
  if (is_arr) {
    apt = arrival_;
  }

  libnav::arinc_leg_seq_t legs_main = {};
  libnav::arinc_leg_seq_t legs = {};

  std::string rwy = libnav::NONE_TRANS;
  if (tp == ProcType::PROC_TYPE_SID)
    rwy = get_ref_for(FplSegment::DEP_RWY).name;
  else if (tp == PROC_TYPE_STAR)
    rwy = arr_rwy_;

  if (tp == PROC_TYPE_SID) {
    legs_main = apt->get_sid(curr_proc, rwy);
    legs = apt->get_sid(curr_proc, trans);
  } else if (tp == PROC_TYPE_STAR) {
    legs_main = apt->get_star(curr_proc, rwy);
    legs = apt->get_star(curr_proc, trans);
  } else if (tp == PROC_TYPE_APPCH) {
    legs_main = apt->get_appch(curr_proc, rwy);
    legs = apt->get_appch(curr_proc, trans);
  }

  return add_trans_legs(tp, trans, legs_main, legs);
}

double FplnInt::get_leg_mag_var_deg(leg_list_node_t* leg) {
  double curr_var = leg->data.leg.get_mag_var_deg();
  if (leg->next != &(leg_list_.tail) && curr_var == 0) {
    return leg->next->data.leg.get_mag_var_deg();
  }
  return curr_var;
}

double FplnInt::get_leg_turn_rad(leg_list_node_t* curr) {
  double outbd_crs_next = curr->next->data.leg.outbd_crs_deg;

  if (!curr->next->data.leg.outbd_crs_true) {
    outbd_crs_next += get_leg_mag_var_deg(curr) * geo::DEG_TO_RAD;
  }

  return get_turn_rad(curr->data.misc_data.true_trk_deg * geo::DEG_TO_RAD,
                      curr->data.misc_data.start, outbd_crs_next,
                      curr->data.misc_data.start);
}

bool FplnInt::get_df_start(leg_seg_t curr_seg, leg_t next, geo::point* out) {
  double brng_end_start = curr_seg.true_trk_deg * geo::DEG_TO_RAD + M_PI;
  geo::point p1 = geo::get_pos_from_brng_dist(
      curr_seg.end, brng_end_start + M_PI / 2, TURN_RADIUS_NM);
  geo::point p2 = geo::get_pos_from_brng_dist(
      curr_seg.end, brng_end_start - M_PI / 2, TURN_RADIUS_NM);

  geo::point next_point = next.main_fix.data.pos;
  double dist1 = next_point.get_gc_dist_nm(p1);
  double dist2 = next_point.get_gc_dist_nm(p2);

  bool left_turn = next.turn_dir == libnav::TurnDir::LEFT;

  if (next.turn_dir == libnav::TurnDir::EITHER) left_turn = dist1 < dist2;

  if (left_turn)  // left turn
  {
    if (TURN_RADIUS_NM > dist1) return true;

    if (dist1 == 0) {
      *out = curr_seg.end;
    }
    double theta = acos(TURN_RADIUS_NM / dist1);
    double ang_main = p1.get_gc_bearing_rad(next_point);
    double ang_doub = ang_main + theta;
    *out = geo::get_pos_from_brng_dist(p1, ang_doub, TURN_RADIUS_NM);
  } else {
    if (TURN_RADIUS_NM > dist2) return true;

    if (dist2 == 0) {
      *out = curr_seg.end;
    }
    double theta = acos(TURN_RADIUS_NM / dist2);
    double ang_main = p2.get_gc_bearing_rad(next_point);
    double ang_doub = ang_main - theta;
    *out = geo::get_pos_from_brng_dist(p2, ang_doub, TURN_RADIUS_NM);
  }

  return false;
}

void FplnInt::get_to_leg_start(leg_seg_t curr_seg, leg_t next,
                               double mag_var_deg, double hdg_trk_diff,
                               geo::point* out) {
  double brng_end_start = curr_seg.true_trk_deg * geo::DEG_TO_RAD + M_PI;
  double crs_rad = double(next.outbd_crs_deg) * geo::DEG_TO_RAD;

  crs_rad -= mag_var_deg * geo::DEG_TO_RAD;

  if (next.leg_type[0] == 'V') crs_rad -= hdg_trk_diff;

  if (brng_end_start < 0) {
    brng_end_start += 2 * M_PI;
  }

  double turn_rad =
      get_turn_by_dir(brng_end_start - M_PI, crs_rad, next.turn_dir);

  if (abs(turn_rad) < M_PI / 2) {
    double theta = (M_PI - abs(turn_rad)) / 2;
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    if (sin_theta != 0 && cos_theta != 0) {
      double offs_nm = TURN_RADIUS_NM * cos_theta / sin_theta;
      offs_nm = std::max(0.0, offs_nm);

      *out = geo::get_pos_from_brng_dist(curr_seg.end, brng_end_start + M_PI,
                                         offs_nm);
    }
  } else {
    double brng_pr = brng_end_start - M_PI / 2;
    double brng_final = brng_end_start + M_PI / 2 + turn_rad;
    if (turn_rad < 0)  // left turn
    {
      brng_pr += M_PI;
      brng_final -= M_PI;
    }

    geo::point p1 =
        geo::get_pos_from_brng_dist(curr_seg.end, brng_pr, TURN_RADIUS_NM);
    geo::point p2 = geo::get_pos_from_brng_dist(p1, brng_final, TURN_RADIUS_NM);
    *out = p2;
  }
}

bool FplnInt::get_cf_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next,
                               double mag_var_deg, geo::point* out,
                               bool* to_inh, double* turn_radius_out) {
  double outbd_brng_deg = double(next.outbd_crs_deg);

  double mag_var = mag_var_deg;
  if (!next.outbd_crs_true) outbd_brng_deg -= mag_var;

  double brng_next_rad = outbd_brng_deg * geo::DEG_TO_RAD;
  double curr_brng_rad = curr_seg.true_trk_deg * geo::DEG_TO_RAD;

  double turn_rad = get_turn_rad(curr_brng_rad, curr_seg.start, brng_next_rad,
                                 next.main_fix.data.pos);

  geo::point intc;
  bool is_bp = false;

  if (abs(turn_rad) >= M_PI / 2 && curr_leg.leg_type != "VI" &&
      curr_leg.leg_type != "CI") {
    *to_inh = true;
    *turn_radius_out = std::max(
        get_cf_big_turn_isect(curr_seg, next, mag_var * geo::DEG_TO_RAD, &intc),
        TURN_RADIUS_NM);
  } else {
    // Replace with straight leg if course deviation is small enough
    *turn_radius_out = TURN_RADIUS_NM;

    double brng_end_to_main_fix =
        curr_seg.end.get_gc_bearing_rad(next.main_fix.data.pos);
    if (next.leg_type[0] == 'F') brng_end_to_main_fix += M_PI;

    double diff = abs(brng_end_to_main_fix - brng_next_rad);

    if (diff < CF_STRAIGHT_DEV_RAD && abs(turn_rad) < CF_STRAIGHT_DEV_RAD) {
      double new_brng_rad = brng_next_rad;
      double brng_from_next = brng_next_rad;
      if (curr_leg.leg_type == "CF") brng_from_next += M_PI;
      if (is_ang_greater(brng_end_to_main_fix, brng_next_rad)) {
        new_brng_rad += M_PI / 2;
      } else {
        new_brng_rad -= M_PI / 2;
      }
      intc = geo::get_pos_from_intc(curr_seg.end, next.main_fix.data.pos,
                                    new_brng_rad, brng_from_next);
      *to_inh = true;
      *out = intc;
      return false;
    }

    double brng_to_main_fix =
        curr_seg.start.get_gc_bearing_rad(next.main_fix.data.pos);

    bool left_turn = is_ang_greater(curr_brng_rad, brng_next_rad);
    bool brng_gr = is_ang_greater(brng_to_main_fix, curr_brng_rad);

    is_bp = true;
    if ((brng_gr && !left_turn) || (!brng_gr && left_turn)) {
      brng_next_rad += M_PI;
      is_bp = false;
    }

    geo::point intc1 = geo::get_pos_from_intc(
        curr_seg.start, next.main_fix.data.pos, curr_brng_rad, brng_next_rad);
    geo::point intc2 =
        geo::get_pos_from_intc(curr_seg.start, next.main_fix.data.pos,
                               curr_brng_rad, brng_next_rad - M_PI);

    double dist1 = next.main_fix.data.pos.get_gc_dist_nm(intc1);
    double dist2 = next.main_fix.data.pos.get_gc_dist_nm(intc2);
    if (dist1 < dist2) {
      intc = intc1;
    } else {
      intc = intc2;
    }
  }

  *out = intc;
  return is_bp;
}

bool FplnInt::get_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next,
                            double mag_var_deg, double hdg_trk_diff,
                            geo::point* out, bool* to_inh,
                            double* turn_radius_nm) {
  if (curr_leg.leg_type == "IF") {
    *out = curr_seg.calc_wpt.data.pos;
    return false;
  }
  if (TURN_OFFS_LEGS.find(next.leg_type) != TURN_OFFS_LEGS.end()) {
    if (next.leg_type == "DF") {
      return get_df_start(curr_seg, next, out);
    } else {
      get_to_leg_start(curr_seg, next, mag_var_deg, hdg_trk_diff, out);
      return false;
    }
  } else if (next.leg_type == "TF") {
    *out = curr_seg.calc_wpt.data.pos;
    return false;
  } else if (next.leg_type == "CF") {
    return get_cf_leg_start(curr_seg, curr_leg, next, mag_var_deg, out, to_inh,
                            turn_radius_nm);
  } else if (next.leg_type[0] == 'F') {
    get_cf_leg_start(curr_seg, curr_leg, next, mag_var_deg, out, to_inh,
                     turn_radius_nm);
    return false;
  }

  *out = curr_seg.end;
  return false;
}

void FplnInt::set_xi_leg(leg_list_node_t* leg) {
  leg_list_node_t* prev_leg = leg->prev;

  libnav::waypoint_t intc_wpt = {};
  intc_wpt.id = INTC_LEG_NM;
  intc_wpt.data.pos = leg->data.misc_data.start;
  intc_wpt.data.type = libnav::NavaidType::WAYPOINT;
  prev_leg->data.misc_data.set_calc_wpt(intc_wpt);

  prev_leg->data.misc_data.end = leg->data.misc_data.start;
  prev_leg->data.leg.outbd_dist_time =
      prev_leg->data.misc_data.start.get_gc_dist_nm(
          prev_leg->data.misc_data.end);
}

void FplnInt::set_turn_offset(leg_list_node_t* leg, leg_list_node_t* prev_leg) {
  double prev_trk_rad = prev_leg->data.misc_data.true_trk_deg * geo::DEG_TO_RAD;
  double curr_trk_rad = leg->data.misc_data.true_trk_deg * geo::DEG_TO_RAD;

  geo::point prev_start = prev_leg->data.misc_data.start;
  geo::point prev_end = prev_leg->data.misc_data.end;
  geo::point curr_start = leg->data.misc_data.start;
  geo::point curr_end = leg->data.misc_data.end;

  double turn_rad =
      get_turn_rad(prev_trk_rad, prev_start, curr_trk_rad, curr_end);
  turn_rad = abs(turn_rad);

  double prev_turn_rad_nm = prev_leg->data.misc_data.turn_rad_nm;
  double dist_nm = prev_start.get_gc_dist_nm(prev_end);

  if (turn_rad < M_PI / 2 && turn_rad != 0) {
    double ang_rad = M_PI - turn_rad;
    double turn_offs_nm =
        prev_turn_rad_nm * cos(ang_rad / 2) / sin(ang_rad / 2);
    double dist_end_start_nm = prev_end.get_gc_dist_nm(curr_start);
    turn_offs_nm = turn_offs_nm - dist_end_start_nm;

    if (turn_offs_nm < dist_nm) {
      dist_nm -= turn_offs_nm;
      double brng_rad = prev_leg->data.misc_data.true_trk_deg * geo::DEG_TO_RAD;
      prev_leg->data.misc_data.end =
          geo::get_pos_from_brng_dist(prev_start, brng_rad, dist_nm);
    }

    double curr_dist_nm = curr_start.get_gc_dist_nm(curr_end);

    if (turn_offs_nm > curr_dist_nm) {
      leg->data.misc_data.is_bypassed = true;
      leg->data.misc_data.is_to_inhibited = true;
    }
  } else {
    std::string leg_tp = leg->data.leg.leg_type;
    if (leg_tp == "TF") {
      double rnp_nm = get_rnp(leg);
      if (rnp_nm < dist_nm) {
        dist_nm -= rnp_nm;
        double brng_rad =
            prev_leg->data.misc_data.true_trk_deg * geo::DEG_TO_RAD;
        prev_leg->data.misc_data.end =
            geo::get_pos_from_brng_dist(prev_start, brng_rad, dist_nm);
      }
    }
  }
}

// The following functions are used to calculate ends of arinc424 legs.

void FplnInt::calculate_alt_leg(leg_list_node_t* leg, double hdg_trk_diff,
                                double curr_alt_ft) {
  leg_t curr_arinc_leg = leg->data.leg;

  libnav::runway_entry_t* rwy_ent = nullptr;

  if (curr_arinc_leg.leg_type != "FA") {
    if (leg->prev->data.seg->data.seg_type == FplSegment::DEP_RWY) {
      rwy_ent = &dep_rnw_data_;
    } else if (leg->prev->data.leg.main_fix.data.type ==
               libnav::NavaidType::RWY) {
      rwy_ent = &arr_rnw_data_;
    }

    if (rwy_ent) leg->data.misc_data.start = rwy_ent->end;
  }

  double mag_var_deg = -get_leg_mag_var_deg(leg);

  if (curr_arinc_leg.leg_type == "VA") {
    mag_var_deg += hdg_trk_diff * geo::RAD_TO_DEG;
  }

  leg->data.misc_data.true_trk_deg = curr_arinc_leg.outbd_crs_deg + mag_var_deg;

  geo::point ref_wpt = leg->data.misc_data.start;

  geo::point end_pt =
      get_xa_end_point(ref_wpt, curr_arinc_leg.outbd_crs_deg + mag_var_deg,
                       abs(curr_alt_ft - curr_arinc_leg.alt1_ft));
  libnav::waypoint_t end_wpt =
      get_ca_va_wpt(end_pt, int(curr_arinc_leg.alt1_ft));

  leg->data.misc_data.is_arc = false;
  leg->data.misc_data.is_finite = true;
  leg->data.misc_data.end = end_pt;
  leg->data.misc_data.turn_rad_nm = TURN_RADIUS_NM;

  leg->data.misc_data.set_calc_wpt(end_wpt);
  leg->data.leg.outbd_dist_time = ref_wpt.get_gc_dist_nm(end_pt);
  leg->data.leg.outbd_dist_as_time = false;
}

void FplnInt::calculate_intc_leg(leg_list_node_t* leg, double hdg_trk_diff) {
  leg_t curr_arinc_leg = leg->data.leg;

  std::string next_tp = leg->next->data.leg.leg_type;
  if (leg->next != &(leg_list_.tail) &&
      AFTER_INTC.find(next_tp) != AFTER_INTC.end()) {
    double curr_brng = double(curr_arinc_leg.outbd_crs_deg);
    if (!curr_arinc_leg.outbd_crs_true) {
      curr_brng -= get_leg_mag_var_deg(leg);
    }

    if (curr_arinc_leg.leg_type == "VI") {
      curr_brng += hdg_trk_diff * geo::RAD_TO_DEG;
    }
    leg->data.misc_data.true_trk_deg = curr_brng;
    leg->data.misc_data.is_arc = false;
    leg->data.misc_data.turn_rad_nm = TURN_RADIUS_NM;
    leg->data.misc_data.is_finite = true;
  }
}

void FplnInt::calculate_fc_leg(leg_list_node_t* leg) {
  leg_t curr_arinc_leg = leg->data.leg;

  double true_brng_rad = double(curr_arinc_leg.outbd_crs_deg) * geo::DEG_TO_RAD;

  if (!curr_arinc_leg.outbd_crs_true) {
    true_brng_rad -= curr_arinc_leg.get_mag_var_deg() * geo::DEG_TO_RAD;
  }

  geo::point leg_end =
      geo::get_pos_from_brng_dist(leg->data.misc_data.start, true_brng_rad,
                                  double(curr_arinc_leg.outbd_dist_time));

  leg->data.misc_data.true_trk_deg = true_brng_rad * geo::RAD_TO_DEG;
  leg->data.misc_data.is_arc = false;
  leg->data.misc_data.turn_rad_nm = TURN_RADIUS_NM;
  leg->data.misc_data.is_finite = true;
  leg->data.misc_data.end = leg_end;
}

void FplnInt::calculate_dme_leg(leg_list_node_t* leg, double hdg_trk_diff) {
  leg_t curr_arinc_leg = leg->data.leg;

  double true_brng_rad = double(curr_arinc_leg.outbd_crs_deg) * geo::DEG_TO_RAD;
  if (!curr_arinc_leg.outbd_crs_true) {
    true_brng_rad -= curr_arinc_leg.get_mag_var_deg() * geo::DEG_TO_RAD;
  }
  if (curr_arinc_leg.leg_type == "VD") {
    true_brng_rad += hdg_trk_diff;
  }

  geo::point end = get_dme_end_point(leg->data.misc_data.start, true_brng_rad,
                                     curr_arinc_leg.recd_navaid.data.pos,
                                     curr_arinc_leg.outbd_dist_time);

  leg->data.misc_data.is_arc = false;
  leg->data.misc_data.is_finite = true;
  leg->data.misc_data.end = end;
  leg->data.misc_data.is_bypassed = false;
  leg->data.misc_data.true_trk_deg = true_brng_rad * geo::RAD_TO_DEG;
  leg->data.misc_data.turn_rad_nm = TURN_RADIUS_NM;

  leg->data.misc_data.set_calc_wpt(get_xd_wpt(
      end, leg->data.leg.recd_navaid.id, int(leg->data.leg.outbd_dist_time)));
}

void FplnInt::calculate_crs_trk_dir_leg(leg_list_node_t* leg) {
  leg_t curr_arinc_leg = leg->data.leg;

  geo::point curr_start = leg->data.misc_data.start;
  geo::point curr_end = curr_arinc_leg.main_fix.data.pos;

  double brng_rad = curr_start.get_gc_bearing_rad(curr_end);
  double dist_nm = curr_start.get_gc_dist_nm(curr_end);
  double turn_rad_nm = TURN_RADIUS_NM;

  leg->data.misc_data.true_trk_deg = brng_rad * geo::RAD_TO_DEG;

  if (leg->data.misc_data.true_trk_deg < 0)
    leg->data.misc_data.true_trk_deg += 360;

  leg->data.leg.outbd_dist_time = dist_nm;
  leg->data.leg.outbd_dist_as_time = false;

  leg->data.misc_data.is_arc = false;
  leg->data.misc_data.is_finite = true;
  leg->data.misc_data.end = curr_end;
  leg->data.misc_data.turn_rad_nm = turn_rad_nm;

  leg->data.misc_data.set_calc_wpt(leg->data.leg.main_fix);
}

void FplnInt::calculate_leg(leg_list_node_t* leg, double hdg_trk_diff,
                            double curr_alt_ft) {
  leg_t curr_arinc_leg = leg->data.leg;

  leg->data.misc_data = {};
  leg->data.misc_data.turn_rad_nm = -1;

  if (curr_arinc_leg.has_main_fix &&
      curr_arinc_leg.main_fix.data.type == libnav::NavaidType::RWY) {
    leg->data.misc_data.is_rwy = true;
  }

  leg_list_node_t* prev_leg = leg->prev;
  bool intc_bp = false;

  if (prev_leg != &(leg_list_.head)) {
    if (prev_leg->data.misc_data.is_bypassed) {
      intc_bp = true;
    }
    if (prev_leg->data.misc_data.turn_rad_nm < 0) {
      prev_leg = prev_leg->prev;
    }
  }

  if (prev_leg != &(leg_list_.head) && !prev_leg->data.is_discon) {
    double m_var = get_leg_mag_var_deg(leg);
    leg->data.misc_data.is_bypassed = get_leg_start(
        prev_leg->data.misc_data, prev_leg->data.leg, curr_arinc_leg, m_var,
        hdg_trk_diff, &leg->data.misc_data.start,
        &leg->data.misc_data.is_to_inhibited,
        &prev_leg->data.misc_data.turn_rad_nm);

    if (prev_leg->data.misc_data.turn_rad_nm != -1) {
      if (!intc_bp && (prev_leg->data.leg.leg_type == "VI" ||
                       prev_leg->data.leg.leg_type == "CI")) {
        set_xi_leg(leg);
      }
    }
  }

  if (leg->data.misc_data.is_bypassed) {
    leg->data.misc_data = prev_leg->data.misc_data;
    leg->data.misc_data.is_bypassed = true;
    if (leg->data.leg.has_main_fix)
      leg->data.misc_data.set_calc_wpt(leg->data.leg.main_fix);
    leg->data.misc_data.is_finite = true;
    leg->data.misc_data.turn_rad_nm = -1;
    return;
  }

  if (curr_arinc_leg.leg_type == "IF") {
    geo::point main_fix_pos = curr_arinc_leg.main_fix.data.pos;
    leg->data.misc_data.is_arc = false;
    leg->data.misc_data.is_finite = true;
    leg->data.misc_data.start = main_fix_pos;
    leg->data.misc_data.end = main_fix_pos;
    leg->data.misc_data.turn_rad_nm = 0;
    leg->data.misc_data.set_calc_wpt(curr_arinc_leg.main_fix);
  } else if (curr_arinc_leg.leg_type == "CA" ||
             curr_arinc_leg.leg_type == "VA" ||
             curr_arinc_leg.leg_type == "FA") {
    calculate_alt_leg(leg, hdg_trk_diff, curr_alt_ft);
  } else if (curr_arinc_leg.leg_type == "FC") {
    calculate_fc_leg(leg);
  } else if (curr_arinc_leg.leg_type == "VI" ||
             curr_arinc_leg.leg_type == "CI") {
    calculate_intc_leg(leg, hdg_trk_diff);
  } else if (curr_arinc_leg.leg_type == "TF" ||
             curr_arinc_leg.leg_type == "CF" ||
             curr_arinc_leg.leg_type == "DF") {
    calculate_crs_trk_dir_leg(leg);
  } else if (curr_arinc_leg.leg_type == "FD" ||
             curr_arinc_leg.leg_type == "CD" ||
             curr_arinc_leg.leg_type == "VD") {
    calculate_dme_leg(leg, hdg_trk_diff);
  }

  if (leg->data.misc_data.true_trk_deg > 360)
    leg->data.misc_data.true_trk_deg -= 360;

  if (prev_leg != &(leg_list_.head) && !prev_leg->data.is_discon) {
    if (prev_leg->data.misc_data.turn_rad_nm != -1) {
      if (TURN_OFFS_LEGS.find(curr_arinc_leg.leg_type) ==
              TURN_OFFS_LEGS.end() &&
          LEGS_CALC.find(prev_leg->data.leg.leg_type) != LEGS_CALC.end() &&
          !prev_leg->data.misc_data.is_to_inhibited) {
        set_turn_offset(leg, prev_leg);
      }
    }
  }

  if (leg->prev != &(leg_list_.head) && leg->prev->data.is_discon) {
    leg->data.misc_data.has_disc = true;
  }
}
}  // namespace test
