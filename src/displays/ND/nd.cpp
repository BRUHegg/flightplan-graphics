/*
    This project is licensed under
    Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

    A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

    This source file contains definitions of classes, functions, etc
    used in the ND implementation. Author: discord/bruh4096#4512
*/

#include "nd.hpp"

#include <cassert>
#include <cstddef>

#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace {

constexpr size_t N_LEG_PROJ_CACHE_SZ = 200;
constexpr size_t N_LN_JOINT_CACHE_SZ = N_LEG_PROJ_CACHE_SZ - 1;
constexpr size_t N_PROJ_CACHE_SZ = 202;
constexpr size_t DEP_RWY_PROJ_IDX = N_PROJ_CACHE_SZ - 2;
constexpr size_t ARR_RWY_PROJ_IDX = N_PROJ_CACHE_SZ - 1;
constexpr size_t N_ND_SDS =
    fms_core::N_INTFCS;  // Essentially this is how many NDs we can have
constexpr size_t N_MP_DATA_SZ = N_ND_SDS * fms_core::N_FPL_SYS_RTES;
constexpr size_t N_EFIS_TYPE_CACHE_SZ = 150;  // Number of fetched POIs per type
constexpr size_t N_EFIS_MAP_ALTN_APTS = 4;
constexpr double N_MAX_DIST_NM = 640;
constexpr double ND_DEFAULT_RNG_NM = 10;
// Percentage of resolution that translates into full range:
static std::unordered_map<fms_core::NDMode, double, util::enum_class_hash_t>
    ND_RNG_FULL_RES_COEFF = {{fms_core::NDMode::MAP, 0.7},
                             {fms_core::NDMode::PLAN, 0.4}};
// Arc angles for TFC rings
const std::vector<std::pair<double, double>> ND_MAP_TFC_ARC_ANGLES = {
    {0, 0},
    {1 / M_1_PI, 2 / M_1_PI},
    {1 / M_1_PI, 2 / M_1_PI},
    {1.2 / M_1_PI, 1.8 / M_1_PI}};
constexpr int N_MAP_TRK_DASH = 4;
// Percentage of horisontal resolution that translates into runway width
constexpr double DEFAULT_RWY_WIDTH = 0.015;
// Percentage of horisontal resolution that translates into thikness of runway
// side line
constexpr double RWY_SIDE_THICK = 0.0025;
// Length of extended runway center line in nautical miles(one direction)
constexpr double N_RWY_EXT_CTR_LINE_NM = 16;
// For extended centerline dashes:
const double RWY_EXT_CTR_LINE_DASH[] = {9.0, 9.0};
constexpr int N_RWY_DASHES =
    sizeof(RWY_EXT_CTR_LINE_DASH) / sizeof(RWY_EXT_CTR_LINE_DASH[0]);
// For flightplan line dashes:
const double FPLN_LN_DASH[] = {9.0, 9.0};
constexpr int N_FPLN_DASHES = sizeof(FPLN_LN_DASH) / sizeof(FPLN_LN_DASH[0]);
constexpr int V_RTE_NOT_DRAWN = -1;

constexpr double ND_WPT_FONT_SZ = 23;
constexpr double ND_ACT_INFO_MAIN_FONT_SZ = 21;
constexpr double ND_ACT_INFO_DIST_FONT_SZ = 19;
constexpr double ND_SPD_BIG_FONT_SZ = 26;
constexpr double ND_SPD_SMALL_FONT_SZ = 21;
constexpr double GS_THRESH_BIG_KTS = 30;
constexpr double TAS_DISPL_THRESH_KTS = 100;
// Inverse of percentage of resolution that contributes to the scaling factor of
// waypoint image
constexpr double WPT_SCALE_FACT = 900;
// Percentage of horizontal resolution that translates into radius of pseudo
// waypoint label
constexpr double PSEUDO_WPT_RADIUS_RAT = 0.007;
constexpr double PSEUDO_WPT_THICK_RAT = 0.0025;
// Percentage of horisontal resolution that translates into magenta line width
constexpr double ND_FPL_LINE_THICK = 0.0042;
constexpr double ND_PRJ_CTR_V_OFFS_PLAN =
    0.01;  // Offset from the display center
constexpr double ND_PRJ_CTR_V_OFFS_MAP =
    0.296;  // Offset from the display center
constexpr double MAP_TRK_LN_LN_INN = 0.706;
constexpr double MAP_TRK_LN_LN_OUT = 0.025;
constexpr double MAP_HTRK_FONT_SZ = 51;
constexpr double MAP_HTRK_STG_TXT_LOFFS = 0.1;
constexpr double MAP_HTRK_STG_TXT_VOFFS = 0.033;
constexpr double MAP_HTRK_STG_FONT_SZ = 38;
constexpr double MAP_TRK_DASH_OFFS = 0.01;
constexpr double MAP_HDG_TRI_VOFFS = 0.718;
// EFIS filters:
constexpr double EFIS_REFRESH_ABSD =
    0.012;  // Corresponds to a distance of about 20 nm
constexpr double EFIS_POI_NAME_FNT_SZ = ND_WPT_FONT_SZ;
constexpr double EFIS_ARPT_SC = 0.3;
constexpr double EFIS_VHF_SC = EFIS_ARPT_SC * 4.0 / 3.0;
// Route drawing
constexpr geom::vect2_t FIX_NAME_OFFS = {0.02, 0.03};
const std::vector<geom::vect3_t> ND_RTE_CLRS = {
    cairo_utils::WHITE, cairo_utils::MAGENTA, cairo_utils::ND_CYAN};
// Active leg
constexpr geom::vect2_t ACT_LEG_NAME_OFFS = {0.911, 0.03};
constexpr geom::vect2_t ACT_LEG_TIME_OFFS = {0.911, 0.05};
constexpr geom::vect2_t ACT_LEG_DIST_OFFS = {0.911, 0.07};
constexpr geom::vect2_t ACT_LEG_NM_OFFS = {0.96, 0.07};
// Speed
constexpr geom::vect2_t GS_OFFS = {0.061, 0.034};
constexpr geom::vect2_t GS_TEXT_OFFS = {0.003, 0.034};
constexpr geom::vect2_t TAS_OFFS = {0.15, 0.034};
constexpr geom::vect2_t TAS_TEXT_OFFS = {0.079, 0.034};
// General:
constexpr geom::vect2_t MAP_AC_TRI_SC = {2, 2};
constexpr geom::vect2_t MAP_HTK_BOX_SC = {0.034, 0.073};
constexpr geom::vect2_t MAP_HTK_TXT_POS = {0.5, 0.029};
constexpr geom::vect2_t MAP_HDG_TRI_SC = {1, 0.8};
constexpr geom::vect2_t MAP_RNG_OFFS = {-0.014, -0.012};

constexpr geom::vect3_t ND_BCKGRND_CLR = cairo_utils::BLACK;

// Names for the UI logic
const std::string ND_TRKUP = "TRK";
const std::string ND_HDGUP = "HDG";
const std::string ND_HTRK_MAG = "MAG";
const std::string ND_HTRK_TRU = "TRU";


const std::vector<double> ND_RANGES_NM = {10, 20, 40, 80, 160, 320, 640};
// Only the supported modes are in ND_MDS
const std::vector<fms_core::NDMode> ND_MDS = {fms_core::NDMode::MAP,
                                          fms_core::NDMode::PLAN};
constexpr double RNG_DEC_1_NM = 2.5;
constexpr double RNG_DEC_2_NM = 1.25;
}  // namespace

namespace fms_displays {
bool poi_data_t::init_ptr(labeled_point_with_dist_t** ptr) {
  labeled_point_with_dist_t* p_new =
      new labeled_point_with_dist_t[N_EFIS_TYPE_CACHE_SZ];
  *ptr = p_new;
  if (p_new == nullptr) {
    return false;
  }
  return true;
}

bool poi_data_t::init() {
  bool ret = init_ptr(&arpts) && init_ptr(&waypts) && init_ptr(&vors_dmes) &&
             init_ptr(&vordmes);
  if (!ret) {
    destroy();
  }
  return ret;
}

void poi_data_t::destroy() {
  delete[] arpts;
  delete[] waypts;
  delete[] vors_dmes;
  delete[] vordmes;
  n_arpts = 0;
  n_waypts = 0;
  n_vors_dmes = 0;
  n_vordmes = 0;
}

bool labeled_point_with_dist_cmp_t::operator()(
    const labeled_point_with_dist_t& pa,
    const labeled_point_with_dist_t& pb) const noexcept {
  return pa.dist_ctr < pb.dist_ctr;
}

// map_poi_container_t definitions:
map_poi_container_t::map_poi_container_t(
    std::shared_ptr<libnav::ArptDB> arpt_ptr,
    std::shared_ptr<libnav::NavaidDB> navaid_ptr) {
  arpt_db_ptr = arpt_ptr;
  navaid_db_ptr = navaid_ptr;
}

void map_poi_container_t::set_add(
    std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t>& st,
    labeled_point_with_dist_t& pt) {
  if (st.size() == N_EFIS_TYPE_CACHE_SZ &&
      pt.dist_ctr < st.rbegin()->dist_ctr) {
    st.erase(*st.rbegin());
  }
  if (st.size() == N_EFIS_TYPE_CACHE_SZ) {
    return;
  }
  st.insert(pt);
}

void map_poi_container_t::set_to_array(
    std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t>& st,
    labeled_point_with_dist_t* arr, size_t* arr_sz) {
  *arr_sz = 0;
  for (auto i : st) {
    arr[*arr_sz] = i;
    *arr_sz = *arr_sz + 1;
  }
}

void map_poi_container_t::project_array(labeled_point_with_dist_t* src,
                                        labeled_point_with_dist_t* dst,
                                        size_t sz, size_t* sz_tgt,
                                        geo::point curr_pos, double rot_rad) {
  for (size_t i = 0; i < sz; i++) {
    geo::point src_pos = {src[i].point.pos.x, src[i].point.pos.y};
    double dist = curr_pos.get_gc_dist_nm(src_pos);
    double brng = curr_pos.get_gc_bearing_rad(src_pos) + rot_rad;
    dst[i].point.pos = geom::get_projection(brng, dist);
    dst[i].point.name = src[i].point.name;
    dst[i].dist_ctr = dist;
  }
  *sz_tgt = sz;
}

void map_poi_container_t::fetch_arpts(geo::point curr_pos) {
  std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t> st;
  for (auto i : arpt_db_ptr->get_arpt_db()) {
    double dist = curr_pos.get_gc_dist_nm(i.second.pos);
    if (dist > ND_RANGES_NM.back()) {
      continue;
    } else if (n_arpts == N_EFIS_TYPE_CACHE_SZ) {
      break;
    }
    labeled_point_with_dist_t curr = {
        {{i.second.pos.lat_rad, i.second.pos.lon_rad}, i.first}, dist};
    set_add(st, curr);
  }
  set_to_array(st, arpts, &n_arpts);
}

void map_poi_container_t::fetch_navaids(geo::point curr_pos) {
  std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t> st_wpt;
  std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t> st_vd;
  std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t> st_v_d;

  for (auto i : navaid_db_ptr->get_db()) {
    for (auto j : i.second) {
      double dist = curr_pos.get_gc_dist_nm(j.pos);
      if (dist > ND_RANGES_NM.back()) {
        continue;
      }
      std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t>*
          tgt_ptr = nullptr;

      if (int(j.type) & int(libnav::NavaidType::WAYPOINT)) {
        tgt_ptr = &st_wpt;
      } else if (int(j.type) & (int(libnav::NavaidType::VOR_DME) +
                                int(libnav::NavaidType::ILS_DME))) {
        tgt_ptr = &st_vd;
      } else if (int(j.type) & (int(libnav::NavaidType::VHF_NAVAID))) {
        tgt_ptr = &st_v_d;
      }

      if (tgt_ptr == nullptr) {
        continue;
      }
      labeled_point_with_dist_t curr = {
          {{j.pos.lat_rad, j.pos.lon_rad}, i.first}, dist};
      set_add(*tgt_ptr, curr);
    }
  }
  set_to_array(st_wpt, waypts, &n_waypts);
  set_to_array(st_v_d, vordmes, &n_vordmes);
  set_to_array(st_vd, vors_dmes, &n_vors_dmes);
}

void map_poi_container_t::fetch(geo::point curr_pos) {
  fetch_arpts(curr_pos);
  fetch_navaids(curr_pos);
}

void map_poi_container_t::project(poi_data_t& tgt, geo::point curr_pos,
                                  double rot_add_rad) {
  project_array(arpts, tgt.arpts, n_arpts, &tgt.n_arpts, curr_pos, rot_add_rad);
  project_array(waypts, tgt.waypts, n_waypts, &tgt.n_waypts, curr_pos,
                rot_add_rad);
  project_array(vordmes, tgt.vordmes, n_vordmes, &tgt.n_vordmes, curr_pos,
                rot_add_rad);
  project_array(vors_dmes, tgt.vors_dmes, n_vors_dmes, &tgt.n_vors_dmes,
                curr_pos, rot_add_rad);
}

// leg_proj_t definitions:

std::string leg_proj_t::get_draw_nm() {
  std::string name_draw;
  for (size_t j = 0; j < end_nm.size(); j++) {
    char curr_char = end_nm[j];
    if (curr_char != '(' && curr_char != ')') {
      name_draw.push_back(curr_char);
    }
  }

  return name_draw;
}

// map_data_t definitions:

bool map_data_t::create() {
  proj_legs = new leg_proj_t[N_PROJ_CACHE_SZ];
  if (proj_legs == nullptr) {
    return false;
  }
  line_joints = new geom::line_joint_t[N_LN_JOINT_CACHE_SZ];
  if (line_joints == nullptr) {
    return false;
  }
  n_act_proj_legs = 0;
  n_act_joints = 0;

  return true;
}

void map_data_t::destroy() {
  delete[] proj_legs;
  delete[] line_joints;
  n_act_proj_legs = 0;
  n_act_joints = 0;
}

// NDData member funcrion definitions:

// Public member functions:

NDData::NDData(std::shared_ptr<fms_core::FPLSys> fpl_sys)
    : poi_data_(fpl_sys->get_arpt_db_ptr(), fpl_sys->get_navaid_db_ptr()) {
  fpl_sys_ptr_ = fpl_sys;
  for (std::size_t i = 0; i < fpl_sys->get_cnt_flplns(); ++i) {
    fpl_vec_.push_back(fpl_sys->get_fpln_ptr(i));
  }

  leg_data_ = std::vector<fms_core::nd_leg_data_t*>(fms_core::N_FPL_SYS_RTES);
  leg_data_sz_ = std::vector<size_t>(fms_core::N_FPL_SYS_RTES, 0);

  rte_draw_seq_ = std::vector<std::vector<int>>(
      N_ND_SDS, std::vector<int>(fms_core::N_FPL_SYS_RTES, V_RTE_NOT_DRAWN));

  mp_data_ = std::vector<map_data_t>(N_MP_DATA_SZ);
  act_leg_idx_sd_ = std::vector<int>(N_MP_DATA_SZ, 0);

  curr_modes_ = std::vector<std::pair<fms_core::NDMode, bool>>(
      N_ND_SDS, {fms_core::DFLT_ND_MODE, false});
  ac_pos_projected_ = std::vector<geom::vect2_t>(N_ND_SDS, {0, 0});
  ac_pos_ok_ = std::vector<bool>(N_ND_SDS, 0);
  range_index_ = std::vector<size_t>(N_ND_SDS, 0);
  map_center_ = std::vector<geo::point>(N_ND_SDS, {0, 0});

  fpl_id_last_ = std::vector<double>(fms_core::N_FPL_SYS_RTES, 0);

  heading_data_ = {};

  has_dep_rwy_ = std::vector<bool>(fms_core::N_FPL_SYS_RTES, false);
  has_arr_rwy_ = std::vector<bool>(fms_core::N_FPL_SYS_RTES, false);

  efis_sel_ = std::vector<efis_selection_t>(N_ND_SDS, efis_selection_t{});

  pois_projected_[0] = {};
  pois_projected_[1] = {};

  act_leg_idx_ = std::vector<int>(fms_core::N_FPL_SYS_RTES, -1);
  ac_pos_last_ = {N_MAX_DIST_NM, N_MAX_DIST_NM};
}

bool NDData::init() {
  std::unique_lock lk(main_mutex_);
  for (size_t i = 0; i < fms_core::N_FPL_SYS_RTES; i++) {
    leg_data_[i] = new fms_core::nd_leg_data_t[N_LEG_PROJ_CACHE_SZ];
    if (leg_data_[i] == nullptr) {
      destroy();
      return false;
    }
  }
  for (size_t i = 0; i < N_MP_DATA_SZ; i++) {
    if (!mp_data_[i].create()) {
      destroy();
      return false;
    }
  }
  if (!poi_data_.init() || !pois_projected_[0].init() ||
      !pois_projected_[1].init()) {
    return false;
  }
  return true;
}

void NDData::set_th_up() { 
  std::unique_lock lk(main_mutex_);
  trk_up_ = !trk_up_; 
}

bool NDData::get_th_up() { 
  std::shared_lock lk(main_mutex_);
  return trk_up_; 
}

void NDData::toggle_efis_arpt_sd(size_t sd_idx) {
  assert(sd_idx < N_ND_SDS);
  std::unique_lock lk(main_mutex_);
  efis_sel_[sd_idx].arpt_on = !efis_sel_[sd_idx].arpt_on;
}

void NDData::toggle_efis_sta_sd(size_t sd_idx) {
  assert(sd_idx < N_ND_SDS);
  std::unique_lock lk(main_mutex_);
  efis_sel_[sd_idx].sta_on = !efis_sel_[sd_idx].sta_on;
}

efis_selection_t NDData::get_efis_sts_sd(size_t sd_idx) {
  assert(sd_idx < N_ND_SDS);
  std::shared_lock lk(main_mutex_);
  return efis_sel_[sd_idx];
}

void NDData::set_mode(size_t sd_idx, fms_core::NDMode md, bool set_ctr) {
  assert(sd_idx < N_ND_SDS);
  std::unique_lock lk(main_mutex_);
  curr_modes_[sd_idx].first = md;
  if (md == fms_core::NDMode::PLAN)
    curr_modes_[sd_idx].second = false;
  else if (set_ctr)
    curr_modes_[sd_idx].second = !curr_modes_[sd_idx].second;
  fpl_sys_ptr_->set_nd_mode(md, sd_idx);
}

std::pair<fms_core::NDMode, bool> NDData::get_mode(size_t sd_idx) const {
  assert(sd_idx < N_ND_SDS);
  std::shared_lock lk(main_mutex_);
  return curr_modes_[sd_idx];
}

std::vector<int> NDData::get_rte_draw_seq(size_t sd_idx) {
  assert(sd_idx < N_ND_SDS);
  std::shared_lock lk(main_mutex_);
  return rte_draw_seq_[sd_idx];
}

size_t NDData::get_proj_legs(leg_proj_t** out, size_t sd_idx, size_t dt_idx) {
  std::shared_lock lk(main_mutex_);
  *out = mp_data_[sd_idx + dt_idx * N_ND_SDS].proj_legs;
  return mp_data_[sd_idx + dt_idx * N_ND_SDS].n_act_proj_legs;
}

int NDData::get_act_leg_idx(size_t sd_idx) { 
  std::shared_lock lk(main_mutex_);
  return act_leg_idx_sd_[sd_idx]; 
}

bool NDData::get_ac_pos(geom::vect2_t* out, size_t sd_idx) {
  std::shared_lock lk(main_mutex_);
  if (!ac_pos_ok_[sd_idx]) return false;

  *out = ac_pos_projected_[sd_idx];
  return true;
}

double NDData::get_hdg_trk() const { 
  std::shared_lock lk(main_mutex_);
  return get_cr_rot(); 
}

fms_core::hdg_info_t NDData::get_hdg_data() { 
  std::shared_lock lk(main_mutex_);
  return heading_data_; 
}

fms_core::spd_info_t NDData::get_spd_data() {
  std::shared_lock lk(main_mutex_);
  return fpl_sys_ptr_->get_spd_info();
}

fms_core::act_leg_info_t NDData::get_act_leg_info() {
  std::shared_lock lk(main_mutex_);
  return fpl_sys_ptr_->get_act_leg_info();
}

bool NDData::has_dep_rwy(size_t idx) { 
  std::shared_lock lk(main_mutex_);
  return has_dep_rwy_[idx]; 
}

bool NDData::has_arr_rwy(size_t idx) { 
  std::shared_lock lk(main_mutex_);
  return has_arr_rwy_[idx]; 
}

void NDData::switch_range(bool down, size_t sd_idx) {
  std::unique_lock lk(main_mutex_);
  if (down) {
    if (range_index_[sd_idx]) range_index_[sd_idx]--;
  } else {
    if (range_index_[sd_idx] + 1 < ND_RANGES_NM.size()) range_index_[sd_idx]++;
  }
}

double NDData::get_range(std::size_t sd_idx) const noexcept {
  std::shared_lock lk(main_mutex_);
  return get_range_impl(sd_idx);
}

// POI functions

size_t NDData::get_num_poi_arpts() { 
  std::shared_lock lk(main_mutex_);
  return poi_data_.n_arpts; 
}

size_t NDData::get_num_poi_waypts() { 
  std::shared_lock lk(main_mutex_);
  return poi_data_.n_waypts; 
}

size_t NDData::get_num_poi_vordmes() { 
  std::shared_lock lk(main_mutex_);
  return poi_data_.n_vordmes; 
}

size_t NDData::get_num_poi_vhf_not_vordmes() { 
  std::shared_lock lk(main_mutex_);
  return poi_data_.n_vors_dmes; 
}

// Get ith POI

labeled_point_with_dist_t NDData::get_arpt(size_t i) {
  std::shared_lock lk(main_mutex_);
  assert(i < pois_projected_[idx_proj_act_].n_arpts);
  return pois_projected_[idx_proj_act_].arpts[i];
}

labeled_point_with_dist_t NDData::get_waypt(size_t i) {
  std::shared_lock lk(main_mutex_);
  assert(i < pois_projected_[idx_proj_act_].n_waypts);
  return pois_projected_[idx_proj_act_].waypts[i];
}

labeled_point_with_dist_t NDData::get_vordme(size_t i) {
  std::shared_lock lk(main_mutex_);
  assert(i < pois_projected_[idx_proj_act_].n_vordmes);
  return pois_projected_[idx_proj_act_].vordmes[i];
}

labeled_point_with_dist_t NDData::get_vhf_not_vordme(size_t i) {
  std::shared_lock lk(main_mutex_);
  assert(i < pois_projected_[idx_proj_act_].n_vors_dmes);
  return pois_projected_[idx_proj_act_].vors_dmes[i];
}

void NDData::update() {
  std::unique_lock lk(main_mutex_);
  heading_data_ = fpl_sys_ptr_->get_hdg_info();
  update_rte_draw_seq();
  for (size_t i = 0; i < N_ND_SDS; i++) {
    update_ctr(i);
    ac_pos_ok_[i] = project_ac_pos(i);
  }
  for (size_t i = 0; i < fms_core::N_FPL_SYS_RTES; i++) update_fpl(i);
  geo::point curr_pos = fpl_sys_ptr_->get_ac_pos();
  double abs_diff = abs(curr_pos.lat_rad - ac_pos_last_.lat_rad) +
                    abs(curr_pos.lon_rad - ac_pos_last_.lon_rad);
  if (abs_diff > EFIS_REFRESH_ABSD) {
    poi_data_.fetch(curr_pos);
  }
  poi_data_.project(pois_projected_[!idx_proj_act_], curr_pos, get_cr_rot());
  idx_proj_act_ = !idx_proj_act_;
  ac_pos_last_ = curr_pos;
}

void NDData::destroy() {
  std::unique_lock lk(main_mutex_);
  for (size_t i = 0; i < fms_core::N_FPL_SYS_RTES; i++) delete[] leg_data_[i];
  for (size_t i = 0; i < N_MP_DATA_SZ; i++) mp_data_[i].destroy();
  pois_projected_[0].destroy();
  pois_projected_[1].destroy();
  poi_data_.destroy();
}

// Private member functions:
// Static member functions:

bool NDData::bound_check(double x1, double x2, double rng) noexcept {
  return (x1 < rng && x2 >= rng) || (x1 >= rng && x2 < rng) ||
         (x1 > -rng && x2 <= -rng) || (x2 > -rng && x1 <= -rng) ||
         (abs(x1) <= rng && abs(x2) <= rng);
}

nd_util_idx_t NDData::get_util_idx(std::size_t gn_idx) noexcept {
  return {gn_idx % N_ND_SDS, gn_idx / N_ND_SDS};
}

// Non-static member functions:

double NDData::get_range_impl(std::size_t sd_idx) const noexcept {
  return ND_RANGES_NM[range_index_[sd_idx]];
}

double NDData::get_cr_rot() const noexcept {
  if (trk_up_) return -(heading_data_.brng_tru_rad + heading_data_.magvar_rad);
  return -(heading_data_.brng_tru_rad + heading_data_.magvar_rad +
           heading_data_.slip_rad);
}

std::pair<geo::point, double> NDData::get_proj_params(
  std::size_t sd_idx) const noexcept {
  assert(sd_idx < curr_modes_.size());
  if (curr_modes_[sd_idx].first == fms_core::NDMode::PLAN) return {map_center_[sd_idx], 0};
  return {fpl_sys_ptr_->get_ac_pos(), get_cr_rot()};
}

bool NDData::in_view(geom::vect2_t start, geom::vect2_t end, size_t sd_idx) const noexcept {
  assert(sd_idx < curr_modes_.size());
  double a = start.x - end.x;
  double rng = get_range_impl(sd_idx);
  if (curr_modes_[sd_idx].second || curr_modes_[sd_idx].first == fms_core::NDMode::PLAN)
    rng /= 2;

  if (a != 0) {
    double b = start.y - end.y;
    double k = b / a;
    double c = start.y - start.x * k;
    double y1 = k * -rng + c;
    double y2 = k * rng + c;

    if (bound_check(y1, y2, rng) && bound_check(start.y, end.y, rng) &&
        bound_check(start.x, end.x, rng)) {
      return true;
    }
  } else {
    if (abs(start.x) <= rng) {
      bool out_of_bounds =
          (start.y > rng && end.y > rng) || (start.y < -rng && end.y < -rng);

      return !out_of_bounds;
    }
  }

  return false;
}

void NDData::update_rte_draw_seq() {
  size_t act_idx = fpl_sys_ptr_->get_act_idx();
  bool exec_st = fpl_sys_ptr_->get_exec();
  for (size_t i = 0; i < N_ND_SDS; i++) {
    std::vector<int> tmp(fms_core::N_FPL_SYS_RTES, V_RTE_NOT_DRAWN);
    size_t sel_idx = fpl_sys_ptr_->get_cdu_sel_fpl_idx(i);
    tmp[1] = fms_core::ACT_RTE_IDX;
    if (act_idx != sel_idx)
      tmp[2] = sel_idx;
    else
      tmp[0] = act_idx;
    if (!exec_st || act_idx >= fms_core::N_FPL_SYS_RTES) tmp[0] = -1;
    for (size_t j = 0; j < fms_core::N_FPL_SYS_RTES; j++) {
      if (ND_RTE_CLRS[j] == cairo_utils::WHITE)
        rte_draw_seq_[i][j] = tmp[0];
      else if (ND_RTE_CLRS[j] == cairo_utils::MAGENTA)
        rte_draw_seq_[i][j] = tmp[1];
      else if (ND_RTE_CLRS[j] == cairo_utils::ND_CYAN)
        rte_draw_seq_[i][j] = tmp[2];
    }
  }
}

void NDData::update_ctr(std::size_t sd_idx) {
  geo::point tmp;
  bool ret = fpl_sys_ptr_->get_ctr(&tmp, sd_idx);
  if (!ret) {
    tmp = fpl_sys_ptr_->get_ac_pos();
  }
  map_center_[sd_idx] = tmp;
}

void NDData::project_legs(std::size_t gn_idx) {
  nd_util_idx_t idxs = get_util_idx(gn_idx);

  std::pair<geo::point, double> mp_prm = get_proj_params(idxs.sd_idx);
  geo::point map_ctr = mp_prm.first;

  leg_proj_t* dst = mp_data_[gn_idx].proj_legs;
  geom::line_joint_t* dst_joint = mp_data_[gn_idx].line_joints;

  std::size_t* sz_ptr = &mp_data_[gn_idx].n_act_proj_legs;
  std::size_t* sz_ptr_joint = &mp_data_[gn_idx].n_act_joints;

  *sz_ptr = 0;
  *sz_ptr_joint = 0;
  act_leg_idx_sd_[gn_idx] = -1;

  bool prev_skipped = false;
  bool prev_bypassed = false;

  for (std::size_t i = 0; i < leg_data_sz_[idxs.dt_idx]; i++) {
    if (i >= N_LEG_PROJ_CACHE_SZ) break;

    dst[*sz_ptr].joint = nullptr;

    if ((!leg_data_[idxs.dt_idx][i].leg_data.is_finite &&
         !leg_data_[idxs.dt_idx][i].leg_data.is_bypassed) ||
        leg_data_[idxs.dt_idx][i].leg_data.is_arc)
      continue;

    if (leg_data_[idxs.dt_idx][i].leg_data.is_finite &&
        leg_data_[idxs.dt_idx][i].leg_data.has_calc_wpt) {
      geo::point end_wpt =
          leg_data_[idxs.dt_idx][i].leg_data.calc_wpt.data.pos;
      std::string end_name = leg_data_[idxs.dt_idx][i].leg_data.calc_wpt.id;

      double dist_wpt = map_ctr.get_gc_dist_nm(end_wpt);
      double brng_wpt = map_ctr.get_gc_bearing_rad(end_wpt) + mp_prm.second;

      dst[*sz_ptr].end_wpt = {dist_wpt * sin(brng_wpt),
                              dist_wpt * cos(brng_wpt)};
      dst[*sz_ptr].end_nm = end_name;
      dst[*sz_ptr].is_finite = true;
      dst[*sz_ptr].is_arc = leg_data_[idxs.dt_idx][i].leg_data.is_arc;
      dst[*sz_ptr].has_path = false;
    }

    if (leg_data_[idxs.dt_idx][i].leg_data.turn_rad_nm != -1) {
      geom::vect2_t start_proj = geom::project_point(
          leg_data_[idxs.dt_idx][i].leg_data.start, map_ctr, mp_prm.second);
      geom::vect2_t end_proj = geom::project_point(
          leg_data_[idxs.dt_idx][i].leg_data.end, map_ctr, mp_prm.second);

      if (!in_view(start_proj, end_proj, idxs.sd_idx)) {
        prev_skipped = true;
        continue;
      }

      if (*sz_ptr && !prev_skipped &&
          !leg_data_[idxs.dt_idx][i].leg_data.has_disc) {
        std::size_t bwd_offs = 1;
        if (prev_bypassed) bwd_offs++;
        geom::vect2_t prev_start = dst[*sz_ptr - bwd_offs].start;
        geom::vect2_t prev_end = dst[*sz_ptr - bwd_offs].end;
        double radius_nm = dst[*sz_ptr - bwd_offs].turn_rad_nm;
        dst_joint[*sz_ptr_joint] = geom::get_line_joint(
            prev_start, prev_end, start_proj, end_proj, radius_nm);

        dst[*sz_ptr].joint = &dst_joint[*sz_ptr_joint];

        *sz_ptr_joint = *sz_ptr_joint + 1;
      }

      dst[*sz_ptr].start = start_proj;
      dst[*sz_ptr].end = end_proj;
      dst[*sz_ptr].has_path = !leg_data_[idxs.dt_idx][i].leg_data.is_bypassed;

      dst[*sz_ptr].is_rwy = leg_data_[idxs.dt_idx][i].leg_data.is_rwy;
      dst[*sz_ptr].turn_rad_nm =
          leg_data_[idxs.dt_idx][i].leg_data.turn_rad_nm;

      if (act_leg_idx_[idxs.dt_idx] != -1 &&
          i == std::size_t(act_leg_idx_[idxs.dt_idx])) {
        act_leg_idx_sd_[gn_idx] = int(*sz_ptr);
      }

      *sz_ptr = *sz_ptr + 1;
      prev_skipped = false;
      prev_bypassed = false;
    } else if (leg_data_[idxs.dt_idx][i].leg_data.is_finite) {
      *sz_ptr = *sz_ptr + 1;
      prev_skipped = !leg_data_[idxs.dt_idx][i].leg_data.is_bypassed;
    }
    prev_bypassed = leg_data_[idxs.dt_idx][i].leg_data.turn_rad_nm == -1;
  }
}

void NDData::project_rwys(std::size_t gn_idx) {
  nd_util_idx_t idxs = get_util_idx(gn_idx);

  std::string dep_rwy = fpl_vec_[idxs.dt_idx]->get_dep_rwy();
  std::string arr_rwy = fpl_vec_[idxs.dt_idx]->get_arr_rwy();

  has_dep_rwy_[idxs.dt_idx] = dep_rwy != "";
  has_arr_rwy_[idxs.dt_idx] = arr_rwy != "";

  if (!has_dep_rwy_[idxs.dt_idx] && !has_arr_rwy_[idxs.dt_idx]) return;

  std::pair<geo::point, double> mp_prm = get_proj_params(idxs.sd_idx);
  geo::point map_ctr = mp_prm.first;

  leg_proj_t* dst = mp_data_[gn_idx].proj_legs;

  if (has_dep_rwy_[idxs.dt_idx]) {
    libnav::runway_entry_t rnw_data;
    bool has_data = fpl_vec_[idxs.dt_idx]->get_dep_rwy_data(&rnw_data);

    if (has_data) {
      dst[DEP_RWY_PROJ_IDX].start =
          geom::project_point(rnw_data.start, map_ctr, mp_prm.second);
      dst[DEP_RWY_PROJ_IDX].end =
          geom::project_point(rnw_data.end, map_ctr, mp_prm.second);
    }
  }

  if (has_arr_rwy_[idxs.dt_idx]) {
    libnav::runway_entry_t rnw_data;
    bool has_data = fpl_vec_[idxs.dt_idx]->get_arr_rwy_data(&rnw_data);

    if (has_data) {
      dst[ARR_RWY_PROJ_IDX].start =
          geom::project_point(rnw_data.start, map_ctr, mp_prm.second);
      dst[ARR_RWY_PROJ_IDX].end =
          geom::project_point(rnw_data.end, map_ctr, mp_prm.second);
    }
  }
}

bool NDData::project_ac_pos(std::size_t sd_idx) {
  geo::point map_ctr = map_center_[sd_idx];
  geo::point curr_pos = fpl_sys_ptr_->get_ac_pos();

  double gc_dist_nm = map_ctr.get_gc_dist_nm(curr_pos);
  double rng = get_range_impl(sd_idx) / 2;

  if (rng < gc_dist_nm) return false;

  double gc_brng_rad = map_ctr.get_gc_bearing_rad(curr_pos);
  ac_pos_projected_[sd_idx] = geom::get_projection(gc_brng_rad, gc_dist_nm);

  return true;
}

void NDData::fetch_legs(std::size_t dt_idx) {
  leg_data_sz_[dt_idx] = fpl_sys_ptr_->get_nd_seg(
      leg_data_[dt_idx], N_LEG_PROJ_CACHE_SZ, dt_idx);
  act_leg_idx_[dt_idx] = fpl_sys_ptr_->get_act_leg_idx();
}

void NDData::update_fpl(std::size_t idx) {
  double id_curr = fpl_vec_[idx]->get_id();

  if (id_curr != fpl_id_last_[idx]) {
    fetch_legs(idx);
  }

  for (std::size_t i = 0; i < N_ND_SDS; i++) {
    project_legs(i + idx * N_ND_SDS);
    project_rwys(i + idx * N_ND_SDS);
  }

  fpl_id_last_[idx] = id_curr;
}

// NDDisplay member functions:

// Public member functions:

NDDisplay::NDDisplay(std::shared_ptr<NDData> data,
                     std::shared_ptr<cairo_utils::texture_manager_t> mngr,
                     cairo_font_face_t* ff, geom::vect2_t pos, geom::vect2_t sz,
                     size_t sd_idx) {
  nd_data = data;
  tex_mngr = mngr;

  font_face = ff;

  scr_pos = pos;
  size = sz;

  side_idx = sd_idx;

  is_trk_up = nd_data->get_th_up();
  is_ctr = false;
  has_tfc = false;
  efis_sel = {};
  rng = nd_data->get_range(side_idx);
}

void NDDisplay::draw(cairo_t* cr) {
  update_mode();
  rng = nd_data->get_range(side_idx);
  update_map_params();
  hdg_data = nd_data->get_hdg_data();

  cairo_utils::draw_rect(cr, scr_pos, size, ND_BCKGRND_CLR);

  draw_background(cr, true);
  draw_efis_filters(cr);
  draw_all_fplns(cr);
  draw_airplane(cr);

  draw_background(cr, false);
  draw_act_leg_info(cr);
  draw_spd_info(cr);
  draw_range(cr);
  // cairo_utils::draw_circle(cr, scr_pos+map_ctr, 3, 3, cairo_utils::MAGENTA);
}

// Private member functions:

void NDDisplay::update_mode() {
  is_trk_up = nd_data->get_th_up();
  std::pair<fms_core::NDMode, bool> md_dt = nd_data->get_mode(side_idx);
  cr_md = md_dt.first;
  is_ctr = md_dt.second;
  efis_sel = nd_data->get_efis_sts_sd(side_idx);
}

void NDDisplay::update_map_params() {
  if (cr_md == fms_core::NDMode::PLAN) {
    curr_rng = rng / 2;
    map_ctr = size.scmul(0.5);
    map_ctr.y += size.y * ND_PRJ_CTR_V_OFFS_PLAN;
  } else if (cr_md == fms_core::NDMode::MAP) {
    curr_rng = rng;
    map_ctr = size.scmul(0.5);
    map_ctr.y += size.y * ND_PRJ_CTR_V_OFFS_MAP;
  }
  scale_factor = size.scmul(ND_RNG_FULL_RES_COEFF[cr_md]).scdiv(curr_rng);
}

geom::vect2_t NDDisplay::get_screen_coords(geom::vect2_t src) {
  src.y *= -1;
  geom::vect2_t out = src * scale_factor + map_ctr + scr_pos;

  return out;
}

void NDDisplay::draw_line_joint(cairo_t* cr, geom::line_joint_t lj,
                                geom::vect3_t ln_clr) {
  double radius_px = lj.turn_radius * scale_factor.x;
  if (lj.tp == geom::JointType::CIRC_CIRC) {
    geom::vect2_t arc1_trans = get_screen_coords(lj.arc1.pos);
    geom::vect2_t arc2_trans = get_screen_coords(lj.arc2.pos);

    cairo_utils::draw_arc(cr, arc1_trans, radius_px, lj.arc1.ang_start_rad,
                          lj.arc1.ang_end_rad, ND_FPL_LINE_THICK * size.x,
                          ln_clr);
    cairo_utils::draw_arc(cr, arc2_trans, radius_px, lj.arc2.ang_start_rad,
                          lj.arc2.ang_end_rad, ND_FPL_LINE_THICK * size.x,
                          ln_clr);
  } else if (lj.tp == geom::JointType::CIRC) {
    geom::vect2_t arc1_trans = get_screen_coords(lj.arc1.pos);
    cairo_utils::draw_arc(cr, arc1_trans, radius_px, lj.arc1.ang_start_rad,
                          lj.arc1.ang_end_rad, ND_FPL_LINE_THICK * size.x,
                          ln_clr);
  }

  geom::vect2_t s_trans = get_screen_coords(lj.line.start);
  geom::vect2_t e_trans = get_screen_coords(lj.line.end);

  cairo_utils::draw_line(cr, s_trans, e_trans, ln_clr,
                         ND_FPL_LINE_THICK * size.x);
}

void NDDisplay::draw_flight_plan(cairo_t* cr, bool draw_labels,
                                 geom::vect3_t ln_clr, size_t idx) {
  leg_proj_t* buf;
  size_t buf_size = nd_data->get_proj_legs(&buf, side_idx, idx);
  int act_leg_idx = -1;
  if (ln_clr == cairo_utils::MAGENTA)
    act_leg_idx = nd_data->get_act_leg_idx(side_idx);

  bool draw_dash = !draw_labels && ln_clr != cairo_utils::MAGENTA;
  if (draw_dash) {
    cairo_save(cr);
    cairo_set_dash(cr, FPLN_LN_DASH, N_FPLN_DASHES, 2);
  }

  for (size_t i = 0; i < buf_size; i++) {
    if (buf[i].is_finite && !buf[i].is_arc) {
      if (!draw_labels) {
        geom::vect2_t start = buf[i].start;
        geom::vect2_t end = buf[i].end;

        if (buf[i].joint != nullptr) {
          geom::line_joint_t curr_joint = *buf[i].joint;

          if (curr_joint.tp != geom::JointType::LINE)
            start = curr_joint.line.start;

          draw_line_joint(cr, curr_joint, ln_clr);
        }

        if (buf[i].has_path) {
          geom::vect2_t s_trans = get_screen_coords(start);
          geom::vect2_t e_trans = get_screen_coords(end);

          cairo_utils::draw_line(cr, s_trans, e_trans, ln_clr,
                                 ND_FPL_LINE_THICK * size.x);
        }
      } else if (buf[i].end_nm.size() && draw_labels) {
        geom::vect2_t end_wpt = buf[i].end_wpt;

        geom::vect2_t ew_trans = get_screen_coords(end_wpt);

        geom::vect2_t text_pos = ew_trans + size * FIX_NAME_OFFS;

        std::string name_draw = buf[i].get_draw_nm();

        bool is_active = false;
        geom::vect3_t tgt_color = cairo_utils::WHITE;

        if (act_leg_idx != -1 && i == size_t(act_leg_idx)) {
          is_active = true;
          tgt_color = cairo_utils::MAGENTA;
        }

        cairo_utils::draw_left_text(cr, font_face, name_draw, text_pos,
                                    tgt_color, ND_WPT_FONT_SZ);

        if (!buf[i].is_rwy && buf[i].end_nm[0] != '(') {
          geom::vect2_t scale = size.scmul(1 / WPT_SCALE_FACT);
          if (is_active) {
            cairo_utils::draw_image(cr, tex_mngr->data[WPT_ACT_NAME], ew_trans,
                                    scale, true);
          } else {
            cairo_utils::draw_image(cr, tex_mngr->data[WPT_INACT_NAME],
                                    ew_trans, scale, true);
          }
        } else if (!buf[i].is_rwy) {
          cairo_utils::draw_circle(cr, ew_trans, size.x * PSEUDO_WPT_RADIUS_RAT,
                                   size.x * PSEUDO_WPT_THICK_RAT, tgt_color);
        }
      }
    }
  }

  if (draw_dash) cairo_restore(cr);
}

void NDDisplay::draw_ext_rwy_ctr_line(cairo_t* cr, leg_proj_t rnw_proj) {
  geom::vect2_t rwy_vec = {rnw_proj.start.x - rnw_proj.end.x,
                           rnw_proj.start.y - rnw_proj.end.y};
  rwy_vec = rwy_vec.get_unit();

  geom::vect2_t end1 = rnw_proj.start + rwy_vec.scmul(N_RWY_EXT_CTR_LINE_NM);
  geom::vect2_t end2 = rnw_proj.end + rwy_vec.scmul(-N_RWY_EXT_CTR_LINE_NM);

  geom::vect2_t rwy_start_trans = get_screen_coords(rnw_proj.start);
  geom::vect2_t rwy_end_trans = get_screen_coords(rnw_proj.end);

  geom::vect2_t end1_trans = get_screen_coords(end1);
  geom::vect2_t end2_trans = get_screen_coords(end2);

  cairo_save(cr);
  cairo_set_dash(cr, RWY_EXT_CTR_LINE_DASH, N_RWY_DASHES, 1);
  cairo_utils::draw_line(cr, end1_trans, rwy_start_trans, cairo_utils::WHITE,
                         RWY_SIDE_THICK * size.x);
  cairo_utils::draw_line(cr, end2_trans, rwy_end_trans, cairo_utils::WHITE,
                         RWY_SIDE_THICK * size.x);
  cairo_restore(cr);
}

void NDDisplay::draw_runway(cairo_t* cr, leg_proj_t rnw_proj) {
  geom::vect2_t start_trans = get_screen_coords(rnw_proj.start);
  geom::vect2_t end_trans = get_screen_coords(rnw_proj.end);

  draw_ext_rwy_ctr_line(cr, rnw_proj);

  geom::vect2_t r_proj_nml_vec = {end_trans.y - start_trans.y,
                                  start_trans.x - end_trans.x};
  r_proj_nml_vec = r_proj_nml_vec.get_unit();

  double half_width = size.x * DEFAULT_RWY_WIDTH / 2;
  geom::vect2_t l_side_start = start_trans + r_proj_nml_vec.scmul(half_width);
  geom::vect2_t l_side_end = end_trans + r_proj_nml_vec.scmul(half_width);

  geom::vect2_t r_side_start =
      start_trans + r_proj_nml_vec.scmul(half_width * -1);
  geom::vect2_t r_side_end = end_trans + r_proj_nml_vec.scmul(half_width * -1);

  cairo_utils::draw_line(cr, l_side_start, l_side_end, cairo_utils::WHITE,
                         RWY_SIDE_THICK * size.x);

  cairo_utils::draw_line(cr, r_side_start, r_side_end, cairo_utils::WHITE,
                         RWY_SIDE_THICK * size.x);
}

void NDDisplay::draw_runways(cairo_t* cr, size_t idx) {
  leg_proj_t* buf;
  size_t buf_size = nd_data->get_proj_legs(&buf, side_idx, idx);
  UNUSED(buf_size);

  if (nd_data->has_dep_rwy(idx)) {
    draw_runway(cr, buf[DEP_RWY_PROJ_IDX]);
  }

  if (nd_data->has_arr_rwy(idx)) {
    draw_runway(cr, buf[ARR_RWY_PROJ_IDX]);
  }
}

void NDDisplay::draw_all_fplns(cairo_t* cr) {
  std::vector<int> fpl_draw_seq = nd_data->get_rte_draw_seq(side_idx);
  for (size_t i = 0; i < fms_core::N_FPL_SYS_RTES; i++) {
    if (fpl_draw_seq[i] == -1) continue;
    draw_runways(cr, fpl_draw_seq[i]);
    draw_flight_plan(cr, false, ND_RTE_CLRS[i], fpl_draw_seq[i]);
    draw_flight_plan(cr, true, ND_RTE_CLRS[i], fpl_draw_seq[i]);
  }
}

void NDDisplay::draw_airplane(cairo_t* cr) {
  geom::vect2_t wpt_scale = size.scmul(1 / WPT_SCALE_FACT);
  if (cr_md == fms_core::NDMode::PLAN) {
    geom::vect2_t pos;
    bool do_drawing = nd_data->get_ac_pos(&pos, side_idx);
    if (do_drawing) {
      geom::vect2_t pos_trans = get_screen_coords(pos);
      cairo_utils::draw_rotated_image(cr, tex_mngr->data[AIRPLANE_NAME],
                                      pos_trans, wpt_scale,
                                      hdg_data.brng_tru_rad);
    }
  } else {
    cairo_surface_t* tgt = tex_mngr->data[MAP_AC_TRI_NAME];
    geom::vect2_t sz =
        cairo_utils::get_surf_sz(tgt) * MAP_AC_TRI_SC * wpt_scale;
    geom::vect2_t sz_shift = {0, 0.5};
    geom::vect2_t pos = scr_pos + map_ctr + sz * sz_shift;
    cairo_utils::draw_image(cr, tgt, pos, MAP_AC_TRI_SC, true);
  }
}

void NDDisplay::draw_htrk(cairo_t* cr) {
  int rot_raw_deg =
      -int(std::round(nd_data->get_hdg_trk() * geom::RAD_TO_DEG)) % 360;
  int rot_deg = (rot_raw_deg + 360) % 360;
  std::string htk_txt = strutils::double_to_str(rot_deg, 0);
  htk_txt = std::string(3 - htk_txt.size(), '0') + htk_txt;
  cairo_utils::draw_centered_text(cr, font_face, htk_txt,
                                  scr_pos + size * MAP_HTK_TXT_POS,
                                  cairo_utils::WHITE, MAP_HTRK_FONT_SZ);

  geom::vect2_t pos_htk =
      geom::vect2_t{0.5 - MAP_HTRK_STG_TXT_LOFFS, MAP_HTRK_STG_TXT_VOFFS};
  geom::vect2_t pos_tmg =
      geom::vect2_t{0.5 + MAP_HTRK_STG_TXT_LOFFS, MAP_HTRK_STG_TXT_VOFFS};
  std::string htk_st_txt = "";
  if (is_trk_up)
    htk_st_txt = ND_TRKUP;
  else
    htk_st_txt = ND_HDGUP;
  cairo_utils::draw_centered_text(cr, font_face, htk_st_txt,
                                  scr_pos + size * pos_htk, cairo_utils::GREEN,
                                  MAP_HTRK_STG_FONT_SZ);
  cairo_utils::draw_centered_text(cr, font_face, "MAG",
                                  scr_pos + size * pos_tmg, cairo_utils::GREEN,
                                  MAP_HTRK_STG_FONT_SZ);
}

void NDDisplay::draw_hdg_tri(cairo_t* cr) {
  if (cr_md == fms_core::NDMode::MAP) {
    double rot_rad = 0;
    if (is_trk_up) rot_rad = -hdg_data.slip_rad;
    geom::vect2_t wpt_scale = size.scmul(1 / WPT_SCALE_FACT);
    geom::vect2_t sc_act = wpt_scale * MAP_HDG_TRI_SC;
    geom::vect2_t tr_vec = {sin(rot_rad), cos(rot_rad)};
    geom::vect2_t pos =
        scr_pos + map_ctr - tr_vec.scmul(size.x * MAP_HDG_TRI_VOFFS);
    cairo_surface_t* tgt = tex_mngr->data[MAP_AC_TRI_NAME];
    cairo_utils::draw_rotated_image(cr, tgt, pos, sc_act, 1 / M_1_PI - rot_rad);
  }
}

void NDDisplay::draw_trk_line(cairo_t* cr, bool is_inn) {
  double rot_rad = 0;
  if (!is_trk_up) rot_rad = -hdg_data.slip_rad;
  geom::vect2_t dir = {size.x * sin(rot_rad), -size.x * cos(rot_rad)};
  geom::vect2_t dir_nml = {dir.y, -dir.x};  // Facing right
  geom::vect2_t ln_end_inn = map_ctr + dir.scmul(MAP_TRK_LN_LN_INN);
  geom::vect2_t pos_start = map_ctr + scr_pos;
  geom::vect2_t pos_end = ln_end_inn + scr_pos;
  geom::vect3_t clr = cairo_utils::WHITE;
  if (!is_inn) {
    pos_start = pos_end + dir.scmul(MAP_TRK_LN_LN_OUT);
  } else {
    for (int i = 1; i < N_MAP_TRK_DASH; i++) {
      double cy = (MAP_TRK_LN_LN_INN / double(N_MAP_TRK_DASH)) * double(i);
      geom::vect2_t l_strt =
          pos_start + dir.scmul(cy) - dir_nml.scmul(MAP_TRK_DASH_OFFS);
      geom::vect2_t l_end =
          pos_start + dir.scmul(cy) + dir_nml.scmul(MAP_TRK_DASH_OFFS);
      cairo_utils::draw_line(cr, l_strt, l_end, clr,
                             ND_FPL_LINE_THICK * size.x);
    }
  }
  cairo_utils::draw_line(cr, pos_start, pos_end, clr,
                         ND_FPL_LINE_THICK * size.x);
}

void NDDisplay::draw_tfc_arcs(cairo_t* cr) {
  for (int i = 1; i < N_MAP_TRK_DASH; i++) {
    double cr_radi =
        size.x * (MAP_TRK_LN_LN_INN / double(N_MAP_TRK_DASH)) * double(i);
    geom::vect2_t ctr_pos = map_ctr + scr_pos;
    cairo_utils::draw_arc(cr, ctr_pos, cr_radi, ND_MAP_TFC_ARC_ANGLES[i].first,
                          ND_MAP_TFC_ARC_ANGLES[i].second,
                          ND_FPL_LINE_THICK * size.x, cairo_utils::WHITE);
  }
}

void NDDisplay::draw_background(cairo_t* cr, bool draw_inner) {
  cairo_surface_t* back_surf;

  if (cr_md == fms_core::NDMode::PLAN) {
    if (draw_inner)
      back_surf = tex_mngr->data[PLN_BACKGND_INNER_NAME];
    else
      back_surf = tex_mngr->data[PLN_BACKGND_OUTER_NAME];
  } else if (cr_md == fms_core::NDMode::MAP) {
    back_surf = tex_mngr->data[MAP_BACKGND_NAME];
  }

  if ((!draw_inner && cr_md == fms_core::NDMode::MAP) ||
      cr_md == fms_core::NDMode::PLAN) {
    geom::vect2_t scale_back = size / cairo_utils::get_surf_sz(back_surf);
    cairo_utils::draw_image(cr, back_surf, scr_pos, scale_back, false);
  }

  if (cr_md == fms_core::NDMode::MAP) {
    if (!draw_inner) {
      cairo_surface_t* map_hdg_surf = tex_mngr->data[MAP_HDG_NAME];
      geom::vect2_t scale_hdg =
          (size / cairo_utils::get_surf_sz(map_hdg_surf)).scmul(1.41);
      geom::vect2_t hdg_pos = scr_pos + map_ctr;
      cairo_utils::draw_rotated_image(cr, map_hdg_surf, hdg_pos, scale_hdg,
                                      nd_data->get_hdg_trk());
      cairo_surface_t* htrk_box = tex_mngr->data[HTRK_BOX_NAME];
      geom::vect2_t box_sz = cairo_utils::get_surf_sz(htrk_box);
      geom::vect2_t scale_box = (box_sz * size.scdiv(900)) * MAP_HTK_BOX_SC;
      double hht = box_sz.y * 0.5 * scale_box.y;
      geom::vect2_t box_pos = scr_pos + geom::vect2_t{size.x / 2, hht};
      cairo_utils::draw_image(cr, htrk_box, box_pos, scale_box, true);
      draw_htrk(cr);
      draw_hdg_tri(cr);
    } else {
      draw_tfc_arcs(cr);
    }
    draw_trk_line(cr, draw_inner);
  }
}

void NDDisplay::draw_act_leg_info(cairo_t* cr) {
  fms_core::act_leg_info_t leg_info = nd_data->get_act_leg_info();

  geom::vect2_t act_name_pos = scr_pos + size * ACT_LEG_NAME_OFFS;
  geom::vect2_t act_time_pos = scr_pos + size * ACT_LEG_TIME_OFFS;
  geom::vect2_t act_dist_pos = scr_pos + size * ACT_LEG_DIST_OFFS;
  geom::vect2_t act_nm_pos = scr_pos + size * ACT_LEG_NM_OFFS;

  cairo_utils::draw_left_text(cr, font_face, leg_info.name, act_name_pos,
                              cairo_utils::MAGENTA, ND_ACT_INFO_MAIN_FONT_SZ);

  cairo_utils::draw_left_text(cr, font_face, "------Z", act_time_pos,
                              cairo_utils::WHITE, ND_ACT_INFO_MAIN_FONT_SZ);

  cairo_utils::draw_left_text(cr, font_face, leg_info.dist_nm, act_dist_pos,
                              cairo_utils::WHITE, leg_info.dist_sz);
  cairo_utils::draw_left_text(cr, font_face, "NM", act_nm_pos,
                              cairo_utils::WHITE, ND_ACT_INFO_DIST_FONT_SZ);
}

void NDDisplay::draw_spd_info(cairo_t* cr) {
  fms_core::spd_info_t spd_info = nd_data->get_spd_data();

  geom::vect2_t gs_text_pos = scr_pos + size * GS_TEXT_OFFS;
  geom::vect2_t gs_pos = scr_pos + size * GS_OFFS;

  double gs_sz = ND_SPD_BIG_FONT_SZ;
  if (spd_info.gs_kts > GS_THRESH_BIG_KTS) {
    gs_sz = ND_SPD_SMALL_FONT_SZ;
  }

  std::string gs_str = strutils::double_to_str(spd_info.gs_kts, 0);

  cairo_utils::draw_left_text(cr, font_face, "GS", gs_text_pos,
                              cairo_utils::WHITE, ND_ACT_INFO_DIST_FONT_SZ);
  cairo_utils::draw_right_text(cr, font_face, gs_str, gs_pos,
                               cairo_utils::WHITE, gs_sz);

  if (spd_info.tas_kts >= TAS_DISPL_THRESH_KTS) {
    geom::vect2_t tas_text_pos = scr_pos + size * TAS_TEXT_OFFS;
    geom::vect2_t tas_pos = scr_pos + size * TAS_OFFS;

    std::string tas_str = strutils::double_to_str(spd_info.tas_kts, 0);

    cairo_utils::draw_left_text(cr, font_face, "TAS", tas_text_pos,
                                cairo_utils::WHITE, ND_ACT_INFO_DIST_FONT_SZ);
    cairo_utils::draw_right_text(cr, font_face, tas_str, tas_pos,
                                 cairo_utils::WHITE, ND_SPD_SMALL_FONT_SZ);
  }
}

void NDDisplay::draw_range(cairo_t* cr) {
  uint8_t half_pr = 0, full_pr = 0;

  if (curr_rng <= RNG_DEC_1_NM)  // Range can never be < 2.5
    full_pr = 1;
  std::string rng_full_str = strutils::double_to_str(curr_rng, full_pr);

  if (curr_rng / 2 <= RNG_DEC_2_NM)
    half_pr = 2;
  else if (curr_rng / 2 <= RNG_DEC_1_NM)
    half_pr = 1;
  std::string rng_half_str = strutils::double_to_str(curr_rng / 2, half_pr);

  geom::vect2_t ctr_trans = map_ctr + scr_pos;

  geom::vect2_t pos_1_dn = {ctr_trans.x,
                            ctr_trans.y + curr_rng * scale_factor.y};
  geom::vect2_t pos_2_dn = {ctr_trans.x,
                            ctr_trans.y + curr_rng * 0.5 * scale_factor.y};
  geom::vect2_t pos_1_up = {ctr_trans.x,
                            ctr_trans.y - curr_rng * scale_factor.y};
  geom::vect2_t pos_2_up = {ctr_trans.x,
                            ctr_trans.y - curr_rng * 0.5 * scale_factor.y};

  if (cr_md == fms_core::NDMode::PLAN || is_ctr) {
    cairo_utils::draw_centered_text(cr, font_face, rng_full_str, pos_1_dn,
                                    cairo_utils::WHITE, ND_WPT_FONT_SZ);
    cairo_utils::draw_centered_text(cr, font_face, rng_half_str, pos_2_dn,
                                    cairo_utils::WHITE, ND_WPT_FONT_SZ);

    cairo_utils::draw_centered_text(cr, font_face, rng_full_str, pos_1_up,
                                    cairo_utils::WHITE, ND_WPT_FONT_SZ);
    cairo_utils::draw_centered_text(cr, font_face, rng_half_str, pos_2_up,
                                    cairo_utils::WHITE, ND_WPT_FONT_SZ);
  } else {
    geom::vect2_t offs = size * MAP_RNG_OFFS;
    cairo_utils::draw_right_text(cr, font_face, rng_half_str, pos_2_up + offs,
                                 cairo_utils::WHITE, ND_WPT_FONT_SZ);
  }
}

void NDDisplay::draw_airports(cairo_t* cr) {
  cairo_surface_t* surf_norm = tex_mngr->data[ARPT_NML_POI_NAME];
  cairo_surface_t* surf_altn = tex_mngr->data[ARPT_ALTN_POI_NAME];
  for (size_t i = 0; i < nd_data->get_num_poi_arpts(); i++) {
    labeled_point_with_dist_t cr_point = nd_data->get_arpt(i);
    if (cr_point.dist_ctr > curr_rng) {
      break;
    }
    if (i < N_EFIS_MAP_ALTN_APTS) {
      draw_labeled_point(cr, surf_altn, cr_point.point, EFIS_ARPT_SC);
    } else {
      draw_labeled_point(cr, surf_norm, cr_point.point, EFIS_ARPT_SC);
    }
  }
}

void NDDisplay::draw_vordmes(cairo_t* cr) {
  cairo_surface_t* tgt_surf = tex_mngr->data[VORDME_POI_NAME];
  for (size_t i = 0; i < nd_data->get_num_poi_vordmes(); i++) {
    labeled_point_with_dist_t cr_point = nd_data->get_vordme(i);
    if (cr_point.dist_ctr > curr_rng) {
      break;
    }
    draw_labeled_point(cr, tgt_surf, cr_point.point, EFIS_VHF_SC);
  }
}

void NDDisplay::draw_vors_dmes(cairo_t* cr) {
  cairo_surface_t* tgt_surf = tex_mngr->data[DME_POI_NAME];
  for (size_t i = 0; i < nd_data->get_num_poi_vhf_not_vordmes(); i++) {
    labeled_point_with_dist_t cr_point = nd_data->get_vhf_not_vordme(i);
    if (cr_point.dist_ctr > curr_rng) {
      break;
    }
    draw_labeled_point(cr, tgt_surf, cr_point.point, EFIS_VHF_SC);
  }
}

void NDDisplay::draw_efis_filters(cairo_t* cr) {
  if (cr_md == fms_core::NDMode::MAP) {
    if (efis_sel.arpt_on) {
      draw_airports(cr);
    }
    if (efis_sel.sta_on) {
      draw_vordmes(cr);
      draw_vors_dmes(cr);
    }
  }
}

void NDDisplay::draw_labeled_point(cairo_t* cr, cairo_surface_t* img,
                                   labeled_point_t& src_point,
                                   double img_scale) {
  geom::vect2_t pos_local = get_screen_coords(src_point.pos);
  geom::vect2_t scale_fact_vec = size.scmul(img_scale).scdiv(WPT_SCALE_FACT);
  geom::vect2_t img_sz = cairo_utils::get_surf_sz(img) * scale_fact_vec;
  geom::vect2_t pos_text = pos_local + img_sz.scmul(0.5);
  double sc_fact_txt = size.x / WPT_SCALE_FACT;
  cairo_utils::draw_image(cr, img, pos_local, scale_fact_vec, true);
  cairo_utils::draw_left_text(cr, font_face, src_point.name, pos_text,
                              cairo_utils::ND_CYAN,
                              EFIS_POI_NAME_FNT_SZ * sc_fact_txt);
}
}  // namespace fms_displays
