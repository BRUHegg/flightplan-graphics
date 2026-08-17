/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        This source file contains declarations of classes, functions, etc
        used in the ND implementation. Author: discord/bruh4096#4512
*/

#pragma once

#include <cstddef>

#include <bitset>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <displays/common/cairo_utils.hpp>
#include <displays/common/texture_manager.hpp>
#include <fpln/environment.hpp>
#include <fpln/fpln_sys.hpp>
#include <libnav/str_utils.hpp>

#include <util/geom.hpp>
#include <util/util.hpp>

namespace fms_displays {

constexpr std::size_t N_ND_SDS =
    fms_core::N_INTFCS;  // Essentially this is how many NDs we can have
const std::vector<double> ND_RANGES_NM = {10, 20, 40, 80, 160, 320, 640};

util::const_str_data_t GetNdTextureNames();

enum class PoiType { AIRPORT, WAYPOINT, VOR_ILS_DME, VHF_NOT_VORDME };

struct nd_util_idx_t {
  std::size_t sd_idx = 0;
  std::size_t dt_idx = 0;
};

struct labeled_point_t {
  geom::vect2_t pos{};
  std::string name = "";
};

struct labeled_point_with_dist_t {
  labeled_point_t point;
  double dist_ctr = 0;
};

struct labeled_point_with_dist_cmp_t {
  bool operator()(const labeled_point_with_dist_t& pa,
                  const labeled_point_with_dist_t& pb) const noexcept;
};

struct poi_data_t {
  std::size_t n_arpts = 0;
  std::size_t n_waypts = 0;
  std::size_t n_vors_dmes = 0;
  std::size_t n_vordmes = 0;
  labeled_point_with_dist_t* arpts = nullptr;
  labeled_point_with_dist_t* waypts = nullptr;
  labeled_point_with_dist_t* vors_dmes = nullptr;
  labeled_point_with_dist_t* vordmes = nullptr;

  bool init_ptr(labeled_point_with_dist_t** ptr);

  bool init();

  void destroy();
};

struct map_poi_container_t : poi_data_t {
  util::OpaquePointer<libnav::ArptDB> arpt_db_ptr;
  util::OpaquePointer<libnav::NavaidDB> navaid_db_ptr;

  map_poi_container_t(util::OpaquePointer<libnav::ArptDB> arpt_ptr,
                      util::OpaquePointer<libnav::NavaidDB> navaid_ptr);

  void set_add(
      std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t>& st,
      labeled_point_with_dist_t& pt);

  void set_to_array(
      std::set<labeled_point_with_dist_t, labeled_point_with_dist_cmp_t>& st,
      labeled_point_with_dist_t* arr, size_t* arr_sz);

  void project_array(labeled_point_with_dist_t* src,
                     labeled_point_with_dist_t* dst, size_t sz, size_t* sz_tgt,
                     geo::point curr_pos, double rot_rad);

  void fetch_arpts(geo::point curr_pos);

  void fetch_navaids(geo::point curr_pos);

  void fetch(geo::point curr_pos);

  void project(poi_data_t& tgt, geo::point curr_pos, double rot_add_rad = 0.0);
};

struct leg_proj_t {
  geom::vect2_t start, end, arc_ctr, end_wpt;
  bool is_arc, is_finite, is_rwy, has_path;
  double turn_rad_nm;
  std::string end_nm;
  geom::line_joint_t* joint;

  std::string get_draw_nm();
};

struct map_data_t {
  leg_proj_t* proj_legs;
  geom::line_joint_t* line_joints;

  size_t n_act_proj_legs;
  size_t n_act_joints;

  bool create();

  void destroy();
};

struct efis_selection_t {
  bool arpt_on = false;
  bool sta_on = false;
};

struct nd_config_t {
  bool is_track_up = false;
  bool efis_airport_on = false;
  bool efis_station_on = false;
  bool efis_waypoint_on = false;
  bool mode_is_ctr = false;
  fms_core::NDMode mode = fms_core::NDMode::MAP;
  std::bitset<fms_core::N_FPL_SYS_RTES> has_dep_rwy;
  std::bitset<fms_core::N_FPL_SYS_RTES> has_arr_rwy;
  std::size_t range_idx = 0;

  nd_config_t();
};

class NDData final {
 public:
  using flightplan_type = typename fms_core::FPLSys::flightplan_type;

  NDData(util::OpaquePointer<fms_core::FPLSys> fpl_sys,
         util::OpaquePointer<fms_environment::EnvDataRefMap> env_map);

  bool init();

  nd_config_t get_config(std::size_t sd_idx) const noexcept;

  std::vector<int> get_rte_draw_seq(size_t sd_idx);

  size_t get_proj_legs(leg_proj_t** out, size_t sd_idx, size_t dt_idx);

  int get_act_leg_idx(size_t sd_idx);

  bool get_ac_pos(geom::vect2_t* out, size_t sd_idx);

  double get_hdg_trk() const;

  fms_core::hdg_info_t get_hdg_data();

  fms_core::spd_info_t get_spd_data();

  fms_core::act_leg_info_t get_act_leg_info();

  // POI functions

  size_t get_num_poi_arpts();

  size_t get_num_poi_waypts();

  size_t get_num_poi_vordmes();

  size_t get_num_poi_vhf_not_vordmes();

  labeled_point_with_dist_t get_arpt(size_t i);

  labeled_point_with_dist_t get_waypt(size_t i);

  labeled_point_with_dist_t get_vordme(size_t i);

  labeled_point_with_dist_t get_vhf_not_vordme(size_t i);

  void update();

  void destroy();

 private:
  mutable std::shared_mutex main_mutex_;

  nd_config_t nd_configs_[N_ND_SDS];

  util::OpaquePointer<fms_environment::EnvDataRefMap> env_map_;

  util::OpaquePointer<flightplan_type> fpl_vec_[fms_core::N_FPL_SYS_RTES];
  util::OpaquePointer<fms_core::FPLSys> fpl_sys_ptr_;

  bool idx_proj_act_ = false;
  poi_data_t pois_projected_[N_ND_SDS];
  map_poi_container_t poi_data_;

  // Of size N_FPL_SYS_RTES
  std::vector<fms_core::nd_leg_data_t*> leg_data_;
  std::vector<size_t> leg_data_sz_;

  std::vector<double> fpl_id_last_;
  std::bitset<fms_core::N_FPL_SYS_RTES> has_dep_rwy_, has_arr_rwy_;
  std::vector<int> act_leg_idx_;
  // -1 if route is not to be drawn, index otherwise.
  // Routes are ordered by color. Refer to ND_RTE_CLRS.
  std::vector<std::vector<int>> rte_draw_seq_;

  // 2*number of routes
  std::vector<map_data_t> mp_data_;

  std::vector<int> act_leg_idx_sd_;
  // Stored 1 per fo, 1 per cap
  std::vector<geom::vect2_t> ac_pos_projected_;
  std::vector<bool> ac_pos_ok_;
  std::vector<geo::point> map_center_;

  fms_core::hdg_info_t heading_data_;
  geo::point ac_pos_last_;

  static bool bound_check(double x1, double x2, double rng) noexcept;

  static nd_util_idx_t get_util_idx(std::size_t gn_idx) noexcept;

  void update_configs() noexcept;

  double get_range_impl(std::size_t sd_idx) const noexcept;

  double get_cr_rot() const noexcept;

  std::pair<geo::point, double> get_proj_params(
      std::size_t sd_idx) const noexcept;

  bool in_view(geom::vect2_t start, geom::vect2_t end,
               std::size_t sd_idx) const noexcept;

  void update_rte_draw_seq();

  void update_ctr(std::size_t sd_idx);

  bool project_ac_pos(std::size_t sd_idx);

  void project_legs(std::size_t gn_idx);

  void project_rwys(std::size_t gn_idx);

  void fetch_legs(std::size_t dt_idx);

  void update_fpl(std::size_t dt_idx);
};

class NDDisplay final {
 public:
  using texture_type = typename TextureManager::texture_t;

  NDDisplay(util::OpaquePointer<NDData> data,
            util::OpaquePointer<TextureManager> mngr, geom::vect2_t pos,
            geom::vect2_t sz, size_t sd_idx);

  std::pair<double, double> GetDrawSize() const noexcept;

  void draw(cairo_t* cr);

 private:
  struct nd_textures_t {
    texture_type wpt_inact;
    texture_type wpt_act;
    texture_type airplane;
    texture_type pln_back_inner;
    texture_type pln_back_outer;
    texture_type map_back;
    texture_type map_hdg;
    texture_type map_ac_ico;
    texture_type hdg_trk_box;
    texture_type normal_arpt_sign;
    texture_type altn_arpt_sign;
    texture_type dme;
    texture_type vordme;
    texture_type waypoint;
    cairo_font_face_t* font_face;

    void init(util::OpaquePointer<TextureManager> tex_manager);
  };

  util::OpaquePointer<NDData> nd_data_;

  nd_config_t config_;
  bool has_tfc_ = false;

  nd_textures_t textures_;

  geom::vect2_t scr_pos_;
  geom::vect2_t size_;
  geom::vect2_t map_ctr_, scale_factor_;
  fms_core::hdg_info_t hdg_data_;
  double rng_, curr_rng_;

  size_t side_idx_;

  void update_map_params();

  geom::vect2_t get_screen_coords(geom::vect2_t src);

  void draw_line_joint(cairo_t* cr, geom::line_joint_t lj,
                       geom::vect3_t ln_clr);

  void draw_flight_plan(cairo_t* cr, bool draw_labels, geom::vect3_t ln_clr,
                        size_t idx = 0);

  void draw_ext_rwy_ctr_line(cairo_t* cr, leg_proj_t rnw_proj);

  void draw_runway(cairo_t* cr, leg_proj_t rnw_proj);

  void draw_runways(cairo_t* cr, size_t idx = 0);

  void draw_all_fplns(cairo_t* cr);

  void draw_airplane(cairo_t* cr);

  void draw_htrk(cairo_t* cr);

  void draw_hdg_tri(cairo_t* cr);

  void draw_trk_line(cairo_t* cr, bool is_inn);

  void draw_tfc_arcs(cairo_t* cr);

  void draw_background(cairo_t* cr, bool draw_inner);

  void draw_act_leg_info(cairo_t* cr);

  void draw_spd_info(cairo_t* cr);

  void draw_range(cairo_t* cr);

  std::size_t draw_airports(cairo_t* cr);

  std::size_t draw_vordmes(cairo_t* cr);

  std::size_t draw_vors_dmes(cairo_t* cr);

  std::size_t draw_waypoints(cairo_t* cr);

  void draw_efis_filters(cairo_t* cr);

  void draw_labeled_point(cairo_t* cr, cairo_surface_t* img,
                          labeled_point_t& src_point, double img_scale);
};
}  // namespace fms_displays
