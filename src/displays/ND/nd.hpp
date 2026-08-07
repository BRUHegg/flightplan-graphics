/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        This source file contains declarations of classes, functions, etc
        used in the ND implementation. Author: discord/bruh4096#4512
*/

#include <common/cairo_utils.hpp>
#include <fpln/fpln_sys.hpp>
#include <geom.hpp>
#include <libnav/str_utils.hpp>
#include <memory>
#include <util.hpp>

namespace StratosphereAvionics {
enum class PoiType { AIRPORT, WAYPOINT, VOR_ILS_DME, VHF_NOT_VORDME };

// Texture names(for use with texture manager)
const std::string WPT_INACT_NAME = "wpt_inact";
const std::string WPT_ACT_NAME = "wpt_act";
const std::string AIRPLANE_NAME = "airplane";
const std::string PLN_BACKGND_INNER_NAME = "pln_back_inner";
const std::string PLN_BACKGND_OUTER_NAME = "pln_back_outer";
const std::string MAP_BACKGND_NAME = "map_back";
const std::string MAP_HDG_NAME = "map_hdg";
const std::string MAP_AC_TRI_NAME = "map_ac_ico";
const std::string HTRK_BOX_NAME = "hdg_trk_box";
const std::string ARPT_NML_POI_NAME = "normal_arpt_sign";
const std::string ARPT_ALTN_POI_NAME = "altn_arpt_sign";
const std::string DME_POI_NAME = "dme";
const std::string VORDME_POI_NAME = "vordme";

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
  std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
  std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

  map_poi_container_t(std::shared_ptr<libnav::ArptDB> arpt_ptr,
                      std::shared_ptr<libnav::NavaidDB> navaid_ptr);

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

class NDData {
 public:
  NDData(std::shared_ptr<test::FPLSys> fpl_sys);

  bool init();

  /*
      Function: set_th_up
      @desc:
      changes current track/heading up status. I.e. if we're heading up,
      change to track up and vice versa.
  */

  void set_th_up();

  /*
      Function: set_th_up
      @return:
      true if track up.
  */

  bool get_th_up();

  void toggle_efis_arpt_sd(size_t sd_idx);

  void toggle_efis_sta_sd(size_t sd_idx);

  efis_selection_t get_efis_sts_sd(size_t sd_idx);

  void set_mode(size_t sd_idx, test::NDMode md, bool set_ctr = false);

  std::pair<test::NDMode, bool> get_mode(size_t sd_idx) const;

  std::vector<int> get_rte_draw_seq(size_t sd_idx);

  size_t get_proj_legs(leg_proj_t** out, size_t sd_idx, size_t dt_idx);

  int get_act_leg_idx(size_t sd_idx);

  bool get_ac_pos(geom::vect2_t* out, size_t sd_idx);

  double get_hdg_trk() const;

  test::hdg_info_t get_hdg_data();

  test::spd_info_t get_spd_data();

  test::act_leg_info_t get_act_leg_info();

  bool has_dep_rwy(size_t idx);

  bool has_arr_rwy(size_t idx);

  void switch_range(bool down, size_t sd_idx);

  double get_range(size_t sd_idx);

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
  std::vector<std::shared_ptr<test::FplnInt>> fpl_vec_;
  std::shared_ptr<test::FPLSys> fpl_sys_ptr_;

  bool idx_proj_act_ = false;
  poi_data_t pois_projected_[2];
  map_poi_container_t poi_data_;

  bool trk_up_ = true;

  // Of size N_FPL_SYS_RTES
  std::vector<test::nd_leg_data_t*> leg_data_;
  std::vector<size_t> leg_data_sz_;

  std::vector<double> fpl_id_last_;
  std::vector<bool> has_dep_rwy_, has_arr_rwy_;
  std::vector<int> act_leg_idx_;
  // -1 if route is not to be drawn, index otherwise.
  // Routes are ordered by color. Refer to ND_RTE_CLRS.
  std::vector<std::vector<int>> rte_draw_seq_;

  // 2*number of routes
  std::vector<map_data_t> mp_data_;

  std::vector<int> act_leg_idx_sd_;
  // Stored 1 per fo, 1 per cap
  std::vector<std::pair<test::NDMode, bool>> curr_modes_;
  std::vector<geom::vect2_t> ac_pos_projected_;
  std::vector<bool> ac_pos_ok_;
  std::vector<size_t> range_index_;
  std::vector<geo::point> map_center_;
  std::vector<efis_selection_t> efis_sel_;

  test::hdg_info_t heading_data_;
  geo::point ac_pos_last_;

  static bool bound_check(double x1, double x2, double rng);

  static nd_util_idx_t get_util_idx(size_t gn_idx);

  double get_cr_rot() const;

  std::pair<geo::point, double> get_proj_params(size_t sd_idx) const;

  bool in_view(geom::vect2_t start, geom::vect2_t end, size_t sd_idx);

  void update_rte_draw_seq();

  void update_ctr(size_t sd_idx);

  bool project_ac_pos(size_t sd_idx);

  void project_legs(size_t gn_idx);

  void project_rwys(size_t gn_idx);

  void fetch_legs(size_t dt_idx);

  void update_fpl(size_t dt_idx);
};

class NDDisplay {
 public:
  NDDisplay(std::shared_ptr<NDData> data,
            std::shared_ptr<cairo_utils::texture_manager_t> mngr,
            cairo_font_face_t* ff, geom::vect2_t pos, geom::vect2_t sz,
            size_t sd_idx);

  void draw(cairo_t* cr);

 private:
  std::shared_ptr<NDData> nd_data;
  std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;

  bool is_trk_up;
  test::NDMode cr_md;
  bool is_ctr, has_tfc;
  efis_selection_t efis_sel;

  cairo_font_face_t* font_face;

  geom::vect2_t scr_pos;
  geom::vect2_t size;
  geom::vect2_t map_ctr, scale_factor;
  test::hdg_info_t hdg_data;
  double rng, curr_rng;

  size_t side_idx;

  void update_mode();

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

  void draw_airports(cairo_t* cr);

  void draw_vordmes(cairo_t* cr);

  void draw_vors_dmes(cairo_t* cr);

  void draw_efis_filters(cairo_t* cr);

  void draw_labeled_point(cairo_t* cr, cairo_surface_t* img,
                          labeled_point_t& src_point, double img_scale);
};
}  // namespace StratosphereAvionics
