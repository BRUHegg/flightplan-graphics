#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <stack>
#include <string>

#include <displays/common/cairo_utils.hpp>
#include <displays/common/texture_manager.hpp>
#include <fpln/fpln_sys.hpp>
#include <fpln/flightpln_int.hpp>
#include <libnav/cifp_parser.hpp>
#include <util/geom.hpp>
#include <util/util.hpp>

#ifdef FPL_DEBUG
#include <iostream>
#endif

namespace fms_displays {

util::const_str_data_t GetCduTextureNames();

enum class CDUPage {
  RTE,
  DEP_ARR_INTRO,
  DEP1,
  ARR1,
  DEP2,
  ARR2,
  LEGS,
  INIT_REF,
  ALTN,
  VNAV,
  FIX,
  HOLD,
  FMC_COMM,
  PROG,
  MENU,
  IDENT,
  POS_INIT,
  INIT_REF_INDEX,
  NAV_RAD,
  PREV_PAGE,
  NEXT_PAGE
};

constexpr double CDU_TEXTURE_ASPECT_RATIO = (488.0 / 751.0);

enum class CDUColor { WHITE, GREEN, CYAN, MAGENTA };

const std::vector<CDUPage> CDU_PAGE_FACES = {
    CDUPage::INIT_REF,  CDUPage::RTE,      CDUPage::DEP_ARR_INTRO,
    CDUPage::ALTN,      CDUPage::VNAV,     CDUPage::FIX,
    CDUPage::LEGS,      CDUPage::HOLD,     CDUPage::FMC_COMM,
    CDUPage::PROG,      CDUPage::MENU,     CDUPage::NAV_RAD,
    CDUPage::PREV_PAGE, CDUPage::NEXT_PAGE};

struct cdu_scr_data_t {
  std::string heading_big, heading_small;
  CDUColor heading_color;
  std::vector<std::string> data_lines;
  std::vector<std::string> chr_sts;

  cdu_scr_data_t();
};

class CDU final {
 public:
  using flightplan_type = typename fms_core::FPLSys::flightplan_type;

  CDU(util::OpaquePointer<fms_core::FPLSys> fs, size_t sd_idx);

  void update() noexcept;

  bool get_exec_lt() const noexcept;

  std::string on_event(int event_key, std::string scratchpad,
                       std::string* s_out) noexcept;

  cdu_scr_data_t get_screen_data() const noexcept;

 private:
  struct ident_info_t {
    unsigned airac_cycle;
    fms_core::aircraft_info_t ac_info;
    int drag = 0;
    int fuel_flow = 0;
    bool is_armed = false;
  };

  struct pos_info_t {
    geo::point last_pos;
    std::optional<libnav::airport_t> ref_airport;
    std::optional<geo::point> inertial_pos;
  };

  mutable std::shared_mutex main_mutex_;

  std::size_t act_sd_idx_;

  fms_core::NDMode nd_mode_; // Obtained from fpl_sys_

  util::OpaquePointer<libnav::ArptDB> airport_db_;
  util::OpaquePointer<libnav::NavaidDB> navaid_db_;

  util::OpaquePointer<fms_core::FPLSys> fpl_sys_;
  util::OpaquePointer<flightplan_type> fpln_;
  util::OpaquePointer<flightplan_type> m_rte1_ptr_;
  util::OpaquePointer<flightplan_type> m_rte2_ptr_;
  util::OpaquePointer<flightplan_type> m_act_ptr_;
  std::size_t sel_fpl_idx_;  // [0;3]
  std::size_t act_fpl_idx_;  // [0;3]

  CDUPage curr_page_ = CDUPage::MENU;
  int n_subpg_ = 1;
  int curr_subpg_ = 1;

  // IDENT data:
  ident_info_t ident_info_;

  pos_info_t pos_init_info_;

  // RTE data
  fms_core::RTECopySts rte_copy_ = fms_core::RTECopySts::UNAVAIL;

  // sel des data
  /*
      How sel des works:
      1) set_sel_des_state is called(typically from get_wpt_from_user)
      2) The current page has to give way to sel_des_
      3) Once the user has selected something, the data is retrieved
      segment list and leg list ids are stored to make sure that either list is
     valid after the user has made the selection. The CDU will display sel des
     only if sel_des_ is set to true. It has priority over any other page. If
     user exits sel des without making a selection, sel_des_ is set to false.
     Other variables get re-set as well in set_page function. sel_des_event_
     stores the event triggered before sel des page was open.
  */

  int sel_des_idx_ = -1;
  int sel_des_event_ = 0;
  int sel_des_subpg_ = 0;
  double sel_des_seg_id_ = 0;
  double sel_des_leg_id_ = 0;
  bool sel_des_ = false;

  // DEP/ARR data:
  /*
      Info about filters:
      filters are there to enable the user to see only the
     runways/procedures/etc that are valid for the existing constraints(i.e.
     runways, procedures) All filters are per-flightplan. rwy_filter is set
     whenever a procedure is selected proc_filter is set whenever a
     runway/approach is selected
  */
  std::vector<bool> dep_arr_rwy_filter_;
  std::vector<bool> dep_arr_proc_filter_;
  std::vector<bool> dep_arr_trans_filter_;
  std::vector<bool> dep_arr_via_filter_;
  std::vector<std::vector<std::string>> procedures_;
  std::vector<std::vector<std::string>> transitions_;
  std::vector<std::vector<std::string>> approaches_;
  std::vector<std::vector<std::string>> rwys_;
  std::vector<std::vector<std::string>> vias_;

  // LEGS data:
  bool leg_sel_pr_ = false;
  size_t n_seg_list_sz_, n_leg_list_sz_;
  std::vector<fms_core::list_node_ref_t<fms_core::fpl_seg_t>> seg_list_;
  std::vector<fms_core::list_node_ref_t<fms_core::leg_list_data_t>> leg_list_;
  std::vector<fms_core::fpln_info_t> fpl_infos_;
  std::vector<std::pair<size_t, double>> leg_sel_;
  std::vector<size_t> pln_ctr_idx_;
  std::vector<geo::point> pln_ctr_pos_;

  // Select desired data:
  std::vector<libnav::waypoint_entry_t> sel_des_data_;
  std::string sel_des_nm_ = "";

  static std::string str_align_right(const std::string& str);

  static std::string get_cdu_line(std::string in, std::string line,
                                  bool align_right = false);

  static void fill_char_state_buf(cdu_scr_data_t& src);

  /*
      Function: get_cdu_leg_prop
      @desc:
      Returns heading and distance data for a leg entry
      @param src: reference to leg in question
      @return string for CDU to display.
  */

  static std::string get_cdu_leg_prop(
      const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static void fill_drag_ff_num(int num, char out_buff[5]) noexcept;

  static std::string get_displayed_pos(geo::point pos) noexcept;

  static std::string get_scratchpad_pos(geo::point pos) noexcept;

  /*
      Function: get_leg_alt
      @desc:
      Returns value in alt1 or alt2 of a leg. Altitudes greater than transition
      altitude are returned as FLXXX.
      @param src: reference to leg in question
      @param alt2: set to true to return alt2
      @param fl: true if the altitude needs to be shortened for a altitude
     within constraint
      @return string for CDU to display.
  */

  static std::string get_leg_alt(const
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src, bool alt2 = false,
      bool fl = false);

  /*
      Function: get_cdu_leg_vcstr
      @desc:
      Returns vertical constraint for a leg entry
      @param src: reference to leg in question
      @return string for CDU to display.
  */

  static std::string get_cdu_leg_vcstr(const
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static std::string get_cdu_leg_spdcstr(const
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static std::string get_cdu_leg_nm(const
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static bool scratchpad_has_delete(const std::string& scratchpad);

  static fms_core::spd_cstr_t get_spd_cstr(const std::string& str);

  static fms_core::alt_cstr_t get_alt_cstr(const std::string& str);

  void update_fpl_infos();

  void set_page(CDUPage pg);

  void set_sel_des_state(double seg_id, double leg_id, std::string& name,
                         std::vector<libnav::waypoint_entry_t>& w_e);

  libnav::waypoint_t get_wpt_from_user(std::string name, double seg_id,
                                       double leg_id, bool* not_in_db,
                                       bool* inv_ent, bool* wait_sel,
                                       bool* sel_used);

  std::string set_departure(std::string icao, std::string* s_out);

  std::string set_arrival(std::string icao, std::string* s_out);

  std::string set_flt_nbr(std::string nbr);

  std::string set_dep_rwy(std::string id);

  std::string load_rte();

  std::string save_rte();

  std::string add_via(size_t next_idx, std::string name);

  std::string delete_via(size_t next_idx);

  std::string add_to(size_t next_idx, std::string name);

  std::string delete_to(size_t next_idx);

  void get_seg_page(cdu_scr_data_t* in) const noexcept;

  std::string get_sts(std::string& cr, std::string& act) const noexcept;

  void get_procs(cdu_scr_data_t* in, std::string curr_proc,
                 std::string curr_trans, std::string act_proc,
                 std::string act_trans, bool rte2) const noexcept;

  void get_rwys(cdu_scr_data_t* in, std::string curr_rwy, std::string act_rwy,
                bool rte2, std::string curr_appr = "",
                std::string curr_via = "", std::string act_appr = "",
                std::string act_via = "", bool get_appr = false) const noexcept;

  std::string get_small_heading() const noexcept;

  void set_procs(fms_core::ProcType ptp, bool is_arr, bool rte2);

  void set_fpl_proc(int event, fms_core::ProcType ptp, bool is_arr, bool rte2);

  void get_rte_dep_arr(cdu_scr_data_t& out, bool rte2) const noexcept;

  bool arr_has_rwys(std::string& cr_appr, bool rte2) const noexcept;

  std::string get_ident_drag_ff() const noexcept;

  std::optional<int> get_ident_entry_number(const std::string& scratchpad);

  std::string get_pos_init_airport_str() const noexcept;

  std::string get_pos_init_gps_pos_str() const noexcept;

  std::string get_pos_init_inertial_pos_str() const noexcept;

  // Per-page fetching of the number of subpages:

  int get_n_sel_des_subpg() const noexcept;

  int get_n_rte_subpg() const noexcept;

  int get_n_dep_arr_subpg(bool rte2) noexcept;

  int get_n_legs_subpg() const noexcept;

  // Per-page event handling:

  std::string handle_menu(int event_key, const std::string& scratchpad);

  std::string handle_ident(int event_key, const std::string& scratchpad);

  std::string handle_pos_init(int event_key, std::string scratchpad,
                         std::string* s_out);

  std::string handle_init_ref_index(
    int event_key, const std::string& scratchpad);

  std::string handle_sel_des(int event_key);

  std::string handle_rte(int event_key, std::string scratchpad,
                         std::string* s_out);

  std::string handle_dep_arr(int event_key);

  std::string handle_dep(int event_key, bool rte2);

  std::string handle_arr(int event_key, bool rte2);

  std::size_t get_leg_start_idx() const noexcept;

  std::size_t get_leg_end_idx() const noexcept;

  std::size_t get_seg_start_idx() const noexcept;

  std::size_t get_seg_end_idx() const noexcept;

  // reset_leg_dto_sel resets selection when user exits legs page

  void reset_leg_dto_sel(size_t fp_idx);

  void reset_leg_all_sel();

  /*
      Function: handle_legs_dto
      @desc:
      Handles legs direct from-to scenario
      @param usr_idx: index to a valid item in leg_list that the user has
     selected
      @return: 1 if we need to handle this event as an insertion. Otherwise 0.
  */

  bool handle_legs_dto(size_t usr_idx, std::string scratchpad,
                       std::string* s_out);

  std::string handle_legs_insert(size_t usr_idx, std::string scratchpad);

  std::string handle_legs_delete(size_t usr_idx);

  std::string handle_legs_cstr_mod(size_t usr_idx, std::string& scratchpad);

  void handle_legs_map_ctr_advance();

  std::string handle_legs(int event_key, std::string scratchpad,
                          std::string* s_out);

  // Per-page content fetching. The CDU displays exactly what these functions
  // output:

  cdu_scr_data_t get_menu_page() const noexcept;

  cdu_scr_data_t get_ident_page() const noexcept;

  cdu_scr_data_t get_pos_init_page() const noexcept;

  cdu_scr_data_t get_init_ref_index_page() const noexcept;

  cdu_scr_data_t get_sel_des_page() const noexcept;

  cdu_scr_data_t get_rte_page() const noexcept;

  cdu_scr_data_t get_dep_arr_page() const noexcept;

  void dep_arr_set_bottom(cdu_scr_data_t& out) const noexcept;

  cdu_scr_data_t get_dep_page(bool rte2) const noexcept;

  cdu_scr_data_t get_arr_page(bool rte2) const noexcept;

  std::string get_legs_btm() const noexcept;

  cdu_scr_data_t get_legs_page() const noexcept;
};

class CDUDisplay final {
 public:
  using event_type = int;
  using texture_type = typename TextureManager::texture_t;

  static std::optional<event_type> get_event_from_str(
    const std::string& str);

  CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, 
             util::OpaquePointer<TextureManager> tm, 
             util::OpaquePointer<CDU> cdu, bool is_free);

  friend void Swap(CDUDisplay& d_a, CDUDisplay& d_b);

  CDUDisplay(CDUDisplay&& other);

  std::pair<double, double> GetDrawSize() const noexcept;

  void on_event(event_type event);

  void draw(cairo_t* cr);

 private:
  struct cdu_textures_t {
    texture_type cdu_big_white;
    texture_type cdu_big_green;
    texture_type cdu_big_cyan;
    texture_type cdu_big_magenta;

    cairo_font_face_t* main_font_face;

    void init(util::OpaquePointer<TextureManager> tm);
  };

  mutable std::mutex main_mutex_;
  std::queue<event_type> events_;

  geom::vect2_t display_pos_;  // position of the CDU display on the screen
  geom::vect2_t display_size_;
  double scale_coeff_;

  cdu_textures_t textures_;
  util::OpaquePointer<CDU> cdu_ptr_;

  std::string scratchpad_;
  size_t scratch_curr_;

  std::stack<std::string> msg_stack_;

  std::chrono::time_point<std::chrono::steady_clock> last_press_tp_;

  void handle_event(event_type event) noexcept;

  void process_events(std::size_t cnt_max) noexcept;

  void add_to_scratchpad(char c);

  void clear_scratchpad();

  void update_scratchpad(event_type event);

  static int get_cdu_letter_idx(char c);

  static CDUColor get_cdu_color(char c);

  static bool chr_is_big(char c);

  texture_type get_font_sfc(CDUColor cl);

  void draw_cdu_letter(cairo_t* cr, char c, geom::vect2_t pos,
                       geom::vect2_t scale, texture_type font_sfc);

  void draw_cdu_line(cairo_t* cr, const std::string& s, geom::vect2_t pos,
                     double l_intv_px, std::string sts = "",
                     geom::vect2_t scale = {}, CDUColor clr = CDUColor::WHITE);

  void draw_screen(cairo_t* cr);
};
}  // namespace fms_displays
