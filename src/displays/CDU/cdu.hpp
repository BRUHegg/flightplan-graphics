#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <stack>
#include <string>

#include <common/bytemap.hpp>
#include <common/cairo_utils.hpp>
#include <fpln/fpln_sys.hpp>
#include <fpln/flightpln_int.hpp>
#include <libnav/cifp_parser.hpp>
#include <util/geom.hpp>

#define FPL_DEBUG
#ifdef FPL_DEBUG
#include <iostream>
#endif

namespace fms_displays {
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
  NAV_RAD,
  PREV_PAGE,
  NEXT_PAGE
};

constexpr double CDU_TEXTURE_ASPECT_RATIO = (488.0 / 751.0);
const std::string CDU_WHITE_TEXT_NAME = "cdu_big_white";
const std::string CDU_GREEN_TEXT_NAME = "cdu_big_green";
const std::string CDU_CYAN_TEXT_NAME = "cdu_big_cyan";
const std::string CDU_MAGENTA_TEXT_NAME = "cdu_big_magenta";

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

class CDU {
 public:
  CDU(std::shared_ptr<fms_core::FPLSys> fs, size_t sd_idx);

  void update() noexcept;

  bool get_exec_lt() const noexcept;

  std::string on_event(int event_key, std::string scratchpad,
                       std::string* s_out) noexcept;

  cdu_scr_data_t get_screen_data() noexcept;

 private:
  std::size_t act_sd_idx_;

  std::shared_ptr<fms_core::FPLSys> fpl_sys_;
  std::shared_ptr<fms_core::FplnInt> fpln_;
  std::shared_ptr<fms_core::FplnInt> m_rte1_ptr_;
  std::shared_ptr<fms_core::FplnInt> m_rte2_ptr_;
  std::shared_ptr<fms_core::FplnInt> m_act_ptr_;
  std::size_t sel_fpl_idx_;  // [0;3]
  std::size_t act_fpl_idx_;  // [0;3]

  CDUPage curr_page_ = CDUPage::RTE;
  int n_subpg_ = 1;
  int curr_subpg_ = 1;

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
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

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

  static std::string get_leg_alt(
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src, bool alt2 = false,
      bool fl = false);

  /*
      Function: get_cdu_leg_vcstr
      @desc:
      Returns vertical constraint for a leg entry
      @param src: reference to leg in question
      @return string for CDU to display.
  */

  static std::string get_cdu_leg_vcstr(
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static std::string get_cdu_leg_spdcstr(
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static std::string get_cdu_leg_nm(
      fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src);

  static bool scratchpad_has_delete(std::string& scratchpad);

  static fms_core::spd_cstr_t get_spd_cstr(std::string& str);

  static fms_core::alt_cstr_t get_alt_cstr(std::string& str);

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

  // Per-page fetching of the number of subpages:

  int get_n_sel_des_subpg() const noexcept;

  int get_n_rte_subpg() const noexcept;

  int get_n_dep_arr_subpg(bool rte2) noexcept;

  int get_n_legs_subpg() const noexcept;

  // Per-page event handling:

  std::string handle_sel_des(int event_key);

  std::string handle_rte(int event_key, std::string scratchpad,
                         std::string* s_out);

  std::string handle_dep_arr(int event_key);

  std::string handle_dep(int event_key, bool rte2);

  std::string handle_arr(int event_key, bool rte2);

  size_t get_leg_stt_idx() const noexcept;

  size_t get_leg_end_idx() const noexcept;

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

  std::string handle_legs(int event_key, std::string scratchpad,
                          std::string* s_out);

  // Per-page content fetching. The CDU displays exactly what these functions
  // output:

  cdu_scr_data_t get_sel_des_page() const noexcept;

  cdu_scr_data_t get_rte_page() const noexcept;

  cdu_scr_data_t get_dep_arr_page() const noexcept;

  void dep_arr_set_bottom(cdu_scr_data_t& out);

  cdu_scr_data_t get_dep_page(bool rte2) noexcept;

  cdu_scr_data_t get_arr_page(bool rte2) noexcept;

  std::string get_legs_btm() const noexcept;

  cdu_scr_data_t get_legs_page() noexcept;
};

class CDUDisplay {
 public:
  using event_type = int;

  CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, cairo_font_face_t* ff,
             std::shared_ptr<cairo_utils::texture_manager_t> tm,
             std::shared_ptr<CDU> cdu);

  void on_event(event_type event);

  void draw(cairo_t* cr);

 private:
  mutable std::mutex main_mutex_;
  std::queue<event_type> events_;

  geom::vect2_t screen_pos_;  // position of the CDU texture on the screen
  geom::vect2_t size_;
  geom::vect2_t display_pos_;  // position of the CDU display on the screen
  geom::vect2_t display_size_;

  cairo_font_face_t* font_face;

  std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr_;
  std::shared_ptr<CDU> cdu_ptr_;

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

  cairo_surface_t* get_font_sfc(CDUColor cl);

  void draw_cdu_letter(cairo_t* cr, char c, geom::vect2_t pos,
                       geom::vect2_t scale, cairo_surface_t* font_sfc);

  void draw_cdu_line(cairo_t* cr, std::string& s, geom::vect2_t pos,
                     double l_intv_px, std::string sts = "",
                     geom::vect2_t scale = {}, CDUColor clr = CDUColor::WHITE);

  void draw_screen(cairo_t* cr);
};
}  // namespace fms_displays
