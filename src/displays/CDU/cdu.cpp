#include "cdu.hpp"

#include <charconv>
#include <chrono>
#include <cstddef>
#include <displays/common/font_names.hpp>
#include <displays/common/texture_manager.hpp>
#include <format>
#include <fpln/flightpln_int.hpp>
#include <libnav/cifp_parser.hpp>
#include <libnav/str_utils.hpp>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <util/date_time.hpp>
#include <utility>
#include <util/geom.hpp>
#include <util/util.hpp>

#include "common.hpp"

namespace {

constexpr int N_DEP_ARR_ROW_DSP = 5;
constexpr int CNT_CDU_SELECT_KEYS = 6;

// Event names:
// LSK
const char CDU_LSK_1_EVENT[] = "cdu_lsk_1"; // 1-6 from top to bottom
const char CDU_LSK_2_EVENT[] = "cdu_lsk_2";
const char CDU_LSK_3_EVENT[] = "cdu_lsk_3";
const char CDU_LSK_4_EVENT[] = "cdu_lsk_4";
const char CDU_LSK_5_EVENT[] = "cdu_lsk_5";
const char CDU_LSK_6_EVENT[] = "cdu_lsk_6";
// RSK
const char CDU_RSK_1_EVENT[] = "cdu_rsk_1"; // 1-6 from top to bottom
const char CDU_RSK_2_EVENT[] = "cdu_rsk_2";
const char CDU_RSK_3_EVENT[] = "cdu_rsk_3";
const char CDU_RSK_4_EVENT[] = "cdu_rsk_4";
const char CDU_RSK_5_EVENT[] = "cdu_rsk_5";
const char CDU_RSK_6_EVENT[] = "cdu_rsk_6";
// Page select:
const char CDU_INIT_REF_EVENT[] = "cdu_init_ref";
const char CDU_RTE_EVENT[] = "cdu_rte";
const char CDU_DEP_ARR_EVENT[] = "cdu_dep_arr";
const char CDU_ALTN_EVENT[] = "cdu_altn";
const char CDU_VNAV_EVENT[] = "cdu_vnav";
const char CDU_FIX_EVENT[] = "cdu_fix";
const char CDU_LEGS_EVENT[] = "cdu_legs";
const char CDU_HOLD_EVENT[] = "cdu_hold";
const char CDU_FMC_COMM_EVENT[] = "cdu_fmc_comm";
const char CDU_PROG_EVENT[] = "cdu_prog";
const char CDU_MENU_EVENT[] = "cdu_menu";
const char CDU_NAV_RAD_EVENT[] = "cdu_nav_rad";
const char CDU_PREV_PAGE_EVENT[] = "cdu_prev_page";
const char CDU_NEXT_PAGE_EVENT[] = "cdu_next_page";
// Letters:
const char CDU_KEY_A_EVENT[] = "cdu_key_a";
const char CDU_KEY_B_EVENT[] = "cdu_key_b";
const char CDU_KEY_C_EVENT[] = "cdu_key_c";
const char CDU_KEY_D_EVENT[] = "cdu_key_d";
const char CDU_KEY_E_EVENT[] = "cdu_key_e";
const char CDU_KEY_F_EVENT[] = "cdu_key_f";
const char CDU_KEY_G_EVENT[] = "cdu_key_g";
const char CDU_KEY_H_EVENT[] = "cdu_key_h";
const char CDU_KEY_I_EVENT[] = "cdu_key_i";
const char CDU_KEY_J_EVENT[] = "cdu_key_j";
const char CDU_KEY_K_EVENT[] = "cdu_key_k";
const char CDU_KEY_L_EVENT[] = "cdu_key_l";
const char CDU_KEY_M_EVENT[] = "cdu_key_m";
const char CDU_KEY_N_EVENT[] = "cdu_key_n";
const char CDU_KEY_O_EVENT[] = "cdu_key_o";
const char CDU_KEY_P_EVENT[] = "cdu_key_p";
const char CDU_KEY_Q_EVENT[] = "cdu_key_q";
const char CDU_KEY_R_EVENT[] = "cdu_key_r";
const char CDU_KEY_S_EVENT[] = "cdu_key_s";
const char CDU_KEY_T_EVENT[] = "cdu_key_t";
const char CDU_KEY_U_EVENT[] = "cdu_key_u";
const char CDU_KEY_V_EVENT[] = "cdu_key_v";
const char CDU_KEY_W_EVENT[] = "cdu_key_w";
const char CDU_KEY_X_EVENT[] = "cdu_key_x";
const char CDU_KEY_Y_EVENT[] = "cdu_key_y";
const char CDU_KEY_Z_EVENT[] = "cdu_key_z";

const char CDU_KEY_SP_EVENT[] = "cdu_key_sp";
const char CDU_KEY_DELETE_EVENT[] = "cdu_key_delete";
const char CDU_KEY_SLASH_EVENT[] = "cdu_key_slash";
const char CDU_KEY_CLR_EVENT[] = "cdu_key_clr";

const char CDU_KEY_1_EVENT[] = "cdu_key_1";
const char CDU_KEY_2_EVENT[] = "cdu_key_2";
const char CDU_KEY_3_EVENT[] = "cdu_key_3";
const char CDU_KEY_4_EVENT[] = "cdu_key_4";
const char CDU_KEY_5_EVENT[] = "cdu_key_5";
const char CDU_KEY_6_EVENT[] = "cdu_key_6";
const char CDU_KEY_7_EVENT[] = "cdu_key_7";
const char CDU_KEY_8_EVENT[] = "cdu_key_8";
const char CDU_KEY_9_EVENT[] = "cdu_key_9";
const char CDU_KEY_DOT_EVENT[] = "cdu_key_dot";
const char CDU_KEY_0_EVENT[] = "cdu_key_0";

const char CDU_KEY_PM_EVENT[] = "cdu_key_pm";
const char CDU_KEY_EXEC_EVENT[] = "cdu_key_exec";

struct cdu_event_desc_t {
  const char* name;
  fms_displays::cdu_event_type event;
};

cdu_event_desc_t CDU_EVENTS[] = {
  {.name=CDU_LSK_1_EVENT, .event=fms_displays::CDU_KEY_LSK_TOP},
  {.name=CDU_LSK_2_EVENT, .event=fms_displays::CDU_KEY_LSK_TOP + 1},
  {.name=CDU_LSK_3_EVENT, .event=fms_displays::CDU_KEY_LSK_TOP + 2},
  {.name=CDU_LSK_4_EVENT, .event=fms_displays::CDU_KEY_LSK_TOP + 3},
  {.name=CDU_LSK_5_EVENT, .event=fms_displays::CDU_KEY_LSK_TOP + 4},
  {.name=CDU_LSK_6_EVENT, .event=fms_displays::CDU_KEY_LSK_TOP + 5},

  {.name=CDU_RSK_1_EVENT, .event=fms_displays::CDU_KEY_RSK_TOP},
  {.name=CDU_RSK_2_EVENT, .event=fms_displays::CDU_KEY_RSK_TOP + 1},
  {.name=CDU_RSK_3_EVENT, .event=fms_displays::CDU_KEY_RSK_TOP + 2},
  {.name=CDU_RSK_4_EVENT, .event=fms_displays::CDU_KEY_RSK_TOP + 3},
  {.name=CDU_RSK_5_EVENT, .event=fms_displays::CDU_KEY_RSK_TOP + 4},
  {.name=CDU_RSK_6_EVENT, .event=fms_displays::CDU_KEY_RSK_TOP + 5},

  {.name=CDU_INIT_REF_EVENT, .event=fms_displays::CDU_KEY_INIT_REF},
  {.name=CDU_RTE_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 1},
  {.name=CDU_DEP_ARR_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 2},
  {.name=CDU_ALTN_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 3},
  {.name=CDU_VNAV_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 4},
  {.name=CDU_FIX_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 5},
  {.name=CDU_LEGS_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 6},
  {.name=CDU_HOLD_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 7},
  {.name=CDU_FMC_COMM_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 8},
  {.name=CDU_PROG_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 9},
  {.name=CDU_MENU_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 10},
  {.name=CDU_NAV_RAD_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 11},
  {.name=CDU_PREV_PAGE_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 12},
  {.name=CDU_NEXT_PAGE_EVENT, .event=fms_displays::CDU_KEY_INIT_REF + 13},

  {.name=CDU_KEY_A_EVENT, .event=fms_displays::CDU_KEY_A},
  {.name=CDU_KEY_B_EVENT, .event=fms_displays::CDU_KEY_A + 1},
  {.name=CDU_KEY_C_EVENT, .event=fms_displays::CDU_KEY_A + 2},
  {.name=CDU_KEY_D_EVENT, .event=fms_displays::CDU_KEY_A + 3},
  {.name=CDU_KEY_E_EVENT, .event=fms_displays::CDU_KEY_A + 4},
  {.name=CDU_KEY_F_EVENT, .event=fms_displays::CDU_KEY_A + 5},
  {.name=CDU_KEY_G_EVENT, .event=fms_displays::CDU_KEY_A + 6},
  {.name=CDU_KEY_H_EVENT, .event=fms_displays::CDU_KEY_A + 7},
  {.name=CDU_KEY_I_EVENT, .event=fms_displays::CDU_KEY_A + 8},
  {.name=CDU_KEY_J_EVENT, .event=fms_displays::CDU_KEY_A + 9},
  {.name=CDU_KEY_K_EVENT, .event=fms_displays::CDU_KEY_A + 10},
  {.name=CDU_KEY_L_EVENT, .event=fms_displays::CDU_KEY_A + 11},
  {.name=CDU_KEY_M_EVENT, .event=fms_displays::CDU_KEY_A + 12},
  {.name=CDU_KEY_N_EVENT, .event=fms_displays::CDU_KEY_A + 13},
  {.name=CDU_KEY_O_EVENT, .event=fms_displays::CDU_KEY_A + 14},
  {.name=CDU_KEY_P_EVENT, .event=fms_displays::CDU_KEY_A + 15},
  {.name=CDU_KEY_Q_EVENT, .event=fms_displays::CDU_KEY_A + 16},
  {.name=CDU_KEY_R_EVENT, .event=fms_displays::CDU_KEY_A + 17},
  {.name=CDU_KEY_S_EVENT, .event=fms_displays::CDU_KEY_A + 18},
  {.name=CDU_KEY_T_EVENT, .event=fms_displays::CDU_KEY_A + 19},
  {.name=CDU_KEY_U_EVENT, .event=fms_displays::CDU_KEY_A + 20},
  {.name=CDU_KEY_V_EVENT, .event=fms_displays::CDU_KEY_A + 21},
  {.name=CDU_KEY_W_EVENT, .event=fms_displays::CDU_KEY_A + 22},
  {.name=CDU_KEY_X_EVENT, .event=fms_displays::CDU_KEY_A + 23},
  {.name=CDU_KEY_Y_EVENT, .event=fms_displays::CDU_KEY_A + 24},
  {.name=CDU_KEY_Z_EVENT, .event=fms_displays::CDU_KEY_A + 25},

  {.name=CDU_KEY_SP_EVENT, .event=fms_displays::CDU_KEY_SP},
  {.name=CDU_KEY_DELETE_EVENT, .event=fms_displays::CDU_KEY_DELETE},
  {.name=CDU_KEY_SLASH_EVENT, .event=fms_displays::CDU_KEY_SLASH},
  {.name=CDU_KEY_CLR_EVENT, .event=fms_displays::CDU_KEY_CLR},

  {.name=CDU_KEY_1_EVENT, .event=fms_displays::CDU_KEY_1},
  {.name=CDU_KEY_2_EVENT, .event=fms_displays::CDU_KEY_1 + 1},
  {.name=CDU_KEY_3_EVENT, .event=fms_displays::CDU_KEY_1 + 2},
  {.name=CDU_KEY_4_EVENT, .event=fms_displays::CDU_KEY_1 + 3},
  {.name=CDU_KEY_5_EVENT, .event=fms_displays::CDU_KEY_1 + 4},
  {.name=CDU_KEY_6_EVENT, .event=fms_displays::CDU_KEY_1 + 5},
  {.name=CDU_KEY_7_EVENT, .event=fms_displays::CDU_KEY_1 + 6},
  {.name=CDU_KEY_8_EVENT, .event=fms_displays::CDU_KEY_1 + 7},
  {.name=CDU_KEY_9_EVENT, .event=fms_displays::CDU_KEY_1 + 8},
  {.name=CDU_KEY_DOT_EVENT, .event=fms_displays::CDU_KEY_DOT},
  {.name=CDU_KEY_0_EVENT, .event=fms_displays::CDU_KEY_0},

  {.name=CDU_KEY_PM_EVENT, .event=fms_displays::CDU_KEY_PM},
  {.name=CDU_KEY_EXEC_EVENT, .event=fms_displays::CDU_KEY_EXEC}
};

// LEGS:
constexpr int SPDCSTR_MX_KT = 340;
constexpr int SPDCSTR_MN_KT = 100;
constexpr int ALTCSTR_MX_FT = 41000;
constexpr int ALTCSTR_MN_FT = 100;

constexpr std::size_t N_CDU_ITM_PP =
    5;  // How many items can be drawn on 1 page
constexpr std::size_t N_CDU_RTES = 2;

constexpr std::size_t N_FLT_NBR_CHR_MAX =
    10;  // Maximum number of characters for flight number

constexpr std::size_t LEGS_BASE_IDX = 2;

constexpr std::size_t N_LEG_PROP_ROWS = 14;
constexpr std::size_t N_LEG_CRS_ROWS = 5;
constexpr std::size_t N_LEG_CSTR_ROWS = 11;
constexpr std::size_t N_LEG_VCSTR_ROWS = 6;

constexpr std::size_t N_LEG_SPDCSTR_MX_LN = 4;
constexpr std::size_t N_LEG_SPDCSTR_MN_LN = 3;
constexpr std::size_t N_LEG_ALTCSTR_MX_LN = N_LEG_VCSTR_ROWS;
constexpr std::size_t N_LEG_ALTCSTR_MN_LN = 2;

constexpr std::size_t CNT_CDU_EVENTS_PER_FRAME = 1;

constexpr double CDU_RES_COEFF = 1.0 / 900.0;
constexpr double CDU_V_OFFS_FIRST_SM = 0.082;
constexpr double CDU_V_OFFS_FIRST_BIG = 0.095;
constexpr double CDU_V_OFFS_SMALL_FIRST = 0.027;
constexpr double CDU_V_OFFS_REG = 0.134;  // * screen height
constexpr double CDU_SMALL_TEXT_OFFS_X = 0.003;
constexpr double CDU_BIG_TEXT_OFFS = 0.05;
constexpr double CDU_TEXT_INTV = 0.041422;
constexpr double CDU_LETTER_WIDTH = 21;
constexpr double CDU_LETTER_HEIGHT = 39;

// Constraint designations used on LEGS page
constexpr char LEGS_CSTR_ABV = 'A';
constexpr char LEGS_CSTR_BLW = 'B';
constexpr char LEGS_CSTR_SEP = '/';

constexpr geom::vect2_t DISPLAY_OFFS = {0.14, 0.068};
constexpr geom::vect2_t DISPLAY_SZ = {0.717, 0.378};
constexpr geom::vect2_t CDU_SMALL_TEXT_SZ = {0.7, 0.7};
constexpr geom::vect2_t CDU_BIG_TEXT_SZ = {0.84, 0.87};

const std::string NAV_DATA_EXPIRED_MSG = "NAV DATA OUT OF DATE";
const std::string INVALID_ENTRY_MSG = "INVALID ENTRY";
const std::string NOT_IN_DB_MSG = "NOT IN DATA BASE";
const std::string INVALID_DELETE_MSG = "INVALID DELETE";
const std::string INVALID_RTE_UPLINK_MSG = "INVALID ROUTE UPLINK";
const std::string DELETE_MSG = "DELETE";

const std::string DISCO_AFTER_SEG = "-- ROUTE DISCONTINUITY -";
const std::string SEG_LAST = "-------            -----";
const std::string SEL_DES_WPT_HDG = " SELECT DESIRED WPT";
// DEP ARR page:
const std::string DEP_ARR_HDG = "DEP/ARR INDEX";
const std::string DEP_ARR_IDX_DASH_L = std::string(8, '-');
const std::string DEP_ARR_IDX_DASH_R = std::string(10, '-');
const std::string DEP_ARR_DEP_OPT = "<DEP" + std::string(6, ' ');
const std::string DEP_ARR_ARR_OPT = std::string(6, ' ') + "ARR>";
const std::string DEP_ARR_IDX_OTHER = " DEP      OTHER      ARR";
const std::string DEP_ARR_ARROWS =
    "<----" + std::string(fms_displays::N_CDU_DATA_COLS - 10, ' ') + "---->";
const std::string DEP_ARR_BOTTOM_INACT = "<INDEX            ROUTE>";
const std::string DEP_ARR_BOTTOM_ACT = "<ERASE            ROUTE>";
const std::string DEP_ARR_NO_PROC = "-NONE-";
// DEP page
const std::string DEP_COLS1 = " SIDS    RTE 1   RUNWAYS";
const std::string DEP_COLS2 = " SIDS    RTE 2   RUNWAYS";
// ARR page
const std::string ARR_COLS1 = " STARS   RTE 1APPROACHES";
const std::string ARR_COLS2 = " STARS   RTE 2APPROACHES";
const std::string ARR_RWYS_STARS1 = " STARS   RTE 1   RUNWAYS";
const std::string ARR_RWYS_STARS2 = " STARS   RTE 2   RUNWAYS";
const std::string ARR_RWYS = std::string(17, ' ') + "RUNWAYS";
// LEGS page:
const char LEG_MAP_CTR[] = "<CTR> ";
const std::string LEGS_BTM_ACT = " LEGS    RTE DATA>";
const std::string LEGS_BTM_MOD = "<ERASE         RTE DATA>";
const std::string LEGS_BTM_INACT = " LEGS    ACTIVATE>";
const char LEGS_BTM_PLN[] = " LEGS        STEP>";
const char LEGS_BTM_MOD_PLN[] = "<ERASE             STEP>";
const char LEGS_MAP_CTR_BORD[] = "-----------------MAP CTR";
const std::string LEG_LAST = std::string(5, '-');
const std::string LEG_NO_SPD = std::string(4, '-');
const std::string LEG_NO_ALT = std::string(6, '-');
const std::string LEG_BIG_SPACING = std::string(8, ' ');
const std::string DISCO_LEG_NM = std::string(5, '@');
const std::string LEG_VECTORS = "(VECTOR)";
const std::string LEG_HOLD = "HOLD";
const std::string DISCO_THEN = " THEN";
const std::string HOLD_DESC = " HOLD AT";
const std::string NAUT_MILES = "NM";
const std::string LEG_BYPASS = "BYPASS";
// MISC:
const std::string ALL_DASH = std::string(fms_displays::N_CDU_DATA_COLS, '-');
const std::string ERASE_NML = "<ERASE";
const std::string ACT = "ACT";
const std::string SEL = "SEL";
const std::string MOD = "MOD";
const std::string RTE_COPY = "RTE COPY";
const std::string COMPLETE = "COMPLETE";

// Texture names:
const char CDU_WHITE_TEXT_NAME[] = "cdu_big_white";
const char CDU_GREEN_TEXT_NAME[] = "cdu_big_green";
const char CDU_CYAN_TEXT_NAME[] = "cdu_big_cyan";
const char CDU_MAGENTA_TEXT_NAME[] = "cdu_big_magenta";

const char* CDU_TEXTURE_ARRAY[] = {CDU_WHITE_TEXT_NAME, CDU_GREEN_TEXT_NAME,
                                   CDU_CYAN_TEXT_NAME, CDU_MAGENTA_TEXT_NAME};



bool check_event_select_key(fms_displays::cdu_event_type event) {
  if(event >= fms_displays::CDU_KEY_LSK_TOP && 
    event < fms_displays::CDU_KEY_LSK_TOP + CNT_CDU_SELECT_KEYS) {
    return true;
  }
  if(event >= fms_displays::CDU_KEY_RSK_TOP && 
    event < fms_displays::CDU_KEY_RSK_TOP + CNT_CDU_SELECT_KEYS) {
    return true;
  }
  return false;
}

std::pair<std::string, std::string> get_segment_endpoint_names(
  const fms_core::list_node_ref_t<fms_core::fpl_seg_t>& node) {
  std::pair<std::string, std::string> out;
  out.first = node.data.name;
  fms_core::leg_list_node_t* end_leg = node.data.end;
  if(end_leg != nullptr) {
    out.second = end_leg->data.leg.main_fix.id;
  }
  return out;
}
}  // namespace

namespace fms_displays {

util::const_str_data_t GetCduTextureNames() {
  return util::const_str_data_t{.ptr = CDU_TEXTURE_ARRAY,
                                .size = MY_ARRAY_SIZE(CDU_TEXTURE_ARRAY)};
}

// CDU definitions:
// Public member functions:

CDU::CDU(util::OpaquePointer<fms_core::FPLSys> fs, size_t sd_idx) : 
  nd_mode_{fms_core::NDMode::MAX}, fpl_sys_{fs}, cntx_{sd_idx, 
    fms_core::RTE1_IDX, fms_core::N_FPL_SYS_RTES, fs}, ident_{fs}, 
  pos_init_{fs}, menu_{fs}, init_ref_index_{fs}, legs_{fs, 
    util::OpaquePointer{&cntx_}} {
  act_sd_idx_ = sd_idx;

  airport_db_ = fs->get_arpt_db_ptr();
  navaid_db_ = fs->get_navaid_db_ptr();

  fpl_sys_ = fs;
  fpln_ = fs->get_fpln_ptr(cntx_.sel_fpl_idx);
  m_rte1_ptr_ = fs->get_fpln_ptr(fms_core::RTE1_IDX);
  m_rte2_ptr_ = fs->get_fpln_ptr(fms_core::RTE2_IDX);
  m_act_ptr_ = fs->get_fpln_ptr(fms_core::ACT_RTE_IDX);

  dep_arr_rwy_filter_ = std::vector<bool>(N_CDU_RTES, false);
  dep_arr_rwy_filter_ = std::vector<bool>(N_CDU_RTES, false);
  dep_arr_proc_filter_ = std::vector<bool>(N_CDU_RTES, false);
  dep_arr_trans_filter_ = std::vector<bool>(N_CDU_RTES, false);
  dep_arr_via_filter_ = std::vector<bool>(N_CDU_RTES, false);

  procedures_ = std::vector<std::vector<std::string>>(N_CDU_RTES);
  transitions_ = std::vector<std::vector<std::string>>(N_CDU_RTES);
  approaches_ = std::vector<std::vector<std::string>>(N_CDU_RTES);
  rwys_ = std::vector<std::vector<std::string>>(N_CDU_RTES);
  vias_ = std::vector<std::vector<std::string>>(N_CDU_RTES);
  fpl_infos_ = std::vector<fms_core::fpln_info_t>(fms_core::N_FPL_SYS_RTES);
  leg_sel_ = std::vector<std::pair<size_t, double>>(N_CDU_RTES, {0LL, -1.0});
  pln_ctr_idx_ = std::vector<size_t>(N_CDU_RTES, 0);
  pln_ctr_pos_ = std::vector<geo::point>(N_CDU_RTES, {0, 0});
}

void CDU::update() noexcept {
  std::unique_lock lk(main_mutex_);

  nd_mode_ = fpl_sys_->get_nd_mode(act_sd_idx_);
  seg_list_ = fpl_sys_->get_seg_list(&n_seg_list_sz_, cntx_.sel_fpl_idx);
  leg_list_ = fpl_sys_->get_leg_list(&n_leg_list_sz_, cntx_.sel_fpl_idx);
  fpln_ = fpl_sys_->get_fpln_ptr(cntx_.sel_fpl_idx);
  cntx_.act_fpl_idx = fpl_sys_->get_act_idx();
  pos_init_.update();
  legs_.update();

  if (sel_des_) {
    n_subpg_ = get_n_sel_des_subpg();
  } else {
    if (curr_page_ == CDUPage::RTE) {
      rte_copy_ = fpl_sys_->act_can_copy();
      n_subpg_ = get_n_rte_subpg();
    } else if (curr_page_ == CDUPage::DEP_ARR_INTRO ||
               curr_page_ == CDUPage::DEP1 || curr_page_ == CDUPage::ARR1 ||
               curr_page_ == CDUPage::DEP2 || curr_page_ == CDUPage::ARR2) {
      bool is_rte2 = curr_page_ == CDUPage::DEP2 || curr_page_ == CDUPage::ARR2;
      n_subpg_ = get_n_dep_arr_subpg(is_rte2);
    }
  }

  if (curr_subpg_ > n_subpg_) {
    curr_subpg_ = 1;
  }

  update_fpl_infos();
  fpl_sys_->set_cdu_sel_fpl_idx(cntx_.sel_fpl_idx, act_sd_idx_);
}

bool CDU::get_exec_lt() const noexcept {
  std::shared_lock lk(main_mutex_);
  bool e_st = fpl_sys_->get_exec();
  if (cntx_.act_fpl_idx == cntx_.sel_fpl_idx) return e_st;
  return false;
}

std::string CDU::on_event(int event_key, std::string scratchpad,
                          std::string* s_out) noexcept {
  std::unique_lock lk(main_mutex_);

  return on_event_impl(event_key, scratchpad, s_out);
}

cdu_pages::cdu_scr_data_t CDU::get_screen_data() const noexcept {
  std::shared_lock lk(main_mutex_);

  if (cntx_.select_desired.get_state() == 
    cdu_pages::SelectDesired::State::WAIT) { 
    return cntx_.select_desired.get_screen_data(); 
  }

  if (curr_page_ == CDUPage::MENU) { return menu_.get_screen_data(); }

  if(curr_page_ == CDUPage::IDENT) { return ident_.get_screen_data(); }

  if(curr_page_ == CDUPage::POS_INIT) { return pos_init_.get_screen_data(); }

  if(curr_page_ == CDUPage::INIT_REF_INDEX) { return init_ref_index_.get_screen_data(); }

  if (curr_page_ == CDUPage::RTE) { return get_rte_page(); }

  if (curr_page_ == CDUPage::DEP_ARR_INTRO) { return get_dep_arr_page(); }

  if (curr_page_ == CDUPage::DEP1) { return get_dep_page(false); }

  if (curr_page_ == CDUPage::ARR1) { return get_arr_page(false); }

  if (curr_page_ == CDUPage::DEP2) { return get_dep_page(true); }

  if (curr_page_ == CDUPage::ARR2) { return get_arr_page(true); }

  if (curr_page_ == CDUPage::LEGS) { return legs_.get_screen_data(); }

  return {};
}

// Private member functions:

std::string CDU::str_align_right(const std::string& str) {
  if(str.size() > N_CDU_DATA_COLS) {
    std::size_t diff = str.size() - N_CDU_DATA_COLS;
    return str.substr(diff);
  }
  std::size_t diff = N_CDU_DATA_COLS - str.size();
  std::string out = std::string(diff, ' ') + str;
  return out;
}

std::string CDU::get_cdu_line(std::string in, std::string line,
                              bool align_right) {
  assert(line.size() + in.size() <= N_CDU_DATA_COLS);
  if (!align_right) {
    return in;
  } else {
    size_t n_sp = N_CDU_DATA_COLS - in.size() - line.size();
    return line + std::string(n_sp, ' ') + in;
  }
}

bool CDU::scratchpad_has_delete(const std::string& scratchpad) {
  if (scratchpad.size() && scratchpad[0] == DELETE_SYMBOL) return 1;
  return 0;
}

// Non-static member-functions:

std::string CDU::on_event_impl(int event_key, std::string scratchpad,
                       std::string* s_out) noexcept {
  if (event_key == CDU_KEY_EXEC) {
    if (cntx_.act_fpl_idx == cntx_.sel_fpl_idx) fpl_sys_->execute();
    return "";
  }

  if (event_key > CDU_KEY_RSK_TOP + 5 && event_key < CDU_KEY_A) {
    CDUPage pg = CDU_PAGE_FACES[event_key - CDU_KEY_RSK_TOP - 6];

    if(cntx_.select_desired.get_state() == cdu_pages::SelectDesired::State::WAIT) {
      cntx_.select_desired.on_page_change(pg);
      *s_out = scratchpad;
      return "";
    } else if(pg == CDUPage::LEGS || curr_page_ == CDUPage::LEGS) {
      legs_.on_page_change(pg);
    }
    if (pg == CDUPage::NEXT_PAGE) {
      curr_subpg_++;
    } else if (pg == CDUPage::PREV_PAGE) {
      curr_subpg_--;
    } else {
      set_page(pg);
    }

    if (curr_subpg_ > n_subpg_) {
      curr_subpg_ = 1;
    } else if (curr_subpg_ == 0) {
      curr_subpg_ = n_subpg_;
    }

    *s_out = scratchpad;

    return "";
  }

  std::string msg = "";
  CDUPage page_number;
  cdu_pages::cdu_event_res_t res{.err=CDUError::NONE, .page=CDUPage::MENU};
  bool handle_new = false;
  if(cntx_.select_desired.get_state() == cdu_pages::SelectDesired::State::WAIT) {
    res = cntx_.select_desired.on_event(event_key, scratchpad, *s_out);
    if(cntx_.select_desired.get_state() == cdu_pages::SelectDesired::State::SUCCESS) {
      return on_event_impl(sel_des_event_, "", s_out);
    } else {
      page_number = curr_page_;
      handle_new = true;
    }
  } else if(curr_page_ == CDUPage::MENU) {
    res = menu_.on_event(event_key, scratchpad, *s_out);
    page_number = CDUPage::MENU;
    handle_new = true;
  } else if(curr_page_ == CDUPage::IDENT) {
    res = ident_.on_event(event_key, scratchpad, *s_out);
    page_number = CDUPage::IDENT;
    handle_new = true;
  } else if(curr_page_ == CDUPage::POS_INIT) {
    res = pos_init_.on_event(event_key, scratchpad, *s_out);
    page_number = CDUPage::POS_INIT;
    handle_new = true;
  } else if (curr_page_ == CDUPage::INIT_REF_INDEX) {
    res = init_ref_index_.on_event(event_key, scratchpad, *s_out);
    page_number = CDUPage::INIT_REF_INDEX;
    handle_new = true;
  } else if (curr_page_ == CDUPage::RTE) {
    msg = handle_rte(event_key, scratchpad, s_out);
  } else if (curr_page_ == CDUPage::DEP_ARR_INTRO) {
    msg = handle_dep_arr(event_key);
  } else if (curr_page_ == CDUPage::DEP1) {
    msg = handle_dep(event_key, false);
  } else if (curr_page_ == CDUPage::ARR1) {
    msg = handle_arr(event_key, false);
  } else if (curr_page_ == CDUPage::DEP2) {
    msg = handle_dep(event_key, true);
  } else if (curr_page_ == CDUPage::ARR2) {
    msg = handle_arr(event_key, true);
  } else if (curr_page_ == CDUPage::LEGS) {
    res = legs_.on_event(event_key, scratchpad, *s_out);
    page_number = CDUPage::LEGS;
    handle_new = true;
  }
  if(handle_new) {
    if(res.err != fms_displays::CDUError::NONE) {
      msg = cdu_pages::string_from_error(res.err);
    } else if(res.page != page_number) {
      set_page(res.page);
    }
  }
  if (cntx_.select_desired.get_state() == cdu_pages::SelectDesired::State::WAIT) { 
    sel_des_event_ = event_key; 
  }

  return msg;
}

void CDU::update_fpl_infos() noexcept {
  for (size_t i = 0; i < fpl_infos_.size(); i++) {
    fpl_infos_[i] = fpl_sys_->get_fpl_info(i);
  }
}

void CDU::set_page(CDUPage pg) {
  curr_subpg_ = 1;
  curr_page_ = pg;
  sel_des_idx_ = -1;
  sel_des_ = false;

  leg_sel_pr_ = false;

  if(curr_page_ == CDUPage::LEGS) {
    fpl_sys_->reset_ctr(act_sd_idx_);
  }

  for (size_t i = 0; i < N_CDU_RTES; i++) {
    dep_arr_rwy_filter_[i] = false;
    dep_arr_proc_filter_[i] = false;
    dep_arr_trans_filter_[i] = false;
    dep_arr_via_filter_[i] = false;
  }
}

void CDU::set_sel_des_state(double seg_id, double leg_id, std::string& name,
                            std::vector<libnav::waypoint_entry_t>& w_e) {
  sel_des_seg_id_ = seg_id;
  sel_des_leg_id_ = leg_id;
  sel_des_data_ = w_e;
  sel_des_nm_ = name;
  sel_des_subpg_ = curr_subpg_;
  curr_subpg_ = 1;
  sel_des_ = true;
}

libnav::waypoint_t CDU::get_wpt_from_user(std::string name, double seg_id,
                                          double leg_id, bool* not_in_db,
                                          bool* inv_ent, bool* wait_sel,
                                          bool* sel_used) {
  UNUSED(name);
  UNUSED(seg_id);
  UNUSED(leg_id);
  *not_in_db = true;
  UNUSED(inv_ent);
  UNUSED(wait_sel);
  UNUSED(sel_used);
  return libnav::waypoint_t{};
}

std::string CDU::set_departure(std::string icao, std::string* s_out) {
  if (icao == "") *s_out = fpln_->get_dep_icao();
  if (icao.size() != 4) return INVALID_ENTRY_MSG;
  libnav::DbErr err = fpln_->set_dep(icao);
  if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
    return NOT_IN_DB_MSG;
  }

  return "";
}

std::string CDU::set_arrival(std::string icao, std::string* s_out) {
  if (icao == "") *s_out = fpln_->get_arr_icao();

  libnav::DbErr err = fpln_->set_arr(icao);
  if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
    return NOT_IN_DB_MSG;
  }

  return "";
}

std::string CDU::set_flt_nbr(std::string nbr) {
  if (nbr.size() > N_FLT_NBR_CHR_MAX) return INVALID_ENTRY_MSG;

  if (nbr != "" && nbr[0] != DELETE_SYMBOL)
    fpl_sys_->set_flt_nbr(nbr);
  else if (nbr[0] == DELETE_SYMBOL)
    fpl_sys_->set_flt_nbr("");

  return "";
}

std::string CDU::set_dep_rwy(std::string id) {
  std::string dep_icao = fpln_->get_dep_icao();
  std::string arr_icao = fpln_->get_arr_icao();
  if (id.size() > 3 || dep_icao == "" || arr_icao == "")
    return INVALID_ENTRY_MSG;

  bool rwy_set = fpln_->set_dep_rwy(id);

  if (!rwy_set) return NOT_IN_DB_MSG;

  return "";
}

std::string CDU::load_rte() {
  std::string dep_nm = fpln_->get_dep_icao();
  std::string arr_nm = fpln_->get_arr_icao();

  if (dep_nm != "" && arr_nm != "") {
    std::string file_nm = (fpl_sys_->get_fpln_dir() + (dep_nm + arr_nm)).Get();
    libnav::DbErr err = fpln_->load_from_fms(file_nm, false);
    if (err != libnav::DbErr::SUCCESS) {
      return INVALID_RTE_UPLINK_MSG;
    }
  }

  return "";
}

std::string CDU::save_rte() {
  std::string dep_nm = fpln_->get_dep_icao();
  std::string arr_nm = fpln_->get_arr_icao();

  if (dep_nm != "" && arr_nm != "") {
    std::string out_nm = (fpl_sys_->get_fpln_dir() + (dep_nm + arr_nm)).Get();
    fpln_->save_to_fms(out_nm);
  }

  return "";
}

std::string CDU::add_via(size_t next_idx, std::string name) {
  fms_core::fpln_info_t f_inf = fpl_sys_->get_fpl_info(cntx_.sel_fpl_idx);
  double id = f_inf.seg_list_id;
  if (name.size() > 7) return INVALID_ENTRY_MSG;
  fms_core::seg_list_node_t* s_ptr = nullptr;
  if (next_idx < n_seg_list_sz_) {
    s_ptr = seg_list_[next_idx].ptr;
  }

  bool retval = fpln_->add_enrt_seg({s_ptr, id}, name);

  if (!retval) return NOT_IN_DB_MSG;
  return "";
}

std::string CDU::delete_via(size_t next_idx) {
  fms_core::fpln_info_t f_inf = fpl_infos_[cntx_.sel_fpl_idx];
  double id = f_inf.seg_list_id;
  fms_core::seg_list_node_t* s_ptr = nullptr;
  if (next_idx < n_seg_list_sz_) {
    s_ptr = seg_list_[next_idx].ptr;
  }
  bool retval = fpln_->delete_via({s_ptr, id});

  if (!retval) return INVALID_DELETE_MSG;
  return "";
}

std::string CDU::add_to(size_t next_idx, std::string name) {
  fms_core::fpln_info_t f_inf = fpl_infos_[cntx_.sel_fpl_idx];
  double sg_id = f_inf.seg_list_id;
  double lg_id = f_inf.leg_list_id;

  bool inv_ent = 0, not_in_db = 0, wait_sel = 0, sel_used = 0;
  libnav::waypoint_t tgt_wpt = get_wpt_from_user(
      name, sg_id, lg_id, &not_in_db, &inv_ent, &wait_sel, &sel_used);
  if (inv_ent) return INVALID_ENTRY_MSG;
  if (not_in_db) return NOT_IN_DB_MSG;
  if (wait_sel) return "";

  if (sel_used) {
    sg_id = sel_des_seg_id_;  // Only need segment id here
  }

  fms_core::seg_list_node_t* s_ptr = nullptr;
  if (next_idx < n_seg_list_sz_) {
    s_ptr = seg_list_[next_idx].ptr;
  }

  bool retval = fpln_->awy_insert({s_ptr, sg_id}, tgt_wpt);

  if (!retval) return NOT_IN_DB_MSG;
  return "";
}

std::string CDU::delete_to(size_t next_idx) {
  fms_core::fpln_info_t f_inf = fpl_infos_[cntx_.sel_fpl_idx];
  double id = f_inf.seg_list_id;
  fms_core::seg_list_node_t* s_ptr = nullptr;
  if (next_idx < n_seg_list_sz_) {
    s_ptr = seg_list_[next_idx].ptr;
  }

  bool retval = fpln_->delete_seg_end({s_ptr, id});

  if (!retval) return INVALID_DELETE_MSG;
  return "";
}

void CDU::get_seg_page(cdu_pages::cdu_scr_data_t* in) const noexcept {
  std::string via_to = " VIA" + std::string(N_CDU_DATA_COLS - 6, ' ') + "TO";
  in->data_lines.push_back(via_to);

  size_t i_start = get_seg_start_idx();
  size_t i_end = get_seg_end_idx();

  for (size_t i = i_start; i < i_end; i++) {
    auto curr_sg = seg_list_[i];
    auto[seg_nm, end_nm] = get_segment_endpoint_names(curr_sg);

    if (seg_nm == fms_core::DISCON_SEG_NAME) {
      seg_nm = std::string(7, '-');
    }
    if (end_nm == "") {
      in->data_lines[in->data_lines.size() - 1] = std::string(10, ' ') + "THEN";
      end_nm = std::string(5, '@');
      std::string curr_seg =
          seg_nm +
          std::string(N_CDU_DATA_COLS - seg_nm.size() - end_nm.size(), ' ') +
          end_nm;
      in->data_lines.push_back(curr_seg);
      in->data_lines.push_back(DISCO_AFTER_SEG);
    } else {
      std::string seg_nm = curr_sg.data.name;
      std::string curr_seg =
          seg_nm +
          std::string(N_CDU_DATA_COLS - seg_nm.size() - end_nm.size(), ' ') +
          end_nm;
      in->data_lines.push_back(curr_seg);
      if (i < i_start + 4) in->data_lines.push_back("");
    }
  }
  if (i_end - i_start < N_CDU_ITM_PP) in->data_lines.push_back(SEG_LAST);

  while (in->data_lines.size() < 10) {
    in->data_lines.push_back("");
  }
}

std::string CDU::get_sts(std::string& cr, std::string& act) const noexcept {
  std::string sts = "<" + SEL + ">";
  if (cr == act) sts = "<" + ACT + ">";
  return sts;
}

void CDU::get_procs(cdu_pages::cdu_scr_data_t* in, std::string curr_proc,
                    std::string curr_trans, std::string act_proc,
                    std::string act_trans, bool rte2) const noexcept {
  size_t start_idx = size_t((curr_subpg_ - 1) * 5);
  size_t j = 1;

  if (curr_proc != "" && procedures_[rte2].size() == 1) start_idx = 0;

  if (!procedures_[rte2].size()) {
    in->data_lines[j] = DEP_ARR_NO_PROC;
    j += 2;
  }

  for (size_t i = start_idx; i < start_idx + 6 && i < procedures_[rte2].size();
       i++) {
    std::string curr = procedures_[rte2][i];
    if (curr == curr_proc) {
      curr = curr + std::string(6 - curr.size(), ' ') +
             get_sts(curr_proc, act_proc);
    }

    in->data_lines[j] = curr;
    j += 2;
  }
  if (transitions_[rte2].size() && procedures_[rte2].size() == 1) {
    size_t trans_start = size_t((curr_subpg_ - 1) * 4);
    if (trans_start < transitions_[rte2].size())
      in->data_lines[j - 1] = " TRANS";

    if (curr_trans == "") curr_trans = libnav::NONE_TRANS;
    for (size_t i = trans_start;
         i < trans_start + 4 && i < transitions_[rte2].size(); i++) {
      std::string curr = transitions_[rte2][i];
      if (curr == curr_trans) {
        curr = curr + std::string(6 - curr.size(), ' ') +
               get_sts(curr_trans, act_trans);
      }
      in->data_lines[j] = curr;
      j += 2;
    }
  } else if (procedures_[rte2].size() == 1 && dep_arr_proc_filter_[rte2] &&
             curr_subpg_ == 1) {
    in->data_lines[j - 1] = " TRANS";
    in->data_lines[j] = DEP_ARR_NO_PROC;
  }
}

void CDU::get_rwys(cdu_pages::cdu_scr_data_t* in, std::string curr_rwy,
                   std::string act_rwy, bool rte2, std::string curr_appr,
                   std::string curr_via, std::string act_appr,
                   std::string act_via, bool get_appr) const noexcept {
  size_t start_idx = size_t((curr_subpg_ - 1) * 5);
  size_t j = 1;

  bool draw_rwys = true;

  if (get_appr) {
    draw_rwys = arr_has_rwys(curr_appr, rte2);
    if (!draw_rwys) start_idx = 0;

    if (curr_appr != "") curr_rwy = "";
    for (size_t i = start_idx;
         i <= start_idx + N_CDU_ITM_PP && i < approaches_[rte2].size(); i++) {
      std::string curr = approaches_[rte2][i];
      if (curr == curr_appr) {
        curr = get_sts(curr_appr, act_appr) +
               std::string(7 - curr.size(), ' ') + curr;
      }
      in->data_lines[j] = get_cdu_line(curr, in->data_lines[j], true);
      j += 2;
    }

    if (approaches_[rte2].size() == 1 && curr_appr != "") {
      size_t via_idx = 4 * (curr_subpg_ - 1);
      if (j - 1 > 0 && (via_idx < vias_[rte2].size() || curr_subpg_ == 1))
        in->data_lines[j - 1] =
            get_cdu_line("TRANS ", in->data_lines[j - 1], true);

      if (!vias_[rte2].size() && curr_subpg_ == 1) {
        in->data_lines[j] =
            get_cdu_line(DEP_ARR_NO_PROC, in->data_lines[j], true);
        j += 2;
      }

      for (size_t i = via_idx; i < via_idx + 4 && i < vias_[rte2].size(); i++) {
        std::string curr = vias_[rte2][i];
        if (curr == curr_via) {
          curr = get_sts(curr_via, act_via) +
                 std::string(7 - curr.size(), ' ') + curr;
        }
        in->data_lines[j] = get_cdu_line(curr, in->data_lines[j], true);
        j += 2;
      }
    }
  }

  if (draw_rwys) {
    if (start_idx >= approaches_[rte2].size()) {
      start_idx -= approaches_[rte2].size();
    } else {
      start_idx = 0;
    }
    if (j - 1 < in->data_lines.size() && get_appr) {
      if (j - 1) {
        in->data_lines[j - 1] = ARR_RWYS;
      } else {
        if (rte2)
          in->data_lines[j - 1] = ARR_RWYS_STARS2;
        else
          in->data_lines[j - 1] = ARR_RWYS_STARS1;
      }
    }

    for (size_t i = start_idx;
         i <= start_idx + N_CDU_ITM_PP && i < rwys_[rte2].size(); i++) {
      if (j > 11) break;
      std::string curr = rwys_[rte2][i];
      if (curr == curr_rwy) {
        curr = get_sts(curr_rwy, act_rwy) + std::string(7 - curr.size(), ' ') +
               curr;
      }
      in->data_lines[j] = get_cdu_line(curr, in->data_lines[j], true);
      j += 2;
    }
  }
}

std::string CDU::get_small_heading() const noexcept {
  std::string curr_spg = std::to_string(curr_subpg_);
  std::string n_spg = std::to_string(n_subpg_);
  std::string out = curr_spg + "/" + n_spg + " ";
  out = std::string(size_t(N_CDU_DATA_COLS) - out.size(), ' ') + out;
  return out;
}

void CDU::set_procs(fms_core::ProcType ptp, bool is_arr, bool rte2) {
  util::OpaquePointer c_fpl = m_rte1_ptr_;
  if (rte2) {
    c_fpl = m_rte2_ptr_;
  }

  procedures_[rte2] = c_fpl->get_arpt_proc(
      ptp, is_arr, dep_arr_rwy_filter_[rte2], dep_arr_proc_filter_[rte2]);
  sort(procedures_[rte2].begin(), procedures_[rte2].end());
  if (ptp == fms_core::PROC_TYPE_STAR) {
    approaches_[rte2] = c_fpl->get_arpt_proc(fms_core::PROC_TYPE_APPCH, is_arr,
                                             dep_arr_rwy_filter_[rte2],
                                             dep_arr_proc_filter_[rte2]);
    sort(approaches_[rte2].begin(), approaches_[rte2].end());
  } else {
    approaches_[rte2] = {};
  }
  if (dep_arr_proc_filter_[rte2]) {
    if (!dep_arr_trans_filter_[rte2]) {
      transitions_[rte2] =
          c_fpl->get_arpt_proc_trans(ptp, false, is_arr, false);
      sort(transitions_[rte2].begin(), transitions_[rte2].end());
    } else {
      transitions_[rte2] = {c_fpl->get_curr_proc(ptp, true)};
    }
    if (ptp == fms_core::PROC_TYPE_STAR) {
      vias_[rte2] = c_fpl->get_arpt_proc_trans(fms_core::PROC_TYPE_APPCH, false,
                                               is_arr, false);
      sort(vias_[rte2].begin(), vias_[rte2].end());
    } else {
      vias_[rte2] = {};
    }
  } else {
    transitions_[rte2] = {};
    vias_[rte2] = {};
  }

  if (!is_arr)
    rwys_[rte2] = c_fpl->get_dep_rwys(dep_arr_rwy_filter_[rte2],
                                      dep_arr_proc_filter_[rte2]);
  else
    rwys_[rte2] = c_fpl->get_arr_rwys(dep_arr_rwy_filter_[rte2],
                                      dep_arr_proc_filter_[rte2]);
  sort(rwys_[rte2].begin(), rwys_[rte2].end());
}

void CDU::set_fpl_proc(int event, fms_core::ProcType ptp, bool is_arr,
                       bool rte2) {
  util::OpaquePointer<flightplan_type> c_fpl = m_rte1_ptr_;
  if (rte2) {
    c_fpl = m_rte2_ptr_;
  }

  int start_idx = (curr_subpg_ - 1) * 5;
  int tr_idx = -1;
  int sz = procedures_[rte2].size();
  int trans_sz = transitions_[rte2].size();
  int curr_idx = 0;
  std::string curr_proc = c_fpl->get_curr_proc(ptp, false);
  if (curr_proc == "" || sz > 1) {
    curr_idx = start_idx + event - CDU_KEY_LSK_TOP;
  } else {
    if (event - CDU_KEY_LSK_TOP) {
      start_idx = (curr_subpg_ - 1) * 4;
      tr_idx = start_idx + event - CDU_KEY_LSK_TOP - 1;
    }
  }
  if (curr_idx < sz && tr_idx == -1) {
    c_fpl->set_arpt_proc(ptp, procedures_[rte2][size_t(curr_idx)], is_arr);
    dep_arr_rwy_filter_[rte2] = !dep_arr_rwy_filter_[rte2];
  } else if (tr_idx != -1 && tr_idx < trans_sz) {
    c_fpl->set_arpt_proc_trans(ptp, transitions_[rte2][size_t(tr_idx)], is_arr);
    dep_arr_trans_filter_[rte2] = !dep_arr_trans_filter_[rte2];
  }
}

void CDU::get_rte_dep_arr(cdu_pages::cdu_scr_data_t& out, bool rte2) const noexcept {
  size_t v_idx = fms_core::RTE1_IDX;
  if (rte2) v_idx = fms_core::RTE2_IDX;

  util::OpaquePointer<flightplan_type> cr_fpln = fpl_sys_->get_fpln_ptr(v_idx);

  std::string dep = cr_fpln->get_dep_icao();
  std::string arr = cr_fpln->get_arr_icao();

  std::string act_sts = " ";
  if (cntx_.act_fpl_idx == v_idx) act_sts = "(ACT)";
  std::string hdg = "RTE 1";
  if (rte2) hdg = "RTE 2";
  hdg = hdg + act_sts;

  if (dep != "" || arr != "") {
    out.data_lines.push_back(std::string(8, ' ') + hdg);
    if (dep != "")
      out.data_lines.push_back(DEP_ARR_DEP_OPT + dep + DEP_ARR_ARR_OPT);
    else
      out.data_lines.push_back(DEP_ARR_DEP_OPT + "    " + DEP_ARR_ARR_OPT);
    out.data_lines.push_back("");
    if (arr != "")
      out.data_lines.push_back(std::string(DEP_ARR_DEP_OPT.size(), ' ') + arr +
                               DEP_ARR_ARR_OPT);
    else
      out.data_lines.push_back(std::string(DEP_ARR_DEP_OPT.size(), ' ') +
                               "    " + DEP_ARR_ARR_OPT);
  } else {
    out.data_lines.push_back(DEP_ARR_IDX_DASH_L + hdg + DEP_ARR_IDX_DASH_R);
    out.data_lines.push_back("");
    out.data_lines.push_back("");
    out.data_lines.push_back("");
  }
}

bool CDU::arr_has_rwys(std::string& cr_appr, bool rte2) const noexcept {
  if (dep_arr_proc_filter_[rte2] && cr_appr != "") return false;
  return true;
}

int CDU::get_n_sel_des_subpg() const noexcept {
  return int(sel_des_data_.size()) / 6 + bool(int(sel_des_data_.size()) % 6);
}

int CDU::get_n_rte_subpg() const noexcept {
  std::string dep_rwy = fpln_->get_dep_rwy();
  if (dep_rwy != "") {
    size_t n_seg_act = n_seg_list_sz_ - 1;
    return 1 + (n_seg_act / N_CDU_ITM_PP) + bool(n_seg_act % N_CDU_ITM_PP);
  }
  return 1;
}

int CDU::get_n_dep_arr_subpg(bool rte2) noexcept {
  CDUPage c_dep_pg = CDUPage::DEP1;
  CDUPage c_arr_pg = CDUPage::ARR1;
  util::OpaquePointer<flightplan_type> c_fpl = m_rte1_ptr_;
  if (rte2) {
    c_dep_pg = CDUPage::DEP2;
    c_arr_pg = CDUPage::ARR2;
    c_fpl = m_rte2_ptr_;
  }
  std::string dep_icao = c_fpl->get_dep_icao();
  std::string arr_icao = c_fpl->get_arr_icao();
  if ((curr_page_ == c_dep_pg && dep_icao == "") ||
      (curr_page_ == c_arr_pg && arr_icao == "")) {
    curr_page_ = CDUPage::DEP_ARR_INTRO;
  }

  if (curr_page_ == c_dep_pg) {
    set_procs(fms_core::PROC_TYPE_SID, false, rte2);
    size_t max_cnt =
        std::max(rwys_[rte2].size(),
                 procedures_[rte2].size() + transitions_[rte2].size());
    return int(max_cnt) / N_DEP_ARR_ROW_DSP + bool(max_cnt % N_DEP_ARR_ROW_DSP);
  } else if (curr_page_ == c_arr_pg) {
    set_procs(fms_core::PROC_TYPE_STAR, true, rte2);
    size_t max_cnt = std::max(
        procedures_[rte2].size() + transitions_[rte2].size(),
        approaches_[rte2].size() + vias_[rte2].size() + rwys_[rte2].size());
    return int(max_cnt) / N_DEP_ARR_ROW_DSP + bool(max_cnt % N_DEP_ARR_ROW_DSP);
  }

  return 1;
}

std::string CDU::handle_sel_des(int event_key) {
  int i_start = (curr_subpg_ - 1) * 6;
  int i_end = std::min(int(sel_des_data_.size()), i_start + 6) - 1;
  int curr_idx = i_start + event_key - 1;
  if (curr_idx <= i_end) {
    sel_des_idx_ = curr_idx;
    sel_des_ = false;
    std::string tmp;
    curr_subpg_ = sel_des_subpg_;
    on_event(sel_des_event_, "", &tmp);
  }

  return "";
}

std::string CDU::handle_rte(int event_key, std::string scratchpad,
                            std::string* s_out) {
  if (event_key == CDU_KEY_RSK_TOP + 5) {
    bool exec_lt = fpl_sys_->get_exec();
    if (exec_lt) return "";
    if (cntx_.sel_fpl_idx != cntx_.act_fpl_idx) {
      fpl_sys_->rte_activate(cntx_.sel_fpl_idx);
    }
    return "";
  } else if (event_key == CDU_KEY_LSK_TOP + 5) {
    bool exec_lt = fpl_sys_->get_exec();
    if (exec_lt) {
      fpl_sys_->erase();
    } else {
      if (cntx_.sel_fpl_idx == fms_core::RTE1_IDX)
        cntx_.sel_fpl_idx = fms_core::RTE2_IDX;
      else
        cntx_.sel_fpl_idx = fms_core::RTE1_IDX;
    }

    return "";
  }
  if (curr_subpg_ == 1) {
    if (event_key == CDU_KEY_LSK_TOP) {
      return set_departure(scratchpad, s_out);
    } else if (event_key == CDU_KEY_RSK_TOP) {
      return set_arrival(scratchpad, s_out);
    } else if (event_key == CDU_KEY_RSK_TOP + 1) {
      return set_flt_nbr(scratchpad);
    } else if (event_key == CDU_KEY_RSK_TOP + 3) {
      if (rte_copy_ == fms_core::RTECopySts::READY &&
          cntx_.sel_fpl_idx == cntx_.act_fpl_idx)
        fpl_sys_->copy_act();
    } else if (event_key == CDU_KEY_LSK_TOP + 1) {
      return set_dep_rwy(scratchpad);
    } else if (event_key == CDU_KEY_LSK_TOP + 2) {
      return load_rte();
    } else if (event_key == CDU_KEY_LSK_TOP + 4) {
      return save_rte();
    }
  } else {
    size_t i_start = get_seg_start_idx();
    size_t i_end = get_seg_end_idx();
    size_t i_event = i_start + size_t(event_key - 1) % 6;
    if (i_event > i_end || (i_event == i_end && i_event < n_seg_list_sz_ - 1)) {
      return INVALID_ENTRY_MSG;
    } else {
      if (event_key >= CDU_KEY_RSK_TOP) {
        if (!scratchpad_has_delete(scratchpad) && scratchpad.size()) {
          return add_to(i_event + 1, scratchpad);
        } else if(scratchpad.empty()) {
          auto[seg_via, seg_to] = get_segment_endpoint_names(
            seg_list_[i_event]);
          *s_out = seg_to;
          return "";
        }

        return delete_to(i_event);
      } else {
        if (!scratchpad_has_delete(scratchpad))
          return add_via(i_event + 1, scratchpad);
        return delete_via(i_event);
      }
    }
  }

  return "";
}

std::string CDU::handle_dep_arr(int event_key) {
  util::OpaquePointer<flightplan_type> rte1 =
      fpl_sys_->get_fpln_ptr(fms_core::RTE1_IDX);
  util::OpaquePointer<flightplan_type> rte2 = rte1;
  std::string dep1 = rte1->get_dep_icao();
  std::string arr1 = rte1->get_arr_icao();
  std::string dep2 = rte2->get_dep_icao();
  std::string arr2 = rte2->get_arr_icao();

  if (dep1 != "" && arr1 != "") {
    if (event_key == CDU_KEY_LSK_TOP) {
      curr_page_ = CDUPage::DEP1;
    } else if (event_key == CDU_KEY_RSK_TOP + 1) {
      curr_page_ = CDUPage::ARR1;
    }
  }
  if (dep2 != "" && arr2 != "") {
    if (event_key == CDU_KEY_LSK_TOP + 2) {
      curr_page_ = CDUPage::DEP2;
    } else if (event_key == CDU_KEY_RSK_TOP + 3) {
      curr_page_ = CDUPage::ARR2;
    }
  }

  return "";
}

std::string CDU::handle_dep(int event_key, bool rte2) {
  if (event_key == CDU_KEY_LSK_TOP + 5) {
    bool exec_lt = fpl_sys_->get_exec();
    if (exec_lt) {
      fpl_sys_->erase();
    } else {
      set_page(CDUPage::INIT_REF);
    }
  } else if (event_key == CDU_KEY_RSK_TOP + 5) {
    set_page(CDUPage::RTE);
  } else if (event_key && event_key < CDU_KEY_LSK_TOP + 5) {
    set_fpl_proc(event_key, fms_core::PROC_TYPE_SID, false, rte2);
  } else if (event_key >= CDU_KEY_RSK_TOP && event_key < CDU_KEY_RSK_TOP + 5) {
    util::OpaquePointer<flightplan_type> c_fpl = m_rte1_ptr_;
    if (rte2) {
      c_fpl = m_rte2_ptr_;
    }

    int start_idx = (curr_subpg_ - 1) * 5;
    int sz = rwys_[rte2].size();
    int curr_idx = start_idx + event_key - CDU_KEY_RSK_TOP;
    if (curr_idx < sz) {
      c_fpl->set_dep_rwy(rwys_[rte2][size_t(curr_idx)]);
      dep_arr_proc_filter_[rte2] = !dep_arr_proc_filter_[rte2];
    }
  }
  return "";
}

std::string CDU::handle_arr(int event_key, bool rte2) {
  if (event_key == CDU_KEY_LSK_TOP + 5) {
    bool exec_lt = fpl_sys_->get_exec();
    if (exec_lt) {
      fpl_sys_->erase();
    } else {
      set_page(CDUPage::INIT_REF);
    }
  } else if (event_key == CDU_KEY_RSK_TOP + 5) {
    set_page(CDUPage::RTE);
  } else if (event_key && event_key < CDU_KEY_LSK_TOP + 5) {
    set_fpl_proc(event_key, fms_core::PROC_TYPE_STAR, true, rte2);
  } else if (event_key >= CDU_KEY_RSK_TOP && event_key < CDU_KEY_RSK_TOP + 5) {
    int start_idx = (curr_subpg_ - 1) * 5;
    int curr_idx = start_idx + event_key - CDU_KEY_RSK_TOP;

    util::OpaquePointer<flightplan_type> c_fpl = m_rte1_ptr_;
    if (rte2) {
      c_fpl = m_rte2_ptr_;
    }

    if (curr_idx < int(approaches_[rte2].size())) {
      c_fpl->set_arpt_proc(fms_core::PROC_TYPE_APPCH,
                           approaches_[rte2][size_t(curr_idx)], true);
      dep_arr_proc_filter_[rte2] = !dep_arr_proc_filter_[rte2];
    } else if (curr_idx >= int(approaches_[rte2].size())) {
      curr_idx -= int(approaches_[rte2].size());
      std::string curr_appr = c_fpl->get_curr_proc(fms_core::PROC_TYPE_APPCH);
      bool h_rwys = arr_has_rwys(curr_appr, rte2);
      if (curr_idx < int(vias_[rte2].size())) {
        c_fpl->set_arpt_proc_trans(fms_core::PROC_TYPE_APPCH,
                                   vias_[rte2][size_t(curr_idx)], true);
        dep_arr_via_filter_[rte2] = !dep_arr_via_filter_[rte2];
      } else if (curr_idx < int(rwys_[rte2].size()) && h_rwys) {
        c_fpl->set_arr_rwy(rwys_[rte2][size_t(curr_idx)]);
        dep_arr_proc_filter_[rte2] = !dep_arr_proc_filter_[rte2];
      }
    }
  }
  return "";
}

std::size_t CDU::get_seg_start_idx() const noexcept {
  assert(curr_subpg_ >= 2);
  return LEGS_BASE_IDX + N_CDU_ITM_PP * std::size_t(curr_subpg_ - 2);
}

std::size_t CDU::get_seg_end_idx() const noexcept {
  std::size_t stt_idx = get_seg_start_idx();
  return std::min(n_seg_list_sz_ - 1, stt_idx + N_CDU_ITM_PP);
}

cdu_pages::cdu_scr_data_t CDU::get_sel_des_page() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};

  out.heading_small = get_small_heading();
  out.heading_big = SEL_DES_WPT_HDG;
  out.heading_color = CDUColor::WHITE;

  size_t start_idx = size_t((curr_subpg_ - 1) * 6);
  size_t end_idx = std::min(sel_des_data_.size(), start_idx + 6);

  for (size_t i = start_idx; i < end_idx; i++) {
    std::string wpt_tp =
        sel_des_nm_ + " " + libnav::navaid_to_str(sel_des_data_[i].type);
    std::string lat_str =
        strutils::lat_to_str(sel_des_data_[i].pos.lat_rad * geo::RAD_TO_DEG);
    std::string lon_str =
        strutils::lon_to_str(sel_des_data_[i].pos.lon_rad * geo::RAD_TO_DEG);
    std::string main_str = lat_str + lon_str;
    if (sel_des_data_[i].navaid) {
      main_str =
          strutils::freq_to_str(sel_des_data_[i].navaid->freq) + " " + main_str;
    }
    out.data_lines.push_back(wpt_tp);
    out.data_lines.push_back(main_str);
  }

  return out;
}

cdu_pages::cdu_scr_data_t CDU::get_rte_page() const noexcept {
  bool exec_lt = fpl_sys_->get_exec();

  cdu_pages::cdu_scr_data_t out = {};

  out.heading_small = get_small_heading();
  std::string rte_offs = std::string(2, ' ');
  std::string act_sts = std::string(4, ' ');

  out.heading_color = CDUColor::CYAN;
  if (cntx_.sel_fpl_idx == cntx_.act_fpl_idx) {
    if (exec_lt)
      act_sts = MOD + " ";
    else
      act_sts = ACT + " ";
    out.heading_color = CDUColor::WHITE;
  }
  std::string c_rte_top = "RTE 1";
  std::string c_rte_btm = "<RTE 2";
  if (cntx_.sel_fpl_idx == fms_core::RTE2_IDX) {
    c_rte_top = "RTE 2";
    c_rte_btm = "<RTE 1";
  }
  out.heading_big = rte_offs + act_sts + c_rte_top;

  if (curr_subpg_ == 1) {
    std::string dest_offs = std::string(N_CDU_DATA_COLS - 7 - 4, ' ');
    std::string origin_dest = " ORIGIN" + dest_offs + "DEST";
    out.data_lines.push_back(origin_dest);
    std::string origin = fpln_->get_dep_icao();
    std::string dest = fpln_->get_arr_icao();
    bool incomplete = false;

    std::string flt_nbr = fpl_sys_->get_flt_nbr();
    if (flt_nbr == "")
      flt_nbr = std::string(10, '-');
    else
      flt_nbr = std::string(N_FLT_NBR_CHR_MAX - flt_nbr.size(), ' ') + flt_nbr;

    if (origin == "") {
      origin = std::string(4, '@');
      incomplete = true;
    }
    if (dest == "") {
      dest = std::string(4, '@');
      incomplete = true;
    }

    std::string od_data =
        origin + std::string(N_CDU_DATA_COLS - 4 - 4, ' ') + dest;
    out.data_lines.push_back(od_data);
    std::string rwy_flt_no =
        " RUNWAY" + std::string(N_CDU_DATA_COLS - 7 - 6, ' ') + "FLT NO";
    out.data_lines.push_back(rwy_flt_no);
    std::string dep_rwy = "";
    if (!incomplete) {
      dep_rwy = fpln_->get_dep_rwy();
      if (dep_rwy == "")
        dep_rwy = std::string(5, '-');
      else
        dep_rwy = "RW" + dep_rwy;
    }
    std::string rf_data =
        dep_rwy +
        std::string(size_t(N_CDU_DATA_COLS) - dep_rwy.size() - 10, ' ') +
        flt_nbr;
    out.data_lines.push_back(rf_data);
    out.data_lines.push_back(
        " ROUTE" + std::string(N_CDU_DATA_COLS - 6 - 8, ' ') + "CO ROUTE");
    std::string co_rte_nm = fpln_->get_co_rte_nm();
    std::string co_rte_dsp = std::string(10, '-');
    if (co_rte_nm != "" && co_rte_nm.size() <= 10) {
      co_rte_dsp = std::string(10 - co_rte_nm.size(), ' ') + co_rte_nm;
    }
    out.data_lines.push_back(
        "<REQUEST" + std::string(size_t(N_CDU_DATA_COLS) - 8 - 10, ' ') +
        co_rte_dsp);
    if (rte_copy_ == fms_core::RTECopySts::READY &&
        cntx_.sel_fpl_idx == cntx_.act_fpl_idx) {
      out.data_lines.push_back("");
      size_t cp_pad = size_t(N_CDU_DATA_COLS) - RTE_COPY.size() - 1;
      out.data_lines.push_back(std::string(cp_pad, ' ') + RTE_COPY + ">");
    } else if (rte_copy_ == fms_core::RTECopySts::COMPLETE &&
               cntx_.sel_fpl_idx == cntx_.act_fpl_idx) {
      size_t cp_pad1 = size_t(N_CDU_DATA_COLS) - RTE_COPY.size();
      size_t cp_pad2 = size_t(N_CDU_DATA_COLS) - COMPLETE.size();
      out.data_lines.push_back(std::string(cp_pad1, ' ') + RTE_COPY);
      out.data_lines.push_back(std::string(cp_pad2, ' ') + COMPLETE);
    } else {
      out.data_lines.push_back("");
      out.data_lines.push_back("");
    }

    std::string rte_final = " ROUTE ";
    out.data_lines.push_back(rte_final +
                             std::string(size_t(N_CDU_DATA_COLS) - 7, '-'));
    out.data_lines.push_back(
        "<SAVE" + std::string(size_t(N_CDU_DATA_COLS) - 10, ' ') + "ALTN>");
  } else {
    get_seg_page(&out);
  }

  if (out.data_lines.size() == 10) {
    if (curr_subpg_ != 1)
      out.data_lines.push_back(ALL_DASH);
    else
      out.data_lines.push_back("");
  }

  std::string btm_line_nrm = c_rte_btm;
  if (cntx_.act_fpl_idx != cntx_.sel_fpl_idx)
    btm_line_nrm +=
        std::string(size_t(N_CDU_DATA_COLS) - 15, ' ') + "ACTIVATE>";
  if (!exec_lt)
    out.data_lines.push_back(btm_line_nrm);
  else
    out.data_lines.push_back(ERASE_NML);

  return out;
}

cdu_pages::cdu_scr_data_t CDU::get_dep_arr_page() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};

  out.heading_small = get_small_heading();
  std::string hdg_offs =
      std::string((N_CDU_DATA_COLS - DEP_ARR_HDG.size()) / 2, ' ');
  out.heading_big = hdg_offs + DEP_ARR_HDG;
  out.heading_color = CDUColor::WHITE;

  get_rte_dep_arr(out, false);
  get_rte_dep_arr(out, true);

  out.data_lines.push_back(std::string(N_CDU_DATA_COLS, '-'));
  out.data_lines.push_back("");
  out.data_lines.push_back(DEP_ARR_IDX_OTHER);
  out.data_lines.push_back(DEP_ARR_ARROWS);

  return out;
}

void CDU::dep_arr_set_bottom(cdu_pages::cdu_scr_data_t& out) const noexcept {
  out.data_lines[10] = std::string(N_CDU_DATA_COLS, '-');

  bool exec_lt = fpl_sys_->get_exec();
  if (exec_lt) {
    out.data_lines[11] = DEP_ARR_BOTTOM_ACT;
  } else {
    out.data_lines[11] = DEP_ARR_BOTTOM_INACT;
  }
}

cdu_pages::cdu_scr_data_t CDU::get_dep_page(bool rte2) const noexcept {
  util::OpaquePointer<flightplan_type> c_fpl = m_rte1_ptr_;
  if (rte2) {
    c_fpl = m_rte2_ptr_;
  }

  std::string dep = c_fpl->get_dep_icao();
  cdu_pages::cdu_scr_data_t out = {};
  for (size_t i = 9; i < 14; i++) out.chr_sts[0][i] = CDU_B_WHITE;

  out.heading_small = get_small_heading();
  out.heading_big = "   " + dep + " DEPARTURES";
  out.heading_color = CDUColor::WHITE;

  for (int i = 0; i < 12; i++) {
    out.data_lines.push_back("");
  }
  if (rte2)
    out.data_lines[0] = DEP_COLS2;
  else
    out.data_lines[0] = DEP_COLS1;

  std::string curr_sid = c_fpl->get_curr_proc(fms_core::PROC_TYPE_SID);
  std::string curr_trans = c_fpl->get_curr_proc(fms_core::PROC_TYPE_SID, true);
  std::string act_sid = m_act_ptr_->get_curr_proc(fms_core::PROC_TYPE_SID);
  std::string act_trans =
      m_act_ptr_->get_curr_proc(fms_core::PROC_TYPE_SID, true);
  get_procs(&out, curr_sid, curr_trans, act_sid, act_trans, rte2);

  std::string dep_rwy = c_fpl->get_dep_rwy();
  std::string act_rwy = m_act_ptr_->get_dep_rwy();
  get_rwys(&out, dep_rwy, act_rwy, rte2);

  dep_arr_set_bottom(out);

  return out;
}

cdu_pages::cdu_scr_data_t CDU::get_arr_page(bool rte2) const noexcept {
  util::OpaquePointer<flightplan_type> c_fpl = m_rte1_ptr_;
  if (rte2) {
    c_fpl = m_rte2_ptr_;
  }

  std::string arr = c_fpl->get_arr_icao();
  cdu_pages::cdu_scr_data_t out = {};
  for (size_t i = 9; i < 14; i++) out.chr_sts[0][i] = CDU_B_WHITE;

  out.heading_small = get_small_heading();
  out.heading_big = "   " + arr + " ARRIVALS";
  out.heading_color = CDUColor::WHITE;

  for (int i = 0; i < 12; i++) {
    out.data_lines.push_back("");
  }
  if (rte2)
    out.data_lines[0] = ARR_COLS2;
  else
    out.data_lines[0] = ARR_COLS1;

  std::string curr_star = c_fpl->get_curr_proc(fms_core::PROC_TYPE_STAR);
  std::string curr_trans = c_fpl->get_curr_proc(fms_core::PROC_TYPE_STAR, true);
  std::string act_star = m_act_ptr_->get_curr_proc(fms_core::PROC_TYPE_STAR);
  std::string act_trans =
      m_act_ptr_->get_curr_proc(fms_core::PROC_TYPE_STAR, true);
  get_procs(&out, curr_star, curr_trans, act_star, act_trans, rte2);

  std::string arr_rwy = c_fpl->get_arr_rwy();
  std::string arr_appr = c_fpl->get_curr_proc(fms_core::PROC_TYPE_APPCH);
  std::string arr_via = c_fpl->get_curr_proc(fms_core::PROC_TYPE_APPCH, true);

  std::string act_rwy = m_act_ptr_->get_arr_rwy();
  std::string act_appr = m_act_ptr_->get_curr_proc(fms_core::PROC_TYPE_APPCH);
  std::string act_via =
      m_act_ptr_->get_curr_proc(fms_core::PROC_TYPE_APPCH, true);

  get_rwys(&out, arr_rwy, act_rwy, rte2, arr_appr, arr_via, act_appr, act_via,
           true);

  dep_arr_set_bottom(out);

  return out;
}

// CDUDisplay definitions:
// Public member functions:

std::optional<CDUDisplay::event_type> CDUDisplay::get_event_from_str(
  const std::string& str) {
  for(std::size_t i = 0; i < MY_ARRAY_SIZE(CDU_EVENTS); ++i) {
    if(CDU_EVENTS[i].name == str) {
      return CDU_EVENTS[i].event;
    }
  }
  return std::nullopt;
}

CDUDisplay::CDUDisplay(geom::vect2_t pos, geom::vect2_t sz,
                       util::OpaquePointer<TextureManager> tm,
                       util::OpaquePointer<CDU> cdu, bool is_free) {
  if (!is_free) {
    display_pos_ = pos + sz * DISPLAY_OFFS;
    display_size_ = sz * DISPLAY_SZ;
    scale_coeff_ = sz.y * CDU_RES_COEFF;
  } else {
    display_pos_ = pos;
    display_size_ = sz;
    scale_coeff_ = sz.y / DISPLAY_SZ.y * CDU_RES_COEFF;
  }

  textures_.init(tm);
  cdu_ptr_ = cdu;

  scratchpad_ = std::string(size_t(N_CDU_DATA_COLS), ' ');
  scratch_curr_ = 0;

  last_press_tp_ = std::chrono::steady_clock::now();
}

void Swap(CDUDisplay& d_a, CDUDisplay& d_b) {
  std::swap(d_a.events_, d_b.events_);
  std::swap(d_a.display_pos_, d_b.display_pos_);
  std::swap(d_a.display_size_, d_b.display_size_);
  std::swap(d_a.scale_coeff_, d_b.scale_coeff_);
  std::swap(d_a.cdu_ptr_, d_b.cdu_ptr_);
  std::swap(d_a.scratchpad_, d_b.scratchpad_);
  std::swap(d_a.last_press_tp_, d_b.last_press_tp_);
  std::swap(d_a.msg_stack_, d_b.msg_stack_);
  std::swap(d_a.textures_, d_b.textures_);
}

CDUDisplay::CDUDisplay(CDUDisplay&& other) {
  Swap(*this, other);
}

std::pair<double, double> CDUDisplay::GetDrawSize() const noexcept {
  return {display_size_.x, display_size_.y};
}

void CDUDisplay::on_event(event_type event) {
  std::unique_lock lk(main_mutex_);
  events_.push(event);
}

void CDUDisplay::draw(cairo_t* cr) {
  std::unique_lock lk(main_mutex_);
  process_events(CNT_CDU_EVENTS_PER_FRAME);
  draw_screen(cr);
}

// Private member functions:

void CDUDisplay::cdu_textures_t::init(util::OpaquePointer<TextureManager> tm) {
  cdu_big_white = tm->GetTexture(CDU_WHITE_TEXT_NAME);
  assert(cdu_big_white != nullptr);
  cdu_big_green = tm->GetTexture(CDU_GREEN_TEXT_NAME);
  assert(cdu_big_green != nullptr);
  cdu_big_cyan = tm->GetTexture(CDU_CYAN_TEXT_NAME);
  assert(cdu_big_cyan != nullptr);
  cdu_big_magenta = tm->GetTexture(CDU_MAGENTA_TEXT_NAME);
  assert(cdu_big_magenta != nullptr);

  main_font_face =
      tm->GetFontData(fms_display_fonts::MAIN_FONT_NAME)->cairo_face;
}

void CDUDisplay::handle_event(event_type event) noexcept {
  if (event && (event < CDU_KEY_A || event == CDU_KEY_EXEC)) {
    std::string scratch_proc = strutils::strip(scratchpad_);
    std::string scr_out;
    std::string msg = cdu_ptr_->on_event(event, scratch_proc, &scr_out);

    if (scr_out != "") {
      scratch_curr_ = scr_out.size();
      scratchpad_ =
          scr_out + std::string(N_CDU_DATA_COLS - scr_out.size(), ' ');
    } else {
      if (msg == "")
        clear_scratchpad();
      else
        msg_stack_.push(msg);
    }
  }
  update_scratchpad(event);
}

void CDUDisplay::process_events(std::size_t cnt_max) noexcept {
  while (cnt_max && events_.size()) {
    event_type curr = events_.front();
    handle_event(curr);
    events_.pop();
  }
}

void CDUDisplay::add_to_scratchpad(char c) {
  if (scratch_curr_ != size_t(N_CDU_DATA_COLS)) {
    scratchpad_[scratch_curr_] = c;
    scratch_curr_++;
  }
}

void CDUDisplay::clear_scratchpad() {
  while (scratch_curr_) {
    scratchpad_[scratch_curr_] = ' ';
    scratch_curr_--;
  }
  scratchpad_[scratch_curr_] = ' ';
}

void CDUDisplay::update_scratchpad(event_type event) {
  if (event == CDU_KEY_DELETE) {
    if (scratchpad_[0] == DELETE_SYMBOL) {
      scratchpad_[0] = ' ';
    } else {
      clear_scratchpad();
      scratchpad_[0] = DELETE_SYMBOL;
    }
  } else if (event == CDU_KEY_CLR) {
    if (msg_stack_.size()) {
      msg_stack_.pop();
    } else {
      if (scratch_curr_) scratch_curr_--;
      scratchpad_[scratch_curr_] = ' ';
    }
  }

  if (scratchpad_[0] != DELETE_SYMBOL) {
    if (event >= CDU_KEY_A && event < CDU_KEY_A + 26) {
      add_to_scratchpad('A' + char(event - CDU_KEY_A));
    } else if (event == CDU_KEY_SP) {
      add_to_scratchpad(' ');
    } else if (event == CDU_KEY_SLASH) {
      add_to_scratchpad('/');
    } else if (event >= CDU_KEY_1 && event < CDU_KEY_1 + 9) {
      add_to_scratchpad('1' + char(event - CDU_KEY_1));
    } else if (event == CDU_KEY_DOT) {
      add_to_scratchpad('.');
    } else if (event == CDU_KEY_0) {
      add_to_scratchpad('0');
    } else if (event == CDU_KEY_PM) {
      if (scratch_curr_ && scratchpad_[scratch_curr_ - 1] == '-')
        scratchpad_[scratch_curr_ - 1] = '+';
      else
        add_to_scratchpad('-');
    }
  }
}

int CDUDisplay::get_cdu_letter_idx(char c) {
  if (c >= '0' && c <= '9')
    return 1 + c - '0';
  else if (c >= 'A' && c <= 'Z')
    return 11 + c - 'A';
  else if (c == '%')
    return 37;
  else if (c == '(')
    return 38;
  else if (c == ')')
    return 39;
  else if (c == '-')
    return 40;
  else if (c == '_')
    return 41;
  else if (c == '+')
    return 42;
  else if (c == '=')
    return 43;
  else if (c == '|')
    return 44;
  else if (c == ':')
    return 45;
  else if (c == '<')
    return 46;
  else if (c == '.')
    return 47;
  else if (c == '>')
    return 48;
  else if (c == ',')
    return 49;
  else if (c == '/')
    return 50;
  else if (c == strutils::DEGREE_SYMBOL)
    return 51;
  else if (c == '@')
    return 52;
  return 0;
}

CDUColor CDUDisplay::get_cdu_color(char c) {
  if (c == CDU_S_WHITE || c == CDU_B_WHITE)
    return CDUColor::WHITE;
  else if (c == CDU_S_CYAN || c == CDU_B_CYAN)
    return CDUColor::CYAN;
  else if (c == CDU_S_GREEN || c == CDU_B_GREEN)
    return CDUColor::GREEN;

  return CDUColor::MAGENTA;
}

bool CDUDisplay::chr_is_big(char c) {
  if (c >= 'a' && c <= 'z') return false;
  return true;
}

CDUDisplay::texture_type CDUDisplay::get_font_sfc(CDUColor cl) {
  texture_type font_sfc;
  if (cl == CDUColor::GREEN)
    font_sfc = textures_.cdu_big_green;
  else if (cl == CDUColor::CYAN)
    font_sfc = textures_.cdu_big_cyan;
  else if (cl == CDUColor::MAGENTA)
    font_sfc = textures_.cdu_big_magenta;
  else
    font_sfc = textures_.cdu_big_white;

  return font_sfc;
}

void CDUDisplay::draw_cdu_letter(cairo_t* cr, char c, geom::vect2_t pos,
                                 geom::vect2_t scale, texture_type font_sfc) {
  if (scale.x == 0 || scale.y == 0) return;

  int idx = get_cdu_letter_idx(c);
  geom::vect2_t offs = {-CDU_LETTER_WIDTH * scale.x * double(idx), 0};
  geom::vect2_t img_pos = pos + offs;
  cairo_save(cr);
  cairo_scale(cr, scale.x, scale.y);
  cairo_set_source_surface(cr, font_sfc, img_pos.x / scale.x,
                           img_pos.y / scale.y);
  cairo_rectangle(cr, pos.x / scale.x, pos.y / scale.y, CDU_LETTER_WIDTH,
                  CDU_LETTER_HEIGHT);
  cairo_clip(cr);
  cairo_paint(cr);
  cairo_restore(cr);
}

void CDUDisplay::draw_cdu_line(cairo_t* cr, const std::string& s,
                               geom::vect2_t pos, double l_intv_px,
                               std::string sts, geom::vect2_t scale,
                               CDUColor clr) {
  if (sts != "") assert(sts.size() >= s.size());

  cairo_surface_t* sfc_const = get_font_sfc(clr);
  geom::vect2_t sc_big = CDU_BIG_TEXT_SZ;
  geom::vect2_t sc_sml = CDU_SMALL_TEXT_SZ;

  scale = scale.scmul(scale_coeff_);
  sc_big = sc_big.scmul(scale_coeff_);
  sc_sml = sc_sml.scmul(scale_coeff_);

  for (size_t i = 0; i < s.size(); i++) {
    cairo_surface_t* sfc = sfc_const;
    geom::vect2_t sc_cr = scale;
    if (sts != "") {
      sfc = get_font_sfc(get_cdu_color(sts[i]));
      if (chr_is_big(sts[i]))
        sc_cr = sc_big;
      else
        sc_cr = sc_sml;
    }
    draw_cdu_letter(cr, s[i], pos, sc_cr, sfc);
    pos.x += l_intv_px;
  }
}

void CDUDisplay::draw_screen(cairo_t* cr) {
  geom::vect2_t offs_hdg_small = {0, display_size_.y * CDU_V_OFFS_SMALL_FIRST};
  geom::vect2_t pos_hdg_small = display_pos_ + offs_hdg_small;
  geom::vect2_t small_offs = {0, display_size_.y * CDU_V_OFFS_FIRST_SM};
  geom::vect2_t pos_small = display_pos_ + small_offs;
  geom::vect2_t big_offs = {
      0, display_size_.y * (CDU_BIG_TEXT_OFFS + CDU_V_OFFS_FIRST_BIG)};
  geom::vect2_t pos_big = display_pos_ + big_offs;

  cdu_pages::cdu_scr_data_t curr_screen = cdu_ptr_->get_screen_data();

  // Clear the screen:
  cairo_utils::draw_rect(cr, display_pos_, display_size_, cairo_utils::BLACK);

  draw_cdu_line(cr, curr_screen.heading_big, display_pos_,
                CDU_TEXT_INTV * display_size_.x, "", CDU_BIG_TEXT_SZ,
                curr_screen.heading_color);

  draw_cdu_line(cr, curr_screen.heading_small, pos_hdg_small,
                CDU_TEXT_INTV * display_size_.x, "", CDU_SMALL_TEXT_SZ);

  size_t j = 0;
  for (size_t i = 0; i < size_t(N_CDU_DATA_LINES); i++) {
    if (j < curr_screen.data_lines.size()) {
      draw_cdu_line(cr, curr_screen.data_lines[j], pos_small,
                    CDU_TEXT_INTV * display_size_.x, curr_screen.chr_sts[j]);
    }
    if (j + 1 < curr_screen.data_lines.size()) {
      draw_cdu_line(cr, curr_screen.data_lines[j + 1], pos_big,
                    CDU_TEXT_INTV * display_size_.x,
                    curr_screen.chr_sts[j + 1]);
    }

    pos_small.y += CDU_V_OFFS_REG * display_size_.y;
    pos_big.y += CDU_V_OFFS_REG * display_size_.y;
    j += 2;
  }

  std::string tgt_scratch;
  if (msg_stack_.size()) {
    tgt_scratch = msg_stack_.top();
  } else if (scratchpad_[0] == DELETE_SYMBOL) {
    tgt_scratch = DELETE_MSG;
  } else {
    tgt_scratch = scratchpad_;
  }
  draw_cdu_line(cr, tgt_scratch, pos_small, CDU_TEXT_INTV * display_size_.x, "",
                CDU_BIG_TEXT_SZ);
}
}  // namespace fms_displays
