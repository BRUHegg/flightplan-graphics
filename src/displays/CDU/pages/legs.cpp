#include "legs.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <displays/CDU/common.hpp>
#include <fpln/fpln_sys.hpp>
#include <libnav/arpt_db.hpp>
#include <util/util.hpp>

#include "base.hpp"
#include "sel_des.hpp"

namespace {

constexpr std::size_t LEGS_BASE_IDX = 2;

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
// Constraint designations used on LEGS page
constexpr char LEGS_CSTR_ABV = 'A';
constexpr char LEGS_CSTR_BLW = 'B';
constexpr char LEGS_CSTR_SEP = '/';

constexpr std::size_t N_LEG_PROP_ROWS = 14;
constexpr std::size_t N_LEG_CRS_ROWS = 5;
constexpr std::size_t N_LEG_CSTR_ROWS = 11;
constexpr std::size_t N_LEG_VCSTR_ROWS = 6;

constexpr std::size_t N_LEG_SPDCSTR_MX_LN = 4;
constexpr std::size_t N_LEG_SPDCSTR_MN_LN = 3;
constexpr std::size_t N_LEG_ALTCSTR_MX_LN = N_LEG_VCSTR_ROWS;
constexpr std::size_t N_LEG_ALTCSTR_MN_LN = 2;

constexpr int SPDCSTR_MX_KT = 340;
constexpr int SPDCSTR_MN_KT = 100;
constexpr int ALTCSTR_MX_FT = 41000;
constexpr int ALTCSTR_MN_FT = 100;

// Misc:
const std::string ACT = "ACT";
const std::string SEL = "SEL";
const std::string MOD = "MOD";
const std::string DISCO_AFTER_SEG = "-- ROUTE DISCONTINUITY -";
} // namespace

namespace cdu_pages {

std::string Legs::get_cdu_leg_nm(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) const noexcept {
  if (src.data.is_discon) {
    return DISCO_LEG_NM;
  }

  if (src.data.leg.leg_type == "FM" || src.data.leg.leg_type == "VM") {
    return LEG_VECTORS;
  }
  if (src.data.misc_data.has_calc_wpt) {
    return src.data.misc_data.calc_wpt.id;
  }
  return "";
}

std::size_t Legs::get_leg_start_idx() const noexcept {
  return LEGS_BASE_IDX + fms_displays::N_CDU_ITM_PP * std::size_t(curr_subpg_ - 1);
}

std::size_t Legs::get_leg_end_idx() const noexcept {
  std::size_t stt_idx = get_leg_start_idx();
  return std::min(leg_list_.size() - 1, stt_idx + fms_displays::N_CDU_ITM_PP);
}

void Legs::reset_leg_dto_sel(std::size_t fp_idx) noexcept {
  assert(fp_idx < fms_displays::N_CDU_RTES);
  leg_sel_[fp_idx].second = -1;
}

void Legs::reset_leg_all_sel() noexcept {
  for (size_t i = 0; i < fms_displays::N_CDU_RTES; i++) {
    reset_leg_dto_sel(i);
  }
}

void Legs::handle_legs_map_ctr_advance() noexcept {
  fpl_sys_->step_ctr(false, cdu_cntx_->side_index);
  fms_core::fpln_info_t sel_data = fpl_sys_->get_fpl_info(cdu_cntx_->sel_fpl_idx);
  std::size_t map_ctr_idx = 0U;
  if(sel_data.map_ctr_idx[cdu_cntx_->side_index] < LEGS_BASE_IDX) {
    return;
  }
  map_ctr_idx = sel_data.map_ctr_idx[cdu_cntx_->side_index] - LEGS_BASE_IDX;
  std::size_t tgt_page = 1U + map_ctr_idx / fms_displays::N_CDU_ITM_PP;
  if((unsigned)tgt_page > cnt_subpg_ || (unsigned)tgt_page <= 0U) {
    curr_subpg_ = 1U;
  } else {
    curr_subpg_ = tgt_page;
  }
}

bool Legs::handle_legs_dto(size_t usr_idx, std::string scratchpad,
                          std::string& s_out) noexcept {
  size_t sd_idx = cdu_cntx_->sel_fpl_idx - 1;
  if (leg_sel_[sd_idx].second == -1) {
    bool is_discon = leg_list_[usr_idx].data.is_discon;
    if (scratchpad == "" && is_discon) return 0;
    if (scratchpad != "")  // User might be trying to insert a waypoint.
      return 1;
    leg_sel_[sd_idx].first = usr_idx;
    leg_sel_[sd_idx].second = fpl_infos_[cdu_cntx_->sel_fpl_idx].leg_list_id;
    if (leg_list_[usr_idx].data.is_discon == false)
      s_out = get_cdu_leg_nm(leg_list_[usr_idx]);
  } else {
    size_t i_fr = usr_idx;
    size_t i_to = leg_sel_[sd_idx].first;
    if (get_cdu_leg_nm(leg_list_[i_to]) != scratchpad) {
      reset_leg_dto_sel(sd_idx);
      return 1;
    }
    if (i_fr != i_to) {
      if (i_fr > i_to) {
        if (i_fr + 1 < leg_list_.size()) i_fr++;
        std::swap(i_fr, i_to);
      } else if (i_fr) {
        i_fr--;
      }

      double cr_lg_id = leg_sel_[sd_idx].second;
      bool rv = fpln_->dir_from_to({leg_list_[i_fr].ptr, cr_lg_id},
                                   {leg_list_[i_to].ptr, cr_lg_id});
      reset_leg_dto_sel(sd_idx);
      UNUSED(rv);
    }
  }
  return 0;
}

cdu_event_res_t Legs::handle_legs_delete(std::size_t usr_idx) noexcept {
  cdu_event_res_t res{.err = fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::LEGS};
  fms_core::fpln_info_t f_inf = fpl_infos_[cdu_cntx_->sel_fpl_idx];
  double lg_id = f_inf.leg_list_id;

  bool retval = fpln_->delete_leg({leg_list_[usr_idx].ptr, lg_id});
  if (!retval) {
    res.err = fms_displays::CDUError::INVALID_DELETE;
  }
  return res;
}

cdu_event_res_t Legs::handle_legs_insert(std::size_t usr_idx, 
  const std::string& scratchpad) noexcept {
  fms_core::fpln_info_t f_inf = fpl_infos_[cdu_cntx_->sel_fpl_idx];
  double lg_id = f_inf.leg_list_id;

  cdu_cntx_->select_desired.set_state(fms_displays::CDUPage::LEGS, 
    scratchpad, lg_id, -1.0);
  auto res_waypoint = cdu_cntx_->select_desired.get_result();
  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::LEGS};
  if(!res_waypoint) {
    return res;
  }
  if(res_waypoint->err == cdu_pages::SelectDesired::Error::INVALID_ENTRY) {
    res.err = fms_displays::CDUError::INVALID_ENTRY;
    return res;
  } else if(res_waypoint->err == 
    cdu_pages::SelectDesired::Error::NOT_IN_DATA_BASE) {
    res.err = fms_displays::CDUError::NOT_IN_DATABASE;
    return res;
  }
  assert(res_waypoint->err == cdu_pages::SelectDesired::Error::NONE);
  fpln_->add_direct(res_waypoint->waypoint, {leg_list_[usr_idx].ptr, 
    res_waypoint->leg_id_});
  return res;
}

fms_core::spd_cstr_t Legs::get_spd_cstr(const std::string& str) noexcept {
  fms_core::spd_cstr_t out = {-1, libnav::SpeedMode::AT};
  if (str.size() > N_LEG_SPDCSTR_MX_LN || str.size() < N_LEG_SPDCSTR_MN_LN)
    return out;
  if (str.size() == N_LEG_SPDCSTR_MX_LN) {
    if (str.back() == LEGS_CSTR_ABV)
      out.mode = libnav::SpeedMode::AT_OR_ABOVE;
    else if (str.back() == LEGS_CSTR_BLW)
      out.mode = libnav::SpeedMode::AT_OR_BELOW;
    else
      return out;
  }
  int cnum = 0;
  int pw = 1;
  assert(N_LEG_SPDCSTR_MX_LN >= 2);
  for (int i = int(N_LEG_SPDCSTR_MX_LN) - 2; i >= 0; i--) {
    if (str[i] > '9' || str[i] < '0') return out;
    cnum += pw * int(str[i] - '0');
    pw *= 10;
  }
  if (cnum > SPDCSTR_MX_KT || cnum < SPDCSTR_MN_KT) return out;
  out.magnitude = cnum;
  return out;
}

fms_core::alt_cstr_t Legs::get_alt_cstr(const std::string& str) noexcept {
  fms_core::alt_cstr_t out = {-1, libnav::AltMode::AT};
  if (str.size() > N_LEG_ALTCSTR_MX_LN || str.size() < N_LEG_ALTCSTR_MN_LN)
    return out;

  if (str.back() == LEGS_CSTR_ABV)
    out.mode = libnav::AltMode::AT_OR_ABOVE;
  else if (str.back() == LEGS_CSTR_BLW)
    out.mode = libnav::AltMode::AT_OR_BELOW;
  else if (str.back() > '9' || str.back() < '0')
    return out;

  int st_add = 0;
  if (str[0] == 'F' && str[1] == 'L') st_add = 2;
  int has_lt = out.mode != libnav::AltMode::AT;
  int cnum = 0;
  int pw = 1;
  int n_dgts = int(str.size()) - st_add - has_lt;
  for (int i = n_dgts + st_add - 1; i >= st_add; i--) {
    if (str[i] > '9' || str[i] < '0') return out;
    cnum += pw * int(str[i] - '0');
    pw *= 10;
  }
  if (cnum < 1000) {
    if (cnum >= 10 || (n_dgts >= 2 && n_dgts <= 3)) cnum *= 100;
  }
  if (cnum > ALTCSTR_MX_FT || cnum < ALTCSTR_MN_FT) return out;
  out.magnitude = cnum;
  return out;
}

cdu_event_res_t Legs::handle_legs_cstr_mod(std::size_t usr_idx, 
  const std::string& scratchpad) noexcept {
  bool scr_is_del = scratchpad_has_delete(scratchpad);
  fms_core::fpln_info_t f_inf = fpl_infos_[cdu_cntx_->sel_fpl_idx];
  double lg_id = f_inf.leg_list_id;

  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::LEGS};
  if (scr_is_del) {
    fpln_->set_spd_cstr({leg_list_[usr_idx].ptr, lg_id},
                        {0, libnav::SpeedMode::AT});
    fpln_->set_alt_cstr({leg_list_[usr_idx].ptr, lg_id},
                        {0, libnav::AltMode::AT});
    return res;
  }

  std::vector<std::string> cst = {"", ""};
  size_t idx = 0;
  for (auto i : scratchpad) {
    if (idx == 0 && i == LEGS_CSTR_SEP) {
      idx = 1;
      continue;
    }
    cst[idx].push_back(i);
  }

  if (cst[0] != "") {
    fms_core::spd_cstr_t spc = get_spd_cstr(cst[0]);
    if (spc.magnitude == -1) {
      res.err = fms_displays::CDUError::INVALID_ENTRY;
      return res;
    }
    fpln_->set_spd_cstr({leg_list_[usr_idx].ptr, lg_id}, spc);
  }
  if (cst[1] != "") {
    fms_core::alt_cstr_t alc = get_alt_cstr(cst[1]);
    if (alc.magnitude == -1) { 
      res.err = fms_displays::CDUError::INVALID_ENTRY;
      return res;
    }
    fpln_->set_alt_cstr({leg_list_[usr_idx].ptr, lg_id}, alc);
  }
  return res;
}

void Legs::update_fpl_infos() noexcept {
  for (size_t i = 0; i < MY_ARRAY_SIZE(fpl_infos_); ++i) {
    fpl_infos_[i] = fpl_sys_->get_fpl_info(i);
  }
}

std::string Legs::get_cdu_leg_prop(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) noexcept {
  if (src.data.is_discon) {
    return DISCO_THEN;
  }
  if (src.data.leg.leg_type[0] == 'H') {
    return HOLD_DESC;
  }
  double crs_deg = src.data.misc_data.true_trk_deg + 360.0;
  if (crs_deg > 360) crs_deg -= 360;
  std::string deg_str = std::string(1, strutils::DEGREE_SYMBOL);
  std::string crs_st = strutils::double_to_str(crs_deg, 0) + deg_str;
  assert(crs_st.size() <= N_LEG_CRS_ROWS);
  size_t crs_offs = N_LEG_CRS_ROWS - crs_st.size();
  crs_st = std::string(crs_offs, ' ') + crs_st;

  std::string out = "";
  bool is_bp = src.data.misc_data.is_bypassed;
  if (is_bp) {
    size_t offs = std::size_t(fms_displays::N_CDU_DATA_COLS) - 
      crs_st.size() - LEG_BYPASS.size();
    out = crs_st + std::string(offs, ' ') + LEG_BYPASS;
  } else {
    double dist_nm = src.data.leg.outbd_dist_time;
    std::string dist_st = strutils::double_to_str(dist_nm, 0) + NAUT_MILES;
    assert(dist_st.size() + crs_st.size() <= N_LEG_PROP_ROWS);
    size_t offs = N_LEG_PROP_ROWS - dist_st.size() - crs_st.size();
    out = crs_st + std::string(offs, ' ') + dist_st;
  }

  return out;
}

std::string Legs::get_cdu_leg_spdcstr(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) noexcept {
  if (src.data.leg.spd_lim_kias == 0) return LEG_NO_SPD;

  std::string spd =
      strutils::double_to_str(double(src.data.leg.spd_lim_kias), 0);

  if (src.data.leg.speed_desc == libnav::SpeedMode::AT_OR_ABOVE)
    return spd + "A";
  if (src.data.leg.speed_desc == libnav::SpeedMode::AT_OR_BELOW)
    return spd + "B";

  return spd;
}

std::string Legs::get_leg_alt(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src, bool alt2,
    bool fl) noexcept {
  int tgt_alt = src.data.leg.alt1_ft;
  if (alt2) tgt_alt = src.data.leg.alt2_ft;
  int tr_alt = src.data.leg.trans_alt;
  bool fl_act = tgt_alt > tr_alt;
  if (fl_act || fl) tgt_alt /= 100;
  std::string alt_str = strutils::double_to_str(double(tgt_alt), 0);
  if ((fl_act || fl) && alt_str.size() < 3)
    alt_str = std::string(3 - alt_str.size(), '0') + alt_str;
  if (fl_act && !fl) alt_str = "FL" + alt_str;
  return alt_str;
}

std::string Legs::get_cdu_leg_vcstr(
    const fms_core::list_node_ref_t<fms_core::leg_list_data_t>& src) noexcept {
  if (src.data.leg.alt1_ft == 0 && src.data.leg.alt2_ft == 0) {
    return LEG_NO_ALT;
  }

  if (src.data.leg.alt_desc == libnav::AltMode::AT_OR_ABOVE ||
      src.data.leg.alt_desc == libnav::AltMode::GS_AT_OR_ABOVE) {
    return get_leg_alt(src) + "A";
  }
  if (src.data.leg.alt_desc == libnav::AltMode::SID_AT_OR_ABOVE ||
      src.data.leg.alt_desc == libnav::AltMode::ALT_STEPDOWN_AT_AT_OR_ABOVE) {
    return get_leg_alt(src, true) + "A";
  }

  if (src.data.leg.alt_desc == libnav::AltMode::AT ||
      src.data.leg.alt_desc == libnav::AltMode::GS_AT) {
    return get_leg_alt(src);
  }
  if (src.data.leg.alt_desc == libnav::AltMode::GS_INTC_AT ||
      src.data.leg.alt_desc == libnav::AltMode::ALT_STEPDOWN_AT_AT) {
    return get_leg_alt(src, true);
  }

  if (src.data.leg.alt_desc == libnav::AltMode::AT_OR_BELOW) {
    return get_leg_alt(src) + "B";
  }
  if (src.data.leg.alt_desc == libnav::AltMode::ALT_STEPDOWN_AT_AT_OR_BELOW) {
    return get_leg_alt(src, true) + "B";
  }
  return get_leg_alt(src, false, true) + "A" + get_leg_alt(src, true, true) +
         "B";
}

std::string Legs::get_legs_bottom() const noexcept {
  bool is_act = cdu_cntx_->sel_fpl_idx == cdu_cntx_->act_fpl_idx;
  bool exec_lt = fpl_sys_->get_exec();

  std::string c_legs_btm = "<RTE 2";
  if (cdu_cntx_->sel_fpl_idx == fms_core::RTE2_IDX) {
    c_legs_btm = "<RTE 1";
  }

  if(nd_mode_ == fms_core::NDMode::PLAN) {
    return c_legs_btm + std::string{LEGS_BTM_PLN};
  }

  if (!is_act && !exec_lt) {
    return c_legs_btm + LEGS_BTM_INACT;
  } else if (is_act && exec_lt) {
    return LEGS_BTM_MOD;
  }
  return c_legs_btm + LEGS_BTM_ACT;
}

Legs::Legs(util::OpaquePointer<fms_core::FPLSys> fpl_sys,
  util::OpaquePointer<fms_displays::cdu_context_t> cntx) : fpl_sys_{fpl_sys},
  cdu_cntx_{cntx} {
  nd_mode_ = fpl_sys_->get_nd_mode(cdu_cntx_->side_index);
  fpln_ = fpl_sys_->get_fpln_ptr(cdu_cntx_->sel_fpl_idx);
  std::fill(fpl_infos_, fpl_infos_ + MY_ARRAY_SIZE(fpl_infos_), 
    fms_core::fpln_info_t{});
  std::fill(leg_sel_, leg_sel_ + MY_ARRAY_SIZE(leg_sel_), 
    std::pair<std::size_t, double>{0LL, -1.0});
  std::fill(pln_ctr_idx_, pln_ctr_idx_ + MY_ARRAY_SIZE(leg_sel_), 0);
  std::fill(pln_ctr_pos_, pln_ctr_pos_ + MY_ARRAY_SIZE(pln_ctr_pos_), 
    geo::point{0, 0});
}

fms_displays::CDUPage Legs::get_page_number() const noexcept {
  return fms_displays::CDUPage::LEGS;
}

void Legs::update() noexcept {
  fpln_ = fpl_sys_->get_fpln_ptr(cdu_cntx_->sel_fpl_idx);
  nd_mode_ = fpl_sys_->get_nd_mode(cdu_cntx_->side_index);
  leg_list_ = fpl_sys_->get_leg_list(&n_leg_list_sz_, cdu_cntx_->sel_fpl_idx);

  update_fpl_infos();

  std::size_t n_leg_act = n_leg_list_sz_ - 2;
  cnt_subpg_ = std::max(1U, (unsigned)((n_leg_act / fms_displays::N_CDU_ITM_PP) + 
    bool(n_leg_act % fms_displays::N_CDU_ITM_PP)));
}

cdu_event_res_t Legs::on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept {
  cdu_event_res_t res{.err=fms_displays::CDUError::NONE, 
    .page=fms_displays::CDUPage::LEGS};
  if (event == fms_displays::CDU_KEY_LSK_TOP + 5) {
    bool exec_lt = fpl_sys_->get_exec();
    if (exec_lt) {
      fpl_sys_->erase();
    } else {
      if (cdu_cntx_->sel_fpl_idx == fms_core::RTE1_IDX) {
        cdu_cntx_->sel_fpl_idx = fms_core::RTE2_IDX;
      }
      else {
        cdu_cntx_->sel_fpl_idx = fms_core::RTE1_IDX;
      }
    }
  } else if (event == fms_displays::CDU_KEY_RSK_TOP + 5) {
    if(nd_mode_ == fms_core::NDMode::PLAN) {
      handle_legs_map_ctr_advance();
      return res;
    }
    bool exec_lt = fpl_sys_->get_exec();
    if (exec_lt) { return res; }
    if (cdu_cntx_->sel_fpl_idx != cdu_cntx_->act_fpl_idx) {
      fpl_sys_->rte_activate(cdu_cntx_->sel_fpl_idx);
    }
    return res;
  } else if (event >= fms_displays::CDU_KEY_LSK_TOP && 
    event <= fms_displays::CDU_KEY_LSK_TOP + 5) {
    std::size_t i_start = get_leg_start_idx();
    std::size_t i_end = get_leg_end_idx();
    std::size_t usr_idx = i_start + std::size_t(
      event - fms_displays::CDU_KEY_LSK_TOP);
    bool scr_is_del = scratchpad_has_delete(scratchpad);
    bool is_ins = 0;
    if ((!scr_is_del && usr_idx == i_end) || leg_sel_pr_) is_ins = 1;
    if (!is_ins) {
      if (usr_idx < i_end && !scr_is_del) {
        is_ins = handle_legs_dto(usr_idx, scratchpad, s_out);
      } else if (scr_is_del) {
        if (usr_idx < i_end) {
          return handle_legs_delete(usr_idx);
        } else {
          res.err = fms_displays::CDUError::INVALID_DELETE;
          return res;
        }
      }
    }
    if (is_ins) {
      return handle_legs_insert(usr_idx, scratchpad);
    }
  } else if (event >= fms_displays::CDU_KEY_RSK_TOP && 
    event <= fms_displays::CDU_KEY_RSK_TOP + 5) {
    std::size_t usr_idx = get_leg_start_idx() + 
      std::size_t(event - fms_displays::CDU_KEY_RSK_TOP);
    std::size_t i_end = get_leg_end_idx();
    if (usr_idx >= i_end) {
      if (scratchpad != "") {
        if (scratchpad_has_delete(scratchpad)) {
          res.err = fms_displays::CDUError::INVALID_DELETE;
        } else {
          res.err = fms_displays::CDUError::INVALID_ENTRY;
        }
        return res;
      }
      return res;
    }
    return handle_legs_cstr_mod(usr_idx, scratchpad);
  }
  return res;
}

void Legs::on_page_change(fms_displays::CDUPage page) noexcept {
  if(page == fms_displays::CDUPage::NEXT_PAGE) {
    curr_subpg_++;
  } else if(page == fms_displays::CDUPage::PREV_PAGE) {
    curr_subpg_--;
  } else {
    curr_subpg_ = 1;
    if(page == fms_displays::CDUPage::LEGS) {
      fpl_sys_->reset_ctr(cdu_cntx_->side_index);
    } else {
      reset_leg_all_sel();
    }
  }
  if(curr_subpg_ == 0U) {
    curr_subpg_ = cnt_subpg_;
  } else if(curr_subpg_ > cnt_subpg_) {
    curr_subpg_ = 1U;
  }
}

cdu_scr_data_t Legs::get_screen_data() const noexcept {
  cdu_pages::cdu_scr_data_t out = {};

  out.heading_small = get_small_heading(curr_subpg_, cnt_subpg_);
  out.heading_color = fms_displays::CDUColor::WHITE;

  out.heading_color = fms_displays::CDUColor::CYAN;
  bool exec_lt = fpl_sys_->get_exec();
  std::string act_sts = std::string(4, ' ');
  if (cdu_cntx_->sel_fpl_idx == cdu_cntx_->act_fpl_idx) {
    if (exec_lt)
      act_sts = MOD + " ";
    else
      act_sts = ACT + " ";
    out.heading_color = fms_displays::CDUColor::WHITE;
  }
  std::string c_legs_top = "RTE 1 LEGS";
  if (cdu_cntx_->sel_fpl_idx == fms_core::RTE2_IDX) {
    c_legs_top = "RTE 2 LEGS";
  }
  out.heading_big = "  " + act_sts + c_legs_top;

  assert(leg_list_.size());
  size_t i_start = get_leg_start_idx();
  size_t i_end = get_leg_end_idx();
  bool disc_pr = false;
  size_t sts_idx = 1;

  fms_core::act_leg_info_t act_info = fpl_sys_->get_act_leg_info();
  std::size_t map_ctr_idx = fpl_infos_[cdu_cntx_->sel_fpl_idx
    ].map_ctr_idx[cdu_cntx_->side_index];

  for (size_t i = i_start; i < i_end; i++) {
    if (!disc_pr) out.data_lines.push_back(get_cdu_leg_prop(leg_list_[i]));
    if (leg_list_[i].data.is_discon) {
      disc_pr = true;
      out.data_lines.push_back("@@@@@");
      out.data_lines.push_back(DISCO_AFTER_SEG);
    } else {
      disc_pr = false;

      std::string cr_name = get_cdu_leg_nm(leg_list_[i]);
      if (cr_name == act_info.name) {
        for (size_t j = 0; j < 5; j++) {
          out.chr_sts[sts_idx][j] = fms_displays::CDU_B_MAGENTA;
        }
      }
      // get leg constraints
      std::string spdcstr = get_cdu_leg_spdcstr(leg_list_[i]);
      std::string vcstr = get_cdu_leg_vcstr(leg_list_[i]);
      if (vcstr.size() < N_LEG_VCSTR_ROWS)
        vcstr = std::string(N_LEG_VCSTR_ROWS - vcstr.size(), ' ') + vcstr;
      std::string cstr = spdcstr + "/" + vcstr;
      
      if(i == map_ctr_idx && nd_mode_ == fms_core::NDMode::PLAN) {
        cstr = std::string{LEG_MAP_CTR} + cstr;
      }

      cr_name =
          cr_name +
          std::string(fms_displays::N_CDU_DATA_COLS - cr_name.size() - 
          cstr.size(), ' ') + cstr;
      out.data_lines.push_back(cr_name);
    }
    sts_idx += 2;
  }

  if (i_end - i_start < fms_displays::N_CDU_ITM_PP) {
    out.data_lines.push_back("");
    out.data_lines.push_back(LEG_LAST);
  }

  while (out.data_lines.size() < 10) {
    out.data_lines.push_back("");
  }

  while (out.data_lines.size() > 10) {
    out.data_lines.pop_back();
  }

  if(nd_mode_ == fms_core::NDMode::PLAN) {
    out.data_lines.push_back(std::string{LEGS_MAP_CTR_BORD});
  } else {
    out.data_lines.push_back(std::string(fms_displays::N_CDU_DATA_COLS, '-'));
  }
  out.data_lines.push_back(get_legs_bottom());

  return out;
}
} // namespace cdu_pages
