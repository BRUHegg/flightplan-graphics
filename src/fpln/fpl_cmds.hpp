/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        Author: discord/bruh4096#4512

        This file contains definitions of commands used to interface the
   flightplan
*/

#pragma once

#include <libnav/str_utils.hpp>

#include "fpln_sys.hpp"

#define UNUSED(x) (void)(x)

namespace {

bool glob_rwy_filter = false;
bool glob_proc_filter = false;
bool glob_trans_filter = false;
}  // namespace

namespace test {
// Command definitions:

typedef void (*cmd)(FPLSys*, std::vector<std::string>&);

inline libnav::waypoint_entry_t select_desired(
    std::string& name, std::vector<libnav::waypoint_entry_t>& wpts) {
  if (wpts.size() == 0) {
    return {};
  }
  if (wpts.size() == 1) {
    return wpts[0];
  }
  std::cout << "Select desired " << name << "\n";
  for (size_t i = 0; i < wpts.size(); i++) {
    std::cout << i + 1 << ". "
              << strutils::lat_to_str(wpts[i].pos.lat_rad * geo::RAD_TO_DEG)
              << " "
              << strutils::lat_to_str(wpts[i].pos.lon_rad * geo::RAD_TO_DEG)
              << "\n";
  }
  while (1) {
    std::string tmp;
    std::getline(std::cin, tmp);

    size_t num = size_t(strutils::stoi_with_strip(tmp));
    if (num != 0 && num < wpts.size() + 1) {
      return wpts[num - 1];
    }
  }
}

inline void set_var(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 2) {
    std::cout << "Command expects 2 arguments: <variable name>, <value>\n";
    return;
  }

  fpl_sys->set_env_var(in[0], in[1]);
}

inline void print(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: <variable name>\n";
    return;
  }

  auto val = fpl_sys->get_env_var(in[0]);
  if (val) {
    std::cout << *val << "\n";
  } else {
    std::cout << "Variable not found\n";
  }
}

inline void quit(FPLSys* fpl_sys, std::vector<std::string>& in) {
  UNUSED(fpl_sys);

  if (in.size()) {
    std::cout << "Too many arguments provided\n";
    return;
  }
  std::exit(0);
}

inline void load_fpln(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size()) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();

  std::shared_ptr<FplnInt> curr_fpln = fpl_sys->get_fpln_ptr(c_idx);
  std::string dep_nm = curr_fpln->get_dep_icao();
  std::string arr_nm = curr_fpln->get_arr_icao();

  if (dep_nm != "" && arr_nm != "") {
    std::string file_nm = fpl_sys->get_fpln_dir() + dep_nm + arr_nm;
    libnav::DbErr err = curr_fpln->load_from_fms(file_nm, false);

    if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
      std::cout << "Failed to load flight plan\n";
    }
  }
}

inline void save_fpln(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size()) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();

  std::shared_ptr<FplnInt> curr_fpln = fpl_sys->get_fpln_ptr(c_idx);
  std::string dep_nm = curr_fpln->get_dep_icao();
  std::string arr_nm = curr_fpln->get_arr_icao();

  if (dep_nm != "" && arr_nm != "") {
    std::string out_nm = fpl_sys->get_fpln_dir() + dep_nm + arr_nm;
    curr_fpln->save_to_fms(out_nm);
  }
}

inline void set_filter(FPLSys* fpl_sys, std::vector<std::string>& in) {
  UNUSED(fpl_sys);
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: {filter type(0 - runway, 1 - "
                 "procedure, 2 - transition)}\n";
    return;
  }

  int flt_type = strutils::stoi_with_strip(in[0]);
  if (flt_type == 0) {
    glob_rwy_filter = !(glob_rwy_filter);
  } else if (flt_type == 1) {
    glob_rwy_filter = !(glob_rwy_filter);
  } else if (flt_type == 2) {
    glob_trans_filter = !(glob_trans_filter);
  } else {
    std::cout << "Filter type out of range\n";
  }
}

inline void fplinfo(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size()) {
    std::cout << "Too many arguments provided\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpln = fpl_sys->get_fpln_ptr(c_idx);

  std::cout << "Departure: " << curr_fpln->get_dep_icao() << "\n";
  std::cout << "Arrival: " << curr_fpln->get_arr_icao() << "\n";
  std::cout << "Departure runway: " << curr_fpln->get_dep_rwy() << "\n";
  std::cout << "Arrival runway: " << curr_fpln->get_arr_rwy() << "\n";
}

inline void set_fpl_dep(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: icao code\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpln = fpl_sys->get_fpln_ptr(c_idx);

  libnav::DbErr err = curr_fpln->set_dep(in[0]);
  if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
    std::cout << "Invalid entry\n";
  } else if (err == libnav::DbErr::PARTIAL_LOAD) {
    std::cout << "Airport partially loaded\n";
  }
}

inline void set_fpl_arr(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: icao code\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  libnav::DbErr err = curr_fpl->set_arr(in[0]);
  if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD) {
    std::cout << "Invalid entry\n";
  } else if (err == libnav::DbErr::PARTIAL_LOAD) {
    std::cout << "Airport partially loaded\n";
  }
}

inline void set_dep_rwy(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: icao code\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  bool rwy_set = curr_fpl->set_dep_rwy(in[0]);

  if (!rwy_set) {
    std::cout << "Runway not set";
  }
}

inline void set_arr_rwy(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: icao code\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  bool rwy_set = curr_fpl->set_arr_rwy(in[0]);

  if (!rwy_set) {
    std::cout << "Runway not set";
  }
}

inline void get_dep_rwys(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 0) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  std::vector<std::string> rwys =
      curr_fpl->get_dep_rwys(glob_rwy_filter, glob_proc_filter);
  for (auto i : rwys) {
    std::cout << i << "\n";
  }
}

inline void get_arr_rwys(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 0) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  std::vector<std::string> rwys = curr_fpl->get_arr_rwys();
  for (auto i : rwys) {
    std::cout << i << "\n";
  }
}

inline void get_proc(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 3) {
    std::cout << "Command expects 3 arguments: {procedure type}, {DEP/ARR}, \
            {PROC/TRANS}\n";
    return;
  }

  int tmp = strutils::stoi_with_strip(in[0]);

  if (tmp < 0 || tmp > 2) {
    std::cout << "procedure type entry out of range\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  bool is_arr = in[1] != "DEP";
  bool is_trans = in[2] == "TRANS";

  if ((in[1] == "ARR" || in[1] == "DEP") &&
      (in[2] == "TRANS" || in[2] == "PROC")) {
    std::vector<std::string> procs;
    if (!is_trans) {
      procs = curr_fpl->get_arpt_proc(ProcType(tmp), is_arr, glob_rwy_filter,
                                      glob_proc_filter);
    } else {
      procs = curr_fpl->get_arpt_proc_trans(ProcType(tmp), false, is_arr);
    }

    for (auto i : procs) {
      std::cout << i << "\n";
    }
  }
}

inline void set_proc(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 4) {
    std::cout << "Command expects 4 arguments: {procedure type}, {proc name}, \
                {DEP/ARR}, {TRANS/PROC}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  int tmp = strutils::stoi_with_strip(in[0]);

  if (tmp < 0 || tmp > 2) {
    std::cout << "procedure type entry out of range\n";
    return;
  }

  bool is_arr = in[2] != "DEP";
  bool is_trans = in[3] != "PROC";
  bool ret = false;
  if (is_trans) {
    ret = curr_fpl->set_arpt_proc_trans(ProcType(tmp), in[1], is_arr);
  } else {
    ret = curr_fpl->set_arpt_proc(ProcType(tmp), in[1], is_arr);
  }

  if (!ret) {
    std::cout << "Failed to set procedure/trantition\n";
  }
}

inline void print_legs(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: layout{1/2}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();

  bool show_dist_trk = in[0] == "2";

  if (in[0] != "1" && in[0] != "2") return;

  size_t n_legs;
  auto legs = fpl_sys->get_leg_list(&n_legs, c_idx);

  size_t cnt = 0;
  for (auto i : legs) {
    if (cnt && cnt < size_t(n_legs - 1)) {
      std::cout << cnt - 1 << ". ";
      if (i.data.is_discon) {
        std::cout << "DISCONTINUITY\n";
        cnt++;
        continue;
      }
      double lat_deg = i.data.leg.main_fix.data.pos.lat_rad * geo::RAD_TO_DEG;
      double lon_deg = i.data.leg.main_fix.data.pos.lon_rad * geo::RAD_TO_DEG;
      if (i.data.leg.leg_type != "IF" && show_dist_trk) {
        if (!i.data.misc_data.is_bypassed) {
          float brng_deg = i.data.misc_data.true_trk_deg;
          float dist_nm = i.data.leg.outbd_dist_time;
          std::string brng_str = strutils::double_to_str(double(brng_deg), 6);
          std::string dist_str = strutils::double_to_str(double(dist_nm), 6);
          std::cout << brng_str << " " << dist_str << "\n";
        } else {
          std::cout << "--- ---\n";
        }
      }
      std::string pos = "";
      if (!show_dist_trk)
        pos = strutils::double_to_str(lat_deg, 6) + " " +
              strutils::double_to_str(lon_deg, 6);
      std::string misc_data =
          i.data.misc_data.calc_wpt.id + " " + i.data.leg.leg_type;

      std::cout << misc_data + " " + pos << "\n";
    }
    cnt++;
  }
}

inline void add_via(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 2) {
    std::cout
        << "Command expects 2 arguments: {Next segment index}, {Airway name}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  fpln_info_t f_inf = fpl_sys->get_fpl_info(c_idx);
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  size_t idx = size_t(strutils::stoi_with_strip(in[0]));
  size_t n_segs;
  auto segs = fpl_sys->get_seg_list(&n_segs, c_idx);

  seg_list_node_t* s_ptr = nullptr;
  if (idx < n_segs) {
    s_ptr = segs[idx].ptr;
  }
  double id = f_inf.seg_list_id;
  bool retval = curr_fpl->add_enrt_seg({s_ptr, id}, in[1]);

  if (!retval) {
    std::cout << "Invalid entry\n";
  }
}

inline void delete_via(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: {Next segment index}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  fpln_info_t f_inf = fpl_sys->get_fpl_info(c_idx);
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  size_t idx = size_t(strutils::stoi_with_strip(in[0]));
  size_t n_segs;
  auto segs = fpl_sys->get_seg_list(&n_segs, c_idx);

  seg_list_node_t* s_ptr = nullptr;
  if (idx < n_segs) {
    s_ptr = segs[idx].ptr;
  }
  double id = f_inf.seg_list_id;
  bool retval = curr_fpl->delete_via({s_ptr, id});

  if (!retval) {
    std::cout << "INVALID DELETE\n";
  }
}

inline void add_to(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 2) {
    std::cout << "Command expects 2 arguments: {Next segment index}, {End "
                 "waypoint name}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  fpln_info_t f_inf = fpl_sys->get_fpl_info(c_idx);
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  std::vector<libnav::waypoint_entry_t> wpt_entr;
  size_t n_found = fpl_sys->get_navaid_db_ptr()->get_wpt_data(in[1], &wpt_entr);

  libnav::waypoint_entry_t tgt;

  if (n_found == 0) {
    std::cout << "Invalid waypoint id\n";
  } else {
    tgt = select_desired(in[1], wpt_entr);
  }

  size_t idx = size_t(strutils::stoi_with_strip(in[0]));
  size_t n_segs;
  auto segs = fpl_sys->get_seg_list(&n_segs, c_idx);

  seg_list_node_t* s_ptr = nullptr;
  if (idx < n_segs) {
    s_ptr = segs[idx].ptr;
  }
  libnav::waypoint_t tgt_wpt = {in[1], tgt};
  double id = f_inf.seg_list_id;
  bool retval = curr_fpl->awy_insert_str({s_ptr, id}, tgt_wpt.get_awy_id());

  if (!retval) {
    std::cout << "Invalid entry\n";
  }
}

inline void delete_to(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: {Next segment index}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  fpln_info_t f_inf = fpl_sys->get_fpl_info(c_idx);
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  size_t idx = size_t(strutils::stoi_with_strip(in[0]));
  size_t n_segs;
  auto segs = fpl_sys->get_seg_list(&n_segs, c_idx);

  seg_list_node_t* s_ptr = nullptr;
  if (idx < segs.size()) {
    s_ptr = segs[idx].ptr;
  }
  double id = f_inf.seg_list_id;
  bool retval = curr_fpl->delete_seg_end({s_ptr, id});

  if (!retval) {
    std::cout << "INVALID DELETE\n";
  }
}

inline void legs_set(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 3 && in.size() != 4) {
    std::cout << "Command expects 3 arguments: {index}, {L/R CDU}, {L/R "
                 "Field}, (optional){Scratch pad content. If not empty}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  fpln_info_t f_inf = fpl_sys->get_fpl_info(c_idx);
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);

  if (in[2] == "R") return;

  std::size_t idx = std::size_t(strutils::stoi_with_strip(in[0])) + 1;
  std::size_t n_legs;
  auto legs = fpl_sys->get_leg_list(&n_legs, c_idx);

  if (idx >= n_legs) {
    std::cout << "Index out of range\n";
    return;
  }

  bool is_rt = false;
  if (in[1] == "R") {
    is_rt = true;
  } else if (in[1] != "L") {
    std::cout << "Invalid second parameter\n";
    return;
  }
  std::pair<std::size_t, double> sel_leg = fpl_sys->get_sel_leg(is_rt);

  if (f_inf.leg_list_id != sel_leg.second) {
    if (in.size() == 4) {
      std::vector<libnav::waypoint_entry_t> wpt_entr;
      size_t n_found =
          fpl_sys->get_navaid_db_ptr()->get_wpt_data(in[3], &wpt_entr);

      libnav::waypoint_entry_t tgt;

      if (n_found == 0) {
        std::cout << "Invalid waypoint id\n";
      } else {
        tgt = select_desired(in[3], wpt_entr);
      }

      curr_fpl->add_direct({in[3], tgt}, {legs[idx].ptr, f_inf.leg_list_id});
    } else if (idx < n_legs - 1) {
      leg_list_node_t* leg_ptr = legs[idx].ptr;
      if (!leg_ptr->data.is_discon) {
        sel_leg.first = idx;
        sel_leg.second = f_inf.leg_list_id;
        fpl_sys->set_sel_leg(sel_leg, is_rt);
      }
    }
  } else if (idx < n_legs - 1) {
    size_t from = idx;
    size_t to = sel_leg.first;
    if (to != from) {
      if (from > to) {
        if (from + 1 < n_legs) from++;
        std::swap(from, to);
      } else if (from) {
        from--;
      }

      curr_fpl->dir_from_to({legs[from].ptr, sel_leg.second},
                            {legs[to].ptr, sel_leg.second});
      sel_leg.second = -1;
      fpl_sys->set_sel_leg(sel_leg, is_rt);
    }
  }
}

inline void delete_leg(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 1) {
    std::cout << "Command expects 1 argument: {leg number}\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);
  fpln_info_t f_inf = fpl_sys->get_fpl_info(c_idx);

  size_t idx = size_t(strutils::stoi_with_strip(in[0])) + 1;
  size_t n_legs;
  auto legs = fpl_sys->get_leg_list(&n_legs, c_idx);

  if (idx >= n_legs - 1) {
    std::cout << "Index out of range\n";
    return;
  }

  bool ret = curr_fpl->delete_leg({legs[idx].ptr, f_inf.leg_list_id});

  if (!ret) {
    std::cout << "INVALID DELETE\n";
  }
}

inline void print_seg(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 0) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();

  size_t n_segs;
  auto segs = fpl_sys->get_seg_list(&n_segs, c_idx);

  for (size_t i = 0; i < n_segs; i++) {
    auto curr_sg = segs[i];
    leg_list_node_t* end_leg = curr_sg.data.end;
    std::string end_nm = "";
    if (end_leg != nullptr) {
      end_nm = end_leg->data.leg.main_fix.id;
    }
    std::cout << curr_sg.data.name << " " << end_nm << " "
              << curr_sg.data.seg_type << "\n";
  }
}

inline void print_refs(FPLSys* fpl_sys, std::vector<std::string>& in) {
  if (in.size() != 0) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  size_t c_idx = fpl_sys->get_cmd_fpl_idx();
  std::shared_ptr<FplnInt> curr_fpl = fpl_sys->get_fpln_ptr(c_idx);
  curr_fpl->print_refs();
}

inline void help(FPLSys* fpl_sys, std::vector<std::string>& in);

std::unordered_map<std::string, cmd> cmd_map = {{"set", set_var},
                                                {"print", print},
                                                {"p", print},
                                                {"quit", quit},
                                                {"q", quit},
                                                {"load", load_fpln},
                                                {"save", save_fpln},
                                                {"setfilt", set_filter},
                                                {"fplinfo", fplinfo},
                                                {"setdep", set_fpl_dep},
                                                {"setarr", set_fpl_arr},
                                                {"setdeprwy", set_dep_rwy},
                                                {"setarrrwy", set_arr_rwy},
                                                {"getdeprwys", get_dep_rwys},
                                                {"getarrrwys", get_arr_rwys},
                                                {"getproc", get_proc},
                                                {"setproc", set_proc},
                                                {"addvia", add_via},
                                                {"deletevia", delete_via},
                                                {"addto", add_to},
                                                {"deleteto", delete_to},
                                                {"legset", legs_set},
                                                {"deleteleg", delete_leg},
                                                {"plegs", print_legs},
                                                {"pseg", print_seg},
                                                {"prefs", print_refs},
                                                {"help", help}};

inline void help(FPLSys* fpl_sys, std::vector<std::string>& in) {
  (void)fpl_sys;

  if (in.size() != 0) {
    std::cout << "Command expects 0 arguments\n";
    return;
  }

  for (auto i : cmd_map) {
    std::cout << i.first << "\n";
  }
}
}  // namespace test
