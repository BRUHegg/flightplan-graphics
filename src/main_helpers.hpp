/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        This header file contains helper functions for the app itself.
    Author: discord/bruh4096#4512(Tim G.)
*/

#pragma once

#include <fstream>
#include <iostream>
#include <libnav/common.hpp>
#include <libnav/geo_utils.hpp>
#include <libnav/str_utils.hpp>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include <fpln/environment.hpp>
#include <displays/CDU/cdu.hpp>
#include <displays/CDU/cdu_widget.hpp>
#include <displays/common/font_names.hpp>
#include <displays/common/texture_manager.hpp>
#include <displays/ND/nd.hpp>
#include <fpln/fpl_cmds.hpp>
#include <fpln/fpln_sys.hpp>
#include <util/json_require.hpp>
#include <util/pathlib.hpp>
#include <util/util.hpp>

namespace fms_core {

const std::string CMD_FILE_NM = "cmds.txt";
const std::string PREFS_FILE_NM = "prefs.txt";

const std::string PREFS_EARTH_PATH = "EPATH";
const std::string PREFS_APT_DIR = "APTDIR";
const std::string PREFS_FPL_DIR = "FPLDIR";

const std::string BOEING_FONT_NAME = "BoeingFont.ttf";
const std::pair<std::string, std::string> CDU_BYTEMAP_NAME = {
    "cdu_key_map", "cdu"};
const std::string TEXTURES_PATH = "textures/";

constexpr double WND_HEIGHT = 900;
constexpr double CDU_WIDTH =
    (fms_displays::CDU_TEXTURE_ASPECT_RATIO * WND_HEIGHT);
constexpr double ND_WIDTH = WND_HEIGHT;
constexpr double WND_WIDTH = CDU_WIDTH + ND_WIDTH;
constexpr geom::vect2_t ND_POS = {CDU_WIDTH, 0};
constexpr geom::vect2_t ND_SZ = {500, 500};
constexpr geom::vect2_t CDU_L_POS = {0, 0};
constexpr geom::vect2_t CDU_L_SZ = {CDU_WIDTH, WND_HEIGHT};

class Avionics {
 public:
  libnav::ArptDB* arpt_db_ptr;
  libnav::NavaidDB* navaid_db_ptr;

  libnav::AwyDB* awy_db;
  libnav::HoldDB* hold_db;

  FPLSys* fpl_sys;
  fms_environment::EnvDataRefMap* env_map_ptr_;

  pathlib::Path cifp_dir_path;

  Avionics(pathlib::Path apt_dat, pathlib::Path custom_apt, pathlib::Path custom_rnw,
           pathlib::Path fix_data, pathlib::Path navaid_data, pathlib::Path awy_data,
           pathlib::Path hold_data, pathlib::Path cifp_path, pathlib::Path fpl_path) {
    cifp_dir_path = cifp_path;

    arpt_db_ptr =
        new libnav::ArptDB{apt_dat.Get(), custom_apt.Get(), custom_rnw.Get()};
    navaid_db_ptr = new libnav::NavaidDB{fix_data.Get(), navaid_data.Get()};
    awy_db = new libnav::AwyDB{awy_data.Get()};
    hold_db = new libnav::HoldDB{hold_data.Get()};

    libnav::DbErr err_arpt = arpt_db_ptr->get_err();
    libnav::DbErr err_wpt = navaid_db_ptr->get_wpt_err();
    libnav::DbErr err_nav = navaid_db_ptr->get_navaid_err();
    libnav::DbErr err_awy = awy_db->get_err();
    libnav::DbErr err_hold = hold_db->get_err();

    std::cout << navaid_db_ptr->get_wpt_cycle() << " "
              << navaid_db_ptr->get_navaid_cycle() << " " << awy_db->get_airac()
              << " " << hold_db->get_airac() << "\n";

    std::cout << "Fix data base version: " << navaid_db_ptr->get_wpt_version()
              << "\n";
    std::cout << "Navaid data base version: "
              << navaid_db_ptr->get_navaid_version() << "\n";

    if (err_arpt != libnav::DbErr::SUCCESS) {
      std::cout << "Unable to load airport database\n";
    }
    if (err_wpt != libnav::DbErr::SUCCESS) {
      std::cout << "Unable to load waypoint database\n";
    }
    if (err_nav != libnav::DbErr::SUCCESS) {
      std::cout << "Unable to load navaid database\n";
    }
    if (err_awy != libnav::DbErr::SUCCESS) {
      std::cout << "Unable to load airway database\n";
    }
    if (err_hold != libnav::DbErr::SUCCESS) {
      std::cout << "Unable to load hold database\n";
    }

    env_map_ptr_ = 
      new fms_environment::EnvDataRefMap{fms_environment::kBaseVariables};

    fpl_sys = new FPLSys{util::OpaquePointer<libnav::ArptDB>{arpt_db_ptr}, 
                        util::OpaquePointer<libnav::NavaidDB>{navaid_db_ptr}, 
                        util::OpaquePointer<libnav::AwyDB>{awy_db}, 
                        util::OpaquePointer{env_map_ptr_}, 
                        cifp_dir_path, fpl_path};
  }

  void update() { fpl_sys->update(); }

  ~Avionics() {
    delete fpl_sys;
    delete env_map_ptr_;
    delete hold_db;
    delete awy_db;
    delete navaid_db_ptr;
    delete arpt_db_ptr;
  }
};

class CMDInterface {
  fms_displays::TextureManager* tex_manager_;
 public:
  std::shared_ptr<Avionics> avncs;
  fms_displays::NDData* nd_data;
  fms_displays::NDDisplay* nd_display;
  fms_displays::CDU* cdu_l;
  fms_displays::CDUDisplay* cdu_display_l;
  fms_displays::CDUWidget* cdu_widget_l;

  byteutils::bytemap_manager_t byte_mngr;

  CMDInterface() {
    FT_Init_FreeType(&lib);
    pre_exec = {};

    pathlib::Path config_path = pathlib::Path{} + "config.json";
    if(!json_data_.init(config_path, GetJsonConfigTree())) {
      throw std::logic_error{"There was a problem loading config.json"};
    }
    tex_manager_ = new fms_displays::TextureManager{
      pathlib::Path{}, json_data_.tex_names, &lib};
    if(!check_texture_manager(tex_manager_, fms_displays::GetCduTextureNames())) {
      throw std::logic_error{"Not all cdu textures loaded"};
    }
    if(!check_texture_manager(tex_manager_, fms_displays::GetCduWidgetTextureNames())) {
      throw std::logic_error{"Not all cdu widget textures loaded"};
    }
    if(!check_texture_manager(tex_manager_, fms_displays::GetNdTextureNames())) {
      throw std::logic_error{"Not all nd textures loaded"};
    }
    util::const_str_data_t font_names{
      .ptr=fms_display_fonts::FONT_NAMES, .size=MY_ARRAY_SIZE(fms_display_fonts::FONT_NAMES)};
    if(!check_texture_manager(tex_manager_, font_names)) {
      throw std::logic_error{"Not all fonts loaded"};
    }
    fetch_prefs_data();
    get_paths_from_user();
    get_pre_exec_cmds();
    create_avionics();
    pre_execute_cmds();
  }

  void set_nd_mode(fms_core::NDMode md, std::size_t side_idx) {
    avncs->env_map_ptr_->Set<std::int64_t>(fms_environment::ND_MODE, 
      static_cast<std::int64_t>(md), side_idx);
  }

  void increment_nd_range(std::size_t side_idx) {
    auto curr_idx = (avncs->env_map_ptr_->Get<std::int64_t>(
      fms_environment::ND_RANGE_IDX, side_idx));
    if(curr_idx) {
      std::size_t next = static_cast<std::size_t>(*curr_idx);
      if(next + 1 < fms_displays::ND_RANGES_NM.size()) {
        next++;
      }
      avncs->env_map_ptr_->Set<std::int64_t>(fms_environment::ND_RANGE_IDX, 
        static_cast<std::int64_t>(next), side_idx);
    }
  }

  void decrement_nd_range(std::size_t side_idx) {
    auto curr_idx = (avncs->env_map_ptr_->Get<std::int64_t>(
      fms_environment::ND_RANGE_IDX, side_idx));
    if(curr_idx) {
      std::size_t next = static_cast<std::size_t>(*curr_idx);
      if(next > 0) {
        next--;
      }
      avncs->env_map_ptr_->Set<std::int64_t>(fms_environment::ND_RANGE_IDX, 
        static_cast<std::int64_t>(next), side_idx);
    }
  }

  void switch_trk_hdg_up() {
    auto val = avncs->env_map_ptr_->Get<bool>(fms_environment::ND_IS_TRACK_UP);
    if(val) {
      bool tgt = !(*val);
      avncs->env_map_ptr_->Set<bool>(fms_environment::ND_IS_TRACK_UP, tgt);
    }
  }

  void toggle_bool_dr(const char* dr_name, std::size_t side_idx) {
    auto val = avncs->env_map_ptr_->Get<bool>(dr_name, side_idx);
    if(val) {
      bool tgt = !(*val);
      avncs->env_map_ptr_->Set<bool>(dr_name, tgt, side_idx);
    }
  }

  void execute_cmd(std::string in_raw) {
    std::string in_proc = strutils::strip(in_raw, ' ');

    std::vector<std::string> line_split = strutils::str_split(in_proc, ' ');
    if (line_split.size()) {
      std::string cmd_name = line_split[0];
      std::vector<std::string> args =
          std::vector<std::string>(line_split.begin() + 1, line_split.end());

      fms_commands::command_res_t cmd_resources{
        .fpl_sys=avncs->fpl_sys, .env_map=avncs->env_map_ptr_};
      if(!fms_commands::invoke(cmd_name, cmd_resources, args)) {
        std::cout << "Invalid command name\n";
      }
    }
  }

  void on_click(geom::vect2_t pos) { cdu_widget_l->on_click(pos); }

  void draw(cairo_t* cr) {
    nd_display->draw(cr);
    cdu_widget_l->draw(cr);
  }

  void update() {
    cdu_l->update();
    nd_data->update();
    avncs->update();
  }

  void main_loop() {
    while (true) {
      std::string in_raw;
      std::cout << ">> ";
      std::getline(std::cin, in_raw);

      execute_cmd(in_raw);

      update();
    }
  }

 private:
  struct json_data_t {
    nlohmann::json tex_names;

    bool init(const pathlib::Path& path, 
      const json_require::RequirementTree& config_tree) noexcept {
      std::ifstream file(path.Get(), std::ios::binary);

      if (!file) {
        return false;
      }

      try {
        nlohmann::json js = nlohmann::json::parse(file);
        if(!config_tree.Verify(js)) {
          return false;
        }
        tex_names = js["textures"];
      } catch (const nlohmann::json::parse_error& e) {
        return false;
      }
      return true;
    }
  };

  json_data_t json_data_;
  pathlib::Path earth_nav_path;
  pathlib::Path apt_dat_dir;
  pathlib::Path fpl_dir;

  std::vector<std::string> pre_exec;

  FT_Library lib;
  FT_Face font;
  cairo_font_face_t* boeing_font_face;

  static json_require::RequirementTree GetJsonConfigTree() {
    json_require::RequirementTree tr;
    auto p2 = tr.Add("textures", nlohmann::json::value_t::array);
    p2.SetPredicate(fms_displays::TextureManager::CheckMainFont, true);
    auto p3 = tr.Add(p2, "name", nlohmann::json::value_t::string);
    auto p6 = tr.Add(p3, "file_name", nlohmann::json::value_t::string, false, true);
    auto p7 = tr.Add(p6, "type", nlohmann::json::value_t::string, false, true);
    p7.SetPredicate(fms_displays::TextureManager::CheckTextureType, true);
    return tr;
  }

  static bool check_texture_manager(
    fms_displays::TextureManager* manager, 
    util::const_str_data_t tex_names) {
    for(std::size_t i = 0; i < tex_names.size; ++i) {
      if(manager->GetTexture(tex_names.ptr[i]) == nullptr && 
        !(manager->GetFontData(tex_names.ptr[i]))) {
        return false;
      }
    }
    return true;
  }

  void fetch_prefs_data() {
    if (libnav::does_file_exist(PREFS_FILE_NM)) {
      std::ifstream file(PREFS_FILE_NM);

      std::string line;
      while (getline(file, line)) {
        line = strutils::strip(line);
        if (line.size() && line[0] != '#') {
          std::vector<std::string> str_split =
              strutils::str_split(line, ' ', 1);

          if (str_split.size() == 2) {
            if (str_split[0] == PREFS_EARTH_PATH)
              earth_nav_path = pathlib::Path{str_split[1]};
            else if (str_split[0] == PREFS_APT_DIR)
              apt_dat_dir = pathlib::Path{str_split[1]};
            else if (str_split[0] == PREFS_FPL_DIR)
              fpl_dir = pathlib::Path{str_split[1]};
          }
        }
      }

      file.close();
    }
  }

  void update_prefs() {
    std::ofstream out(PREFS_FILE_NM, std::ofstream::out);

    out << PREFS_EARTH_PATH << " " << earth_nav_path.Get() << "\n";
    out << PREFS_APT_DIR << " " << apt_dat_dir.Get() << "\n";
    out << PREFS_FPL_DIR << " " << fpl_dir.Get() << "\n";

    out.close();
  }

  pathlib::Path get_path_from_user() {
    std::string out;
    std::cin >> out;
    return pathlib::Path{out};
  }

  void get_paths_from_user() {
    bool write_to_prefs = false;

    if (earth_nav_path.Get() == "") {
      std::cout
          << "Please enter path to your Resources/default data directory\n";
      earth_nav_path = get_path_from_user();

      write_to_prefs = true;
    }

    if (apt_dat_dir.Get() == "") {
      std::cout << "Please enter path to the directory containing apt.dat\n";
      apt_dat_dir = get_path_from_user();

      write_to_prefs = true;
    }

    if (fpl_dir.Get() == "") {
      std::cout << "Please enter path to the directory where flight plans "
                   "should be stored\n";
      fpl_dir = get_path_from_user();

      write_to_prefs = true;
    }

    if (write_to_prefs) {
      update_prefs();
    }
  }

  void get_pre_exec_cmds() {
    if (libnav::does_file_exist(CMD_FILE_NM)) {
      std::ifstream file(CMD_FILE_NM);

      std::string line;
      while (getline(file, line)) {
        line = strutils::strip(line);
        if (line.size() && line[0] != '#') pre_exec.push_back(line);
      }

      file.close();
    }
  }

  void load_bytemaps() {
    std::vector<std::pair<std::string, std::string>> tgt = {CDU_BYTEMAP_NAME};

    for (size_t i = 0; i < tgt.size(); i++) {
      geom::vect2_t tex_sz =
          cairo_utils::get_surf_sz(tex_manager_->GetTexture(tgt[i].second));
      bool added = byte_mngr.add_bytemap(TEXTURES_PATH, tgt[i].first,
                                         size_t(tex_sz.x), size_t(tex_sz.y));
      if (!added) {
        std::cout << "Failed to load bytemaps. Aborting\n";
        exit(0);
      }
    }
  }

  void create_avionics() {
    load_bytemaps();

    avncs = std::make_shared<Avionics>(
        apt_dat_dir + "apt.dat", pathlib::Path{"777_arpt.dat"}, 
        pathlib::Path{"777_rnw.dat"},
        earth_nav_path + "earth_fix.dat", earth_nav_path + "earth_nav.dat",
        earth_nav_path + "earth_awy.dat", earth_nav_path + "earth_hold.dat",
        earth_nav_path + "CIFP", fpl_dir);

    nd_data = new fms_displays::NDData{util::OpaquePointer{avncs->fpl_sys},
      util::OpaquePointer{avncs->env_map_ptr_}};
    if (!nd_data->init()) {
      throw "Failed to allocate nd_data\n";
    }
    nd_display = new fms_displays::NDDisplay{
        util::OpaquePointer{nd_data}, util::OpaquePointer{tex_manager_}, 
        ND_POS, ND_SZ, 0};
    cdu_l = new fms_displays::CDU{util::OpaquePointer{avncs->fpl_sys}, 0};
    byteutils::Bytemap* cdu_map = byte_mngr.get_bytemap(CDU_BYTEMAP_NAME.first);
    cdu_display_l = new fms_displays::CDUDisplay{
        CDU_L_POS, CDU_L_SZ, util::OpaquePointer{tex_manager_}, 
        util::OpaquePointer{cdu_l}, false};
    cdu_widget_l = new fms_displays::CDUWidget{
      CDU_L_POS, CDU_L_SZ, util::OpaquePointer{tex_manager_}, 
      util::OpaquePointer{cdu_l}, util::OpaquePointer{cdu_display_l}, 
      util::OpaquePointer{cdu_map}};

    std::cout << "Avionics loaded\n";
  }

  

  void pre_execute_cmds() {
    for (size_t i = 0; i < pre_exec.size(); i++) {
      execute_cmd(pre_exec[i]);

      update();
    }
  }
};
}  // namespace test
