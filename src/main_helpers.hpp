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
#include <displays/common/texture_manager.hpp>
#include <displays/ND/nd.hpp>
#include <fpln/fpl_cmds.hpp>
#include <fpln/fpln_sys.hpp>
#include <util/json_require.hpp>
#include <util/pathlib.hpp>

namespace fms_core {
const std::string CMD_FILE_NM = "cmds.txt";
const std::string PREFS_FILE_NM = "prefs.txt";

const std::string PREFS_EARTH_PATH = "EPATH";
const std::string PREFS_APT_DIR = "APTDIR";
const std::string PREFS_FPL_DIR = "FPLDIR";

const std::string BOEING_FONT_NAME = "BoeingFont.ttf";
const std::pair<std::string, std::string> CDU_BYTEMAP_NAME = {
    "cdu_key_map", fms_displays::CDU_TEXTURE_NAME};
const std::string TEXTURES_PATH = "textures/";

constexpr double WND_HEIGHT = 900;
constexpr double CDU_WIDTH =
    (fms_displays::CDU_TEXTURE_ASPECT_RATIO * WND_HEIGHT);
constexpr double ND_WIDTH = WND_HEIGHT;
constexpr double WND_WIDTH = CDU_WIDTH + ND_WIDTH;
constexpr geom::vect2_t ND_POS = {CDU_WIDTH, 0};
constexpr geom::vect2_t ND_SZ = {ND_WIDTH, ND_WIDTH};
constexpr geom::vect2_t CDU_L_POS = {0, 0};
constexpr geom::vect2_t CDU_L_SZ = {CDU_WIDTH, WND_HEIGHT};

class Avionics {
 public:
  std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
  std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

  std::shared_ptr<libnav::AwyDB> awy_db;
  std::shared_ptr<libnav::HoldDB> hold_db;

  std::shared_ptr<FPLSys> fpl_sys;
  std::shared_ptr<fms_environment::EnvDataRefMap> env_map_ptr_;

  pathlib::Path cifp_dir_path;

  Avionics(pathlib::Path apt_dat, pathlib::Path custom_apt, pathlib::Path custom_rnw,
           pathlib::Path fix_data, pathlib::Path navaid_data, pathlib::Path awy_data,
           pathlib::Path hold_data, pathlib::Path cifp_path, pathlib::Path fpl_path) {
    cifp_dir_path = cifp_path;

    arpt_db_ptr =
        std::make_shared<libnav::ArptDB>(apt_dat.Get(), custom_apt.Get(), custom_rnw.Get());
    navaid_db_ptr = std::make_shared<libnav::NavaidDB>(fix_data.Get(), navaid_data.Get());
    awy_db = std::make_shared<libnav::AwyDB>(awy_data.Get());
    hold_db = std::make_shared<libnav::HoldDB>(hold_data.Get());

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
      std::make_shared<fms_environment::EnvDataRefMap>(
        fms_environment::kBaseVariables);

    fpl_sys = std::make_shared<FPLSys>(arpt_db_ptr, navaid_db_ptr, awy_db,
                                      env_map_ptr_,
                                       cifp_dir_path, fpl_path);
  }

  void update() { fpl_sys->update(); }

  ~Avionics() {
    fpl_sys.reset();
    env_map_ptr_.reset();
    hold_db.reset();
    awy_db.reset();
    navaid_db_ptr.reset();
    navaid_db_ptr.reset();
    arpt_db_ptr.reset();
  }
};

class CMDInterface {
 public:
  std::shared_ptr<Avionics> avncs;
  std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;
  std::shared_ptr<fms_displays::NDData> nd_data;
  std::shared_ptr<fms_displays::NDDisplay> nd_display;
  std::shared_ptr<fms_displays::CDU> cdu_l;
  std::shared_ptr<fms_displays::CDUDisplay> cdu_display_l;
  std::shared_ptr<fms_displays::CDUWidget> cdu_widget_l;

  byteutils::bytemap_manager_t byte_mngr;

  CMDInterface() {

    pre_exec = {};

    if(!json_data_.init(pathlib::Path{"config.json"}, GetJsonConfigTree())) {
      throw std::logic_error{"There was a problem loading config.json"};
    }
    fetch_prefs_data();
    get_paths_from_user();
    get_pre_exec_cmds();
    create_avionics();
    pre_execute_cmds();
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
    avncs->update();
    nd_data->update();
  }

  void main_loop() {
    while (1) {
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
      }catch (const nlohmann::json::parse_error& e) {
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

  void load_fonts() {
    FT_Init_FreeType(&lib);

    bool font_loaded = false;
    if (libnav::does_file_exist(BOEING_FONT_NAME)) {
      font_loaded = cairo_utils::load_font(BOEING_FONT_NAME, lib, &font,
                                           &boeing_font_face);
    } else {
      std::cout << "Font file " << BOEING_FONT_NAME << " was not found.\n";
    }

    if (!font_loaded) {
      std::cout << "Failed to load font: " << BOEING_FONT_NAME
                << " . Aborting\n";
      exit(0);
    }
  }

  void load_textures() {
    std::vector<std::string> tgt_names = {
        fms_displays::WPT_ACT_NAME,
        fms_displays::WPT_INACT_NAME,
        fms_displays::AIRPLANE_NAME,
        fms_displays::PLN_BACKGND_INNER_NAME,
        fms_displays::PLN_BACKGND_OUTER_NAME,
        fms_displays::MAP_BACKGND_NAME,
        fms_displays::MAP_AC_TRI_NAME,
        fms_displays::MAP_HDG_NAME,
        fms_displays::HTRK_BOX_NAME,
        fms_displays::ARPT_NML_POI_NAME,
        fms_displays::ARPT_ALTN_POI_NAME,
        fms_displays::VORDME_POI_NAME,
        fms_displays::DME_POI_NAME,

        fms_displays::CDU_TEXTURE_NAME,
        fms_displays::CDU_WHITE_TEXT_NAME,
        fms_displays::CDU_GREEN_TEXT_NAME,
        fms_displays::CDU_CYAN_TEXT_NAME,
        fms_displays::CDU_MAGENTA_TEXT_NAME};

    tex_mngr = std::make_shared<cairo_utils::texture_manager_t>();

    if (!tex_mngr->load(tgt_names, TEXTURES_PATH)) {
      std::cout << "Failed to load textures. Aborting\n";
      tex_mngr->destroy();
      exit(0);
    }
  }

  void load_bytemaps() {
    std::vector<std::pair<std::string, std::string>> tgt = {CDU_BYTEMAP_NAME};

    for (size_t i = 0; i < tgt.size(); i++) {
      geom::vect2_t tex_sz =
          cairo_utils::get_surf_sz(tex_mngr->data[tgt[i].second]);
      bool added = byte_mngr.add_bytemap(TEXTURES_PATH, tgt[i].first,
                                         size_t(tex_sz.x), size_t(tex_sz.y));
      if (!added) {
        std::cout << "Failed to load bytemaps. Aborting\n";
        exit(0);
      }
    }
  }

  void create_avionics() {
    load_fonts();
    load_textures();
    load_bytemaps();

    avncs = std::make_shared<Avionics>(
        apt_dat_dir + "apt.dat", pathlib::Path{"777_arpt.dat"}, 
        pathlib::Path{"777_rnw.dat"},
        earth_nav_path + "earth_fix.dat", earth_nav_path + "earth_nav.dat",
        earth_nav_path + "earth_awy.dat", earth_nav_path + "earth_hold.dat",
        earth_nav_path + "CIFP", fpl_dir);

    nd_data = std::make_shared<fms_displays::NDData>(avncs->fpl_sys);
    if (!nd_data->init()) {
      throw "Failed to allocate nd_data\n";
    }
    nd_display = std::make_shared<fms_displays::NDDisplay>(
        nd_data, tex_mngr, boeing_font_face, ND_POS, ND_SZ, 0);
    cdu_l = std::make_shared<fms_displays::CDU>(avncs->fpl_sys, 0);
    byteutils::Bytemap* cdu_map = byte_mngr.get_bytemap(CDU_BYTEMAP_NAME.first);
    cdu_display_l = std::make_shared<fms_displays::CDUDisplay>(
        CDU_L_POS, CDU_L_SZ, boeing_font_face, tex_mngr, cdu_l);
    cdu_widget_l = std::make_shared<fms_displays::CDUWidget>(
      CDU_L_POS, CDU_L_SZ, tex_mngr, cdu_l, cdu_display_l, cdu_map);

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
