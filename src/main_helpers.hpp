/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This header file contains helper functions for the app itself.
    Author: discord/bruh4096#4512(Tim G.)
*/


#pragma once


#include "fpln/fpln_sys.hpp"
#include "fpln/fpl_cmds.hpp"
#include "displays/ND/nd.hpp"
#include "displays/CDU/cdu.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <libnav/geo_utils.hpp>
#include <libnav/common.hpp>
#include <libnav/str_utils.hpp>


namespace test
{
    const std::string CMD_FILE_NM = "cmds.txt";
    const std::string PREFS_FILE_NM = "prefs.txt";

    const std::string PREFS_EARTH_PATH = "EPATH";
    const std::string PREFS_APT_DIR = "APTDIR";
    const std::string PREFS_FPL_DIR = "FPLDIR";

    const std::string BOEING_FONT_NAME = "BoeingFont.ttf";
    const std::pair<std::string, std::string> CDU_BYTEMAP_NAME = {"cdu_key_map", 
        StratosphereAvionics::CDU_TEXTURE_NAME};
    const std::string TEXTURES_PATH = "textures/";

    constexpr double WND_HEIGHT = 900;
    constexpr double CDU_WIDTH = (StratosphereAvionics::CDU_TEXTURE_ASPECT_RATIO * WND_HEIGHT);
    constexpr double ND_WIDTH = WND_HEIGHT;
    constexpr double WND_WIDTH = CDU_WIDTH + ND_WIDTH;
    constexpr geom::vect2_t ND_POS = {CDU_WIDTH, 0};
    constexpr geom::vect2_t ND_SZ = {ND_WIDTH, ND_WIDTH};
    constexpr geom::vect2_t CDU_L_POS = {0, 0};
    constexpr geom::vect2_t CDU_L_SZ = {CDU_WIDTH, WND_HEIGHT};


    class Avionics
    {
    public:
        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::HoldDB> hold_db;

        std::shared_ptr<FPLSys> fpl_sys;

        std::string cifp_dir_path;


        Avionics(std::string apt_dat, std::string custom_apt, std::string custom_rnw,
            std::string fix_data, std::string navaid_data, std::string awy_data,
            std::string hold_data, std::string cifp_path, std::string fpl_path)
        {

            cifp_dir_path = cifp_path;

            arpt_db_ptr = 
                std::make_shared<libnav::ArptDB>(apt_dat, custom_apt, custom_rnw);
	        navaid_db_ptr = 
                std::make_shared<libnav::NavaidDB>(fix_data, navaid_data);
            awy_db = std::make_shared<libnav::AwyDB>(awy_data);
            hold_db = std::make_shared<libnav::HoldDB>(hold_data);

            libnav::DbErr err_arpt = arpt_db_ptr->get_err();
            libnav::DbErr err_wpt = navaid_db_ptr->get_wpt_err();
            libnav::DbErr err_nav = navaid_db_ptr->get_navaid_err();
            libnav::DbErr err_awy = awy_db->get_err();
            libnav::DbErr err_hold = hold_db->get_err();

            std::cout << navaid_db_ptr->get_wpt_cycle() << " " <<
                navaid_db_ptr->get_navaid_cycle() << " " << 
                awy_db->get_airac() << " " << hold_db->get_airac() << "\n";

            std::cout << "Fix data base version: " << 
                navaid_db_ptr->get_wpt_version() << "\n";
            std::cout << "Navaid data base version: " << 
                navaid_db_ptr->get_navaid_version() << "\n";

            if(err_arpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airport database\n";
            }
            if(err_wpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load waypoint database\n";
            }
            if(err_nav != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load navaid database\n";
            }
            if(err_awy != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airway database\n";
            }
            if(err_hold != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load hold database\n";
            }

            fpl_sys = std::make_shared<FPLSys>(arpt_db_ptr, navaid_db_ptr, awy_db, 
                cifp_dir_path, fpl_path);
        }

        void update()
        {
            fpl_sys->update();
        }

        ~Avionics()
        {
            fpl_sys.reset();
            hold_db.reset();
            awy_db.reset();
            navaid_db_ptr.reset();
            navaid_db_ptr.reset();
            arpt_db_ptr.reset();
        }
    };


    class CMDInterface
    {
    public:
        std::shared_ptr<Avionics> avncs;
        std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;
        std::shared_ptr<StratosphereAvionics::NDData> nd_data;
        std::shared_ptr<StratosphereAvionics::NDDisplay> nd_display;
        std::shared_ptr<StratosphereAvionics::CDU> cdu_l;
        std::shared_ptr<StratosphereAvionics::CDUDisplay> cdu_display_l;

        byteutils::bytemap_manager_t byte_mngr;


        CMDInterface()
        {
            earth_nav_path = "";
            apt_dat_dir = "";

            pre_exec = {};

            fetch_prefs_data();
            get_paths_from_user();
            get_pre_exec_cmds();
            create_avionics();
            pre_execute_cmds();
        }

        void execute_cmd(std::string in_raw)
        {
            std::string in_proc = strutils::strip(in_raw, ' ');

            std::vector<std::string> line_split = strutils::str_split(in_proc, ' ');
            if(line_split.size())
            {
                std::string cmd_name = line_split[0];
                std::vector<std::string> args = std::vector<std::string>(line_split.begin() + 1, 
                    line_split.end());

                if(test::cmd_map.find(cmd_name) != test::cmd_map.end())
                {
                    test::FPLSys *ptr = avncs->fpl_sys.get();
                    test::cmd_map[cmd_name](ptr, args);
                }
                else
                {
                    std::cout << "Invalid command name\n";
                }
            }
        }

        void on_click(geom::vect2_t pos)
        {
            cdu_display_l->on_click(pos);
        }

        void draw(cairo_t *cr)
        {
            cdu_l->update();
            nd_display->draw(cr);
            cdu_display_l->draw(cr);
        }

        void update()
        {
            avncs->update();
            nd_data->update();
        }

        void main_loop()
        {
            while(1)
            {
                std::string in_raw;
                std::cout << ">> ";
                std::getline(std::cin, in_raw);

                execute_cmd(in_raw);
                
                update();
            }
        }
    
    private:
        std::string earth_nav_path;
	    std::string apt_dat_dir;
        std::string fpl_dir;

        std::vector<std::string> pre_exec;

        FT_Library lib;
        FT_Face font;
        cairo_font_face_t* boeing_font_face;


        void fetch_prefs_data()
        {
            if(libnav::does_file_exist(PREFS_FILE_NM))
            {
                std::ifstream file(PREFS_FILE_NM);

                std::string line;
                while(getline(file, line))
                {
                    line = strutils::strip(line);
                    if(line.size() && line[0] != '#')
                    {
                        std::vector<std::string> str_split = strutils::str_split(line, ' ', 1);

                        if(str_split.size() == 2)
                        {
                            if(str_split[0] == PREFS_EARTH_PATH)
                                earth_nav_path = str_split[1];
                            else if(str_split[0] == PREFS_APT_DIR)
                                apt_dat_dir = str_split[1];
                            else if(str_split[0] == PREFS_FPL_DIR)
                                fpl_dir = str_split[1];
                        }
                    }
                }

                file.close();
            }
        }

        void update_prefs()
        {
            std::ofstream out(PREFS_FILE_NM, std::ofstream::out);

            out << PREFS_EARTH_PATH << " " << earth_nav_path << "\n";
            out << PREFS_APT_DIR << " " << apt_dat_dir << "\n";
            out << PREFS_FPL_DIR << " " << fpl_dir << "\n";

            out.close();
        }

        std::string get_path_from_user()
        {
            std::string out;
            while(!out.size())
                std::getline(std::cin, out);
            if(out.back() != '/')
                out += "/";
            return out;
        }

        void get_paths_from_user()
        {
            bool write_to_prefs = false;

            if(earth_nav_path == "")
            {
                std::cout << "Please enter path to your Resources/default data directory\n";
                earth_nav_path = get_path_from_user();

                write_to_prefs = true;
            }

            if(apt_dat_dir == "")
            {
                std::cout << "Please enter path to the directory containing apt.dat\n";
                apt_dat_dir = get_path_from_user();

                write_to_prefs = true;
            }

            if(fpl_dir == "")
            {
                std::cout << "Please enter path to the directory where flight plans should be stored\n";
                fpl_dir = get_path_from_user();

                write_to_prefs = true;
            }

            if(write_to_prefs)
            {
                update_prefs();
            }
        }

        void get_pre_exec_cmds()
        {
            if(libnav::does_file_exist(CMD_FILE_NM))
            {
                std::ifstream file(CMD_FILE_NM);

                std::string line;
                while(getline(file, line))
                {
                    line = strutils::strip(line);
                    if(line.size() && line[0] != '#')
                        pre_exec.push_back(line);
                }

                file.close();
            }
        }

        void load_fonts()
        {
            FT_Init_FreeType(&lib);

            bool font_loaded = false;
            if(libnav::does_file_exist(BOEING_FONT_NAME))
            {
                font_loaded = cairo_utils::load_font(BOEING_FONT_NAME, lib, &font, 
                    &boeing_font_face);
            }
            else
            {
                std::cout << "Font file " << BOEING_FONT_NAME << " was not found.\n";
            }

            if(!font_loaded)
            {
                std::cout << "Failed to load font: " << BOEING_FONT_NAME << " . Aborting\n";
                exit(0);
            }
        }

        void load_textures()
        {
            std::vector<std::string> tgt_names = {
                StratosphereAvionics::WPT_ACT_NAME,
                StratosphereAvionics::WPT_INACT_NAME,
                StratosphereAvionics::AIRPLANE_NAME,
                StratosphereAvionics::PLN_BACKGND_INNER_NAME,
                StratosphereAvionics::PLN_BACKGND_OUTER_NAME,
                StratosphereAvionics::MAP_BACKGND_NAME,
                StratosphereAvionics::MAP_AC_TRI_NAME,
                StratosphereAvionics::MAP_HDG_NAME,
                StratosphereAvionics::HTRK_BOX_NAME,
                
                StratosphereAvionics::CDU_TEXTURE_NAME,
                StratosphereAvionics::CDU_WHITE_TEXT_NAME,
                StratosphereAvionics::CDU_GREEN_TEXT_NAME,
                StratosphereAvionics::CDU_CYAN_TEXT_NAME,
                StratosphereAvionics::CDU_MAGENTA_TEXT_NAME
                };
            
            tex_mngr = std::make_shared<cairo_utils::texture_manager_t>();
            
            if(!tex_mngr->load(tgt_names, TEXTURES_PATH))
            {
                std::cout << "Failed to load textures. Aborting\n";
                tex_mngr->destroy();
                exit(0);
            }
        }

        void load_bytemaps()
        {
            std::vector<std::pair<std::string, std::string>> tgt = {CDU_BYTEMAP_NAME};

            for(size_t i = 0; i < tgt.size(); i++)
            {
                geom::vect2_t tex_sz = cairo_utils::get_surf_sz(
                    tex_mngr->data[tgt[i].second]);
                bool added = byte_mngr.add_bytemap(TEXTURES_PATH, tgt[i].first, 
                    size_t(tex_sz.x), size_t(tex_sz.y));
                if(!added)
                {
                    std::cout << "Failed to load bytemaps. Aborting\n";
                    exit(0);
                }
            }
        }

        void create_avionics()
        {
            load_fonts();
            load_textures();
            load_bytemaps();

            avncs = std::make_shared<Avionics>(apt_dat_dir+"apt.dat", "777_arpt.dat", 
                "777_rnw.dat", earth_nav_path+"earth_fix.dat", 
                earth_nav_path+"earth_nav.dat", 
                earth_nav_path+"earth_awy.dat", 
                earth_nav_path+"earth_hold.dat", earth_nav_path+"CIFP", fpl_dir);

            nd_data = std::make_shared<StratosphereAvionics::NDData>(avncs->fpl_sys);
            nd_display = std::make_shared<StratosphereAvionics::NDDisplay>(
                nd_data, tex_mngr, boeing_font_face, ND_POS, ND_SZ, 0);
            cdu_l = std::make_shared<StratosphereAvionics::CDU>(avncs->fpl_sys, 0);
            byteutils::Bytemap *cdu_map = byte_mngr.get_bytemap(CDU_BYTEMAP_NAME.first);
            cdu_display_l = std::make_shared<StratosphereAvionics::CDUDisplay>(
                CDU_L_POS, CDU_L_SZ, boeing_font_face, tex_mngr, cdu_l, cdu_map
            );

            std::cout << "Avionics loaded\n";
        }

        void pre_execute_cmds()
        {
            for(size_t i = 0; i < pre_exec.size(); i++)
            {
                execute_cmd(pre_exec[i]);

                update();
            }
        }
    };
}
