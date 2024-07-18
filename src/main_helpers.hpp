#pragma once


#include "fpln/fpln_sys.hpp"
#include "fpln/fpl_cmds.hpp"
#include "displays/ND/nd.hpp"
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
        std::shared_ptr<Avionics> avncs;
        std::shared_ptr<StratosphereAvionics::NDData> nd_data;

        std::string earth_nav_path;
	    std::string apt_dat_dir;
        std::string fpl_dir;

        std::vector<std::string> pre_exec;


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

        void create_avionics()
        {
            avncs = std::make_shared<Avionics>(apt_dat_dir+"apt.dat", "777_arpt.dat", 
                "777_rnw.dat", earth_nav_path+"earth_fix.dat", 
                earth_nav_path+"earth_nav.dat", 
                earth_nav_path+"earth_awy.dat", 
                earth_nav_path+"earth_hold.dat", earth_nav_path+"CIFP", fpl_dir);

            nd_data = std::make_shared<StratosphereAvionics::NDData>(avncs->fpl_sys);

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
