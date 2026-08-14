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

#include <memory>
#include <string>

#include <libnav/str_utils.hpp>

#include "environment.hpp"
#include "fpln_sys.hpp"

#define UNUSED(x) (void)(x)

namespace fms_commands {

struct command_res_t {
  fms_core::FPLSys* fpl_sys;
  fms_environment::EnvDataRefMap* env_map;
};

typedef void (*cmd_t)(command_res_t, std::vector<std::string>&);

using flightplan_type = typename fms_core::FPLSys::flightplan_type;

bool invoke(const std::string& cmd_name, 
  command_res_t cmd_resources, std::vector<std::string>& in);

void set_var(command_res_t cmd_resources, std::vector<std::string>& in);

void print(command_res_t cmd_resources, std::vector<std::string>& in);

void quit(command_res_t cmd_resources, std::vector<std::string>& in);

void load_fpln(command_res_t cmd_resources, std::vector<std::string>& in);

void save_fpln(command_res_t cmd_resources, std::vector<std::string>& in);

void set_filter(command_res_t cmd_resources, std::vector<std::string>& in);

void fplinfo(command_res_t cmd_resources, std::vector<std::string>& in);

void set_fpl_dep(command_res_t cmd_resources, std::vector<std::string>& in);

void set_fpl_arr(command_res_t cmd_resources, std::vector<std::string>& in);

void set_dep_rwy(command_res_t cmd_resources, std::vector<std::string>& in);

void set_arr_rwy(command_res_t cmd_resources, std::vector<std::string>& in);

void get_dep_rwys(command_res_t cmd_resources, std::vector<std::string>& in);

void get_arr_rwys(command_res_t cmd_resources, std::vector<std::string>& in);

void get_proc(command_res_t cmd_resources, std::vector<std::string>& in);

void set_proc(command_res_t cmd_resources, std::vector<std::string>& in);

void print_legs(command_res_t cmd_resources, std::vector<std::string>& in);

void add_via(command_res_t cmd_resources, std::vector<std::string>& in);

void delete_via(command_res_t cmd_resources, std::vector<std::string>& in);

void add_to(command_res_t cmd_resources, std::vector<std::string>& in);

void delete_to(command_res_t cmd_resources, std::vector<std::string>& in);

void legs_set(command_res_t cmd_resources, std::vector<std::string>& in);

void delete_leg(command_res_t cmd_resources, std::vector<std::string>& in);

void print_seg(command_res_t cmd_resources, std::vector<std::string>& in);

void print_refs(command_res_t cmd_resources, std::vector<std::string>& in);

void help(command_res_t cmd_resources, std::vector<std::string>& in);
}  // namespace fms_commands
