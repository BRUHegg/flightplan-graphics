/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512

	This file contains declarations of member functions for FPLSys class. 
    This class is used to manage all of the flight plans.
*/

#pragma once

#include "flightpln_int.hpp"
#include <iostream>

#define UNUSED(x) (void)(x)


namespace test
{
    constexpr double AC_LAT_DEF = 45.588670483;
    constexpr double AC_LON_DEF = -122.598150383;


    class FPLSys
    {
    public:
        // These are used by commands
        double ac_lat;
        double ac_lon;

        double leg_list_id;
        double seg_list_id;

        bool flt_rwy, flt_proc, flt_trans;

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db_ptr;

        std::shared_ptr<FplnInt> fpl;

        std::pair<size_t, double> leg_sel_cdu_l;
        std::pair<size_t, double> leg_sel_cdu_r;

        std::unordered_map<std::string, std::string> env_vars;

        std::string cifp_dir_path;
        std::string fpl_dir;


        FPLSys(std::shared_ptr<libnav::ArptDB> arpt_db, 
            std::shared_ptr<libnav::NavaidDB> navaid_db, 
            std::shared_ptr<libnav::AwyDB> awy_db, std::string cifp_path, 
            std::string fpl_path);

        std::vector<list_node_ref_t<fpl_seg_t>> get_seg_list(size_t *sz);

        std::vector<list_node_ref_t<leg_list_data_t>> get_leg_list(size_t *sz);

        size_t get_nd_seg(nd_leg_data_t *out, size_t n_max);

        bool get_ctr(geo::point *out, bool fo_side);

        geo::point get_ac_pos();

        void step_ctr(bool bwd, bool fo_side);

        void update();

    private:
        std::vector<list_node_ref_t<fpl_seg_t>> seg_list;
        size_t n_act_seg_list_sz;
        std::vector<list_node_ref_t<leg_list_data_t>> leg_list;
        size_t n_act_leg_list_sz;

        size_t cap_ctr_idx, fo_ctr_idx;
        double fpl_id_last;


        void update_seg_list();

        void update_leg_list();

        void update_lists();

        void update_pos();
    };
}  // namespace test
