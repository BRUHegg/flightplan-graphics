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
    const std::string AC_LAT_DEG_VAR = "ac_lat_deg";
    const std::string AC_LON_DEG_VAR = "ac_lon_deg";
    const std::string AC_BRNG_TRU_DEG_VAR = "ac_brng_tru_deg";
    const std::string AC_SLIP_DEG_VAR = "ac_slip_deg";
    const std::string AC_MAGVAR_DEG_VAR = "ac_magvar_deg";
    const std::string AC_GS_KTS_VAR = "ac_gs_kts";
    const std::string AC_TAS_KTS_VAR = "ac_tas_kts";
    const std::string FPL_SEL = "fpl_sel";

    
    // FplSys stores 3 routes in fpl_vec
    // Index 0 is for active route, 1 for RTE1 and 2 for RTE2
    constexpr size_t N_FPL_SYS_RTES = 3;
    constexpr size_t ACT_RTE_IDX = 0;
    constexpr size_t RTE1_IDX = 1;
    constexpr size_t RTE2_IDX = 2;

    constexpr double AC_LAT_DEF = 45.588670483;
    constexpr double AC_LON_DEF = -122.598150383;
    constexpr double AC_BRNG_TRU_DEF = 175;
    constexpr double AC_SLIP_DEF = 0;
    constexpr double AC_MAGVAR_DEF = 0;
    constexpr double AC_GS_KTS_DEF = 0;
    constexpr double AC_TAS_KTS_DEF = 0;
    constexpr double FPL_SEL_DEF = 1;

    constexpr double DIST_FONT_SZ_DD = 21; // Double digits;
    constexpr double DIST_FONT_SZ_TD = 19;

    const std::vector<std::string> RSV_VARS = {AC_LAT_DEG_VAR, AC_LON_DEG_VAR, 
        AC_BRNG_TRU_DEG_VAR, AC_SLIP_DEG_VAR, AC_MAGVAR_DEG_VAR, AC_GS_KTS_VAR,
        AC_TAS_KTS_VAR, FPL_SEL};

    const std::vector<double> RSV_VAR_VAL = {AC_LAT_DEF, AC_LON_DEF, AC_BRNG_TRU_DEF,
        AC_SLIP_DEF, AC_MAGVAR_DEF, AC_GS_KTS_DEF, AC_TAS_KTS_DEF, FPL_SEL_DEF};


    struct hdg_info_t
    {
        double brng_tru_rad, slip_rad, magvar_rad;
    };

    struct spd_info_t
    {
        double gs_kts, tas_kts;
    };

    struct act_leg_info_t
    {
        std::string name, time_z, dist_nm;
        double dist_sz;
    };

    struct fpln_info_t
    {
        double leg_list_id;
        double seg_list_id;

        size_t cap_ctr_idx, fo_ctr_idx;
        double fpl_id_last;
        int act_leg_idx;
    };

    struct fpln_data_t: fpln_info_t
    {
        std::vector<list_node_ref_t<fpl_seg_t>> seg_list;
        std::vector<list_node_ref_t<leg_list_data_t>> leg_list;
    };

    class FPLSys
    {
    public:
        // These are used by commands
        // Position:
        double ac_lat_deg;
        double ac_lon_deg;
        double ac_brng_deg;
        double ac_slip_deg;
        double ac_magvar_deg;
        // Speed
        double ac_gs_kts;
        double ac_tas_kts;
        
        size_t cmd_fpl_idx;

        bool flt_rwy, flt_proc, flt_trans;

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db_ptr;

        std::vector<std::shared_ptr<FplnInt>> fpl_vec;

        std::pair<size_t, double> leg_sel_cdu_l;
        std::pair<size_t, double> leg_sel_cdu_r;

        std::unordered_map<std::string, std::string> env_vars;

        std::string cifp_dir_path;
        std::string fpl_dir;


        FPLSys(std::shared_ptr<libnav::ArptDB> arpt_db, 
            std::shared_ptr<libnav::NavaidDB> navaid_db, 
            std::shared_ptr<libnav::AwyDB> awy_db, std::string cifp_path, 
            std::string fpl_path);

        bool get_exec();

        size_t get_act_idx();

        std::vector<list_node_ref_t<fpl_seg_t>> get_seg_list(size_t *sz, size_t idx=0);

        std::vector<list_node_ref_t<leg_list_data_t>> get_leg_list(size_t *sz, size_t idx=0);

        size_t get_nd_seg(nd_leg_data_t *out, size_t n_max, size_t idx=0);

        int get_act_leg_idx(size_t idx=0);

        bool get_ctr(geo::point *out, bool fo_side, size_t idx=0);

        geo::point get_ac_pos();

        hdg_info_t get_hdg_info();

        spd_info_t get_spd_info();

        fpln_info_t get_fpl_info(size_t idx=0);

        act_leg_info_t get_act_leg_info(size_t idx=0);

        void step_ctr(bool bwd, bool fo_side, size_t idx=0);

        void rte_activate(size_t idx);

        void set_flt_nbr(std::string str);

        std::string get_flt_nbr();

        void execute();

        void erase();

        void update();

    private:
        std::vector<fpln_data_t> fpl_datas;

        size_t act_rte_idx;
        double act_id;
        std::vector<size_t> cdu_rte_idx;
        std::string flt_nbr;
        std::vector<std::string> fnb_dep_icao;  // Departure icaos used to reset flight number

        bool m_exec_st;


        void update_seg_list(size_t idx=0);

        void update_leg_list(size_t idx=0);

        void update_lists(size_t idx=0);

        void update_flt_nbr(size_t idx=0);

        void update_pos();
    };
}  // namespace test
