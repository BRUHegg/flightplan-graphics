/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512

	This file contains declarations of member functions for flightplan interface class. 
    This class acts as a layer ontop of the flightplan class. Its job is to fetch data
    from appropriate navigation data bases and store it in the flightplan correctly.
*/


#pragma once

#include "flightplan.hpp"
#include <geom.hpp>
#include <libnav/awy_db.hpp>
#include <libnav/str_utils.hpp>
#include <libnav/common.hpp>
#include <assert.h>
#include <set>

#include <iostream>

namespace test
{
    enum ProcType
    {
        PROC_TYPE_SID = 0,
        PROC_TYPE_STAR = 1,
        PROC_TYPE_APPCH = 2
    };
    

    constexpr size_t N_PROC_DB_SZ = 5;
    constexpr size_t N_ARR_DB_OFFSET = 2;
    constexpr size_t N_DFMS_ENRT_WORDS = 6;
    constexpr double DEFAULT_VS_FPM = 2000;
    constexpr double DEFAULT_GS_KTS = 250;
    constexpr double CLB_RATE_FT_PER_NM = 500;
    constexpr double TURN_RADIUS_NM = 1; // Untill there is a VNAV
    constexpr double ASSUMED_RNP_PROC_NM = 1;
    constexpr double ASSUMED_RNP_ENRT_NM = 3;
    constexpr double CF_STRAIGHT_DEV_RAD = (5 * geo::DEG_TO_RAD);
    const std::string NONE_TRANS = "NONE";
    const std::string MISSED_APPR_SEG_NM = "MISSED APPRCH";
    const std::string INTC_LEG_NM = "(INTC)";
    // X-Plane .fms format stuff
    constexpr char DFMS_COL_SEP = ' ';
    constexpr uint8_t N_DFMS_OUT_PREC = 6;
    const std::string DFMS_PADDING = "I\n1100 Version\n";
    const std::string DFMS_RWY_PREFIX = "RW";
    // .fms row headers:
    const std::string DFMS_AIRAC_CYCLE_NM = "CYCLE";
    const std::string DFMS_DEP_NM = "ADEP";
    const std::string DFMS_DEP_RWY_NM = "DEPRWY";
    const std::string DFMS_SID_NM = "SID";
    const std::string DFMS_SID_TRANS_NM = "SIDTRANS";
    const std::string DFMS_ARR_NM = "ADES";
    const std::string DFMS_ARR_RWY_NM = "DESRWY";
    const std::string DFMS_STAR_NM = "STAR";
    const std::string DFMS_STAR_TRANS_NM = "STARTRANS";
    const std::string DFMS_N_ENRT_NM = "NUMENR";

    const std::string DFMS_DIR_SEG_NM = "DRCT";

    const std::string DFMS_FILE_POSTFIX = ".fms";

    const std::set<std::string> NOT_FOLLOWED_BY_DF = {"AF", "CI", "PI", "RF", "VI"};
    const std::set<std::string> AFTER_INTC = {"AF", "CF", "FA", "FC", "FD", "FM", "IF"};
    // The following set contains legs that allow to be offset by a turn(onto 
    /// the current leg)
    const std::set<std::string> TURN_OFFS_LEGS = {"DF", "CI", "CA", "CD", 
        "CR", "VA", "VI", "VR", "VD"};
    const std::set<std::string> LEGS_CALC = {"DF", "TF", "CF", "VA", "CA", "FA", "VI", 
        "CI", "FD", "CD", "VD"};
    //const std::map<std::string, std::set<std::string>> ILLEGAL_NEXT_LEG = {
    //    {"AF", {"DF", "IF", "PI"}},
    //    {"CA", {"AF", "HA", "HF", "HM", "PI", "RF", "TF"}},
    //    {"CD", {"HA", "HF", "HM", "PI", "RF", "TF"}},
    //    {"CF", {"IF"}},
    //    {"CI", {"CA", "CD", "CD", "CR", "DF", "HA", "HF", "HM", "PI", "RF", 
    //    "TF", "VA", "VD", "VI", "VM", "VR"}},
    //    {}
    //};


    struct dfms_arr_data_t
    {
        std::string star, star_trans, arr_rwy, arr_icao;
    };


    /*
        General info:
        turns:
        positive - right turn
        negative - left turn
    */

    bool is_ang_greater(double ang1_rad, double ang2_rad);  // true if ang1 > ang2

    double get_turn_rad(double ang1, geo::point p1, double ang2, geo::point p2);

    double get_turn_by_dir(double curr_brng_rad, double tgt_brng_rad, libnav::TurnDir t_dir);

    double get_cf_big_turn_isect(leg_seg_t curr, leg_t next, double m_var, geo::point *out);

    std::string get_appr_rwy(std::string& appr);

    std::string get_dfms_rwy(std::string& rwy_nm);

    geo::point get_xa_end_point(geo::point prev, float brng_deg, float va_alt_ft, 
        double clb_ft_nm=CLB_RATE_FT_PER_NM);

    geo::point get_dme_end_point(geo::point start, double true_brng_rad, 
        geo::point st, double dist_nm);

    libnav::waypoint_t get_ca_va_wpt(geo::point pos, int n_ft);

    libnav::waypoint_t get_xd_wpt(geo::point pos, std::string main_nm, int dme_nm);

    double get_rnp(leg_list_node_t *leg);


    class FplnInt: public FlightPlan
    {
    public:
        FplnInt(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::shared_ptr<libnav::AwyDB> aw_db, 
            std::string cifp_path);

        // Import from .fms file:

        libnav::DbErr load_from_fms(std::string& file_nm, bool set_arpts=true);

        // Export to .fms file:

        void save_to_fms(std::string& file_nm, bool save_sid_star=true);

        // Airport functions:

        libnav::DbErr set_dep(std::string icao);

        std::string get_dep_icao();

        libnav::DbErr set_arr(std::string icao);

        std::string get_arr_icao();

        // Runway functions:

        std::vector<std::string> get_dep_rwys(bool filter_rwy=false, bool filter_sid=false);

        std::vector<std::string> get_arr_rwys();

        bool set_dep_rwy(std::string& rwy);

        std::string get_dep_rwy();

        bool get_dep_rwy_data(libnav::runway_entry_t *out);

        bool set_arr_rwy(std::string& rwy);

        std::string get_arr_rwy();

        bool get_arr_rwy_data(libnav::runway_entry_t *out);

        // Airport procedure functions:

        std::vector<std::string> get_arpt_proc(ProcType tp, bool is_arr=false, 
            bool filter_rwy=false, bool filter_proc=false);

        std::vector<std::string> get_arpt_proc_trans(ProcType tp, bool is_rwy=false, bool is_arr=false);

        bool set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr=false);

        bool set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr=false);

        // Enroute:

        bool add_enrt_seg(timed_ptr_t<seg_list_node_t> next, std::string name);

        // End MUST be an airway id

        bool awy_insert(timed_ptr_t<seg_list_node_t> next, std::string end_id);

        bool delete_via(timed_ptr_t<seg_list_node_t> next);

        bool delete_seg_end(timed_ptr_t<seg_list_node_t> next);

        // Leg list interface functions:

        void dir_from_to(timed_ptr_t<leg_list_node_t> from, 
            timed_ptr_t<leg_list_node_t> to);

        void add_direct(libnav::waypoint_t wpt, timed_ptr_t<leg_list_node_t> next);

        bool delete_leg(timed_ptr_t<leg_list_node_t> next);

        // Calculation function

        void update(double hdg_trk_diff);

    private:
        std::string arr_rwy;

        std::vector<libnav::str_umap_t> proc_db;
        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::NavaidDB> navaid_db;

        libnav::arinc_rwy_db_t dep_rnw, arr_rnw;
        bool has_dep_rnw_data, has_arr_rnw_data;
        libnav::runway_entry_t dep_rnw_data, arr_rnw_data;

        double fpl_id_calc;


        // Static member functions:

        static size_t get_proc_db_idx(ProcType tp, bool is_arr=false);

        static fpl_segment_types get_proc_tp(ProcType tp);

        static fpl_segment_types get_trans_tp(ProcType tp);

        static std::vector<std::string> get_proc(libnav::str_umap_t& db, std::string rw="", 
            bool is_appch=false);

        static std::vector<std::string> get_proc_trans(std::string proc, libnav::str_umap_t& db, 
            libnav::arinc_rwy_db_t& rwy_db, bool is_rwy=false);

        static std::string get_dfms_enrt_leg(leg_list_node_t* lg, bool force_dir=false);

        // Non-static member functions:

        // Auxiliury functions for import from .fms:

        /*
            Function: process_dfms_term_line
            Description:
            Processes a single line of .fms file describing airports/procedures
            @param l_split: reference to the target split line
            @return error code
        */

        libnav::DbErr process_dfms_proc_line(std::vector<std::string>& l_split, 
            bool set_arpts, dfms_arr_data_t* arr_data);

        libnav::DbErr set_dfms_arr_data(dfms_arr_data_t* arr_data, bool set_arpt);

        bool get_dfms_wpt(std::vector<std::string>& l_split, libnav::waypoint_t* out);

        // Auxiliury functions for export to .fms:

        std::string get_dfms_arpt_leg(bool is_arr=false);
        
        size_t get_dfms_enrt_legs(std::vector<std::string>* out);

        // Other auxiliury functions:

        libnav::arinc_rwy_data_t get_rwy_data(std::string nm, bool is_arr=false);

        bool add_fpl_seg(libnav::arinc_leg_seq_t& legs, fpl_segment_types seg_tp, std::string seg_nm,
            seg_list_node_t *next=nullptr, bool set_ref=true);

        /*
            Function: get_awy_tf_leg
            Description:
            Makes a TF leg using a waypoint id taken from airway data base.
            @param wpt_id: id of the waypoint taken from airway data base. MUST be a valid id
            @return: arinc424 TF leg
        */

        leg_t get_awy_tf_leg(libnav::awy_point_t awy_pt);

        void add_awy_seg(std::string awy, seg_list_node_t *next, 
            std::vector<libnav::awy_point_t>& awy_pts);

        bool set_sid_star(std::string proc_nm, bool is_star=false, bool reset_rwy=true);

        bool set_appch_legs(std::string appch, std::string& arr_rwy, 
            libnav::arinc_leg_seq_t legs);

        bool set_appch(std::string appch);

        bool set_proc_trans(ProcType tp, std::string trans, bool is_arr=false);

        // Calculation functions:

        double get_leg_mag_var_deg(leg_list_node_t *leg);

        double get_leg_turn_rad(leg_list_node_t *curr);

        static bool get_df_start(leg_seg_t curr_seg, leg_t next, geo::point *out);

        /*
            Function: get_to_leg_start
            Description:
            Calculates start of a leg that can be offset by a turn(see TURN_OFFS_LEGS)
            @param curr_seg: current segment
            @param next: next arinc424 leg
            @param out: pointer to where the output should be stored
        */

        static void get_to_leg_start(leg_seg_t curr_seg, leg_t next, 
            double mag_var_deg, double hdg_trk_diff, geo::point *out);

        static bool get_cf_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next, 
            double mag_var_deg, geo::point *out, bool *to_inh, double *turn_radius_out);

        /*
            Function: get_leg_start
            Description:
            Calculates start of next leg.
            @param curr_seg: current segment
            @param curr_leg: current arinc424 leg
            @param next: next arinc424 leg
            @param out: pointer to where the output should be stored
            @param to_inh: set to true when turn offset is inhibited
            (90 degree and more turns). Otherwise not set
            @param turn_radius_nm: where to output turn radius if required
        */

        bool get_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next, 
            double mag_var_deg, double hdg_trk_diff, geo::point *out, 
            bool *to_inh, double *turn_radius_nm);

        static void set_xi_leg(leg_list_node_t *leg);

        /*
            Function: set_turn_offset
            Description:
            Offsets the end of previous leg so that a turn without overshoot is possible.
            @param leg: pointer to a node of leg list
            @param prev_leg: non-bypassed leg before leg. THIS IS IMPORTANT: 
            IT MUST NOT BE BYPASSED.
        */

        static void set_turn_offset(leg_list_node_t *leg, leg_list_node_t *prev_leg);

        // The following functions are used to calculate ends of arinc424 legs.

        /*
            Function: calculate_alt_leg
            Description:
            Calculates end of CA or VA leg
            @param leg: pointer to a node of leg list
            @param hdg_trk_diff: difference between heading and track in radians
        */

        void calculate_alt_leg(leg_list_node_t *leg, double hdg_trk_diff, 
            double curr_alt_ft);

        /*
            Function: calculate_alt_leg
            Description:
            Calculates end of CI or VI leg
            @param leg: pointer to a node of leg list
            @param hdg_trk_diff: difference between heading and track in radians
        */

        void calculate_intc_leg(leg_list_node_t *leg, double hdg_trk_diff);

        void calculate_fc_leg(leg_list_node_t *leg);

        /*
            Function: calculate_dme_leg
            Description:
            Calculates end of CD, FD or VD leg
            @param leg: pointer to a node of leg list
            @param hdg_trk_diff: difference between heading and track in radians
        */

        void calculate_dme_leg(leg_list_node_t *leg, double hdg_trk_diff);

        /*
            Function: calculate_alt_leg
            Description:
            Calculates end of CF/TF/DF leg
            @param leg: pointer to a node of leg list
        */

        void calculate_crs_trk_dir_leg(leg_list_node_t *leg);

        /*
            Function: calculate_leg
            Description:
            Handles all supported leg types.
            @param leg: pointer to a node of leg list
            @param hdg_trk_diff: difference between heading and track in radians
        */

        void calculate_leg(leg_list_node_t *leg, double hdg_trk_diff, double curr_alt_ft);
    };
} // namespace test
