/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512

	This file contains declarations of member functions for flightplan class. This class
    acts as a container that holds arinc424 legs.
*/


#pragma once

#include <libnav/cifp_parser.hpp>
#include <libnav/arpt_db.hpp>
#include <libnav/navaid_db.hpp>
#include <libnav/hold_db.hpp>
#include <libnav/awy_db.hpp>
#include <mutex>
#include <map>
#include "linked_list.hpp"


namespace test
{
    constexpr size_t N_FPL_LEG_CACHE_SZ = 200;
    constexpr size_t N_FPL_SEG_CACHE_SZ = 100;
    constexpr size_t N_FPL_REF_SZ = 9;
    const std::string DISCON_SEG_NAME = "DISCONTINUITY";
    const std::string DCT_LEG_NAME = "DCT";

    enum fpl_segment_types
    {
        FPL_SEG_NONE = 0,
        FPL_SEG_DEP_RWY = 1,
        FPL_SEG_SID = FPL_SEG_DEP_RWY + 1,
        FPL_SEG_SID_TRANS = FPL_SEG_SID + 1,
        FPL_SEG_ENRT = FPL_SEG_SID_TRANS + 1,
        FPL_SEG_STAR_TRANS = FPL_SEG_ENRT + 1,
        FPL_SEG_STAR = FPL_SEG_STAR_TRANS + 1,
        FPL_SEG_APPCH_TRANS = FPL_SEG_STAR + 1,
        FPL_SEG_APPCH = FPL_SEG_APPCH_TRANS + 1
    };

    typedef libnav::arinc_leg_t leg_t;

    const libnav::appr_pref_db_t APPR_PREF_MOD = { // Prefixes for all approaches supported by 777's FMC
        {'B', "LOC"},
        {'G', "IGS"},
        {'I', "ILS"},
        {'N', "NDB"},
        {'Q', "NDME"},
        {'R', "RNV"},
        {'S', "VOR"},
        {'V', "VOR"},
        {'U', "SDF"},
        {'X', "LDA"}
    };


    struct leg_seg_t
    {
        bool is_arc, is_finite, is_rwy;
        geo::point start, end;
        double turn_rad_nm, true_trk_deg;
    };

    struct nd_leg_data_t
    {
        leg_seg_t leg_data;
        geo::point arc_ctr, end_wpt;
        std::string end_name;
    };

    struct leg_list_data_t;

    struct fpl_seg_t
    {
        bool is_direct;
        bool is_discon;
        std::string name;
        fpl_segment_types seg_type;

        struct_util::list_node_t<leg_list_data_t> *end;
    };

    struct leg_list_data_t
    {
        leg_t leg;
        bool is_discon;
        leg_seg_t misc_data;
        struct_util::list_node_t<fpl_seg_t> *seg;
    };

    struct fpl_ref_t
    {
        std::string name;
        struct_util::list_node_t<fpl_seg_t> *ptr;
    };

    template <class T>
    struct list_node_ref_t
    {
        struct_util::list_node_t<T> *ptr;
        T data;
    };

    template <class T>
    struct timed_ptr_t
    {
        T* ptr;
        double id;
    };
    
    
    static const struct struct_util::list_node_t<leg_list_data_t> EmptyNode = 
        {nullptr, nullptr, {}};
    static const struct struct_util::list_node_t<fpl_seg_t> EmptySeg = 
        {nullptr, nullptr, {}};
    static const struct fpl_ref_t EmptyRef = {"", nullptr};


    typedef struct_util::list_node_t<leg_list_data_t> leg_list_node_t;
    typedef struct_util::list_node_t<fpl_seg_t> seg_list_node_t;

    //DEBUG
    std::string get_leg_str(leg_t& leg);


    class FlightPlan
    {
        /*
            Class: FlightPlan
            Description:
            This class is used to store the flight plan. It's supposed to be flexible,
            so it doesn't just store a sequence of legs. There's some added complexity.
            Here's the layout of how things work:
            The flightplan is stored in 3 layers:
            1) Refs
            2) Segments
            3) Individual legs

            1) Refs:
            These are references to a particular section of the flight plan. E.g. SID or
            SID transition. All of these sections are defined in the fpl_segment_types enum.
            Each ref stores a pointer to its end segment and a name associated with it(
                e.g. name of SID, transition, etc.
            ). If a ref doesn't have an end segment i.e. it doesn't exist in the flightplan,
            its pointer to end segment is null. There can be only one ref per each flightplan
            section.
            2) Segments:
            These are sequences of legs. They can represent legs belonging to one airway
            segment or legs that belong to one procedure.
            3) Legs:
            These are basically arinc424 legs.
        */

        typedef std::unordered_map<std::string, std::set<std::string>> str_set_map_t;

    public:
        FlightPlan(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::string cifp_path);

        double get_id();

        size_t get_leg_list_sz();

        size_t get_seg_list_sz();

        /*
            Function: get_ll_seg
            Description:
            Returns a segment of leg list defined by start position and length.
            @param start: start position
            @param l: length
            @param out: pointer to output vector
        */

        double get_ll_seg(size_t start, size_t l, 
            std::vector<list_node_ref_t<leg_list_data_t>>* out);

        /*
            Function: get_sl_seg
            Description:
            Returns a segment of segment list defined by start position and length.
            @param start: start position
            @param l: length
            @param out: pointer to output vector
        */

        double get_sl_seg(size_t start, size_t l, 
            std::vector<list_node_ref_t<fpl_seg_t>>* out);

        ~FlightPlan();

        void print_refs();

    protected:
        std::shared_ptr<libnav::ArptDB> arpt_db;
        std::shared_ptr<libnav::NavaidDB> navaid_db;

        libnav::Airport *departure, *arrival;

        std::vector<fpl_ref_t> fpl_refs;

        struct_util::linked_list_t<leg_list_data_t> leg_list;
        struct_util::linked_list_t<fpl_seg_t> seg_list;

        struct_util::ll_node_stack_t<leg_list_node_t> leg_data_stack;
        struct_util::ll_node_stack_t<seg_list_node_t> seg_stack;

        std::mutex fpl_mtx;

        std::chrono::time_point<std::chrono::steady_clock> start;

        double fpl_id_curr;


        void update_id();

        bool legcmp(leg_t& leg1, leg_t& leg2);

        libnav::DbErr set_arpt(std::string icao, libnav::Airport **ptr, bool is_arr=false);

        /*
            Function: delete_range
            Description:
            Safely deletes all legs between start and end. start and end don't get
            deleted. The function takes care of updating segments and refs if necessary.
            @param start: start node
            @param end: end node
        */

        void delete_range(leg_list_node_t *start, leg_list_node_t *end);

        void delete_ref(fpl_segment_types ref);
        
        void delete_segment(seg_list_node_t *seg, bool leave_seg=true, bool add_disc=false,
            bool ignore_tail=false);

        void add_segment(std::vector<leg_t>& legs, fpl_segment_types seg_tp,
            std::string seg_name, seg_list_node_t *next, bool is_direct=false);

        void add_discon(seg_list_node_t *next);

        void add_legs(leg_t start, std::vector<leg_t>& legs, fpl_segment_types seg_tp,
            std::string seg_name, seg_list_node_t *next=nullptr);

        void add_direct_leg(leg_t leg, leg_list_node_t *next);

        bool delete_singl_leg(leg_list_node_t *leg);

    private:
        std::map<fpl_segment_types, std::string> seg_to_str = {
            {FPL_SEG_DEP_RWY, "DEP RWY"},
            {FPL_SEG_SID, "SID"},
            {FPL_SEG_SID_TRANS, "SID TRANS"},
            {FPL_SEG_ENRT, "ENROUTE"},
            {FPL_SEG_STAR_TRANS, "STAR TRANS"},
            {FPL_SEG_STAR, "STAR"},
            {FPL_SEG_APPCH_TRANS, "APPR TRANS"},
            {FPL_SEG_APPCH, "APPR"}
        };

        
        std::string cifp_dir_path;


        // WARNING: these do not lock flight plan mutex
        void reset_fpln(bool leave_dep_rwy=false);

        void delete_between(leg_list_node_t *start, leg_list_node_t *end);

        void add_singl_leg(leg_list_node_t *next, leg_list_data_t data);

        /*
            Function: get_insert_seg
            Description:
            This function determines the segment before which the other segment 
            should be inserted for the add_legs function.
            @param seg_tp: type of the segment to be inserted
            @param next_seg: pointer to where the function should write the second
            return value
            @return: pointer to segment before which the new segment should be inserted,
            pointer to the segment after the segment sequence of seg_tp.
        */

        seg_list_node_t *get_insert_seg(fpl_segment_types seg_tp, 
            seg_list_node_t **next_seg);

        void merge_seg(seg_list_node_t *tgt);

        seg_list_node_t *subdivide(leg_list_node_t *prev_leg, leg_list_node_t *next_leg);
    };
}
