/*
    This project is licensed under
    Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

    A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

    Author: discord/bruh4096#4512

    This file contains definitions of member functions for FPLSys class.
    This class is used to manage all of the flight plans.
*/

#include "fpln_sys.hpp"

namespace test
{
    // FPLSys member function definitions:

    // Public member functions:

    FPLSys::FPLSys(std::shared_ptr<libnav::ArptDB> arpt_db,
        std::shared_ptr<libnav::NavaidDB> navaid_db,
        std::shared_ptr<libnav::AwyDB> awy_db, std::string cifp_path, std::string fpl_path)
    {
        // Initialize reserved variables:
        assert(RSV_VARS.size() == RSV_VAR_VAL.size());

        for(size_t i = 0; i < RSV_VARS.size(); i++)
        {
            env_vars[RSV_VARS[i]] = strutils::double_to_str(RSV_VAR_VAL[i], 9);
        }

        leg_list_id = -1;
        seg_list_id = -1;

        flt_rwy = false;
        flt_proc = false;
        flt_trans = false;

        cifp_dir_path = cifp_path;
        fpl_dir = fpl_path;

        ac_lat_deg = AC_LAT_DEF;
        ac_lon_deg = AC_LON_DEF;

        arpt_db_ptr = arpt_db;
        navaid_db_ptr = navaid_db;
        awy_db_ptr = awy_db;

        fpl = std::make_shared<FplnInt>(arpt_db_ptr, navaid_db_ptr, awy_db_ptr,
                                        cifp_dir_path);

        leg_sel_cdu_l = {0, 0};
        leg_sel_cdu_r = {0, 0};

        n_act_seg_list_sz = 0;
        n_act_leg_list_sz = 0;

        cap_ctr_idx = 1;
        fo_ctr_idx = 1;

        fpl_id_last = 0;
        act_leg_idx = -1;
    }

    std::vector<list_node_ref_t<fpl_seg_t>> FPLSys::get_seg_list(size_t *sz)
    {
        *sz = n_act_seg_list_sz;
        return seg_list;
    }

    std::vector<list_node_ref_t<leg_list_data_t>> FPLSys::get_leg_list(size_t *sz)
    {
        *sz = n_act_leg_list_sz;
        return leg_list;
    }

    size_t FPLSys::get_nd_seg(nd_leg_data_t *out, size_t n_max)
    {
        if(n_act_leg_list_sz == 0)
            return 0;
        size_t n_written = 0;
        size_t i_start = 1;

        if(act_leg_idx != -1 && act_leg_idx)
            i_start = size_t(act_leg_idx) - 1;

        for (size_t i = i_start; i < n_act_leg_list_sz-1; i++)
        {
            if (!n_max)
                return n_written;

            if(leg_list[i].data.is_discon)
                continue;

            nd_leg_data_t tmp;
            tmp.leg_data = leg_list[i].data.misc_data;
            tmp.arc_ctr = leg_list[i].data.leg.center_fix.data.pos;
            out[n_written] = tmp;

            n_max--;
            n_written++;
        }

        return n_written;
    }

    int FPLSys::get_act_leg_idx()
    {
        if(act_leg_idx == -1)
            return -1;
        return 1;
    }

    bool FPLSys::get_ctr(geo::point *out, bool fo_side)
    {
        size_t curr_idx = cap_ctr_idx;

        if (fo_side)
            curr_idx = fo_ctr_idx;

        if (curr_idx+1 < n_act_leg_list_sz)
        {
            bool has_pos = leg_list[curr_idx].data.misc_data.has_calc_wpt;
            if (has_pos)
            {
                *out = leg_list[curr_idx].data.misc_data.calc_wpt.data.pos;
                return true;
            }
        }

        return false;
    }

    geo::point FPLSys::get_ac_pos()
    {
        return {ac_lat_deg * geo::DEG_TO_RAD, ac_lon_deg * geo::DEG_TO_RAD};
    }

    hdg_info_t FPLSys::get_hdg_info()
    {
        hdg_info_t out = {};
        out.brng_tru_rad = ac_brng_deg * geo::DEG_TO_RAD;
        out.slip_rad = ac_slip_deg * geo::DEG_TO_RAD;
        out.magvar_rad = ac_magvar_deg * geo::DEG_TO_RAD;
        return out;
    }

    spd_info_t FPLSys::get_spd_info()
    {
        spd_info_t out = {};
        out.gs_kts = ac_gs_kts;
        out.tas_kts = ac_tas_kts;

        return out;
    }

    act_leg_info_t FPLSys::get_act_leg_info()
    {
        leg_seg_t act_seg = leg_list[act_leg_idx].data.misc_data;
        geo::point curr_pos = {ac_lat_deg * geo::DEG_TO_RAD, 
            ac_lon_deg * geo::DEG_TO_RAD};
        
        act_leg_info_t out;
        out.name = act_seg.calc_wpt.id;
        out.dist_sz = DIST_FONT_SZ_DD;
        double dist_nm = -1;
        if(act_seg.has_calc_wpt)
            dist_nm = curr_pos.get_gc_dist_nm(act_seg.end);

        if(dist_nm == -1)
        {
            out.dist_nm = "----";
        }
        else
        {
            uint8_t out_prec = 0;
            if(dist_nm < 10)
                out_prec = 1;

            if(dist_nm >= 100)
                out.dist_sz = DIST_FONT_SZ_TD;
            
            out.dist_nm = strutils::double_to_str(dist_nm, out_prec);
        }

        return out;
    }

    void FPLSys::step_ctr(bool bwd, bool fo_side)
    {
        if (!n_act_leg_list_sz)
            return;

        size_t *curr_idx = &cap_ctr_idx;

        if (fo_side)
            curr_idx = &fo_ctr_idx;

        if (bwd)
        {
            if ((*curr_idx)-1)
                *curr_idx = *curr_idx - 1;
            else
                *curr_idx = n_act_leg_list_sz-1;
            size_t curr_v = *curr_idx;

            while(leg_list[*curr_idx].data.is_discon || 
                !leg_list[*curr_idx].data.misc_data.has_calc_wpt)
            {
                if(*curr_idx)
                    *curr_idx = *curr_idx - 1;
                else
                    *curr_idx = n_act_leg_list_sz-1;
                if(*curr_idx == curr_v)
                    break;
            }
        }
        else
        {
            if (*curr_idx < n_act_leg_list_sz-1)
                *curr_idx = *curr_idx + 1;
            else
                *curr_idx = 1;
            size_t curr_v = *curr_idx;

            while(leg_list[*curr_idx].data.is_discon || 
                !leg_list[*curr_idx].data.misc_data.has_calc_wpt)
            {
                if (*curr_idx < n_act_leg_list_sz-1)
                    *curr_idx = *curr_idx + 1;
                else
                    *curr_idx = 1;
                if(*curr_idx == curr_v)
                    break;
            }
        }
    }

    void FPLSys::update()
    {
        update_pos();
        fpl->update(0);

        update_lists();
    }

    // Private member functions:

    void FPLSys::update_seg_list()
    {
        n_act_seg_list_sz = fpl->get_seg_list_sz();
        seg_list_id = fpl->get_sl_seg(0, n_act_seg_list_sz, &seg_list);
    }

    void FPLSys::update_leg_list()
    {
        n_act_leg_list_sz = fpl->get_leg_list_sz();
        leg_list_id = fpl->get_ll_seg(0, n_act_leg_list_sz, &leg_list, &act_leg_idx);

        if (cap_ctr_idx >= n_act_leg_list_sz && n_act_leg_list_sz != 0)
        {
            cap_ctr_idx = n_act_leg_list_sz - 1;
        }

        if (fo_ctr_idx >= n_act_leg_list_sz && n_act_leg_list_sz != 0)
        {
            fo_ctr_idx = n_act_leg_list_sz - 1;
        }
    }

    void FPLSys::update_lists()
    {
        double fpl_id_curr = fpl->get_id();
        if (fpl_id_curr != fpl_id_last)
        {
            update_seg_list();
            update_leg_list();
        }

        fpl_id_last = fpl_id_curr;
    }

    void FPLSys::update_pos()
    {
        for(size_t i = 0; i < RSV_VARS.size(); i++)
        {
            if(!strutils::is_numeric(env_vars[RSV_VARS[i]]))
                return;
        }

        ac_lat_deg = strutils::strtod(env_vars[AC_LAT_DEG_VAR]);
        ac_lon_deg = strutils::strtod(env_vars[AC_LON_DEG_VAR]);
        ac_brng_deg = strutils::strtod(env_vars[AC_BRNG_TRU_DEG_VAR]);
        ac_slip_deg = strutils::strtod(env_vars[AC_SLIP_DEG_VAR]);
        ac_magvar_deg = strutils::strtod(env_vars[AC_MAGVAR_DEG_VAR]);
        ac_gs_kts = strutils::strtod(env_vars[AC_GS_KTS_VAR]);
        ac_tas_kts = strutils::strtod(env_vars[AC_TAS_KTS_VAR]);
    }
}
