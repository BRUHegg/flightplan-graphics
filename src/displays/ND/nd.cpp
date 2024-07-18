/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This source file contains definitions of classes, functions, etc 
	used in the ND implementation. Author: discord/bruh4096#4512
*/

#include "nd.hpp"


namespace StratosphereAvionics
{
    // NDData member funcrion definitions

    // Public member functions:

    NDData::NDData(std::shared_ptr<test::FPLSys> fpl_sys)
    {
        m_fpl_sys_ptr = fpl_sys;
        m_fpl_ptr = fpl_sys->fpl;

        m_leg_data = new test::nd_leg_data_t[N_LEG_PROJ_CACHE_SZ];

        m_proj_legs_cap = new leg_proj_t[N_LEG_PROJ_CACHE_SZ];
        m_proj_legs_fo = new leg_proj_t[N_LEG_PROJ_CACHE_SZ];

        m_n_act_proj_legs_cap = 0;
        m_n_act_proj_legs_fo = 0;

        m_fpl_id_last = 0;
    }

    size_t NDData::get_proj_legs(leg_proj_t **out, bool fo_side)
    {
        if(fo_side)
        {
            *out = m_proj_legs_fo;
            return m_n_act_proj_legs_fo;
        }

        *out = m_proj_legs_cap;
        return m_n_act_proj_legs_cap;
    }

    void NDData::update()
    {
        double id_curr = m_fpl_ptr->get_id();

        if(id_curr != m_fpl_id_last)
        {
            fetch_legs();
        }

        m_fpl_id_last = id_curr;
    }

    NDData::~NDData()
    {
        delete[] m_leg_data;
        delete[] m_proj_legs_cap;
        delete[] m_proj_legs_fo;
    }

    // Private member functions:

    void NDData::update_ctr(geo::point *ctr, bool fo_side)
    {
        bool ret = m_fpl_sys_ptr->get_ctr(ctr, fo_side);
        if(!ret)
        {
            *ctr = m_fpl_sys_ptr->get_ac_pos();
        }
    }

    void NDData::project_legs(bool fo_side)
    {
        geo::point map_ctr = m_ctr_cap;
        if(fo_side)
            map_ctr = m_ctr_fo;
        
        leg_proj_t *dst = m_proj_legs_cap;

        if(fo_side)
            dst = m_proj_legs_fo;

        size_t *sz_ptr = &m_n_act_proj_legs_cap;
        if(fo_side)
            sz_ptr = &m_n_act_proj_legs_fo;

        *sz_ptr = 0;

        bool out_of_rng = false;
        for(size_t i = 0; i < m_n_act_leg_data; i++)
        {
            if(i >= N_LEG_PROJ_CACHE_SZ)
                break;

            if(m_leg_data[i].leg_data.turn_rad_nm == -1 || 
                !m_leg_data[i].leg_data.is_finite || m_leg_data[i].leg_data.is_arc)
                continue;
            
            double dist_start = map_ctr.get_gc_dist_nm(m_leg_data[i].leg_data.start);
            double dist_end = map_ctr.get_gc_dist_nm(m_leg_data[i].leg_data.end);

            if(dist_start > N_MAX_DIST_NM && dist_end > N_MAX_DIST_NM)
            {
                if(out_of_rng)
                    continue;
                else
                    out_of_rng = true;
            }

            double brng_start = map_ctr.get_gc_bearing_rad(m_leg_data[i].leg_data.start);
            double brng_end = map_ctr.get_gc_bearing_rad(m_leg_data[i].leg_data.end);

            dst[i].start = {dist_start * sin(brng_start), dist_start * cos(brng_start)};
            dst[i].end = {dist_end * sin(brng_end), dist_end * cos(brng_end)};

            dst[i].is_arc = false;
            dst[i].is_finite = true;
            dst[i].turn_rad_nm = m_leg_data[i].leg_data.turn_rad_nm;
            dst[i].end_nm = m_leg_data[i].end_name;

            *sz_ptr = *sz_ptr + 1;
        }
    }

    void NDData::fetch_legs()
    {
        update_ctr(&m_ctr_cap, false);
        update_ctr(&m_ctr_fo, true);
        
        m_n_act_leg_data = m_fpl_sys_ptr->get_nd_seg(m_leg_data, N_LEG_PROJ_CACHE_SZ);

        project_legs(false);
        project_legs(true);
    }
} // namespace StratosphereAvionics
