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
    // NDData member funcrion definitions:

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

        m_rng_idx_cap = 0;
        m_rng_idx_fo = 0;

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

    void NDData::switch_range(bool down, bool fo_side)
    {
        size_t *tgt = &m_rng_idx_cap;

        if(fo_side)
            tgt = &m_rng_idx_fo;

        if(down)
        {
            if(*tgt)
                *tgt = *tgt - 1;
        }
        else
        {
            if(*tgt + 1 < ND_RANGES_NM.size())
                *tgt = *tgt + 1;
        }
    }

    double NDData::get_range(bool fo_side)
    {
        if(fo_side)
            return ND_RANGES_NM[m_rng_idx_fo];
        else
            return ND_RANGES_NM[m_rng_idx_cap];
    }

    void NDData::update()
    {
        double id_curr = m_fpl_ptr->get_id();

        if(id_curr != m_fpl_id_last)
        {
            fetch_legs();
        }

        update_ctr(&m_ctr_cap, false);
        update_ctr(&m_ctr_fo, true);

        project_legs(false);
        project_legs(true);

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

            if(m_leg_data[i].leg_data.is_finite)
            {
                double dist_wpt = map_ctr.get_gc_dist_nm(m_leg_data[i].end_wpt);
                double brng_wpt = map_ctr.get_gc_bearing_rad(m_leg_data[i].end_wpt);

                dst[i].end_wpt = {dist_wpt * sin(brng_wpt), dist_wpt * cos(brng_wpt)};
            }
            

            dst[i].start = {dist_start * sin(brng_start), dist_start * cos(brng_start)};
            dst[i].end = {dist_end * sin(brng_end), dist_end * cos(brng_end)};

            dst[i].is_arc = false;
            dst[i].is_finite = true;
            dst[i].is_rwy = m_leg_data[i].leg_data.is_rwy;
            dst[i].turn_rad_nm = m_leg_data[i].leg_data.turn_rad_nm;
            dst[i].end_nm = m_leg_data[i].end_name;

            *sz_ptr = *sz_ptr + 1;
        }
    }

    void NDData::fetch_legs()
    {
        m_n_act_leg_data = m_fpl_sys_ptr->get_nd_seg(m_leg_data, N_LEG_PROJ_CACHE_SZ);
    }

    // NDDisplay member functions:

    // Public member functions:

    NDDisplay::NDDisplay(std::shared_ptr<NDData> data, 
        std::shared_ptr<cairo_utils::texture_manager_t> mngr,
        cairo_font_face_t *ff, geom::vect2_t pos, geom::vect2_t sz, bool fo_sd)
    {
        nd_data = data;
        tex_mngr = mngr;

        font_face = ff;

        scr_pos = pos;
        size = sz;

        fo_side = fo_sd;

        rng = nd_data->get_range(fo_side);
    }

    void NDDisplay::draw(cairo_t *cr)
    {
        rng = nd_data->get_range(fo_side);

        cairo_utils::draw_rect(cr, scr_pos, size, cairo_utils::DARK_BLUE);

        draw_flight_plan(cr, false);
        draw_flight_plan(cr, true);
    }

    void NDDisplay::draw_flight_plan(cairo_t *cr, bool draw_labels)
    {
        leg_proj_t *buf;
        size_t buf_size = nd_data->get_proj_legs(&buf, fo_side);

        double curr_rng = rng / 2;
        geom::vect2_t map_ctr = scr_pos + size.scmul(0.5);
        geom::vect2_t s_fac = size.scmul(ND_RNG_FULL_RES_COEFF).scdiv(curr_rng);

        for(size_t i = 0; i < buf_size; i++)
        {
            if(buf[i].is_finite && !buf[i].is_arc)
            {
                if(!draw_labels)
                {
                    geom::vect2_t start = buf[i].start;
                    geom::vect2_t end = buf[i].end;

                    geom::vect2_t s_trans = start * s_fac + map_ctr;
                    geom::vect2_t e_trans = end * s_fac + map_ctr;
                    s_trans.y = size.y - s_trans.y;
                    e_trans.y = size.y - e_trans.y;
                    cairo_utils::draw_line(cr, s_trans, e_trans, 
                        cairo_utils::MAGENTA, ND_FPL_LINE_THICK);
                }
                else
                {
                    geom::vect2_t end_wpt = buf[i].end_wpt;

                    geom::vect2_t ew_trans = end_wpt * s_fac + map_ctr;
                    ew_trans.y = size.y - ew_trans.y;
                    geom::vect2_t text_pos = ew_trans + size * FIX_NAME_OFFS;
                    cairo_utils::draw_left_text(cr, font_face, buf[i].end_nm, text_pos, 
                        cairo_utils::WHITE, ND_WPT_FONT_SZ);

                    if(!buf[i].is_rwy)
                        cairo_utils::draw_image(cr, tex_mngr->data[WPT_INACT_NAME], ew_trans, 
                            true);
                }
            }
        }
    }
} // namespace StratosphereAvionics
