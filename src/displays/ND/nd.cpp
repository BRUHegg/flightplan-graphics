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
    geom::vect2_t get_projection(double brng_rad, double dist_nm)
    {
        return {dist_nm * sin(brng_rad), dist_nm * cos(brng_rad)};
    }

    geom::vect2_t project_point(geo::point tgt, geo::point p_ctr)
    {
        double brng_rad = p_ctr.get_gc_bearing_rad(tgt);
        double dist_nm = p_ctr.get_gc_dist_nm(tgt);

        return get_projection(brng_rad, dist_nm);
    }

    // NDData member funcrion definitions:

    // Public member functions:

    NDData::NDData(std::shared_ptr<test::FPLSys> fpl_sys)
    {
        m_fpl_sys_ptr = fpl_sys;
        m_fpl_ptr = fpl_sys->fpl;

        m_leg_data = new test::nd_leg_data_t[N_LEG_PROJ_CACHE_SZ];

        m_proj_legs_cap = new leg_proj_t[N_PROJ_CACHE_SZ];
        m_proj_legs_fo = new leg_proj_t[N_PROJ_CACHE_SZ];

        m_n_act_proj_legs_cap = 0;
        m_n_act_proj_legs_fo = 0;

        m_rng_idx_cap = 0;
        m_rng_idx_fo = 0;

        m_fpl_id_last = 0;

        m_has_dep_rwy = false;
        m_has_arr_rwy = false;
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

    bool NDData::has_dep_rwy()
    {
        return m_has_dep_rwy;
    }

    bool NDData::has_arr_rwy()
    {
        return m_has_arr_rwy;
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

        project_rwys(false);
        project_rwys(true);

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

    bool NDData::in_view(geom::vect2_t start, geom::vect2_t end, bool fo_side)
    {
        double a = start.x - end.x;
        double rng = get_range(fo_side)/2;

        if(a != 0)
        {
            double b = start.y - end.y;
            double k = b / a;
            double c = start.y - start.x * k;
            double y1 = k * -rng + c;
            double y2 = k * rng + c;

            if((y1 < rng && y2 >= rng) || (y1 >= rng && y2 < rng) || 
                (y1 > -rng && y2 <= -rng) || (y2 > -rng && y1 <= -rng) ||
                (abs(y1) <= rng && abs(y2) <= rng))
                return true;
        }
        else
        {
            if(abs(start.x) <= rng)
            {
                bool out_of_bounds = (start.y > rng && end.y > rng) ||
                    (start.y < -rng && end.y < -rng);

                return !out_of_bounds;
            }
        }

        return false;
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

        for(size_t i = 0; i < m_n_act_leg_data; i++)
        {
            if(i >= N_LEG_PROJ_CACHE_SZ)
                break;

            if(m_leg_data[i].leg_data.turn_rad_nm == -1 || 
                !m_leg_data[i].leg_data.is_finite || m_leg_data[i].leg_data.is_arc)
                continue;

            if(m_leg_data[i].leg_data.is_finite)
            {
                double dist_wpt = map_ctr.get_gc_dist_nm(m_leg_data[i].end_wpt);
                double brng_wpt = map_ctr.get_gc_bearing_rad(m_leg_data[i].end_wpt);

                dst[*sz_ptr].end_wpt = {dist_wpt * sin(brng_wpt), dist_wpt * cos(brng_wpt)};
            }
            
            geom::vect2_t start_proj = project_point(m_leg_data[i].leg_data.start, 
                map_ctr);
            geom::vect2_t end_proj = project_point(m_leg_data[i].leg_data.end, 
                map_ctr);

            if(!in_view(start_proj, end_proj, fo_side))
                continue;

            dst[*sz_ptr].start = start_proj;
            dst[*sz_ptr].end = end_proj;

            dst[*sz_ptr].is_arc = false;
            dst[*sz_ptr].is_finite = true;
            dst[*sz_ptr].is_rwy = m_leg_data[i].leg_data.is_rwy;
            dst[*sz_ptr].turn_rad_nm = m_leg_data[i].leg_data.turn_rad_nm;
            dst[*sz_ptr].end_nm = m_leg_data[i].end_name;

            *sz_ptr = *sz_ptr + 1;
        }
    }

    void NDData::project_rwys(bool fo_side)
    {
        std::string dep_rwy = m_fpl_ptr->get_dep_rwy();
        std::string arr_rwy = m_fpl_ptr->get_arr_rwy();

        m_has_dep_rwy = dep_rwy != "";
        m_has_arr_rwy = arr_rwy != "";

        if(!m_has_dep_rwy && !m_has_arr_rwy)
            return;

        geo::point map_ctr = m_ctr_cap;
        if(fo_side)
            map_ctr = m_ctr_fo;
        
        leg_proj_t *dst = m_proj_legs_cap;

        if(fo_side)
            dst = m_proj_legs_fo;

        if(m_has_dep_rwy)
        {
            libnav::runway_entry_t rnw_data;
            bool has_data = m_fpl_ptr->get_dep_rwy_data(&rnw_data);

            if(has_data)
            {
                dst[DEP_RWY_PROJ_IDX].start = project_point(rnw_data.start, map_ctr);
                dst[DEP_RWY_PROJ_IDX].end = project_point(rnw_data.end, map_ctr);
            }
        }

        if(m_has_arr_rwy)
        {
            libnav::runway_entry_t rnw_data;
            bool has_data = m_fpl_ptr->get_arr_rwy_data(&rnw_data);

            if(has_data)
            {
                dst[ARR_RWY_PROJ_IDX].start = project_point(rnw_data.start, map_ctr);
                dst[ARR_RWY_PROJ_IDX].end = project_point(rnw_data.end, map_ctr);
            }
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

        draw_runways(cr);
        draw_flight_plan(cr, false);
        draw_flight_plan(cr, true);
    }

    // Private member functions:

    geom::vect2_t NDDisplay::get_screen_coords(geom::vect2_t src, geom::vect2_t map_ctr, 
        geom::vect2_t sc)
    {
        geom::vect2_t out = src * sc + map_ctr;
        out.y = size.y - out.y;

        return out;
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

                    geom::vect2_t s_trans = get_screen_coords(start, map_ctr, s_fac);
                    geom::vect2_t e_trans = get_screen_coords(end, map_ctr, s_fac);

                    cairo_utils::draw_line(cr, s_trans, e_trans, 
                        cairo_utils::MAGENTA, ND_FPL_LINE_THICK * size.x);
                }
                else
                {
                    geom::vect2_t end_wpt = buf[i].end_wpt;

                    geom::vect2_t ew_trans = get_screen_coords(end_wpt, map_ctr, s_fac);

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

    void NDDisplay::draw_runway(cairo_t *cr, leg_proj_t rnw_proj)
    {
        double curr_rng = rng / 2;
        geom::vect2_t map_ctr = scr_pos + size.scmul(0.5);
        geom::vect2_t s_fac = size.scmul(ND_RNG_FULL_RES_COEFF).scdiv(curr_rng);
        
        geom::vect2_t start_trans = get_screen_coords(rnw_proj.start, map_ctr, s_fac);
        geom::vect2_t end_trans = get_screen_coords(rnw_proj.end, map_ctr, s_fac);

        geom::vect2_t r_proj_nml_vec = {
            end_trans.y - start_trans.y,
            start_trans.x - end_trans.x
            };
        r_proj_nml_vec = r_proj_nml_vec.get_unit();
        
        double half_width = size.x * DEFAULT_RWY_WIDTH / 2;
        geom::vect2_t l_side_start = start_trans + r_proj_nml_vec.scmul(
            half_width);
        geom::vect2_t l_side_end = end_trans + r_proj_nml_vec.scmul(
            half_width);

        geom::vect2_t r_side_start = start_trans + r_proj_nml_vec.scmul(
            half_width * -1);
        geom::vect2_t r_side_end = end_trans + r_proj_nml_vec.scmul(
            half_width * -1);

        cairo_utils::draw_line(cr, l_side_start, l_side_end, 
            cairo_utils::WHITE, RWY_SIDE_THICK * size.x);

        cairo_utils::draw_line(cr, r_side_start, r_side_end, 
            cairo_utils::WHITE, RWY_SIDE_THICK * size.x);
    }

    void NDDisplay::draw_runways(cairo_t *cr)
    {
        leg_proj_t *buf;
        size_t buf_size = nd_data->get_proj_legs(&buf, fo_side);
        UNUSED(buf_size);

        if(nd_data->has_dep_rwy())
        {
            draw_runway(cr, buf[DEP_RWY_PROJ_IDX]);
        }
        
        if(nd_data->has_arr_rwy())
        {
            draw_runway(cr, buf[ARR_RWY_PROJ_IDX]);
        }
    }
} // namespace StratosphereAvionics
