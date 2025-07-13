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
    // leg_proj_t definitions:

    std::string leg_proj_t::get_draw_nm()
    {
        std::string name_draw;
        for (size_t j = 0; j < end_nm.size(); j++)
        {
            char curr_char = end_nm[j];
            if (curr_char != '(' && curr_char != ')')
            {
                name_draw.push_back(curr_char);
            }
        }

        return name_draw;
    }

    void map_data_t::create()
    {
        proj_legs = new leg_proj_t[N_PROJ_CACHE_SZ];
        line_joints = new geom::line_joint_t[N_LN_JOINT_CACHE_SZ];
        n_act_proj_legs = 0;
        n_act_joints = 0;
    }

    void map_data_t::destroy()
    {
        delete[] proj_legs;
        delete[] line_joints;
        n_act_proj_legs = 0;
        n_act_joints = 0;
    }

    // NDData member funcrion definitions:

    // Public member functions:

    NDData::NDData(std::shared_ptr<test::FPLSys> fpl_sys)
    {
        m_fpl_sys_ptr = fpl_sys;
        m_fpl_ptr = fpl_sys->fpl_vec[test::ACT_RTE_IDX];

        m_leg_data = new test::nd_leg_data_t[N_LEG_PROJ_CACHE_SZ];

        m_mp_data = std::vector<map_data_t>(N_MP_DATA_SZ);
        for(size_t i = 0; i < N_MP_DATA_SZ; i++)
        {
            m_mp_data[i].create();
        }

        m_ac_pos_proj_cap = {};
        m_ac_pos_proj_fo = {};

        m_rng_idx_cap = 0;
        m_rng_idx_fo = 0;

        m_fpl_id_last = 0;

        m_hdg_data = {};

        m_has_dep_rwy = false;
        m_has_arr_rwy = false;
        m_ac_pos_ok_cap = false;
        m_ac_pos_ok_fo = false;

        m_act_leg_idx = -1;
        m_act_leg_idx_cap = -1;
        m_act_leg_idx_fo = -1;
    }

    size_t NDData::get_proj_legs(leg_proj_t **out, bool fo_side)
    {
        *out = m_mp_data[fo_side].proj_legs;
        return m_mp_data[fo_side].n_act_proj_legs;
    }

    int NDData::get_act_leg_idx(bool fo_side)
    {
        if(fo_side)
            return m_act_leg_idx_fo;
        return m_act_leg_idx_cap;
    }

    bool NDData::get_ac_pos(geom::vect2_t *out, bool fo_side)
    {
        bool is_ok = m_ac_pos_ok_cap;
        if(fo_side)
            is_ok = m_ac_pos_ok_fo;
        
        if(!is_ok)
            return false;
        
        geom::vect2_t src = m_ac_pos_proj_cap;
        if(fo_side)
            src = m_ac_pos_proj_fo;

        *out = src;
        return true;
    }

    test::hdg_info_t NDData::get_hdg_data()
    {
        return m_hdg_data;
    }

    test::spd_info_t NDData::get_spd_data()
    {
        return m_fpl_sys_ptr->get_spd_info();
    }

    test::act_leg_info_t NDData::get_act_leg_info()
    {
        return m_fpl_sys_ptr->get_act_leg_info();
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

        if (fo_side)
            tgt = &m_rng_idx_fo;

        if (down)
        {
            if (*tgt)
                *tgt = *tgt - 1;
        }
        else
        {
            if (*tgt + 1 < ND_RANGES_NM.size())
                *tgt = *tgt + 1;
        }
    }

    double NDData::get_range(bool fo_side)
    {
        if (fo_side)
            return ND_RANGES_NM[m_rng_idx_fo];
        else
            return ND_RANGES_NM[m_rng_idx_cap];
    }

    void NDData::update()
    {
        double id_curr = m_fpl_ptr->get_id();

        if (id_curr != m_fpl_id_last)
        {
            fetch_legs();
        }

        m_hdg_data = m_fpl_sys_ptr->get_hdg_info();

        update_ctr(&m_ctr_cap, false);
        update_ctr(&m_ctr_fo, true);

        project_legs(false);
        project_legs(true);

        project_rwys(false);
        project_rwys(true);

        m_ac_pos_ok_cap = project_ac_pos(false);
        m_ac_pos_ok_fo = project_ac_pos(true);

        m_fpl_id_last = id_curr;
    }

    NDData::~NDData()
    {
        delete[] m_leg_data;
        for(size_t i = 0; i < N_MP_DATA_SZ; i++)
        {
            m_mp_data[i].destroy();
        }
    }

    // Private member functions:

    void NDData::update_ctr(geo::point *ctr, bool fo_side)
    {
        bool ret = m_fpl_sys_ptr->get_ctr(ctr, fo_side);
        if (!ret)
        {
            *ctr = m_fpl_sys_ptr->get_ac_pos();
        }
    }

    bool NDData::bound_check(double x1, double x2, double rng)
    {
        return (x1 < rng && x2 >= rng) || (x1 >= rng && x2 < rng) ||
               (x1 > -rng && x2 <= -rng) || (x2 > -rng && x1 <= -rng) ||
               (abs(x1) <= rng && abs(x2) <= rng);
    }

    bool NDData::in_view(geom::vect2_t start, geom::vect2_t end, bool fo_side)
    {
        double a = start.x - end.x;
        double rng = get_range(fo_side) / 2;

        if (a != 0)
        {
            double b = start.y - end.y;
            double k = b / a;
            double c = start.y - start.x * k;
            double y1 = k * -rng + c;
            double y2 = k * rng + c;

            if (bound_check(y1, y2, rng) && bound_check(start.y, end.y, rng) &&
                bound_check(start.x, end.x, rng))
            {
                return true;
            }
        }
        else
        {
            if (abs(start.x) <= rng)
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
        if (fo_side)
            map_ctr = m_ctr_fo;

        leg_proj_t *dst = m_mp_data[fo_side].proj_legs;
        geom::line_joint_t *dst_joint = m_mp_data[fo_side].line_joints;

        size_t *sz_ptr = &m_mp_data[fo_side].n_act_proj_legs;
        size_t *sz_ptr_joint = &m_mp_data[fo_side].n_act_joints;
        int *act_idx_ptr = &m_act_leg_idx_cap;
        if (fo_side)
        {
            act_idx_ptr = &m_act_leg_idx_fo;
        }

        *sz_ptr = 0;
        *sz_ptr_joint = 0;
        *act_idx_ptr = -1;

        bool prev_skipped = false;
        bool prev_bypassed = false;

        for (size_t i = 0; i < m_n_act_leg_data; i++)
        {
            if (i >= N_LEG_PROJ_CACHE_SZ)
                break;

            dst[*sz_ptr].joint = nullptr;

            if ((!m_leg_data[i].leg_data.is_finite && !m_leg_data[i].leg_data.is_bypassed) ||
                m_leg_data[i].leg_data.is_arc)
                continue;

            if (m_leg_data[i].leg_data.is_finite && m_leg_data[i].leg_data.has_calc_wpt)
            {
                geo::point end_wpt = m_leg_data[i].leg_data.calc_wpt.data.pos;
                std::string end_name = m_leg_data[i].leg_data.calc_wpt.id;

                double dist_wpt = map_ctr.get_gc_dist_nm(end_wpt);
                double brng_wpt = map_ctr.get_gc_bearing_rad(end_wpt);

                dst[*sz_ptr].end_wpt = {dist_wpt * sin(brng_wpt), dist_wpt * cos(brng_wpt)};
                dst[*sz_ptr].end_nm = end_name;
                dst[*sz_ptr].is_finite = true;
                dst[*sz_ptr].is_arc = m_leg_data[i].leg_data.is_arc;
                dst[*sz_ptr].has_path = false;
            }

            if (m_leg_data[i].leg_data.turn_rad_nm != -1)
            {
                geom::vect2_t start_proj = geom::project_point(m_leg_data[i].leg_data.start,
                                                               map_ctr);
                geom::vect2_t end_proj = geom::project_point(m_leg_data[i].leg_data.end,
                                                             map_ctr);

                if (!in_view(start_proj, end_proj, fo_side))
                {
                    prev_skipped = true;
                    continue;
                }

                if (*sz_ptr && !prev_skipped && !m_leg_data[i].leg_data.has_disc)
                {
                    size_t bwd_offs = 1;
                    if(prev_bypassed)
                        bwd_offs++;
                    geom::vect2_t prev_start = dst[*sz_ptr - bwd_offs].start;
                    geom::vect2_t prev_end = dst[*sz_ptr - bwd_offs].end;
                    double radius_nm = dst[*sz_ptr - bwd_offs].turn_rad_nm;
                    dst_joint[*sz_ptr_joint] = geom::get_line_joint(prev_start, prev_end,
                                                                    start_proj, end_proj, 
                                                                    radius_nm);

                    dst[*sz_ptr].joint = &dst_joint[*sz_ptr_joint];

                    *sz_ptr_joint = *sz_ptr_joint + 1;
                }

                dst[*sz_ptr].start = start_proj;
                dst[*sz_ptr].end = end_proj;
                dst[*sz_ptr].has_path = !m_leg_data[i].leg_data.is_bypassed;

                dst[*sz_ptr].is_rwy = m_leg_data[i].leg_data.is_rwy;
                dst[*sz_ptr].turn_rad_nm = m_leg_data[i].leg_data.turn_rad_nm;

                if(m_act_leg_idx != -1 && i == size_t(m_act_leg_idx))
                {
                    *act_idx_ptr = int(*sz_ptr);
                }

                *sz_ptr = *sz_ptr + 1;
                prev_skipped = false;
                prev_bypassed = false;
            }
            else if (m_leg_data[i].leg_data.is_finite)
            {
                *sz_ptr = *sz_ptr + 1;
                prev_skipped = !m_leg_data[i].leg_data.is_bypassed;
            }
            prev_bypassed = m_leg_data[i].leg_data.turn_rad_nm == -1;
        }
    }

    void NDData::project_rwys(bool fo_side)
    {
        std::string dep_rwy = m_fpl_ptr->get_dep_rwy();
        std::string arr_rwy = m_fpl_ptr->get_arr_rwy();

        m_has_dep_rwy = dep_rwy != "";
        m_has_arr_rwy = arr_rwy != "";

        if (!m_has_dep_rwy && !m_has_arr_rwy)
            return;

        geo::point map_ctr = m_ctr_cap;
        if (fo_side)
            map_ctr = m_ctr_fo;

        leg_proj_t *dst = m_mp_data[fo_side].proj_legs;

        if (m_has_dep_rwy)
        {
            libnav::runway_entry_t rnw_data;
            bool has_data = m_fpl_ptr->get_dep_rwy_data(&rnw_data);

            if (has_data)
            {
                dst[DEP_RWY_PROJ_IDX].start = geom::project_point(rnw_data.start, map_ctr);
                dst[DEP_RWY_PROJ_IDX].end = geom::project_point(rnw_data.end, map_ctr);
            }
        }

        if (m_has_arr_rwy)
        {
            libnav::runway_entry_t rnw_data;
            bool has_data = m_fpl_ptr->get_arr_rwy_data(&rnw_data);

            if (has_data)
            {
                dst[ARR_RWY_PROJ_IDX].start = geom::project_point(rnw_data.start, map_ctr);
                dst[ARR_RWY_PROJ_IDX].end = geom::project_point(rnw_data.end, map_ctr);
            }
        }
    }

    bool NDData::project_ac_pos(bool fo_side)
    {
        geo::point map_ctr = m_ctr_cap;
        if (fo_side)
            map_ctr = m_ctr_fo;

        geom::vect2_t *dst = &m_ac_pos_proj_cap;
        if(fo_side)
            dst = &m_ac_pos_proj_fo;

        geo::point curr_pos = m_fpl_sys_ptr->get_ac_pos();
        
        double gc_dist_nm = map_ctr.get_gc_dist_nm(curr_pos);
        double rng = get_range(fo_side) / 2;

        if(rng < gc_dist_nm)
            return false;

        double gc_brng_rad = map_ctr.get_gc_bearing_rad(curr_pos);
        *dst = geom::get_projection(gc_brng_rad, gc_dist_nm);

        return true;
    }

    void NDData::fetch_legs()
    {
        m_n_act_leg_data = m_fpl_sys_ptr->get_nd_seg(m_leg_data, N_LEG_PROJ_CACHE_SZ);
        m_act_leg_idx = m_fpl_sys_ptr->get_act_leg_idx();
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
        update_map_params();
        hdg_data = nd_data->get_hdg_data();

        cairo_utils::draw_rect(cr, scr_pos, size, cairo_utils::DARK_BLUE);

        draw_background(cr, true);

        draw_runways(cr);
        draw_flight_plan(cr, false);
        draw_flight_plan(cr, true);
        draw_airplane(cr);

        draw_background(cr, false);
        draw_act_leg_info(cr);
        draw_spd_info(cr);
        draw_range(cr);
    }

    // Private member functions:

    void NDDisplay::update_map_params()
    {
        curr_rng = rng / 2;
        map_ctr = size.scmul(0.5);
        map_ctr.y += size.y * 0.01;
        scale_factor = size.scmul(ND_RNG_FULL_RES_COEFF).scdiv(curr_rng);
    }

    geom::vect2_t NDDisplay::get_screen_coords(geom::vect2_t src)
    {
        src.y *= -1;
        geom::vect2_t out = src * scale_factor + map_ctr + scr_pos;

        return out;
    }

    void NDDisplay::draw_line_joint(cairo_t *cr, geom::line_joint_t lj)
    {
        double radius_px = lj.turn_radius * scale_factor.x;
        if (lj.tp == geom::JointType::CIRC_CIRC)
        {
            geom::vect2_t arc1_trans = get_screen_coords(lj.arc1.pos);
            geom::vect2_t arc2_trans = get_screen_coords(lj.arc2.pos);

            cairo_utils::draw_arc(cr, arc1_trans, radius_px, lj.arc1.ang_start_rad,
                                  lj.arc1.ang_end_rad, ND_FPL_LINE_THICK * size.x, 
                                    cairo_utils::MAGENTA);
            cairo_utils::draw_arc(cr, arc2_trans, radius_px, lj.arc2.ang_start_rad,
                                  lj.arc2.ang_end_rad, ND_FPL_LINE_THICK * size.x, 
                                  cairo_utils::MAGENTA);
        }
        else if (lj.tp == geom::JointType::CIRC)
        {
            geom::vect2_t arc1_trans = get_screen_coords(lj.arc1.pos);
            cairo_utils::draw_arc(cr, arc1_trans, radius_px, lj.arc1.ang_start_rad,
                                  lj.arc1.ang_end_rad, ND_FPL_LINE_THICK * size.x, 
                                  cairo_utils::MAGENTA);
        }

        geom::vect2_t s_trans = get_screen_coords(lj.line.start);
        geom::vect2_t e_trans = get_screen_coords(lj.line.end);

        cairo_utils::draw_line(cr, s_trans, e_trans,
                               cairo_utils::MAGENTA, ND_FPL_LINE_THICK * size.x);
    }

    void NDDisplay::draw_flight_plan(cairo_t *cr, bool draw_labels)
    {
        leg_proj_t *buf;
        size_t buf_size = nd_data->get_proj_legs(&buf, fo_side);
        int act_leg_idx = nd_data->get_act_leg_idx(fo_side);

        for (size_t i = 0; i < buf_size; i++)
        {
            if (buf[i].is_finite && !buf[i].is_arc)
            {
                if (!draw_labels)
                {
                    geom::vect2_t start = buf[i].start;
                    geom::vect2_t end = buf[i].end;

                    if(buf[i].joint != nullptr)
                    {
                        geom::line_joint_t curr_joint = *buf[i].joint;

                        if(curr_joint.tp != geom::JointType::LINE)
                            start = curr_joint.line.start;

                        draw_line_joint(cr, curr_joint);
                    }

                    if(buf[i].has_path)
                    {
                        geom::vect2_t s_trans = get_screen_coords(start);
                        geom::vect2_t e_trans = get_screen_coords(end);
                        
                        cairo_utils::draw_line(cr, s_trans, e_trans,
                                            cairo_utils::MAGENTA, ND_FPL_LINE_THICK * size.x);
                    }
                }
                else if (buf[i].end_nm.size() && draw_labels)
                {
                    geom::vect2_t end_wpt = buf[i].end_wpt;

                    geom::vect2_t ew_trans = get_screen_coords(end_wpt);

                    geom::vect2_t text_pos = ew_trans + size * FIX_NAME_OFFS;

                    std::string name_draw = buf[i].get_draw_nm();

                    bool is_active = false;
                    geom::vect3_t tgt_color = cairo_utils::WHITE;

                    if(act_leg_idx != -1 && i == size_t(act_leg_idx))
                    {
                        is_active = true;
                        tgt_color = cairo_utils::MAGENTA;
                    }

                    cairo_utils::draw_left_text(cr, font_face, name_draw, text_pos,
                                                tgt_color, ND_WPT_FONT_SZ);

                    if (!buf[i].is_rwy && buf[i].end_nm[0] != '(')
                    {
                        geom::vect2_t scale = size.scmul(1 / WPT_SCALE_FACT);
                        if(is_active)
                        {
                            cairo_utils::draw_image(cr, tex_mngr->data[WPT_ACT_NAME],
                                                ew_trans, scale, true);
                        }
                        else
                        {
                            cairo_utils::draw_image(cr, tex_mngr->data[WPT_INACT_NAME],
                                                ew_trans, scale, true);
                        }
                    }
                    else if (!buf[i].is_rwy)
                    {
                        cairo_utils::draw_circle(cr, ew_trans,
                                                 size.x * PSEUDO_WPT_RADIUS_RAT, size.x * PSEUDO_WPT_THICK_RAT,
                                                 tgt_color);
                    }
                }
            }
        }
    }

    void NDDisplay::draw_ext_rwy_ctr_line(cairo_t *cr, leg_proj_t rnw_proj)
    {
        geom::vect2_t rwy_vec = {rnw_proj.start.x - rnw_proj.end.x,
                                 rnw_proj.start.y - rnw_proj.end.y};
        rwy_vec = rwy_vec.get_unit();

        geom::vect2_t end1 = rnw_proj.start + rwy_vec.scmul(N_RWY_EXT_CTR_LINE_NM);
        geom::vect2_t end2 = rnw_proj.end + rwy_vec.scmul(-N_RWY_EXT_CTR_LINE_NM);

        geom::vect2_t rwy_start_trans = get_screen_coords(rnw_proj.start);
        geom::vect2_t rwy_end_trans = get_screen_coords(rnw_proj.end);

        geom::vect2_t end1_trans = get_screen_coords(end1);
        geom::vect2_t end2_trans = get_screen_coords(end2);

        cairo_save(cr);
        cairo_set_dash(cr, RWY_EXT_CTR_LINE_DASH, N_RWY_DASHES, 1);
        cairo_utils::draw_line(cr, end1_trans, rwy_start_trans,
                               cairo_utils::WHITE, RWY_SIDE_THICK * size.x);
        cairo_utils::draw_line(cr, end2_trans, rwy_end_trans,
                               cairo_utils::WHITE, RWY_SIDE_THICK * size.x);
        cairo_restore(cr);
    }

    void NDDisplay::draw_runway(cairo_t *cr, leg_proj_t rnw_proj)
    {
        geom::vect2_t start_trans = get_screen_coords(rnw_proj.start);
        geom::vect2_t end_trans = get_screen_coords(rnw_proj.end);

        draw_ext_rwy_ctr_line(cr, rnw_proj);

        geom::vect2_t r_proj_nml_vec = {
            end_trans.y - start_trans.y,
            start_trans.x - end_trans.x};
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

        if (nd_data->has_dep_rwy())
        {
            draw_runway(cr, buf[DEP_RWY_PROJ_IDX]);
        }

        if (nd_data->has_arr_rwy())
        {
            draw_runway(cr, buf[ARR_RWY_PROJ_IDX]);
        }
    }

    void NDDisplay::draw_airplane(cairo_t *cr)
    {
        geom::vect2_t pos;
        bool do_drawing = nd_data->get_ac_pos(&pos, fo_side);

        if(do_drawing)
        {
            geom::vect2_t pos_trans = get_screen_coords(pos);
            geom::vect2_t scale = size.scmul(1 / WPT_SCALE_FACT);

            cairo_utils::draw_rotated_image(cr, tex_mngr->data[AIRPLANE_NAME], pos_trans, 
                scale, hdg_data.brng_tru_rad);
        }
    }

    void NDDisplay::draw_background(cairo_t *cr, bool draw_inner)
    {
        cairo_surface_t *back_surf;
        if(draw_inner)
            back_surf = tex_mngr->data[PLN_BACKGND_INNER_NAME];
        else
            back_surf = tex_mngr->data[PLN_BACKGND_OUTER_NAME];
        
        geom::vect2_t scale = size / cairo_utils::get_surf_sz(back_surf);

        cairo_utils::draw_image(cr, back_surf, scr_pos, scale, false);
    }

    void NDDisplay::draw_act_leg_info(cairo_t *cr)
    {
        test::act_leg_info_t leg_info = nd_data->get_act_leg_info();

        geom::vect2_t act_name_pos = scr_pos + size * ACT_LEG_NAME_OFFS;
        geom::vect2_t act_time_pos = scr_pos + size * ACT_LEG_TIME_OFFS;
        geom::vect2_t act_dist_pos = scr_pos + size * ACT_LEG_DIST_OFFS;
        geom::vect2_t act_nm_pos = scr_pos + size * ACT_LEG_NM_OFFS;

        cairo_utils::draw_left_text(cr, font_face, leg_info.name, act_name_pos, 
            cairo_utils::MAGENTA, ND_ACT_INFO_MAIN_FONT_SZ);

        cairo_utils::draw_left_text(cr, font_face, "------Z", act_time_pos, 
            cairo_utils::WHITE, ND_ACT_INFO_MAIN_FONT_SZ);
        
        cairo_utils::draw_left_text(cr, font_face, leg_info.dist_nm, act_dist_pos, 
            cairo_utils::WHITE, leg_info.dist_sz);
        cairo_utils::draw_left_text(cr, font_face, "NM", act_nm_pos, 
            cairo_utils::WHITE, ND_ACT_INFO_DIST_FONT_SZ);
    }

    void NDDisplay::draw_spd_info(cairo_t *cr)
    {
        test::spd_info_t spd_info = nd_data->get_spd_data();

        geom::vect2_t gs_text_pos = scr_pos + size * GS_TEXT_OFFS;
        geom::vect2_t gs_pos = scr_pos + size * GS_OFFS;

        double gs_sz = ND_SPD_BIG_FONT_SZ;
        if(spd_info.gs_kts > GS_THRESH_BIG_KTS)
        {
            gs_sz = ND_SPD_SMALL_FONT_SZ;
        }

        std::string gs_str = strutils::double_to_str(spd_info.gs_kts, 0);

        cairo_utils::draw_left_text(cr, font_face, "GS", gs_text_pos, 
            cairo_utils::WHITE, ND_ACT_INFO_DIST_FONT_SZ);
        cairo_utils::draw_right_text(cr, font_face, gs_str, gs_pos, 
            cairo_utils::WHITE, gs_sz);

        if(spd_info.tas_kts >= TAS_DISPL_THRESH_KTS)
        {
            geom::vect2_t tas_text_pos = scr_pos + size * TAS_TEXT_OFFS;
            geom::vect2_t tas_pos = scr_pos + size * TAS_OFFS;

            std::string tas_str = strutils::double_to_str(spd_info.tas_kts, 0);

            cairo_utils::draw_left_text(cr, font_face, "TAS", tas_text_pos, 
                cairo_utils::WHITE, ND_ACT_INFO_DIST_FONT_SZ);
            cairo_utils::draw_right_text(cr, font_face, tas_str, tas_pos, 
                cairo_utils::WHITE, ND_SPD_SMALL_FONT_SZ);
        }
    }

    void NDDisplay::draw_range(cairo_t *cr)
    {
        uint8_t half_pr = 0, full_pr = 0;

        if (curr_rng <= RNG_DEC_1_NM) // Range can never be < 2.5
            full_pr = 1;
        std::string rng_full_str = strutils::double_to_str(curr_rng, full_pr);

        if (curr_rng / 2 <= RNG_DEC_2_NM)
            half_pr = 2;
        else if (curr_rng / 2 <= RNG_DEC_1_NM)
            half_pr = 1;
        std::string rng_half_str = strutils::double_to_str(curr_rng / 2, half_pr);

        geom::vect2_t ctr_trans = map_ctr + scr_pos;

        geom::vect2_t pos_1_dn = {ctr_trans.x, ctr_trans.y + curr_rng * scale_factor.y};
        geom::vect2_t pos_2_dn = {ctr_trans.x, ctr_trans.y + curr_rng * 0.5 * scale_factor.y};
        geom::vect2_t pos_1_up = {ctr_trans.x, ctr_trans.y - curr_rng * scale_factor.y};
        geom::vect2_t pos_2_up = {ctr_trans.x, ctr_trans.y - curr_rng * 0.5 * scale_factor.y};

        cairo_utils::draw_centered_text(cr, font_face, rng_full_str,
                                        pos_1_dn, cairo_utils::WHITE, ND_WPT_FONT_SZ);
        cairo_utils::draw_centered_text(cr, font_face, rng_full_str,
                                        pos_1_up, cairo_utils::WHITE, ND_WPT_FONT_SZ);
        cairo_utils::draw_centered_text(cr, font_face, rng_half_str,
                                        pos_2_dn, cairo_utils::WHITE, ND_WPT_FONT_SZ);
        cairo_utils::draw_centered_text(cr, font_face, rng_half_str,
                                        pos_2_up, cairo_utils::WHITE, ND_WPT_FONT_SZ);
    }
} // namespace StratosphereAvionics
