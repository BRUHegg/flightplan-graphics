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

    // map_data_t definitions:

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
        m_fpl_vec = fpl_sys->fpl_vec;

        trk_up = true;

        m_leg_data = std::vector<test::nd_leg_data_t*>(test::N_FPL_SYS_RTES);
        m_leg_data_sz = std::vector<size_t>(test::N_FPL_SYS_RTES, 0);
        for(size_t i = 0; i < test::N_FPL_SYS_RTES; i++)
            m_leg_data[i] = new test::nd_leg_data_t[N_LEG_PROJ_CACHE_SZ];
        
        m_rte_draw_seq = std::vector<std::vector<int>>(N_ND_SDS, 
            std::vector<int>(test::N_FPL_SYS_RTES, V_RTE_NOT_DRAWN));

        m_mp_data = std::vector<map_data_t>(N_MP_DATA_SZ);
        m_act_leg_idx_sd = std::vector<int>(N_MP_DATA_SZ, 0);
        for(size_t i = 0; i < N_MP_DATA_SZ; i++)
        {
            m_mp_data[i].create();
        }

        cr_mds = std::vector<std::pair<test::NDMode, bool>>(N_ND_SDS, 
            {test::DFLT_ND_MODE, false});
        m_ac_pos_proj = std::vector<geom::vect2_t>(N_ND_SDS, {0, 0});
        m_ac_pos_ok = std::vector<bool>(N_ND_SDS, 0);
        m_rng_idx = std::vector<size_t>(N_ND_SDS, 0);
        m_ctr = std::vector<geo::point>(N_ND_SDS, {0, 0});

        m_fpl_id_last = std::vector<double>(test::N_FPL_SYS_RTES, 0);

        m_hdg_data = {};

        m_has_dep_rwy = std::vector<bool>(test::N_FPL_SYS_RTES, false);
        m_has_arr_rwy = std::vector<bool>(test::N_FPL_SYS_RTES, false);

        m_act_leg_idx = std::vector<int>(test::N_FPL_SYS_RTES, -1);
    }

    void NDData::set_th_up()
    {
        trk_up = !trk_up;
    }

    bool NDData::get_th_up()
    {
        return trk_up;
    }

    void NDData::set_mode(size_t sd_idx, test::NDMode md, bool set_ctr)
    {
        assert(sd_idx < N_ND_SDS);
        cr_mds[sd_idx].first = md;
        if(md == test::NDMode::PLAN)
            cr_mds[sd_idx].second = false;
        else if(set_ctr)
            cr_mds[sd_idx].second = !cr_mds[sd_idx].second;
        m_fpl_sys_ptr->set_nd_mode(md, sd_idx);
    }

    std::pair<test::NDMode, bool> NDData::get_mode(size_t sd_idx) const
    {
        assert(sd_idx < N_ND_SDS);
        return cr_mds[sd_idx];
    }

    std::vector<int> NDData::get_rte_draw_seq(size_t sd_idx)
    {
        assert(sd_idx < N_ND_SDS);
        return m_rte_draw_seq[sd_idx];
    }

    size_t NDData::get_proj_legs(leg_proj_t **out, size_t sd_idx, size_t dt_idx)
    {
        *out = m_mp_data[sd_idx+dt_idx*N_ND_SDS].proj_legs;
        return m_mp_data[sd_idx+dt_idx*N_ND_SDS].n_act_proj_legs;
    }

    int NDData::get_act_leg_idx(size_t sd_idx)
    {
        return m_act_leg_idx_sd[sd_idx];
    }

    bool NDData::get_ac_pos(geom::vect2_t *out, size_t sd_idx)
    { 
        if(!m_ac_pos_ok[sd_idx])
            return false;

        *out = m_ac_pos_proj[sd_idx];
        return true;
    }

    double NDData::get_hdg_trk() const
    {
        return get_cr_rot();
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

    bool NDData::has_dep_rwy(size_t idx)
    {
        return m_has_dep_rwy[idx];
    }

    bool NDData::has_arr_rwy(size_t idx)
    {
        return m_has_arr_rwy[idx];
    }

    void NDData::switch_range(bool down, size_t sd_idx)
    {
        if (down)
        {
            if (m_rng_idx[sd_idx])
                m_rng_idx[sd_idx]--;
        }
        else
        {
            if (m_rng_idx[sd_idx] + 1 < ND_RANGES_NM.size())
                m_rng_idx[sd_idx]++;
        }
    }

    double NDData::get_range(size_t sd_idx)
    {
        return ND_RANGES_NM[m_rng_idx[sd_idx]];
    }

    void NDData::update()
    {
        m_hdg_data = m_fpl_sys_ptr->get_hdg_info();
        update_rte_draw_seq();
        for(size_t i = 0; i < N_ND_SDS; i++)
        {
            update_ctr(i);
            m_ac_pos_ok[i] = project_ac_pos(i);
        }
        for(size_t i = 0; i < test::N_FPL_SYS_RTES; i++)
            update_fpl(i);
    }

    NDData::~NDData()
    {
        for(size_t i = 0; i < test::N_FPL_SYS_RTES; i++)
            delete[] m_leg_data[i];
        for(size_t i = 0; i < N_MP_DATA_SZ; i++)
            m_mp_data[i].destroy();
    }

    // Private member functions:
    // Static member functions:

    bool NDData::bound_check(double x1, double x2, double rng)
    {
        return (x1 < rng && x2 >= rng) || (x1 >= rng && x2 < rng) ||
               (x1 > -rng && x2 <= -rng) || (x2 > -rng && x1 <= -rng) ||
               (abs(x1) <= rng && abs(x2) <= rng);
    }

    nd_util_idx_t NDData::get_util_idx(size_t gn_idx)
    {
        return {gn_idx%N_ND_SDS, gn_idx/N_ND_SDS};
    }

    // Non-static member functions:

    double NDData::get_cr_rot() const
    {
        if(trk_up)
            return -(m_hdg_data.brng_tru_rad+m_hdg_data.magvar_rad);
        return -(m_hdg_data.brng_tru_rad+m_hdg_data.magvar_rad+m_hdg_data.slip_rad);
    }

    std::pair<geo::point, double> NDData::get_proj_params(size_t sd_idx) const
    {
        assert(sd_idx < cr_mds.size());
        if(cr_mds[sd_idx].first == test::NDMode::PLAN)
            return {m_ctr[sd_idx], 0};
        return {m_fpl_sys_ptr->get_ac_pos(), get_cr_rot()};
    }

    bool NDData::in_view(geom::vect2_t start, geom::vect2_t end, size_t sd_idx)
    {
        assert(sd_idx < cr_mds.size());
        double a = start.x - end.x;
        double rng = get_range(sd_idx);
        if(cr_mds[sd_idx].second || cr_mds[sd_idx].first == test::NDMode::PLAN)
            rng /= 2;

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

    void NDData::update_rte_draw_seq()
    {
        size_t act_idx = m_fpl_sys_ptr->get_act_idx();
        bool exec_st = m_fpl_sys_ptr->get_exec();
        for(size_t i = 0; i < N_ND_SDS; i++)
        {
            std::vector<int> tmp(test::N_FPL_SYS_RTES, V_RTE_NOT_DRAWN);
            size_t sel_idx = m_fpl_sys_ptr->get_cdu_sel_fpl_idx(i);
            tmp[1] = test::ACT_RTE_IDX;
            if(act_idx != sel_idx)
                tmp[2] = sel_idx;
            else
                tmp[0] = act_idx;
            if(!exec_st || act_idx >= test::N_FPL_SYS_RTES)
                tmp[0] = -1;
            for(size_t j = 0; j < test::N_FPL_SYS_RTES; j++)
            {
                if(ND_RTE_CLRS[j] == cairo_utils::WHITE)
                    m_rte_draw_seq[i][j] = tmp[0];
                else if(ND_RTE_CLRS[j] == cairo_utils::MAGENTA)
                    m_rte_draw_seq[i][j] = tmp[1];
                else if(ND_RTE_CLRS[j] == cairo_utils::ND_CYAN)
                    m_rte_draw_seq[i][j] = tmp[2];
            }
        }
    }

    void NDData::update_ctr(size_t sd_idx)
    {
        geo::point tmp;
        bool ret = m_fpl_sys_ptr->get_ctr(&tmp, sd_idx);
        if (!ret)
        {
            tmp = m_fpl_sys_ptr->get_ac_pos();
        }
        m_ctr[sd_idx] = tmp;
    }

    void NDData::project_legs(size_t gn_idx)
    {
        nd_util_idx_t idxs = get_util_idx(gn_idx);

        std::pair<geo::point, double> mp_prm = get_proj_params(idxs.sd_idx);
        geo::point map_ctr = mp_prm.first;

        leg_proj_t *dst = m_mp_data[gn_idx].proj_legs;
        geom::line_joint_t *dst_joint = m_mp_data[gn_idx].line_joints;

        size_t *sz_ptr = &m_mp_data[gn_idx].n_act_proj_legs;
        size_t *sz_ptr_joint = &m_mp_data[gn_idx].n_act_joints;

        *sz_ptr = 0;
        *sz_ptr_joint = 0;
        m_act_leg_idx_sd[gn_idx] = -1;

        bool prev_skipped = false;
        bool prev_bypassed = false;

        for (size_t i = 0; i < m_leg_data_sz[idxs.dt_idx]; i++)
        {
            if (i >= N_LEG_PROJ_CACHE_SZ)
                break;

            dst[*sz_ptr].joint = nullptr;

            if ((!m_leg_data[idxs.dt_idx][i].leg_data.is_finite && 
                !m_leg_data[idxs.dt_idx][i].leg_data.is_bypassed) ||
                m_leg_data[idxs.dt_idx][i].leg_data.is_arc)
                continue;

            if (m_leg_data[idxs.dt_idx][i].leg_data.is_finite && 
                m_leg_data[idxs.dt_idx][i].leg_data.has_calc_wpt)
            {
                geo::point end_wpt = m_leg_data[idxs.dt_idx][i].leg_data.calc_wpt.data.pos;
                std::string end_name = m_leg_data[idxs.dt_idx][i].leg_data.calc_wpt.id;

                double dist_wpt = map_ctr.get_gc_dist_nm(end_wpt);
                double brng_wpt = map_ctr.get_gc_bearing_rad(end_wpt)+mp_prm.second;

                dst[*sz_ptr].end_wpt = {dist_wpt * sin(brng_wpt), dist_wpt * cos(brng_wpt)};
                dst[*sz_ptr].end_nm = end_name;
                dst[*sz_ptr].is_finite = true;
                dst[*sz_ptr].is_arc = m_leg_data[idxs.dt_idx][i].leg_data.is_arc;
                dst[*sz_ptr].has_path = false;
            }

            if (m_leg_data[idxs.dt_idx][i].leg_data.turn_rad_nm != -1)
            {
                geom::vect2_t start_proj = geom::project_point(m_leg_data[idxs.dt_idx][i].leg_data.start,
                                                               map_ctr, mp_prm.second);
                geom::vect2_t end_proj = geom::project_point(m_leg_data[idxs.dt_idx][i].leg_data.end,
                                                             map_ctr, mp_prm.second);

                if (!in_view(start_proj, end_proj, idxs.sd_idx))
                {
                    prev_skipped = true;
                    continue;
                }

                if (*sz_ptr && !prev_skipped && !m_leg_data[idxs.dt_idx][i].leg_data.has_disc)
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
                dst[*sz_ptr].has_path = !m_leg_data[idxs.dt_idx][i].leg_data.is_bypassed;

                dst[*sz_ptr].is_rwy = m_leg_data[idxs.dt_idx][i].leg_data.is_rwy;
                dst[*sz_ptr].turn_rad_nm = m_leg_data[idxs.dt_idx][i].leg_data.turn_rad_nm;

                if(m_act_leg_idx[idxs.dt_idx] != -1 && i == size_t(m_act_leg_idx[idxs.dt_idx]))
                {
                    m_act_leg_idx_sd[gn_idx] = int(*sz_ptr);
                }

                *sz_ptr = *sz_ptr + 1;
                prev_skipped = false;
                prev_bypassed = false;
            }
            else if (m_leg_data[idxs.dt_idx][i].leg_data.is_finite)
            {
                *sz_ptr = *sz_ptr + 1;
                prev_skipped = !m_leg_data[idxs.dt_idx][i].leg_data.is_bypassed;
            }
            prev_bypassed = m_leg_data[idxs.dt_idx][i].leg_data.turn_rad_nm == -1;
        }
    }

    void NDData::project_rwys(size_t gn_idx)
    {
        nd_util_idx_t idxs = get_util_idx(gn_idx);

        std::string dep_rwy = m_fpl_vec[idxs.dt_idx]->get_dep_rwy();
        std::string arr_rwy = m_fpl_vec[idxs.dt_idx]->get_arr_rwy();

        m_has_dep_rwy[idxs.dt_idx] = dep_rwy != "";
        m_has_arr_rwy[idxs.dt_idx] = arr_rwy != "";

        if (!m_has_dep_rwy[idxs.dt_idx] && !m_has_arr_rwy[idxs.dt_idx])
            return;

        std::pair<geo::point, double> mp_prm = get_proj_params(idxs.sd_idx);
        geo::point map_ctr = mp_prm.first;

        leg_proj_t *dst = m_mp_data[gn_idx].proj_legs;

        if (m_has_dep_rwy[idxs.dt_idx])
        {
            libnav::runway_entry_t rnw_data;
            bool has_data = m_fpl_vec[idxs.dt_idx]->get_dep_rwy_data(&rnw_data);

            if (has_data)
            {
                dst[DEP_RWY_PROJ_IDX].start = geom::project_point(rnw_data.start, map_ctr, 
                    mp_prm.second);
                dst[DEP_RWY_PROJ_IDX].end = geom::project_point(rnw_data.end, map_ctr, 
                    mp_prm.second);
            }
        }

        if (m_has_arr_rwy[idxs.dt_idx])
        {
            libnav::runway_entry_t rnw_data;
            bool has_data = m_fpl_vec[idxs.dt_idx]->get_arr_rwy_data(&rnw_data);

            if (has_data)
            {
                dst[ARR_RWY_PROJ_IDX].start = geom::project_point(rnw_data.start, map_ctr, 
                    mp_prm.second);
                dst[ARR_RWY_PROJ_IDX].end = geom::project_point(rnw_data.end, map_ctr, 
                    mp_prm.second);
            }
        }
    }

    bool NDData::project_ac_pos(size_t sd_idx)
    {
        geo::point map_ctr = m_ctr[sd_idx];
        geo::point curr_pos = m_fpl_sys_ptr->get_ac_pos();
        
        double gc_dist_nm = map_ctr.get_gc_dist_nm(curr_pos);
        double rng = get_range(sd_idx) / 2;

        if(rng < gc_dist_nm)
            return false;

        double gc_brng_rad = map_ctr.get_gc_bearing_rad(curr_pos);
        m_ac_pos_proj[sd_idx] = geom::get_projection(gc_brng_rad, gc_dist_nm);

        return true;
    }

    void NDData::fetch_legs(size_t dt_idx)
    {
        m_leg_data_sz[dt_idx] = m_fpl_sys_ptr->get_nd_seg(m_leg_data[dt_idx], 
            N_LEG_PROJ_CACHE_SZ, dt_idx);
        m_act_leg_idx[dt_idx] = m_fpl_sys_ptr->get_act_leg_idx();
    }

    void NDData::update_fpl(size_t idx)
    {
        double id_curr = m_fpl_vec[idx]->get_id();

        if (id_curr != m_fpl_id_last[idx])
        {
            fetch_legs(idx);
        }

        for(size_t i = 0; i < N_ND_SDS; i++)
        {
            project_legs(i+idx*N_ND_SDS);
            project_rwys(i+idx*N_ND_SDS);
        }

        m_fpl_id_last[idx] = id_curr;
    }

    // NDDisplay member functions:

    // Public member functions:

    NDDisplay::NDDisplay(std::shared_ptr<NDData> data,
                         std::shared_ptr<cairo_utils::texture_manager_t> mngr,
                         cairo_font_face_t *ff, geom::vect2_t pos, geom::vect2_t sz, 
                         size_t sd_idx)
    {
        nd_data = data;
        tex_mngr = mngr;

        font_face = ff;

        scr_pos = pos;
        size = sz;

        side_idx = sd_idx;

        is_trk_up = nd_data->get_th_up();
        rng = nd_data->get_range(side_idx);
    }

    void NDDisplay::draw(cairo_t *cr)
    {
        update_mode();
        rng = nd_data->get_range(side_idx);
        update_map_params();
        hdg_data = nd_data->get_hdg_data();

        cairo_utils::draw_rect(cr, scr_pos, size, ND_BCKGRND_CLR);

        draw_background(cr, true);
        draw_all_fplns(cr);
        draw_airplane(cr);

        draw_background(cr, false);
        draw_act_leg_info(cr);
        draw_spd_info(cr);
        draw_range(cr);
        //cairo_utils::draw_circle(cr, scr_pos+map_ctr, 3, 3, cairo_utils::MAGENTA);
    }

    // Private member functions:

    void NDDisplay::update_mode()
    {
        is_trk_up = nd_data->get_th_up();
        std::pair<test::NDMode, bool> md_dt = nd_data->get_mode(side_idx);
        cr_md = md_dt.first;
        is_ctr = md_dt.second;
    }

    void NDDisplay::update_map_params()
    {
        if(cr_md == test::NDMode::PLAN)
        {
            curr_rng = rng / 2;
            map_ctr = size.scmul(0.5);
            map_ctr.y += size.y * ND_PRJ_CTR_V_OFFS_PLAN;
        }
        else if(cr_md == test::NDMode::MAP)
        {
            curr_rng = rng;
            map_ctr = size.scmul(0.5);
            map_ctr.y += size.y * ND_PRJ_CTR_V_OFFS_MAP;
        }
        scale_factor = size.scmul(ND_RNG_FULL_RES_COEFF[cr_md]).scdiv(curr_rng);
    }

    geom::vect2_t NDDisplay::get_screen_coords(geom::vect2_t src)
    {
        src.y *= -1;
        geom::vect2_t out = src * scale_factor + map_ctr + scr_pos;

        return out;
    }

    void NDDisplay::draw_line_joint(cairo_t *cr, geom::line_joint_t lj, geom::vect3_t ln_clr)
    {
        double radius_px = lj.turn_radius * scale_factor.x;
        if (lj.tp == geom::JointType::CIRC_CIRC)
        {
            geom::vect2_t arc1_trans = get_screen_coords(lj.arc1.pos);
            geom::vect2_t arc2_trans = get_screen_coords(lj.arc2.pos);

            cairo_utils::draw_arc(cr, arc1_trans, radius_px, lj.arc1.ang_start_rad,
                                  lj.arc1.ang_end_rad, ND_FPL_LINE_THICK * size.x, 
                                    ln_clr);
            cairo_utils::draw_arc(cr, arc2_trans, radius_px, lj.arc2.ang_start_rad,
                                  lj.arc2.ang_end_rad, ND_FPL_LINE_THICK * size.x, 
                                  ln_clr);
        }
        else if (lj.tp == geom::JointType::CIRC)
        {
            geom::vect2_t arc1_trans = get_screen_coords(lj.arc1.pos);
            cairo_utils::draw_arc(cr, arc1_trans, radius_px, lj.arc1.ang_start_rad,
                                  lj.arc1.ang_end_rad, ND_FPL_LINE_THICK * size.x, 
                                  ln_clr);
        }

        geom::vect2_t s_trans = get_screen_coords(lj.line.start);
        geom::vect2_t e_trans = get_screen_coords(lj.line.end);

        cairo_utils::draw_line(cr, s_trans, e_trans,
                               ln_clr, ND_FPL_LINE_THICK * size.x);
    }

    void NDDisplay::draw_flight_plan(cairo_t *cr, bool draw_labels, 
        geom::vect3_t ln_clr, size_t idx)
    {
        leg_proj_t *buf;
        size_t buf_size = nd_data->get_proj_legs(&buf, side_idx, idx);
        int act_leg_idx = -1;
        if(ln_clr == cairo_utils::MAGENTA)
            act_leg_idx = nd_data->get_act_leg_idx(side_idx);

        bool draw_dash = !draw_labels && ln_clr != cairo_utils::MAGENTA;
        if(draw_dash)
        {
            cairo_save(cr);
            cairo_set_dash(cr, FPLN_LN_DASH, N_FPLN_DASHES, 2);
        }

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

                        draw_line_joint(cr, curr_joint, ln_clr);
                    }

                    if(buf[i].has_path)
                    {
                        geom::vect2_t s_trans = get_screen_coords(start);
                        geom::vect2_t e_trans = get_screen_coords(end);
                        
                        cairo_utils::draw_line(cr, s_trans, e_trans,
                                            ln_clr, ND_FPL_LINE_THICK * size.x);
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

        if(draw_dash)
            cairo_restore(cr);
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

    void NDDisplay::draw_runways(cairo_t *cr, size_t idx)
    {
        leg_proj_t *buf;
        size_t buf_size = nd_data->get_proj_legs(&buf, side_idx, idx);
        UNUSED(buf_size);

        if (nd_data->has_dep_rwy(idx))
        {
            draw_runway(cr, buf[DEP_RWY_PROJ_IDX]);
        }

        if (nd_data->has_arr_rwy(idx))
        {
            draw_runway(cr, buf[ARR_RWY_PROJ_IDX]);
        }
    }

    void NDDisplay::draw_all_fplns(cairo_t *cr)
    {
        std::vector<int> fpl_draw_seq = nd_data->get_rte_draw_seq(side_idx);
        for(size_t i = 0; i < test::N_FPL_SYS_RTES; i++)
        {
            if(fpl_draw_seq[i] == -1)
                continue;
            draw_runways(cr, fpl_draw_seq[i]);
            draw_flight_plan(cr, false, ND_RTE_CLRS[i], fpl_draw_seq[i]);
            draw_flight_plan(cr, true, ND_RTE_CLRS[i], fpl_draw_seq[i]);
        }
    }

    void NDDisplay::draw_airplane(cairo_t *cr)
    {
        if(cr_md == test::NDMode::PLAN)
        {
            geom::vect2_t pos;
            bool do_drawing = nd_data->get_ac_pos(&pos, side_idx);
            geom::vect2_t wpt_scale = size.scmul(1 / WPT_SCALE_FACT);
            if(do_drawing)
            {
                geom::vect2_t pos_trans = get_screen_coords(pos);
                cairo_utils::draw_rotated_image(cr, tex_mngr->data[AIRPLANE_NAME], pos_trans, 
                    wpt_scale, hdg_data.brng_tru_rad);
            }
        }
        else
        {
            cairo_surface_t *tgt = tex_mngr->data[MAP_AC_TRI_NAME];
            geom::vect2_t sz = cairo_utils::get_surf_sz(tgt) * MAP_AC_TRI_SC;
            geom::vect2_t sz_shift = {0, 0.5};
            geom::vect2_t pos = scr_pos + map_ctr + sz * sz_shift;
            cairo_utils::draw_image(cr, tgt, pos, MAP_AC_TRI_SC, true);
        }
    }

    void NDDisplay::draw_htrk(cairo_t *cr)
    {
        int rot_raw_deg = -int(std::round(nd_data->get_hdg_trk()*geom::RAD_TO_DEG)) % 360;
        int rot_deg = (rot_raw_deg+360) % 360;
        std::string htk_txt = strutils::double_to_str(rot_deg, 0);
        htk_txt = std::string(3-htk_txt.size(), '0')+htk_txt;
        cairo_utils::draw_centered_text(cr, font_face, htk_txt, scr_pos+size*MAP_HTK_TXT_POS, 
            cairo_utils::WHITE, MAP_HTRK_FONT_SZ);
        
        geom::vect2_t pos_htk = geom::vect2_t{0.5-MAP_HTRK_STG_TXT_LOFFS, 
            MAP_HTRK_STG_TXT_VOFFS};
        geom::vect2_t pos_tmg = geom::vect2_t{0.5+MAP_HTRK_STG_TXT_LOFFS, 
            MAP_HTRK_STG_TXT_VOFFS};
        std::string htk_st_txt = "";
        if(is_trk_up)
            htk_st_txt = ND_TRKUP;
        else
            htk_st_txt = ND_HDGUP;
        cairo_utils::draw_centered_text(cr, font_face, htk_st_txt, scr_pos+size*pos_htk, 
            cairo_utils::GREEN, MAP_HTRK_STG_FONT_SZ);
        cairo_utils::draw_centered_text(cr, font_face, "MAG", scr_pos+size*pos_tmg, 
            cairo_utils::GREEN, MAP_HTRK_STG_FONT_SZ);
    }

    void NDDisplay::draw_htrk_line(cairo_t *cr, bool is_inn)
    {
        double rot_rad = 0;
        if(!is_trk_up)
            rot_rad = -hdg_data.slip_rad;
        geom::vect2_t dir = geom::vect2_t{size.x*sin(rot_rad), -size.x*cos(rot_rad)};
        geom::vect2_t ln_end_inn = map_ctr + dir.scmul(MAP_TRK_LN_LN_INN);
        geom::vect2_t pos_start = map_ctr+scr_pos;
        geom::vect2_t pos_end = ln_end_inn+scr_pos;
        geom::vect3_t clr = cairo_utils::WHITE;
        if(!is_inn)
        {
            pos_start = pos_end+dir.scmul(MAP_TRK_LN_LN_OUT);
            //clr = cairo_utils::MAGENTA;
        }
        cairo_utils::draw_line(cr, pos_start, pos_end, 
            clr, ND_FPL_LINE_THICK*size.x);
    }

    void NDDisplay::draw_background(cairo_t *cr, bool draw_inner)
    {
        cairo_surface_t *back_surf;

        if(cr_md == test::NDMode::PLAN)
        {
            if(draw_inner)
                back_surf = tex_mngr->data[PLN_BACKGND_INNER_NAME];
            else
                back_surf = tex_mngr->data[PLN_BACKGND_OUTER_NAME];
        }
        else if(cr_md == test::NDMode::MAP)
        {
            back_surf = tex_mngr->data[MAP_BACKGND_NAME];
        }
        
        if((!draw_inner && cr_md == test::NDMode::MAP) || cr_md == test::NDMode::PLAN)
        {
            geom::vect2_t scale_back = size / cairo_utils::get_surf_sz(back_surf);
            cairo_utils::draw_image(cr, back_surf, scr_pos, scale_back, false);
        }

        if(cr_md == test::NDMode::MAP)
        {
            if(!draw_inner)
            {
                cairo_surface_t *map_hdg_surf = tex_mngr->data[MAP_HDG_NAME];
                geom::vect2_t scale_hdg = (size / cairo_utils::get_surf_sz(map_hdg_surf)).scmul(1.41);
                geom::vect2_t hdg_pos = scr_pos+map_ctr;
                cairo_utils::draw_rotated_image(cr, 
                    map_hdg_surf, hdg_pos, scale_hdg, nd_data->get_hdg_trk());
                cairo_surface_t *htrk_box = tex_mngr->data[HTRK_BOX_NAME];
                geom::vect2_t box_sz = cairo_utils::get_surf_sz(htrk_box);
                geom::vect2_t scale_box = (box_sz*size.scdiv(900))*MAP_HTK_BOX_SC;
                double hht = box_sz.y*0.5*scale_box.y;
                geom::vect2_t box_pos = scr_pos+geom::vect2_t{size.x/2, hht};
                cairo_utils::draw_image(cr, htrk_box, box_pos, scale_box, true);
                draw_htrk(cr);
            }
            draw_htrk_line(cr, draw_inner);
        }
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

        if(cr_md == test::NDMode::PLAN || is_ctr)
        {
            cairo_utils::draw_centered_text(cr, font_face, rng_full_str,
                                            pos_1_dn, cairo_utils::WHITE, ND_WPT_FONT_SZ);
            cairo_utils::draw_centered_text(cr, font_face, rng_half_str,
                                            pos_2_dn, cairo_utils::WHITE, ND_WPT_FONT_SZ);
        }
        cairo_utils::draw_centered_text(cr, font_face, rng_full_str,
                                        pos_1_up, cairo_utils::WHITE, ND_WPT_FONT_SZ);
        cairo_utils::draw_centered_text(cr, font_face, rng_half_str,
                                        pos_2_up, cairo_utils::WHITE, ND_WPT_FONT_SZ);
    }
} // namespace StratosphereAvionics
