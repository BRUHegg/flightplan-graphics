#include "cdu.hpp"

namespace StratosphereAvionics
{
    // cdu_scr_data_t definitions:

    cdu_scr_data_t::cdu_scr_data_t()
    {
        data_lines.reserve(2 * N_CDU_DATA_LINES);
        chr_sts.reserve(2 * N_CDU_DATA_LINES);
    }

    // CDU definitions:
    // Public member functions:

    CDU::CDU(std::shared_ptr<test::FPLSys> fs)
    {
        fpl_sys = fs;
        sel_fpl_idx = test::RTE1_IDX;
        act_fpl_idx = test::N_FPL_SYS_RTES;
        fpln = fs->fpl_vec[sel_fpl_idx];
        m_rte1_ptr = fs->fpl_vec[test::RTE1_IDX];
        m_rte2_ptr = fs->fpl_vec[test::RTE2_IDX];
        m_act_ptr = fs->fpl_vec[test::ACT_RTE_IDX];

        curr_page = CDUPage::RTE;
        curr_subpg = 1;
        n_subpg = 1;

        rte_copy = test::RTECopySts::UNAVAIL;

        sel_des_idx = -1;
        sel_des_subpg = 0;
        sel_des_event = 0;
        sel_des_seg_id = 0;
        sel_des_leg_id = 0;
        sel_des = false;

        dep_arr_rwy_filter = std::vector<bool>(N_CDU_RTES, false);
        dep_arr_rwy_filter = std::vector<bool>(N_CDU_RTES, false);
        dep_arr_proc_filter = std::vector<bool>(N_CDU_RTES, false);
        dep_arr_trans_filter = std::vector<bool>(N_CDU_RTES, false);
        dep_arr_via_filter = std::vector<bool>(N_CDU_RTES, false);

        procs = std::vector<std::vector<std::string>>(N_CDU_RTES);
        trans = std::vector<std::vector<std::string>>(N_CDU_RTES);
        apprs = std::vector<std::vector<std::string>>(N_CDU_RTES);
        rwys = std::vector<std::vector<std::string>>(N_CDU_RTES);
        vias = std::vector<std::vector<std::string>>(N_CDU_RTES);
        fpl_infos = std::vector<test::fpln_info_t>(test::N_FPL_SYS_RTES);
        leg_sel = std::vector<std::pair<size_t, double>>(N_CDU_RTES, {0LL, -1.0});

        leg_sel_pr = false;

        sel_des_nm = "";
    }

    void CDU::update()
    {
        seg_list = fpl_sys->get_seg_list(&n_seg_list_sz, sel_fpl_idx);
        leg_list = fpl_sys->get_leg_list(&n_leg_list_sz, sel_fpl_idx);
        fpln = fpl_sys->fpl_vec[sel_fpl_idx];
        act_fpl_idx = fpl_sys->get_act_idx();

        if (sel_des)
        {
            n_subpg = get_n_sel_des_subpg();
        }
        else
        {
            if (curr_page == CDUPage::RTE)
            {
                n_subpg = get_n_rte_subpg();
            }
            else if (curr_page == CDUPage::DEP_ARR_INTRO ||
                     curr_page == CDUPage::DEP1 || curr_page == CDUPage::ARR1 ||
                     curr_page == CDUPage::DEP2 || curr_page == CDUPage::ARR2)
            {
                bool is_rte2 = curr_page == CDUPage::DEP2 || curr_page == CDUPage::ARR2;
                n_subpg = get_n_dep_arr_subpg(is_rte2);
            }
            else if (curr_page == CDUPage::LEGS)
            {
                n_subpg = get_n_legs_subpg();
            }
        }

        if (curr_subpg > n_subpg)
        {
            curr_subpg = 1;
        }

        update_fpl_infos();
    }

    bool CDU::get_exec_lt()
    {
        bool e_st = fpl_sys->get_exec();
        if (act_fpl_idx == sel_fpl_idx)
            return e_st;
        return false;
    }

    std::string CDU::on_event(int event_key, std::string scratchpad, std::string *s_out)
    {
        if (curr_page != CDUPage::LEGS)
        {
            reset_leg_all_sel();
        }

        if (event_key == CDU_KEY_EXEC)
        {
            if (act_fpl_idx == sel_fpl_idx)
                fpl_sys->execute();
            return "";
        }

        if (event_key > CDU_KEY_RSK_TOP + 5 && event_key < CDU_KEY_A)
        {
            CDUPage pg = CDU_PAGE_FACES[event_key - CDU_KEY_RSK_TOP - 6];

            if (pg == CDUPage::NEXT_PAGE)
            {
                curr_subpg++;
            }
            else if (pg == CDUPage::PREV_PAGE)
            {
                curr_subpg--;
            }
            else
            {
                set_page(pg);
            }

            if (curr_subpg > n_subpg)
            {
                curr_subpg = 1;
            }
            else if (curr_subpg == 0)
            {
                curr_subpg = n_subpg;
            }

            *s_out = scratchpad;

            return "";
        }
        if (sel_des)
            return handle_sel_des(event_key);

        std::string msg = "";
        if (curr_page == CDUPage::RTE)
        {
            msg = handle_rte(event_key, scratchpad, s_out);
        }
        else if (curr_page == CDUPage::DEP_ARR_INTRO)
        {
            msg = handle_dep_arr(event_key);
        }
        else if (curr_page == CDUPage::DEP1)
        {
            msg = handle_dep(event_key, false);
        }
        else if (curr_page == CDUPage::ARR1)
        {
            msg = handle_arr(event_key, false);
        }
        else if (curr_page == CDUPage::DEP2)
        {
            msg = handle_dep(event_key, true);
        }
        else if (curr_page == CDUPage::ARR2)
        {
            msg = handle_arr(event_key, true);
        }
        else if (curr_page == CDUPage::LEGS)
        {
            msg = handle_legs(event_key, scratchpad, s_out);
        }
        if(sel_des)
            sel_des_event = event_key;

        return msg;
    }

    cdu_scr_data_t CDU::get_screen_data()
    {
        if (sel_des)
            return get_sel_des_page();

        if (curr_page == CDUPage::RTE)
            return get_rte_page();

        if (curr_page == CDUPage::DEP_ARR_INTRO)
            return get_dep_arr_page();

        if (curr_page == CDUPage::DEP1)
            return get_dep_page(false);

        if (curr_page == CDUPage::ARR1)
            return get_arr_page(false);

        if (curr_page == CDUPage::DEP2)
            return get_dep_page(true);

        if (curr_page == CDUPage::ARR2)
            return get_arr_page(true);

        if (curr_page == CDUPage::LEGS)
            return get_legs_page();

        return {};
    }

    // Private member functions:

    std::string CDU::get_cdu_line(std::string in, std::string line,
                                  bool align_right)
    {
        assert(line.size() + in.size() <= N_CDU_DATA_COLS);
        if (!align_right)
        {
            return in;
        }
        else
        {
            size_t n_sp = N_CDU_DATA_COLS - in.size() - line.size();
            return line + std::string(n_sp, ' ') + in;
        }
    }

    void CDU::fill_char_state_buf(cdu_scr_data_t &src)
    {
        for (int i = 0; i < N_CDU_DATA_LINES; i++)
        {
            src.chr_sts.push_back(CDU_ALL_S_WHITE);
            src.chr_sts.push_back(CDU_ALL_B_WHITE);
        }
    }

    std::string CDU::get_cdu_leg_prop(test::list_node_ref_t<test::leg_list_data_t> &src)
    {
        if (src.data.is_discon)
        {
            return DISCO_THEN;
        }
        if (src.data.leg.leg_type[0] == 'H')
        {
            return HOLD_DESC;
        }
        double crs_deg = src.data.misc_data.true_trk_deg+360.0;
        if(crs_deg > 360)
            crs_deg -= 360;
        std::string deg_str = std::string(1, strutils::DEGREE_SYMBOL);
        std::string crs_st = strutils::double_to_str(crs_deg, 0) + deg_str;
        assert(crs_st.size() <= N_LEG_CRS_ROWS);
        size_t crs_offs = N_LEG_CRS_ROWS - crs_st.size();
        crs_st = std::string(crs_offs, ' ') + crs_st;

        std::string out = "";
        bool is_bp = src.data.misc_data.is_bypassed;
        if(is_bp)
        {
            size_t offs = size_t(N_CDU_DATA_COLS)-crs_st.size()-LEG_BYPASS.size();
            out = crs_st + std::string(offs, ' ') + LEG_BYPASS;
        }
        else
        {
            double dist_nm = src.data.leg.outbd_dist_time;
            std::string dist_st = strutils::double_to_str(dist_nm, 0) + NAUT_MILES;
            assert(dist_st.size() + crs_st.size() <= N_LEG_PROP_ROWS);
            size_t offs = N_LEG_PROP_ROWS - dist_st.size() - crs_st.size();
            out = crs_st + std::string(offs, ' ') +
                            dist_st;
        }
        
        return out;
    }

    std::string CDU::get_leg_alt(test::list_node_ref_t<test::leg_list_data_t> &src,
                                 bool alt2, bool fl)
    {
        int tgt_alt = src.data.leg.alt1_ft;
        if (alt2)
            tgt_alt = src.data.leg.alt2_ft;
        int tr_alt = src.data.leg.trans_alt;
        bool fl_act = tgt_alt > tr_alt;
        if (fl_act || fl)
            tgt_alt /= 100;
        std::string alt_str = strutils::double_to_str(double(tgt_alt), 0);
        if ((fl_act || fl) && alt_str.size() < 3)
            alt_str = std::string(3 - alt_str.size(), '0') + alt_str;
        if (fl_act && !fl)
            alt_str = "FL" + alt_str;
        return alt_str;
    }

    std::string CDU::get_cdu_leg_vcstr(test::list_node_ref_t<test::leg_list_data_t> &src)
    {
        if (src.data.leg.alt1_ft == 0 && src.data.leg.alt2_ft == 0)
        {
            return LEG_NO_ALT;
        }

        if (src.data.leg.alt_desc == libnav::AltMode::AT_OR_ABOVE ||
            src.data.leg.alt_desc == libnav::AltMode::GS_AT_OR_ABOVE)
        {
            return get_leg_alt(src) + "A";
        }
        if (src.data.leg.alt_desc == libnav::AltMode::SID_AT_OR_ABOVE ||
            src.data.leg.alt_desc == libnav::AltMode::ALT_STEPDOWN_AT_AT_OR_ABOVE)
        {
            return get_leg_alt(src, true) + "A";
        }

        if (src.data.leg.alt_desc == libnav::AltMode::AT ||
            src.data.leg.alt_desc == libnav::AltMode::GS_AT)
        {
            return get_leg_alt(src);
        }
        if (src.data.leg.alt_desc == libnav::AltMode::GS_INTC_AT ||
            src.data.leg.alt_desc == libnav::AltMode::ALT_STEPDOWN_AT_AT)
        {
            return get_leg_alt(src, true);
        }

        if (src.data.leg.alt_desc == libnav::AltMode::AT_OR_BELOW)
        {
            return get_leg_alt(src) + "B";
        }
        if (src.data.leg.alt_desc == libnav::AltMode::ALT_STEPDOWN_AT_AT_OR_BELOW)
        {
            return get_leg_alt(src, true) + "B";
        }
        return get_leg_alt(src, false, true) + "A" + get_leg_alt(src, true, true) + "B";
    }

    std::string CDU::get_cdu_leg_spdcstr(test::list_node_ref_t<test::leg_list_data_t> &src)
    {
        if (src.data.leg.spd_lim_kias == 0)
            return LEG_NO_SPD;

        std::string spd = strutils::double_to_str(double(src.data.leg.spd_lim_kias), 0);

        if (src.data.leg.speed_desc == libnav::SpeedMode::AT_OR_ABOVE)
            return spd + "A";
        if (src.data.leg.speed_desc == libnav::SpeedMode::AT_OR_BELOW)
            return spd + "B";

        return spd;
    }

    std::string CDU::get_cdu_leg_nm(test::list_node_ref_t<test::leg_list_data_t> &src)
    {
        if (src.data.is_discon)
        {
            return DISCO_LEG_NM;
        }

        if (src.data.leg.leg_type == "FM" || src.data.leg.leg_type == "VM")
        {
            return LEG_VECTORS;
        }
        if (src.data.misc_data.has_calc_wpt)
        {
            return src.data.misc_data.calc_wpt.id;
        }
        return "";
    }

    bool CDU::scratchpad_has_delete(std::string &scratchpad)
    {
        if (scratchpad.size() && scratchpad[0] == DELETE_SYMBOL)
            return 1;
        return 0;
    }

    // Non-static member-functions:

    void CDU::update_fpl_infos()
    {
        for (size_t i = 0; i < fpl_infos.size(); i++)
        {
            fpl_infos[i] = fpl_sys->get_fpl_info(i);
        }
    }

    void CDU::set_page(CDUPage pg)
    {
        curr_subpg = 1;
        curr_page = pg;
        sel_des_idx = -1;
        sel_des = false;

        leg_sel_pr = false;

        for (size_t i = 0; i < N_CDU_RTES; i++)
        {
            dep_arr_rwy_filter[i] = false;
            dep_arr_proc_filter[i] = false;
            dep_arr_trans_filter[i] = false;
            dep_arr_via_filter[i] = false;
        }
    }

    void CDU::set_sel_des_state(double seg_id, double leg_id, std::string& name, 
            std::vector<libnav::waypoint_entry_t>& w_e)
    {
        sel_des_seg_id = seg_id;
        sel_des_leg_id = leg_id;
        sel_des_data = w_e;
        sel_des_nm = name;
        sel_des_subpg = curr_subpg;
        curr_subpg = 1;
        sel_des = true;
    }

    libnav::waypoint_t CDU::get_wpt_from_user(std::string name, double seg_id, double leg_id, 
        bool *not_in_db, bool *inv_ent, bool *wait_sel, bool *sel_used)
    {
        libnav::waypoint_entry_t tgt;
        std::string wpt_nm;

        if (sel_des_idx == -1)
        {
            if (name.size() > 5)
            {
                *inv_ent = 1;
                return {};
            }
            std::vector<libnav::waypoint_entry_t> wpt_entr;
            size_t n_found = fpl_sys->navaid_db_ptr->get_wpt_data(name, &wpt_entr);

            if (n_found == 0)
            {
                *not_in_db = 1;
                return {};
            }
            else if (n_found > 1)
            {
                set_sel_des_state(seg_id, leg_id, name, wpt_entr);
                *wait_sel = 1;
                return {};
            }
            wpt_nm = name;
            tgt = wpt_entr[0];
        }
        else
        {
            tgt = sel_des_data[size_t(sel_des_idx)];
            wpt_nm = sel_des_nm;
            sel_des_idx = -1;
            *sel_used = 1;
        }
        return {wpt_nm, tgt};
    }

    std::string CDU::set_departure(std::string icao, std::string *s_out)
    {
        if (icao == "")
            *s_out = fpln->get_dep_icao();
        if (icao.size() != 4)
            return INVALID_ENTRY_MSG;
        libnav::DbErr err = fpln->set_dep(icao);
        if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
        {
            return NOT_IN_DB_MSG;
        }

        return "";
    }

    std::string CDU::set_arrival(std::string icao, std::string *s_out)
    {
        if (icao == "")
            *s_out = fpln->get_arr_icao();

        libnav::DbErr err = fpln->set_arr(icao);
        if (err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
        {
            return NOT_IN_DB_MSG;
        }

        return "";
    }

    std::string CDU::set_flt_nbr(std::string nbr)
    {
        if (nbr.size() > N_FLT_NBR_CHR_MAX)
            return INVALID_ENTRY_MSG;

        if (nbr != "" && nbr[0] != DELETE_SYMBOL)
            fpl_sys->set_flt_nbr(nbr);
        else if (nbr[0] == DELETE_SYMBOL)
            fpl_sys->set_flt_nbr("");

        return "";
    }

    std::string CDU::set_dep_rwy(std::string id)
    {
        std::string dep_icao = fpln->get_dep_icao();
        std::string arr_icao = fpln->get_arr_icao();
        if (id.size() > 3 || dep_icao == "" || arr_icao == "")
            return INVALID_ENTRY_MSG;

        bool rwy_set = fpln->set_dep_rwy(id);

        if (!rwy_set)
            return NOT_IN_DB_MSG;

        return "";
    }

    std::string CDU::load_rte()
    {
        std::string dep_nm = fpln->get_dep_icao();
        std::string arr_nm = fpln->get_arr_icao();

        if (dep_nm != "" && arr_nm != "")
        {
            std::string file_nm = fpl_sys->fpl_dir + dep_nm + arr_nm;
            libnav::DbErr err = fpln->load_from_fms(file_nm, false);
            UNUSED(err);
        }

        return "";
    }

    std::string CDU::save_rte()
    {
        std::string dep_nm = fpln->get_dep_icao();
        std::string arr_nm = fpln->get_arr_icao();

        if (dep_nm != "" && arr_nm != "")
        {
            std::string out_nm = fpl_sys->fpl_dir + dep_nm + arr_nm;
            fpln->save_to_fms(out_nm);
        }

        return "";
    }

    std::string CDU::add_via(size_t next_idx, std::string name)
    {
        test::fpln_info_t f_inf = fpl_sys->get_fpl_info(sel_fpl_idx);
        double id = f_inf.seg_list_id;
        if (name.size() > 7)
            return INVALID_ENTRY_MSG;
        test::seg_list_node_t *s_ptr = nullptr;
        if (next_idx < n_seg_list_sz)
        {
            s_ptr = seg_list[next_idx].ptr;
        }

        bool retval = fpln->add_enrt_seg({s_ptr, id}, name);

        if (!retval)
            return NOT_IN_DB_MSG;
        return "";
    }

    std::string CDU::delete_via(size_t next_idx)
    {
        test::fpln_info_t f_inf = fpl_infos[sel_fpl_idx];
        double id = f_inf.seg_list_id;
        test::seg_list_node_t *s_ptr = nullptr;
        if (next_idx < n_seg_list_sz)
        {
            s_ptr = seg_list[next_idx].ptr;
        }
        bool retval = fpln->delete_via({s_ptr, id});

        if (!retval)
            return INVALID_DELETE_MSG;
        return "";
    }

    std::string CDU::add_to(size_t next_idx, std::string name)
    {
        test::fpln_info_t f_inf = fpl_infos[sel_fpl_idx];
        double sg_id = f_inf.seg_list_id;
        double lg_id = f_inf.leg_list_id;

        bool inv_ent = 0, not_in_db = 0, wait_sel = 0, sel_used = 0;
        libnav::waypoint_t tgt_wpt = get_wpt_from_user(name, sg_id, lg_id, &not_in_db, &inv_ent, 
            &wait_sel, &sel_used);
        if(inv_ent)
            return INVALID_ENTRY_MSG;
        if(not_in_db)
            return NOT_IN_DB_MSG;
        if(wait_sel)
            return "";
        
        if(sel_used)
        {
            sg_id = sel_des_seg_id; // Only need segment id here
        }
            

        test::seg_list_node_t *s_ptr = nullptr;
        if (next_idx < n_seg_list_sz)
        {
            s_ptr = seg_list[next_idx].ptr;
        }

        bool retval = fpln->awy_insert({s_ptr, sg_id}, tgt_wpt);

        if (!retval)
            return NOT_IN_DB_MSG;
        return "";
    }

    std::string CDU::delete_to(size_t next_idx)
    {
        test::fpln_info_t f_inf = fpl_infos[sel_fpl_idx];
        double id = f_inf.seg_list_id;
        test::seg_list_node_t *s_ptr = nullptr;
        if (next_idx < n_seg_list_sz)
        {
            s_ptr = seg_list[next_idx].ptr;
        }

        bool retval = fpln->delete_seg_end({s_ptr, id});

        if (!retval)
            return INVALID_DELETE_MSG;
        return "";
    }

    void CDU::get_seg_page(cdu_scr_data_t *in)
    {
        std::string via_to = " VIA" + std::string(N_CDU_DATA_COLS - 6, ' ') + "TO";
        in->data_lines.push_back(via_to);

        size_t i_start = 1 + N_CDU_ITM_PP * size_t(curr_subpg - 2);
        size_t i_end = std::min(seg_list.size() - 1, i_start + N_CDU_ITM_PP);

        for (size_t i = i_start; i < i_end; i++)
        {
            auto curr_sg = seg_list[i];
            test::leg_list_node_t *end_leg = curr_sg.data.end;
            std::string end_nm = "";
            std::string seg_nm = curr_sg.data.name;
            if (end_leg != nullptr)
            {
                end_nm = end_leg->data.leg.main_fix.id;
            }

            if (seg_nm == test::DISCON_SEG_NAME)
            {
                seg_nm = std::string(7, '-');
            }
            if (end_nm == "")
            {
                in->data_lines[in->data_lines.size() - 1] = std::string(10, ' ') + "THEN";
                end_nm = std::string(5, '@');
                std::string curr_seg = seg_nm + std::string(N_CDU_DATA_COLS - seg_nm.size() - end_nm.size(), ' ') + end_nm;
                in->data_lines.push_back(curr_seg);
                in->data_lines.push_back(DISCO_AFTER_SEG);
            }
            else
            {
                std::string seg_nm = curr_sg.data.name;
                std::string curr_seg = seg_nm + std::string(N_CDU_DATA_COLS - seg_nm.size() - end_nm.size(), ' ') + end_nm;
                in->data_lines.push_back(curr_seg);
                if (i < i_start + 4)
                    in->data_lines.push_back("");
            }
        }
        if (i_end - i_start < N_CDU_ITM_PP)
            in->data_lines.push_back(SEG_LAST);

        while (in->data_lines.size() < 10)
        {
            in->data_lines.push_back("");
        }
    }

    std::string CDU::get_sts(std::string &cr, std::string &act)
    {
        std::string sts = "<" + SEL + ">";
        if (cr == act)
            sts = "<" + ACT + ">";
        return sts;
    }

    void CDU::get_procs(cdu_scr_data_t *in, std::string curr_proc, std::string curr_trans,
                        std::string act_proc, std::string act_trans,
                        bool rte2)
    {
        size_t start_idx = size_t((curr_subpg - 1) * 5);
        size_t j = 1;

        if (curr_proc != "" && procs[rte2].size() == 1)
            start_idx = 0;

        for (size_t i = start_idx; i < start_idx + 6 && i < procs[rte2].size(); i++)
        {
            std::string curr = procs[rte2][i];
            if (curr == curr_proc)
            {
                curr = curr + std::string(6 - curr.size(), ' ') + get_sts(curr_proc, act_proc);
            }

            in->data_lines[j] = curr;
            j += 2;
        }
        if (trans[rte2].size() && procs[rte2].size() == 1)
        {
            size_t trans_start = size_t((curr_subpg - 1) * 4);
            in->data_lines[j - 1] = " TRANS";
            if (curr_trans == "")
                curr_trans = test::NONE_TRANS;
            for (size_t i = trans_start; i < trans_start + 4 && i < trans[rte2].size(); i++)
            {
                std::string curr = trans[rte2][i];
                if (curr == curr_trans)
                {
                    curr = curr + std::string(6 - curr.size(), ' ') + get_sts(curr_trans, act_trans);
                }
                in->data_lines[j] = curr;
                j += 2;
            }
        }
    }

    void CDU::get_rwys(cdu_scr_data_t *in, std::string curr_rwy, std::string act_rwy,
                       bool rte2, std::string curr_appr, std::string curr_via,
                       std::string act_appr, std::string act_via, bool get_appr)
    {
        size_t start_idx = size_t((curr_subpg - 1) * 5);
        size_t j = 1;

        bool draw_rwys = true;

        if (get_appr)
        {
            if ((dep_arr_rwy_filter[rte2] || dep_arr_proc_filter[rte2]) && curr_appr != "")
            {
                draw_rwys = false;
                start_idx = 0;
            }

            if (curr_appr != "")
                curr_rwy = "";
            for (size_t i = start_idx; i <= start_idx + N_CDU_ITM_PP && i < apprs[rte2].size(); i++)
            {
                std::string curr = apprs[rte2][i];
                if (curr == curr_appr)
                {
                    curr = get_sts(curr_appr, act_appr) +
                           std::string(7 - curr.size(), ' ') + curr;
                }
                in->data_lines[j] = get_cdu_line(curr, in->data_lines[j], true);
                j += 2;
            }

            if (apprs[rte2].size() == 1 && curr_appr != "")
            {
                size_t via_idx = 4 * (curr_subpg - 1);
                if (j - 1 > 0)
                    in->data_lines[j - 1] = get_cdu_line("TRANS ", in->data_lines[j - 1], true);
                for (size_t i = via_idx; i < via_idx + 4 && i < vias[rte2].size(); i++)
                {
                    std::string curr = vias[rte2][i];
                    if (curr == curr_via)
                    {
                        curr = get_sts(curr_via, act_via) +
                               std::string(7 - curr.size(), ' ') + curr;
                    }
                    in->data_lines[j] = get_cdu_line(curr, in->data_lines[j], true);
                    j += 2;
                }
            }
        }

        if (draw_rwys)
        {
            if (start_idx >= apprs[rte2].size())
            {
                start_idx -= apprs[rte2].size();
            }
            else
            {
                start_idx = 0;
            }
            if (j - 1 < in->data_lines.size() && get_appr)
            {
                if (j - 1)
                {
                    in->data_lines[j - 1] = ARR_RWYS;
                }
                else
                {
                    if (rte2)
                        in->data_lines[j - 1] = ARR_RWYS_STARS2;
                    else
                        in->data_lines[j - 1] = ARR_RWYS_STARS1;
                }
            }

            for (size_t i = start_idx; i <= start_idx + N_CDU_ITM_PP && i < rwys[rte2].size(); i++)
            {
                if (j > 11)
                    break;
                std::string curr = rwys[rte2][i];
                if (curr == curr_rwy)
                {
                    curr = get_sts(curr_rwy, act_rwy) +
                           std::string(7 - curr.size(), ' ') + curr;
                }
                in->data_lines[j] = get_cdu_line(curr, in->data_lines[j], true);
                j += 2;
            }
        }
    }

    std::string CDU::get_small_heading()
    {
        std::string curr_spg = std::to_string(curr_subpg);
        std::string n_spg = std::to_string(n_subpg);
        std::string out = curr_spg + "/" + n_spg + " ";
        out = std::string(size_t(N_CDU_DATA_COLS) - out.size(), ' ') + out;
        return out;
    }

    void CDU::set_procs(test::ProcType ptp, bool is_arr, bool rte2)
    {
        std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
        if (rte2)
        {
            c_fpl = m_rte2_ptr;
        }

        procs[rte2] = c_fpl->get_arpt_proc(ptp, is_arr,
                                           dep_arr_rwy_filter[rte2],
                                           dep_arr_proc_filter[rte2]);
        sort(procs[rte2].begin(), procs[rte2].end());
        if (ptp == test::PROC_TYPE_STAR)
        {
            apprs[rte2] = c_fpl->get_arpt_proc(test::PROC_TYPE_APPCH, is_arr,
                                               dep_arr_rwy_filter[rte2],
                                               dep_arr_proc_filter[rte2]);
            sort(apprs[rte2].begin(), apprs[rte2].end());
        }
        else
        {
            apprs[rte2] = {};
        }
        if (dep_arr_proc_filter[rte2])
        {
            if (!dep_arr_trans_filter[rte2])
            {
                trans[rte2] = c_fpl->get_arpt_proc_trans(ptp,
                                                         false, is_arr);
                sort(trans[rte2].begin(), trans[rte2].end());
            }
            else
            {
                trans[rte2] = {c_fpl->get_curr_proc(ptp, true)};
            }
            if (ptp == test::PROC_TYPE_STAR)
            {
                vias[rte2] = c_fpl->get_arpt_proc_trans(test::PROC_TYPE_APPCH, false, is_arr);
                sort(vias[rte2].begin(), vias[rte2].end());
            }
            else
            {
                vias[rte2] = {};
            }
        }
        else
        {
            trans[rte2] = {};
            vias[rte2] = {};
        }

        if (!is_arr)
            rwys[rte2] = c_fpl->get_dep_rwys(dep_arr_rwy_filter[rte2],
                                             dep_arr_proc_filter[rte2]);
        else
            rwys[rte2] = c_fpl->get_arr_rwys(dep_arr_rwy_filter[rte2],
                                             dep_arr_proc_filter[rte2]);
        sort(rwys[rte2].begin(), rwys[rte2].end());
    }

    void CDU::set_fpl_proc(int event, test::ProcType ptp, bool is_arr, bool rte2)
    {
        std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
        if (rte2)
        {
            c_fpl = m_rte2_ptr;
        }

        int start_idx = (curr_subpg - 1) * 5;
        int tr_idx = -1;
        int sz = procs[rte2].size();
        int trans_sz = trans[rte2].size();
        int curr_idx = 0;
        std::string curr_proc = c_fpl->get_curr_proc(ptp, false);
        if (curr_proc == "" || sz > 1)
        {
            curr_idx = start_idx + event - CDU_KEY_LSK_TOP;
        }
        else
        {
            if (event - CDU_KEY_LSK_TOP)
            {
                start_idx = (curr_subpg - 1) * 4;
                tr_idx = start_idx + event - CDU_KEY_LSK_TOP - 1;
            }
        }
        if (curr_idx < sz && tr_idx == -1)
        {
            c_fpl->set_arpt_proc(ptp, procs[rte2][size_t(curr_idx)], is_arr);
            dep_arr_rwy_filter[rte2] = !dep_arr_rwy_filter[rte2];
        }
        else if (tr_idx != -1 && tr_idx < trans_sz)
        {
            c_fpl->set_arpt_proc_trans(ptp, trans[rte2][size_t(tr_idx)], is_arr);
            dep_arr_trans_filter[rte2] = !dep_arr_trans_filter[rte2];
        }
    }

    void CDU::get_rte_dep_arr(cdu_scr_data_t &out, bool rte2)
    {
        size_t v_idx = test::RTE1_IDX;
        if (rte2)
            v_idx = test::RTE2_IDX;

        std::shared_ptr<test::FplnInt> cr_fpln = fpl_sys->fpl_vec[v_idx];

        std::string dep = cr_fpln->get_dep_icao();
        std::string arr = cr_fpln->get_arr_icao();

        std::string act_sts = " ";
        if (act_fpl_idx == v_idx)
            act_sts = "(ACT)";
        std::string hdg = "RTE 1";
        if (rte2)
            hdg = "RTE 2";
        hdg = hdg + act_sts;

        if (dep != "" || arr != "")
        {
            out.data_lines.push_back(std::string(8, ' ') + hdg);
            if (dep != "")
                out.data_lines.push_back(DEP_ARR_DEP_OPT + dep + DEP_ARR_ARR_OPT);
            else
                out.data_lines.push_back(DEP_ARR_DEP_OPT + "    " + DEP_ARR_ARR_OPT);
            out.data_lines.push_back("");
            if (arr != "")
                out.data_lines.push_back(std::string(DEP_ARR_DEP_OPT.size(), ' ') + arr + DEP_ARR_ARR_OPT);
            else
                out.data_lines.push_back(std::string(DEP_ARR_DEP_OPT.size(), ' ') + "    " + DEP_ARR_ARR_OPT);
        }
        else
        {
            out.data_lines.push_back(DEP_ARR_IDX_DASH_L + hdg + DEP_ARR_IDX_DASH_R);
            out.data_lines.push_back("");
            out.data_lines.push_back("");
            out.data_lines.push_back("");
        }
    }

    int CDU::get_n_sel_des_subpg()
    {
        return int(sel_des_data.size()) / 6 + bool(int(sel_des_data.size()) % 6);
    }

    int CDU::get_n_rte_subpg()
    {
        rte_copy = fpl_sys->act_can_copy();

        std::string dep_rwy = fpln->get_dep_rwy();
        if (dep_rwy != "")
        {
            size_t n_seg_act = n_seg_list_sz - 1;
            return 1 + (n_seg_act / N_CDU_ITM_PP) + bool(n_seg_act % N_CDU_ITM_PP);
        }
        return 1;
    }

    int CDU::get_n_dep_arr_subpg(bool rte2)
    {
        CDUPage c_dep_pg = CDUPage::DEP1;
        CDUPage c_arr_pg = CDUPage::ARR1;
        std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
        if (rte2)
        {
            c_dep_pg = CDUPage::DEP2;
            c_arr_pg = CDUPage::ARR2;
            c_fpl = m_rte2_ptr;
        }
        std::string dep_icao = c_fpl->get_dep_icao();
        std::string arr_icao = c_fpl->get_arr_icao();
        if ((curr_page == c_dep_pg && dep_icao == "") ||
            (curr_page == c_arr_pg && arr_icao == ""))
        {
            curr_page = CDUPage::DEP_ARR_INTRO;
        }

        if (curr_page == c_dep_pg)
        {
            set_procs(test::PROC_TYPE_SID, false, rte2);
            size_t max_cnt = std::max(rwys[rte2].size(),
                                      procs[rte2].size() + trans[rte2].size());
            return int(max_cnt) / N_DEP_ARR_ROW_DSP + bool(max_cnt % N_DEP_ARR_ROW_DSP);
        }
        else if (curr_page == c_arr_pg)
        {
            set_procs(test::PROC_TYPE_STAR, true, rte2);
            size_t max_cnt = std::max(procs[rte2].size() + trans[rte2].size(),
                                      apprs[rte2].size() + vias[rte2].size() +
                                          rwys[rte2].size());
            return int(max_cnt) / N_DEP_ARR_ROW_DSP + bool(max_cnt % N_DEP_ARR_ROW_DSP);
        }

        return 1;
    }

    int CDU::get_n_legs_subpg()
    {
        size_t n_leg_act = n_leg_list_sz - 2;
        return (n_leg_act / N_CDU_ITM_PP) + bool(n_leg_act % N_CDU_ITM_PP);
    }

    std::string CDU::handle_sel_des(int event_key)
    {
        int i_start = (curr_subpg - 1) * 6;
        int i_end = std::min(int(sel_des_data.size()), i_start + 6) - 1;
        int curr_idx = i_start + event_key - 1;
        if (curr_idx <= i_end)
        {
            sel_des_idx = curr_idx;
            sel_des = false;
            std::string tmp;
            curr_subpg = sel_des_subpg;
            on_event(sel_des_event, "", &tmp);
        }

        return "";
    }

    std::string CDU::handle_rte(int event_key, std::string scratchpad, std::string *s_out)
    {
        if (event_key == CDU_KEY_RSK_TOP + 5)
        {
            bool exec_lt = fpl_sys->get_exec();
            if (exec_lt)
                return "";
            if (sel_fpl_idx != act_fpl_idx)
            {
                fpl_sys->rte_activate(sel_fpl_idx);
            }
            return "";
        }
        else if (event_key == CDU_KEY_LSK_TOP + 5)
        {
            bool exec_lt = fpl_sys->get_exec();
            if (exec_lt)
            {
                fpl_sys->erase();
            }
            else
            {
                if (sel_fpl_idx == test::RTE1_IDX)
                    sel_fpl_idx = test::RTE2_IDX;
                else
                    sel_fpl_idx = test::RTE1_IDX;
            }

            return "";
        }
        if (curr_subpg == 1)
        {
            if (event_key == CDU_KEY_LSK_TOP)
            {
                return set_departure(scratchpad, s_out);
            }
            else if (event_key == CDU_KEY_RSK_TOP)
            {
                return set_arrival(scratchpad, s_out);
            }
            else if (event_key == CDU_KEY_RSK_TOP + 1)
            {
                return set_flt_nbr(scratchpad);
            }
            else if (event_key == CDU_KEY_RSK_TOP + 3)
            {
                if (rte_copy == test::RTECopySts::READY && sel_fpl_idx == act_fpl_idx)
                    fpl_sys->copy_act();
            }
            else if (event_key == CDU_KEY_LSK_TOP + 1)
            {
                return set_dep_rwy(scratchpad);
            }
            else if (event_key == CDU_KEY_LSK_TOP + 2)
            {
                return load_rte();
            }
            else if (event_key == CDU_KEY_LSK_TOP + 4)
            {
                return save_rte();
            }
        }
        else
        {
            size_t i_start = 1 + N_CDU_ITM_PP * size_t(curr_subpg - 2);
            size_t i_end = std::min(n_seg_list_sz - 1, i_start + N_CDU_ITM_PP);
            size_t i_event = i_start + size_t(event_key - 1) % 6;
            if (i_event > i_end || (i_event == i_end && i_event < n_seg_list_sz - 1))
            {
                return INVALID_ENTRY_MSG;
            }
            else
            {
                if (event_key >= CDU_KEY_RSK_TOP)
                {
                    if (!scratchpad_has_delete(scratchpad))
                    {
                        return add_to(i_event + 1, scratchpad);
                    }

                    return delete_to(i_event);
                }
                else
                {
                    if (!scratchpad_has_delete(scratchpad))
                        return add_via(i_event + 1, scratchpad);
                    return delete_via(i_event);
                }
            }
        }

        return "";
    }

    std::string CDU::handle_dep_arr(int event_key)
    {
        std::shared_ptr<test::FplnInt> rte1 = fpl_sys->fpl_vec[test::RTE1_IDX];
        std::shared_ptr<test::FplnInt> rte2 = fpl_sys->fpl_vec[test::RTE1_IDX];
        std::string dep1 = rte1->get_dep_icao();
        std::string arr1 = rte1->get_arr_icao();
        std::string dep2 = rte2->get_dep_icao();
        std::string arr2 = rte2->get_arr_icao();

        if (dep1 != "" && arr1 != "")
        {
            if (event_key == CDU_KEY_LSK_TOP)
            {
                curr_page = CDUPage::DEP1;
            }
            else if (event_key == CDU_KEY_RSK_TOP + 1)
            {
                curr_page = CDUPage::ARR1;
            }
        }
        if (dep2 != "" && arr2 != "")
        {
            if (event_key == CDU_KEY_LSK_TOP + 2)
            {
                curr_page = CDUPage::DEP2;
            }
            else if (event_key == CDU_KEY_RSK_TOP + 3)
            {
                curr_page = CDUPage::ARR2;
            }
        }

        return "";
    }

    std::string CDU::handle_dep(int event_key, bool rte2)
    {
        if (event_key == CDU_KEY_LSK_TOP + 5)
        {
            bool exec_lt = fpl_sys->get_exec();
            if (exec_lt)
            {
                fpl_sys->erase();
            }
            else
            {
                set_page(CDUPage::INIT_REF);
            }
        }
        else if (event_key == CDU_KEY_RSK_TOP + 5)
        {
            set_page(CDUPage::RTE);
        }
        else if (event_key && event_key < CDU_KEY_LSK_TOP + 5)
        {
            set_fpl_proc(event_key, test::PROC_TYPE_SID, false, rte2);
        }
        else if (event_key >= CDU_KEY_RSK_TOP && event_key < CDU_KEY_RSK_TOP + 5)
        {
            std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
            if (rte2)
            {
                c_fpl = m_rte2_ptr;
            }

            int start_idx = (curr_subpg - 1) * 5;
            int sz = rwys[rte2].size();
            int curr_idx = start_idx + event_key - CDU_KEY_RSK_TOP;
            if (curr_idx < sz)
            {
                c_fpl->set_dep_rwy(rwys[rte2][size_t(curr_idx)]);
                dep_arr_proc_filter[rte2] = !dep_arr_proc_filter[rte2];
            }
        }
        return "";
    }

    std::string CDU::handle_arr(int event_key, bool rte2)
    {
        if (event_key == CDU_KEY_LSK_TOP + 5)
        {
            bool exec_lt = fpl_sys->get_exec();
            if (exec_lt)
            {
                fpl_sys->erase();
            }
            else
            {
                set_page(CDUPage::INIT_REF);
            }
        }
        else if (event_key == CDU_KEY_RSK_TOP + 5)
        {
            set_page(CDUPage::RTE);
        }
        else if (event_key && event_key < CDU_KEY_LSK_TOP + 5)
        {
            set_fpl_proc(event_key, test::PROC_TYPE_STAR, true, rte2);
        }
        else if (event_key >= CDU_KEY_RSK_TOP && event_key < CDU_KEY_RSK_TOP + 5)
        {
            int start_idx = (curr_subpg - 1) * 5;
            int curr_idx = start_idx + event_key - CDU_KEY_RSK_TOP;

            std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
            if (rte2)
            {
                c_fpl = m_rte2_ptr;
            }

            if (curr_idx < int(apprs[rte2].size()))
            {
                c_fpl->set_arpt_proc(test::PROC_TYPE_APPCH,
                                     apprs[rte2][size_t(curr_idx)], true);
                dep_arr_proc_filter[rte2] = !dep_arr_proc_filter[rte2];
            }
            else if (curr_idx >= int(apprs[rte2].size()))
            {
                curr_idx -= int(apprs[rte2].size());
                if (curr_idx < int(vias[rte2].size()))
                {
                    c_fpl->set_arpt_proc_trans(test::PROC_TYPE_APPCH, vias[rte2][size_t(curr_idx)],
                                               true);
                    dep_arr_via_filter[rte2] = !dep_arr_via_filter[rte2];
                }
                else if (curr_idx < int(rwys.size()))
                {
                    c_fpl->set_arr_rwy(rwys[rte2][size_t(curr_idx)]);
                    dep_arr_proc_filter[rte2] = !dep_arr_proc_filter[rte2];
                }
            }
        }
        return "";
    }

    size_t CDU::get_leg_stt_idx()
    {
        return 2 + N_CDU_ITM_PP * size_t(curr_subpg - 1);
    }

    size_t CDU::get_leg_end_idx()
    {
        size_t stt_idx = get_leg_stt_idx();
        return std::min(leg_list.size() - 1, stt_idx + N_CDU_ITM_PP);
    }

    void CDU::reset_leg_dto_sel(size_t fp_idx)
    {
        assert(fp_idx < N_CDU_RTES);
        leg_sel[fp_idx].second = -1;
    }

    void CDU::reset_leg_all_sel()
    {
        for (size_t i = 0; i < N_CDU_RTES; i++)
        {
            reset_leg_dto_sel(i);
        }
    }

    bool CDU::handle_legs_dto(size_t usr_idx, std::string scratchpad, std::string *s_out)
    {
        size_t sd_idx = sel_fpl_idx - 1;
        if (leg_sel[sd_idx].second == -1)
        {
            bool is_discon = leg_list[usr_idx].data.is_discon;
            if(scratchpad == "" && is_discon)
                return 0;
            if(scratchpad != "") // User might be trying to insert a waypoint.
                return 1;
            leg_sel[sd_idx].first = usr_idx;
            leg_sel[sd_idx].second = fpl_infos[sel_fpl_idx].leg_list_id;
            if (leg_list[usr_idx].data.is_discon == false)
                *s_out = get_cdu_leg_nm(leg_list[usr_idx]);
        }
        else
        {
            size_t i_fr = usr_idx;
            size_t i_to = leg_sel[sd_idx].first;
            if (get_cdu_leg_nm(leg_list[i_to]) != scratchpad)
            {
                reset_leg_dto_sel(sd_idx);
                return 1;
            }
            if (i_fr != i_to)
            {
                if (i_fr > i_to)
                {
                    if (i_fr + 1 < leg_list.size())
                        i_fr++;
                    std::swap(i_fr, i_to);
                }
                else if (i_fr)
                {
                    i_fr--;
                }

                double cr_lg_id = leg_sel[sd_idx].second;
                bool rv = fpln->dir_from_to({leg_list[i_fr].ptr, cr_lg_id},
                                            {leg_list[i_to].ptr, cr_lg_id});
                reset_leg_dto_sel(sd_idx);
                UNUSED(rv);
            }
        }
        return 0;
    }

    std::string CDU::handle_legs_insert(size_t usr_idx, std::string scratchpad)
    {
        test::fpln_info_t f_inf = fpl_infos[sel_fpl_idx];
        double sg_id = f_inf.seg_list_id;
        double lg_id = f_inf.leg_list_id;

        bool inv_ent = 0, not_in_db = 0, sel_used = 0;
        leg_sel_pr = 0;
        libnav::waypoint_t tgt_wpt = get_wpt_from_user(scratchpad, sg_id, lg_id, &not_in_db, &inv_ent, 
            &leg_sel_pr, &sel_used);
        if(inv_ent)
            return INVALID_ENTRY_MSG;
        if(not_in_db)
            return NOT_IN_DB_MSG;
        if(leg_sel_pr)
            return "";
        
        if(sel_used)
        {
            lg_id = sel_des_leg_id; // Only need legs id here
        }

        fpln->add_direct(tgt_wpt, {leg_list[usr_idx].ptr, lg_id});
        return "";
    }

    std::string CDU::handle_legs_delete(size_t usr_idx)
    {
        test::fpln_info_t f_inf = fpl_infos[sel_fpl_idx];
        double lg_id = f_inf.leg_list_id;

        bool retval = fpln->delete_leg({leg_list[usr_idx].ptr, lg_id});
        if(!retval)
            return INVALID_DELETE_MSG;
        return "";
    }

    std::string CDU::handle_legs(int event_key, std::string scratchpad, std::string *s_out)
    {
        if (event_key == CDU_KEY_LSK_TOP + 5)
        {
            bool exec_lt = fpl_sys->get_exec();
            if (exec_lt)
            {
                fpl_sys->erase();
            }
            else
            {
                if (sel_fpl_idx == test::RTE1_IDX)
                    sel_fpl_idx = test::RTE2_IDX;
                else
                    sel_fpl_idx = test::RTE1_IDX;
            }
        }
        else if (event_key == CDU_KEY_RSK_TOP + 5)
        {
            bool exec_lt = fpl_sys->get_exec();
            if (exec_lt)
                return "";
            if (sel_fpl_idx != act_fpl_idx)
            {
                fpl_sys->rte_activate(sel_fpl_idx);
            }
            return "";
        }
        else if (event_key >= CDU_KEY_LSK_TOP && event_key <= CDU_KEY_LSK_TOP + 5)
        {
            size_t i_start = get_leg_stt_idx();
            size_t i_end = get_leg_end_idx();
            size_t usr_idx = i_start + size_t(event_key - CDU_KEY_LSK_TOP);
            bool scr_is_del = scratchpad_has_delete(scratchpad);
            bool is_ins = 0;
            if((!scr_is_del && usr_idx == i_end) || leg_sel_pr)
                is_ins = 1;
            if(!is_ins)
            {
                if (usr_idx < i_end && !scr_is_del)
                {
                    is_ins = handle_legs_dto(usr_idx, scratchpad, s_out);
                }
                else if (scr_is_del)
                {
                    if(usr_idx < i_end)
                        return handle_legs_delete(usr_idx);
                    else
                        return INVALID_DELETE_MSG;
                }
            }
            if(is_ins)
            {
                return handle_legs_insert(usr_idx, scratchpad);
            }
        }
        return "";
    }

    cdu_scr_data_t CDU::get_sel_des_page()
    {
        cdu_scr_data_t out = {};
        fill_char_state_buf(out);

        out.heading_small = get_small_heading();
        out.heading_big = SEL_DES_WPT_HDG;
        out.heading_color = CDUColor::WHITE;

        size_t start_idx = size_t((curr_subpg - 1) * 6);
        size_t end_idx = std::min(sel_des_data.size(), start_idx + 6);

        for (size_t i = start_idx; i < end_idx; i++)
        {
            std::string wpt_tp = sel_des_nm + " " + libnav::navaid_to_str(sel_des_data[i].type);
            std::string lat_str = strutils::lat_to_str(sel_des_data[i].pos.lat_rad * geo::RAD_TO_DEG);
            std::string lon_str = strutils::lon_to_str(sel_des_data[i].pos.lon_rad * geo::RAD_TO_DEG);
            std::string main_str = lat_str + lon_str;
            if (sel_des_data[i].navaid)
            {
                main_str = strutils::freq_to_str(sel_des_data[i].navaid->freq) + " " + main_str;
            }
            out.data_lines.push_back(wpt_tp);
            out.data_lines.push_back(main_str);
        }

        return out;
    }

    cdu_scr_data_t CDU::get_rte_page()
    {
        bool exec_lt = fpl_sys->get_exec();

        cdu_scr_data_t out = {};
        fill_char_state_buf(out);

        out.heading_small = get_small_heading();
        std::string rte_offs = std::string(2, ' ');
        std::string act_sts = std::string(4, ' ');

        out.heading_color = CDUColor::CYAN;
        if (sel_fpl_idx == act_fpl_idx)
        {
            if (exec_lt)
                act_sts = MOD + " ";
            else
                act_sts = ACT + " ";
            out.heading_color = CDUColor::WHITE;
        }
        std::string c_rte_top = "RTE 1";
        std::string c_rte_btm = "<RTE 2";
        if (sel_fpl_idx == test::RTE2_IDX)
        {
            c_rte_top = "RTE 2";
            c_rte_btm = "<RTE 1";
        }
        out.heading_big = rte_offs + act_sts + c_rte_top;

        if (curr_subpg == 1)
        {
            std::string dest_offs = std::string(N_CDU_DATA_COLS - 7 - 4, ' ');
            std::string origin_dest = " ORIGIN" + dest_offs + "DEST";
            out.data_lines.push_back(origin_dest);
            std::string origin = fpln->get_dep_icao();
            std::string dest = fpln->get_arr_icao();
            bool incomplete = false;

            std::string flt_nbr = fpl_sys->get_flt_nbr();
            if (flt_nbr == "")
                flt_nbr = std::string(10, '-');
            else
                flt_nbr = std::string(N_FLT_NBR_CHR_MAX - flt_nbr.size(), ' ') + flt_nbr;

            if (origin == "")
            {
                origin = std::string(4, '@');
                incomplete = true;
            }
            if (dest == "")
            {
                dest = std::string(4, '@');
                incomplete = true;
            }

            std::string od_data = origin + std::string(N_CDU_DATA_COLS - 4 - 4, ' ') + dest;
            out.data_lines.push_back(od_data);
            std::string rwy_flt_no = " RUNWAY" + std::string(N_CDU_DATA_COLS - 7 - 6, ' ') + "FLT NO";
            out.data_lines.push_back(rwy_flt_no);
            std::string dep_rwy = "";
            if (!incomplete)
            {
                dep_rwy = fpln->get_dep_rwy();
                if (dep_rwy == "")
                    dep_rwy = std::string(5, '-');
                else
                    dep_rwy = "RW" + dep_rwy;
            }
            std::string rf_data = dep_rwy + std::string(size_t(N_CDU_DATA_COLS) - dep_rwy.size() - 10, ' ') + flt_nbr;
            out.data_lines.push_back(rf_data);
            out.data_lines.push_back(" ROUTE" + std::string(N_CDU_DATA_COLS - 6 - 8, ' ') + "CO ROUTE");
            out.data_lines.push_back("<REQUEST" + std::string(size_t(N_CDU_DATA_COLS) - 8 - 10, ' ') + std::string(10, '-'));
            if (rte_copy == test::RTECopySts::READY && sel_fpl_idx == act_fpl_idx)
            {
                out.data_lines.push_back("");
                size_t cp_pad = size_t(N_CDU_DATA_COLS) - RTE_COPY.size() - 1;
                out.data_lines.push_back(std::string(cp_pad, ' ') + RTE_COPY + ">");
            }
            else if (rte_copy == test::RTECopySts::COMPLETE && sel_fpl_idx == act_fpl_idx)
            {
                size_t cp_pad1 = size_t(N_CDU_DATA_COLS) - RTE_COPY.size();
                size_t cp_pad2 = size_t(N_CDU_DATA_COLS) - COMPLETE.size();
                out.data_lines.push_back(std::string(cp_pad1, ' ') + RTE_COPY);
                out.data_lines.push_back(std::string(cp_pad2, ' ') + COMPLETE);
            }
            else
            {
                out.data_lines.push_back("");
                out.data_lines.push_back("");
            }

            std::string rte_final = " ROUTE ";
            out.data_lines.push_back(rte_final + std::string(size_t(N_CDU_DATA_COLS) - 7, '-'));
            out.data_lines.push_back("<SAVE" + std::string(size_t(N_CDU_DATA_COLS) - 10, ' ') + "ALTN>");
        }
        else
        {
            get_seg_page(&out);
        }

        if (out.data_lines.size() == 10)
        {
            if (curr_subpg != 1)
                out.data_lines.push_back(ALL_DASH);
            else
                out.data_lines.push_back("");
        }

        std::string btm_line_nrm = c_rte_btm;
        if (act_fpl_idx != sel_fpl_idx)
            btm_line_nrm += std::string(size_t(N_CDU_DATA_COLS) - 15, ' ') + "ACTIVATE>";
        if (!exec_lt)
            out.data_lines.push_back(btm_line_nrm);
        else
            out.data_lines.push_back(ERASE_NML);

        return out;
    }

    cdu_scr_data_t CDU::get_dep_arr_page()
    {
        cdu_scr_data_t out = {};
        fill_char_state_buf(out);

        out.heading_small = get_small_heading();
        std::string hdg_offs = std::string((N_CDU_DATA_COLS - DEP_ARR_HDG.size()) / 2, ' ');
        out.heading_big = hdg_offs + DEP_ARR_HDG;
        out.heading_color = CDUColor::WHITE;

        get_rte_dep_arr(out, false);
        get_rte_dep_arr(out, true);

        out.data_lines.push_back(std::string(N_CDU_DATA_COLS, '-'));
        out.data_lines.push_back("");
        out.data_lines.push_back(DEP_ARR_IDX_OTHER);
        out.data_lines.push_back(DEP_ARR_ARROWS);

        return out;
    }

    void CDU::dep_arr_set_bottom(cdu_scr_data_t &out)
    {
        out.data_lines[10] = std::string(N_CDU_DATA_COLS, '-');

        bool exec_lt = fpl_sys->get_exec();
        if (exec_lt)
        {
            out.data_lines[11] = DEP_ARR_BOTTOM_ACT;
        }
        else
        {
            out.data_lines[11] = DEP_ARR_BOTTOM_INACT;
        }
    }

    cdu_scr_data_t CDU::get_dep_page(bool rte2)
    {
        std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
        if (rte2)
        {
            c_fpl = m_rte2_ptr;
        }

        std::string dep = c_fpl->get_dep_icao();
        cdu_scr_data_t out = {};
        fill_char_state_buf(out);

        out.heading_small = get_small_heading();
        out.heading_big = "   " + dep + " DEPARTURES";
        out.heading_color = CDUColor::WHITE;

        for (int i = 0; i < 12; i++)
        {
            out.data_lines.push_back("");
        }
        if (rte2)
            out.data_lines[0] = DEP_COLS2;
        else
            out.data_lines[0] = DEP_COLS1;

        std::string curr_sid = c_fpl->get_curr_proc(test::PROC_TYPE_SID);
        std::string curr_trans = c_fpl->get_curr_proc(test::PROC_TYPE_SID, true);
        std::string act_sid = m_act_ptr->get_curr_proc(test::PROC_TYPE_SID);
        std::string act_trans = m_act_ptr->get_curr_proc(test::PROC_TYPE_SID, true);
        get_procs(&out, curr_sid, curr_trans, act_sid, act_trans, rte2);

        std::string dep_rwy = c_fpl->get_dep_rwy();
        std::string act_rwy = m_act_ptr->get_dep_rwy();
        get_rwys(&out, dep_rwy, act_rwy, rte2);

        dep_arr_set_bottom(out);

        return out;
    }

    cdu_scr_data_t CDU::get_arr_page(bool rte2)
    {
        std::shared_ptr<test::FplnInt> c_fpl = m_rte1_ptr;
        if (rte2)
        {
            c_fpl = m_rte2_ptr;
        }

        std::string arr = c_fpl->get_arr_icao();
        cdu_scr_data_t out = {};
        fill_char_state_buf(out);

        out.heading_small = get_small_heading();
        out.heading_big = "   " + arr + " ARRIVALS";
        out.heading_color = CDUColor::WHITE;

        for (int i = 0; i < 12; i++)
        {
            out.data_lines.push_back("");
        }
        if (rte2)
            out.data_lines[0] = ARR_COLS2;
        else
            out.data_lines[0] = ARR_COLS1;

        std::string curr_star = c_fpl->get_curr_proc(test::PROC_TYPE_STAR);
        std::string curr_trans = c_fpl->get_curr_proc(test::PROC_TYPE_STAR, true);
        std::string act_star = m_act_ptr->get_curr_proc(test::PROC_TYPE_STAR);
        std::string act_trans = m_act_ptr->get_curr_proc(test::PROC_TYPE_STAR, true);
        get_procs(&out, curr_star, curr_trans, act_star, act_trans, rte2);

        std::string arr_rwy = c_fpl->get_arr_rwy();
        std::string arr_appr = c_fpl->get_curr_proc(test::PROC_TYPE_APPCH);
        std::string arr_via = c_fpl->get_curr_proc(test::PROC_TYPE_APPCH, true);

        std::string act_rwy = m_act_ptr->get_arr_rwy();
        std::string act_appr = m_act_ptr->get_curr_proc(test::PROC_TYPE_APPCH);
        std::string act_via = m_act_ptr->get_curr_proc(test::PROC_TYPE_APPCH, true);

        get_rwys(&out, arr_rwy, act_rwy, rte2, arr_appr, arr_via, act_appr, act_via, true);

        dep_arr_set_bottom(out);

        return out;
    }

    std::string CDU::get_legs_btm()
    {
        bool is_act = sel_fpl_idx == act_fpl_idx;
        bool exec_lt = fpl_sys->get_exec();

        std::string c_legs_btm = "<RTE 2";
        if (sel_fpl_idx == test::RTE2_IDX)
        {
            c_legs_btm = "<RTE 1";
        }

        if (!is_act && !exec_lt)
        {
            return c_legs_btm + LEGS_BTM_INACT;
        }
        else if (is_act && exec_lt)
        {
            return LEGS_BTM_MOD;
        }
        return c_legs_btm + LEGS_BTM_ACT;
    }

    cdu_scr_data_t CDU::get_legs_page()
    {
        cdu_scr_data_t out = {};
        fill_char_state_buf(out);

        out.heading_small = get_small_heading();
        out.heading_color = CDUColor::WHITE;

        out.heading_color = CDUColor::CYAN;
        bool exec_lt = fpl_sys->get_exec();
        std::string act_sts = std::string(4, ' ');
        if (sel_fpl_idx == act_fpl_idx)
        {
            if (exec_lt)
                act_sts = MOD + " ";
            else
                act_sts = ACT + " ";
            out.heading_color = CDUColor::WHITE;
        }
        std::string c_legs_top = "RTE 1 LEGS";
        if (sel_fpl_idx == test::RTE2_IDX)
        {
            c_legs_top = "RTE 2 LEGS";
        }
        out.heading_big = "  " + act_sts + c_legs_top;

        assert(leg_list.size());
        size_t i_start = get_leg_stt_idx();
        size_t i_end = get_leg_end_idx();
        bool disc_pr = false;
        size_t sts_idx = 1;

        test::act_leg_info_t act_info = fpl_sys->get_act_leg_info();

        for (size_t i = i_start; i < i_end; i++)
        {
            if (!disc_pr)
                out.data_lines.push_back(get_cdu_leg_prop(leg_list[i]));
            if (leg_list[i].data.is_discon)
            {
                disc_pr = true;
                out.data_lines.push_back("@@@@@");
                out.data_lines.push_back(DISCO_AFTER_SEG);
            }
            else
            {
                disc_pr = false;

                std::string cr_name = get_cdu_leg_nm(leg_list[i]);
                if (cr_name == act_info.name)
                {
                    for (size_t j = 0; j < 5; j++)
                        out.chr_sts[sts_idx][j] = CDU_B_MAGENTA;
                }
                // get leg constraints
                std::string spdcstr = get_cdu_leg_spdcstr(leg_list[i]);
                std::string vcstr = get_cdu_leg_vcstr(leg_list[i]);
                if (vcstr.size() < N_LEG_VCSTR_ROWS)
                    vcstr = std::string(N_LEG_VCSTR_ROWS - vcstr.size(), ' ') + vcstr;
                std::string cstr = spdcstr + "/" + vcstr;

                cr_name = cr_name + std::string(N_CDU_DATA_COLS - cr_name.size() - cstr.size(), ' ') + cstr;
                out.data_lines.push_back(cr_name);
            }
            sts_idx += 2;
        }

        if (i_end - i_start < N_CDU_ITM_PP)
        {
            out.data_lines.push_back("");
            out.data_lines.push_back(LEG_LAST);
        }

        while (out.data_lines.size() < 10)
        {
            out.data_lines.push_back("");
        }

        while (out.data_lines.size() > 10)
        {
            out.data_lines.pop_back();
        }

        out.data_lines.push_back(std::string(N_CDU_DATA_COLS, '-'));
        out.data_lines.push_back(get_legs_btm());

        return out;
    }

    // CDUDisplay definitions:
    // Public member functions:

    CDUDisplay::CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, cairo_font_face_t *ff,
                           std::shared_ptr<cairo_utils::texture_manager_t> tm,
                           std::shared_ptr<CDU> cdu, byteutils::Bytemap *bm)
    {
        scr_pos = pos;
        size = sz;
        disp_pos = scr_pos + size * DISPLAY_OFFS;
        disp_size = size * DISPLAY_SZ;

        font_face = ff;

        tex_mngr = tm;
        cdu_ptr = cdu;

        key_map = bm;

        tex_size = cairo_utils::get_surf_sz(tex_mngr->data[CDU_TEXTURE_NAME]);
        tex_scale = size / tex_size;

        scratchpad = std::string(size_t(N_CDU_DATA_COLS), ' ');
        scratch_curr = 0;

        last_press_tp = std::chrono::steady_clock::now();
    }

    void CDUDisplay::on_click(geom::vect2_t pos)
    {
        auto curr_tp = std::chrono::steady_clock::now();
        std::chrono::duration<double> dur = curr_tp - last_press_tp;
        double dur_cnt = dur.count();

        last_press_tp = curr_tp;

        if (dur_cnt < CDU_PRS_INTV_SEC)
            return;

        pos = (pos - scr_pos) / tex_scale;
        if (pos.x >= 0 && pos.y >= 0 && pos.x < tex_size.x && pos.y < tex_size.y)
        {
            int event = int(key_map->get_at(size_t(pos.x), size_t(pos.y)));

            if (event && (event < CDU_KEY_A || event == CDU_KEY_EXEC))
            {
                std::string scratch_proc = strutils::strip(scratchpad);
                std::string scr_out;
                std::string msg = cdu_ptr->on_event(event, scratch_proc, &scr_out);

                if (scr_out != "")
                {
                    scratch_curr = scr_out.size();
                    scratchpad = scr_out + std::string(N_CDU_DATA_COLS - scr_out.size(), ' ');
                }
                else
                {
                    if (msg == "")
                        clear_scratchpad();
                    else
                        msg_stack.push(msg);
                }
            }
            update_scratchpad(event);
        }
    }

    void CDUDisplay::draw(cairo_t *cr)
    {
        cairo_utils::draw_image(cr, tex_mngr->data[CDU_TEXTURE_NAME], scr_pos,
                                tex_scale, false);

        bool dr_exc = cdu_ptr->get_exec_lt();
        if (dr_exc)
            draw_exec(cr);

        draw_screen(cr);
    }

    // Private member functions:

    void CDUDisplay::add_to_scratchpad(char c)
    {
        if (scratch_curr != size_t(N_CDU_DATA_COLS))
        {
            scratchpad[scratch_curr] = c;
            scratch_curr++;
        }
    }

    void CDUDisplay::clear_scratchpad()
    {
        while (scratch_curr)
        {
            scratchpad[scratch_curr] = ' ';
            scratch_curr--;
        }
        scratchpad[scratch_curr] = ' ';
    }

    void CDUDisplay::update_scratchpad(int event)
    {
        if (event == CDU_KEY_DELETE)
        {
            if (scratchpad[0] == DELETE_SYMBOL)
            {
                scratchpad[0] = ' ';
            }
            else
            {
                clear_scratchpad();
                scratchpad[0] = DELETE_SYMBOL;
            }
        }
        else if (event == CDU_KEY_CLR)
        {
            if (msg_stack.size())
            {
                msg_stack.pop();
            }
            else
            {
                if (scratch_curr)
                    scratch_curr--;
                scratchpad[scratch_curr] = ' ';
            }
        }

        if (scratchpad[0] != DELETE_SYMBOL)
        {
            if (event >= CDU_KEY_A && event < CDU_KEY_A + 26)
            {
                add_to_scratchpad('A' + char(event - CDU_KEY_A));
            }
            else if (event == CDU_KEY_SP)
            {
                add_to_scratchpad(' ');
            }
            else if (event == CDU_KEY_SLASH)
            {
                add_to_scratchpad('/');
            }
            else if (event >= CDU_KEY_1 && event < CDU_KEY_1 + 9)
            {
                add_to_scratchpad('1' + char(event - CDU_KEY_1));
            }
            else if (event == CDU_KEY_DOT)
            {
                add_to_scratchpad('.');
            }
            else if (event == CDU_KEY_0)
            {
                add_to_scratchpad('0');
            }
            else if (event == CDU_KEY_PM)
            {
                if (scratch_curr && scratchpad[scratch_curr - 1] == '-')
                    scratchpad[scratch_curr - 1] = '+';
                else
                    add_to_scratchpad('-');
            }
        }
    }

    int CDUDisplay::get_cdu_letter_idx(char c)
    {
        if (c >= '0' && c <= '9')
            return 1 + c - '0';
        else if (c >= 'A' && c <= 'Z')
            return 11 + c - 'A';
        else if (c == '%')
            return 37;
        else if (c == '(')
            return 38;
        else if (c == ')')
            return 39;
        else if (c == '-')
            return 40;
        else if (c == '_')
            return 41;
        else if (c == '+')
            return 42;
        else if (c == '=')
            return 43;
        else if (c == '|')
            return 44;
        else if (c == ':')
            return 45;
        else if (c == '<')
            return 46;
        else if (c == '.')
            return 47;
        else if (c == '>')
            return 48;
        else if (c == ',')
            return 49;
        else if (c == '/')
            return 50;
        else if (c == strutils::DEGREE_SYMBOL)
            return 51;
        else if (c == '@')
            return 52;
        return 0;
    }

    CDUColor CDUDisplay::get_cdu_color(char c)
    {
        if (c == CDU_S_WHITE || c == CDU_B_WHITE)
            return CDUColor::WHITE;
        else if (c == CDU_S_CYAN || c == CDU_B_CYAN)
            return CDUColor::CYAN;
        else if (c == CDU_S_GREEN || c == CDU_B_GREEN)
            return CDUColor::GREEN;

        return CDUColor::MAGENTA;
    }

    bool CDUDisplay::chr_is_big(char c)
    {
        if (c >= 'a' && c <= 'z')
            return false;
        return true;
    }

    cairo_surface_t *CDUDisplay::get_font_sfc(CDUColor cl)
    {
        cairo_surface_t *font_sfc;
        if (cl == CDUColor::GREEN)
            font_sfc = tex_mngr->data[CDU_GREEN_TEXT_NAME];
        else if (cl == CDUColor::CYAN)
            font_sfc = tex_mngr->data[CDU_CYAN_TEXT_NAME];
        else if (cl == CDUColor::MAGENTA)
            font_sfc = tex_mngr->data[CDU_MAGENTA_TEXT_NAME];
        else
            font_sfc = tex_mngr->data[CDU_WHITE_TEXT_NAME];

        return font_sfc;
    }

    void CDUDisplay::draw_cdu_letter(cairo_t *cr, char c, geom::vect2_t pos,
                                     geom::vect2_t scale, cairo_surface_t *font_sfc)
    {
        if (scale.x == 0 || scale.y == 0)
            return;

        int idx = get_cdu_letter_idx(c);
        geom::vect2_t offs = {-CDU_LETTER_WIDTH * scale.x * double(idx), 0};
        geom::vect2_t img_pos = pos + offs;
        cairo_save(cr);
        cairo_scale(cr, scale.x, scale.y);
        cairo_set_source_surface(cr, font_sfc, img_pos.x / scale.x, img_pos.y / scale.y);
        cairo_rectangle(cr, pos.x / scale.x, pos.y / scale.y,
                        CDU_LETTER_WIDTH, CDU_LETTER_HEIGHT);
        cairo_clip(cr);
        cairo_paint(cr);
        cairo_restore(cr);
    }

    void CDUDisplay::draw_cdu_line(cairo_t *cr, std::string &s, geom::vect2_t pos,
                                   double l_intv_px, std::string sts, geom::vect2_t scale, CDUColor clr)
    {
        if (sts != "")
            assert(sts.size() >= s.size());

        cairo_surface_t *sfc_const = get_font_sfc(clr);
        geom::vect2_t sc_big = CDU_BIG_TEXT_SZ;
        geom::vect2_t sc_sml = CDU_SMALL_TEXT_SZ;

        double res_ratio = size.y * CDU_RES_COEFF;
        scale = scale.scmul(res_ratio);
        sc_big = sc_big.scmul(res_ratio);
        sc_sml = sc_sml.scmul(res_ratio);

        for (size_t i = 0; i < s.size(); i++)
        {
            cairo_surface_t *sfc = sfc_const;
            geom::vect2_t sc_cr = scale;
            if (sts != "")
            {
                sfc = get_font_sfc(get_cdu_color(sts[i]));
                if (chr_is_big(sts[i]))
                    sc_cr = sc_big;
                else
                    sc_cr = sc_sml;
            }
            draw_cdu_letter(cr, s[i], pos, sc_cr, sfc);
            pos.x += l_intv_px;
        }
    }

    void CDUDisplay::draw_exec(cairo_t *cr)
    {
        geom::vect2_t lt_pos = {scr_pos.x + size.x * EXEC_LT_POS.x,
                                scr_pos.y + size.y * EXEC_LT_POS.y};
        geom::vect2_t lt_sz = {size.x * EXEC_LT_SZ.x, size.y * EXEC_LT_SZ.y};

        cairo_utils::draw_rect(cr, lt_pos, lt_sz, EXEC_LT_CLR);
    }

    void CDUDisplay::draw_screen(cairo_t *cr)
    {
        geom::vect2_t offs_hdg_small = {0, disp_size.y * CDU_V_OFFS_SMALL_FIRST};
        geom::vect2_t pos_hdg_small = disp_pos + offs_hdg_small;
        geom::vect2_t small_offs = {0, disp_size.y * CDU_V_OFFS_FIRST};
        geom::vect2_t pos_small = disp_pos + small_offs;
        geom::vect2_t big_offs = {0, disp_size.y * (CDU_BIG_TEXT_OFFS + CDU_V_OFFS_FIRST)};
        geom::vect2_t pos_big = disp_pos + big_offs;

        cdu_scr_data_t curr_screen = cdu_ptr->get_screen_data();

        draw_cdu_line(cr, curr_screen.heading_big, disp_pos,
                      CDU_TEXT_INTV * disp_size.x, "",
                      CDU_BIG_TEXT_SZ, curr_screen.heading_color);

        draw_cdu_line(cr, curr_screen.heading_small, pos_hdg_small,
                      CDU_TEXT_INTV * disp_size.x, "", CDU_SMALL_TEXT_SZ);

        size_t j = 0;
        for (size_t i = 0; i < size_t(N_CDU_DATA_LINES); i++)
        {
            if (j < curr_screen.data_lines.size())
            {
                draw_cdu_line(cr, curr_screen.data_lines[j], pos_small,
                              CDU_TEXT_INTV * disp_size.x, curr_screen.chr_sts[j]);
            }
            if (j + 1 < curr_screen.data_lines.size())
            {
                draw_cdu_line(cr, curr_screen.data_lines[j + 1], pos_big,
                              CDU_TEXT_INTV * disp_size.x, curr_screen.chr_sts[j + 1]);
            }

            pos_small.y += CDU_V_OFFS_REG * disp_size.y;
            pos_big.y += CDU_V_OFFS_REG * disp_size.y;
            j += 2;
        }

        std::string tgt_scratch;
        if (msg_stack.size())
        {
            tgt_scratch = msg_stack.top();
        }
        else if (scratchpad[0] == DELETE_SYMBOL)
        {
            tgt_scratch = DELETE_MSG;
        }
        else
        {
            tgt_scratch = scratchpad;
        }
        draw_cdu_line(cr, tgt_scratch, pos_small,
                      CDU_TEXT_INTV * disp_size.x, "", CDU_BIG_TEXT_SZ);
    }
}
