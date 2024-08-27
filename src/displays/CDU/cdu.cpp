#include "cdu.hpp"

namespace StratosphereAvionics
{
    // CDU definitions:
    // Public member functions:

    CDU::CDU(std::shared_ptr<test::FPLSys> fs)
    {
        fpl_sys = fs;
        fpln = fs->fpl;
        curr_page = CDUPage::RTE;
        curr_subpg = 1;
        n_subpg = 1;

        sel_des_idx = -1;
        sel_des_subpg = 0;
        sel_des_event = 0;
        sel_des_id = 0;
        sel_des = false;

        dep_arr_rwy_filter = false;
        dep_arr_proc_filter = false;

        procs = {};
        trans = {};
        apprs = {};
        rwys = {};
        vias = {};

        sel_des_nm = "";
    }

    void CDU::update()
    {
        seg_list = fpl_sys->get_seg_list(&n_seg_list_sz);
        leg_list = fpl_sys->get_leg_list(&n_leg_list_sz);

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
            else if(curr_page == CDUPage::DEP_ARR_INTRO || 
                curr_page == CDUPage::DEP || curr_page == CDUPage::ARR)
            {
                n_subpg = get_n_dep_arr_subpg();
            }
        }

        if (curr_subpg > n_subpg)
        {
            curr_subpg = n_subpg;
        }
    }

    std::string CDU::on_event(int event_key, std::string scratchpad, std::string *s_out)
    {
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
                curr_page = pg;
                curr_subpg = 1;
                sel_des = false;
                dep_arr_proc_filter = false;
                dep_arr_rwy_filter = false;
            }

            if (curr_subpg > n_subpg)
            {
                curr_subpg = 1;
            }
            else if (curr_subpg == 0)
            {
                curr_subpg = n_subpg;
            }

            return "";
        }
        if (sel_des)
            return handle_sel_des(event_key);
        if (curr_page == CDUPage::RTE)
        {
            return handle_rte(event_key, scratchpad, s_out);
        }
        else if(curr_page == CDUPage::DEP_ARR_INTRO)
        {
            return handle_dep_arr(event_key);
        }
        else if(curr_page == CDUPage::DEP)
        {
            return handle_dep(event_key);
        }
        else if(curr_page == CDUPage::ARR)
        {
            return handle_arr(event_key);
        }

        return "";
    }

    cdu_scr_data_t CDU::get_screen_data()
    {
        if (sel_des)
            return get_sel_des_page();

        if (curr_page == CDUPage::RTE)
            return get_rte_page();
        
        if(curr_page == CDUPage::DEP_ARR_INTRO)
            return get_dep_arr_page();

        if(curr_page == CDUPage::DEP)
            return get_dep_page();

        if(curr_page == CDUPage::ARR)
            return get_arr_page();

        return {};
    }

    // Private member functions:

    void CDU::set_sel_des_state(double id, std::string &name,
                           std::vector<libnav::waypoint_entry_t> &w_e)
    {
        sel_des_id = id;
        sel_des_data = w_e;
        sel_des_nm = name;
        sel_des_subpg = curr_subpg;
        curr_subpg = 1;
        sel_des = true;
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
        double id = fpl_sys->seg_list_id;
        if (name.size() > 7)
            return INVALID_ENTRY_MSG;
        test::seg_list_node_t *s_ptr = nullptr;
        if (next_idx < n_seg_list_sz)
        {
            s_ptr = seg_list[next_idx].ptr;
        }

        bool retval = fpl_sys->fpl->add_enrt_seg({s_ptr, id}, name);

        if (!retval)
            return NOT_IN_DB_MSG;
        return "";
    }

    std::string CDU::delete_via(size_t next_idx)
    {
        double id = fpl_sys->seg_list_id;
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
        double id = fpl_sys->seg_list_id;
        libnav::waypoint_entry_t tgt;
        std::string wpt_nm;

        if (sel_des_idx == -1)
        {
            if (name.size() > 5)
                return INVALID_ENTRY_MSG;
            std::vector<libnav::waypoint_entry_t> wpt_entr;
            size_t n_found = fpl_sys->navaid_db_ptr->get_wpt_data(name, &wpt_entr);

            if (n_found == 0)
            {
                return NOT_IN_DB_MSG;
            }
            else if (n_found > 1)
            {
                set_sel_des_state(id, name, wpt_entr);
                return "";
            }
            wpt_nm = name;
            tgt = wpt_entr[0];
        }
        else
        {
            tgt = sel_des_data[size_t(sel_des_idx)];
            wpt_nm = sel_des_nm;
            sel_des_idx = -1;
            id = sel_des_id;
        }

        test::seg_list_node_t *s_ptr = nullptr;
        if (next_idx < n_seg_list_sz)
        {
            s_ptr = seg_list[next_idx].ptr;
        }
        libnav::waypoint_t tgt_wpt = {wpt_nm, tgt};

        bool retval = fpl_sys->fpl->awy_insert({s_ptr, id}, tgt_wpt);

        if (!retval)
            return NOT_IN_DB_MSG;
        return "";
    }

    std::string CDU::delete_to(size_t next_idx)
    {
        double id = fpl_sys->seg_list_id;
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

        size_t i_start = 1 + 6 * size_t(curr_subpg - 2);
        size_t i_end = std::min(n_seg_list_sz - 1, i_start + 6);

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
                in->data_lines.push_back("");
            }
        }
        in->data_lines.push_back(SEG_LAST);
    }

    void CDU::get_procs(cdu_scr_data_t *in, std::string curr_proc, std::string curr_trans)
    {
        size_t start_idx = size_t((curr_subpg - 1) * 5);
        size_t j = 1;

        for(size_t i = start_idx; i < start_idx + 6 && i < procs.size(); i++)
        {
            std::string curr = procs[i];
            if(curr == curr_proc)
                curr = curr + " <SEL>";
            in->data_lines[j] = curr;
            j += 2;
        }
        if(trans.size() && procs.size() == 1)
        {
            size_t trans_start = size_t((curr_subpg - 1) * 4);
            in->data_lines[j-1] = " TRANS";
            for(size_t i = trans_start; i < trans_start + 4 && i < trans.size(); i++)
            {
                std::string curr = trans[i];
                if(curr == curr_trans)
                    curr = curr + " <SEL>";
                in->data_lines[j] = curr;
                j += 2;
            }
        }
    }

    void CDU::get_rwys(cdu_scr_data_t *in, std::string curr_rwy)
    {
        size_t start_idx = size_t((curr_subpg - 1) * 5);
        size_t j = 1;

        for(size_t i = start_idx; i < start_idx + 6 && i < rwys.size(); i++)
        {
            std::string curr = rwys[i];
            if(curr == curr_rwy)
            {
                curr = "<SEL> " + curr;
            }
            size_t n_sp = size_t(N_CDU_DATA_COLS) - curr.size() - in->data_lines[j].size();
            in->data_lines[j] = in->data_lines[j] + std::string(n_sp, ' ') + curr;
            j += 2;
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

    void CDU::set_procs(test::ProcType ptp, bool is_arr)
    {
        procs = fpln->get_arpt_proc(ptp, is_arr, 
            dep_arr_rwy_filter, dep_arr_proc_filter);
        if(ptp == test::PROC_TYPE_STAR)
        {
            apprs = fpln->get_arpt_proc(test::PROC_TYPE_APPCH, is_arr, 
                dep_arr_rwy_filter, dep_arr_proc_filter);
        }
        else
        {
            apprs = {};
        }
        if(dep_arr_proc_filter)
        {
            trans = fpln->get_arpt_proc_trans(ptp, 
                false, is_arr);
            if(ptp == test::PROC_TYPE_STAR)
            {
                vias = fpln->get_arpt_proc_trans(test::PROC_TYPE_APPCH, false, is_arr);
            }
            else
            {
                vias = {};
            }
        }
        else
        {
            trans = {};
            vias = {};
        }

        if(!is_arr)
            rwys = fpln->get_dep_rwys(dep_arr_rwy_filter, dep_arr_proc_filter);
        else
            rwys = fpln->get_arr_rwys();
    }

    int CDU::get_n_sel_des_subpg()
    {
        return int(sel_des_data.size()) / 6 + bool(int(sel_des_data.size()) % 6);
    }

    int CDU::get_n_rte_subpg()
    {
        std::string dep_rwy = fpln->get_dep_rwy();
        if (dep_rwy != "")
        {
            size_t n_seg_act = n_seg_list_sz - 1;
            return 1 + (n_seg_act / 6) + bool(n_seg_act % 6);
        }
        return 1;
    }

    int CDU::get_n_dep_arr_subpg()
    {
        std::string dep_icao = fpln->get_dep_icao();
        std::string arr_icao = fpln->get_arr_icao();
        if((curr_page == CDUPage::DEP && dep_icao == "") || 
            (curr_page == CDUPage::ARR && arr_icao == ""))
        {
            curr_page = CDUPage::DEP_ARR_INTRO;
        }

        if(curr_page == CDUPage::DEP)
        {
            set_procs(test::PROC_TYPE_SID, false);
            size_t max_cnt = std::max(rwys.size(), procs.size()+trans.size());
            return int(max_cnt) / N_DEP_ARR_ROW_DSP + bool(max_cnt % N_DEP_ARR_ROW_DSP);
        }
        else if(curr_page == CDUPage::ARR)
        {
            set_procs(test::PROC_TYPE_STAR, true);
            size_t max_cnt = std::max(procs.size()+trans.size(), 
                apprs.size()+vias.size());
            return int(max_cnt) / N_DEP_ARR_ROW_DSP + bool(max_cnt % N_DEP_ARR_ROW_DSP);
        }

        return 1;
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
            size_t i_start = 1 + 6 * size_t(curr_subpg - 2);
            size_t i_end = std::min(n_seg_list_sz - 1, i_start + 6);
            size_t i_event = i_start + size_t(event_key - 1) % 6;
            if (i_event > i_end)
            {
                return INVALID_ENTRY_MSG;
            }
            else
            {
                if (event_key >= CDU_KEY_RSK_TOP)
                {
                    if (scratchpad[0] != DELETE_SYMBOL)
                    {
                        std::string ret = add_to(i_event + 1, scratchpad);
                        if (sel_des)
                        {
                            sel_des_event = event_key;
                        }
                        return ret;
                    }

                    return delete_to(i_event);
                }
                else
                {
                    if (scratchpad[0] != DELETE_SYMBOL)
                        return add_via(i_event + 1, scratchpad);
                    return delete_via(i_event);
                }
            }
        }

        return "";
    }

    std::string CDU::handle_dep_arr(int event_key)
    {
        if(event_key == CDU_KEY_LSK_TOP)
        {
            curr_page = CDUPage::DEP;
        }
        else if(event_key == CDU_KEY_RSK_TOP+1)
        {
            curr_page = CDUPage::ARR;
        }

        return "";
    }

    std::string CDU::handle_dep(int event_key)
    {
        if(event_key == CDU_KEY_LSK_TOP + 5)
        {
            curr_page = CDUPage::INIT_REF;
            curr_subpg = 1;
        }
        else if(event_key == CDU_KEY_RSK_TOP + 5)
        {
            curr_page = CDUPage::RTE;
            curr_subpg = 1;
        }
        else if(event_key && event_key < CDU_KEY_LSK_TOP + 5)
        {
            int start_idx = (curr_subpg-1) * 5;
            int sz = procs.size();
            int trans_sz = trans.size();
            int curr_idx = start_idx + event_key-CDU_KEY_LSK_TOP;
            if(curr_idx < sz)
            {
                fpln->set_arpt_proc(test::PROC_TYPE_SID, procs[size_t(curr_idx)]);
                dep_arr_rwy_filter = !dep_arr_rwy_filter;
            }
            else if(curr_idx < sz+trans_sz)
            {
                int tr_idx = curr_idx-sz;
                fpln->set_arpt_proc_trans(test::PROC_TYPE_SID, trans[size_t(tr_idx)]);
            }
        }
        else if(event_key >= CDU_KEY_RSK_TOP && event_key < CDU_KEY_RSK_TOP + 5)
        {
            int start_idx = (curr_subpg-1) * 5;
            int sz = rwys.size();
            int curr_idx = start_idx + event_key-CDU_KEY_RSK_TOP;
            if(curr_idx < sz)
            {
                fpln->set_dep_rwy(rwys[size_t(curr_idx)]);
                dep_arr_proc_filter = !dep_arr_proc_filter;
            }
        }
        return "";
    }

    std::string CDU::handle_arr(int event_key)
    {
        if(event_key == CDU_KEY_LSK_TOP + 5)
        {
            curr_page = CDUPage::INIT_REF;
            curr_subpg = 1;
        }
        if(event_key == CDU_KEY_RSK_TOP + 5)
        {
            curr_page = CDUPage::RTE;
            curr_subpg = 1;
        }
        return "";
    }

    cdu_scr_data_t CDU::get_sel_des_page()
    {
        cdu_scr_data_t out = {};
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
        cdu_scr_data_t out = {};
        out.heading_small = get_small_heading();
        std::string rte_offs = std::string(6, ' ');
        out.heading_big = rte_offs + "RTE1";
        out.heading_color = CDUColor::CYAN;

        if (curr_subpg == 1)
        {
            std::string dest_offs = std::string(N_CDU_DATA_COLS - 7 - 4, ' ');
            std::string origin_dest = " ORIGIN" + dest_offs + "DEST";
            out.data_lines.push_back(origin_dest);
            std::string origin = fpln->get_dep_icao();
            std::string dest = fpln->get_arr_icao();
            bool incomplete = false;

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
            std::string rf_data = dep_rwy + std::string(size_t(N_CDU_DATA_COLS) - dep_rwy.size() - 10, ' ') + std::string(10, '-');
            out.data_lines.push_back(rf_data);
            out.data_lines.push_back(" ROUTE" + std::string(N_CDU_DATA_COLS - 6 - 8, ' ') + "CO ROUTE");
            out.data_lines.push_back("<LOAD" + std::string(size_t(N_CDU_DATA_COLS) - 5 - 10, ' ') + std::string(10, '-'));
            out.data_lines.push_back("");
            out.data_lines.push_back("");
            std::string rte_final = " ROUTE ";
            out.data_lines.push_back(rte_final + std::string(size_t(N_CDU_DATA_COLS) - 7, '-'));
            out.data_lines.push_back("<SAVE" + std::string(size_t(N_CDU_DATA_COLS) - 10, ' ') + "ALTN>");
            out.data_lines.push_back("");
            out.data_lines.push_back("<RTE 2" + std::string(size_t(N_CDU_DATA_COLS) - 15, ' ') + "ACTIVATE>");
        }
        else
        {
            get_seg_page(&out);
        }

        return out;
    }

    cdu_scr_data_t CDU::get_dep_arr_page()
    {
        cdu_scr_data_t out = {};
        out.heading_small = get_small_heading();
        std::string hdg_offs = std::string((N_CDU_DATA_COLS-DEP_ARR_HDG.size())/2, ' ');
        out.heading_big = hdg_offs + DEP_ARR_HDG;
        out.heading_color = CDUColor::WHITE;

        std::string dep = fpln->get_dep_icao();
        std::string arr = fpln->get_arr_icao();

        if(dep != "" || arr != "")
        {
            out.data_lines.push_back(std::string(8, ' ') + "RTE 1");
            if(dep != "")
                out.data_lines.push_back(DEP_ARR_DEP_OPT + dep + DEP_ARR_ARR_OPT);
            out.data_lines.push_back("");
            if(arr != "")
                out.data_lines.push_back(std::string(DEP_ARR_DEP_OPT.size(), ' ') + arr + DEP_ARR_ARR_OPT);
            
        }
        else
        {
            out.data_lines.push_back(DEP_ARR_IDX_DASH_L + "RTE 1 " + DEP_ARR_IDX_DASH_R);
            out.data_lines.push_back("");
            out.data_lines.push_back("");
            out.data_lines.push_back("");
        }

        out.data_lines.push_back(DEP_ARR_IDX_DASH_L + "RTE 2 " + DEP_ARR_IDX_DASH_R);
        out.data_lines.push_back("");
        out.data_lines.push_back("");
        out.data_lines.push_back("");
        out.data_lines.push_back(std::string(N_CDU_DATA_COLS, '-'));
        out.data_lines.push_back("");
        out.data_lines.push_back(DEP_ARR_IDX_OTHER);
        out.data_lines.push_back(DEP_ARR_ARROWS);

        return out;
    }

    cdu_scr_data_t CDU::get_dep_page()
    {
        std::string dep = fpln->get_dep_icao();
        cdu_scr_data_t out = {};
        out.heading_small = get_small_heading();
        out.heading_big = "   " + dep + " DEPARTURES";
        out.heading_color = CDUColor::WHITE;

        for(int i = 0; i < 12; i++)
        {
            out.data_lines.push_back("");
        }
        out.data_lines[0] = DEP_COLS;

        std::string curr_sid = fpln->get_curr_proc(test::PROC_TYPE_SID);
        std::string curr_trans = fpln->get_curr_proc(test::PROC_TYPE_SID, true);
        get_procs(&out, curr_sid, curr_trans);
        
        std::string dep_rwy = fpln->get_dep_rwy();
        get_rwys(&out, dep_rwy);

        out.data_lines[10] = std::string(N_CDU_DATA_COLS, '-');
        out.data_lines[11] = DEP_ARR_BOTTOM;

        return out;
    }

    cdu_scr_data_t CDU::get_arr_page()
    {
        std::string arr = fpln->get_arr_icao();
        cdu_scr_data_t out = {};
        out.heading_small = get_small_heading();
        out.heading_big = "   " + arr + " ARRIVALS";
        out.heading_color = CDUColor::WHITE;

        for(int i = 0; i < 12; i++)
        {
            out.data_lines.push_back("");
        }
        out.data_lines[0] = ARR_COLS;

        std::string curr_star = fpln->get_curr_proc(test::PROC_TYPE_STAR);
        std::string curr_trans = fpln->get_curr_proc(test::PROC_TYPE_STAR, true);
        get_procs(&out, curr_star, curr_trans);
        
        std::string arr_rwy = fpln->get_arr_rwy();
        get_rwys(&out, arr_rwy);

        out.data_lines[10] = std::string(N_CDU_DATA_COLS, '-');
        out.data_lines[11] = DEP_ARR_BOTTOM;

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
    }

    void CDUDisplay::on_click(geom::vect2_t pos)
    {
        pos = (pos - scr_pos) / tex_scale;
        if (pos.x >= 0 && pos.y >= 0 && pos.x < tex_size.x && pos.y < tex_size.y)
        {
            int event = int(key_map->get_at(size_t(pos.x), size_t(pos.y)));

            if (event && event < CDU_KEY_A)
            {
                std::string scratch_proc = strutils::strip(scratchpad);
                std::string scr_out;
                std::string msg = cdu_ptr->on_event(event, scratch_proc, &scr_out);

                if (scr_out != "")
                {
                    scratch_curr = scr_out.size() + 1;
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
                                   geom::vect2_t scale, double l_intv_px, CDUColor color)
    {
        cairo_surface_t *font_sfc;
        if (color == CDUColor::GREEN)
            font_sfc = tex_mngr->data[CDU_GREEN_TEXT_NAME];
        else if (color == CDUColor::CYAN)
            font_sfc = tex_mngr->data[CDU_CYAN_TEXT_NAME];
        else if (color == CDUColor::MAGENTA)
            font_sfc = tex_mngr->data[CDU_MAGENTA_TEXT_NAME];
        else
            font_sfc = tex_mngr->data[CDU_WHITE_TEXT_NAME];

        double res_ratio = size.y * CDU_RES_COEFF;
        scale = scale.scmul(res_ratio);

        for (size_t i = 0; i < s.size(); i++)
        {
            draw_cdu_letter(cr, s[i], pos, scale, font_sfc);
            pos.x += l_intv_px;
        }
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

        draw_cdu_line(cr, curr_screen.heading_big, disp_pos, CDU_BIG_TEXT_SZ,
                      CDU_TEXT_INTV * disp_size.x, curr_screen.heading_color);

        draw_cdu_line(cr, curr_screen.heading_small, pos_hdg_small, CDU_SMALL_TEXT_SZ,
                      CDU_TEXT_INTV * disp_size.x);

        size_t j = 0;
        for (size_t i = 0; i < size_t(N_CDU_DATA_LINES); i++)
        {
            if (j < curr_screen.data_lines.size())
            {
                draw_cdu_line(cr, curr_screen.data_lines[j], pos_small, CDU_SMALL_TEXT_SZ,
                              CDU_TEXT_INTV * disp_size.x);
            }
            if (j + 1 < curr_screen.data_lines.size())
            {
                draw_cdu_line(cr, curr_screen.data_lines[j + 1], pos_big,
                              CDU_BIG_TEXT_SZ, CDU_TEXT_INTV * disp_size.x);
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
        draw_cdu_line(cr, tgt_scratch, pos_small, CDU_BIG_TEXT_SZ,
                      CDU_TEXT_INTV * disp_size.x);
    }
}
