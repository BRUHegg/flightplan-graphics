/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512

	This file contains definitions of member functions for flightplan interface class. 
    This class acts as a layer ontop of the flightplan class. Its job is to fetch data
    from appropriate navigation data bases and store it in the flightplan correctly.
*/


#include "flightpln_int.hpp"


namespace test
{
    bool is_ang_greater(double ang1_rad, double ang2_rad)
    {
        if(ang1_rad < 0)
        {
            ang1_rad += 2 * M_PI;
        }
        if(ang2_rad < 0)
        {
            ang2_rad += 2 * M_PI;
        }

        if(ang1_rad - ang2_rad > M_PI)
        {
            ang2_rad += 2 * M_PI;
        }
        else if(ang2_rad - ang1_rad > M_PI)
        {
            ang1_rad += 2 * M_PI;
        }

        return ang1_rad > ang2_rad;
    }

    std::string get_appr_rwy(std::string& appr)
    {
        std::string rw;

        for(size_t i = 0; i < appr.size(); i++)
        {
            if(rw.size() < 2 && appr[i] >= '0' && appr[i] <= '9')
            {
                rw.push_back(appr[i]);
            }
            else if(rw.size() && (appr[i] == 'L' || appr[i] == 'R' || appr[i] == 'C'))
            {
                rw.push_back(appr[i]);
                break;
            }
            else if(rw.size())
            {
                break;
            }
        }

        return strutils::normalize_rnw_id(rw);
    }

    std::string get_dfms_rwy(std::string& rwy_nm)
    {
        if(rwy_nm[0] == 'R' && rwy_nm[1] == 'W')
        {
            std::string ret = rwy_nm.substr(2, rwy_nm.size()-2);

            return ret;
        }
        return "";
    }

    geo::point get_xa_end_point(geo::point prev, float brng_deg, float va_alt_ft, 
        libnav::runway_entry_t *rnw_data, double clb_ft_nm)
    {
        double clb_nm = va_alt_ft / clb_ft_nm;

        if(rnw_data != nullptr)
        {
            clb_nm += rnw_data->get_impl_length_m() * geo::M_TO_FT * geo::FT_TO_NM;
        }

        geo::point curr = geo::get_pos_from_brng_dist(prev, 
            brng_deg * geo::DEG_TO_RAD, clb_nm);

        return curr;
    }

    libnav::waypoint_t get_ca_va_wpt(geo::point pos, int n_ft)
    {
        libnav::waypoint_t out;
        out.id = "(" + std::to_string(n_ft) + ")";
        out.data.pos = pos;
        out.data.arinc_type = 0;
        out.data.area_code = "";
        out.data.country_code = "";
        out.data.type = libnav::NavaidType::WAYPOINT;
        out.data.navaid = nullptr;

        return out;
    }

    double get_rnp(leg_list_node_t *leg)
    {
        if(leg->data.leg.rnp != 0)
            return double(leg->data.leg.rnp);

        fpl_segment_types seg_tp = leg->data.seg->data.seg_type;

        if(seg_tp != FPL_SEG_ENRT)
            return ASSUMED_RNP_PROC_NM;

        return ASSUMED_RNP_ENRT_NM;
    }

    // FplnInt member functions:
    // Public functions:

    FplnInt::FplnInt(std::shared_ptr<libnav::ArptDB> apt_db, 
            std::shared_ptr<libnav::NavaidDB> nav_db, std::shared_ptr<libnav::AwyDB> aw_db, 
            std::string cifp_path):
        FlightPlan(apt_db, nav_db, cifp_path)
    {
        awy_db = aw_db;
        navaid_db = nav_db;
        proc_db.resize(N_PROC_DB_SZ);

        fpl_id_calc = 0;

        has_dep_rnw_data = false;
        has_arr_rnw_data = false;
    }

    libnav::DbErr FplnInt::load_from_fms(std::string& file_nm, bool set_arpts)
    {
        if(libnav::does_file_exist(file_nm+DFMS_FILE_POSTFIX))
        {
            std::ifstream file(file_nm+DFMS_FILE_POSTFIX);
            if(file.is_open())
            {
                std::string line;
                bool read_enrt = false;
                dfms_arr_data_t arr_data;
                std::string awy_last = "";
                std::string end_last = "";
                while(getline(file, line))
                {
                    std::vector<std::string> ln_split = strutils::str_split(line);
                    
                    if(!read_enrt && ln_split.size() > 1)
                    {
                        if(ln_split[0] == DFMS_N_ENRT_NM)
                        {
                            read_enrt = true;
                        }
                        else
                        {
                            libnav::DbErr err = process_dfms_proc_line(ln_split, 
                                set_arpts, &arr_data);

                            if(err != libnav::DbErr::SUCCESS)
                            {
                                return err;
                            }
                        }
                    }
                    else if(read_enrt && ln_split.size() == N_DFMS_ENRT_WORDS)
                    {
                        std::string wpt_id = "";
                        bool add_awy_seg = false;
                        if(ln_split[2] != DFMS_DEP_NM && ln_split[2] != DFMS_ARR_NM)
                        {
                            libnav::waypoint_t wpt;
                            bool ret = get_dfms_wpt(ln_split, &wpt);
                            wpt_id = wpt.get_awy_id();

                            if(ret)
                            {
                                if(ln_split[2] == DFMS_DIR_SEG_NM || 
                                    (ln_split[2] != awy_last && awy_last != ""))
                                {
                                    add_awy_seg = true;
                                }
                                else
                                {
                                    awy_last = ln_split[2];
                                    end_last = wpt_id;
                                }
                            }
                            else
                            {
                                return libnav::DbErr::DATA_BASE_ERROR;
                            }
                        }
                        else
                        {
                            add_awy_seg = true;
                        }

                        if(add_awy_seg)
                        {
                            if(awy_last != "" && end_last != "")
                            {
                                add_enrt_seg({nullptr, seg_list.id}, awy_last);
                                awy_insert({&(seg_list.tail), seg_list.id}, 
                                    end_last);
                            }
                            awy_last = "";
                            end_last = "";

                            if(wpt_id != "")
                                awy_insert({nullptr, seg_list.id}, wpt_id);
                        }
                    }
                }

                file.close();

                return set_dfms_arr_data(&arr_data, set_arpts);
            }
            
            file.close();
        }
        return libnav::DbErr::FILE_NOT_FOUND;
    }

    void FplnInt::save_to_fms(std::string& file_nm, bool save_sid_star)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(!arpt_db->is_airport(departure->icao_code) || 
            !arpt_db->is_airport(arrival->icao_code))
        {
            return;
        }

        std::ofstream out(file_nm+DFMS_FILE_POSTFIX, std::ofstream::out);

        out << DFMS_PADDING;
        std::string curr_cycle = std::to_string(navaid_db->get_wpt_cycle());
        out << DFMS_AIRAC_CYCLE_NM + DFMS_COL_SEP + curr_cycle + "\n";

        out << DFMS_DEP_NM << DFMS_COL_SEP << departure->icao_code << "\n";
        std::string dep_rwy = fpl_refs[FPL_SEG_DEP_RWY].name;

        if(dep_rwy != "")
        {
            out << DFMS_DEP_RWY_NM << DFMS_COL_SEP << DFMS_RWY_PREFIX+dep_rwy << "\n";
        }
        
        if(save_sid_star)
        {
            std::string sid_name = fpl_refs[FPL_SEG_SID].name;
            std::string sid_trans_name = fpl_refs[FPL_SEG_SID_TRANS].name;

            if(sid_name != "")
            {
                out << DFMS_SID_NM << DFMS_COL_SEP << sid_name << "\n";

                if(sid_trans_name != "")
                {
                    out << DFMS_SID_TRANS_NM << DFMS_COL_SEP << sid_trans_name << "\n";
                }
            }
        }

        out << DFMS_ARR_NM << DFMS_COL_SEP << arrival->icao_code << "\n";

        if(arr_rwy != "")
        {
            out << DFMS_ARR_RWY_NM << DFMS_COL_SEP << DFMS_RWY_PREFIX+arr_rwy << "\n";
        }

        if(save_sid_star)
        {
            std::string star_name = fpl_refs[FPL_SEG_STAR].name;
            std::string star_trans_name = fpl_refs[FPL_SEG_STAR_TRANS].name;

            if(star_name != "")
            {
                out << DFMS_STAR_NM << DFMS_COL_SEP << star_name << "\n";

                if(star_trans_name != "")
                {
                    out << DFMS_STAR_TRANS_NM << DFMS_COL_SEP << star_trans_name << "\n";
                }
            }
        }

        std::vector<std::string> vec;
        size_t n_legs = get_dfms_enrt_legs(&vec);

        out << DFMS_N_ENRT_NM << DFMS_COL_SEP << std::to_string(int(n_legs)) << "\n";

        for(size_t i = 0; i < n_legs; i++)
        {
            out << vec[i] << "\n";
        }

        out.close();
    }

    libnav::DbErr FplnInt::set_dep(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        libnav::DbErr out = set_arpt(icao, &departure);
        if(departure != nullptr && departure->icao_code == icao
            && out != libnav::DbErr::ERR_NONE)
        {
            dep_rnw = departure->get_rwy_db();
            proc_db[PROC_TYPE_SID] = departure->get_all_sids();
            proc_db[PROC_TYPE_STAR] = departure->get_all_stars();
            proc_db[PROC_TYPE_APPCH] = departure->get_all_appch();

            // If there is an arrival and departure was changed, clear arrival data
            if(arrival != nullptr)
            {
                arr_rwy = "";
                has_arr_rnw_data = false;
                arr_rnw.clear();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_STAR].clear();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_APPCH].clear();
                delete arrival;
                arrival = nullptr;
            }
        }
        
        return out;
    }

    std::string FplnInt::get_dep_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr)
            return departure->icao_code;
        return "";
    }

    libnav::DbErr FplnInt::set_arr(std::string icao)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure == nullptr)
        {
            return libnav::DbErr::ERR_NONE;
        }
        
        libnav::DbErr err = set_arpt(icao, &arrival, true);
        if(err != libnav::DbErr::ERR_NONE)
        {
            arr_rwy = "";
            has_arr_rnw_data = false;

            if(arrival != nullptr)
            {
                arr_rnw = arrival->get_rwy_db();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_STAR] = arrival->get_all_stars();
                proc_db[N_ARR_DB_OFFSET+PROC_TYPE_APPCH] = arrival->get_all_appch();
            }
        }
        return err;
    }

    std::string FplnInt::get_arr_icao()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
            return arrival->icao_code;
        return "";
    }

    std::vector<std::string> FplnInt::get_dep_rwys(bool filter_rwy, bool filter_sid)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        std::vector<std::string> out = {};
        
        std::string curr_rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
        if(filter_sid && curr_rwy != "")
        {
            out.push_back(curr_rwy);
        }
        else
        {
            std::string curr_sid = fpl_refs[FPL_SEG_SID].name;
            size_t db_idx = get_proc_db_idx(PROC_TYPE_SID, false);

            for(auto i: dep_rnw)
            {
                if(filter_rwy && curr_sid != "" && 
                    proc_db[db_idx][curr_sid].find(i.first) == proc_db[db_idx][curr_sid].end())
                {
                    continue;
                }
                out.push_back(i.first);
            }
        }
        return out;
    }

    std::vector<std::string> FplnInt::get_arr_rwys()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(arrival != nullptr)
        {
            return arrival->get_rwys();
        }
        return {};
    }

    bool FplnInt::set_dep_rwy(std::string& rwy)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(dep_rnw.find(rwy) != dep_rnw.end())
        {
            std::string curr_rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
            if(rwy != curr_rwy)
            {
                int data_found = arpt_db->get_rnw_data(departure->icao_code, 
                    rwy, &dep_rnw_data);
                has_dep_rnw_data = data_found;
                if(!data_found)
                {
                    has_dep_rnw_data = false;
                    dep_rnw_data = {};
                }

                std::string curr_sid = fpl_refs[FPL_SEG_SID].name;
                std::string curr_trans = fpl_refs[FPL_SEG_SID_TRANS].name;
                delete_ref(FPL_SEG_SID_TRANS);
                delete_ref(FPL_SEG_SID);

                libnav::arinc_rwy_data_t rwy_data = dep_rnw[rwy];

                leg_t ins_leg;
                ins_leg.leg_type = "IF";
                ins_leg.main_fix.id = rwy;
                ins_leg.main_fix.data.pos = rwy_data.pos;
                ins_leg.main_fix.data.type = libnav::NavaidType::RWY;

                std::vector<leg_t> legs = {};

                add_legs(ins_leg, legs, FPL_SEG_DEP_RWY, rwy);
                fpl_refs[FPL_SEG_DEP_RWY].name = rwy;

                set_sid_star(curr_sid);
                set_proc_trans(PROC_TYPE_SID, curr_trans, false);
            }

            return true;
        }

        return false;
    }

    std::string FplnInt::get_dep_rwy()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(departure != nullptr && fpl_refs[FPL_SEG_DEP_RWY].ptr != nullptr)
            return fpl_refs[FPL_SEG_DEP_RWY].name;
        return "";
    }

    bool FplnInt::get_dep_rwy_data(libnav::runway_entry_t *out)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(has_dep_rnw_data)
        {
            *out = dep_rnw_data;
            return true;
        }
            
        return false;
    }

    bool FplnInt::set_arr_rwy(std::string& rwy)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(arr_rnw.find(rwy) != arr_rnw.end())
        {
            if(arr_rwy != rwy)
            {
                int data_found = arpt_db->get_rnw_data(arrival->icao_code, 
                    rwy, &arr_rnw_data);
                has_arr_rnw_data = true;
                if(!data_found)
                {
                    has_arr_rnw_data = false;
                    arr_rnw_data = {};
                }

                arr_rwy = rwy;

                libnav::arinc_rwy_data_t rwy_data = arr_rnw[rwy];
                libnav::waypoint_t rwy_wpt = {arr_rwy, {libnav::NavaidType::RWY, 0, rwy_data.pos, 
                    arrival->icao_code, "", nullptr}};
                leg_t rwy_leg{};
                rwy_leg.leg_type = "TF";
                rwy_leg.set_main_fix(rwy_wpt);

                libnav::arinc_leg_seq_t legs = {};

                delete_ref(FPL_SEG_APPCH_TRANS);

                set_sid_star(fpl_refs[size_t(FPL_SEG_STAR)].name, true, false);
                //set_proc_trans(PROC_TYPE_STAR, 
                //    fpl_refs[size_t(FPL_SEG_STAR_TRANS)].name, true);

                add_legs(rwy_leg, legs, FPL_SEG_APPCH, arr_rwy);
                fpl_refs[size_t(FPL_SEG_APPCH)].name = arr_rwy;

            }
            
            return true;
        }

        return false;
    }

    std::string FplnInt::get_arr_rwy()
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        return arr_rwy;
    }

    bool FplnInt::get_arr_rwy_data(libnav::runway_entry_t *out)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(has_arr_rnw_data)
        {
            *out = arr_rnw_data;
            return true;
        }
            
        return false;
    }

    std::vector<std::string> FplnInt::get_arpt_proc(ProcType tp, bool is_arr,
        bool filter_rwy, bool filter_proc)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        fpl_segment_types s_tp = get_proc_tp(tp);
        size_t tp_idx = size_t(s_tp);

        if(filter_rwy && fpl_refs[tp_idx].name != "")
        {
            return {fpl_refs[tp_idx].name};
        }

        size_t db_idx = get_proc_db_idx(tp, is_arr);

        if(db_idx != N_PROC_DB_SZ)
        {
            std::string rwy = "";

            if(filter_proc)
            {
                if(is_arr)
                {
                    rwy = arr_rwy;
                }
                else
                {
                    rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
                }
            }

            return get_proc(proc_db[db_idx], rwy, tp == PROC_TYPE_APPCH);
        }
        
        return {};
    }

    std::vector<std::string> FplnInt::get_arpt_proc_trans(ProcType tp, bool is_rwy, bool is_arr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        size_t ref_idx = size_t(get_proc_tp(tp));
        std::string proc_name = fpl_refs[ref_idx].name;
        if(proc_name != "")
        {
            size_t db_idx = get_proc_db_idx(tp, is_arr);
            if(is_arr)
            {
                return get_proc_trans(proc_name, proc_db[db_idx], arr_rnw, is_rwy);
            }
            return get_proc_trans(proc_name, proc_db[db_idx], dep_rnw, is_rwy);
        }
        return {};
    }

    bool FplnInt::set_arpt_proc(ProcType tp, std::string proc_nm, bool is_arr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        size_t db_idx = get_proc_db_idx(tp, is_arr);

        switch(db_idx)
        {
        case PROC_TYPE_SID:
            return set_sid_star(proc_nm);
        case PROC_TYPE_STAR+N_ARR_DB_OFFSET:
            return set_sid_star(proc_nm, true);
        case PROC_TYPE_APPCH+N_ARR_DB_OFFSET:
            return set_appch(proc_nm);
        default:
            return false;
        }

        return false;
    }

    bool FplnInt::set_arpt_proc_trans(ProcType tp, std::string trans, bool is_arr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        return set_proc_trans(tp, trans, is_arr);
    }

    bool FplnInt::add_enrt_seg(timed_ptr_t<seg_list_node_t> next, std::string name)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(next.id == seg_list.id && next.ptr != &(seg_list.head))
        {
            if(next.ptr == nullptr)
            {
                seg_list_node_t *prev = seg_list.tail.prev;
                leg_list_node_t *end_leg = prev->data.end;
                
                if(prev->data.seg_type <= FPL_SEG_ENRT && prev != &(seg_list.head))
                {
                    bool add_seg = false;

                    if(end_leg != nullptr)
                    {
                        libnav::waypoint_t end_fix = end_leg->data.leg.main_fix;
                        std::string end_leg_awy_id = end_fix.get_awy_id();
                        add_seg = awy_db->is_in_awy(name, end_leg_awy_id);
                    }
                    else if(prev->prev->data.seg_type > FPL_SEG_DEP_RWY)
                    {
                        leg_list_node_t *base_end_leg = prev->prev->data.end;
                        if(base_end_leg != nullptr)
                        {
                            std::string base_awy_id = base_end_leg->data.leg.main_fix.get_awy_id();
                            std::vector<libnav::awy_point_t> awy_pts;
                            size_t n_pts = awy_db->get_aa_path(prev->data.name, 
                                base_awy_id, name, &awy_pts);
                            
                            if(n_pts)
                            {
                                delete_segment(prev);
                                add_awy_seg(prev->data.name, next.ptr, awy_pts);
                                add_seg = true;
                            }
                        }
                    }
                    if(add_seg)
                    {
                        seg_list_node_t *seg_add = seg_stack.get_new();
                        if(seg_add != nullptr)
                        {
                            seg_add->data.name = name;
                            seg_add->data.seg_type = FPL_SEG_ENRT;
                            seg_add->data.is_direct = false;
                            seg_add->data.is_discon = false;
                            seg_add->data.end = nullptr;
                            seg_list.insert_before(&(seg_list.tail), seg_add);
                            update_id();

                            return true;
                        }
                    }
                }
            }
            else if(next.ptr != &(seg_list.head) && next.ptr->prev != &(seg_list.head))
            {
                seg_list_node_t *prev = next.ptr->prev;
                seg_list_node_t *base_seg = prev->prev;
                leg_list_node_t *prev_end_leg = prev->data.end;
                leg_list_node_t *end_leg = base_seg->data.end;

                if(end_leg != nullptr)
                {
                    libnav::waypoint_t end_fix = end_leg->data.leg.main_fix;
                    std::string end_leg_awy_id = end_fix.get_awy_id();

                    if(awy_db->is_in_awy(name, end_leg_awy_id))
                    {
                        if(prev_end_leg != nullptr && !(prev->data.is_discon))
                        {
                            libnav::waypoint_t prev_end_fix = prev_end_leg->data.leg.main_fix;
                            std::string prev_end_leg_awy_id = prev_end_fix.get_awy_id();

                            if(awy_db->is_in_awy(name, prev_end_leg_awy_id))
                            {
                                std::vector<libnav::awy_point_t> awy_pts;
                                size_t n_pts = awy_db->get_ww_path(name, 
                                    end_leg_awy_id, prev_end_leg_awy_id, &awy_pts);
                                
                                if(n_pts)
                                {
                                    delete_segment(prev);
                                    add_awy_seg(name, next.ptr, awy_pts);

                                    return true;
                                }
                            }
                        }
                        fpl_segment_types prev_tp = prev->data.seg_type;
                        if(!(prev->data.is_discon))
                        {
                            delete_segment(prev, true, true);
                        }
                            
                        seg_list_node_t *seg_add = seg_stack.get_new();
                        if(seg_add != nullptr)
                        {
                            seg_add->data.name = name;
                            seg_add->data.seg_type = prev_tp;
                            seg_add->data.is_direct = false;
                            seg_add->data.is_discon = false;
                            seg_add->data.end = nullptr;
                            seg_list.insert_before(base_seg->next, seg_add);
                            update_id();

                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    bool FplnInt::awy_insert(timed_ptr_t<seg_list_node_t> next, std::string end_id)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(next.id == seg_list.id && next.ptr != &(seg_list.head))
        {
            if(next.ptr == nullptr)
            {
                seg_list_node_t *prev = seg_list.tail.prev;
                if(prev->data.end != nullptr)
                {
                    leg_t dir_leg = get_awy_tf_leg(end_id);
                    add_direct_leg(dir_leg, &(leg_list.tail));
                    return true;
                }
            }
            else
            {
                seg_list_node_t *prev = next.ptr->prev;

                if(prev != &(seg_list.head))
                {
                    std::string prev_name = prev->data.name;
                    seg_list_node_t *prev_full = prev->prev;

                    bool in_awy = awy_db->is_in_awy(prev_name, end_id);
                    if(prev_full->data.end != nullptr && in_awy)
                    {
                        leg_list_node_t *prev_leg = prev_full->data.end;
                        libnav::waypoint_t start_fix = prev_leg->data.leg.main_fix;
                        std::string start_id = start_fix.get_awy_id();

                        std::vector<libnav::awy_point_t> awy_pts;
                        size_t n_pts = size_t(awy_db->get_ww_path(prev_name, start_id, 
                            end_id, &awy_pts));
                        
                        if(n_pts)
                        {
                            delete_segment(prev, true, true);
                            add_awy_seg(prev_name, prev_full->next, awy_pts);

                            return true;
                        }
                    }
                    else if(prev_full->data.end != nullptr && !in_awy)
                    {
                        delete_segment(prev, true, true);
                        leg_t dir_leg = get_awy_tf_leg(end_id);
                        add_direct_leg(dir_leg, prev_full->data.end->next);

                        return true;
                    }
                }
            }
        }

        return false;
    }

    bool FplnInt::delete_via(timed_ptr_t<seg_list_node_t> curr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(curr.id == seg_list.id && curr.ptr != &(seg_list.head) && curr.ptr != nullptr &&
            curr.ptr->prev != &(seg_list.head))
        {
            if(!curr.ptr->data.is_discon && !curr.ptr->data.is_direct)
            {
                delete_segment(curr.ptr, true, false, true);

                return true;
            }
        }
        return false;
    }

    bool FplnInt::delete_seg_end(timed_ptr_t<seg_list_node_t> curr)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(curr.id == seg_list.id && curr.ptr != &(seg_list.head) && curr.ptr != nullptr &&
            !curr.ptr->data.is_discon)
        {
            seg_list_node_t *next = curr.ptr->next;
            if(curr.ptr != &(seg_list.tail) && !next->data.is_direct && 
                !next->data.is_discon && curr.ptr->data.end != nullptr)
            {
                delete_segment(curr.ptr->next, true, false, true);
            }

            delete_segment(curr.ptr, false, true);
            return true;
        }

        return false;
    }

    void FplnInt::dir_from_to(timed_ptr_t<leg_list_node_t> from, 
            timed_ptr_t<leg_list_node_t> to)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(from.id == leg_list.id && from.id == to.id)
        {
            delete_range(from.ptr, to.ptr);
        }
    }

    void FplnInt::add_direct(libnav::waypoint_t wpt, timed_ptr_t<leg_list_node_t> next)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(next.id == leg_list.id)
        {
            leg_t dir_leg{};
            dir_leg.leg_type = "TF";
            dir_leg.set_main_fix(wpt);

            add_direct_leg(dir_leg, next.ptr);
        }
    }

    bool FplnInt::delete_leg(timed_ptr_t<leg_list_node_t> next)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);

        if(next.id == leg_list.id)
        {
            return delete_singl_leg(next.ptr);
        }
        return false;
    }

    void FplnInt::update(double hdg_trk_diff)
    {
        std::lock_guard<std::mutex> lock(fpl_mtx);
        if(fpl_id_calc != fpl_id_curr)
        {
            leg_list_node_t *leg_curr = leg_list.head.next;

            while(leg_curr != &(leg_list.tail))
            {
                leg_list_node_t *next_leg = leg_curr->next;
                // Delete double discons:
                if(leg_curr->data.is_discon && (leg_curr->prev->data.is_discon || 
                    leg_curr->prev == &(leg_list.head)))
                {
                    delete_segment(leg_curr->data.seg, false);
                    leg_curr = next_leg;
                    continue;
                }

                if(leg_curr->prev->data.is_discon)
                {
                    leg_curr->data.leg.leg_type = "IF";
                }
                else if(leg_curr->prev != &(leg_list.head) && 
                    leg_curr->data.leg.leg_type == "IF")
                {
                    std::string prev_type = leg_curr->prev->data.leg.leg_type;
                    if(NOT_FOLLOWED_BY_DF.find(prev_type) != NOT_FOLLOWED_BY_DF.end())
                    {
                        leg_curr->data.leg.leg_type = "CF";
                    }
                    else
                    {
                        leg_curr->data.leg.leg_type = "DF";
                    }
                }

                if(!leg_curr->data.is_discon)
                {
                    calculate_leg(leg_curr, hdg_trk_diff);
                }
                
                leg_curr = next_leg;
            }
            fpl_id_calc = fpl_id_curr;
        }
    }

    // Private functions:

    // Static member functions:

    size_t FplnInt::get_proc_db_idx(ProcType tp, bool is_arr)
    {
        if(tp == PROC_TYPE_SID && is_arr)
        {
            return N_PROC_DB_SZ;
        }

        return tp + N_ARR_DB_OFFSET * is_arr;
    }

    fpl_segment_types FplnInt::get_proc_tp(ProcType tp)
    {
        switch (tp)
        {
        case PROC_TYPE_SID:
            return FPL_SEG_SID;
        case PROC_TYPE_STAR:
            return FPL_SEG_STAR;
        case PROC_TYPE_APPCH:
            return FPL_SEG_APPCH;
        default:
            return FPL_SEG_NONE;
        }
    }

    fpl_segment_types FplnInt::get_trans_tp(ProcType tp)
    {
        switch (tp)
        {
        case PROC_TYPE_SID:
            return FPL_SEG_SID_TRANS;
        case PROC_TYPE_STAR:
            return FPL_SEG_STAR_TRANS;
        case PROC_TYPE_APPCH:
            return FPL_SEG_APPCH_TRANS;
        default:
            return FPL_SEG_NONE;
        }
    }

    std::vector<std::string> FplnInt::get_proc(libnav::str_umap_t& db, std::string rw, bool is_appch)
    {
        std::vector<std::string> out;

        for(auto i: db)
        {
            if(rw != "" && i.second.find(rw) == i.second.end() && !is_appch)
            {
                continue;
            }
            else if(is_appch && rw != "")
            {
                std::string curr_nm = i.first;
                std::string c_rnw = get_appr_rwy(curr_nm);
                if(c_rnw != rw)
                {
                    continue;
                }
            }
            out.push_back(i.first);
        }

        return out;
    }

    std::vector<std::string> FplnInt::get_proc_trans(std::string proc, libnav::str_umap_t& db, 
        libnav::arinc_rwy_db_t& rwy_db, bool is_rwy)
    {
        std::vector<std::string> out;

        assert(db.find(proc) != db.end());

        for(auto i: db[proc])
        {
            if(is_rwy && rwy_db.find(i) != rwy_db.end())
            {
                out.push_back(i);
            }
            else if(!is_rwy && rwy_db.find(i) == rwy_db.end())
            {
                out.push_back(i);
            }
        }

        return out;
    }

    std::string FplnInt::get_dfms_enrt_leg(leg_list_node_t* lg, bool force_dir)
    {
        leg_t leg = lg->data.leg;
        libnav::navaid_type_t xp_type = libnav::libnav_to_xp_fix(leg.main_fix.data.type);
        std::string type_str = std::to_string(int(xp_type));
        std::string awy_nm = lg->data.seg->data.name;
        std::string dfms_awy_nm = DFMS_DIR_SEG_NM;

        if(awy_nm != DCT_LEG_NAME && !force_dir)
        {
            dfms_awy_nm = awy_nm;
        }

        double curr_alt_ft = 0;
        if(leg.alt_desc == libnav::AltMode::AT)
        {
            curr_alt_ft = double(leg.alt2_ft);
        }

        std::string alt_str = strutils::double_to_str(curr_alt_ft, N_DFMS_OUT_PREC);
        std::string lat_str = strutils::double_to_str(leg.main_fix.data.pos.lat_rad * 
            geo::RAD_TO_DEG, N_DFMS_OUT_PREC);
        std::string lon_str = strutils::double_to_str(leg.main_fix.data.pos.lon_rad * 
            geo::RAD_TO_DEG, N_DFMS_OUT_PREC);

        return type_str + DFMS_COL_SEP + leg.main_fix.id + DFMS_COL_SEP + dfms_awy_nm + 
            DFMS_COL_SEP + alt_str + DFMS_COL_SEP + lat_str + DFMS_COL_SEP + lon_str;
    }

    // Non-static member functions:

    libnav::DbErr FplnInt::process_dfms_proc_line(std::vector<std::string>& l_split, 
        bool set_arpts, dfms_arr_data_t* arr_data)
    {
        bool db_err = false;

        if(set_arpts && l_split[0] == DFMS_DEP_NM)
        {
            libnav::DbErr err = set_dep(l_split[1]);
            if(err != libnav::DbErr::SUCCESS && 
                err != libnav::DbErr::PARTIAL_LOAD)
            {
                return err;
            }
        }
        else if(l_split[0] == DFMS_DEP_RWY_NM)
        {
            std::string tmp_rwy = get_dfms_rwy(l_split[1]);
            db_err = !set_dep_rwy(tmp_rwy);
        }
        else if(l_split[0] == DFMS_SID_NM)
        {
            db_err = !set_arpt_proc(PROC_TYPE_SID, l_split[1]);
        }
        else if(l_split[0] == DFMS_SID_TRANS_NM)
        {
            db_err = !set_arpt_proc_trans(PROC_TYPE_SID, l_split[1]);
        }
        else if(set_arpts && l_split[0] == DFMS_ARR_NM)
        {
            arr_data->arr_icao = l_split[1];
        }
        else if(l_split[0] == DFMS_ARR_RWY_NM)
        {
            arr_data->arr_rwy = l_split[1];
        }
        else if(l_split[0] == DFMS_STAR_NM)
        {
            arr_data->star = l_split[1];
        }
        else if(l_split[0] == DFMS_SID_TRANS_NM)
        {
            arr_data->star_trans = l_split[1];
        }

        if(db_err)
        {
            return libnav::DbErr::DATA_BASE_ERROR;
        }

        return libnav::DbErr::SUCCESS;
    }

    libnav::DbErr FplnInt::set_dfms_arr_data(dfms_arr_data_t* arr_data, bool set_arpt)
    {
        if(set_arpt && arr_data->arr_icao != "")
        {
            libnav::DbErr err = set_arr(arr_data->arr_icao);
            if(err != libnav::DbErr::SUCCESS && 
                err != libnav::DbErr::PARTIAL_LOAD)
            {
                return err;
            }
        }
        std::string tmp_rwy = get_dfms_rwy(arr_data->arr_rwy);
        if(tmp_rwy != "" && !set_arr_rwy(tmp_rwy))
            return libnav::DbErr::DATA_BASE_ERROR;
        
        if(arr_data->star != "" && !set_arpt_proc(PROC_TYPE_STAR, arr_data->star, true))
            return libnav::DbErr::DATA_BASE_ERROR;
        
        if(arr_data->star_trans != "" && 
            !set_arpt_proc_trans(PROC_TYPE_STAR, arr_data->star_trans, true))
            return libnav::DbErr::DATA_BASE_ERROR;

        return libnav::DbErr::SUCCESS;
    }

    bool FplnInt::get_dfms_wpt(std::vector<std::string>& l_split, libnav::waypoint_t* out)
    {
        libnav::navaid_type_t tp = libnav::navaid_type_t(
            strutils::stoi_with_strip(l_split[0]));
        libnav::NavaidType nav_tp = libnav::xp_fix_type_to_libnav(tp);

        std::vector<libnav::waypoint_entry_t> wpt_entrs;
        size_t n_ent = navaid_db->get_wpt_data(l_split[1], &wpt_entrs, "", "", nav_tp);

        if(n_ent)
        {
            double p_lat_rad = double(strutils::stof_with_strip(l_split[4])) * geo::DEG_TO_RAD;
            double p_lon_rad = double(strutils::stof_with_strip(l_split[5])) * geo::DEG_TO_RAD;

            libnav::sort_wpt_entry_by_dist(&wpt_entrs, {p_lat_rad, p_lon_rad});

            *out = {l_split[1], wpt_entrs[0]};
            return true;
        }
        
        return false;
    }

    std::string FplnInt::get_dfms_arpt_leg(bool is_arr)
    {
        libnav::Airport *ptr = departure;
        std::string seg_nm = DFMS_DEP_NM;

        if(is_arr)
        {
            ptr = arrival;
            seg_nm = DFMS_ARR_NM;
        }

        std::string icao_cd = ptr->icao_code;

        double alt_restr = 0;
        double arpt_lat_deg = 0;
        double arpt_lon_deg = 0;

        libnav::airport_data_t arpt_data;
        arpt_db->get_airport_data(icao_cd, &arpt_data);

        alt_restr = arpt_data.elevation_ft;
        arpt_lat_deg = arpt_data.pos.lat_rad * geo::RAD_TO_DEG;
        arpt_lon_deg = arpt_data.pos.lon_rad * geo::RAD_TO_DEG;
        
        std::string alt_restr_str = strutils::double_to_str(alt_restr, N_DFMS_OUT_PREC);
        std::string arpt_lat_str = strutils::double_to_str(arpt_lat_deg, N_DFMS_OUT_PREC);
        std::string arpt_lon_str = strutils::double_to_str(arpt_lon_deg, N_DFMS_OUT_PREC);

        std::string seg_tp = "1";

        return seg_tp + DFMS_COL_SEP + icao_cd + DFMS_COL_SEP + seg_nm + DFMS_COL_SEP + 
            alt_restr_str + DFMS_COL_SEP + arpt_lat_str + DFMS_COL_SEP + arpt_lon_str;
    }
        
    size_t FplnInt::get_dfms_enrt_legs(std::vector<std::string>* out)
    {
        out->push_back(get_dfms_arpt_leg());

        leg_list_node_t *start = &(leg_list.head);

        while(start->next != &(leg_list.tail) && 
            (start->next->data.seg->data.seg_type < FPL_SEG_ENRT || 
            start->data.seg->data.seg_type == FPL_SEG_DEP_RWY))
        {
            start = start->next;
        }

        bool first_leg = true;

        while(start != &(leg_list.tail) && 
            start->data.seg->data.seg_type <= FPL_SEG_ENRT)
        {
            if(!start->data.is_discon)
            {
                std::string tmp = get_dfms_enrt_leg(start, first_leg);
                out->push_back(tmp);
                if(first_leg)
                {
                    first_leg = false;
                }
            }

            start = start->next;
        }

        out->push_back(get_dfms_arpt_leg(true));

        return out->size();
    }

    bool FplnInt::add_fpl_seg(libnav::arinc_leg_seq_t& legs, fpl_segment_types seg_tp, std::string seg_nm,
        seg_list_node_t *next, bool set_ref)
    {
        if(legs.size())
        {
            size_t seg_idx = size_t(seg_tp);
            leg_t start = legs[0];
            std::vector<leg_t> legs_ins;

            for(size_t i = 1; i < legs.size(); i++)
            {
                legs_ins.push_back(legs[i]);
            }

            add_legs(start, legs_ins, seg_tp, seg_nm, next);
            if(set_ref)
                fpl_refs[seg_idx].name = seg_nm;

            return true;
        }

        return false;
    }

    leg_t FplnInt::get_awy_tf_leg(libnav::awy_point_t awy_pt)
    {
        std::string wpt_uid = awy_pt.get_uid();
        std::vector<std::string> uid_split = strutils::str_split(wpt_uid, libnav::AUX_ID_SEP); // This should be somewhere in libnav
        std::vector<libnav::waypoint_entry_t> cand;
        size_t n_cand = navaid_db->get_wpt_by_awy_str(wpt_uid, &cand);
        assert(n_cand != 0);
        libnav::waypoint_t wpt = {uid_split[0], cand[0]};
        leg_t out{};
        out.leg_type = "TF";
        out.set_main_fix(wpt);

        return out;
    }

    void FplnInt::add_awy_seg(std::string awy, seg_list_node_t *next,
        std::vector<libnav::awy_point_t>& awy_pts)
    {
        leg_t start_leg = get_awy_tf_leg(awy_pts[0]);
        std::vector<leg_t> legs;

        for(size_t i = 1; i < awy_pts.size(); i++)
        {
            legs.push_back(get_awy_tf_leg(awy_pts[i]));
        }

        add_legs(start_leg, legs, FPL_SEG_ENRT, awy, next);
    }

    bool FplnInt::set_sid_star(std::string proc_nm, bool is_star, bool reset_rwy)
    {
        size_t db_idx;
        ProcType proc_tp;
        fpl_segment_types proc_seg, trans_seg;
        if(!is_star) 
        {
            proc_tp = PROC_TYPE_SID;
            db_idx = get_proc_db_idx(PROC_TYPE_SID, false);
            proc_seg = FPL_SEG_SID;
            trans_seg = FPL_SEG_SID_TRANS;
        }
        else
        {
            proc_tp = PROC_TYPE_STAR;
            db_idx = get_proc_db_idx(PROC_TYPE_STAR, true);
            proc_seg = FPL_SEG_STAR;
            trans_seg = FPL_SEG_STAR_TRANS;
        }

        if(proc_db[db_idx].find(proc_nm) != proc_db[db_idx].end())
        {
            std::string rwy;
            if(!is_star)
                rwy = fpl_refs[FPL_SEG_DEP_RWY].name;
            else
                rwy = arr_rwy;

            if(rwy == "")
            {
                delete_ref(trans_seg);
                delete_ref(proc_seg);
                fpl_refs[size_t(proc_seg)].name = proc_nm;
            }
            else
            {
                libnav::arinc_leg_seq_t legs;
                libnav::arinc_leg_seq_t legs_add;  // Additional legs. Inserted if there is a blank transition
                std::string none_trans = NONE_TRANS;

                if(!is_star)
                {
                    legs = departure->get_sid(proc_nm, rwy);
                    legs_add = departure->get_sid(proc_nm, none_trans);
                }
                else
                {
                    legs = arrival->get_star(proc_nm, rwy);
                    legs_add = arrival->get_star(proc_nm, none_trans);
                }

                libnav::arinc_leg_seq_t *l_add = &legs_add;
                libnav::arinc_leg_seq_t *l_tmp = &legs;
                if(!is_star)
                    std::swap(l_add, l_tmp);

                size_t i_beg = l_add->size() > 0;
                for(size_t i = i_beg; i < l_tmp->size(); i++)
                {
                    l_add->push_back(l_tmp->at(i));
                }

                std::string trans_nm = fpl_refs[size_t(trans_seg)].name;
                delete_ref(trans_seg);
                bool retval = add_fpl_seg(*l_add, proc_seg, proc_nm);
                if(!retval) // Case: runway doesn't belong to sid
                {
                    delete_ref(proc_seg);
                    if(is_star)
                    {
                        if(reset_rwy)
                        {
                            arr_rwy = "";
                            has_arr_rnw_data = false;
                            delete_ref(FPL_SEG_APPCH);
                            delete_ref(FPL_SEG_APPCH_TRANS);
                        }
                        
                        delete_ref(FPL_SEG_STAR_TRANS);
                    }
                    else
                    {
                        if(reset_rwy)
                            delete_ref(FPL_SEG_DEP_RWY);
                        delete_ref(FPL_SEG_SID_TRANS);
                    }

                    if(reset_rwy)
                        fpl_refs[size_t(proc_seg)].name = proc_nm;
                }
                else
                {
                    set_proc_trans(proc_tp, trans_nm, is_star);
                }
                
                return retval;
            }
        }

        return false;
    }

    bool FplnInt::set_appch_legs(std::string appch, std::string& arr_rwy, 
        libnav::arinc_leg_seq_t legs)
    {
        size_t rwy_idx = 0;
        bool rwy_found = false;
        libnav::arinc_leg_seq_t appch_legs = {};
        libnav::arinc_leg_seq_t ga_legs = {}; // Missed approach legs
        for(size_t i = 0; i < legs.size(); i++)
        {
            appch_legs.push_back(legs[i]);

            if(legs[i].main_fix.id == arr_rwy)
            {
                rwy_idx = i;
                rwy_found = true;
                break;
            }
        }

        bool added = add_fpl_seg(appch_legs, FPL_SEG_APPCH, appch);

        if(added)
        {
            if(rwy_found)
            {
                for(size_t i = rwy_idx; i < legs.size(); i++)
                {
                    ga_legs.push_back(legs[i]);
                }

                seg_list_node_t *appr_seg = fpl_refs[size_t(FPL_SEG_APPCH)].ptr;
                if(appr_seg != nullptr)
                {
                    seg_list_node_t *seg_ins = fpl_refs[size_t(FPL_SEG_APPCH)].ptr->next;
                    return add_fpl_seg(ga_legs, FPL_SEG_APPCH, MISSED_APPR_SEG_NM, seg_ins, false);
                }
                else
                {
                    return false;
                }
            }

            return true;
        }
        return false;
    }

    bool FplnInt::set_appch(std::string appch)
    {
        size_t db_idx = get_proc_db_idx(PROC_TYPE_APPCH, true);
        if(proc_db[db_idx].find(appch) != proc_db[db_idx].end())
        {
            std::string curr_star = fpl_refs[FPL_SEG_STAR].name;
            std::string curr_star_trans = fpl_refs[FPL_SEG_STAR_TRANS].name;
            delete_ref(FPL_SEG_STAR_TRANS);
            delete_ref(FPL_SEG_STAR);

            std::string tmp = NONE_TRANS;
            libnav::arinc_leg_seq_t legs = arrival->get_appch(appch, tmp);
            std::string curr_tr = fpl_refs[FPL_SEG_APPCH].name;
            delete_ref(FPL_SEG_APPCH_TRANS);

            std::string tmp_rwy = get_appr_rwy(appch);
            bool added = set_appch_legs(appch, tmp_rwy, legs);
            if(added)
            {
                arr_rwy = tmp_rwy;
                int data_found = arpt_db->get_rnw_data(arrival->icao_code, 
                    arr_rwy, &arr_rnw_data);
                has_arr_rnw_data = true;
                if(!data_found)
                {
                    has_arr_rnw_data = false;
                    arr_rnw_data = {};
                }

                set_sid_star(curr_star, true, false);
                set_proc_trans(PROC_TYPE_STAR, curr_star_trans, true);
                set_proc_trans(PROC_TYPE_APPCH, curr_tr, true);
                return true;
            }
        }

        arr_rwy = "";
        delete_ref(FPL_SEG_STAR_TRANS);
        delete_ref(FPL_SEG_STAR);
        delete_ref(FPL_SEG_APPCH_TRANS);
        delete_ref(FPL_SEG_APPCH);
        return false;
    }

    bool FplnInt::set_proc_trans(ProcType tp, std::string trans, bool is_arr)
    {
        if(trans == "NONE")
        {
            trans = "";
        }

        size_t db_idx = get_proc_db_idx(tp, is_arr);
        size_t seg_tp = size_t(get_proc_tp(tp));
        fpl_segment_types t_tp = get_trans_tp(tp);
        size_t t_idx = size_t(t_tp);

        std::string curr_proc = fpl_refs[seg_tp].name;
        
        if(curr_proc != "" && fpl_refs[seg_tp].ptr == nullptr && 
            proc_db[db_idx][curr_proc].find(trans) != proc_db[db_idx][curr_proc].end())
        {
            delete_ref(t_tp);
            fpl_refs[t_idx].name = trans;

            return false;
        }
        else if(curr_proc == "")
        {
            delete_ref(t_tp);

            return false;
        }
        libnav::Airport *apt = departure;
        if(is_arr)
        {
            apt = arrival;
        }

        libnav::arinc_leg_seq_t legs = {};

        if(tp == PROC_TYPE_SID)
        {
            legs = apt->get_sid(curr_proc, trans);
        }
        else if(tp == PROC_TYPE_STAR)
        {
            legs = apt->get_star(curr_proc, trans);
        }
        else if(tp == PROC_TYPE_APPCH)
        {
            legs = apt->get_appch(curr_proc, trans);
        }
        
        bool added = add_fpl_seg(legs, t_tp, trans);
        if(!added)
        {
            delete_ref(t_tp);
        }
        else
        {
            return true;
        }

        return false;
    }

    bool FplnInt::get_leg_start(leg_seg_t curr_seg, leg_t curr_leg, leg_t next, 
        geo::point *out)
    {
        if(curr_leg.leg_type == "IF")
        {
            *out = curr_leg.main_fix.data.pos;
            return false;
        }
        if(TURN_OFFS_LEGS.find(next.leg_type) != TURN_OFFS_LEGS.end())
        {
            double brng1 = curr_seg.end.get_gc_bearing_rad(curr_seg.start);
            if(next.leg_type == "DF")
            {
                geo::point p1 = geo::get_pos_from_brng_dist(curr_seg.end, brng1 + M_PI/2, 
                    TURN_RADIUS_NM);
                geo::point p2 = geo::get_pos_from_brng_dist(curr_seg.end, brng1 - M_PI/2, 
                    TURN_RADIUS_NM);

                geo::point next_point = next.main_fix.data.pos;
                double dist1 = next_point.get_gc_dist_nm(p1);
                double dist2 = next_point.get_gc_dist_nm(p2);

                if(dist1 < dist2)
                {
                    if(dist1 == 0)
                    {
                        *out = curr_seg.end;
                        return false;
                    }
                    double theta = acos(TURN_RADIUS_NM / dist1);
                    double ang_main = p1.get_gc_bearing_rad(next_point);
                    double ang_end = p1.get_gc_bearing_rad(curr_seg.end);
                    double ang_doub = ang_end - ang_main - theta;
                    if(ang_doub < 0)
                    {
                        ang_doub += 2 * M_PI;
                    }

                    if(ang_doub / 2 != M_PI / 2)
                    {
                        double offs_nm = TURN_RADIUS_NM * tan(ang_doub / 2);
                        *out = geo::get_pos_from_brng_dist(curr_seg.end, brng1 + M_PI, offs_nm);
                        return false;
                    }
                    else
                    {
                        *out = geo::get_pos_from_brng_dist(p1, brng1 + M_PI/2, 
                            TURN_RADIUS_NM);
                        return false;
                    }
                }
                else
                {
                    if(dist2 == 0)
                    {
                        *out = curr_seg.end;
                        return false;
                    }
                    double theta = acos(TURN_RADIUS_NM / dist2);
                    double ang_main = p2.get_gc_bearing_rad(next_point);
                    double ang_end = p2.get_gc_bearing_rad(curr_seg.end);
                    double ang_doub = ang_main - theta - ang_end;
                    if(ang_doub < 0)
                    {
                        ang_doub += 2 * M_PI;
                    }

                    if(ang_doub / 2 != M_PI / 2)
                    {
                        double offs_nm = TURN_RADIUS_NM * tan(ang_doub / 2);
                        *out = geo::get_pos_from_brng_dist(curr_seg.end, brng1 + M_PI, offs_nm);
                        return false;
                    }
                    else
                    {
                        *out = geo::get_pos_from_brng_dist(p1, brng1 - M_PI/2, 
                            TURN_RADIUS_NM);
                        return false;
                    }
                }
            }
            else
            {
                double crs_rad = double(next.outbd_crs_deg) * geo::DEG_TO_RAD;
                if(next.leg_type[0] == 'C')
                    crs_rad += next.get_mag_var_deg() * geo::DEG_TO_RAD;
                if(brng1 < 0)
                {
                    brng1 += 2 * M_PI;
                }
                double turn_rad = abs(crs_rad - brng1);
                if(turn_rad > M_PI)
                {
                    turn_rad = 2 * M_PI - turn_rad;
                }
                else if(turn_rad < -M_PI)
                {
                    turn_rad += 2 * M_PI;
                }
                double theta = turn_rad / 2;
                double sin_theta = sin(theta);
                double cos_theta = cos(theta);
                if(sin_theta != 0 && cos_theta != 0)
                {
                    double offs_nm = TURN_RADIUS_NM * cos_theta / sin_theta;
                    assert(offs_nm >= 0);

                    *out = geo::get_pos_from_brng_dist(curr_seg.end, brng1 + M_PI, offs_nm);
                    return false;
                }
            }
        }
        else if(next.leg_type == "TF")
        {
            *out = curr_leg.main_fix.data.pos;
            return false;
        }
        else if(next.leg_type == "CF")
        {
            double outbd_brng_deg = double(next.outbd_crs_deg);
            
            if(!next.outbd_crs_true)
                outbd_brng_deg += next.get_mag_var_deg();
            
            double curr_brng_rad = curr_seg.true_trk_deg * geo::DEG_TO_RAD;
            double brng_to_main_fix = curr_seg.start.get_gc_bearing_rad(
                next.main_fix.data.pos);
            
            double brng_next_rad = outbd_brng_deg * geo::DEG_TO_RAD;

            bool left_turn = is_ang_greater(curr_brng_rad, brng_next_rad);
            bool brng_gr = is_ang_greater(brng_to_main_fix, curr_brng_rad);

            bool is_bp = true;
            if((brng_gr && !left_turn) || (!brng_gr && left_turn))
            {
                brng_next_rad += M_PI;
                is_bp = false;
            }

            geo::point intc = geo::get_pos_from_intc(curr_seg.start, 
                next.main_fix.data.pos, curr_brng_rad, brng_next_rad);
                
            *out = intc;
            return is_bp;
        }
        else if(next.leg_type[0] == 'F')
        {
            *out = next.main_fix.data.pos;
            return false;
        }

        *out = curr_seg.end;
        return false;
    }

    void FplnInt::calculate_leg(leg_list_node_t *leg, double hdg_trk_diff)
    {
        leg_t curr_arinc_leg = leg->data.leg;

        leg->data.misc_data.is_rwy = false;
        leg->data.misc_data.is_arc = false;
        leg->data.misc_data.is_finite = false;
        leg->data.misc_data.turn_rad_nm = -1;

        if(leg->data.leg.main_fix.data.type == libnav::NavaidType::RWY)
        {
            leg->data.misc_data.is_rwy = true;
        }

        leg_list_node_t *prev_leg = leg->prev;
        bool is_bypassed = false;
        if(prev_leg != &(leg_list.head) && !prev_leg->data.is_discon)
        {
            is_bypassed = get_leg_start(prev_leg->data.misc_data, 
                prev_leg->data.leg, curr_arinc_leg, &(leg->data.misc_data.start));
            
            if(prev_leg->data.misc_data.turn_rad_nm != -1)
            {
                if((prev_leg->data.leg.leg_type == "VI" || prev_leg->data.leg.leg_type == "CI"))
                {
                    prev_leg->data.misc_data.end = leg->data.misc_data.start;
                    prev_leg->data.leg.main_fix.id = INTC_LEG_NM;
                    prev_leg->data.leg.main_fix.data.pos = leg->data.misc_data.start;
                    prev_leg->data.leg.outbd_dist_time = prev_leg->data.misc_data.start.get_gc_dist_nm(
                        prev_leg->data.misc_data.end);
                }

                if(TURN_OFFS_LEGS.find(curr_arinc_leg.leg_type) == TURN_OFFS_LEGS.end() &&
                    LEGS_CALC.find(prev_leg->data.leg.leg_type) != LEGS_CALC.end())
                {
                    double rnp_nm = get_rnp(leg);
                    double prev_turn_rad_nm = prev_leg->data.misc_data.turn_rad_nm;

                    double turn_offs_nm = sqrt((prev_turn_rad_nm + rnp_nm) * 
                        (prev_turn_rad_nm + rnp_nm) - prev_turn_rad_nm * prev_turn_rad_nm);

                    geo::point prev_start = prev_leg->data.misc_data.start;
                    geo::point curr_start = leg->data.misc_data.start;
                    double dist_nm = prev_start.get_gc_dist_nm(curr_start);

                    if(turn_offs_nm < dist_nm)
                    {
                        dist_nm -= turn_offs_nm;
                        double brng_rad = prev_start.get_gc_bearing_rad(curr_start);
                        prev_leg->data.misc_data.end = geo::get_pos_from_brng_dist(prev_start, 
                            brng_rad, dist_nm);
                    }
                }
            }
        }

        if(is_bypassed)
            return;

        if(curr_arinc_leg.leg_type == "IF")
        {
            geo::point main_fix_pos = curr_arinc_leg.main_fix.data.pos;
            leg->data.misc_data.is_arc = false;
            leg->data.misc_data.is_finite = true;
            leg->data.misc_data.start = main_fix_pos;
            leg->data.misc_data.end = main_fix_pos;
            leg->data.misc_data.turn_rad_nm = 0;
        }
        else if(curr_arinc_leg.leg_type == "CA" || curr_arinc_leg.leg_type == "VA")
        {
            libnav::runway_entry_t *rwy_ent = nullptr;
            if(leg->prev->data.seg->data.seg_type == FPL_SEG_DEP_RWY)
            {
                rwy_ent = &dep_rnw_data;
            }
            else if(leg->prev->data.leg.main_fix.data.type == libnav::NavaidType::RWY)
            {
                rwy_ent = &arr_rnw_data;
            }

            float mag_var_deg = curr_arinc_leg.get_mag_var_deg();

            if(curr_arinc_leg.leg_type == "VA")
            {
                mag_var_deg += hdg_trk_diff;
            }

            leg->data.misc_data.true_trk_deg = curr_arinc_leg.outbd_crs_deg+mag_var_deg;

            geo::point end_pt = get_xa_end_point(leg->data.misc_data.start, 
                curr_arinc_leg.outbd_crs_deg+mag_var_deg, curr_arinc_leg.alt1_ft, rwy_ent);
            libnav::waypoint_t end_wpt = get_ca_va_wpt(end_pt, int(curr_arinc_leg.alt1_ft));

            leg->data.misc_data.is_arc = false;
            leg->data.misc_data.is_finite = true;
            leg->data.misc_data.end = end_pt;
            leg->data.misc_data.turn_rad_nm = TURN_RADIUS_NM;

            leg->data.leg.set_main_fix(end_wpt);
            leg->data.leg.outbd_dist_time = curr_arinc_leg.alt1_ft / float(CLB_RATE_FT_PER_NM);
            leg->data.leg.outbd_dist_as_time = false;
        }
        else if(curr_arinc_leg.leg_type == "VI" || curr_arinc_leg.leg_type == "CI")
        {
            std::string next_tp = leg->next->data.leg.leg_type;
            if(leg->next != &(leg_list.tail) && 
                AFTER_INTC.find(next_tp) != AFTER_INTC.end())
            {
                double curr_brng = double(curr_arinc_leg.outbd_crs_deg);
                if(!curr_arinc_leg.outbd_crs_true)
                    curr_brng += curr_arinc_leg.get_mag_var_deg();
                if(curr_arinc_leg.leg_type == "VI")
                {
                    curr_brng += hdg_trk_diff;
                }
                leg->data.misc_data.true_trk_deg = curr_brng;
                leg->data.misc_data.is_arc = false;
                leg->data.misc_data.turn_rad_nm = TURN_RADIUS_NM;
                leg->data.misc_data.is_finite = true;
            }
        }
        else if(curr_arinc_leg.leg_type == "TF" || curr_arinc_leg.leg_type == "CF" || 
            curr_arinc_leg.leg_type == "DF")
        {
            geo::point curr_start = leg->data.misc_data.start;
            geo::point curr_end = curr_arinc_leg.main_fix.data.pos;

            double brng_rad = curr_start.get_gc_bearing_rad(curr_end);
            double dist_nm = curr_start.get_gc_dist_nm(curr_end);
            double turn_rad_nm = TURN_RADIUS_NM;

            leg->data.misc_data.end = curr_end;

            leg->data.misc_data.true_trk_deg = brng_rad * geo::RAD_TO_DEG;
            
            if(leg->data.misc_data.true_trk_deg < 0)
                leg->data.misc_data.true_trk_deg += 360;

            leg->data.leg.outbd_dist_time = dist_nm;
            leg->data.leg.outbd_dist_as_time = false;

            leg->data.misc_data.is_arc = false;
            leg->data.misc_data.is_finite = true;
            leg->data.misc_data.end = curr_end;
            leg->data.misc_data.turn_rad_nm = turn_rad_nm;
        }

        if(leg->data.misc_data.true_trk_deg > 360)
            leg->data.misc_data.true_trk_deg -= 360;
    }
} // namespace test
