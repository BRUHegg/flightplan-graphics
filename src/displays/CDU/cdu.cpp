#include "cdu.hpp"

namespace StratosphereAvionics
{
    // CDU definitions:
    // Public member functions:

    CDU::CDU(std::shared_ptr<test::FPLSys> fs)
    {
        fpl_sys = fs;
        curr_page = CDUPage::RTE1;
        curr_subpg = 1;
        n_subpg = 1;
    }

    void CDU::on_event(int event_key, std::string scratchpad)
    {
        if(curr_page == CDUPage::RTE1)
        {
            handle_rte(event_key, scratchpad);
        }
    }

    cdu_scr_data_t CDU::get_screen_data()
    {
        if(curr_page == CDUPage::RTE1)
        {
            return get_rte_page();
        }

        return {};
    }

    // Private member functions:

    void CDU::set_departure(std::string icao)
    {
        libnav::DbErr err = fpl_sys->fpl->set_dep(icao);
        UNUSED(err);
    }

    void CDU::set_arrival(std::string icao)
    {
        libnav::DbErr err = fpl_sys->fpl->set_arr(icao);
        UNUSED(err);
    }

    void CDU::load_rte()
    {
        std::string dep_nm = fpl_sys->fpl->get_dep_icao();
        std::string arr_nm = fpl_sys->fpl->get_arr_icao();

        if(dep_nm != "" && arr_nm != "")
        {
            std::string file_nm = fpl_sys->fpl_dir+dep_nm+arr_nm;
            libnav::DbErr err = fpl_sys->fpl->load_from_fms(file_nm, false);
            UNUSED(err);
        }
    }

    void CDU::save_rte()
    {
        std::string dep_nm = fpl_sys->fpl->get_dep_icao();
        std::string arr_nm = fpl_sys->fpl->get_arr_icao();

        if(dep_nm != "" && arr_nm != "")
        {
            std::string out_nm = fpl_sys->fpl_dir+dep_nm+arr_nm;
            fpl_sys->fpl->save_to_fms(out_nm);
        }
    }


    void CDU::handle_rte(int event_key, std::string scratchpad)
    {
        if(curr_subpg == 1)
        {
            if(event_key == CDU_KEY_LSK_TOP)
            {
                set_departure(scratchpad);
            }
            else if(event_key == CDU_KEY_RSK_TOP)
            {
                set_arrival(scratchpad);
            }
            else if(event_key == CDU_KEY_LSK_TOP + 2)
            {
                load_rte();
            }
            else if(event_key == CDU_KEY_LSK_TOP + 4)
            {
                save_rte();
            }
        }
    }

    cdu_scr_data_t CDU::get_rte_page()
    {
        cdu_scr_data_t out = {};
        if(curr_subpg == 1)
        {
            std::string rte_offs = std::string(6, ' ');
            out.heading_big = rte_offs + "RTE1";
            out.heading_color = CDUColor::CYAN;

            std::string dest_offs = std::string(N_CDU_DATA_COLS-7-4, ' ');
            std::string origin_dest = " ORIGIN" + dest_offs + "DEST";
            out.data_lines.push_back(origin_dest);
            std::string origin = fpl_sys->fpl->get_dep_icao();
            std::string dest = fpl_sys->fpl->get_arr_icao();
            bool incomplete = false;

            if(origin == "")
            {
                origin = std::string(4, '@');
                incomplete = true;
            }
            if(dest == "")
            {
                dest = std::string(4, '@');
                incomplete = true;
            }

            std::string od_data = origin + std::string(N_CDU_DATA_COLS-4-4, ' ') + dest;
            out.data_lines.push_back(od_data);
            std::string rwy_flt_no = " RUNWAY" + std::string(N_CDU_DATA_COLS-7-6, ' ') + "FLT NO";
            out.data_lines.push_back(rwy_flt_no);
            std::string dep_rwy = "";
            if(!incomplete)
            {
                 dep_rwy = fpl_sys->fpl->get_dep_rwy();
                if(dep_rwy == "")
                    dep_rwy = std::string(5, '-');
                else
                    dep_rwy = "RW" + dep_rwy;    
            }
            std::string rf_data = dep_rwy + std::string(size_t(N_CDU_DATA_COLS)-dep_rwy.size()-10, ' ') + std::string(10, '-');
            out.data_lines.push_back(rf_data);
            out.data_lines.push_back(" ROUTE" + std::string(N_CDU_DATA_COLS-6-8, ' ') + "CO ROUTE");
            out.data_lines.push_back("<LOAD" + std::string(size_t(N_CDU_DATA_COLS)-5-10, ' ') + std::string(10, '-'));
            out.data_lines.push_back("");
            out.data_lines.push_back("");
            std::string rte_final = " ROUTE ";
            out.data_lines.push_back(rte_final+std::string(size_t(N_CDU_DATA_COLS)-7, '-'));
            out.data_lines.push_back("<SAVE" + std::string(size_t(N_CDU_DATA_COLS)-10, ' ') + "ALTN>");
            out.data_lines.push_back("");
            out.data_lines.push_back("<RTE 2" + std::string(size_t(N_CDU_DATA_COLS)-15, ' ') + "ACTIVATE>");
        }

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

        tex_scale = size / cairo_utils::get_surf_sz(tex_mngr->data[CDU_TEXTURE_NAME]);

        scratchpad = std::string(size_t(N_CDU_DATA_COLS), ' ');
        scratch_curr = 0;
    }

    void CDUDisplay::on_click(geom::vect2_t pos)
    {
        pos = (pos - scr_pos) / tex_scale;
        if (pos.x >= 0 && pos.y >= 0 && pos.x < size.x && pos.y < size.y)
        {
            int event = int(key_map->get_at(size_t(pos.x), size_t(pos.y)));

            if(event && event < CDU_KEY_A)
            {
                std::string scratch_proc = strutils::strip(scratchpad);
                cdu_ptr->on_event(event, scratch_proc);
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

    void CDUDisplay::update_scratchpad(int event)
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
        else if (event == CDU_KEY_CLR)
        {
            if (scratch_curr)
                scratch_curr--;
            scratchpad[scratch_curr] = ' ';
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
            if (scratch_curr && scratchpad[scratch_curr-1] == '-')
                scratchpad[scratch_curr-1] = '+';
            else
                add_to_scratchpad('-');
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
        else if (c == '*')
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

        for (size_t i = 0; i < s.size(); i++)
        {
            draw_cdu_letter(cr, s[i], pos, scale, font_sfc);
            pos.x += l_intv_px;
        }
    }

    void CDUDisplay::draw_screen(cairo_t *cr)
    {
        geom::vect2_t small_offs = {0, disp_size.y * CDU_V_OFFS_FIRST};
        geom::vect2_t pos_small = disp_pos + small_offs;
        geom::vect2_t big_offs = {0, disp_size.y * (CDU_BIG_TEXT_OFFS + CDU_V_OFFS_FIRST)};
        geom::vect2_t pos_big = disp_pos + big_offs;
        
        cdu_scr_data_t curr_screen = cdu_ptr->get_screen_data();

        draw_cdu_line(cr, curr_screen.heading_big, disp_pos, CDU_BIG_TEXT_SZ,
                      CDU_TEXT_INTV * disp_size.x, curr_screen.heading_color);

        size_t j = 0;
        for (size_t i = 0; i < size_t(N_CDU_DATA_LINES); i++)
        {
            if(j < curr_screen.data_lines.size())
            {
                draw_cdu_line(cr, curr_screen.data_lines[j], pos_small, CDU_SMALL_TEXT_SZ,
                                CDU_TEXT_INTV * disp_size.x);
            }
            if(j+1 < curr_screen.data_lines.size())
            {
                draw_cdu_line(cr, curr_screen.data_lines[j+1], pos_big,
                                CDU_BIG_TEXT_SZ, CDU_TEXT_INTV * disp_size.x);
            }

            pos_small.y += CDU_V_OFFS_REG * disp_size.y;
            pos_big.y += CDU_V_OFFS_REG * disp_size.y;
            j += 2;
        }

        draw_cdu_line(cr, scratchpad, pos_small, CDU_BIG_TEXT_SZ,
                      CDU_TEXT_INTV * disp_size.x);
    }
}
