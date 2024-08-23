#include "cdu.hpp"


namespace StratosphereAvionics
{
    // CDUDisplay definitions:
    // Public member functions:

    CDUDisplay::CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, cairo_font_face_t *ff,
        std::shared_ptr<cairo_utils::texture_manager_t> tm, byteutils::Bytemap *bm)
    {
        scr_pos = pos;
        size = sz;
        disp_pos = scr_pos + size * DISPLAY_OFFS;
        disp_size = size * DISPLAY_SZ;

        font_face = ff;

        tex_mngr = tm;

        key_map = bm;

        tex_scale = size / cairo_utils::get_surf_sz(tex_mngr->data[CDU_TEXTURE_NAME]);
    }

    void CDUDisplay::on_click(geom::vect2_t pos)
    {
        pos = (pos - scr_pos) / tex_scale;
        if(pos.x >= 0 && pos.y >= 0 && pos.x < size.x && pos.y < size.y)
        {
            std::cout << int(key_map->get_at(size_t(pos.x), size_t(pos.y))) << "\n";
        }
    }

    void CDUDisplay::draw(cairo_t *cr)
    {
        cairo_utils::draw_image(cr, tex_mngr->data[CDU_TEXTURE_NAME], scr_pos, 
            tex_scale, false);

        draw_screen(cr);
    }

    // Private member functions:

    int CDUDisplay::get_cdu_letter_idx(char c)
    {
        if(c >= '0' && c <= '9')
            return 1+c-'0';
        else if(c >= 'A' && c <= 'Z')
            return 11+c-'A';
        else if(c == '%')
            return 37;
        else if(c == '(')
            return 38;
        else if(c == ')')
            return 39;
        else if(c == '-')
            return 40;
        else if(c == '_')
            return 41;
        else if(c == '+')
            return 42;
        else if(c == '=')
            return 43;
        else if(c == '|')
            return 44;
        else if(c == ':')
            return 45;
        else if(c == '<')
            return 46;
        else if(c == '.')
            return 47;
        else if(c == '>')
            return 48;
        else if(c == ',')
            return 49;
        else if(c == '/')
            return 50;
        else if(c == '*')
            return 51;
        else if(c == '@')
            return 52;
        return 0;
    }

    void CDUDisplay::draw_cdu_letter(cairo_t *cr, char c, geom::vect2_t pos, 
        geom::vect2_t scale, cairo_surface_t *font_sfc)
    {
        if(scale.x == 0 || scale.y == 0)
            return;

        int idx = get_cdu_letter_idx(c);
        geom::vect2_t offs = {-CDU_LETTER_WIDTH * scale.x * double(idx), 0};
        geom::vect2_t img_pos = pos + offs;
        cairo_save(cr);
        cairo_scale(cr, scale.x, scale.y);
        cairo_set_source_surface(cr, font_sfc, img_pos.x/scale.x, img_pos.y/scale.y);
        cairo_rectangle(cr, pos.x/scale.x, pos.y/scale.y, 
            CDU_LETTER_WIDTH, CDU_LETTER_HEIGHT);
        cairo_clip(cr);
        cairo_paint(cr);
        cairo_restore(cr);
    }

    void CDUDisplay::draw_cdu_line(cairo_t *cr, std::string& s, geom::vect2_t pos, 
        geom::vect2_t scale, double l_intv_px, CDUColor color)
    {
        cairo_surface_t *font_sfc;
        if(color == CDUColor::GREEN)
            font_sfc = tex_mngr->data[CDU_GREEN_TEXT_NAME];
        else if(color == CDUColor::CYAN)
            font_sfc = tex_mngr->data[CDU_CYAN_TEXT_NAME];
        else if(color == CDUColor::MAGENTA)
            font_sfc = tex_mngr->data[CDU_MAGENTA_TEXT_NAME];
        else
            font_sfc = tex_mngr->data[CDU_WHITE_TEXT_NAME];

        for(size_t i = 0; i < s.size(); i++)
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
        std::string test_str = std::string(24, 'X');

        draw_cdu_line(cr, test_str, disp_pos, CDU_BIG_TEXT_SZ, 
                CDU_TEXT_INTV * disp_size.x);
        for(int i = 0; i < N_CDU_DATA_LINES; i++)
        {
            draw_cdu_line(cr, test_str, pos_small, CDU_SMALL_TEXT_SZ, 
                CDU_TEXT_INTV * disp_size.x, CDUColor::MAGENTA);
            draw_cdu_line(cr, test_str, pos_big,
                CDU_BIG_TEXT_SZ, CDU_TEXT_INTV * disp_size.x, CDUColor::GREEN);

            pos_small.y += CDU_V_OFFS_REG * disp_size.y;
            pos_big.y += CDU_V_OFFS_REG * disp_size.y;
        }
    }
}
