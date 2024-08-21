#include "cdu.hpp"


namespace StratosphereAvionics
{
    // CDUDisplay definitions:
    // Public member functions:

    CDUDisplay::CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, cairo_font_face_t *ff,
        std::shared_ptr<cairo_utils::texture_manager_t> tm)
    {
        scr_pos = pos;
        size = sz;
        disp_pos = scr_pos + size * DISPLAY_OFFS;
        disp_size = size * DISPLAY_SZ;

        font_face = ff;

        tex_mngr = tm;

        tex_scale = size / cairo_utils::get_surf_sz(tex_mngr->data[CDU_TEXTURE_NAME]);
    }

    void CDUDisplay::draw(cairo_t *cr)
    {
        cairo_utils::draw_image(cr, tex_mngr->data[CDU_TEXTURE_NAME], scr_pos, 
            tex_scale, false);

        draw_screen(cr);
    }

    // Private member functions:

    void CDUDisplay::draw_text_line(cairo_t *cr, std::string& text, geom::vect2_t pos, 
            geom::vect3_t color, double sz, double l_intv_px)
    {
        for(size_t i = 0; i < text.size(); i++)
        {
            std::string curr = std::string(1, text[i]);
            cairo_utils::draw_left_text(cr, font_face, curr, pos, 
                color, sz);
            
            pos.x += l_intv_px;
        }
    }

    void CDUDisplay::draw_screen(cairo_t *cr)
    {
        double v_offs = disp_size.y / N_CDU_LINES;

        geom::vect2_t pos_small = disp_pos;
        geom::vect2_t big_offs = {0, disp_size.y * CDU_BIG_TEXT_OFFS};
        geom::vect2_t pos_big = disp_pos + big_offs;
        std::string test_str = std::string(24, 'X');
        for(int i = 0; i < int(N_CDU_LINES); i++)
        {
            draw_text_line(cr, test_str, pos_small, cairo_utils::WHITE, 
                CDU_SMALL_TEXT_SZ, CDU_SMALL_TEXT_INTV * disp_size.x);
            draw_text_line(cr, test_str, pos_big, cairo_utils::WHITE, 
                CDU_BIG_TEXT_SZ, CDU_BIG_TEXT_INTV * disp_size.x);

            pos_small.y += v_offs;
            pos_big.y += v_offs;
        }
    }
}
