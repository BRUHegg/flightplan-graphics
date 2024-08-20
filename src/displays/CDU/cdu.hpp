#include <string>
#include <memory>
#include <geom.hpp>
#include "cairo_utils.hpp"


namespace StratosphereAvionics
{
    constexpr double N_CDU_LINES = 8; // small and big lines are counted as 1
    constexpr double CDU_SMALL_TEXT_SZ = 18;
    constexpr double CDU_BIG_TEXT_SZ = 21;
    constexpr double CDU_BIG_TEXT_OFFS = 0.04;

    const geom::vect2_t DISPLAY_OFFS = {0.1, 0.1};
    const geom::vect2_t DISPLAY_SZ = {0.9, 0.4};

    const std::string CDU_TEXTURE_NAME = "cdu";


    class CDUDisplay
    {
    public:
        CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, cairo_font_face_t *ff,
            std::shared_ptr<cairo_utils::texture_manager_t> tm);

        void draw(cairo_t *cr);
    private:
        geom::vect2_t scr_pos; // position of the CDU texture on the screen
        geom::vect2_t size;
        geom::vect2_t disp_pos; // position of the CDU display on the screen
        geom::vect2_t disp_size;
        geom::vect2_t tex_scale;

        cairo_font_face_t *font_face;

        std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;

        
        void draw_screen(cairo_t *cr);
    };
}
