#include <string>
#include <memory>
#include <geom.hpp>
#include "common/cairo_utils.hpp"
#include "common/bytemap.hpp"
#include "fpln/fpln_sys.hpp"


namespace StratosphereAvionics
{
    enum class CDUPage
    {
        RTE1,
        DEP_ARR_INTRO,
        DEP,
        ARR,
        LEGS
    };

    enum class CDUColor
    {
        WHITE,
        GREEN,
        CYAN,
        MAGENTA
    };


    constexpr int N_CDU_DATA_LINES = 6;
    constexpr int N_CDU_DATA_COLS = 24;

    // CDU keys:
    constexpr int CDU_KEY_LSK_TOP = 1;
    constexpr int CDU_KEY_RSK_TOP = 7;
    constexpr int CDU_KEY_A = 27;
    constexpr int CDU_KEY_SP = 53;
    constexpr int CDU_KEY_DELETE = 54;
    constexpr int CDU_KEY_SLASH = 55;
    constexpr int CDU_KEY_CLR = 56;
    constexpr int CDU_KEY_1 = 57;
    constexpr int CDU_KEY_DOT = 66;
    constexpr int CDU_KEY_0 = 67;
    constexpr int CDU_KEY_PM = 68;  // +/- key
    constexpr int CDU_KEY_EXEC = 69;

    constexpr double CDU_V_OFFS_FIRST = 0.095;
    constexpr double CDU_V_OFFS_REG = 0.134; // * screen height
    constexpr double CDU_SMALL_TEXT_OFFS_X = 0.003;
    constexpr double CDU_BIG_TEXT_OFFS = 0.05;
    constexpr double CDU_TEXT_INTV = 0.033;
    constexpr double CDU_LETTER_WIDTH = 21;
    constexpr double CDU_LETTER_HEIGHT = 39;

    constexpr double CDU_TEXTURE_ASPECT_RATIO = (488.0/751.0);

    const geom::vect2_t DISPLAY_OFFS = {0.14, 0.068};
    const geom::vect2_t DISPLAY_SZ = {0.9, 0.378};
    constexpr geom::vect2_t CDU_SMALL_TEXT_SZ = {0.7, 0.7};
    constexpr geom::vect2_t CDU_BIG_TEXT_SZ = {0.9, 0.96};

    const std::string CDU_TEXTURE_NAME = "cdu";
    const std::string CDU_WHITE_TEXT_NAME = "cdu_big_white";
    const std::string CDU_GREEN_TEXT_NAME = "cdu_big_green";
    const std::string CDU_CYAN_TEXT_NAME = "cdu_big_cyan";
    const std::string CDU_MAGENTA_TEXT_NAME = "cdu_big_magenta";


    struct cdu_scr_data_t
    {
        std::string heading_big, heading_small;
        CDUColor heading_color;
        std::vector<std::string> data_lines;
    };

    class CDU
    {
    public:
        CDU(std::shared_ptr<test::FPLSys> fs);

        void on_event(int event_key, std::string scratchpad);

        cdu_scr_data_t get_screen_data();

    private:
        std::shared_ptr<test::FPLSys> fpl_sys;
        CDUPage curr_page;
        int n_subpg;
        int curr_subpg;


        void set_departure(std::string icao);

        void set_arrival(std::string icao);

        void load_rte();

        void save_rte();


        void handle_rte(int event_key, std::string scratchpad);

        cdu_scr_data_t get_rte_page();
    };

    class CDUDisplay
    {
    public:
        CDUDisplay(geom::vect2_t pos, geom::vect2_t sz, cairo_font_face_t *ff,
            std::shared_ptr<cairo_utils::texture_manager_t> tm, 
            std::shared_ptr<CDU> cdu, byteutils::Bytemap *bm);

        void on_click(geom::vect2_t pos);

        void draw(cairo_t *cr);
    private:
        geom::vect2_t scr_pos; // position of the CDU texture on the screen
        geom::vect2_t size;
        geom::vect2_t disp_pos; // position of the CDU display on the screen
        geom::vect2_t disp_size;
        geom::vect2_t tex_scale;

        cairo_font_face_t *font_face;

        std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;
        std::shared_ptr<CDU> cdu_ptr;

        byteutils::Bytemap *key_map;

        std::string scratchpad;
        size_t scratch_curr;


        void add_to_scratchpad(char c);

        void update_scratchpad(int event);

        static int get_cdu_letter_idx(char c);

        void draw_cdu_letter(cairo_t *cr, char c, geom::vect2_t pos, geom::vect2_t scale,
            cairo_surface_t *font_sfc);

        void draw_cdu_line(cairo_t *cr, std::string& s, geom::vect2_t pos, 
            geom::vect2_t scale, double l_intv_px, CDUColor color=CDUColor::WHITE);
        
        void draw_screen(cairo_t *cr);
    };
}
