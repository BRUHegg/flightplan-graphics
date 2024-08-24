#include <string>
#include <memory>
#include <stack>
#include <geom.hpp>
#include "common/cairo_utils.hpp"
#include "common/bytemap.hpp"
#include "fpln/fpln_sys.hpp"


namespace StratosphereAvionics
{
    enum class CDUPage
    {
        RTE,
        DEP_ARR_INTRO,
        DEP,
        ARR,
        LEGS,
        INIT_REF,
        ALTN,
        VNAV,
        FIX,
        HOLD,
        FMC_COMM,
        PROG,
        MENU,
        NAV_RAD,
        PREV_PAGE,
        NEXT_PAGE
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
    constexpr int CDU_KEY_INIT_REF = 13;
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
    constexpr double CDU_V_OFFS_SMALL_FIRST = 0.027;
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

    const std::string INVALID_ENTRY_MSG = "INVALID ENTRY";
    const std::string NOT_IN_DB_MSG = "NOT IN DATA BASE";
    const std::string CDU_TEXTURE_NAME = "cdu";
    const std::string CDU_WHITE_TEXT_NAME = "cdu_big_white";
    const std::string CDU_GREEN_TEXT_NAME = "cdu_big_green";
    const std::string CDU_CYAN_TEXT_NAME = "cdu_big_cyan";
    const std::string CDU_MAGENTA_TEXT_NAME = "cdu_big_magenta";
    const std::string DISCO_AFTER_SEG = "-- ROUTE DISCONTINUITY -";
    const std::string SEG_LAST = "-------            -----";


    const std::vector<CDUPage> CDU_PAGE_FACES = {
        CDUPage::INIT_REF,
        CDUPage::RTE,
        CDUPage::DEP_ARR_INTRO,
        CDUPage::ALTN,
        CDUPage::VNAV,
        CDUPage::FIX,
        CDUPage::LEGS,
        CDUPage::HOLD,
        CDUPage::FMC_COMM,
        CDUPage::PROG,
        CDUPage::MENU,
        CDUPage::NAV_RAD,
        CDUPage::PREV_PAGE,
        CDUPage::NEXT_PAGE};

    
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

        void update();

        std::string on_event(int event_key, std::string scratchpad, std::string *s_out);

        cdu_scr_data_t get_screen_data();

    private:
        std::shared_ptr<test::FPLSys> fpl_sys;
        std::shared_ptr<test::FplnInt> fpln;
        CDUPage curr_page;
        int n_subpg;
        int curr_subpg;

        size_t n_seg_list_sz, n_leg_list_sz;
        std::vector<test::list_node_ref_t<test::fpl_seg_t>> seg_list;
        std::vector<test::list_node_ref_t<test::leg_list_data_t>> leg_list;


        std::string set_departure(std::string icao, std::string *s_out);

        std::string set_arrival(std::string icao, std::string *s_out);

        std::string set_dep_rwy(std::string id);

        std::string load_rte();

        std::string save_rte();

        std::string add_via(size_t next_idx, std::string name);

        std::string add_to(size_t next_idx, std::string name);

        void get_seg_page(cdu_scr_data_t *in);

        std::string get_small_heading();


        int get_n_rte_subpg();

        std::string handle_rte(int event_key, std::string scratchpad, std::string *s_out);

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

        std::stack<std::string> msg_stack;


        void add_to_scratchpad(char c);

        void clear_scratchpad();

        void update_scratchpad(int event);

        static int get_cdu_letter_idx(char c);

        void draw_cdu_letter(cairo_t *cr, char c, geom::vect2_t pos, geom::vect2_t scale,
            cairo_surface_t *font_sfc);

        void draw_cdu_line(cairo_t *cr, std::string& s, geom::vect2_t pos, 
            geom::vect2_t scale, double l_intv_px, CDUColor color=CDUColor::WHITE);
        
        void draw_screen(cairo_t *cr);
    };
}
